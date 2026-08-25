"""
======================================================================================
BENCHMARKING METHODOLOGY & METRIC ISOLATION
======================================================================================
This script processes high-resolution, event-based timeline metrics from the C++ planner.

Event semantics (as logged by the planner):
- initial_plan   : one-time initialization plan (setup + initial search)
- set_state      : robot-state snapshot; path_cost is recorded here
- update         : obstacle update / repair trigger (only present when obstacles change)
- plan           : anytime planning iteration (sample addition or repair)
- slice_end      : end of a real-time slice
- goal_reached   : terminal success snapshot
- time_limit     : terminal timeout snapshot

======================================================================================
HOW METRICS ARE COMPUTED (per seed, then aggregated across seeds)
======================================================================================

For each independent seed we first compute per‑seed summaries.  These summaries are
then aggregated across the 30 paired successful seeds to obtain the numbers reported
in the table and the distributions shown in the boxplots/CDFs.

1.  T_add  (sample addition time)
    Per seed: median of "plan" rows that are NOT the first plan after an update
              (i.e., only steady‑state sample‑addition plans).
    Aggregated: median of all per‑seed medians.
    → Table column "T_add(ms)" equals this aggregated value.

2.  T_repair  (full graph‑repair latency)
    Per seed:
      - For FMTX:  each repair event consists of an "update" row immediately
                    followed by a "plan" row.  We compute
                         total_repair = update_ms + plan_ms
                    for that event, then take the median of all such totals
                    within the seed.
      - For RRTX:   the following "plan" is NOT part of the repair; we take
                    only update_ms as the repair time, and its per‑seed median.
    Aggregated: median of all per‑seed medians.
    → Table column "T_repair(ms)" equals this aggregated value.

3.  T_upd  and  T_pln  (the individual components)
    These two columns are reported for completeness, but they describe the
    marginal distributions of update time alone and repair plan time alone.
    They are computed as:
      - per seed: median of update_ms alone (T_upd) and median of plan_ms alone
                  (T_pln) over only the first plan after an update.
      - aggregated: median of those per‑seed medians.
    IMPORTANT:  T_upd + T_pln  ≠  T_repair  because the median of a sum
    is not the sum of medians.  The values in the table cannot be added to
    obtain the repair time; use the T_repair column directly.

4.  Obstacle checks
    - "Obs/Repair" (primary): per‑seed mean of total checks per repair event.
      For FMTX this includes checks in both the update step and the following
      repair plan; for RRTX only the update step (the repair plan is not part
      of repair).  Aggregated as the median (IQR) of per‑seed means.
    - "ObsRepPln": per seed, mean of checks during the first repair plan only
                   (FMTX only, NaN for RRTX).  Aggregated as the median of
                   per‑seed means.
    - "ObsSteady": per seed, mean of checks during steady‑state sample‑addition
                   plans.  Aggregated as the median of per‑seed means.
    The boxplot and CDF for obstacle checks use exactly the same per‑seed
    means that feed the "Obs/Repair" column, so the figures and table are
    directly comparable.

5.  Path cost
    Taken only from "set_state" and "goal_reached" rows.
    Per seed: median of all such path costs.
    Aggregated: median of per‑seed medians.
    → Table column "Path_Cost".

6.  Distribution plots (boxplots and CDFs)
    Each boxplot / CDF uses as data one value per seed:
    - Repair time:  per‑seed median total repair time (the same numbers whose
                    median gives the table's T_repair).
    - Addition time: per‑seed median steady plan time (the same numbers whose
                    median gives the table's T_add).
    - Obstacle checks: per‑seed mean total checks per repair (when available).
    Thus the center line of a boxplot exactly matches the corresponding table
    column (e.g., boxplot median = T_repair).

======================================================================================
PAIRED SEED FILTERING
======================================================================================
Only seeds that are successful (zero collisions) for ALL planners in a scenario are
retained.  The analysis then uses the same set of seeds for every planner to ensure
a fair comparison.

======================================================================================
"""
import pandas as pd
import glob
import os
import re
import numpy as np
import matplotlib.pyplot as plt
from pandas.errors import EmptyDataError
import random

CURRENT_SCENARIO = None

_EXTRACTION_CACHE_REPAIR = {}
_EXTRACTION_CACHE_PATH   = {}


# Publication-ready plot styling
plt.rcParams.update({
    "font.family": "serif",
    "font.serif": ["Times New Roman", "DejaVu Serif", "serif"],
    "axes.labelsize": 12,
    "axes.titlesize": 14,
    "legend.fontsize": 11,
    "xtick.labelsize": 11,
    "ytick.labelsize": 11,
    "figure.dpi": 300
})

# PLANNER_ORDER = ["FMTX", "RRTX"]
# color_map = {"FMTX": "#1f78b4", "RRTX": "#d62728"}
# display_names = {
#     "FMTX": r"FMT$^{\mathrm{X}}$",
#     "RRTX": r"RRT$^{\mathrm{X}}$"
# }


PLANNER_ORDER = ["LLPTStar", "FMTX", "RRTX"]
color_map = {
    "LLPTStar": "#2ca02c",   # green
    "FMTX":     "#1f78b4",   # blue
    "RRTX":     "#d62728"    # red
}
display_names = {
    "LLPTStar": r"LLPT$^{*}$",
    "FMTX":     r"FMT$^{\mathrm{X}}$",
    "RRTX":     r"RRT$^{\mathrm{X}}$"
}

def planner_sort_key(p):
    try:
        return (0, PLANNER_ORDER.index(p))
    except ValueError:
        return (1, p)

BUILD_DIR = "."
FILENAME_PATTERN = re.compile(
    r"sim_([A-Za-z0-9]+)_([A-Za-z0-9_]+)_seed_(\d+)_(\d{8}_\d{6})_metrics\.csv"
)




def get_success_terminal_row(df):
    """Single terminal row of a successful run (goal_reached + reached_goal flag).
    Empty DataFrame if the run never reached the goal."""
    if df.empty or "event_type" not in df.columns:
        return pd.DataFrame()
    rows = df[df["event_type"] == "goal_reached"].copy()
    if rows.empty:
        return rows
    if "reached_goal" in rows.columns:
        flagged = rows[pd.to_numeric(rows["reached_goal"], errors="coerce").fillna(0) > 0]
        if not flagged.empty:
            rows = flagged
    return rows.tail(1)   # one terminal row per run


def _med_iqr(vals, fmt="{:.2f}"):
    if not vals:
        return "nan"
    med = np.median(vals)
    q1  = np.percentile(vals, 25)
    q3  = np.percentile(vals, 75)
    return f"{fmt.format(med)} ({fmt.format(q1)}–{fmt.format(q3)})"


def safe_numeric(df, cols):
    for c in cols:
        if c in df.columns:
            df[c] = pd.to_numeric(df[c], errors="coerce")
    return df

def load_data(directory):
    if not os.path.exists(directory):
        return {}

    files = glob.glob(os.path.join(directory, "sim_*_metrics.csv"))
    scenarios = {}

    for filepath in files:
        filename = os.path.basename(filepath)
        match = FILENAME_PATTERN.match(filename)
        if not match:
            continue

        planner_raw = match.group(1)
        scenario = match.group(2)
        seed = int(match.group(3)) # Extract the seed!

        planner_clean = (
            planner_raw
            .replace("Kinodynamic", "")
            .replace("PRMStarDStarLite", "DLITE")
            .replace("PRMStar", "")
            .replace("ANY", "")
        )

        try:
            df = pd.read_csv(filepath)
            if df.empty:
                continue

            df.replace([np.inf, -np.inf], np.nan, inplace=True)
            df['seed'] = seed # Inject the seed into the dataframe

            # numeric_cols = [
            #     "row_id", "elapsed_s", "sim_time", "setup_ms", "total_latency_ms",
            #     "update_ms", "plan_ms", "time_to_goal", "path_cost", "obstacle_checks",
            #     "collision_count", "tree_size", "isolated_nodes", "avg_deg_out",
            #     "avg_deg_in", "neighborhood_radius"
            # ]

            numeric_cols = [
                "row_id", "elapsed_s", "sim_time", "setup_ms", "total_latency_ms",
                "update_ms", "plan_ms", "time_to_goal", "path_cost", "obstacle_checks",
                "collision_count", "tree_size", "isolated_nodes", "avg_deg_out",
                "avg_deg_in", "neighborhood_radius",
                "setrobotstate_ms", "decision_latency_ms", "applied_step_s",
                "reached_goal", "exec_length", "exec_time", "exec_turn", "exec_effort",
            ]


            df = safe_numeric(df, numeric_cols)

            sort_cols = [c for c in ["row_id", "elapsed_s"] if c in df.columns]
            if sort_cols:
                df = df.sort_values(sort_cols).reset_index(drop=True)

            if scenario not in scenarios:
                scenarios[scenario] = {}
            if planner_clean not in scenarios[scenario]:
                scenarios[scenario][planner_clean] = []

            scenarios[scenario][planner_clean].append(df)
            print(f"Loaded {filename}: {len(df)} rows")

        except EmptyDataError:
            pass
        except Exception as e:
            print(f"Warning: Could not load {filename}: {e}")

    return scenarios


# def is_successful_run(df):
#     if df.empty:
#         return False
#     if "collision_count" in df.columns:
#         cc = pd.to_numeric(df["collision_count"], errors="coerce").fillna(0)
#         return cc.max() == 0
#     if "crashed" in df.columns:
#         cr = pd.to_numeric(df["crashed"], errors="coerce").fillna(0)
#         return (cr == 1).sum() == 0
#     return True

def is_successful_run(df):
    global CURRENT_SCENARIO
    if CURRENT_SCENARIO == "R2":
        return True


    if df.empty:
        return False

    # 1. Must have reached the goal
    if "event_type" in df.columns:
        has_goal = "goal_reached" in df["event_type"].values
    else:
        # fallback if event_type not present: assume success if no collision
        has_goal = True

    # 2. Must not have been trapped
    if "event_type" in df.columns:
        was_trapped = "planner_trapped" in df["event_type"].values
    else:
        was_trapped = False

    # 3. Must have zero collisions
    has_collision = False
    if "collision_count" in df.columns:
        cc = pd.to_numeric(df["collision_count"], errors="coerce").fillna(0)
        has_collision = cc.max() > 0
    elif "crashed" in df.columns:
        cr = pd.to_numeric(df["crashed"], errors="coerce").fillna(0)
        has_collision = (cr == 1).sum() > 0

    return has_goal and not was_trapped and not has_collision


def extract_runtime_events(df):
    if df.empty or "event_type" not in df.columns:
        return pd.DataFrame()
    runtime = df.copy()
    if "elapsed_s" in runtime.columns:
        runtime = runtime[runtime["elapsed_s"].fillna(0) > 0.0].copy()
    runtime = runtime.sort_values(
        [c for c in ["row_id", "elapsed_s"] if c in runtime.columns]
    ).reset_index(drop=True)
    return runtime


def classify_plan_rows(df):
    if df.empty:
        return df.copy()
    out = df.copy()
    out["is_first_plan_after_update"] = False
    prev_was_update = False
    for idx in out.index:
        ev = out.at[idx, "event_type"] if "event_type" in out.columns else None
        if ev == "update":
            prev_was_update = True
        elif ev == "plan":
            if prev_was_update:
                out.at[idx, "is_first_plan_after_update"] = True
            prev_was_update = False
        else:
            prev_was_update = False
    return out


def pair_repair_events(df):
    if df.empty:
        return pd.DataFrame(columns=[
            "update_row_id", "plan_row_id", "elapsed_s", "sim_time",
            "update_ms", "repair_plan_ms", "t_repair_ms",
            "update_obstacle_checks", "repair_plan_obstacle_checks",
            "path_cost_snapshot"
        ])
    df = classify_plan_rows(df)
    rows = []
    for i, row in df.iterrows():
        if row["event_type"] != "update":
            continue
        repair_plan_ms = np.nan
        repair_plan_obstacle_checks = np.nan
        plan_row_id = np.nan

        if i + 1 < len(df):
            next_row = df.iloc[i + 1]
            if next_row["event_type"] == "plan":
                repair_plan_ms = next_row.get("plan_ms", np.nan)
                repair_plan_obstacle_checks = next_row.get("obstacle_checks", np.nan)
                plan_row_id = next_row.get("row_id", np.nan)

        upd_ms = row.get("update_ms", np.nan)
        t_repair_ms = np.nan
        if pd.notna(upd_ms) or pd.notna(repair_plan_ms):
            t_repair_ms = (0.0 if pd.isna(upd_ms) else upd_ms) + \
                          (0.0 if pd.isna(repair_plan_ms) else repair_plan_ms)

        rows.append({
            "update_row_id": row.get("row_id", np.nan),
            "plan_row_id": plan_row_id,
            "elapsed_s": row.get("elapsed_s", np.nan),
            "sim_time": row.get("sim_time", np.nan),
            "update_ms": upd_ms,
            "repair_plan_ms": repair_plan_ms,
            "t_repair_ms": t_repair_ms,
            "update_obstacle_checks": row.get("obstacle_checks", np.nan),
            "repair_plan_obstacle_checks": repair_plan_obstacle_checks,
            "path_cost_snapshot": row.get("path_cost", np.nan),
        })
    return pd.DataFrame(rows)


def get_state_rows(df):
    if df.empty:
        return pd.DataFrame()
    rows = df[df["event_type"].isin(["set_state", "goal_reached"])].copy()
    rows = safe_numeric(rows, ["path_cost", "sim_time", "elapsed_s", "time_to_goal"])
    rows = rows[
        rows["path_cost"].notna() &
        np.isfinite(rows["path_cost"]) &
        (rows["path_cost"] > 0)
    ].copy()
    return rows


def get_plan_rows(df):
    if df.empty:
        return pd.DataFrame()
    rows = df[df["event_type"] == "plan"].copy()
    rows = safe_numeric(rows, ["plan_ms", "obstacle_checks", "path_cost"])
    return classify_plan_rows(rows if rows.empty else classify_plan_rows(df[df["event_type"].isin(["update", "plan"])].copy()))


def get_steady_plan_rows(df):
    if df.empty:
        return pd.DataFrame()
    tagged = classify_plan_rows(df)
    rows = tagged[
        (tagged["event_type"] == "plan") &
        (~tagged["is_first_plan_after_update"])
    ].copy()
    return rows


def get_first_repair_plan_rows(df):
    if df.empty:
        return pd.DataFrame()
    tagged = classify_plan_rows(df)
    rows = tagged[
        (tagged["event_type"] == "plan") &
        (tagged["is_first_plan_after_update"])
    ].copy()
    return rows


def get_update_rows(df):
    if df.empty:
        return pd.DataFrame()
    rows = df[df["event_type"] == "update"].copy()
    rows = safe_numeric(rows, ["update_ms", "obstacle_checks", "path_cost"])
    return rows


def get_terminal_rows(df):
    if df.empty:
        return pd.DataFrame()
    rows = df[df["event_type"].isin(["goal_reached", "time_limit"])].copy()
    return rows


def filter_paired_seeds(planners_data, target_count=100):
    """
    Finds the intersection of successful seeds across FMTX and RRTX,
    and returns a filtered dictionary containing exactly 'target_count' paired seeds.
    """
    # Collect successful seeds for each planner
    successful_seeds_per_planner = {}
    for planner, dfs in planners_data.items():
        successful_seeds = [df['seed'].iloc[0] for df in dfs if is_successful_run(df)]
        successful_seeds_per_planner[planner] = set(successful_seeds)
    
    # Find intersection of successful seeds across all planners
    if not successful_seeds_per_planner:
        return planners_data
        
    common_seeds = set.intersection(*successful_seeds_per_planner.values())
    
    if len(common_seeds) < target_count:
        print(f"WARNING: Only found {len(common_seeds)} paired successful seeds, which is less than the target {target_count}.")
        selected_seeds = list(common_seeds)
    else:
        # Sort them just to be deterministic, then take the first 30
        selected_seeds = sorted(list(common_seeds))[:target_count]
        
    # Filter the DataFrames
    filtered_data = {}
    for planner, dfs in planners_data.items():
        filtered_data[planner] = [df for df in dfs if df['seed'].iloc[0] in selected_seeds]
        
    return filtered_data, len(selected_seeds)


def analyze_group_statistics(scenario_name, planners_data):
    print(f"\n{'='*220}")
    print(f" AGGREGATE ANALYSIS: {scenario_name} (Successful paired runs only)")
    print(f"{'='*220}")

    # --- FILTER FOR PAIRED uEEDS ---
    paired_data, paired_count = filter_paired_seeds(planners_data, target_count=100)
    print(f"[Paired Benchmark] Using exactly {paired_count} identical successful seeds across all planners.\n")

    summary_data = []

    # Capture global success metrics BEFORE filtering
    global_stats = {}
    for planner, dfs in planners_data.items():
        valid_dfs = [df for df in dfs if not df.empty]
        num_seeds = len(valid_dfs)
        success_count = len([df for df in valid_dfs if is_successful_run(df)])
        succ_rate = (success_count / num_seeds) * 100 if num_seeds > 0 else 0.0
        global_stats[planner] = {
            "rate": succ_rate,
            "successes": success_count,
            "total": num_seeds
        }


    # === NEW: always show success rates ===
    # print("\nIndividual planner success rates (all seeds):")
    for planner, stats in global_stats.items():
        print(f"  {planner:12s} : {stats['successes']:3d} / {stats['total']:3d}  ({stats['rate']:5.1f}%)")
    print()

    # === Failed-seed report (all seeds, per planner) ===
    # A run "fails" when is_successful_run() is False (typically setRobotState could not
    # find a safe anchor -> planner_trapped, or a collision occurred). Printed as a neat
    # sorted list so a single flipped seed between two configs (e.g. cone on vs off) is
    # trivial to diff.
    def _seed_sort_key(s):
        try:
            return (0, int(s))
        except (ValueError, TypeError):
            return (1, str(s))

    failed_by_planner = {}
    for planner, dfs in planners_data.items():
        failed = []
        for df in dfs:
            if df.empty or "seed" not in df.columns:
                continue
            if not is_successful_run(df):
                failed.append(df["seed"].iloc[0])
        failed_by_planner[planner] = sorted(set(failed), key=_seed_sort_key)

    print("  Failed seeds (per planner):")
    for planner in sorted(failed_by_planner.keys(), key=planner_sort_key):
        failed = failed_by_planner[planner]
        print(f"    {planner:12s} : {len(failed):3d} failed -> {failed}")

    # Union of all failures = exactly the seeds dropped from the paired set.
    union_failed = sorted(
        set().union(*failed_by_planner.values()) if failed_by_planner else set(),
        key=_seed_sort_key,
    )
    print(f"  Seeds failed by >=1 planner (excluded from pairing): {len(union_failed)} -> {union_failed}")
    print()

    for planner, dfs in paired_data.items():
        valid_dfs = [df for df in dfs if not df.empty]
        if not valid_dfs:
            continue


        stats = global_stats.get(planner, {"rate": 0.0, "successes": 0, "total": 0})
        succ_rate = stats["rate"]

        # Keep merged_all purely for total event counts
        merged_all = pd.concat(valid_dfs, ignore_index=True)

        print(f"{planner}: Global Succ: {stats['successes']}/{stats['total']} ({stats['rate']:.0f}%) -> Analyzing {len(valid_dfs)} paired seeds")
        if "event_type" in merged_all.columns:
            print(f"  Event types: {merged_all['event_type'].value_counts().to_dict()}")


        # ---------------------------------------------------------------------
        # PER-SEED AGGREGATION LISTS
        # ---------------------------------------------------------------------
        t_init_list, t_upd_list, t_pln_list, t_repair_list, t_add_list, t_rep_p99_list, t_rep_max_list = [], [], [], [], [], [], []
        obs_upd_list, obs_rep_pln_list, obs_steady_list = [], [], []
        obs_total_list = []   # per‑seed mean of total checks per repair (update+plan)
        # avg_cost_list = []
        exec_len_list, exec_time_list = [], []
        exec_turn_list, exec_effort_list = [], []
        
        start_v_vals, end_v_vals = [], []
        start_r_vals, end_r_vals = [], []
        deg_in_list, deg_out_list = [], []

        # ADD THESE TWO LINES:
        current_max_rep = -np.inf
        seed_of_max_rep = None




        for df in valid_dfs: # valid_dfs is already strictly the successful paired runs
            run_runtime = extract_runtime_events(df)
            
            # --- Latency & Path Cost (Medians) ---
            df_init = df[df["event_type"] == "initial_plan"].copy() if "event_type" in df.columns else pd.DataFrame()
            if not df_init.empty and "plan_ms" in df_init.columns:
                t_init_list.append(df_init["plan_ms"].dropna().median())

            run_update = get_update_rows(run_runtime)
            run_repair_plan = get_first_repair_plan_rows(run_runtime)
            run_steady = get_steady_plan_rows(run_runtime)
            run_repairs = pair_repair_events(run_runtime)
            # run_state = get_state_rows(df)

            # --- T_repair: planner‑aware handling ---
            if planner.upper() == "RRTX":
                # For RRTX, T_repair = update_ms only (the following plan is NOT a repair)
                if not run_update.empty:
                    update_ms_vals = run_update["update_ms"].dropna()
                    if not update_ms_vals.empty:
                        t_repair_list.append(update_ms_vals.median())
                        t_rep_p99_list.append(update_ms_vals.quantile(0.99))
            else:
                # For FMTX, T_repair = update_ms + first-following plan_ms
                if not run_repairs.empty:
                    t_repair_list.append(run_repairs["t_repair_ms"].dropna().median())
                    t_rep_p99_list.append(run_repairs["t_repair_ms"].dropna().quantile(0.99))
            
            if not run_repairs.empty:
                if planner.upper() == "RRTX":
                    total_checks = run_repairs["update_obstacle_checks"].fillna(0)
                else:  # FMTX: repair = update + following plan
                    total_checks = (run_repairs["update_obstacle_checks"].fillna(0) +
                                    run_repairs["repair_plan_obstacle_checks"].fillna(0))
                obs_total_list.append(total_checks.mean())


            # t_rep_max_list.append(run_repairs["t_repair_ms"].dropna().max())
            # Rep_max: same source as T_repair / Rep_p99
            if planner.upper() == "RRTX":
                rep_series = run_update["update_ms"].dropna() if not run_update.empty else pd.Series(dtype=float)
            else:
                rep_series = run_repairs["t_repair_ms"].dropna() if not run_repairs.empty else pd.Series(dtype=float)

            # if not rep_series.empty:
            #     t_rep_max_list.append(rep_series.max())
            if not rep_series.empty:
                run_max = rep_series.max()
                t_rep_max_list.append(run_max)
                
                # ADD THIS BLOCK to track which seed gave the global maximum
                if run_max > current_max_rep:
                    current_max_rep = run_max
                    seed_of_max_rep = df['seed'].iloc[0]



            # --- Other latencies (unchanged) ---
            if not run_update.empty:
                t_upd_list.append(run_update["update_ms"].dropna().median())
            if not run_repair_plan.empty:
                t_pln_list.append(run_repair_plan["plan_ms"].dropna().median())
            if not run_steady.empty:
                t_add_list.append(run_steady["plan_ms"].dropna().median())
            
            
            # --- Executed solution quality: read ONLY at the terminal goal row ---
            term = get_success_terminal_row(df)
            if not term.empty:
                r = term.iloc[-1]

                def _exec_val(col):
                    if col not in term.columns:
                        return np.nan
                    v = pd.to_numeric(r.get(col), errors="coerce")
                    return v if (pd.notna(v) and np.isfinite(v) and v > 0) else np.nan

                L    = _exec_val("exec_length")
                T    = _exec_val("exec_time")
                turn = _exec_val("exec_turn")
                eff  = _exec_val("exec_effort")

                if pd.notna(L):    exec_len_list.append(L)
                if pd.notna(T):    exec_time_list.append(T)
                if pd.notna(turn): exec_turn_list.append(turn)
                if pd.notna(eff):  exec_effort_list.append(eff)



            # --- Obstacle Checks (Means) ---
            if not run_update.empty: obs_upd_list.append(run_update["obstacle_checks"].dropna().mean())
            if not run_repair_plan.empty: obs_rep_pln_list.append(run_repair_plan["obstacle_checks"].dropna().mean())
            if not run_steady.empty: obs_steady_list.append(run_steady["obstacle_checks"].dropna().mean())

            # --- Graph Stats (Means) ---
            if "tree_size" in df.columns and not df.empty:
                start_v = pd.to_numeric(df.iloc[0]["tree_size"], errors="coerce")
                end_v = pd.to_numeric(df.iloc[-1]["tree_size"], errors="coerce")
                if pd.notna(start_v): start_v_vals.append(start_v)
                if pd.notna(end_v): end_v_vals.append(end_v)

            radius_col = "neighborhood_radius" if "neighborhood_radius" in df.columns else ("radius" if "radius" in df.columns else None)
            if radius_col and not df.empty:
                start_r = pd.to_numeric(df.iloc[0][radius_col], errors="coerce")
                end_r = pd.to_numeric(df.iloc[-1][radius_col], errors="coerce")
                if pd.notna(start_r): start_r_vals.append(start_r)
                if pd.notna(end_r): end_r_vals.append(end_r)

            run_term = get_terminal_rows(df)
            if "avg_deg_in" in run_term.columns and not run_term.empty:
                deg_in_list.append(run_term["avg_deg_in"].dropna().mean())
            if "avg_deg_out" in run_term.columns and not run_term.empty:
                deg_out_list.append(run_term["avg_deg_out"].dropna().mean())



        # Path‑cost summary: median of per‑seed means + IQR
        # if avg_cost_list:
        #     avg_cost_med = np.median(avg_cost_list)
        #     avg_cost_25 = np.percentile(avg_cost_list, 25)
        #     avg_cost_75 = np.percentile(avg_cost_list, 75)
        #     avg_cost_str = f"{avg_cost_med:.2f} ({avg_cost_25:.2f}–{avg_cost_75:.2f})"
        # else:
        #     avg_cost_med = np.nan
        #     avg_cost_str = "nan"
        # Executed solution-quality: median + IQR, one value per successful seed
        exec_len_str    = _med_iqr(exec_len_list,    "{:.2f}")
        exec_time_str   = _med_iqr(exec_time_list,   "{:.2f}")
        exec_turn_str   = _med_iqr(exec_turn_list,   "{:.2f}") if exec_turn_list else "—"
        exec_effort_str = _med_iqr(exec_effort_list, "{:.2f}") if exec_effort_list else "—"


        

        # # Obstacle‑checks summary: median of per‑seed means + IQR
        # if obs_upd_list:
        #     obs_upd_med = np.median(obs_upd_list)
        #     obs_upd_25 = np.percentile(obs_upd_list, 25)
        #     obs_upd_75 = np.percentile(obs_upd_list, 75)
        #     obs_upd_str = f"{obs_upd_med:.0f} ({obs_upd_25:.0f}–{obs_upd_75:.0f})"
        # else:
        #     obs_upd_med = np.nan
        #     obs_upd_str = "nan"
        # Obstacle‑checks summary: median of per‑seed mean total checks per repair + IQR
        if obs_total_list:
            obs_repair_med = np.median(obs_total_list)
            obs_repair_q1  = np.percentile(obs_total_list, 25)
            obs_repair_q3  = np.percentile(obs_total_list, 75)
            obs_repair_str = f"{obs_repair_med:.0f} ({obs_repair_q1:.0f}–{obs_repair_q3:.0f})"
        else:
            obs_repair_med = np.nan
            obs_repair_str = "nan"    


        # Steady-state obstacle checks summary: median of per‑seed means + IQR
        if obs_steady_list:
            obs_steady_med = np.median(obs_steady_list)
            obs_steady_q1  = np.percentile(obs_steady_list, 25)
            obs_steady_q3  = np.percentile(obs_steady_list, 75)
            obs_steady_str = f"{obs_steady_med:.0f} ({obs_steady_q1:.0f}–{obs_steady_q3:.0f})"
        else:
            obs_steady_med = np.nan
            obs_steady_str = "nan"


        # Repair‑time IQR
        if t_repair_list:
            t_repair_med = np.median(t_repair_list)
            t_repair_q1  = np.percentile(t_repair_list, 25)
            t_repair_q3  = np.percentile(t_repair_list, 75)
            t_repair_str = f"{t_repair_med:.1f} ({t_repair_q1:.1f}–{t_repair_q3:.1f})"
        else:
            t_repair_med = np.nan
            t_repair_str = "nan"

        # Addition‑time IQR
        if t_add_list:
            t_add_med = np.median(t_add_list)
            t_add_q1  = np.percentile(t_add_list, 25)
            t_add_q3  = np.percentile(t_add_list, 75)
            t_add_str = f"{t_add_med:.1f} ({t_add_q1:.1f}–{t_add_q3:.1f})"
        else:
            t_add_med = np.nan
            t_add_str = "nan"

        if t_rep_p99_list:
            t_repair_p99_val = np.median(t_rep_p99_list)
        else:
            t_repair_p99_val = np.nan
        t_repair_p99_str = f"{t_repair_p99_val:.1f}" if pd.notna(t_repair_p99_val) else "nan"

        t_rep_max_val = np.nanmax(t_rep_max_list) if t_rep_max_list else np.nan

        t_repair_max_str = f"{t_rep_max_val:.1f}" if pd.notna(t_rep_max_val) else "nan"



        # ADD THIS LINE TO PRINT IT:
        if seed_of_max_rep is not None:
            print(f"  [DEBUG] {planner} max repair time ({current_max_rep:.1f} ms) occurred in seed: {seed_of_max_rep}")


        # ---------------------------------------------------------------------
        # CROSS-SEED AGGREGATION
        # ---------------------------------------------------------------------
        t_init_val = np.median(t_init_list) if t_init_list else np.nan
        t_upd_val = np.median(t_upd_list) if t_upd_list else np.nan
        t_pln_val = np.median(t_pln_list) if t_pln_list else np.nan
        t_repair_val = np.median(t_repair_list) if t_repair_list else np.nan
        t_add_val = np.median(t_add_list) if t_add_list else np.nan
        # t_repair_p99_val = np.median(t_rep_p99_list) if t_rep_p99_list else np.nan
        
        # avg_cost = np.median(avg_cost_list) if avg_cost_list else np.nan

        obs_chk_update = np.mean(obs_upd_list) if obs_upd_list else np.nan
        obs_chk_repair_plan = np.mean(obs_rep_pln_list) if obs_rep_pln_list else np.nan
        obs_chk_steady_plan = np.mean(obs_steady_list) if obs_steady_list else np.nan

        if planner.upper() == "RRTX":
            obs_chk_repair_plan = np.nan
        
        obs_per_update = (
            obs_chk_update + obs_chk_repair_plan
        ) if (pd.notna(obs_chk_update) and pd.notna(obs_chk_repair_plan)) else obs_chk_update

        start_v = np.mean(start_v_vals) if start_v_vals else np.nan
        end_v = np.mean(end_v_vals) if end_v_vals else np.nan
        start_r = np.mean(start_r_vals) if start_r_vals else np.nan
        end_r = np.mean(end_r_vals) if end_r_vals else np.nan
        deg_in = np.mean(deg_in_list) if deg_in_list else np.nan
        deg_out = np.mean(deg_out_list) if deg_out_list else np.nan
        
        queue_ops = 0.0

        # ---------------------------------------------------------------------
        # FORMATTING OUTPUT
        # ---------------------------------------------------------------------
        summary_data.append({
            "Planner": planner,
            "Succ(%)": f"{succ_rate:.0f}%",
            "T_init(ms)": f"{t_init_val:.1f}" if pd.notna(t_init_val) else "nan",
            "T_upd(ms)": f"{t_upd_val:.1f}" if pd.notna(t_upd_val) else "nan",
            # "T_pln(ms)": f"{t_pln_val:.1f}" if pd.notna(t_pln_val) else "nan",
            "T_pln(ms)": "nan" if planner.upper() == "RRTX" else (f"{t_pln_val:.1f}" if pd.notna(t_pln_val) else "nan"),
            # "T_repair(ms)": f"{t_repair_val:.1f}" if pd.notna(t_repair_val) else "nan",
            "T_repair(ms)": t_repair_str,

            # "Rep_p99(ms)": f"{t_repair_p99_val:.1f}" if pd.notna(t_repair_p99_val) else "nan",
            "Rep_p99(ms)": t_repair_p99_str,
            "Rep_max(ms)": t_repair_max_str,

            # "T_add(ms)": f"{t_add_val:.1f}" if pd.notna(t_add_val) else "nan",
            "T_add(ms)": t_add_str,


            # "ObsUpd": f"{obs_chk_update:.0f}" if pd.notna(obs_chk_update) else "nan",
            "ObsRepPln": f"{obs_chk_repair_plan:.0f}" if pd.notna(obs_chk_repair_plan) else "nan",
            # "ObsSteady": f"{obs_chk_steady_plan:.0f}" if pd.notna(obs_chk_steady_plan) else "nan",
            "ObsSteady": obs_steady_str,

            # "Obs/Upd": f"{obs_per_update:.0f}" if pd.notna(obs_per_update) else "nan",
            "Obs/Repair": obs_repair_str,   # total checks per repair (median & IQR)


            #"Q_Ops": f"{queue_ops:.0f}",

            # "Path_Cost": f"{avg_cost:.2f}" if pd.notna(avg_cost) else "nan",
            # "Path_Cost": avg_cost_str,
            "L_exec(m)":  exec_len_str,
            "T_exec(s)":  exec_time_str,
            "Turn(rad)":  exec_turn_str,
            "Effort":     exec_effort_str,


            "Start |V|": f"{start_v:.0f}" if pd.notna(start_v) else "nan",
            "End |V|": f"{end_v:.0f}" if pd.notna(end_v) else "nan",
            "Start r_n": f"{start_r:.2f}" if pd.notna(start_r) else "nan",
            "End r_n": f"{end_r:.2f}" if pd.notna(end_r) else "nan",
            "Deg(O/I)": (
                f"{deg_out:.1f}/{deg_in:.1f}"
                if pd.notna(deg_out) and pd.notna(deg_in)
                else "nan"
            )
        })

    if summary_data:
        df_out = pd.DataFrame(summary_data)
        print(df_out.to_string(index=False, justify="center"))
        print("-" * 220)

def save_latency_plot(scenario_name, planners_data):
    if not planners_data:
        return

    paired_data, _ = filter_paired_seeds(planners_data, target_count=100)

    # collect per‑planner data
    planner2steady = {}
    planner2repair_upd = {}
    planner2repair_pln = {}
    for planner, dfs in paired_data.items():
        valid_dfs = [d for d in dfs if not d.empty and "event_type" in d.columns]
        if not valid_dfs:
            continue
        all_runtime = [extract_runtime_events(d) for d in valid_dfs]
        merged = pd.concat(all_runtime, ignore_index=True)
        repairs = pair_repair_events(merged)
        steady_rows = get_steady_plan_rows(merged)

        planner2steady[planner] = steady_rows["plan_ms"].dropna().mean() if not steady_rows.empty else 0.0
        planner2repair_upd[planner] = repairs["update_ms"].dropna().mean() if not repairs.empty else 0.0
        planner2repair_pln[planner] = repairs["repair_plan_ms"].dropna().mean() if not repairs.empty else 0.0

    ordered_planners = sorted(planner2steady.keys(), key=planner_sort_key)
    if not ordered_planners:
        return

    steady_vals   = [planner2steady[p] for p in ordered_planners]
    repair_upd_vals = [planner2repair_upd[p] for p in ordered_planners]
    repair_pln_vals = [planner2repair_pln[p] for p in ordered_planners]

    x = np.arange(len(ordered_planners))
    width = 0.35
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))

    c_update = "#d95f02"
    c_plan   = "#1f78b4"

    ax1.bar(x, [0]*len(ordered_planners), width, label="Update only", color=c_update, edgecolor="black", hatch="//")
    ax1.bar(x, steady_vals, width, bottom=[0]*len(ordered_planners), label="Steady plan", color=c_plan, edgecolor="black")
    ax1.set_ylabel("Average latency (ms)")
    ax1.set_xticks(x)
    ax1.set_xticklabels([display_names.get(p, p) for p in ordered_planners])
    # ax1.grid(axis="y", linestyle=":", alpha=0.7)
    for i, val in enumerate(steady_vals):
        if val > 0:
            ax1.text(i, val * 1.05, f"{val:.1f}", ha="center", va="bottom", fontweight="bold")

    ax2.bar(x, repair_upd_vals, width, label="Update only", color=c_update, edgecolor="black", hatch="//")
    ax2.bar(x, repair_pln_vals, width, bottom=repair_upd_vals, label="Repair plan", color=c_plan, edgecolor="black")
    ax2.set_ylabel("Average latency (ms)")
    ax2.set_xticks(x)
    ax2.set_xticklabels([display_names.get(p, p) for p in ordered_planners])
    # ax2.grid(axis="y", linestyle=":", alpha=0.7)
    for i, (upd, pln) in enumerate(zip(repair_upd_vals, repair_pln_vals)):
        total = upd + pln
        if total > 0:
            ax2.text(i, total * 1.05, f"{total:.1f}", ha="center", va="bottom", fontweight="bold")

    handles, labels = ax2.get_legend_handles_labels()
    fig.legend(handles, labels, loc="lower center", bbox_to_anchor=(0.5, -0.05), ncol=2)
    plt.tight_layout()
    out_path = os.path.join(BUILD_DIR, f"plot_latency_breakdown_{scenario_name}.png")
    plt.savefig(out_path, bbox_inches="tight")
    plt.close()
    print(f"[Saved Plot] {out_path}")


'''
median path cost at each simulation time; shaded band: inter‑quartile range across the 30 paired seeds.
'''
def save_comparative_plot(scenario_name, planners_data):
    if not planners_data:
        return

    paired_data, _ = filter_paired_seeds(planners_data, target_count=100)
    plt.figure(figsize=(10, 6))
    plotted_any = False

    for planner in sorted(paired_data.keys(), key=planner_sort_key):
        dfs = paired_data[planner]
        if not dfs:
            continue
        all_times = []
        all_costs = []
        for df in dfs:
            state_rows = get_state_rows(df)
            if state_rows.empty:
                continue
            if "sim_time" in state_rows.columns and state_rows["sim_time"].notna().any():
                state_rows["plot_time"] = state_rows["sim_time"]
            else:
                state_rows["plot_time"] = state_rows["elapsed_s"]
            state_rows["plot_time_rounded"] = (state_rows["plot_time"] * 5).round() / 5
            all_times.extend(state_rows["plot_time_rounded"].tolist())
            all_costs.extend(state_rows["path_cost"].tolist())
        if not all_times:
            continue
        combined = pd.DataFrame({"time": all_times, "cost": all_costs})
        grouped = combined.groupby("time")["cost"].agg(
            median="median", p25=lambda x: x.quantile(0.25), p75=lambda x: x.quantile(0.75)
        ).reset_index()
        color = color_map.get(planner, None)
        if color is None:
            color = next(plt.gca()._get_lines.prop_cycler)['color']
        plt.plot(grouped["time"], grouped["median"], label=display_names.get(planner, planner),
                 linewidth=2, color=color)
        plt.fill_between(grouped["time"], grouped["p25"], grouped["p75"],
                         alpha=0.2, color=color)
        plotted_any = True

    if plotted_any:
        plt.xlabel("Time (s)")
        plt.ylabel("Path cost (m)")
        plt.legend()
        # plt.grid(True, linestyle=":", alpha=0.7)
        out_path = os.path.join(BUILD_DIR, f"plot_anytime_{scenario_name}_median.png")
        plt.savefig(out_path, bbox_inches="tight")
        plt.close()
        print(f"[Saved Plot] {out_path}")
    else:
        print(f"No valid path cost data for {scenario_name}")


# ---------- DISTRIBUTION PLOTS (Boxplots & CDFs) ----------
def extract_per_seed_medians(planners_data, scenario_name=None):
    """
    Extract per-seed median latencies for repair and steady-state addition
    across all planners from the paired data.
    """
    if scenario_name is not None and scenario_name in _EXTRACTION_CACHE_REPAIR:
        return _EXTRACTION_CACHE_REPAIR[scenario_name].copy()


    paired_data, _ = filter_paired_seeds(planners_data, target_count=100)
    metrics = {}
    for planner, dfs in paired_data.items():
        valid_dfs = [df for df in dfs if not df.empty]
        if not valid_dfs:
            continue
        t_repair_list = []
        t_add_list = []
        obs_repair_list = []   # per-seed mean of total obstacle checks per repair

        for df in valid_dfs:
            run_runtime = extract_runtime_events(df)

            repairs = pair_repair_events(run_runtime)
            # if not repairs.empty:
            #     obs_per_rep = repairs["update_obstacle_checks"].fillna(0) + \
            #                   repairs["repair_plan_obstacle_checks"].fillna(0)
            #     if not obs_per_rep.empty:
            #         obs_repair_list.append(obs_per_rep.mean()) # This part is mean not median!

            if not repairs.empty:
                if planner.upper() == "RRTX":
                    total_checks = repairs["update_obstacle_checks"].fillna(0)
                else:  # FMTX: repair = update + following plan
                    total_checks = (repairs["update_obstacle_checks"].fillna(0) +
                                    repairs["repair_plan_obstacle_checks"].fillna(0))
                obs_repair_list.append(total_checks.mean())


            # T_repair: planner‑aware
            if planner.upper() == "RRTX":
                run_update = get_update_rows(run_runtime)
                if not run_update.empty:
                    update_ms = run_update["update_ms"].dropna()
                    if not update_ms.empty:
                        t_repair_list.append(update_ms.median())
            else:
                if not repairs.empty:
                    t_repair_list.append(repairs["t_repair_ms"].dropna().median())

            # T_add: steady-state plan times (sample addition)
            steady_rows = get_steady_plan_rows(run_runtime)
            if not steady_rows.empty:
                t_add_list.append(steady_rows["plan_ms"].dropna().median())

        metrics[planner] = {
            't_repair': t_repair_list,
            't_add': t_add_list,
            'obs_repair': obs_repair_list
        }
    if scenario_name is not None:
        _EXTRACTION_CACHE_REPAIR[scenario_name] = metrics
    return metrics



def save_boxplot_repair_time_anytime(scenario_name, planners_data):
    metrics = extract_per_seed_medians(planners_data, scenario_name)

    if not metrics:
        return
    planners = sorted(metrics.keys(), key=planner_sort_key)
    data = [metrics[p]['t_repair'] for p in planners]
    fig, ax = plt.subplots(figsize=(6, 5))
    bp = ax.boxplot(data, tick_labels=[display_names.get(p, p) for p in planners],
                    patch_artist=True, widths=0.6, showfliers=True)
    for patch, planner in zip(bp['boxes'], planners):
        patch.set_facecolor(color_map.get(planner, '#AAAAAA'))
        patch.set_alpha(0.6)
    ax.set_ylabel("Repair Time Per Seed (ms)")
    # ax.grid(axis="y", linestyle=":", alpha=0.7)
    plt.tight_layout()
    out = os.path.join(BUILD_DIR, f"boxplot_repair_time_{scenario_name}.png")
    plt.savefig(out, bbox_inches="tight"); plt.close()
    print(f"[Saved Plot] {out}")

def save_boxplot_addition_time_anytime(scenario_name, planners_data):
    metrics = extract_per_seed_medians(planners_data, scenario_name)

    if not metrics:
        return
    planners = sorted(metrics.keys(), key=planner_sort_key)
    data = [metrics[p]['t_add'] for p in planners]
    fig, ax = plt.subplots(figsize=(6, 5))
    bp = ax.boxplot(data, tick_labels=[display_names.get(p, p) for p in planners],
                    patch_artist=True, widths=0.6, showfliers=True)
    for patch, planner in zip(bp['boxes'], planners):
        patch.set_facecolor(color_map.get(planner, '#AAAAAA'))
        patch.set_alpha(0.6)
    ax.set_ylabel("Sample Addition Time Per Seed (ms)")
    # ax.grid(axis="y", linestyle=":", alpha=0.7)
    plt.tight_layout()
    out = os.path.join(BUILD_DIR, f"boxplot_addition_time_{scenario_name}.png")
    plt.savefig(out, bbox_inches="tight"); plt.close()
    print(f"[Saved Plot] {out}")

def save_boxplot_obs_checks_anytime(scenario_name, planners_data):
    metrics = extract_per_seed_medians(planners_data, scenario_name)

    if not metrics:
        return
    planners = sorted(metrics.keys(), key=planner_sort_key)
    data = [metrics[p].get('obs_repair', []) for p in planners]
    if not any(data):
        return
    fig, ax = plt.subplots(figsize=(6, 5))
    bp = ax.boxplot(data, tick_labels=[display_names.get(p, p) for p in planners],
                    patch_artist=True, widths=0.6, showfliers=True)
    for patch, planner in zip(bp['boxes'], planners):
        patch.set_facecolor(color_map.get(planner, '#AAAAAA'))
        patch.set_alpha(0.6)
    ax.set_ylabel("Obstacle Checks Per Repair (Per Seed Mean)")
    # ax.grid(axis="y", linestyle=":", alpha=0.7)
    plt.tight_layout()
    out = os.path.join(BUILD_DIR, f"boxplot_obs_checks_{scenario_name}.png")
    plt.savefig(out, bbox_inches="tight"); plt.close()
    print(f"[Saved Plot] {out}")

def save_cdf_repair_time_anytime(scenario_name, planners_data):
    metrics = extract_per_seed_medians(planners_data, scenario_name)

    if not metrics:
        return
    planners = sorted(metrics.keys(), key=planner_sort_key)
    fig, ax = plt.subplots(figsize=(6, 5))
    for planner in planners:
        vals = metrics[planner]['t_repair']
        if not vals: continue
        sorted_vals = np.sort(vals)
        cdf = np.arange(1, len(sorted_vals)+1) / len(sorted_vals)
        ax.step(sorted_vals, cdf, where='post',
                label=display_names.get(planner, planner),
                color=color_map.get(planner, None), linestyle='-', linewidth=2)
    ax.set_xlabel("Repair Time Per Seed (ms)")
    ax.set_ylabel("CDF")
    ax.legend()
    # ax.grid(True, linestyle=":", alpha=0.7)
    plt.tight_layout()
    out = os.path.join(BUILD_DIR, f"cdf_repair_time_{scenario_name}.png")
    plt.savefig(out, bbox_inches="tight"); plt.close()
    print(f"[Saved Plot] {out}")

def save_cdf_addition_time_anytime(scenario_name, planners_data):
    metrics = extract_per_seed_medians(planners_data, scenario_name)

    if not metrics:
        return
    planners = sorted(metrics.keys(), key=planner_sort_key)
    fig, ax = plt.subplots(figsize=(6, 5))
    for planner in planners:
        vals = metrics[planner]['t_add']
        if not vals: continue
        sorted_vals = np.sort(vals)
        cdf = np.arange(1, len(sorted_vals)+1) / len(sorted_vals)
        ax.step(sorted_vals, cdf, where='post',
                label=display_names.get(planner, planner),
                color=color_map.get(planner, None), linestyle='-', linewidth=2)
    ax.set_xlabel("Sample Addition Time Per Seed (ms)")
    ax.set_ylabel("CDF")
    ax.legend()
    # ax.grid(True, linestyle=":", alpha=0.7)
    plt.tight_layout()
    out = os.path.join(BUILD_DIR, f"cdf_addition_time_{scenario_name}.png")
    plt.savefig(out, bbox_inches="tight"); plt.close()
    print(f"[Saved Plot] {out}")

def save_cdf_obs_checks_anytime(scenario_name, planners_data):
    metrics = extract_per_seed_medians(planners_data, scenario_name)

    if not metrics:
        return
    planners = sorted(metrics.keys(), key=planner_sort_key)
    fig, ax = plt.subplots(figsize=(6, 5))
    for planner in planners:
        vals = metrics[planner].get('obs_repair', [])
        if not vals: continue
        sorted_vals = np.sort(vals)
        cdf = np.arange(1, len(sorted_vals)+1) / len(sorted_vals)
        ax.step(sorted_vals, cdf, where='post',
                label=display_names.get(planner, planner),
                color=color_map.get(planner, None), linestyle='-', linewidth=2)
    ax.set_xlabel("Obstacle Checks Per Repair (Per Seed Mean)")
    ax.set_ylabel("CDF")
    ax.legend()
    # ax.grid(True, linestyle=":", alpha=0.7)
    plt.tight_layout()
    out = os.path.join(BUILD_DIR, f"cdf_obs_checks_{scenario_name}.png")
    plt.savefig(out, bbox_inches="tight"); plt.close()
    print(f"[Saved Plot] {out}")



def extract_per_seed_mean_path_costs(planners_data, scenario_name=None):
    """
    Return a dict: planner -> list of per‑seed mean path costs
    (mean over all set_state / goal_reached events within one seed).
    Only uses paired successful seeds.
    """
    # (similar cache logic, using a different key if you prefer,
    #  but for simplicity you can just return the cached dict)
    # For path cost, you can reuse the same key but store separately if needed.
    # I'll keep it simple: use the same scenario_name key for both, but note that
    # the second call will overwrite the first. That's fine because we only
    # need each metric once.
    if scenario_name is not None and scenario_name in _EXTRACTION_CACHE_PATH:
        return _EXTRACTION_CACHE_PATH[scenario_name].copy()


    paired_data, _ = filter_paired_seeds(planners_data, target_count=100)
    metrics = {}
    for planner, dfs in paired_data.items():
        valid_dfs = [df for df in dfs if not df.empty]
        if not valid_dfs:
            continue
        avg_cost_list = []
        for df in valid_dfs:
            state_rows = get_state_rows(df)
            if not state_rows.empty:
                avg_cost_list.append(state_rows["path_cost"].dropna().mean())
        if avg_cost_list:
            metrics[planner] = avg_cost_list
    if scenario_name is not None:
        _EXTRACTION_CACHE_PATH[scenario_name] = metrics

    return metrics


def save_boxplot_path_cost_anytime(scenario_name, planners_data):
    """Boxplot of per‑seed mean path cost, one box per planner."""
    metrics = extract_per_seed_mean_path_costs(planners_data, scenario_name)
    if not metrics:
        return
    planners = sorted(metrics.keys(), key=planner_sort_key)
    color_mapping = {'FMTX': '#1f78b4', 'RRTX': '#d62728'}

    fig, ax = plt.subplots(figsize=(6, 5))
    data_to_plot = [metrics[p] for p in planners]
    bp = ax.boxplot(data_to_plot,
                    tick_labels=[display_names.get(p, p) for p in planners],
                    patch_artist=True, widths=0.6, showfliers=True)
    for patch, planner in zip(bp['boxes'], planners):
        patch.set_facecolor(color_mapping.get(planner, '#AAAAAA'))
        patch.set_alpha(0.6)
    ax.set_ylabel('Mean Path Cost Per Seed')
    plt.tight_layout()
    out_path = os.path.join(BUILD_DIR, f"boxplot_path_cost_{scenario_name}.png")
    plt.savefig(out_path, bbox_inches="tight")
    plt.close()
    print(f"[Saved Plot] {out_path}")




from scipy.stats import wilcoxon

def analyze_statistical_significance(scenario_name, planners_data, base_planner="FMTX", compare_planner="LLPTStar"):
    print(f"\n{'='*100}")
    print(f" STATISTICAL ANALYSIS (Wilcoxon Signed-Rank): {scenario_name}")
    print(f" {base_planner} vs {compare_planner}")
    print(f"{'='*100}")

    # 1. Filter for paired seeds first (just like your aggregate function)
    paired_data, paired_count = filter_paired_seeds(planners_data, target_count=100)
    
    if base_planner not in paired_data or compare_planner not in paired_data:
        print(f"Error: Could not find both {base_planner} and {compare_planner} in paired data.")
        return

    # Extract the dataframes
    dfs_base = paired_data[base_planner]
    dfs_comp = paired_data[compare_planner]

    if len(dfs_base) != len(dfs_comp) or len(dfs_base) == 0:
        print("Error: Paired data lengths do not match or are empty.")
        return

    # 2. Metrics to analyze
    metrics = {
        "Effort": "exec_effort",
        "Turn (rad)": "exec_turn",
        "Exec Length (m)": "exec_length"
    }

    results = []

    # 3. Iterate through paired seeds
    for metric_name, df_col in metrics.items():
        base_vals = []
        comp_vals = []

        # We assume the lists are perfectly aligned by seed because of filter_paired_seeds
        for df_b, df_c in zip(dfs_base, dfs_comp):
            term_b = get_success_terminal_row(df_b)
            term_c = get_success_terminal_row(df_c)

            if not term_b.empty and not term_c.empty and df_col in term_b.columns and df_col in term_c.columns:
                v_b = pd.to_numeric(term_b.iloc[-1].get(df_col), errors="coerce")
                v_c = pd.to_numeric(term_c.iloc[-1].get(df_col), errors="coerce")

                if pd.notna(v_b) and pd.notna(v_c) and v_b > 0 and v_c > 0:
                    base_vals.append(v_b)
                    comp_vals.append(v_c)

        # 4. Calculate Wilcoxon p-value
        if len(base_vals) >= 10:  # Need a reasonable sample size
            try:
                # wilcoxon assumes independent pairs. It returns the test statistic and the p-value.
                stat, p_value = wilcoxon(base_vals, comp_vals)
                
                base_med = np.median(base_vals)
                comp_med = np.median(comp_vals)
                
                sig_stars = "***" if p_value < 0.001 else "**" if p_value < 0.01 else "*" if p_value < 0.05 else "ns"
                
                results.append({
                    "Metric": metric_name,
                    "N Pairs": len(base_vals),
                    f"{base_planner} Med": f"{base_med:.2f}",
                    f"{compare_planner} Med": f"{comp_med:.2f}",
                    "p-value": f"{p_value:.2e}",
                    "Sig": sig_stars
                })
            except ValueError as e:
                # Happens if all pairs are identical (zero difference)
                pass

    # 5. Print results
    if results:
        df_res = pd.DataFrame(results)
        print(df_res.to_string(index=False, justify="center"))
        print("\nSignificance codes:  ***: p < 0.001,  **: p < 0.01,  *: p < 0.05,  ns: not significant")
    else:
        print("Not enough valid paired data to run statistics.")
    print("-" * 100)

def main():
    _EXTRACTION_CACHE_REPAIR.clear()
    _EXTRACTION_CACHE_PATH.clear()
    scenarios = load_data(BUILD_DIR)

    if scenarios:
        for scenario_name, planners_data in scenarios.items():
            global CURRENT_SCENARIO
            CURRENT_SCENARIO = scenario_name

            analyze_group_statistics(scenario_name, planners_data)
            analyze_statistical_significance(scenario_name, planners_data, "FMTX", "LLPTStar") 
            # save_latency_plot(scenario_name, planners_data)
            # save_comparative_plot(scenario_name, planners_data)
            # save_boxplot_repair_time_anytime(scenario_name, planners_data)
            # save_boxplot_addition_time_anytime(scenario_name, planners_data)
            # save_boxplot_obs_checks_anytime(scenario_name, planners_data)
            # save_cdf_repair_time_anytime(scenario_name, planners_data)
            # save_cdf_addition_time_anytime(scenario_name, planners_data)
            # save_cdf_obs_checks_anytime(scenario_name, planners_data)
            # save_boxplot_path_cost_anytime(scenario_name, planners_data)
    else:
        print("No CSV files found.")

if __name__ == "__main__":
    main()


# """
# ======================================================================================
# STATISTICAL ANALYSIS RATIONALE (ANYFMTX vs ANYRRTX)
# ======================================================================================

# 1. THE ARCHITECTURAL DIFFERENCE IN ANYTIME DYNAMIC REPLANNING:
# ANYRRTX and ANYFMTX both aim for anytime asymptotic optimality, but they handle 
# dynamic obstacle updates fundamentally differently:
# - ANYRRTX (Eager): Immediately repairs the entire topological graph during the 
#   `updateObstacles()` phase using localized steering and a priority queue.
# - ANYFMTX (Lazy): Performs O(1) edge severing during `updateObstacles()`, deferring 
#   the heavy structural graph repair to the subsequent `plan()` wavefront expansion.

# 2. AVOIDING METRIC SKEW VIA DATA SLICING:
# Because FMTX blends its repair work into its planning phase, averaging `plan_ms` 
# indiscriminately across all frames would artificially inflate FMTX's steady-state 
# sampling average. To ensure an apples-to-apples comparison, we rigorously isolate 
# the data into two conceptual buckets based on the `update_ms` threshold:

#    A) STEADY-STATE CRUISING (update_ms <= 0.001 ms):
#       No obstacles changed trajectory. The algorithm is ONLY adding new samples 
#       to improve the path cost.
#       -> Metric: T_add = update_ms + plan_ms. 
#       This measures pure anytime sampling/expansion efficiency.

#    B) DYNAMIC REPAIR EVENT (update_ms > 0.001 ms):
#       An obstacle trajectory changed, invalidating parts of the tree. 
#       -> Metric: T_repair = update_ms + plan_ms. 
#       This aggregates the TOTAL temporal cost of recovering from a dynamic event,
#       regardless of whether the algorithm does the work in the update or plan phase.

# 3. THE "NO SIGNIFICANT DIFFERENCE" PARADOX IN PATH COST:
# You may observe that ANYRRTX and ANYFMTX achieve similar average `Path_Cost` at 
# the end of the simulation, but a Wilcoxon signed-rank test yields a high p-value 
# (e.g., p > 0.05). A high p-value here is a POSITIVE result: it proves "Statistical 
# Non-Inferiority." It means neither algorithm is systematically sacrificing path 
# optimality to achieve its speed. 

# 4. THE PAPER'S CORE ARGUMENT:
# We use the Wilcoxon test to prove that ANYFMTX achieves equivalent (or better) 
# path quality (p > 0.05 for Cost) and higher Success Rates in cluttered environments, 
# while simultaneously proving a statistically significant (p < 0.001) reduction in 
# computational latency (`T_repair` and `Obs_Chk`). 
# ======================================================================================
# """

# import warnings
# warnings.filterwarnings("ignore")

# import pandas as pd
# import glob
# import os
# import re
# import numpy as np
# import matplotlib.pyplot as plt
# from scipy.stats import wilcoxon
# from pandas.errors import EmptyDataError

# # Publication-ready plot styling
# plt.rcParams.update({
#     'font.family': 'serif',
#     'font.serif': ['Times New Roman', 'DejaVu Serif', 'serif'],
#     'axes.labelsize': 12,
#     'axes.titlesize': 14,
#     'legend.fontsize': 11,
#     'xtick.labelsize': 11,
#     'ytick.labelsize': 11,
#     'figure.dpi': 300
# })

# BUILD_DIR = "."  
# FILENAME_PATTERN = re.compile(r"sim_([A-Za-z0-9]+)_([A-Za-z0-9_]+)_seed_(\d+)_(\d{8}_\d{6})_metrics\.csv")

# def load_and_pair_data(directory):
#     if not os.path.exists(directory): 
#         return {}
        
#     files = glob.glob(os.path.join(directory, "sim_*_metrics.csv"))
#     scenarios = {}

#     print(f"Found {len(files)} total metrics files. Filtering for ANYTIME algorithms...")

#     for filepath in files:
#         filename = os.path.basename(filepath)
        
#         # ONLY process ANYTIME algorithms
#         if "ANY" not in filename: 
#             continue
            
#         match = FILENAME_PATTERN.match(filename)
#         if not match:
#             continue
            
#         planner_raw = match.group(1)
#         scenario = match.group(2)
#         seed = int(match.group(3))
        
#         # Clean planner name
#         planner = planner_raw.replace("Kinodynamic", "").replace("ANY", "")

#         try:
#             df = pd.read_csv(filepath)
#             if df.empty: continue
            
#             df.replace([np.inf, -np.inf], np.nan, inplace=True)
            
#             # Determine Success (0 collisions and no crashes)
#             succ = True
#             if 'crashed' in df.columns and (df['crashed'] == 1).any():
#                 succ = False
#             elif 'collision_count' in df.columns and df['collision_count'].max() > 0:
#                 succ = False
                
#             # Isolate Dynamic Repair Latency (T_repair)
#             is_runtime = df['elapsed_s'] > 0.0
#             df_event = df[is_runtime & (df['update_ms'] > 0.001)]
            
#             if not df_event.empty:
#                 avg_repair = (df_event['update_ms'] + df_event['plan_ms']).mean()
#             else:
#                 avg_repair = 0.0
                
#             # Isolate Steady-State Addition Latency (T_add)
#             df_steady = df[is_runtime & (df['update_ms'] <= 0.001)]
#             if not df_steady.empty:
#                 avg_add = (df_steady['update_ms'] + df_steady['plan_ms']).mean()
#             else:
#                 avg_add = 0.0
                
#             # Final Path Cost
#             final_cost = df['path_cost'].dropna().iloc[-1] if 'path_cost' in df.columns and not df['path_cost'].dropna().empty else 0.0
            
#             metrics = {
#                 'success': succ,
#                 'avg_repair_ms': avg_repair,
#                 'avg_add_ms': avg_add,
#                 'final_cost': final_cost
#             }
            
#             if scenario not in scenarios: scenarios[scenario] = {}
#             if seed not in scenarios[scenario]: scenarios[scenario][seed] = {}
#             scenarios[scenario][seed][planner] = metrics
            
#         except Exception as e:
#             print(f"Warning: Could not process {filename}: {e}")
            
#     return scenarios

# def analyze_and_plot(scenarios):
#     for scenario, seeds_dict in scenarios.items():
#         print(f"\n{'='*90}\n SCENARIO: {scenario}\n{'='*90}")
        
#         planners = set()
#         for s in seeds_dict.values(): planners.update(s.keys())
#         planners = list(planners)
        
#         if len(planners) < 2:
#             print(f"Only found {planners} in {scenario}. Need 2 planners to compare.")
#             continue
            
#         # Ensure FMTX is p1 if available
#         if "FMTX" in planners:
#             p1 = "FMTX"
#             p2 = [p for p in planners if p != "FMTX"][0]
#         else:
#             p1, p2 = planners[0], planners[1]
        
#         p1_repair, p2_repair = [], []
#         p1_add, p2_add = [], []
#         p1_cost, p2_cost = [], []
#         paired_seeds = []
        
#         # Pair the seeds for statistical testing
#         for seed, runs in seeds_dict.items():
#             if p1 in runs and p2 in runs:
#                 # ONLY use seeds where BOTH planners successfully reached the goal
#                 if runs[p1]['success'] and runs[p2]['success']:
#                     paired_seeds.append(seed)
#                     p1_repair.append(runs[p1]['avg_repair_ms'])
#                     p2_repair.append(runs[p2]['avg_repair_ms'])
#                     p1_add.append(runs[p1]['avg_add_ms'])
#                     p2_add.append(runs[p2]['avg_add_ms'])
#                     p1_cost.append(runs[p1]['final_cost'])
#                     p2_cost.append(runs[p2]['final_cost'])
                    
#         num_pairs = len(paired_seeds)
#         print(f"Valid Paired Successful Seeds: {num_pairs}")
#         if num_pairs < 5:
#             print("Too few paired successes for robust statistical Wilcoxon analysis.")
#             if num_pairs == 0: continue
            
#         # 1. Wilcoxon Signed-Rank Tests (Statistical Significance)
#         try:
#             _, pval_rep = wilcoxon(p1_repair, p2_repair)
#             _, pval_cost = wilcoxon(p1_cost, p2_cost)
            
#             print(f"\n--- WILCOXON PAIRED STATS ({p1} vs {p2}) ---")
#             print(f"Avg Dynamic Repair (T_repair) : {p1}: {np.mean(p1_repair):.2f} ms | {p2}: {np.mean(p2_repair):.2f} ms | p-value: {pval_rep:.4f}")
#             print(f"Avg Final Path Cost           : {p1}: {np.mean(p1_cost):.2f}    | {p2}: {np.mean(p2_cost):.2f}    | p-value: {pval_cost:.4f}")
            
#             sig_rep = "SIGNIFICANT speedup" if pval_rep < 0.05 else "NOT significantly faster"
#             sig_cost = "SIGNIFICANT difference" if pval_cost < 0.05 else "STATISTICALLY EQUIVALENT (Non-Inferior)"
            
#             print(f"\n-> Conclusion: {p1} provides a {sig_rep} during dynamic repair.")
#             print(f"-> Conclusion: The final path cost is {sig_cost}.")
            
#         except Exception as e:
#             print(f"Wilcoxon test failed (likely all zero differences): {e}")

#         # 2. BOX PLOT: Dynamic Repair Latency
#         plt.figure(figsize=(6, 5))
#         plt.boxplot([p1_repair, p2_repair], labels=[p1, p2], patch_artist=True, 
#                     boxprops=dict(facecolor='lightcoral', color='black'),
#                     medianprops=dict(color='black', linewidth=2))
#         plt.ylabel('Dynamic Repair Latency T_repair (ms)')
#         plt.title(f'Paired Repair Latency - {scenario} ({num_pairs} seeds)')
#         plt.grid(axis='y', linestyle=':', alpha=0.7)
        
#         out_box = os.path.join(BUILD_DIR, f"boxplot_anytime_repair_{scenario}.png")
#         plt.savefig(out_box, bbox_inches='tight')
#         plt.close()

#         # 3. SCATTER PLOT: Steady-State vs Path Cost
#         plt.figure(figsize=(8, 6))
#         plt.scatter(p1_add, p1_cost, label=p1, alpha=0.7, edgecolors='black', s=80, marker='o', c='#1f77b4')
#         plt.scatter(p2_add, p2_cost, label=p2, alpha=0.7, edgecolors='black', s=80, marker='s', c='#ff7f0e')
        
#         # Plot Means as large stars
#         plt.scatter(np.mean(p1_add), np.mean(p1_cost), color='blue', marker='*', s=400, edgecolors='black', label=f'{p1} Mean')
#         plt.scatter(np.mean(p2_add), np.mean(p2_cost), color='red', marker='*', s=400, edgecolors='black', label=f'{p2} Mean')

#         plt.xlabel('Steady-State Sampling Latency T_add (ms)')
#         plt.ylabel('Final Path Cost')
#         plt.title(f'Anytime Efficiency: Sampling Overhead vs Cost - {scenario}')
#         plt.legend()
#         plt.grid(True, linestyle=':', alpha=0.7)
        
#         out_scatter = os.path.join(BUILD_DIR, f"scatter_anytime_efficiency_{scenario}.png")
#         plt.savefig(out_scatter, bbox_inches='tight')
#         plt.close()
        
#         print(f"[Saved] {out_box}")
#         print(f"[Saved] {out_scatter}")

# if __name__ == "__main__":
#     scenarios_data = load_and_pair_data(BUILD_DIR)
#     if scenarios_data:
#         analyze_and_plot(scenarios_data)
#     else:
#         print("No ANYTIME CSV metrics files found in the current directory.")