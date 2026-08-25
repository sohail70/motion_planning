"""
======================================================================================
BENCHMARKING METHODOLOGY & METRIC ISOLATION (NON-ANYTIME VERSION)
======================================================================================
This script processes high-resolution, event-based timeline metrics from the C++ planner.

Event semantics:
- initial_plan   : one-time initialization plan (Graph construction & setup)
- set_state      : robot-state snapshot; path_cost belongs here
- update         : obstacle update & repair triggered here
- slice_end      : end of real-time slice
- goal_reached   : terminal success snapshot
- time_limit     : terminal timeout snapshot

Metric isolation policy:
1. T_repair:
   Per repair event = update_ms
   Because plan() happens inside updateObstacles() in non-anytime planners.
   Aggregation: per-seed median of update_ms → cross-seed median of those per-seed medians.
   Using median inside each seed avoids skew from a few extremely slow repairs,
   while the global median summarises typical planner latency robustly.

2. Path cost (solution quality):
   Source: set_state / goal_reached rows only (instantaneous planned cost-to-go).
   Since the robot always finishes exactly at the time budget, total simulation time
   does not discriminate planner quality. Instead we measure how much suboptimal
   remaining cost the robot endured over the whole run.
   - Within one seed: we compute the **mean** of all valid path_cost values.
     The mean captures prolonged detours: if the robot stays at high cost for a
     long time, the mean rises. The median would hide that duration effect,
     and in our case high-cost episodes are exactly the behaviour we want to
     penalise – they are the signal, not outliers.
   - Across seeds: we report the **median** of the per-seed means.
     This provides an outlier-robust summary (one unusually hard seed won't
     inflate the central value) and corresponds to the "typical" mission quality.

3. Obstacle checks:
   Reported as Obs/Upd directly from the 'update' event.
   Per-seed: mean obstacle checks per update event.
   Cross-seed: median of those per-seed means.

4. Fixed Graph Stats:
   Samples, r_n, Setup(ms), and Isolated nodes are parsed from the 'initial_plan' event.

======================================================================================
"""

import pandas as pd
import glob
import os
import re
import numpy as np
import matplotlib.pyplot as plt
from pandas.errors import EmptyDataError

CURRENT_SCENARIO = None


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

PLANNER_ORDER = ["FMTX", "DLITE"]
color_map = {"FMTX": "#1f78b4", "DLITE": "#d62728"}
display_names = {
    "FMTX":    r"D-FMT$^*$",
    "DLITE":   r"D$^*$Lite"
}


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


def planner_sort_key(p):
    """Return sort key: FMTX first, then DLITE, then others alphabetically."""
    try:
        return (0, PLANNER_ORDER.index(p))
    except ValueError:
        return (1, p)

BUILD_DIR = "."
FILENAME_PATTERN = re.compile(
    r"sim_([A-Za-z0-9]+)_([A-Za-z0-9_]+)_seed_(\d+)_(\d{8}_\d{6})_metrics\.csv"
)

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
        seed = int(match.group(3)) # Extract the seed

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
            df['seed'] = seed # Inject seed into dataframe

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
    Finds the intersection of successful seeds across planners,
    and returns a filtered dictionary containing exactly 'target_count' paired seeds.
    """
    successful_seeds_per_planner = {}
    for planner, dfs in planners_data.items():
        successful_seeds = [df['seed'].iloc[0] for df in dfs if is_successful_run(df)]
        successful_seeds_per_planner[planner] = set(successful_seeds)
    
    if not successful_seeds_per_planner:
        return planners_data, 0
        
    common_seeds = set.intersection(*successful_seeds_per_planner.values())
    
    if len(common_seeds) < target_count:
        print(f"WARNING: Only found {len(common_seeds)} paired successful seeds, which is less than the target {target_count}.")
        selected_seeds = list(common_seeds)
    else:
        # Sort them to be deterministic, then take the first 30
        selected_seeds = sorted(list(common_seeds))[:target_count]
        
    filtered_data = {}
    for planner, dfs in planners_data.items():
        filtered_data[planner] = [df for df in dfs if df['seed'].iloc[0] in selected_seeds]
        
    return filtered_data, len(selected_seeds)


def save_latency_plot(scenario_name, planners_data):
    if not planners_data:
        return

    # Filter to identical seeds
    paired_data, _ = filter_paired_seeds(planners_data, target_count=100)

    planners = []
    repair_update = []

    # collect per‑planner data
    planner2repair = {}
    for planner, dfs in paired_data.items():
        valid_dfs = [df for df in dfs if not df.empty and "event_type" in df.columns]
        if not valid_dfs:
            continue
        per_seed_medians = []
        for df in valid_dfs:
            runtime_df = extract_runtime_events(df)
            upd_rows = get_update_rows(runtime_df)
            if not upd_rows.empty:
                per_seed_medians.append(upd_rows["update_ms"].dropna().median())
        planner2repair[planner] = np.median(per_seed_medians) if per_seed_medians else 0.0

    # fixed order
    ordered_planners = sorted(planner2repair.keys(), key=planner_sort_key)
    repair_update = [planner2repair[p] for p in ordered_planners]



    x = np.arange(len(ordered_planners))
    width = 0.5
    fig, ax = plt.subplots(figsize=(6, 5))

    c_update = "#d95f02"
    bars = ax.bar(x, repair_update, width,
                  color=[color_map.get(p, c_update) for p in ordered_planners],
                  edgecolor="black")

    ax.set_title("Dynamic Replan Event Latency (Non-Anytime)")
    ax.set_ylabel("Average Latency (ms)")
    ax.set_xticks(x)
    ax.set_xticklabels([display_names.get(p, p) for p in ordered_planners])
    # ax.grid(axis="y", linestyle=":", alpha=0.7)

    for bar in bars:
        height = bar.get_height()
        if height > 0:
            ax.text(bar.get_x() + bar.get_width()/2., height * 1.05, f"{height:.1f}", ha="center", va="bottom", fontweight="bold")

    plt.suptitle(f"Algorithmic Latency (Paired Seeds): {scenario_name}", y=1.05)
    plt.tight_layout()

    out_path = os.path.join(BUILD_DIR, f"plot_latency_breakdown_{scenario_name}_non_anytime.png")
    plt.savefig(out_path, bbox_inches="tight")
    plt.close()
    print(f"[Saved Plot] {out_path}")


def analyze_group_statistics(scenario_name, planners_data):
    print(f"\n{'='*200}")
    print(f" NON-ANYTIME ANALYSIS: {scenario_name} (Successful paired runs only)")
    print(f"{'='*200}")

    paired_data, paired_count = filter_paired_seeds(planners_data, target_count=100)
    print(f"[Paired Benchmark] Using exactly {paired_count} identical successful seeds across all planners.\n")

    summary_data = []

    # Capture global success metrics before filtering
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

    for planner, dfs in paired_data.items():
        valid_dfs = [df for df in dfs if not df.empty]
        if not valid_dfs:
            continue

        stats = global_stats.get(planner, {"rate": 0.0, "successes": 0, "total": 0})
        succ_rate = stats["rate"]
        merged_all = pd.concat(valid_dfs, ignore_index=True)

        print(f"{planner}: Global Succ: {stats['successes']}/{stats['total']} ({stats['rate']:.0f}%) -> Analyzing {len(valid_dfs)} paired seeds")


        if "event_type" in merged_all.columns:
            print(f"  Event types: {merged_all['event_type'].value_counts().to_dict()}")

        # ---------------------------------------------------------------------
        # PER-SEED AGGREGATION LISTS
        # ---------------------------------------------------------------------

        setup_list, t_init_list, isolated_list, samples_list, r_n_list = [], [], [], [], []
        t_repair_list, t_repair_p99_list, obs_upd_list = [], [], []
        deg_in_list, deg_out_list = [], []
        exec_len_list, exec_time_list = [], []
        exec_turn_list, exec_effort_list = [], []



        for df in valid_dfs:
            run_runtime = extract_runtime_events(df)
            df_init = df[df["event_type"] == "initial_plan"].copy() if "event_type" in df.columns else pd.DataFrame()
            run_update = get_update_rows(run_runtime)
            # run_state = get_state_rows(df)
            run_term = get_terminal_rows(df)

            # Init metrics (1 event per run)
            if not df_init.empty:
                if "setup_ms" in df_init.columns: setup_list.append(df_init["setup_ms"].dropna().median())
                if "plan_ms" in df_init.columns: t_init_list.append(df_init["plan_ms"].dropna().median())
                if "isolated_nodes" in df_init.columns: isolated_list.append(df_init["isolated_nodes"].dropna().mean())
                if "tree_size" in df_init.columns: samples_list.append(df_init["tree_size"].dropna().mean())
                
                radius_col = "neighborhood_radius" if "neighborhood_radius" in df_init.columns else ("radius" if "radius" in df_init.columns else None)
                if radius_col: r_n_list.append(df_init[radius_col].dropna().mean())

            # Repair latency and counts
            if not run_update.empty: 
                t_repair_list.append(run_update["update_ms"].dropna().median())
                t_repair_p99_list.append(run_update["update_ms"].dropna().quantile(0.99))
                obs_upd_list.append(run_update["obstacle_checks"].dropna().mean())

            # Path Cost
            # if not run_state.empty: 
            #     avg_cost_list.append(run_state["path_cost"].dropna().mean())
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



            # Graph degrees (terminal event)
            if not run_term.empty:
                if "avg_deg_in" in run_term.columns: deg_in_list.append(run_term["avg_deg_in"].dropna().mean())
                if "avg_deg_out" in run_term.columns: deg_out_list.append(run_term["avg_deg_out"].dropna().mean())


        # # Path‑cost summary: median of per‑seed means + IQR
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




        # Obstacle‑checks summary: median of per‑seed means + IQR
        if obs_upd_list:
            obs_upd_med = np.median(obs_upd_list)
            obs_upd_25 = np.percentile(obs_upd_list, 25)
            obs_upd_75 = np.percentile(obs_upd_list, 75)
            obs_upd_str = f"{obs_upd_med:.0f} ({obs_upd_25:.0f}–{obs_upd_75:.0f})"
        else:
            obs_upd_med = np.nan
            obs_upd_str = "nan"

        # repair
        if t_repair_list:
            t_repair_med = np.median(t_repair_list)
            t_repair_q1  = np.percentile(t_repair_list, 25)
            t_repair_q3  = np.percentile(t_repair_list, 75)
            t_repair_str = f"{t_repair_med:.1f} ({t_repair_q1:.1f}–{t_repair_q3:.1f})"
        else:
            t_repair_med = np.nan
            t_repair_str = "nan"

        if t_repair_p99_list:
            t_repair_p99_val = np.median(t_repair_p99_list)
            t_repair_p99_str = f"{t_repair_p99_val:.1f}"
        else:
            t_repair_p99_str = "nan"



        # ---------------------------------------------------------------------
        # CROSS-SEED AGGREGATION
        # ---------------------------------------------------------------------
        # Median of Medians for Times
        setup_val = np.median(setup_list) if setup_list else np.nan
        t_init_val = np.median(t_init_list) if t_init_list else np.nan
        t_repair_val = np.median(t_repair_list) if t_repair_list else np.nan
        # t_repair_p99_val = np.median(t_repair_p99_list) if t_repair_p99_list else np.nan
        # avg_cost_val = np.median(avg_cost_list) if avg_cost_list else np.nan

        # Mean of Means for Counts/Graph stats
        obs_upd_val = np.mean(obs_upd_list) if obs_upd_list else np.nan
        samples_val = np.mean(samples_list) if samples_list else np.nan
        isolated_val = np.mean(isolated_list) if isolated_list else np.nan
        r_n_val = np.mean(r_n_list) if r_n_list else np.nan
        deg_in_val = np.mean(deg_in_list) if deg_in_list else np.nan
        deg_out_val = np.mean(deg_out_list) if deg_out_list else np.nan

        summary_data.append({
            "Planner": display_names.get(planner, planner),
            "Succ(%)": f"{succ_rate:.0f}%",
            "Setup(ms)": f"{setup_val:.1f}" if pd.notna(setup_val) else "nan",
            "T_init(ms)": f"{t_init_val:.1f}" if pd.notna(t_init_val) else "nan",
            # "T_repair(ms)": f"{t_repair_val:.1f}" if pd.notna(t_repair_val) else "nan",
            "T_repair(ms)": t_repair_str,
            # "Rep_p99(ms)": f"{t_repair_p99_val:.1f}" if pd.notna(t_repair_p99_val) else "nan",
            "Rep_p99(ms)": t_repair_p99_str,

            # "Obs/Upd": f"{obs_upd_val:.0f}" if pd.notna(obs_upd_val) else "nan",
            "Obs/Upd": obs_upd_str,
            #"Path_Cost": f"{avg_cost_val:.2f}" if pd.notna(avg_cost_val) else "nan",
            # "Path_Cost": avg_cost_str,
            "L_exec(m)":  exec_len_str,
            "T_exec(s)":  exec_time_str,
            "Turn(rad)":  exec_turn_str,
            "Effort":     exec_effort_str,

            "Samples": f"{samples_val:.0f}" if pd.notna(samples_val) else "nan",
            "Isolated": f"{isolated_val:.0f}" if pd.notna(isolated_val) else "nan",
            "r_n": f"{r_n_val:.2f}" if pd.notna(r_n_val) else "nan",
            "Deg(I/O)": (
                f"{deg_in_val:.1f}/{deg_out_val:.1f}"
                if pd.notna(deg_in_val) and pd.notna(deg_out_val)
                else "nan"
            )
        })

    if summary_data:
        df_out = pd.DataFrame(summary_data)
        print(df_out.to_string(index=False, justify="center"))
        print("-" * 200)



def save_comparative_plot(scenario_name, planners_data):
    if not planners_data:
        return

    # Filter to identical seeds
    paired_data, _ = filter_paired_seeds(planners_data, target_count=100)

    # Fixed colour mapping: FMTX blue, DLITE red (others fall back to default cycle)
    color_map = {
        "FMTX":  "#1f78b4",   # blue
        "DLITE": "#d62728",   # red
    }

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

            # Group events into 0.2 second bins for cross-seed comparison
            state_rows["plot_time_rounded"] = (state_rows["plot_time"] * 5).round() / 5

            all_times.extend(state_rows["plot_time_rounded"].tolist())
            all_costs.extend(state_rows["path_cost"].tolist())

        if not all_times:
            continue

        combined = pd.DataFrame({"time": all_times, "cost": all_costs})

        grouped = combined.groupby("time")["cost"].agg(
            median="median",
            p25=lambda x: x.quantile(0.25),
            p75=lambda x: x.quantile(0.75)
        ).reset_index()

        # Get the correct colour for this planner
        color = color_map.get(planner, None)
        if color is None:
            # Fallback to default colour cycle
            color = next(plt.gca()._get_lines.prop_cycler)['color']

        plt.plot(grouped["time"], grouped["median"], label=display_names.get(planner, planner), linewidth=2, color=color)
        plt.fill_between(
            grouped["time"],
            grouped["p25"],
            grouped["p75"],
            alpha=0.2,
            color=color
        )
        plotted_any = True

    if plotted_any:
        plt.xlabel("Time (s)")
        plt.ylabel("Path Cost (m)")
        # Title removed as requested
        plt.legend()
        # plt.grid(True, linestyle=":", alpha=0.7)

        out_path = os.path.join(BUILD_DIR, f"plot_non_anytime_{scenario_name}_median.png")
        plt.savefig(out_path, bbox_inches="tight")
        plt.close()
        print(f"[Saved Plot] {out_path}")


# ---------- DISTRIBUTION PLOTS (Non‑Anytime) ----------

def extract_non_anytime_per_seed_medians(planners_data):
    """
    Extract per-seed median repair time and mean obstacle checks per repair
    for non‑anytime planners (FMTX, DLITE, etc.).
    """
    paired_data, _ = filter_paired_seeds(planners_data, target_count=100)
    metrics = {}

    for planner, dfs in paired_data.items():
        valid_dfs = [df for df in dfs if not df.empty]
        if not valid_dfs:
            continue
        t_repair_list = []
        obs_repair_list = []   # per-seed mean obstacle checks per update event

        for df in valid_dfs:
            runtime_df = extract_runtime_events(df)
            run_update = get_update_rows(runtime_df)

            if not run_update.empty:
                # Median repair time
                t_repair_list.append(run_update["update_ms"].dropna().median())
                # Mean obstacle checks per repair event
                obs_checks = run_update["obstacle_checks"].dropna().mean()
                if pd.notna(obs_checks):
                    obs_repair_list.append(obs_checks)

        if t_repair_list:   # only include if there is at least one seed with repairs
            metrics[planner] = {
                't_repair': t_repair_list,
                'obs_repair': obs_repair_list
            }
    return metrics


# --- Separate boxplots for repair time and obstacle checks ---

def save_boxplot_repair_time_nonanytime(scenario_name, planners_data):
    metrics = extract_non_anytime_per_seed_medians(planners_data)
    if not metrics:
        return
    planners = sorted(metrics.keys(), key=planner_sort_key)
    color_mapping = {'FMTX': '#1f78b4', 'DLITE': '#d62728'}

    fig, ax = plt.subplots(figsize=(6, 5))
    data_to_plot = [metrics[p]['t_repair'] for p in planners]
    bp = ax.boxplot(data_to_plot,tick_labels=[display_names.get(p, p) for p in planners] , patch_artist=True,
                    widths=0.6, showfliers=True)
    for patch, planner in zip(bp['boxes'], planners):
        patch.set_facecolor(color_mapping.get(planner, '#AAAAAA'))
        patch.set_alpha(0.6)
    ax.set_ylabel('Repair Time Per Seed (ms)')
    # ax.grid(axis='y', linestyle=':', alpha=0.7)
    plt.tight_layout()
    out_path = os.path.join(BUILD_DIR, f"boxplot_repair_time_{scenario_name}.png")
    plt.savefig(out_path, bbox_inches="tight")
    plt.close()
    print(f"[Saved Plot] {out_path}")


def save_boxplot_obs_checks_nonanytime(scenario_name, planners_data):
    metrics = extract_non_anytime_per_seed_medians(planners_data)
    if not metrics:
        return
    planners = sorted(metrics.keys(), key=planner_sort_key)
    color_mapping = {'FMTX': '#1f78b4', 'DLITE': '#d62728'}

    fig, ax = plt.subplots(figsize=(6, 5))
    data_to_plot = [metrics[p]['obs_repair'] for p in planners]
    bp = ax.boxplot(data_to_plot, tick_labels=[display_names.get(p, p) for p in planners] , patch_artist=True,
                    widths=0.6, showfliers=True)
    for patch, planner in zip(bp['boxes'], planners):
        patch.set_facecolor(color_mapping.get(planner, '#AAAAAA'))
        patch.set_alpha(0.6)
    ax.set_ylabel('Obstacle Checks Per Event (Per Seed Mean)')
    # ax.grid(axis='y', linestyle=':', alpha=0.7)
    plt.tight_layout()
    out_path = os.path.join(BUILD_DIR, f"boxplot_obs_checks_{scenario_name}.png")
    plt.savefig(out_path, bbox_inches="tight")
    plt.close()
    print(f"[Saved Plot] {out_path}")


# --- Separate CDF plots for repair time and obstacle checks ---

def save_cdf_repair_time_nonanytime(scenario_name, planners_data):
    metrics = extract_non_anytime_per_seed_medians(planners_data)
    if not metrics:
        return
    planners = sorted(metrics.keys(), key=planner_sort_key)
    color_mapping = {'FMTX': '#1f78b4', 'DLITE': '#d62728'}

    fig, ax = plt.subplots(figsize=(6, 5))
    for planner in planners:
        vals = metrics[planner]['t_repair']
        if not vals:
            continue
        sorted_vals = np.sort(vals)
        cdf = np.arange(1, len(sorted_vals)+1) / len(sorted_vals)
        ax.step(sorted_vals, cdf, where='post',
                label=planner, color=color_mapping.get(planner, None),
                linestyle='-', linewidth=2)
    ax.set_xlabel('Repair Time Per Seed (ms)')
    ax.set_ylabel('CDF')
    ax.legend()
    # ax.grid(True, linestyle=':', alpha=0.7)
    plt.tight_layout()
    out_path = os.path.join(BUILD_DIR, f"cdf_repair_time_{scenario_name}.png")
    plt.savefig(out_path, bbox_inches="tight")
    plt.close()
    print(f"[Saved Plot] {out_path}")


def save_cdf_obs_checks_nonanytime(scenario_name, planners_data):
    metrics = extract_non_anytime_per_seed_medians(planners_data)
    if not metrics:
        return
    planners = sorted(metrics.keys(), key=planner_sort_key)
    color_mapping = {'FMTX': '#1f78b4', 'DLITE': '#d62728'}

    fig, ax = plt.subplots(figsize=(6, 5))
    for planner in planners:
        vals = metrics[planner]['obs_repair']
        if not vals:
            continue
        sorted_vals = np.sort(vals)
        cdf = np.arange(1, len(sorted_vals)+1) / len(sorted_vals)
        ax.step(sorted_vals, cdf, where='post',
                label=planner, color=color_mapping.get(planner, None),
                linestyle='-', linewidth=2)
    ax.set_xlabel('Obstacle Checks Per Event (Per Seed Mean)')
    ax.set_ylabel('CDF')
    ax.legend()
    # ax.grid(True, linestyle=':', alpha=0.7)
    plt.tight_layout()
    out_path = os.path.join(BUILD_DIR, f"cdf_obs_checks_{scenario_name}.png")
    plt.savefig(out_path, bbox_inches="tight")
    plt.close()
    print(f"[Saved Plot] {out_path}")


def extract_per_seed_mean_path_costs(planners_data):
    """
    Return a dict: planner -> list of per‑seed mean path costs
    (mean over all set_state / goal_reached events within one seed).
    Only uses paired successful seeds.
    """
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
    return metrics


def save_boxplot_path_cost_nonanytime(scenario_name, planners_data):
    """Boxplot of per‑seed mean path cost, one box per planner."""
    metrics = extract_per_seed_mean_path_costs(planners_data)
    if not metrics:
        return
    planners = sorted(metrics.keys(), key=planner_sort_key)
    color_mapping = {'FMTX': '#1f78b4', 'DLITE': '#d62728'}

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


def main():
    scenarios = load_data(BUILD_DIR)

    if scenarios:
        for scenario_name, planners_data in scenarios.items():
            global CURRENT_SCENARIO
            CURRENT_SCENARIO = scenario_name

            analyze_group_statistics(scenario_name, planners_data)
            # save_latency_plot(scenario_name, planners_data)
            # save_comparative_plot(scenario_name, planners_data)
            # save_boxplot_repair_time_nonanytime(scenario_name, planners_data)
            # save_boxplot_obs_checks_nonanytime(scenario_name, planners_data)
            # save_cdf_repair_time_nonanytime(scenario_name, planners_data)
            # save_cdf_obs_checks_nonanytime(scenario_name, planners_data)
            # save_boxplot_path_cost_nonanytime(scenario_name, planners_data)


    else:
        print("No CSV files found.")

if __name__ == "__main__":
    main()



# #################################################################################
#     """
#     =====================================================================================
#     STATISTICAL ANALYSIS RATIONALE (FMTX vs PRM*-D*Lite)
#     =====================================================================================
    
#     1. THE "NO SIGNIFICANT DIFFERENCE" PARADOX IN PATH COST:
#     It is common to see a difference in raw averages (e.g., PRM* = 2.77 vs FMTX = 3.01) 
#     while the Wilcoxon signed-rank test yields a high p-value (e.g., p = 0.44). 
#     A p-value > 0.05 does NOT mean the averages are identical; it means the observed 
#     difference is not consistent enough to be distinguished from random environmental noise.
    
#     Because the obstacle trajectories and random node placements vary drastically per seed, 
#     the path cost fluctuates wildly. A p-value of 0.44 means there is a 44% chance that 
#     PRM*'s slight 0.24 advantage is purely due to the random seeds drawn, rather than a 
#     systematic algorithmic superiority.
    
#     2. WHY THE DIFFERENCE EXISTS DESPITE IDENTICAL SAMPLES:
#     Even with the same sample size (N=1000) and neighborhood radius, the algorithms build 
#     their graphs differently:
#     - PRM* exhaustively evaluates and rewires almost all valid connections in a neighborhood.
#     - FMT* uses a "lazy" dynamic programming approach (Open/Unvisited sets) to expand the 
#       tree faster, intentionally skipping some collision checks and exhaustive rewiring.
      
#     Because of this structural difference at finite sample sizes, FMT* may occasionally 
#     lock into a slightly suboptimal anchor parent compared to PRM*. (This suboptimality 
#     vanishes asymptotically as N -> infinity).
    
#     3. THE PAPER'S CORE ARGUMENT:
#     We use the Wilcoxon test to mathematically defend the algorithm against reviewers. 
#     By proving the cost difference yields p > 0.05, we establish "Statistical Non-Inferiority" 
#     in trajectory quality. We then contrast this with the Replanning Latency (p < 0.001), 
#     proving that FMTX achieves a massive (e.g., 5x) computational speedup without making a 
#     statistically meaningful sacrifice to path optimality.
#     =====================================================================================
#     """


# import warnings
# # Suppress the pandas bottleneck and matplotlib 3D warnings
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

# def load_and_pair_data(directory):
#     if not os.path.exists(directory): 
#         return {}
        
#     files = glob.glob(os.path.join(directory, "sim_*_metrics.csv"))
#     scenarios = {}

#     print(f"Found {len(files)} total metrics files. Filtering...")

#     for filepath in files:
#         filename = os.path.basename(filepath)
        
#         # Skip ANYTIME algorithms
#         if "ANY" in filename: 
#             continue
            
#         # 1. Extract Seed safely
#         seed_match = re.search(r"_seed_(\d+)", filename)
#         if not seed_match:
#             continue
#         seed = int(seed_match.group(1))
        
#         # 2. Extract Planner Name
#         if "FMTX" in filename:
#             planner = "FMTX"
#         elif "PRM" in filename or "DLITE" in filename or "DStar" in filename:
#             planner = "PRM*-D*Lite"
#         else:
#             planner = "Other"
            
#         # 3. Extract Scenario (State Space)
#         scenario = "Default"
#         if "_R2_" in filename: scenario = "R2"
#         elif "_R2T_" in filename: scenario = "R2T"
#         elif "_Dubins_" in filename: scenario = "Dubins"
#         elif "_Thruster_" in filename: scenario = "Thruster"
#         else:
#             parts = filename.split('_')
#             if len(parts) >= 3:
#                 scenario = parts[2]

#         try:
#             df = pd.read_csv(filepath)
#             if df.empty: continue
            
#             # Clean headers (removes accidental spaces from C++ output)
#             df.columns = [c.strip().replace('_', '') for c in df.columns]
            
#             # Standardize names
#             col_map = {
#                 'elapseds': 'elapsed_s', 'totallatencyms': 'total_latency_ms',
#                 'updatems': 'update_ms', 'planms': 'plan_ms', 
#                 'timetogoal': 'time_to_goal', 'pathcost': 'path_cost', 
#                 'collisioncount': 'collision_count'
#             }
#             df.rename(columns=col_map, inplace=True)
#             df.replace([np.inf, -np.inf], np.nan, inplace=True)
            
#             # Success logic (0 collisions)
#             succ = False
#             if 'collision_count' in df.columns and df['collision_count'].max() == 0:
#                 succ = True
                
#             # Repair latency (elapsed > 0 and update_ms > 0)
#             if 'elapsed_s' in df.columns and 'update_ms' in df.columns and 'total_latency_ms' in df.columns:
#                 repair_df = df[(df['elapsed_s'] > 0.0) & (df['update_ms'] > 0.001)]
#                 avg_repair = repair_df['total_latency_ms'].mean() if not repair_df.empty else 0.0
#             else:
#                 avg_repair = 0.0
                
#             # Final stats
#             arrival_time = df['time_to_goal'].dropna().iloc[-1] if 'time_to_goal' in df.columns else 0.0
#             final_cost = df['path_cost'].dropna().iloc[-1] if 'path_cost' in df.columns else 0.0
            
#             metrics = {
#                 'success': succ,
#                 'avg_repair_ms': avg_repair,
#                 'arrival_time': arrival_time,
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
#         print(f"\n{'='*80}\n SCENARIO: {scenario}\n{'='*80}")
        
#         # Identity planners
#         planners = set()
#         for s in seeds_dict.values(): planners.update(s.keys())
#         planners = list(planners)
        
#         if len(planners) < 2:
#             print(f"Only found {planners} in {scenario}. Need 2 planners to compare.")
#             continue
            
#         # Ensure FMTX is p1 if available for consistent coloring
#         if "FMTX" in planners:
#             p1 = "FMTX"
#             p2 = [p for p in planners if p != "FMTX"][0]
#         else:
#             p1, p2 = planners[0], planners[1]
        
#         p1_repair, p2_repair = [], []
#         p1_cost, p2_cost = [], []
#         p1_arrival, p2_arrival = [], []
#         paired_seeds = []
        
#         # Pair the seeds for statistical testing
#         for seed, runs in seeds_dict.items():
#             if p1 in runs and p2 in runs:
#                 # ONLY use seeds where BOTH planners successfully reached the goal without crashing
#                 if runs[p1]['success'] and runs[p2]['success']:
#                     paired_seeds.append(seed)
#                     p1_repair.append(runs[p1]['avg_repair_ms'])
#                     p2_repair.append(runs[p2]['avg_repair_ms'])
#                     p1_cost.append(runs[p1]['final_cost'])
#                     p2_cost.append(runs[p2]['final_cost'])
#                     p1_arrival.append(runs[p1]['arrival_time'])
#                     p2_arrival.append(runs[p2]['arrival_time'])
                    
#         num_pairs = len(paired_seeds)
#         print(f"Valid Paired Successful Seeds: {num_pairs}")
#         if num_pairs < 5:
#             print("Too few paired successes for statistical analysis.")
#             continue
            
#         # 1. Wilcoxon Signed-Rank Tests (Statistical Significance)
#         stat_rep, pval_rep = wilcoxon(p1_repair, p2_repair)
#         stat_cost, pval_cost = wilcoxon(p1_cost, p2_cost)
        
#         print(f"\n--- WILCOXON PAIRED STATS ({p1} vs {p2}) ---")
#         print(f"Average Replanning Latency (ms): {p1}: {np.mean(p1_repair):.2f} | {p2}: {np.mean(p2_repair):.2f} | p-value: {pval_rep:.4f}")
#         print(f"Final Path Cost:                 {p1}: {np.mean(p1_cost):.2f}   | {p2}: {np.mean(p2_cost):.2f}   | p-value: {pval_cost:.4f}")
        
#         sig_str = "SIGNIFICANT" if pval_rep < 0.05 else "NOT Significant"
#         print(f"-> Conclusion: The difference in Replanning Latency is {sig_str}.")

#         # 2. BOX PLOT: Replanning Latency
#         plt.figure(figsize=(6, 5))
#         plt.boxplot([p1_repair, p2_repair], labels=[p1, p2], patch_artist=True, 
#                     boxprops=dict(facecolor='lightblue', color='black'),
#                     medianprops=dict(color='red', linewidth=2))
#         plt.ylabel('Average Replanning Latency (ms)')
#         plt.title(f'Paired Replanning Latency ({num_pairs} paired seeds)')
#         plt.grid(axis='y', linestyle=':', alpha=0.7)
        
#         out_box = os.path.join(BUILD_DIR, f"boxplot_latency_{scenario}.png")
#         plt.savefig(out_box, bbox_inches='tight')
#         plt.close()

#         # 3. SCATTER PLOT: Pareto Front (Arrival Time vs Path Cost)
#         plt.figure(figsize=(8, 6))
#         plt.scatter(p1_arrival, p1_cost, label=p1, alpha=0.7, edgecolors='black', s=80, marker='o', c='#1f77b4')
#         plt.scatter(p2_arrival, p2_cost, label=p2, alpha=0.7, edgecolors='black', s=80, marker='s', c='#ff7f0e')
        
#         # Plot Means as large stars
#         plt.scatter(np.mean(p1_arrival), np.mean(p1_cost), color='blue', marker='*', s=400, edgecolors='black', label=f'{p1} Mean')
#         plt.scatter(np.mean(p2_arrival), np.mean(p2_cost), color='red', marker='*', s=400, edgecolors='black', label=f'{p2} Mean')

#         plt.xlabel('Arrival Time (s) [T_robot]')
#         plt.ylabel('Kinematic Path Cost')
#         plt.title(f'Time-Energy Pareto Distribution ({num_pairs} paired seeds)')
#         plt.legend()
#         plt.grid(True, linestyle=':', alpha=0.7)
        
#         out_scatter = os.path.join(BUILD_DIR, f"scatter_pareto_{scenario}.png")
#         plt.savefig(out_scatter, bbox_inches='tight')
#         plt.close()
        
#         print(f"[Saved] {out_box}")
#         print(f"[Saved] {out_scatter}")

# if __name__ == "__main__":
#     scenarios_data = load_and_pair_data(BUILD_DIR)
#     if scenarios_data:
#         analyze_and_plot(scenarios_data)
#     else:
#         print("No CSV metrics files found in the current directory.")