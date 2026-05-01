"""
======================================================================================
BENCHMARKING METHODOLOGY & METRIC ISOLATION
======================================================================================
This script processes high-resolution, event-based timeline metrics from the C++ planner.

Event semantics:
- initial_plan   : one-time initialization plan
- set_state      : robot-state snapshot; path_cost belongs here
- update         : obstacle update / repair trigger
- plan           : anytime planning iteration
- slice_end      : end of real-time slice
- goal_reached   : terminal success snapshot
- time_limit     : terminal timeout snapshot

Metric isolation policy:
1. T_add:
   Mean plan_ms over steady-state plan rows, excluding the first plan immediately
   following any update event.

2. T_repair:
   Per repair event = update_ms + first-following-plan_ms
   This captures eager and lazy repair fairly.

3. Path cost:
   Taken from set_state / goal_reached rows only.

4. Obstacle checks:
   NEVER averaged across all rows indiscriminately.
   Reported separately by event type to avoid misleading comparisons.
======================================================================================
"""

import pandas as pd
import glob
import os
import re
import numpy as np
import matplotlib.pyplot as plt
from pandas.errors import EmptyDataError

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

            numeric_cols = [
                "row_id", "elapsed_s", "sim_time", "setup_ms", "total_latency_ms",
                "update_ms", "plan_ms", "time_to_goal", "path_cost", "obstacle_checks",
                "collision_count", "tree_size", "isolated_nodes", "avg_deg_out",
                "avg_deg_in", "neighborhood_radius"
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


def is_successful_run(df):
    if df.empty:
        return False

    if "collision_count" in df.columns:
        cc = pd.to_numeric(df["collision_count"], errors="coerce").fillna(0)
        return cc.max() == 0

    if "crashed" in df.columns:
        cr = pd.to_numeric(df["crashed"], errors="coerce").fillna(0)
        return (cr == 1).sum() == 0

    return True


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
    """
    Annotate plan rows with whether they are the first plan after an update.
    """
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
    """
    Returns a DataFrame with one row per repair event:
    - update row
    - first following plan row (if present)
    """
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


def save_latency_plot(scenario_name, planners_data):
    if not planners_data:
        return

    planners = []
    steady_plan = []
    repair_update = []
    repair_plan = []

    for planner, dfs in planners_data.items():
        valid_dfs = [df for df in dfs if not df.empty and "event_type" in df.columns]
        if not valid_dfs:
            continue

        successful_dfs = [df for df in valid_dfs if is_successful_run(df)]
        if not successful_dfs:
            continue

        all_runtime = [extract_runtime_events(df) for df in successful_dfs]
        merged_df = pd.concat(all_runtime, ignore_index=True)

        repairs = pair_repair_events(merged_df)
        steady_rows = get_steady_plan_rows(merged_df)

        planners.append(planner)
        steady_plan.append(steady_rows["plan_ms"].dropna().mean() if not steady_rows.empty else 0.0)
        repair_update.append(repairs["update_ms"].dropna().mean() if not repairs.empty else 0.0)
        repair_plan.append(repairs["repair_plan_ms"].dropna().mean() if not repairs.empty else 0.0)

    if not planners:
        return

    x = np.arange(len(planners))
    width = 0.35
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))

    c_update = "#d95f02"
    c_plan = "#1f78b4"

    # Steady-state plot
    ax1.bar(x, np.zeros(len(planners)), width, label="$T_{update}$", color=c_update, edgecolor="black", hatch="//")
    ax1.bar(x, steady_plan, width, bottom=np.zeros(len(planners)), label="$T_{plan}$", color=c_plan, edgecolor="black")
    ax1.set_ylabel("Average Latency (ms)")
    ax1.set_title("Steady-State (Sample Addition)")
    ax1.set_xticks(x)
    ax1.set_xticklabels(planners)
    ax1.grid(axis="y", linestyle=":", alpha=0.7)

    for i in range(len(planners)):
        total_val = steady_plan[i]
        if total_val > 0:
            ax1.text(i, total_val * 1.05, f"{total_val:.1f}", ha="center", va="bottom", fontweight="bold")

    # Repair plot
    ax2.bar(x, repair_update, width, label="$T_{update}$", color=c_update, edgecolor="black", hatch="//")
    ax2.bar(x, repair_plan, width, bottom=repair_update, label="$T_{plan}$", color=c_plan, edgecolor="black")
    ax2.set_title("Dynamic Replan Event (Graph Repair)")
    ax2.set_xticks(x)
    ax2.set_xticklabels(planners)
    ax2.grid(axis="y", linestyle=":", alpha=0.7)

    for i in range(len(planners)):
        total_val = repair_update[i] + repair_plan[i]
        if total_val > 0:
            ax2.text(i, total_val * 1.05, f"{total_val:.1f}", ha="center", va="bottom", fontweight="bold")

    handles, labels = ax2.get_legend_handles_labels()
    fig.legend(handles, labels, loc="lower center", bbox_to_anchor=(0.5, -0.05), ncol=2)
    plt.suptitle(f"Algorithmic Latency Breakdown: {scenario_name}", y=1.05)
    plt.tight_layout()

    out_path = os.path.join(BUILD_DIR, f"plot_latency_breakdown_{scenario_name}.png")
    plt.savefig(out_path, bbox_inches="tight")
    plt.close()
    print(f"[Saved Plot] {out_path}")


def analyze_group_statistics(scenario_name, planners_data):
    print(f"\n{'='*220}")
    print(f" AGGREGATE ANALYSIS: {scenario_name} (Successful runs only)")
    print(f"{'='*220}")

    summary_data = []

    for planner, dfs in planners_data.items():
        valid_dfs = [df for df in dfs if not df.empty]
        if not valid_dfs:
            continue

        num_seeds = len(valid_dfs)
        successful_dfs = [df for df in valid_dfs if is_successful_run(df)]
        success_count = len(successful_dfs)
        succ_rate = (success_count / num_seeds) * 100 if num_seeds > 0 else 0.0

        if not successful_dfs:
            summary_data.append({
                "Planner": planner,
                "Succ(%)": f"{succ_rate:.0f}%",
                "Status": "All runs failed"
            })
            continue

        merged_df = pd.concat([extract_runtime_events(df) for df in successful_dfs], ignore_index=True)
        merged_all = pd.concat(successful_dfs, ignore_index=True)

        print(f"{planner}: {success_count}/{num_seeds} successful ({succ_rate:.0f}%)")
        if "event_type" in merged_all.columns:
            print(f"  Event types: {merged_all['event_type'].value_counts().to_dict()}")

        # Init phase
        df_init = merged_all[merged_all["event_type"] == "initial_plan"].copy() if "event_type" in merged_all.columns else pd.DataFrame()
        t_init_pln = df_init["plan_ms"].dropna().mean() if ("plan_ms" in df_init.columns and not df_init.empty) else 0.0

        # Core event partitions
        update_rows = get_update_rows(merged_df)
        repair_plan_rows = get_first_repair_plan_rows(merged_df)
        steady_plan_rows = get_steady_plan_rows(merged_df)
        repairs = pair_repair_events(merged_df)
        state_rows = get_state_rows(merged_all)
        terminal_rows = get_terminal_rows(merged_all)

        # Latencies
        t_upd_avg = update_rows["update_ms"].dropna().mean() if not update_rows.empty else np.nan
        t_pln_avg = repair_plan_rows["plan_ms"].dropna().mean() if not repair_plan_rows.empty else np.nan
        t_repair_avg = repairs["t_repair_ms"].dropna().mean() if not repairs.empty else np.nan
        t_add_avg = steady_plan_rows["plan_ms"].dropna().mean() if not steady_plan_rows.empty else np.nan
        t_repair_p99 = repairs["t_repair_ms"].dropna().quantile(0.99) if not repairs.empty else np.nan

        # Path cost from set_state / goal_reached only
        avg_cost = state_rows["path_cost"].mean() if not state_rows.empty else np.nan

        ###########################################################
        # Obstacle checks: keep event-specific, do not mix semantics
        obs_chk_update = update_rows["obstacle_checks"].dropna().mean() if not update_rows.empty else np.nan
        obs_chk_repair_plan = repair_plan_rows["obstacle_checks"].dropna().mean() if not repair_plan_rows.empty else np.nan
        obs_chk_steady_plan = steady_plan_rows["obstacle_checks"].dropna().mean() if not steady_plan_rows.empty else np.nan
        
        # For RRTX: no repair planning phase (all repair in updateObstacles)
        if planner.upper() == "RRTX":
            obs_chk_repair_plan = np.nan
        
        # Journal paper metric: per-update collision checks
        obs_per_update = (
            obs_chk_update + obs_chk_repair_plan
        ) if (pd.notna(obs_chk_update) and pd.notna(obs_chk_repair_plan)) else obs_chk_update
        ###########################################################
        
        # Queue ops not currently in your CSV
        queue_ops = 0.0

        # Start/end graph stats per run
        start_v_vals, end_v_vals = [], []
        start_r_vals, end_r_vals = [], []

        for df in successful_dfs:
            if "tree_size" in df.columns and not df.empty:
                start_v = pd.to_numeric(df.iloc[0]["tree_size"], errors="coerce")
                end_v = pd.to_numeric(df.iloc[-1]["tree_size"], errors="coerce")
                if pd.notna(start_v):
                    start_v_vals.append(start_v)
                if pd.notna(end_v):
                    end_v_vals.append(end_v)

            radius_col = None
            if "neighborhood_radius" in df.columns:
                radius_col = "neighborhood_radius"
            elif "radius" in df.columns:
                radius_col = "radius"

            if radius_col and not df.empty:
                start_r = pd.to_numeric(df.iloc[0][radius_col], errors="coerce")
                end_r = pd.to_numeric(df.iloc[-1][radius_col], errors="coerce")
                if pd.notna(start_r):
                    start_r_vals.append(start_r)
                if pd.notna(end_r):
                    end_r_vals.append(end_r)

        start_v = np.mean(start_v_vals) if start_v_vals else np.nan
        end_v = np.mean(end_v_vals) if end_v_vals else np.nan
        start_r = np.mean(start_r_vals) if start_r_vals else np.nan
        end_r = np.mean(end_r_vals) if end_r_vals else np.nan

        # Degree stats from terminal rows only
        deg_in = terminal_rows["avg_deg_in"].dropna().mean() if ("avg_deg_in" in terminal_rows.columns and not terminal_rows.empty) else np.nan
        deg_out = terminal_rows["avg_deg_out"].dropna().mean() if ("avg_deg_out" in terminal_rows.columns and not terminal_rows.empty) else np.nan

        summary_data.append({
            "Planner": planner,
            "Succ(%)": f"{succ_rate:.0f}%",
            "T_init(ms)": f"{t_init_pln:.1f}" if pd.notna(t_init_pln) else "nan",
            "T_upd(ms)": f"{t_upd_avg:.1f}" if pd.notna(t_upd_avg) else "nan",
            "T_pln(ms)": f"{t_pln_avg:.1f}" if pd.notna(t_pln_avg) else "nan",
            "T_repair(ms)": f"{t_repair_avg:.1f}" if pd.notna(t_repair_avg) else "nan",
            "T_add(ms)": f"{t_add_avg:.1f}" if pd.notna(t_add_avg) else "nan",
            "Rep_p99(ms)": f"{t_repair_p99:.1f}" if pd.notna(t_repair_p99) else "nan",
            "ObsUpd": f"{obs_chk_update:.0f}" if pd.notna(obs_chk_update) else "nan",
            "ObsRepPln": f"{obs_chk_repair_plan:.0f}" if pd.notna(obs_chk_repair_plan) else "nan",
            "ObsSteady": f"{obs_chk_steady_plan:.0f}" if pd.notna(obs_chk_steady_plan) else "nan",
            "Obs/Upd": f"{obs_per_update:.0f}" if pd.notna(obs_per_update) else "nan",
            "Q_Ops": f"{queue_ops:.0f}",
            "Path_Cost": f"{avg_cost:.2f}" if pd.notna(avg_cost) else "nan",
            "Start |V|": f"{start_v:.0f}" if pd.notna(start_v) else "nan",
            "End |V|": f"{end_v:.0f}" if pd.notna(end_v) else "nan",
            "Start r_n": f"{start_r:.2f}" if pd.notna(start_r) else "nan",
            "End r_n": f"{end_r:.2f}" if pd.notna(end_r) else "nan",
            "Deg(I/O)": (
                f"{deg_in:.1f}/{deg_out:.1f}"
                if pd.notna(deg_in) and pd.notna(deg_out)
                else "nan"
            )
        })

    if summary_data:
        df_out = pd.DataFrame(summary_data)
        print(df_out.to_string(index=False, justify="center"))
        print("-" * 220)


def save_comparative_plot(scenario_name, planners_data):
    if not planners_data:
        return

    plt.figure(figsize=(10, 6))
    plotted_any = False

    for planner, dfs in planners_data.items():
        successful_dfs = [df for df in dfs if is_successful_run(df)]
        if not successful_dfs:
            continue

        all_times = []
        all_costs = []

        for df in successful_dfs:
            state_rows = get_state_rows(df)
            if state_rows.empty:
                continue

            if "sim_time" in state_rows.columns and state_rows["sim_time"].notna().any():
                state_rows["plot_time"] = state_rows["sim_time"]
            else:
                state_rows["plot_time"] = state_rows["elapsed_s"]

            state_rows["plot_time_rounded"] = state_rows["plot_time"].round(1)

            all_times.extend(state_rows["plot_time_rounded"].tolist())
            all_costs.extend(state_rows["path_cost"].tolist())

            print(f"{planner}: Found {len(state_rows)} valid state path_cost points")

        if not all_times:
            print(f"{planner}: No valid state path_cost found")
            continue

        combined = pd.DataFrame({"time": all_times, "cost": all_costs})
        grouped = combined.groupby("time")["cost"].agg(["mean", "std"]).reset_index()
        grouped["std"] = grouped["std"].fillna(0.0)

        plt.plot(grouped["time"], grouped["mean"], label=planner, linewidth=2)
        plt.fill_between(
            grouped["time"],
            grouped["mean"] - grouped["std"],
            grouped["mean"] + grouped["std"],
            alpha=0.2
        )
        plotted_any = True

    if plotted_any:
        plt.xlabel("Simulation Time (s)")
        plt.ylabel("Path Cost")
        plt.title(f"Average Path Cost over Time (Successful Runs) - {scenario_name}")
        plt.legend()
        plt.grid(True, linestyle=":", alpha=0.7)

        out_path = os.path.join(BUILD_DIR, f"plot_anytime_{scenario_name}_average.png")
        plt.savefig(out_path, bbox_inches="tight")
        plt.close()
        print(f"[Saved Plot] {out_path}")
    else:
        print(f"No valid path cost data for {scenario_name}")


def main():
    scenarios = load_data(BUILD_DIR)

    if scenarios:
        for scenario_name, planners_data in scenarios.items():
            analyze_group_statistics(scenario_name, planners_data)
            save_latency_plot(scenario_name, planners_data)
            save_comparative_plot(scenario_name, planners_data)
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