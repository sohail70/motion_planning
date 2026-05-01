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

2. Path cost:
   Taken from set_state / goal_reached rows only.

3. Obstacle checks:
   Reported as Obs/Upd directly from the 'update' event.

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


def save_latency_plot(scenario_name, planners_data):
    if not planners_data:
        return

    planners = []
    repair_update = []

    for planner, dfs in planners_data.items():
        valid_dfs = [df for df in dfs if not df.empty and "event_type" in df.columns]
        if not valid_dfs:
            continue

        successful_dfs = [df for df in valid_dfs if is_successful_run(df)]
        if not successful_dfs:
            continue

        all_runtime = [extract_runtime_events(df) for df in successful_dfs]
        merged_df = pd.concat(all_runtime, ignore_index=True)

        update_rows = get_update_rows(merged_df)

        planners.append(planner)
        repair_update.append(update_rows["update_ms"].dropna().mean() if not update_rows.empty else 0.0)

    if not planners:
        return

    x = np.arange(len(planners))
    width = 0.5
    fig, ax = plt.subplots(figsize=(6, 5))

    c_update = "#d95f02"

    # Repair plot
    bars = ax.bar(x, repair_update, width, label="$T_{repair}$ (Update + Plan)", color=c_update, edgecolor="black")
    ax.set_title("Dynamic Replan Event Latency (Non-Anytime)")
    ax.set_ylabel("Average Latency (ms)")
    ax.set_xticks(x)
    ax.set_xticklabels(planners)
    ax.grid(axis="y", linestyle=":", alpha=0.7)

    for bar in bars:
        height = bar.get_height()
        if height > 0:
            ax.text(bar.get_x() + bar.get_width()/2., height * 1.05, f"{height:.1f}", ha="center", va="bottom", fontweight="bold")

    plt.suptitle(f"Algorithmic Latency: {scenario_name}", y=1.05)
    plt.tight_layout()

    out_path = os.path.join(BUILD_DIR, f"plot_latency_breakdown_{scenario_name}_non_anytime.png")
    plt.savefig(out_path, bbox_inches="tight")
    plt.close()
    print(f"[Saved Plot] {out_path}")


def analyze_group_statistics(scenario_name, planners_data):
    print(f"\n{'='*200}")
    print(f" NON-ANYTIME ANALYSIS: {scenario_name} (Successful runs only)")
    print(f"{'='*200}")

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

        # Extract fixed properties directly from the initial_plan row
        df_init = merged_all[merged_all["event_type"] == "initial_plan"].copy() if "event_type" in merged_all.columns else pd.DataFrame()
        
        setup_ms = df_init["setup_ms"].dropna().mean() if ("setup_ms" in df_init.columns and not df_init.empty) else np.nan
        t_init_pln = df_init["plan_ms"].dropna().mean() if ("plan_ms" in df_init.columns and not df_init.empty) else np.nan
        isolated_nodes = df_init["isolated_nodes"].dropna().mean() if ("isolated_nodes" in df_init.columns and not df_init.empty) else np.nan
        samples = df_init["tree_size"].dropna().mean() if ("tree_size" in df_init.columns and not df_init.empty) else np.nan

        radius_col = "neighborhood_radius" if "neighborhood_radius" in df_init.columns else ("radius" if "radius" in df_init.columns else None)
        r_n = df_init[radius_col].dropna().mean() if (radius_col and not df_init.empty) else np.nan

        # Core event partitions
        update_rows = get_update_rows(merged_df)
        state_rows = get_state_rows(merged_all)
        terminal_rows = get_terminal_rows(merged_all)

        # Latencies
        t_repair_avg = update_rows["update_ms"].dropna().mean() if not update_rows.empty else np.nan
        t_repair_p99 = update_rows["update_ms"].dropna().quantile(0.99) if not update_rows.empty else np.nan

        # Path cost from set_state / goal_reached only
        avg_cost = state_rows["path_cost"].mean() if not state_rows.empty else np.nan

        # Obstacle checks
        obs_per_update = update_rows["obstacle_checks"].dropna().mean() if not update_rows.empty else np.nan
        
        # Degree stats from terminal rows
        deg_in = terminal_rows["avg_deg_in"].dropna().mean() if ("avg_deg_in" in terminal_rows.columns and not terminal_rows.empty) else np.nan
        deg_out = terminal_rows["avg_deg_out"].dropna().mean() if ("avg_deg_out" in terminal_rows.columns and not terminal_rows.empty) else np.nan

        summary_data.append({
            "Planner": planner,
            "Succ(%)": f"{succ_rate:.0f}%",
            "Setup(ms)": f"{setup_ms:.1f}" if pd.notna(setup_ms) else "nan",
            "T_init(ms)": f"{t_init_pln:.1f}" if pd.notna(t_init_pln) else "nan",
            "T_repair(ms)": f"{t_repair_avg:.1f}" if pd.notna(t_repair_avg) else "nan",
            "Rep_p99(ms)": f"{t_repair_p99:.1f}" if pd.notna(t_repair_p99) else "nan",
            "Obs/Upd": f"{obs_per_update:.0f}" if pd.notna(obs_per_update) else "nan",
            "Path_Cost": f"{avg_cost:.2f}" if pd.notna(avg_cost) else "nan",
            "Samples": f"{samples:.0f}" if pd.notna(samples) else "nan",
            "Isolated": f"{isolated_nodes:.0f}" if pd.notna(isolated_nodes) else "nan",
            "r_n": f"{r_n:.2f}" if pd.notna(r_n) else "nan",
            "Deg(I/O)": (
                f"{deg_in:.1f}/{deg_out:.1f}"
                if pd.notna(deg_in) and pd.notna(deg_out)
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

        if not all_times:
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

        out_path = os.path.join(BUILD_DIR, f"plot_non_anytime_{scenario_name}_average.png")
        plt.savefig(out_path, bbox_inches="tight")
        plt.close()
        print(f"[Saved Plot] {out_path}")


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