"""
======================================================================================
BENCHMARKING METHODOLOGY & METRIC ISOLATION
======================================================================================
This script processes high-resolution, per-cycle timeline metrics from the C++ planner.
It is explicitly designed to handle the architectural differences between eager (ANYRRTX) 
and lazy (ANYFMTX) dynamic replanning strategies without metric skew.

Architectural Context:
- ANYRRTX: Performs full topological graph repair immediately during `updateObstacles()`.
- ANYFMTX: Performs O(1) edge severing in `updateObstacles()`, but delegates the 
  actual structural graph repair to the subsequent `plan()` wavefront expansion.

Data Slicing Logic to Prevent Metric Skew:
If we averaged all `plan_ms` indiscriminately, ANYFMTX's steady-state sampling average 
would be artificially inflated by the frames where it was doing heavy graph repair. 
To prevent this, the script rigorously isolates the data into two conceptual buckets 
based on whether a dynamic obstacle event occurred (`update_ms` threshold):

1. STEADY-STATE CRUISING (update_ms <= 0.001):
   No obstacles changed trajectory. The `plan()` function is ONLY performing standard 
   sample addition and nominal wavefront expansion. 
   -> Extracted Metric: T_add (Pure sampling/expansion latency).

2. DYNAMIC REPAIR EVENT (update_ms > 0.001):
   An obstacle trajectory changed. Because ANYFMTX blends its repair into the planning 
   phase, the script aggregates `update_ms` + `plan_ms` for these specific frames.
   -> Extracted Metric: T_repair (Total aggregate repair latency).
   
This strict separation ensures a 1-to-1, apples-to-apples comparison: it aggregates 
the total temporal cost of recovering from a dynamic event (T_repair) while keeping 
the steady-state averages (T_add) mathematically pure.
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
    'font.family': 'serif',
    'font.serif': ['Times New Roman', 'DejaVu Serif', 'serif'],
    'axes.labelsize': 12,
    'axes.titlesize': 14,
    'legend.fontsize': 11,
    'xtick.labelsize': 11,
    'ytick.labelsize': 11,
    'figure.dpi': 300
})

BUILD_DIR = "." 
FILENAME_PATTERN = re.compile(r"sim_([A-Za-z0-9]+)_([A-Za-z0-9_]+)_seed_(\d+)_(\d{8}_\d{6})_metrics\.csv")

def load_data(directory):
    if not os.path.exists(directory): return {}
    files = glob.glob(os.path.join(directory, "sim_*_metrics.csv"))
    scenarios = {}

    for filepath in files:
        filename = os.path.basename(filepath)
        match = FILENAME_PATTERN.match(filename)
        if match:
            planner_raw = match.group(1)
            scenario = match.group(2)
            planner_clean = (planner_raw.replace("Kinodynamic", "").replace("PRMStarDStarLite", "DLITE").replace("PRMStar", "").replace("ANY", ""))
            try:
                df = pd.read_csv(filepath)
                if df.empty: continue
                df.replace([np.inf, -np.inf], np.nan, inplace=True)
                
                if scenario not in scenarios: scenarios[scenario] = {}
                if planner_clean not in scenarios[scenario]: scenarios[scenario][planner_clean] = []
                scenarios[scenario][planner_clean].append(df)
            except EmptyDataError: pass
            except Exception as e: print(f"Warning: Could not load {filename}: {e}")
    return scenarios

def save_latency_plot(scenario_name, planners_data):
    if not planners_data: return
    planners, steady_update, steady_plan, repair_update, repair_plan = [], [], [], [], []

    for planner, dfs in planners_data.items():
        valid_dfs = [df for df in dfs if not df.empty and 'update_ms' in df.columns]
        if not valid_dfs: continue
        
        merged_df = pd.concat(valid_dfs, ignore_index=True)
        planners.append(planner)

        # Runtime filter: elapsed_s > 0
        is_runtime = merged_df['elapsed_s'] > 0.0

        # Steady state cruising: update_ms near 0
        df_steady = merged_df[is_runtime & (merged_df['update_ms'] <= 0.001)]
        steady_update.append(df_steady['update_ms'].mean() if not df_steady.empty else 0)
        steady_plan.append(df_steady['plan_ms'].mean() if not df_steady.empty else 0)

        # Repair event: update_ms > 0.001
        df_repair = merged_df[is_runtime & (merged_df['update_ms'] > 0.001)]
        repair_update.append(df_repair['update_ms'].mean() if not df_repair.empty else 0)
        repair_plan.append(df_repair['plan_ms'].mean() if not df_repair.empty else 0)

    if not planners: return
    x = np.arange(len(planners))
    width = 0.35
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))

    # Colors optimized for academic papers
    c_update, c_plan = '#d95f02', '#1f78b4'

    # Plot 1: Steady State
    ax1.bar(x, steady_update, width, label='$T_{update}$', color=c_update, edgecolor='black', hatch='//')
    ax1.bar(x, steady_plan, width, bottom=steady_update, label='$T_{plan}$', color=c_plan, edgecolor='black')
    ax1.set_ylabel('Average Latency (ms)')
    ax1.set_title('Steady-State (Sample Addition)')
    ax1.set_xticks(x)
    ax1.set_xticklabels(planners)
    ax1.grid(axis='y', linestyle=':', alpha=0.7)
    
    for i in range(len(planners)):
        total_val = steady_update[i] + steady_plan[i]
        if total_val > 0: ax1.text(i, total_val * 1.05, f"{total_val:.1f}", ha='center', va='bottom', fontweight='bold')

    # Plot 2: Repair Event
    ax2.bar(x, repair_update, width, label='$T_{update}$', color=c_update, edgecolor='black', hatch='//')
    ax2.bar(x, repair_plan, width, bottom=repair_update, label='$T_{plan}$', color=c_plan, edgecolor='black')
    ax2.set_title('Dynamic Replan Event (Graph Repair)')
    ax2.set_xticks(x)
    ax2.set_xticklabels(planners)
    ax2.grid(axis='y', linestyle=':', alpha=0.7)
    
    for i in range(len(planners)):
        total_val = repair_update[i] + repair_plan[i]
        if total_val > 0: ax2.text(i, total_val * 1.05, f"{total_val:.1f}", ha='center', va='bottom', fontweight='bold')

    handles, labels = ax2.get_legend_handles_labels()
    fig.legend(handles, labels, loc='lower center', bbox_to_anchor=(0.5, -0.05), ncol=2)
    plt.suptitle(f"Algorithmic Latency Breakdown: {scenario_name}", y=1.05)
    plt.tight_layout()
    
    out_path = os.path.join(BUILD_DIR, f"plot_latency_breakdown_{scenario_name}.png")
    plt.savefig(out_path, bbox_inches='tight')
    plt.close()
    print(f"[Saved Plot] {out_path}")

def analyze_group_statistics(scenario_name, planners_data):
    print(f"\n{'='*150}")
    print(f" AGGREGATE ANALYSIS: {scenario_name} (Successful runs only)")
    print(f"{'='*150}")
    
    summary_data = []

    for planner, dfs in planners_data.items():
        valid_dfs = [df for df in dfs if not df.empty]
        if not valid_dfs: continue
        
        num_seeds = len(valid_dfs)
        successful_dfs = []
        success_count = 0
        
        for df in valid_dfs:
            if 'crashed' in df.columns and not (df['crashed'] == 1).any():
                success_count += 1; successful_dfs.append(df)
            elif 'collision_count' in df.columns and df['collision_count'].max() == 0:
                success_count += 1; successful_dfs.append(df)
            elif 'crashed' not in df.columns and 'collision_count' not in df.columns:
                success_count += 1; successful_dfs.append(df)
                
        succ_rate = (success_count / num_seeds) * 100
        if not successful_dfs:
            summary_data.append({"Planner": planner, "Succ(%)": f"{succ_rate:.0f}%", "Status": "All runs failed"})
            continue
            
        merged_df = pd.concat(successful_dfs, ignore_index=True)
        if 'update_ms' not in merged_df.columns: continue

        is_runtime = merged_df['elapsed_s'] > 0.0

        # Init phase
        df_init = merged_df[~is_runtime]
        t_init_set = df_init['setup_ms'].mean() if 'setup_ms' in df_init.columns else 0.0
        t_init_pln = df_init['plan_ms'].mean() if not df_init.empty else 0.0

        # Steady Phase (Just adding samples, no obstacles turned around)
        df_steady = merged_df[is_runtime & (merged_df['update_ms'] <= 0.001)]
        t_add_avg = (df_steady['update_ms'] + df_steady['plan_ms']).mean() if not df_steady.empty else 0.0

        # Repair phase (Obstacles turned around, massive repair needed)
        df_event = merged_df[is_runtime & (merged_df['update_ms'] > 0.001)]
        if not df_event.empty:
            t_upd_avg = df_event['update_ms'].mean()
            t_pln_avg = df_event['plan_ms'].mean()
            t_repair_avg = t_upd_avg + t_pln_avg  # Total aggregate repair latency
            t_repair_p99 = df_event['total_latency_ms'].quantile(0.99)
            obs_chk   = df_event['obstacle_checks'].mean()
            queue_ops = df_event['queue_operations'].mean() if 'queue_operations' in df_event.columns else 0.0
            avg_cost  = df_event['path_cost'].dropna().mean()
        else:
            t_upd_avg = t_pln_avg = t_repair_avg = t_repair_p99 = obs_chk = queue_ops = avg_cost = 0.0

        tree_sz = merged_df['tree_size'].max()
        deg_in  = merged_df['avg_deg_in'].mean() if 'avg_deg_in' in merged_df.columns else 0.0
        deg_out = merged_df['avg_deg_out'].mean() if 'avg_deg_out' in merged_df.columns else 0.0

        summary_data.append({
            "Planner": planner,
            "Succ(%)": f"{succ_rate:.0f}%",
            "T_init(ms)": f"{t_init_pln:.1f}",
            "T_upd(ms)": f"{t_upd_avg:.1f}",
            "T_pln(ms)": f"{t_pln_avg:.1f}",
            "T_repair(ms)": f"{t_repair_avg:.1f}",
            "T_add(ms)": f"{t_add_avg:.1f}",
            "Rep_p99(ms)": f"{t_repair_p99:.1f}",
            "Obs_Chk": f"{obs_chk:.0f}",
            "Q_Ops": f"{queue_ops:.0f}",
            "Path_Cost": f"{avg_cost:.2f}",
            "|V|": f"{tree_sz:.0f}",
            "Deg(I/O)": f"{deg_in:.1f}/{deg_out:.1f}"
        })

    if summary_data:
        df_out = pd.DataFrame(summary_data)
        print(df_out.to_string(index=False, justify='center'))
        print("-" * 150)

def save_comparative_plot(scenario_name, planners_data):
    if not planners_data: return
    plt.figure(figsize=(10, 6))

    for planner, dfs in planners_data.items():
        successful_dfs = []
        for df in dfs:
            if df.empty or 'path_cost' not in df.columns: continue
            if 'crashed' in df.columns and (df['crashed'] == 1).any(): continue
            if 'collision_count' in df.columns and df['collision_count'].max() > 0: continue
            successful_dfs.append(df)
            
        if not successful_dfs: continue
        is_geometric = (successful_dfs[0]['time_to_goal'].sum() == 0) if 'time_to_goal' in successful_dfs[0].columns else True
        all_times, all_costs = [], []
        
        for df in successful_dfs:
            df_clean = df[df['path_cost'].notna() & (df['path_cost'] != np.inf)].copy()
            if not is_geometric and 'time_to_goal' in df_clean.columns:
                df_clean['sim_time'] = df_clean['time_to_goal'].max() - df_clean['time_to_goal']
            else:
                df_clean['sim_time'] = df_clean['elapsed_s']
                
            df_clean['sim_time_rounded'] = df_clean['sim_time'].round(1)
            all_times.extend(df_clean['sim_time_rounded'].tolist())
            all_costs.extend(df_clean['path_cost'].tolist())

        if not all_times: continue
        combined = pd.DataFrame({'time': all_times, 'cost': all_costs})
        grouped = combined.groupby('time')['cost'].agg(['mean', 'std']).reset_index()
        
        plt.plot(grouped['time'], grouped['mean'], label=planner, linewidth=2)
        plt.fill_between(grouped['time'], grouped['mean'] - grouped['std'], grouped['mean'] + grouped['std'], alpha=0.2)

    plt.xlabel("Simulation Time (s)")
    plt.ylabel("Path Cost")
    plt.title(f"Average Path Cost over Time (Successful Runs) - {scenario_name}")
    plt.legend()
    plt.grid(True, linestyle=':', alpha=0.7)
    
    out_path = os.path.join(BUILD_DIR, f"plot_anytime_{scenario_name}_average.png")
    plt.savefig(out_path, bbox_inches='tight')
    plt.close()
    print(f"[Saved Plot] {out_path}")

def main():
    scenarios = load_data(BUILD_DIR)
    if scenarios:
        for scenario_name, planners_data in scenarios.items():
            analyze_group_statistics(scenario_name, planners_data)
            save_latency_plot(scenario_name, planners_data)
            save_comparative_plot(scenario_name, planners_data)
    else: print("No CSV files found.")

if __name__ == "__main__": main()