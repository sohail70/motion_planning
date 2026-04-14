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
FALLBACK_PATTERN = re.compile(r"sim_([A-Za-z0-9]+)_([A-Za-z0-9_]+)_(\d{8}_\d{6})_metrics\.csv")

def load_data(directory):
    if not os.path.exists(directory): return {}
    files = glob.glob(os.path.join(directory, "sim_*_metrics.csv"))
    scenarios = {}

    for filepath in files:
        filename = os.path.basename(filepath)
        match = FILENAME_PATTERN.match(filename)
        scenario = "default"
        
        if match:
            planner_raw = match.group(1)
            scenario = match.group(2)
        else:
            fallback = FALLBACK_PATTERN.match(filename)
            if fallback:
                planner_raw = fallback.group(1)
                scenario = fallback.group(2)
            else:
                parts = filename.replace("sim_", "").replace("_metrics.csv", "").split("_")
                planner_raw = parts[0]
                if len(parts) > 2: scenario = "_".join(parts[1:-2])
        
        # Skip anytime algorithms
        if "ANY" in planner_raw: 
            continue 
        
        # Clean names for the paper
        planner_clean = planner_raw.replace("Kinodynamic", "").replace("PRMStarDStarLite", "DLITE").replace("FMTX", "FMTX")
        
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
    planners, init_plan, repair_lat = [], [], []

    for planner, dfs in planners_data.items():
        valid_dfs = [df for df in dfs if not df.empty]
        if not valid_dfs: continue
        
        merged_df = pd.concat(valid_dfs, ignore_index=True)
        planners.append(planner)

        # Initial Plan time (from the cold-start frame)
        df_init = merged_df[merged_df['elapsed_s'] == 0.0]
        init_plan.append(df_init['plan_ms'].mean() if not df_init.empty else 0)

        # Repair latency (only when obstacles move)
        df_event = merged_df[(merged_df['elapsed_s'] > 0.0) & (merged_df['update_ms'] > 0.001)]
        if not df_event.empty:
            repair_lat.append(df_event['total_latency_ms'].mean())
        else:
            repair_lat.append(0)

    if not planners: return
    x = np.arange(len(planners))
    width = 0.4
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10, 5))

    c_init, c_repair = '#2ca02c', '#d95f02'

    # Plot 1: Initial Planning Time
    bars1 = ax1.bar(x, init_plan, width, color=c_init, edgecolor='black', hatch='\\\\')
    ax1.set_ylabel('Average Latency (ms)')
    ax1.set_title('Graph Construction ($T_{init}$)')
    ax1.set_xticks(x)
    ax1.set_xticklabels(planners)
    ax1.grid(axis='y', linestyle=':', alpha=0.7)
    for bar in bars1:
        val = bar.get_height()
        if val > 0: ax1.text(bar.get_x() + bar.get_width()/2., val * 1.05, f"{val:.1f}", ha='center', va='bottom', fontweight='bold')

    # Plot 2: Dynamic Repair Time
    bars2 = ax2.bar(x, repair_lat, width, color=c_repair, edgecolor='black', hatch='//')
    ax2.set_title('Dynamic Replan Event ($T_{repair}$)')
    ax2.set_xticks(x)
    ax2.set_xticklabels(planners)
    ax2.grid(axis='y', linestyle=':', alpha=0.7)
    for bar in bars2:
        val = bar.get_height()
        if val > 0: ax2.text(bar.get_x() + bar.get_width()/2., val * 1.05, f"{val:.1f}", ha='center', va='bottom', fontweight='bold')

    plt.suptitle(f"Fixed-Graph Latency Breakdown: {scenario_name}", y=1.05)
    plt.tight_layout()
    
    out_path = os.path.join(BUILD_DIR, f"plot_fixed_latency_{scenario_name}.png")
    plt.savefig(out_path, bbox_inches='tight')
    plt.close()
    print(f"[Saved Plot] {out_path}")

def analyze_group_statistics(scenario_name, planners_data):
    print(f"\n{'='*155}")
    print(f" FIXED-GRAPH ANALYSIS: {scenario_name} (Successful runs only)")
    print(f"{'='*155}")
    
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
                
        succ_rate = (success_count / num_seeds) * 100 if num_seeds > 0 else 0.0
        if not successful_dfs:
            summary_data.append({"Planner": planner, "Succ(%)": f"{succ_rate:.0f}%", "Status": "All runs failed"})
            continue
            
        merged_df = pd.concat(successful_dfs, ignore_index=True)
        if 'update_ms' not in merged_df.columns: continue

        # Initial Phase (Graph Construction)
        df_init = merged_df[merged_df['elapsed_s'] == 0.0]
        t_set = df_init['setup_ms'].mean() if 'setup_ms' in df_init.columns else 0.0
        t_init = df_init['plan_ms'].mean() if not df_init.empty else 0.0

        # Dynamic Repair Phase (No new samples, only graph repair)
        df_event = merged_df[(merged_df['elapsed_s'] > 0.0) & (merged_df['update_ms'] > 0.001)]
        
        if not df_event.empty:
            t_repair_avg = df_event['total_latency_ms'].mean()
            t_repair_p99 = df_event['total_latency_ms'].quantile(0.99)
            obs_chk   = df_event['obstacle_checks'].mean()
            queue_ops = df_event['queue_operations'].mean() if 'queue_operations' in df_event.columns else 0.0
            avg_cost  = df_event['path_cost'].dropna().mean()
        else:
            t_repair_avg = t_repair_p99 = obs_chk = queue_ops = avg_cost = 0.0

        tree_sz = merged_df['tree_size'].max()
        
        # Calculate isolated nodes string with percentage
        if 'isolated_nodes' in merged_df.columns:
            iso_count = merged_df['isolated_nodes'].max()
            iso_pct = (iso_count / tree_sz) * 100 if tree_sz > 0 else 0.0
            iso_str = f"{iso_count:.0f} ({iso_pct:.1f}%)"
        else:
            iso_str = "0 (0.0%)"

        deg_in  = merged_df['avg_deg_in'].mean() if 'avg_deg_in' in merged_df.columns else 0.0
        deg_out = merged_df['avg_deg_out'].mean() if 'avg_deg_out' in merged_df.columns else 0.0

        summary_data.append({
            "Planner": planner,
            "Succ(%)": f"{succ_rate:.0f}%",
            "T_set(ms)": f"{t_set:.1f}",
            "T_init(ms)": f"{t_init:.1f}",
            "T_repair(ms)": f"{t_repair_avg:.2f}",
            "Rep_p99(ms)": f"{t_repair_p99:.1f}",
            "Obs_Chk": f"{obs_chk:.0f}",
            "Q_Ops": f"{queue_ops:.0f}",
            "Path_Cost": f"{avg_cost:.2f}",
            "|V|": f"{tree_sz:.0f}",
            "Iso_Nodes": iso_str,
            "Deg(I/O)": f"{deg_in:.1f}/{deg_out:.1f}"
        })

    if summary_data:
        df_out = pd.DataFrame(summary_data)
        
        # Update header to include Iso_Nodes
        header = (f"{'Planner':<12} | {'Succ%':<6} | {'T_set(ms)':<9} | {'T_init(ms)':<10} | {'T_repair':<8} | {'Rep_p99':<8} | "
                  f"{'Obs_Chk':<7} | {'Q_Ops':<7} | {'|V|':<6} | {'Iso_Nodes':<12} | {'Deg(I/O)':<9} | {'Path_Cost':<9}")
        print(header)
        print("-" * 155)
        
        # Manual clean printing to ensure alignment
        for row in summary_data:
            if "Status" in row:
                print(f"{row['Planner']:<12} | {row['Succ(%)']:>4} | {row['Status']}")
            else:
                print(f"{row['Planner']:<12} | {row['Succ(%)']:>4} | \033[93m{row['T_set(ms)']:<9}\033[0m | \033[93m{row['T_init(ms)']:<10}\033[0m | "
                      f"\033[91m{row['T_repair(ms)']:<8}\033[0m | {row['Rep_p99(ms)']:<8} | "
                      f"{row['Obs_Chk']:<7} | {row['Q_Ops']:<7} | {row['|V|']:<6} | {row['Iso_Nodes']:<12} | "
                      f"{row['Deg(I/O)']:<9} | {row['Path_Cost']:<9}")

        print("-" * 155)

        # Generate LaTeX table code 
        latex_str = df_out.to_latex(
            index=False, 
            caption=f"Fixed-Graph Performance Metrics: {scenario_name}",
            label=f"tab:fixed_metrics_{scenario_name}",
            column_format="l" + "c"*(len(df_out.columns)-1),
            escape=False 
        )
        print("\n--- LaTeX Table Snippet ---")
        print(latex_str)
        print("---------------------------\n")

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
            if df_clean.empty: continue
            
            if not is_geometric and 'time_to_goal' in df_clean.columns:
                start_budget = df_clean['time_to_goal'].max()
                df_clean['sim_time'] = start_budget - df_clean['time_to_goal']
            else:
                df_clean['sim_time'] = df_clean['elapsed_s']
                
            df_clean['sim_time_rounded'] = df_clean['sim_time'].round(1)
            all_times.extend(df_clean['sim_time_rounded'].tolist())
            all_costs.extend(df_clean['path_cost'].tolist())

        if not all_times: continue
        combined = pd.DataFrame({'time': all_times, 'cost': all_costs})
        combined.dropna(subset=['cost'], inplace=True)
        
        grouped = combined.groupby('time')['cost'].agg(['mean', 'std']).reset_index()
        
        plt.plot(grouped['time'], grouped['mean'], label=planner, linewidth=2)
        plt.fill_between(grouped['time'], grouped['mean'] - grouped['std'], grouped['mean'] + grouped['std'], alpha=0.2)

    plt.xlabel("Simulation Time (s)")
    plt.ylabel("Path Cost")
    plt.title(f"Average Path Cost over Time (Successful Runs) - {scenario_name}")
    plt.legend()
    plt.grid(True, linestyle=':', alpha=0.7)
    
    out_path = os.path.join(BUILD_DIR, f"plot_fixed_anytime_{scenario_name}_average.png")
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
    else:
        print("No NON-ANYTIME CSV files found.")

if __name__ == "__main__":
    main()