import pandas as pd
import glob
import os
import re
import numpy as np
import matplotlib.pyplot as plt
from pandas.errors import EmptyDataError

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

        df_steady = merged_df[merged_df['update_ms'] <= 0.001]
        steady_update.append(df_steady['update_ms'].mean() if not df_steady.empty else 0)
        steady_plan.append(df_steady['plan_ms'].mean() if not df_steady.empty else 0)

        df_repair = merged_df[merged_df['update_ms'] > 0.001]
        repair_update.append(df_repair['update_ms'].mean() if not df_repair.empty else 0)
        repair_plan.append(df_repair['plan_ms'].mean() if not df_repair.empty else 0)

    if not planners: return
    x = np.arange(len(planners))
    width = 0.35
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 6))

    ax1.bar(x, steady_update, width, label='Update Obstacles', color='#ff9999', edgecolor='black')
    ax1.bar(x, steady_plan, width, bottom=steady_update, label='Plan Time', color='#66b3ff', edgecolor='black')
    ax1.set_ylabel('Average Latency (ms)', fontsize=12)
    ax1.set_title('Steady-State Cruising', fontsize=13)
    ax1.set_xticks(x)
    ax1.set_xticklabels(planners, fontsize=11)
    ax1.grid(axis='y', linestyle='--', alpha=0.7)

    ax2.bar(x, repair_update, width, label='Update Obstacles', color='#ff9999', edgecolor='black')
    ax2.bar(x, repair_plan, width, bottom=repair_update, label='Plan Time', color='#66b3ff', edgecolor='black')
    ax2.set_title('Dynamic Repair Event', fontsize=13)
    ax2.set_xticks(x)
    ax2.set_xticklabels(planners, fontsize=11)
    ax2.grid(axis='y', linestyle='--', alpha=0.7)
    
    for i in range(len(planners)):
        total_val = repair_update[i] + repair_plan[i]
        ax2.text(i, total_val + (total_val * 0.02), f"{total_val:.1f} ms", ha='center', va='bottom', fontweight='bold')

    handles, labels = ax2.get_legend_handles_labels()
    fig.legend(handles, labels, loc='upper center', bbox_to_anchor=(0.5, 1.05), ncol=2, fontsize=11)
    plt.suptitle(f"Algorithmic Latency Breakdown - {scenario_name}", fontsize=16, y=1.12)
    plt.tight_layout()
    out_path = os.path.join(BUILD_DIR, f"plot_latency_breakdown_{scenario_name}.png")
    plt.savefig(out_path, dpi=200, bbox_inches='tight')
    plt.close()
    print(f"[Saved Plot] {out_path}")

def analyze_group_statistics(scenario_name, planners_data):
    print(f"\n{'='*140}")
    print(f" ANYTIME AGGREGATE ANALYSIS: {scenario_name}")
    print(f" (Efficiency metrics are averaged over SUCCESSFUL runs only)")
    print(f"{'='*140}")
    
    header = (f"{'Planner':<12} | {'Succ%':<6} | {'Avg Lat':<8} | {'p99 Lat':<8} | "
              f"{'Plan(ms)':<8} | {'Upd(ms)':<8} | {'Obs Chk':<8} | {'Tree Sz':<8} | {'Avg Cost':<9} | {'Rad(S->E)':<12}")
    print(header)
    print("-" * 140)

    for planner, dfs in planners_data.items():
        valid_dfs = [df for df in dfs if not df.empty]
        if not valid_dfs: continue
        
        num_seeds = len(valid_dfs)
        successful_dfs = []
        success_count = 0
        
        for df in valid_dfs:
            if 'crashed' in df.columns:
                if not (df['crashed'] == 1).any():
                    success_count += 1
                    successful_dfs.append(df)
            elif 'collision_count' in df.columns:
                if df['collision_count'].max() == 0:
                    success_count += 1
                    successful_dfs.append(df)
            else:
                success_count += 1
                successful_dfs.append(df)
                
        success_rate = (success_count / num_seeds) * 100
        
        if not successful_dfs:
            print(f"{planner:<12} | {success_rate:>4.0f}% | [All runs failed]")
            continue
            
        merged_df = pd.concat(successful_dfs, ignore_index=True)
        if 'update_ms' not in merged_df.columns: continue

        df_event = merged_df[merged_df['update_ms'] > 0.001]
        
        if not df_event.empty:
            avg_lat  = df_event['total_latency_ms'].mean()
            p99_lat  = df_event['total_latency_ms'].quantile(0.99)
            avg_plan = df_event['plan_ms'].mean()
            avg_upd  = df_event['update_ms'].mean()
            avg_obs  = df_event['obstacle_checks'].mean()
            avg_sz   = df_event['tree_size'].max()  
            avg_cost = df_event.loc[df_event['path_cost'] != np.inf, 'path_cost'].mean()
            
            start_rad = merged_df['radius'].iloc[0] if 'radius' in merged_df.columns else 0.0
            end_rad   = merged_df['radius'].iloc[-1] if 'radius' in merged_df.columns else 0.0
            rad_str   = f"{start_rad:.1f}->{end_rad:.1f}"

            print(f"{planner:<12} | {success_rate:>4.0f}% | \033[91m{avg_lat:<8.2f}\033[0m | {p99_lat:<8.1f} | "
                  f"{avg_plan:<8.2f} | {avg_upd:<8.2f} | {avg_obs:<8.1f} | {avg_sz:<8.0f} | {avg_cost:<9.2f} | {rad_str:<12}")

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
            df_clean = df[df['path_cost'] != np.inf].copy()
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
        
        plt.plot(grouped['time'].values, grouped['mean'].values, label=planner, linewidth=2)
        plt.fill_between(grouped['time'].values, (grouped['mean'] - grouped['std']).values, 
                         (grouped['mean'] + grouped['std']).values, alpha=0.2)

    plt.xlabel("Simulation Time (s)", fontsize=12)
    plt.ylabel("Path Cost", fontsize=12)
    plt.title(f"Average Path Cost over Time (Successful Runs) - {scenario_name}", fontsize=14)
    plt.legend(fontsize=10)
    plt.grid(True, linestyle='--', alpha=0.7)
    out_path = os.path.join(BUILD_DIR, f"plot_anytime_{scenario_name}_average.png")
    plt.savefig(out_path, dpi=200, bbox_inches='tight')
    plt.close()
    print(f"[Saved Plot] {out_path}")

def main():
    scenarios = load_data(BUILD_DIR)
    if scenarios:
        for scenario_name, planners_data in scenarios.items():
            analyze_group_statistics(scenario_name, planners_data)
            save_comparative_plot(scenario_name, planners_data)
            save_latency_plot(scenario_name, planners_data)
    else:
        print("No ANYTIME CSV files found.")

if __name__ == "__main__":
    main()