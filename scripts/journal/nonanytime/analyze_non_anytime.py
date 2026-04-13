import pandas as pd
import glob
import os
import re
import numpy as np
import matplotlib.pyplot as plt
from pandas.errors import EmptyDataError

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
        
        if "ANY" in planner_raw: 
            continue 
        
        planner_clean = planner_raw.replace("Kinodynamic", "").replace("PRMStarDStarLite", "DLITE")
        
        try:
            df = pd.read_csv(filepath)
            if df.empty: continue
            df.replace([np.inf, -np.inf], np.nan, inplace=True)
            
            if scenario not in scenarios: scenarios[scenario] = {}
            if planner_clean not in scenarios[scenario]: scenarios[scenario][planner_clean] = []
            scenarios[scenario][planner_clean].append(df)
        except EmptyDataError: 
            pass
        except Exception as e: 
            print(f"Warning: Could not load {filename}: {e}")
            
    return scenarios

def analyze_group_statistics(scenario_name, planners_data):
    print(f"\n{'='*170}")
    print(f" NON-ANYTIME AGGREGATE ANALYSIS: {scenario_name}")
    print(f" (Efficiency metrics are averaged over SUCCESSFUL runs only)")
    print(f"{'='*170}")
    
    # Added Setup(ms) to header
    header = (f"{'Planner':<12} | {'Succ%':<6} | {'Setup(ms)':<9} | {'InitPlan(ms)':<12} | {'Avg Lat':<8} | {'p99 Lat':<8} | "
              f"{'Upd(ms)':<7} | {'Obs Chk':<7} | {'Q Ops':<7} | {'Tree Sz':<7} | {'DegOut':<6} | {'DegIn':<6} | {'Rad':<5} | {'Avg Cost':<8}")
    print(header)
    print("-" * 170)

    for planner, dfs in planners_data.items():
        valid_dfs = [df for df in dfs if not df.empty]
        if not valid_dfs: continue
        
        num_seeds = len(valid_dfs)
        successful_dfs = []
        
        for df in valid_dfs:
            if 'collision_count' in df.columns:
                if df['collision_count'].max() == 0:
                    successful_dfs.append(df)
            else:
                successful_dfs.append(df)
                
        success_count = len(successful_dfs)
        success_rate = (success_count / num_seeds) * 100 if num_seeds > 0 else 0.0
        
        if not successful_dfs:
            print(f"{planner:<12} | {success_rate:>4.0f}% | [All runs failed, no efficiency metrics available]")
            continue
            
        merged_df = pd.concat(successful_dfs, ignore_index=True)
        if 'update_ms' not in merged_df.columns: continue

        # Extract Setup and Init Plan times from the first row of each successful run
        setup_ms_list = [df['setup_ms'].iloc[0] for df in successful_dfs if 'setup_ms' in df.columns and not df.empty]
        avg_setup_ms = sum(setup_ms_list) / len(setup_ms_list) if setup_ms_list else 0.0
        
        init_plan_ms_list = [df['plan_ms'].iloc[0] for df in successful_dfs if not df.empty]
        avg_init_plan = sum(init_plan_ms_list) / len(init_plan_ms_list) if init_plan_ms_list else 0.0

        df_event = merged_df[merged_df['update_ms'] > 0.001]
        
        if not df_event.empty:
            avg_lat  = df_event['total_latency_ms'].mean()
            p99_lat  = df_event['total_latency_ms'].quantile(0.99)
            avg_upd  = df_event['update_ms'].mean()
            avg_obs  = df_event['obstacle_checks'].mean()
            
            avg_q    = df_event['queue_operations'].mean() if 'queue_operations' in df_event.columns else 0.0
            avg_sz   = merged_df['tree_size'].max()  
            avg_deg_out = merged_df['avg_deg_out'].mean()
            avg_deg_in  = merged_df['avg_deg_in'].mean() if 'avg_deg_in' in merged_df.columns else 0.0
            avg_rad     = merged_df['radius'].mean() if 'radius' in merged_df.columns else 0.0
            
            avg_cost = merged_df.loc[merged_df['path_cost'].notna(), 'path_cost'].mean()

            # Added avg_setup_ms to the output row formatting
            print(f"{planner:<12} | {success_rate:>4.0f}% | \033[93m{avg_setup_ms:<9.1f}\033[0m | \033[93m{avg_init_plan:<12.1f}\033[0m | "
                  f"\033[91m{avg_lat:<8.2f}\033[0m | {p99_lat:<8.1f} | {avg_upd:<7.2f} | "
                  f"{avg_obs:<7.1f} | {avg_q:<7.0f} | {avg_sz:<7.0f} | {avg_deg_out:<6.2f} | "
                  f"{avg_deg_in:<6.2f} | {avg_rad:<5.2f} | {avg_cost:<8.2f}")
        else:
             print(f"{planner:<12} | {success_rate:>4.0f}% | \033[93m{avg_setup_ms:<9.1f}\033[0m | \033[93m{avg_init_plan:<12.1f}\033[0m | [No dynamic events recorded]")

def save_comparative_plot(scenario_name, planners_data):
    if not planners_data: return
    plt.figure(figsize=(12, 7))

    for planner, dfs in planners_data.items():
        successful_dfs = []
        for df in dfs:
            if df.empty or 'path_cost' not in df.columns: continue
            if 'collision_count' in df.columns and df['collision_count'].max() > 0: continue
            successful_dfs.append(df)
            
        if not successful_dfs: continue
        
        is_geometric = (successful_dfs[0]['time_to_goal'].sum() == 0) if 'time_to_goal' in successful_dfs[0].columns else True
        
        all_times, all_costs = [], []
        for df in successful_dfs:
            df_clean = df[df['path_cost'].notna()].copy()
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
        
        plt.plot(grouped['time'].values, grouped['mean'].values, label=planner, linewidth=2)
        plt.fill_between(grouped['time'].values, 
                         (grouped['mean'] - grouped['std']).values, 
                         (grouped['mean'] + grouped['std']).values, alpha=0.2)

    plt.xlabel("Simulation Time (s)", fontsize=12)
    plt.ylabel("Path Cost", fontsize=12)
    plt.title(f"Average Path Cost over Time (Successful Runs) - {scenario_name}", fontsize=14)
    plt.legend(fontsize=10)
    plt.grid(True, linestyle='--', alpha=0.7)
    out_path = os.path.join(BUILD_DIR, f"plot_non_anytime_{scenario_name}_average.png")
    plt.savefig(out_path, dpi=200, bbox_inches='tight')
    plt.close()
    print(f"[Saved Plot] {out_path}")

def main():
    scenarios = load_data(BUILD_DIR)
    if scenarios:
        for scenario_name, planners_data in scenarios.items():
            analyze_group_statistics(scenario_name, planners_data)
            save_comparative_plot(scenario_name, planners_data)
    else:
        print("No NON-ANYTIME CSV files found. Check regex or the files in the directory.")

if __name__ == "__main__":
    main()