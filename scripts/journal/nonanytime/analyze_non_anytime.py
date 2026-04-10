import pandas as pd
import glob
import os
import re
import numpy as np
import matplotlib.pyplot as plt

# --- CONFIGURATION ---
BUILD_DIR = "."  # Looking in current directory
# Matches: sim_KinodynamicFMTX_Dubins_seed_42_20260407_123456_metrics.csv
FILENAME_PATTERN = re.compile(r"sim_([A-Za-z0-9]+)_([A-Za-z0-9_]+)_seed_(\d+)_(\d{8}_\d{6})_metrics\.csv")

def load_data(directory):
    if not os.path.exists(directory): 
        print(f"Directory {directory} not found.")
        return {}
    
    files = glob.glob(os.path.join(directory, "sim_*_metrics.csv"))
    scenarios = {}

    for filepath in files:
        filename = os.path.basename(filepath)
        match = FILENAME_PATTERN.match(filename)
        if match:
            planner_raw = match.group(1)
            scenario = match.group(2)
            seed = match.group(3)
            
            # Identify if it's a non-anytime algorithm
            if "ANY" in planner_raw:
                continue # Skip ANYFMTX and ANYRRTX to focus only on non-anytime
            
            # Shorten long planner names for clean printing
            planner_clean = (planner_raw
                             .replace("Kinodynamic", "")
                             .replace("PRMStarDStarLite", "DLITE"))
            
            try:
                df = pd.read_csv(filepath)
                if df.empty: continue
                df.replace([np.inf, -np.inf], np.nan, inplace=True)
                
                if scenario not in scenarios: 
                    scenarios[scenario] = {}
                if planner_clean not in scenarios[scenario]: 
                    scenarios[scenario][planner_clean] = []
                    
                scenarios[scenario][planner_clean].append(df)
            except Exception as e:
                print(f"Warning: Could not load {filename}: {e}")
                
    return scenarios

def analyze_group_statistics(scenario_name, planners_data):
    """Prints a statistical summary averaging across all seeds for NON-ANYTIME algorithms."""
    print(f"\n{'='*140}")
    print(f" NON-ANYTIME AGGREGATE ANALYSIS: {scenario_name} (Average of all seeds)")
    print(f"{'='*140}")
    
    # We add 'Init Plan(ms)' to highlight the massive upfront cost of non-anytime algorithms
    header = (f"{'Planner':<12} | {'Seeds':<5} | {'Init Plan(ms)':<14} | {'Avg Lat':<8} | {'Max Lat':<8} | "
              f"{'Upd(ms)':<8} | {'Obs Chk':<8} | "
              f"{'Tree Sz':<8} | {'Deg(Out)':<8} | {'Avg Cost':<9}")
    print(header)
    print("-" * 140)

    for planner, dfs in planners_data.items():
        if not dfs: continue
        
        num_seeds = len(dfs)
        merged_df = pd.concat(dfs, ignore_index=True) if len(dfs) > 1 else dfs[0]
        
        if 'update_ms' not in merged_df.columns:
            print(f"{planner:<12} | Old Data Format - Skipping")
            continue

        # In non-anytime, the first frame is the massive initial batch graph construction.
        # We want to extract this specifically to show how long it takes to boot up.
        init_plan_ms_list = []
        for df in dfs:
            if not df.empty:
                # The maximum plan time usually occurs on the very first iteration
                init_plan_ms_list.append(df['plan_ms'].max())
        
        avg_init_plan = sum(init_plan_ms_list) / len(init_plan_ms_list) if init_plan_ms_list else 0.0

        # Extract metrics for the REPAIR phase (when obstacles move)
        df_event = merged_df[merged_df['update_ms'] > 0.001]
        
        if not df_event.empty:
            avg_lat  = df_event['total_latency_ms'].mean()
            max_lat  = df_event['total_latency_ms'].max()
            avg_upd  = df_event['update_ms'].mean()
            
            avg_obs   = df_event['obstacle_checks'].mean()
            avg_sz    = merged_df['tree_size'].max()  # The total tree size
            avg_deg_out = merged_df['avg_deg_out'].mean()
            avg_cost  = merged_df['path_cost'].mean()

            print(f"{planner:<12} | {num_seeds:<5} | "
                  f"\033[93m{avg_init_plan:<13.1f}\033[0m | "
                  f"\033[91m{avg_lat:<8.2f}\033[0m | {max_lat:<8.1f} | "
                  f"{avg_upd:<8.2f} | "
                  f"{avg_obs:<8.1f} | "
                  f"{avg_sz:<8.0f} | {avg_deg_out:<8.2f} | {avg_cost:<9.2f}")
        else:
             print(f"{planner:<12} | {num_seeds:<5} | \033[93m{avg_init_plan:<13.1f}\033[0m | [No dynamic events recorded]")

def save_comparative_plot(scenario_name, planners_data):
    """Creates a line plot showing mean path cost over time with std-dev fill."""
    if not planners_data: return

    plt.figure(figsize=(10, 6))

    for planner, dfs in planners_data.items():
        if not dfs: continue
        
        is_geometric = (dfs[0]['time_to_goal'].sum() == 0)
        time_col = 'elapsed_s' if is_geometric else 'time_to_goal'
        
        all_times = []
        all_costs = []
        
        for df in dfs:
            if 'path_cost' not in df.columns: continue
            
            df_clean = df.copy()
            if not is_geometric:
                max_budget = df_clean['time_to_goal'].max()
                df_clean['sim_time'] = max_budget - df_clean['time_to_goal']
            else:
                df_clean['sim_time'] = df_clean['elapsed_s']
                
            df_clean['sim_time_rounded'] = df_clean['sim_time'].round(1)
            
            all_times.extend(df_clean['sim_time_rounded'].tolist())
            all_costs.extend(df_clean['path_cost'].tolist())

        combined = pd.DataFrame({'time': all_times, 'cost': all_costs})
        grouped = combined.groupby('time')['cost'].agg(['mean', 'std']).reset_index()
        
        plt.plot(grouped['time'], grouped['mean'], label=planner, linewidth=2)
        plt.fill_between(grouped['time'], 
                         grouped['mean'] - grouped['std'], 
                         grouped['mean'] + grouped['std'], 
                         alpha=0.2)

    plt.xlabel("Simulation Time (s)", fontsize=12)
    plt.ylabel("Path Cost", fontsize=12)
    plt.title(f"NON-ANYTIME: Average Path Cost over Time (30 Seeds) - {scenario_name}", fontsize=14)
    plt.legend(fontsize=10)
    plt.grid(True, linestyle='--', alpha=0.7)

    out_path = os.path.join(BUILD_DIR, f"plot_non_anytime_{scenario_name}_average.png")
    plt.savefig(out_path, dpi=200, bbox_inches='tight')
    plt.close()

    print(f"[Saved Plot] {out_path}")

def main():
    scenarios = load_data(BUILD_DIR)
    
    if not scenarios:
        print("No NON-ANYTIME CSV files matching the pattern were found.")
        return

    for scenario_name, planners_data in scenarios.items():
        analyze_group_statistics(scenario_name, planners_data)
        save_comparative_plot(scenario_name, planners_data)

if __name__ == "__main__":
    main()