import pandas as pd
import glob
import os
import re
import numpy as np
import matplotlib.pyplot as plt

# --- CONFIGURATION ---
BUILD_DIR = "." 
# Matches: sim_ANYFMTX_Dubins_seed_42_20260407_123456_metrics.csv
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
            
            # Shorten long planner names for clean printing
            planner_clean = (planner_raw
                             .replace("Kinodynamic", "")
                             .replace("PRMStarDStarLite", "DLITE")
                             .replace("PRMStar", "")
                             .replace("ANY", ""))
            
            try:
                df = pd.read_csv(filepath)
                if df.empty: continue
                df.replace([np.inf, -np.inf], np.nan, inplace=True)
                
                # Create nested dictionary: Scenario -> Planner -> List of DataFrames (Seeds)
                if scenario not in scenarios: 
                    scenarios[scenario] = {}
                if planner_clean not in scenarios[scenario]: 
                    scenarios[scenario][planner_clean] = []
                    
                scenarios[scenario][planner_clean].append(df)
            except Exception as e:
                print(f"Warning: Could not load {filename}: {e}")
                
    return scenarios


def save_latency_plot(scenario_name, planners_data):
    """Creates a stacked bar chart comparing Steady-State vs Repair Latency."""
    if not planners_data: return

    planners = []
    steady_update = []
    steady_plan = []
    repair_update = []
    repair_plan = []

    for planner, dfs in planners_data.items():
        if not dfs: continue
        
        merged_df = pd.concat(dfs, ignore_index=True) if len(dfs) > 1 else dfs[0]
        if 'update_ms' not in merged_df.columns: continue
        
        planners.append(planner)

        # 1. STEADY STATE (No major obstacle changes, mostly just sampling)
        df_steady = merged_df[merged_df['update_ms'] <= 0.001]
        steady_update.append(df_steady['update_ms'].mean() if not df_steady.empty else 0)
        steady_plan.append(df_steady['plan_ms'].mean() if not df_steady.empty else 0)

        # 2. REPAIR EVENT (Obstacles turned around, massive updates required)
        df_repair = merged_df[merged_df['update_ms'] > 0.001]
        repair_update.append(df_repair['update_ms'].mean() if not df_repair.empty else 0)
        repair_plan.append(df_repair['plan_ms'].mean() if not df_repair.empty else 0)

    x = np.arange(len(planners))
    width = 0.35

    # Create a 1x2 subplot because Repair Latency is WAY higher than Steady Latency
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 6))

    # --- LEFT PLOT: STEADY STATE ---
    ax1.bar(x, steady_update, width, label='Update Obstacles Time', color='#ff9999', edgecolor='black')
    ax1.bar(x, steady_plan, width, bottom=steady_update, label='Sampling / Plan Time', color='#66b3ff', edgecolor='black')
    
    ax1.set_ylabel('Average Latency (ms)', fontsize=12)
    ax1.set_title('Steady-State Cruising (Sampling Only)', fontsize=13)
    ax1.set_xticks(x)
    ax1.set_xticklabels(planners, fontsize=11)
    ax1.grid(axis='y', linestyle='--', alpha=0.7)

    # --- RIGHT PLOT: REPAIR EVENTS ---
    ax2.bar(x, repair_update, width, label='Update Obstacles Time (Eager Repair)', color='#ff9999', edgecolor='black')
    ax2.bar(x, repair_plan, width, bottom=repair_update, label='Sampling / Plan Time (Lazy Repair)', color='#66b3ff', edgecolor='black')
    
    ax2.set_title('Dynamic Repair Event (Obstacles Turned)', fontsize=13)
    ax2.set_xticks(x)
    ax2.set_xticklabels(planners, fontsize=11)
    ax2.grid(axis='y', linestyle='--', alpha=0.7)
    
    # Add data labels on top of the Repair bars to show the EXACT total ms
    for i in range(len(planners)):
        total_val = repair_update[i] + repair_plan[i]
        ax2.text(i, total_val + (total_val * 0.02), f"{total_val:.1f} ms", ha='center', va='bottom', fontweight='bold')

    # Shared Legend
    handles, labels = ax2.get_legend_handles_labels()
    fig.legend(handles, labels, loc='upper center', bbox_to_anchor=(0.5, 1.05), ncol=2, fontsize=11)

    plt.suptitle(f"Algorithmic Latency Breakdown - {scenario_name}", fontsize=16, y=1.12)
    plt.tight_layout()

    out_path = os.path.join(BUILD_DIR, f"plot_latency_breakdown_{scenario_name}.png")
    plt.savefig(out_path, dpi=200, bbox_inches='tight')
    plt.close()
    print(f"[Saved Plot] {out_path}")

def analyze_group_statistics(scenario_name, planners_data):
    """Prints a statistical summary averaging across all 30 seeds."""
    print(f"\n{'='*140}")
    print(f" AGGREGATE ANALYSIS: {scenario_name} (Average of all seeds)")
    print(f"{'='*140}")
    
    header = (f"{'Planner':<12} | {'Seeds':<5} | {'Avg Lat':<8} | {'Max Lat':<8} | "
              f"{'Plan(ms)':<8} | {'Upd(ms)':<8} | {'Obs Chk':<8} | "
              f"{'Tree Sz':<8} | {'Deg(Out)':<8} | {'Avg Cost':<9} | {'Rad(S->E)':<12}")
    print(header)
    print("-" * 140)

    for planner, dfs in planners_data.items():
        if not dfs: continue
        
        num_seeds = len(dfs)
        
        # We concatenate all dataframes to get overall averages
        merged_df = pd.concat(dfs, ignore_index=True) if len(dfs) > 1 else dfs[0]
        
        # Check if the new format exists
        if 'update_ms' not in merged_df.columns:
            print(f"{planner:<12} | Old Data Format - Skipping")
            continue

        # Extract overall metrics
        df_event = merged_df[merged_df['update_ms'] > 0.001]
        
        if not df_event.empty:
            avg_lat  = df_event['total_latency_ms'].mean()
            max_lat  = df_event['total_latency_ms'].max()
            avg_plan = df_event['plan_ms'].mean()
            avg_upd  = df_event['update_ms'].mean()
            
            avg_obs   = df_event['obstacle_checks'].mean()
            avg_sz    = df_event['tree_size'].max()  # Max tree size reached
            avg_deg_out = df_event['avg_deg_out'].mean()
            avg_cost  = df_event['path_cost'].mean()
            
            start_rad = merged_df['radius'].iloc[0] if 'radius' in merged_df.columns else 0.0
            end_rad   = merged_df['radius'].iloc[-1] if 'radius' in merged_df.columns else 0.0
            rad_str   = f"{start_rad:.1f}->{end_rad:.1f}"

            print(f"{planner:<12} | {num_seeds:<5} | "
                  f"\033[91m{avg_lat:<8.2f}\033[0m | {max_lat:<8.1f} | "
                  f"{avg_plan:<8.2f} | {avg_upd:<8.2f} | "
                  f"{avg_obs:<8.1f} | "
                  f"{avg_sz:<8.0f} | {avg_deg_out:<8.2f} | {avg_cost:<9.2f} | {rad_str:<12}")

def save_comparative_plot(scenario_name, planners_data):
    """Creates a line plot showing mean path cost over time with std-dev fill."""
    if not planners_data: return

    plt.figure(figsize=(10, 6))

    for planner, dfs in planners_data.items():
        if not dfs: continue
        
        # We need a common time axis to average across multiple random seeds.
        # For Kinodynamic (R2T/Dubins/Thruster) we use `time_to_goal`.
        # For Geometric (R2) `time_to_goal` is 0, so we use `elapsed_s`.
        
        is_geometric = (dfs[0]['time_to_goal'].sum() == 0)
        
        time_col = 'elapsed_s' if is_geometric else 'time_to_goal'
        
        all_times = []
        all_costs = []
        
        # Round the time column to 1 decimal place so we can group/bin the seeds together
        for df in dfs:
            if 'path_cost' not in df.columns: continue
            
            df_clean = df.copy()
            if not is_geometric:
                # Reverse time_to_goal so it goes from 0 -> Budget
                max_budget = df_clean['time_to_goal'].max()
                df_clean['sim_time'] = max_budget - df_clean['time_to_goal']
            else:
                df_clean['sim_time'] = df_clean['elapsed_s']
                
            df_clean['sim_time_rounded'] = df_clean['sim_time'].round(1)
            
            all_times.extend(df_clean['sim_time_rounded'].tolist())
            all_costs.extend(df_clean['path_cost'].tolist())

        # Create a giant dataframe of all seeds to calculate mean and standard deviation
        combined = pd.DataFrame({'time': all_times, 'cost': all_costs})
        grouped = combined.groupby('time')['cost'].agg(['mean', 'std']).reset_index()
        
        # Plot Mean
        plt.plot(grouped['time'], grouped['mean'], label=planner, linewidth=2)
        
        # Fill Standard Deviation (transparency 0.2)
        plt.fill_between(grouped['time'], 
                         grouped['mean'] - grouped['std'], 
                         grouped['mean'] + grouped['std'], 
                         alpha=0.2)

    plt.xlabel("Simulation Time (s)", fontsize=12)
    plt.ylabel("Path Cost", fontsize=12)
    plt.title(f"Average Path Cost over Time (30 Seeds) - {scenario_name}", fontsize=14)
    plt.legend(fontsize=10)
    plt.grid(True, linestyle='--', alpha=0.7)

    out_path = os.path.join(BUILD_DIR, f"plot_{scenario_name}_average.png")
    plt.savefig(out_path, dpi=200, bbox_inches='tight')
    plt.close()

    print(f"[Saved Plot] {out_path}")

def main():
    # Load all CSV files in the build directory
    scenarios = load_data(BUILD_DIR)
    
    if not scenarios:
        print(f"No CSV files matching the pattern were found in {BUILD_DIR}")
        return

    # Process each State Space separately
    for scenario_name, planners_data in scenarios.items():
        analyze_group_statistics(scenario_name, planners_data)
        save_comparative_plot(scenario_name, planners_data)
        save_latency_plot(scenario_name, planners_data)     # New Latency Plot

if __name__ == "__main__":
    main()