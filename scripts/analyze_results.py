import pandas as pd
import glob
import os
import re
import numpy as np

# --- CONFIGURATION ---
BUILD_DIR = "../build/" 
FILENAME_PATTERN = re.compile(r"sim_([A-Za-z0-9]+)_([A-Za-z0-9_]+)_(\d{8}_\d{6})_metrics\.csv")

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
            planner_clean = planner_raw.replace("Kinodynamic", "").replace("PRMStar", "")
            try:
                df = pd.read_csv(filepath)
                if df.empty: continue
                df.replace([np.inf, -np.inf], np.nan, inplace=True)
                if scenario not in scenarios: scenarios[scenario] = []
                scenarios[scenario].append({"planner": planner_clean, "data": df})
            except Exception: pass
    return scenarios

def analyze_group(group_name, results, scenario_name):
    if not results: return

    print(f"\n{'='*140}")
    print(f" {group_name} ANALYSIS: {scenario_name}")
    print(f"{'='*140}")

    has_split = 'update_ms' in results[0]['data'].columns

    if has_split:
        # --- HEADER ---
        header = (f"{'Planner':<10} | {'Type':<8} | {'Count':<6} | "
                  f"{'Avg Lat':<8} | {'Max Lat':<8} | {'Plan(ms)':<8} | {'Upd(ms)':<8} | "
                  f"{'Obs Chk':<8} | {'Rewires':<8} | {'Tree Sz':<8} | {'Deg(Out)':<8} | {'Deg(In)':<8} | {'Path Cost':<9}")
        print(header)
        print("-" * 140)

        for res in results:
            df = res['data']
            planner = res['planner']
            
            # --- 1. DYNAMIC EVENT FRAMES (Repair Phase) ---
            df_event = df[df['update_ms'] > 0.001]
            if not df_event.empty:
                avg_lat  = df_event['total_latency_ms'].mean()
                max_lat  = df_event['total_latency_ms'].max()
                avg_plan = df_event['plan_ms'].mean()
                avg_upd  = df_event['update_ms'].mean()
                
                # New Metrics
                avg_obs  = df_event['obstacle_checks'].mean() if 'obstacle_checks' in df.columns else 0
                avg_rew  = df_event['rewire_neighbor_searches'].mean() if 'rewire_neighbor_searches' in df.columns else 0
                avg_sz   = df_event['tree_size'].max() if 'tree_size' in df.columns else 0
                avg_deg_out = df_event['avg_deg_out'].max() if 'avg_deg_out' in df.columns else 0
                avg_deg_in  = df_event['avg_deg_in'].max() if 'avg_deg_in' in df.columns else 0

                avg_cost = df_event['path_cost'].mean() if 'path_cost' in df.columns else 0
                
                print(f"{planner:<10} | {'REPAIR':<8} | {len(df_event):<6} | "
                      f"\033[91m{avg_lat:<8.2f}\033[0m | {max_lat:<8.1f} | "
                      f"{avg_plan:<8.2f} | {avg_upd:<8.2f} | "
                      f"{avg_obs:<8.1f} | {avg_rew:<8.1f} | {avg_sz:<8.0f} | {avg_deg_out:<8.2f} | {avg_deg_in:<8.2f} | {avg_cost:<9.2f}")

            # --- 2. STEADY STATE FRAMES (Exploration Phase) ---
# --- 2. STEADY STATE FRAMES (Exploration Phase) ---
            df_steady = df[df['update_ms'] <= 0.001]
            if not df_steady.empty:
                avg_lat = df_steady['total_latency_ms'].mean()
                max_lat = df_steady['total_latency_ms'].max()
                
                # NEW: Calculate steady state Plan and Update times
                avg_plan = df_steady['plan_ms'].mean()
                avg_upd  = df_steady['update_ms'].mean() # This will naturally be ~0.00
                
                # New Metrics
                avg_obs  = df_steady['obstacle_checks'].mean() if 'obstacle_checks' in df.columns else 0
                avg_rew  = df_steady['rewire_neighbor_searches'].mean() if 'rewire_neighbor_searches' in df.columns else 0
                avg_sz   = df_steady['tree_size'].max() if 'tree_size' in df.columns else 0
                avg_deg_out = df_event['avg_deg_out'].max() if 'avg_deg_out' in df.columns else 0
                avg_deg_in  = df_event['avg_deg_in'].max() if 'avg_deg_in' in df.columns else 0
                
                # USING .max() FOR INITIAL PATH COST AS DISCUSSED
                init_cost = df_steady['path_cost'].max() if 'path_cost' in df.columns else 0
                
                # PRINT: Replaced the '--' with actual averages
                print(f"{planner:<10} | {'STEADY':<8} | {len(df_steady):<6} | "
                      f"\033[92m{avg_lat:<8.2f}\033[0m | {max_lat:<8.1f} | "
                      f"{avg_plan:<8.2f} | {avg_upd:<8.2f} | "
                      f"{avg_obs:<8.1f} | {avg_rew:<8.1f} | {avg_sz:<8.0f} | {avg_deg_out:<8.2f} | {avg_deg_in:<8.2f} | {avg_cost:<9.2f}")
            
            print("-" * 140)
            
    else:
        print("Old data format detected. Please re-run simulation with new C++ code.")

def main():
    scenarios = load_data(BUILD_DIR)
    for scenario_name, results in scenarios.items():
        results.sort(key=lambda x: x['planner'])
        analyze_group("DETAILED", results, scenario_name)

if __name__ == "__main__":
    main()
