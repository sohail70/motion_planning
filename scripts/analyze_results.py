import pandas as pd
import glob
import os
import re
import numpy as np
from scipy import stats

# --- CONFIGURATION ---
BUILD_DIR = "../build/"

FILENAME_PATTERN = re.compile(r"sim_([A-Za-z0-9]+)_([A-Za-z0-9_]+)_(\d{8}_\d{6})_metrics\.csv")

def parse_files(directory):
    data_registry = {}
    if not os.path.exists(directory):
        print(f"Error: Build directory '{directory}' does not exist.")
        return {}

    files = glob.glob(os.path.join(directory, "sim_*_metrics.csv"))
    print(f"Found {len(files)} metric files.")

    for filepath in files:
        filename = os.path.basename(filepath)
        match = FILENAME_PATTERN.match(filename)
        
        if match:
            planner_name = match.group(1)
            scenario_name = match.group(2)
            timestamp = match.group(3)
            
            if planner_name.startswith("Kinodynamic"):
                planner_name = planner_name.replace("Kinodynamic", "")

            try:
                df = pd.read_csv(filepath)
                if df.empty: continue
                df.replace([np.inf, -np.inf], np.nan, inplace=True)
                
                # Check for necessary columns
                required = ['avg_degree', 'tree_size', 'radius']
                missing = [col for col in required if col not in df.columns]
                if missing:
                    print(f"[WARN] File {filename} missing columns: {missing}. Skipping.")
                    continue

                if scenario_name not in data_registry:
                    data_registry[scenario_name] = []
                
                data_registry[scenario_name].append({
                    "planner": planner_name,
                    "data": df
                })
            except Exception as e:
                print(f"Error reading {filename}: {e}")
    return data_registry

def print_diagnostic_report(scenario, results_list):
    print(f"\n" + "="*95)
    print(f"DIAGNOSTIC REPORT: {scenario}")
    print("="*95)
    print(f"{'Planner':<15} | {'Nodes':<6} | {'Start Rad':<10} | {'End Rad':<10} | {'Avg Deg':<8} | {'Slope':<8} | {'Proof'}")
    print("-" * 95)

    for entry in results_list:
        df = entry['data'].sort_values('tree_size')
        planner = entry['planner']
        
        # Data points
        nodes = df['tree_size'].iloc[-1]
        start_rad = df['radius'].iloc[0]
        end_rad = df['radius'].iloc[-1]
        avg_deg = df['avg_degree'].iloc[-1]
        
        # Complexity Slope (Degree vs Tree Size)
        # We ignore initialization noise
        valid_df = df[df['tree_size'] > 20]
        if valid_df.empty:
            proof = "N/A (Too few nodes)"
            slope = 0.0
        else:
            slope, _, _, _, _ = stats.linregress(valid_df['tree_size'], valid_df['avg_degree'])
            
            # Proof logic
            # If degree increases linearly with N, it's O(N^2)
            if slope > 0.01:
                proof = f"!! O(N^2) - RADIUS NOT SHRINKING FAST !!"
            elif slope > 0.002:
                proof = "Degrading toward O(N^2)"
            else:
                proof = "O(N log N) - Correct Behavior"

        print(f"{planner:<15} | {int(nodes):<6} | {start_rad:<10.2f} | {end_rad:<10.2f} | {avg_deg:<8.2f} | {slope:<8.5f} | {proof}")
    
    print("-" * 95 + "\n")

def main():
    data_map = parse_files(BUILD_DIR)
    if not data_map:
        print("No valid CSV files found!")
        return

    for scenario, results in data_map.items():
        results.sort(key=lambda x: x['planner'])
        print_diagnostic_report(scenario, results)

if __name__ == "__main__":
    main()
