import pandas as pd
import matplotlib.pyplot as plt
import glob
import os
import re
import numpy as np

# --- CONFIGURATION ---
BUILD_DIR = "../build/"  # Directory containing the CSV files
OUTPUT_DIR = "./analysis_results/"  # Where to save plots

# Regex to parse filenames
# Expected format: sim_PLANNERNAME_SCENARIONAME_DATE_TIME_metrics.csv
# Example: sim_ANYFMTX_R2_Geometric_Test_20260212_135708_metrics.csv
FILENAME_PATTERN = re.compile(r"sim_([A-Za-z0-9]+)_([A-Za-z0-9_]+)_(\d{8}_\d{6})_metrics\.csv")

# Planners to compare colors for consistency
PLANNER_COLORS = {
    "ANYFMTX": "blue",
    "KinodynamicANYFMTX": "blue",
    "ANYRRTX": "red",
    "KinodynamicANYRRTX": "red",
    "RRTX": "orange",
    "KinodynamicRRTX": "orange",
    "FMTX": "cyan",
    "KinodynamicFMTX": "cyan",
    "PRMSTARDSTARLITE": "green",
    "KinodynamicPRMStarDStarLite": "green"
}

def parse_files(directory):
    data_registry = {}  # Key: Scenario Name, Value: List of (Planner, DataFrame)

    files = glob.glob(os.path.join(directory, "sim_*_metrics.csv"))
    print(f"Found {len(files)} metric files.")

    for filepath in files:
        filename = os.path.basename(filepath)
        match = FILENAME_PATTERN.match(filename)
        
        if match:
            planner_name = match.group(1)
            scenario_name = match.group(2)
            timestamp = match.group(3)
            
            # Clean up Planner Name (Remove 'Kinodynamic' prefix if inconsistent)
            if planner_name.startswith("Kinodynamic"):
                planner_name = planner_name.replace("Kinodynamic", "")

            try:
                df = pd.read_csv(filepath)
                if df.empty:
                    print(f"Skipping empty file: {filename}")
                    continue
                
                # Basic data cleaning
                df.replace([np.inf, -np.inf], np.nan, inplace=True)
                # Fill path cost gaps (if cost is missing, forward fill)
                df['path_cost'] = df['path_cost'].ffill() 

                if scenario_name not in data_registry:
                    data_registry[scenario_name] = []
                
                data_registry[scenario_name].append({
                    "planner": planner_name,
                    "timestamp": timestamp,
                    "data": df,
                    "filename": filename
                })
            except Exception as e:
                print(f"Error reading {filename}: {e}")
        else:
            print(f"Skipping unrecognized file format: {filename}")

    return data_registry

def plot_scenario_comparison(scenario_name, results_list):
    print(f"Generating plots for scenario: {scenario_name}")
    
    # Metrics to plot
    metrics = [
        ("duration_ms", "Update/Plan Time (ms)", "Latency"),
        ("path_cost", "Path Cost", "Solution Quality"),
        ("obstacle_checks", "Cumulative Obstacle Checks", "Work Done"),
        ("tree_size", "Number of Nodes", "Exploration/Tree Growth")
    ]

    fig, axes = plt.subplots(2, 2, figsize=(16, 12))
    fig.suptitle(f"Benchmark Comparison: {scenario_name}", fontsize=16)
    
    # Flatten axes for easy iteration
    axes_flat = axes.flatten()

    for idx, (col_name, y_label, title) in enumerate(metrics):
        ax = axes_flat[idx]
        
        for entry in results_list:
            planner = entry['planner']
            df = entry['data']
            
            # Use 'elapsed_s' as X-axis (Simulation Time)
            x_data = df['elapsed_s']
            y_data = df[col_name]

            # Smoothing for noisy duration data (Moving Average)
            if col_name == "duration_ms":
                y_data = y_data.rolling(window=5, min_periods=1).mean()
                alpha = 0.7
                linewidth = 1.5
            else:
                alpha = 1.0
                linewidth = 2.0

            color = PLANNER_COLORS.get(planner, "gray")
            
            ax.plot(x_data, y_data, label=f"{planner}", color=color, alpha=alpha, linewidth=linewidth)

        ax.set_title(title)
        ax.set_xlabel("Simulation Time (s)")
        ax.set_ylabel(y_label)
        ax.grid(True, linestyle='--', alpha=0.6)
        ax.legend()

    # Special handling for Cost Plot to ignore Inf
    axes_flat[1].set_ylim(bottom=0) 
    
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    
    # Save
    save_path = os.path.join(OUTPUT_DIR, f"{scenario_name}_comparison.png")
    plt.savefig(save_path)
    print(f"Saved: {save_path}")
    plt.close()
def generate_summary_table(results_list):
    summary_data = []
    
    for entry in results_list:
        df = entry['data']
        planner = entry['planner']
        
        # Calculate Basic Statistics
        avg_update_time = df['duration_ms'].mean()
        final_cost = df['path_cost'].iloc[-1] if not df['path_cost'].empty else np.nan
        final_tree_size = df['tree_size'].iloc[-1] if 'tree_size' in df.columns else 0
        
        total_checks = df['obstacle_checks'].sum()
        total_orphans = df['orphaned_nodes'].sum()
        
        # --- Efficiency Metrics ---
        
        # 1. Work per Node (How expensive is it to maintain one node?)
        work_per_node = total_checks / final_tree_size if final_tree_size > 0 else 0
        
        # 2. Resilience (How many nodes were lost vs how many were kept?)
        # A lower ratio suggests the tree is more stable during obstacle movements.
        orphan_ratio = total_orphans / final_tree_size if final_tree_size > 0 else 0

        summary_data.append({
            "Planner": planner,
            "Avg Update (ms)": f"{avg_update_time:.2f}",
            "Final Cost": f"{final_cost:.2f}",
            "Tree Size": int(final_tree_size),
            "Work/Node": f"{work_per_node:.2f}",
            "Orphan Ratio": f"{orphan_ratio:.2f}"
        })
        
    return pd.DataFrame(summary_data)

def main():
    if not os.path.exists(OUTPUT_DIR):
        os.makedirs(OUTPUT_DIR)

    data_map = parse_files(BUILD_DIR)

    if not data_map:
        print("No valid CSV files found in directory!")
        return

    for scenario, results in data_map.items():
        # Sort results by planner name for consistent plotting order
        results.sort(key=lambda x: x['planner'])
        
        # Generate Plots
        plot_scenario_comparison(scenario, results)
        
        # Print Summary Table to Console
        print(f"\n--- Summary Table: {scenario} ---")
        summary_df = generate_summary_table(results)
        print(summary_df.to_string(index=False))
        print("-------------------------------------------------\n")

if __name__ == "__main__":
    main()
