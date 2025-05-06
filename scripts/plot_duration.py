import glob
import os
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from scipy import stats

def get_planner_name(filename):
    """Extract planner type from filename"""
    base = os.path.basename(filename)
    if 'fmtx' in base.lower():
        return 'FMTx'
    elif 'rrtx' in base.lower():
        return 'RRTX'
    else:
        return os.path.splitext(base)[0]  # fallback

def plot_comparison(csv_files):
    """Plot and compare planner performance"""
    plt.figure(figsize=(12, 6))
    cmap = plt.get_cmap('tab10')
    
    # Load and plot all files
    all_data = []
    for i, filename in enumerate(csv_files):
        try:
            df = pd.read_csv(filename)
            planner = get_planner_name(filename)
            color = cmap(i)
            
            if 'elapsed_s' in df.columns:  # Timed data
                plt.plot(df['elapsed_s'], df['duration_ms'], 
                         color=color, label=planner, alpha=0.7)
                plt.xlabel("Time (seconds)")
            else:  # Raw data
                plt.plot(df.iloc[:, 0],  # First column as durations
                         color=color, label=planner, alpha=0.7)
                plt.xlabel("Iteration")
            
            all_data.append((planner, df))
            
        except Exception as e:
            print(f"Error with {filename}: {e}")

    plt.ylabel("Duration (ms)")
    plt.title("Planner Performance Comparison")
    plt.legend()
    plt.grid(True)
    plt.show()
    
    # Statistical analysis if we have exactly 2 planners
    if len(all_data) == 2:
        perform_statistical_analysis(all_data[0], all_data[1])

def perform_statistical_analysis(planner_a, planner_b):
    """Compare two planners statistically"""
    name_a, data_a = planner_a
    name_b, data_b = planner_b
    
    # Get duration data (handle both raw and timed formats)
    durations_a = data_a['duration_ms'] if 'duration_ms' in data_a.columns else data_a.iloc[:, 0]
    durations_b = data_b['duration_ms'] if 'duration_ms' in data_b.columns else data_b.iloc[:, 0]
    
    print(f"\n=== {name_a} vs {name_b} Performance ===")
    print(f"{'Metric':<15} {name_a:<10} {name_b:<10}")
    print("-"*40)
    
    stats_data = [
        ("Median", np.median),
        ("Mean", np.mean),
        ("Std Dev", np.std),
        ("95%ile", lambda x: np.percentile(x, 95))
    ]
    
    for stat_name, func in stats_data:
        val_a = f"{func(durations_a):.1f} ms"
        val_b = f"{func(durations_b):.1f} ms"
        print(f"{stat_name:<15} {val_a:<10} {val_b:<10}")
    
    # Mann-Whitney U test
    stat, p = stats.mannwhitneyu(durations_a, durations_b)
    print(f"\nStatistical Test:")
    print(f"U = {stat:.1f}, p = {p:.6f}")
    if p < 0.001:
        print("STRONG significant difference (p < 0.001)")
    elif p < 0.05:
        print("Significant difference (p < 0.05)")
    
    # Speed ratio
    ratio = np.median(durations_b) / np.median(durations_a)
    print(f"\n{name_a} is {ratio:.1f}x faster than {name_b}")

def main():
    # Find all data files (both formats)
    files = sorted(glob.glob("../build/sim_*_*.csv"))
    
    if not files:
        print("No data files found in ../build/")
        print("Expected files like:")
        print("sim_fmtx_*_timed.csv, sim_rrtx_*_raw.csv")
        return
    
    plot_comparison(files)

if __name__ == "__main__":
    main()
