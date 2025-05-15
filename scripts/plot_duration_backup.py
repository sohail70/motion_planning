import glob
import os
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from scipy import stats
import re
from itertools import combinations
from matplotlib.ticker import AutoLocator, AutoMinorLocator, MultipleLocator

def get_planner_name(filename):
    """Enhanced to extract planner type AND sample count from filename"""
    base = os.path.basename(filename)
    
    # Extract planner type
    if 'fmtx' in base.lower():
        planner = 'FMTx'
    elif 'rrtx' in base.lower():
        planner = 'RRTx'
    else:
        planner = os.path.splitext(base)[0]  # fallback
    
    # Extract sample count (look for pattern like "1000samples")
    samples = "N/A"
    match = re.search(r'(\d+)samples_', base.lower())
    if match:
        samples = match.group(1)
    
    return planner, int(samples) if samples != "N/A" else 0

def plot_comparison(csv_files):
    """Plot and compare planner performance"""
    plt.figure(figsize=(12, 6))
    cmap = plt.get_cmap('tab10')
    
    # Group files by sample count
    sample_groups = {}
    for filename in csv_files:
        try:
            planner, samples = get_planner_name(filename)
            if samples not in sample_groups:
                sample_groups[samples] = []
            sample_groups[samples].append((planner, filename))
        except Exception as e:
            print(f"Error processing {filename}: {e}")

    # Plot all files with consistent colors per planner
    color_map = {'FMTx': 'blue', 'RRTx': 'orange'}
    all_data = []
    
    for samples, group in sample_groups.items():
        for planner, filename in group:
            try:
                df = pd.read_csv(filename)
                label = f"{planner} ({samples} samples)"
                color = color_map.get(planner, 'gray')
                
                if 'elapsed_s' in df.columns:
                    line = plt.plot(df['elapsed_s'], df['duration_ms'], 
                                  color=color, label=label, alpha=0.7)
                else:
                    line = plt.plot(df.iloc[:, 0],
                                  color=color, label=label, alpha=0.7)
                
                all_data.append((planner, samples, df))
                
            except Exception as e:
                print(f"Error with {filename}: {e}")

    data_by_samples = {}
    for planner, samples, df in all_data:
        data_by_samples.setdefault(samples, {})[planner] = df

    # 2×2 grid for [1k, 5k, 10k, 50k] samples
    sample_sizes = sorted(data_by_samples.keys())  # e.g. [1000, 5000, 10000, 50000]
    fig, axes = plt.subplots(2, 2, figsize=(12, 8), sharex=False, sharey=False)

    for ax, samples in zip(axes.flat, sample_sizes):
        dfs = data_by_samples[samples]
        max_y = 0
        for planner, df in dfs.items():
            xs = df['elapsed_s']
            ys = df['duration_ms']
            ax.plot(xs, ys, label=planner)
            # ax.scatter(xs, ys, s=10, alpha=0.6)
            max_y = max(max_y, ys.max())
        ax.set_title(f"{samples:,} samples")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Replanning Duration (ms)")
        ax.grid(True)

        # Optionally pad the top 10%
        ax.set_ylim(0, max_y * 1.1)
        # Set finer y-tick granularity
        tick_step = 50  # or 20/10 depending on your data scale

        # Increase the number of major ticks
        ax.yaxis.set_major_locator(plt.MaxNLocator(nbins=12))

        # Add minor ticks for more granularity
        ax.yaxis.set_minor_locator(AutoMinorLocator())
        ax.tick_params(axis='y', which='minor', length=4, color='gray')


        ax.legend(loc='upper left', fontsize='small')

    plt.tight_layout()
    # plt.savefig("planner_comparison.svg", format="svg", bbox_inches="tight")
    # plt.savefig("planner_comparison.eps", format="eps", bbox_inches="tight")
    plt.savefig("planner_comparison.pdf", format="pdf", bbox_inches="tight")

    plt.show()

    # plt.xlabel("Time (seconds)" if 'elapsed_s' in df.columns else "Iteration")
    # plt.ylabel("Duration (ms)")
    # plt.title("Planner Performance Comparison")
    # plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    # plt.grid(True)
    # plt.tight_layout()
    # plt.show()
    
    # Perform statistical comparisons for each sample size
    perform_all_statistical_analyses(all_data)

def perform_all_statistical_analyses(all_data):
    """Compare all FMTx vs RRTx pairs with matching sample sizes"""
    # Group by sample count
    sample_groups = {}
    for planner, samples, df in all_data:
        if samples not in sample_groups:
            sample_groups[samples] = {}
        sample_groups[samples][planner] = df
    
    # Compare pairs within each sample group
    for samples, planners in sample_groups.items():
        if len(planners) >= 2 and 'FMTx' in planners and 'RRTx' in planners:
            print(f"\n{'='*50}")
            print(f"COMPARISON FOR {samples} SAMPLES")
            print(f"{'='*50}")
            
            fmtx_data = ('FMTx', planners['FMTx'])
            rrtx_data = ('RRTx', planners['RRTx'])
            perform_statistical_analysis(fmtx_data, rrtx_data)

def perform_statistical_analysis(planner_a, planner_b):
    """Compare two planners statistically"""
    name_a, data_a = planner_a
    name_b, data_b = planner_b
    
    # Get duration data
    durations_a = data_a['duration_ms'] if 'duration_ms' in data_a.columns else data_a.iloc[:, 0]
    durations_b = data_b['duration_ms'] if 'duration_ms' in data_b.columns else data_b.iloc[:, 0]
    
    print(f"\n=== {name_a} vs {name_b} Performance ===")
    print(f"{'Replan Metric':<15} {name_a:<10} {name_b:<10}")
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
