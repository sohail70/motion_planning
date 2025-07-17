import pandas as pd
import matplotlib.pyplot as plt
import glob
import os
import re
import numpy as np
from scipy import stats
from matplotlib.ticker import AutoLocator, AutoMinorLocator

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
    match = re.search(r'(\d+)samples', base.lower())
    if match:
        samples = match.group(1)
    
    return planner, int(samples) if samples != "N/A" else 0

def plot_time_series_comparison(data_by_samples):
    """
    Plot a time-series comparison with a smoothed moving average.
    """
    sample_sizes = sorted(data_by_samples.keys())
    if not sample_sizes:
        print("No valid data found for time-series plot.")
        return
        
    fig, axes = plt.subplots(2, 2, figsize=(15, 10), sharex=False, sharey=False)
    fig.suptitle("Planner Performance Over Time by Sample Count", fontsize=20, fontweight='bold')

    for ax, samples in zip(axes.flat, sample_sizes):
        dfs = data_by_samples.get(samples, {})
        max_y = 0
        
        for planner, df in dfs.items():
            if 'elapsed_s' in df.columns and 'duration_ms' in df.columns:
                xs = df['elapsed_s']
                ys = df['duration_ms']
                
                # Plot the raw, "ragged" data with some transparency
                ax.plot(xs, ys, label=f"{planner} (Raw)", alpha=0.3)
                
                # Calculate and plot a smoothed moving average
                window_size = max(1, len(ys) // 20) # Adjust window size for smoothness
                ys_smooth = ys.rolling(window=window_size, center=True, min_periods=1).mean()
                ax.plot(xs, ys_smooth, label=f"{planner} (Smoothed)", linewidth=2.5)

                max_y = max(max_y, ys.quantile(0.99)) # Use 99th percentile for better y-limit
        
        ax.set_title(f"{samples:,} Samples", fontsize=14, fontweight='bold')
        ax.set_xlabel("Simulation Time (s)", fontsize=12)
        ax.set_ylabel("Update Duration (ms)", fontsize=12)
        ax.grid(True, which='both', linestyle='--', linewidth=0.5)
        ax.set_ylim(0, max_y * 1.15)
        ax.yaxis.set_major_locator(plt.MaxNLocator(nbins=8))
        ax.yaxis.set_minor_locator(AutoMinorLocator())
        ax.tick_params(axis='y', which='minor', length=3, color='gray')
        ax.legend(loc='upper left', fontsize='medium')

    # Handle empty subplots
    num_plots = len(sample_sizes)
    for i in range(num_plots, 4):
        axes.flat[i].set_visible(False)

    plt.tight_layout(pad=3.0, rect=[0, 0, 1, 0.96])
    plot_filename = "planner_timeseries_comparison.pdf"
    plt.savefig(plot_filename, format="pdf", bbox_inches="tight")
    print(f"\nTime-series plot saved to '{plot_filename}'")
    plt.show()

def plot_statistical_distribution(data_by_samples):
    """
    Generate a box plot to compare the distribution of update times.
    This is often more insightful than a time-series plot for this kind of data.
    """
    sample_sizes = sorted(data_by_samples.keys())
    if not sample_sizes:
        print("No valid data found for statistical plot.")
        return

    fig, axes = plt.subplots(1, len(sample_sizes), figsize=(5 * len(sample_sizes), 7), sharey=True)
    if len(sample_sizes) == 1: # Ensure axes is always iterable
        axes = [axes]
        
    fig.suptitle("Statistical Distribution of Planner Update Times", fontsize=20, fontweight='bold')

    for ax, samples in zip(axes, sample_sizes):
        planners_data = data_by_samples.get(samples, {})
        
        plot_data = [df['duration_ms'] for df in planners_data.values()]
        labels = [name for name in planners_data.keys()]
        
        box = ax.boxplot(plot_data, labels=labels, patch_artist=True, showfliers=False) # showfliers=False to hide outliers for clarity
        
        colors = ['lightblue', 'lightgreen']
        for patch, color in zip(box['boxes'], colors):
            patch.set_facecolor(color)
            
        ax.set_title(f"{samples:,} Samples", fontsize=14, fontweight='bold')
        ax.set_ylabel("Update Duration (ms)", fontsize=12)
        ax.grid(True, axis='y', linestyle='--', linewidth=0.5)

    plt.tight_layout(rect=[0, 0, 1, 0.95])
    plot_filename = "planner_statistical_comparison.pdf"
    plt.savefig(plot_filename, format="pdf", bbox_inches="tight")
    print(f"Statistical distribution plot saved to '{plot_filename}'")
    plt.show()


def perform_all_statistical_analyses(all_data):
    """Compare all FMTx vs RRTx pairs with matching sample sizes"""
    sample_groups = {}
    for planner, samples, df in all_data:
        if samples not in sample_groups:
            sample_groups[samples] = {}
        sample_groups[samples][planner] = df
    
    for samples, planners in sorted(sample_groups.items()):
        if len(planners) >= 2 and 'FMTx' in planners and 'RRTx' in planners:
            print(f"\n{'='*55}")
            print(f" STATISTICAL COMPARISON FOR {samples:,} SAMPLES")
            print(f"{'='*55}")
            
            fmtx_data = ('FMTx', planners['FMTx'])
            rrtx_data = ('RRTx', planners['RRTx'])
            perform_statistical_analysis(fmtx_data, rrtx_data)

def perform_statistical_analysis(planner_a, planner_b):
    """Compare two planners statistically"""
    name_a, data_a = planner_a
    name_b, data_b = planner_b
    
    durations_a = data_a['duration_ms']
    durations_b = data_b['duration_ms']
    
    print(f"\n--- {name_a} vs {name_b} Performance ---")
    print(f"{'Metric':<15} {name_a:<12} {name_b:<12}")
    print("-" * 42)
    
    stats_data = [
        ("Median", np.median),
        ("Mean", np.mean),
        ("Std Dev", np.std),
        ("95th %ile", lambda x: np.percentile(x, 95))
    ]
    
    for stat_name, func in stats_data:
        val_a = f"{func(durations_a):.2f} ms"
        val_b = f"{func(durations_b):.2f} ms"
        print(f"{stat_name:<15} {val_a:<12} {val_b:<12}")
    
    try:
        stat, p = stats.mannwhitneyu(durations_a, durations_b, alternative='two-sided')
        print(f"\nStatistical Test (Mann-Whitney U):")
        print(f"  - U-statistic = {stat:.1f}, p-value = {p:.6f}")
        if p < 0.001:
            print("  - Result: STRONG evidence of a significant difference (p < 0.001)")
        elif p < 0.05:
            print("  - Result: Evidence of a significant difference (p < 0.05)")
        else:
            print("  - Result: No significant difference detected (p >= 0.05)")
        
        median_a = np.median(durations_a)
        median_b = np.median(durations_b)
        if median_a > 0 and median_b > 0:
            if median_a < median_b:
                ratio = median_b / median_a
                print(f"\n  - Speed Ratio: {name_a} is {ratio:.2f}x faster than {name_b} (based on median)")
            else:
                ratio = median_a / median_b
                print(f"\n  - Speed Ratio: {name_b} is {ratio:.2f}x faster than {name_a} (based on median)")
    except ValueError as e:
        print(f"\nCould not perform statistical test: {e}")

def main():
    search_path = "../build/sim_*_*.csv"
    files = sorted(glob.glob(search_path))
    
    if not files:
        print(f"No data files found in '{os.path.abspath('../build/')}'")
        print("Please ensure your C++ application is saving CSV files with names like:")
        print("  - sim_fmtx_5000samples_..._timed.csv")
        print("  - sim_rrtx_10000samples_..._timed.csv")
        return
    
    print(f"Found {len(files)} log files to analyze.")
    
    # --- Data Loading and Grouping ---
    all_data = []
    data_by_samples = {}
    for filename in files:
        try:
            planner, samples = get_planner_name(filename)
            df = pd.read_csv(filename)
            if 'duration_ms' not in df.columns:
                print(f"Skipping {filename}: missing 'duration_ms' column.")
                continue
            all_data.append((planner, samples, df))
            data_by_samples.setdefault(samples, {})[planner] = df
        except Exception as e:
            print(f"Error processing {filename}: {e}")
            
    # --- Generate Visualizations and Analysis ---
    plot_time_series_comparison(data_by_samples)
    plot_statistical_distribution(data_by_samples)
    perform_all_statistical_analyses(all_data)

if __name__ == "__main__":
    main()