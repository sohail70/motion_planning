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
    match = re.search(r'(\d+)samples_', base.lower())
    if match:
        samples = match.group(1)
    
    return planner, int(samples) if samples != "N/A" else 0

def plot_comparison(csv_files):
    """Plot and compare planner performance"""
    all_data = []
    
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

    for samples, group in sample_groups.items():
        for planner, filename in group:
            try:
                df = pd.read_csv(filename)
                all_data.append((planner, samples, df))
            except Exception as e:
                print(f"Error with {filename}: {e}")

    data_by_samples = {}
    for planner, samples, df in all_data:
        data_by_samples.setdefault(samples, {})[planner] = df

    # 2x2 grid for [1k, 5k, 10k, 50k] samples
    sample_sizes = sorted(data_by_samples.keys())
    if not sample_sizes:
        print("No valid data found to plot.")
        return
        
    fig, axes = plt.subplots(2, 2, figsize=(14, 9), sharex=False, sharey=False)

    for ax, samples in zip(axes.flat, sample_sizes):
        dfs = data_by_samples.get(samples, {})
        max_y = 0
        for planner, df in dfs.items():
            if 'elapsed_s' in df.columns and 'duration_ms' in df.columns:
                xs = df['elapsed_s']
                ys = df['duration_ms']
                ax.plot(xs, ys, label=planner, alpha=0.8)
                max_y = max(max_y, ys.max())
        
        ax.set_title(f"{samples:,} Samples", fontsize=14, fontweight='bold')
        ax.set_xlabel("Simulation Time (s)", fontsize=12)
        ax.set_ylabel("Update Duration (ms)", fontsize=12)
        ax.grid(True, which='both', linestyle='--', linewidth=0.5)

        # Optionally pad the top 10%
        if max_y > 0:
            ax.set_ylim(0, max_y * 1.1)

        # Increase the number of major ticks
        ax.yaxis.set_major_locator(plt.MaxNLocator(nbins=10))

        # Add minor ticks for more granularity
        ax.yaxis.set_minor_locator(AutoMinorLocator())
        ax.tick_params(axis='y', which='minor', length=3, color='gray')
        ax.legend(loc='upper left', fontsize='medium')

    # Handle empty subplots if there are fewer than 4 sample sizes
    num_plots = len(sample_sizes)
    for i in range(num_plots, 4):
        axes.flat[i].set_visible(False)

    plt.tight_layout(pad=3.0)
    fig.suptitle("Planner Performance by Sample Count", fontsize=20, fontweight='bold')
    
    # Save the plot to a file
    plot_filename = "planner_comparison_by_samples.pdf"
    plt.savefig(plot_filename, format="pdf", bbox_inches="tight")
    print(f"\nPlot saved to '{plot_filename}'")
    plt.show()
    
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
    
    # Get duration data
    durations_a = data_a['duration_ms'] if 'duration_ms' in data_a.columns else data_a.iloc[:, 0]
    durations_b = data_b['duration_ms'] if 'duration_ms' in data_b.columns else data_b.iloc[:, 0]
    
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
    
    # Mann-Whitney U test
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
        
        # Speed ratio
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
    # Find all data files in the specified directory
    # Adjust the path if your build directory is located elsewhere
    search_path = "../build/sim_*_*.csv"
    files = sorted(glob.glob(search_path))
    
    if not files:
        print(f"No data files found in '{os.path.abspath('../build/')}'")
        print("Please ensure your C++ application is saving CSV files with names like:")
        print("  - sim_fmtx_5000samples_..._timed.csv")
        print("  - sim_rrtx_10000samples_..._timed.csv")
        return
    
    print(f"Found {len(files)} log files to analyze.")
    plot_comparison(files)

if __name__ == "__main__":
    main()
