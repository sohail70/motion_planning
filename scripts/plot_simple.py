# import pandas as pd
# import matplotlib.pyplot as plt
# import glob
# import os
# import re
# import numpy as np
# from scipy import stats
# from matplotlib.ticker import AutoLocator, AutoMinorLocator

# def get_planner_name(filename):
#     """Enhanced to extract planner type AND sample count from filename"""
#     base = os.path.basename(filename)
    
#     # Extract planner type
#     if 'fmtx' in base.lower():
#         planner = 'FMTx'
#     elif 'rrtx' in base.lower():
#         planner = 'RRTx'
#     else:
#         planner = os.path.splitext(base)[0]  # fallback
    
#     # Extract sample count (look for pattern like "1000samples")
#     samples = "N/A"
#     match = re.search(r'(\d+)samples', base.lower())
#     if match:
#         samples = match.group(1)
    
#     return planner, int(samples) if samples != "N/A" else 0

# def plot_time_series_comparison(data_by_samples):
#     """
#     Plot a time-series comparison with a smoothed moving average.
#     """
#     sample_sizes = sorted(data_by_samples.keys())
#     if not sample_sizes:
#         print("No valid data found for time-series plot.")
#         return
        
#     fig, axes = plt.subplots(2, 2, figsize=(15, 10), sharex=False, sharey=False)
#     fig.suptitle("Planner Performance Over Time by Sample Count", fontsize=20, fontweight='bold')

#     for ax, samples in zip(axes.flat, sample_sizes):
#         dfs = data_by_samples.get(samples, {})
#         max_y = 0
        
#         for planner, df in dfs.items():
#             if 'elapsed_s' in df.columns and 'duration_ms' in df.columns:
#                 xs = df['elapsed_s']
#                 ys = df['duration_ms']
                
#                 # Plot the raw, "ragged" data with some transparency
#                 ax.plot(xs, ys, label=f"{planner} (Raw)", alpha=0.3)
                
#                 # Calculate and plot a smoothed moving average
#                 window_size = max(1, len(ys) // 20) # Adjust window size for smoothness
#                 ys_smooth = ys.rolling(window=window_size, center=True, min_periods=1).mean()
#                 ax.plot(xs, ys_smooth, label=f"{planner} (Smoothed)", linewidth=2.5)

#                 max_y = max(max_y, ys.quantile(0.99)) # Use 99th percentile for better y-limit
        
#         ax.set_title(f"{samples:,} Samples", fontsize=14, fontweight='bold')
#         ax.set_xlabel("Simulation Time (s)", fontsize=12)
#         ax.set_ylabel("Update Duration (ms)", fontsize=12)
#         ax.grid(True, which='both', linestyle='--', linewidth=0.5)
#         ax.set_ylim(0, max_y * 1.15)
#         ax.yaxis.set_major_locator(plt.MaxNLocator(nbins=8))
#         ax.yaxis.set_minor_locator(AutoMinorLocator())
#         ax.tick_params(axis='y', which='minor', length=3, color='gray')
#         ax.legend(loc='upper left', fontsize='medium')

#     # Handle empty subplots
#     num_plots = len(sample_sizes)
#     for i in range(num_plots, 4):
#         axes.flat[i].set_visible(False)

#     plt.tight_layout(pad=3.0, rect=[0, 0, 1, 0.96])
#     plot_filename = "planner_timeseries_comparison.pdf"
#     plt.savefig(plot_filename, format="pdf", bbox_inches="tight")
#     print(f"\nTime-series plot saved to '{plot_filename}'")
#     plt.show()

# def plot_statistical_distribution(data_by_samples):
#     """
#     Generate a box plot to compare the distribution of update times.
#     This is often more insightful than a time-series plot for this kind of data.
#     """
#     sample_sizes = sorted(data_by_samples.keys())
#     if not sample_sizes:
#         print("No valid data found for statistical plot.")
#         return

#     fig, axes = plt.subplots(1, len(sample_sizes), figsize=(5 * len(sample_sizes), 7), sharey=True)
#     if len(sample_sizes) == 1: # Ensure axes is always iterable
#         axes = [axes]
        
#     fig.suptitle("Statistical Distribution of Planner Update Times", fontsize=20, fontweight='bold')

#     for ax, samples in zip(axes, sample_sizes):
#         planners_data = data_by_samples.get(samples, {})
        
#         plot_data = [df['duration_ms'] for df in planners_data.values()]
#         labels = [name for name in planners_data.keys()]
        
#         box = ax.boxplot(plot_data, labels=labels, patch_artist=True, showfliers=False) # showfliers=False to hide outliers for clarity
        
#         colors = ['lightblue', 'lightgreen']
#         for patch, color in zip(box['boxes'], colors):
#             patch.set_facecolor(color)
            
#         ax.set_title(f"{samples:,} Samples", fontsize=14, fontweight='bold')
#         ax.set_ylabel("Update Duration (ms)", fontsize=12)
#         ax.grid(True, axis='y', linestyle='--', linewidth=0.5)

#     plt.tight_layout(rect=[0, 0, 1, 0.95])
#     plot_filename = "planner_statistical_comparison.pdf"
#     plt.savefig(plot_filename, format="pdf", bbox_inches="tight")
#     print(f"Statistical distribution plot saved to '{plot_filename}'")
#     plt.show()


# def perform_all_statistical_analyses(all_data):
#     """Compare all FMTx vs RRTx pairs with matching sample sizes"""
#     sample_groups = {}
#     for planner, samples, df in all_data:
#         if samples not in sample_groups:
#             sample_groups[samples] = {}
#         sample_groups[samples][planner] = df
    
#     for samples, planners in sorted(sample_groups.items()):
#         if len(planners) >= 2 and 'FMTx' in planners and 'RRTx' in planners:
#             print(f"\n{'='*55}")
#             print(f" STATISTICAL COMPARISON FOR {samples:,} SAMPLES")
#             print(f"{'='*55}")
            
#             fmtx_data = ('FMTx', planners['FMTx'])
#             rrtx_data = ('RRTx', planners['RRTx'])
#             perform_statistical_analysis(fmtx_data, rrtx_data)

# def perform_statistical_analysis(planner_a, planner_b):
#     """Compare two planners statistically"""
#     name_a, data_a = planner_a
#     name_b, data_b = planner_b
    
#     durations_a = data_a['duration_ms']
#     durations_b = data_b['duration_ms']
    
#     print(f"\n--- {name_a} vs {name_b} Performance ---")
#     print(f"{'Metric':<15} {name_a:<12} {name_b:<12}")
#     print("-" * 42)
    
#     stats_data = [
#         ("Median", np.median),
#         ("Mean", np.mean),
#         ("Std Dev", np.std),
#         ("95th %ile", lambda x: np.percentile(x, 95))
#     ]
    
#     for stat_name, func in stats_data:
#         val_a = f"{func(durations_a):.2f} ms"
#         val_b = f"{func(durations_b):.2f} ms"
#         print(f"{stat_name:<15} {val_a:<12} {val_b:<12}")
    
#     try:
#         stat, p = stats.mannwhitneyu(durations_a, durations_b, alternative='two-sided')
#         print(f"\nStatistical Test (Mann-Whitney U):")
#         print(f"  - U-statistic = {stat:.1f}, p-value = {p:.6f}")
#         if p < 0.001:
#             print("  - Result: STRONG evidence of a significant difference (p < 0.001)")
#         elif p < 0.05:
#             print("  - Result: Evidence of a significant difference (p < 0.05)")
#         else:
#             print("  - Result: No significant difference detected (p >= 0.05)")
        
#         median_a = np.median(durations_a)
#         median_b = np.median(durations_b)
#         if median_a > 0 and median_b > 0:
#             if median_a < median_b:
#                 ratio = median_b / median_a
#                 print(f"\n  - Speed Ratio: {name_a} is {ratio:.2f}x faster than {name_b} (based on median)")
#             else:
#                 ratio = median_a / median_b
#                 print(f"\n  - Speed Ratio: {name_b} is {ratio:.2f}x faster than {name_a} (based on median)")
#     except ValueError as e:
#         print(f"\nCould not perform statistical test: {e}")

# def main():
#     search_path = "../build/sim_*_*.csv"
#     files = sorted(glob.glob(search_path))
    
#     if not files:
#         print(f"No data files found in '{os.path.abspath('../build/')}'")
#         print("Please ensure your C++ application is saving CSV files with names like:")
#         print("  - sim_fmtx_5000samples_..._timed.csv")
#         print("  - sim_rrtx_10000samples_..._timed.csv")
#         return
    
#     print(f"Found {len(files)} log files to analyze.")
    
#     # --- Data Loading and Grouping ---
#     all_data = []
#     data_by_samples = {}
#     for filename in files:
#         try:
#             planner, samples = get_planner_name(filename)
#             df = pd.read_csv(filename)
#             if 'duration_ms' not in df.columns:
#                 print(f"Skipping {filename}: missing 'duration_ms' column.")
#                 continue
#             all_data.append((planner, samples, df))
#             data_by_samples.setdefault(samples, {})[planner] = df
#         except Exception as e:
#             print(f"Error processing {filename}: {e}")
            
#     # --- Generate Visualizations and Analysis ---
#     plot_time_series_comparison(data_by_samples)
#     plot_statistical_distribution(data_by_samples)
#     perform_all_statistical_analyses(all_data)

# if __name__ == "__main__":
#     main()










#####################################################################
import pandas as pd
import matplotlib.pyplot as plt
import glob
import os
import re
import numpy as np
from scipy import stats
from matplotlib.ticker import AutoLocator, AutoMinorLocator

def get_planner_info(filename):
    """
    Extracts planner type, sample count, and factor from a filename.
    Example: sim_rrtx_5000samples_factor2.0_..._timed.csv -> ('RRTx', 5000, 2.0)
    """
    base = os.path.basename(filename)
    
    planner_match = re.search(r'sim_(fmtx|rrtx)', base.lower())
    samples_match = re.search(r'(\d+)samples', base.lower())
    factor_match = re.search(r'factor([\d\.]+)', base.lower()) # Updated to capture float

    planner = planner_match.group(1).upper() if planner_match else 'Unknown'
    samples = int(samples_match.group(1)) if samples_match else 0
    factor = float(factor_match.group(1)) if factor_match else 0.0
    
    return planner, samples, factor

def aggregate_run_data(files):
    """
    Loads all CSV files and aggregates them by planner configuration.
    Returns a dictionary: {(planner, samples, factor): concatenated_dataframe}
    """
    aggregated_data = {}
    print("--- Loading and Aggregating Data ---")
    for filename in files:
        try:
            planner, samples, factor = get_planner_info(filename)
            if samples == 0:
                print(f"Warning: Could not determine sample count for {filename}. Skipping.")
                continue

            df = pd.read_csv(filename)
            if 'duration_ms' not in df.columns:
                print(f"Skipping {filename}: missing 'duration_ms' column.")
                continue
            
            # Create a unique key for each experimental setup
            config_key = (planner, samples, factor)
            
            # If this is the first file for this config, create a new list
            if config_key not in aggregated_data:
                aggregated_data[config_key] = []
            
            # Append the dataframe to the list for this config
            aggregated_data[config_key].append(df)
            
        except Exception as e:
            print(f"Error processing {filename}: {e}")

    # Concatenate the lists of dataframes into single dataframes
    final_data = {key: pd.concat(dfs, ignore_index=True) for key, dfs in aggregated_data.items()}
    print(f"Successfully aggregated data for {len(final_data)} configurations.")
    return final_data

def plot_statistical_distribution(aggregated_data):
    """
    Generates a box plot to compare the distribution of update times for each configuration.
    """
    # Group data by sample and factor size for plotting
    plot_groups = {}
    for (planner, samples, factor), df in aggregated_data.items():
        group_key = (samples, factor)
        if group_key not in plot_groups:
            plot_groups[group_key] = {}
        plot_groups[group_key][planner] = df['duration_ms']

    if not plot_groups:
        print("No data to plot for statistical distribution.")
        return

    num_groups = len(plot_groups)
    fig, axes = plt.subplots(1, num_groups, figsize=(6 * num_groups, 8), sharey=True, squeeze=False)
    fig.suptitle("Statistical Distribution of Planner Update Times", fontsize=20, fontweight='bold')

    for i, ((samples, factor), planners_data) in enumerate(sorted(plot_groups.items())):
        ax = axes[0, i]
        
        plot_data = []
        labels = []
        # Ensure consistent order for plotting
        if 'RRTX' in planners_data:
            plot_data.append(planners_data['RRTX'])
            labels.append('RRTx')
        if 'FMTX' in planners_data:
            plot_data.append(planners_data['FMTX'])
            labels.append('FMTx')
            
        if not plot_data:
            continue

        box = ax.boxplot(plot_data, labels=labels, patch_artist=True, showfliers=False, medianprops={'color':'red', 'linewidth':2})
        
        colors = ['lightblue', 'lightgreen']
        for patch, color in zip(box['boxes'], colors):
            patch.set_facecolor(color)
            
        ax.set_title(f"{samples:,} Samples | Factor: {factor}", fontsize=14, fontweight='bold')
        if i == 0:
            ax.set_ylabel("Update Duration (ms)", fontsize=12)
        ax.grid(True, axis='y', linestyle='--', linewidth=0.5)

    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plot_filename = "planner_statistical_comparison.pdf"
    plt.savefig(plot_filename, format="pdf", bbox_inches="tight")
    print(f"\nStatistical distribution plot saved to '{plot_filename}'")
    plt.show()

def plot_time_series_with_average(aggregated_data):
    """
    Generates a time-series plot comparing planner performance,
    including a horizontal line for the average update time.
    """
    # Group data by sample and factor size for plotting
    plot_groups = {}
    for (planner, samples, factor), df in aggregated_data.items():
        group_key = (samples, factor)
        if group_key not in plot_groups:
            plot_groups[group_key] = {}
        plot_groups[group_key][planner] = df

    if not plot_groups:
        print("No data to plot for time-series analysis.")
        return

    num_groups = len(plot_groups)
    fig, axes = plt.subplots(1, num_groups, figsize=(8 * num_groups, 7), sharey=True, squeeze=False)
    fig.suptitle("Time-Series of Planner Update Durations", fontsize=20, fontweight='bold')

    for i, ((samples, factor), planners_data) in enumerate(sorted(plot_groups.items())):
        ax = axes[0, i]
        max_y = 0

        # Use a consistent color map
        colors = {'FMTX': 'green', 'RRTX': 'blue'}
        
        # Sort planners to ensure consistent plotting order
        for planner_name in sorted(planners_data.keys()):
            df = planners_data[planner_name]
            color = colors.get(planner_name, 'gray')
            
            if 'elapsed_s' in df.columns and 'duration_ms' in df.columns:
                # Plot raw data as scatter points for clarity
                ax.plot(df['elapsed_s'], df['duration_ms'], 'o', color=color, markersize=3, alpha=0.2, label=f'{planner_name} (Raw Data)')

                # Calculate and plot the average as a horizontal line
                mean_duration = df['duration_ms'].mean()
                ax.axhline(y=mean_duration, color=color, linestyle='--', linewidth=2.5, label=f'{planner_name} Avg: {mean_duration:.2f} ms')
                
                # Use 98th percentile for a more robust y-axis limit that ignores extreme outliers
                max_y = max(max_y, df['duration_ms'].quantile(0.98)) 
        
        ax.set_title(f"{samples:,} Samples | Factor: {factor}", fontsize=14, fontweight='bold')
        ax.set_xlabel("Simulation Time (s)", fontsize=12)
        if i == 0:
            ax.set_ylabel("Update Duration (ms)", fontsize=12)
        ax.grid(True, which='major', linestyle='--', linewidth=0.7)
        ax.legend(loc='upper right', fontsize='medium')
        ax.set_ylim(0, max_y * 1.15) # Set a common y-limit based on the data

    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plot_filename = "planner_timeseries_average_comparison.pdf"
    plt.savefig(plot_filename, format="pdf", bbox_inches="tight")
    print(f"\nTime-series plot with averages saved to '{plot_filename}'")
    plt.show()

def perform_statistical_analysis(aggregated_data):
    """Performs and prints statistical analysis for each configuration."""
    plot_groups = {}
    for (planner, samples, factor), df in aggregated_data.items():
        group_key = (samples, factor)
        if group_key not in plot_groups:
            plot_groups[group_key] = {}
        plot_groups[group_key][planner] = df['duration_ms']

    for (samples, factor), planners_data in sorted(plot_groups.items()):
        if 'FMTX' in planners_data and 'RRTX' in planners_data:
            print(f"\n{'='*60}")
            print(f" STATISTICAL COMPARISON: {samples:,} SAMPLES | FACTOR: {factor}")
            print(f"{'='*60}")
            
            durations_a = planners_data['FMTX']
            durations_b = planners_data['RRTX']
            
            print(f"{'Metric':<15} {'FMTx':<15} {'RRTx':<15}")
            print("-" * 47)
            
            stats_to_run = [
                ("Median", np.median),
                ("Mean", np.mean),
                ("Std Dev", np.std),
                ("95th %ile", lambda x: np.percentile(x, 95)),
                ("Max", np.max)
            ]
            
            for stat_name, func in stats_to_run:
                val_a = f"{func(durations_a):.2f} ms"
                val_b = f"{func(durations_b):.2f} ms"
                print(f"{stat_name:<15} {val_a:<15} {val_b:<15}")
            
            try:
                # Mann-Whitney U test is robust for non-normal distributions
                stat, p = stats.mannwhitneyu(durations_a, durations_b, alternative='two-sided')
                print(f"\nStatistical Test (Mann-Whitney U):")
                print(f"  - U-statistic = {stat:.1f}, p-value = {p:.6f}")
                if p < 0.001:
                    print("  - Result: STRONG evidence of a significant difference.")
                elif p < 0.05:
                    print("  - Result: Evidence of a significant difference.")
                else:
                    print("  - Result: No significant difference detected.")
                
                median_a = np.median(durations_a)
                median_b = np.median(durations_b)
                if median_a > 1e-6 and median_b > 1e-6:
                    if median_a < median_b:
                        ratio = median_b / median_a
                        print(f"  - Conclusion: FMTx is ~{ratio:.2f}x faster than RRTx (based on median).")
                    else:
                        ratio = median_a / median_b
                        print(f"  - Conclusion: RRTx is ~{ratio:.2f}x faster than FMTx (based on median).")
            except ValueError as e:
                print(f"\nCould not perform statistical test: {e}")

def main():
    """Main function to load data, generate plots, and run analysis."""
    # The search path should point to the directory where your build files are.
    search_path = "../build/sim_*_timed.csv"
    files = sorted(glob.glob(search_path))
    
    if not files:
        print(f"Error: No data files found matching the pattern in '{os.path.abspath('../build/')}'")
        print("Please ensure your C++ application is running and saving CSV files with names like:")
        print("  - sim_fmtx_5000samples_factor2.0_..._timed.csv")
        print("  - sim_rrtx_5000samples_factor2.0_..._timed.csv")
        return
    
    # 1. Load and aggregate all data from all runs
    aggregated_data = aggregate_run_data(files)
    
    if not aggregated_data:
        print("No valid data could be aggregated. Exiting.")
        return

    # 2. Generate plots
    plot_statistical_distribution(aggregated_data)
    plot_time_series_with_average(aggregated_data) # <-- Added this new plot
    
    # 3. Perform and print statistical analysis
    perform_statistical_analysis(aggregated_data)

if __name__ == "__main__":
    # Set a professional plot style
    plt.style.use('seaborn-v0_8-whitegrid')
    main()