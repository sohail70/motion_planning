# # analyze_metrics.py

# import pandas as pd
# import matplotlib.pyplot as plt
# import glob
# import os
# import re
# import numpy as np
# from scipy import stats
# from matplotlib.ticker import EngFormatter

# def get_planner_info(filename):
#     """
#     Extracts planner type, sample count, and factor from a filename.
#     e.g., sim_rrtx_2000samples_20250720_225501_metrics.csv -> ('RRTX', 2000)
#     """
#     base = os.path.basename(filename)
    
#     # Use re.IGNORECASE for robustness
#     planner_match = re.search(r'sim_(fmtx|rrtx)', base, re.IGNORECASE)
#     samples_match = re.search(r'(\d+)samples', base, re.IGNORECASE)
    
#     planner = planner_match.group(1).upper() if planner_match else 'Unknown'
#     samples = int(samples_match.group(1)) if samples_match else 0
    
#     return planner, samples

# def aggregate_run_data(files):
#     """
#     Loads all metric CSV files and groups them by planner and sample count.
#     Returns a dictionary: {(planner, samples): [list_of_trial_dataframes]}
#     """
#     aggregated_data = {}
#     print("--- Loading and Aggregating Data ---")
#     required_columns = ['elapsed_s', 'duration_ms', 'path_cost', 'obstacle_checks', 'rewire_neighbor_searches', 'orphaned_nodes']
    
#     for filename in files:
#         try:
#             planner, samples = get_planner_info(filename)
#             if samples == 0:
#                 print(f"Warning: Could not determine sample count for {filename}. Skipping.")
#                 continue

#             df = pd.read_csv(filename)
#             # Validate that all required columns are present
#             if not all(col in df.columns for col in required_columns):
#                 print(f"Skipping {filename}: missing one or more required columns.")
#                 continue
            
#             # Remove rows where path_cost is infinite (no solution found yet) for cleaner stats
#             df.replace([np.inf, -np.inf], np.nan, inplace=True)
#             df.dropna(subset=['path_cost'], inplace=True)
            
#             config_key = (planner, samples)
#             if config_key not in aggregated_data:
#                 aggregated_data[config_key] = []
            
#             aggregated_data[config_key].append(df)
            
#         except Exception as e:
#             print(f"Error processing {filename}: {e}")
    
#     print(f"Successfully aggregated data for {len(aggregated_data)} configurations across {len(files)} files.")
#     return aggregated_data

# def plot_metric_distribution(aggregated_data, metric_col, title, y_label, use_log_scale=False):
#     """
#     Generates a comparative box plot for a given metric.
#     """
#     # Group data by sample count for plotting side-by-side
#     plot_groups = {}
#     for (planner, samples), df_list in aggregated_data.items():
#         if samples not in plot_groups:
#             plot_groups[samples] = {}
#         # Concatenate all trials for this planner config into one series
#         plot_groups[samples][planner] = pd.concat(df_list, ignore_index=True)[metric_col]

#     if not plot_groups:
#         print(f"No data to plot for metric '{metric_col}'.")
#         return

#     num_samples = len(plot_groups)
    
#     # === THE FIX IS HERE ===
#     # Set sharey=False to allow each subplot to have its own y-axis range.
#     fig, axes = plt.subplots(1, num_samples, figsize=(6 * num_samples, 7), sharey=False, squeeze=False)
#     # =======================

#     fig.suptitle(title, fontsize=22, fontweight='bold')

#     for i, (samples, planners_data) in enumerate(sorted(plot_groups.items())):
#         ax = axes[0, i]
        
#         plot_data = []
#         labels = []
#         # Ensure consistent RRTx, FMTx order
#         for planner_name in ['RRTX', 'FMTX']:
#             if planner_name in planners_data:
#                 plot_data.append(planners_data[planner_name].dropna()) # Drop NaNs for plotting
#                 labels.append(planner_name)
            
#         if not plot_data:
#             continue

#         box = ax.boxplot(plot_data, labels=labels, patch_artist=True, showfliers=False, medianprops={'color':'#D95319', 'linewidth':2.5})
        
#         colors = ['#0072BD', '#4DBEEE'] # Blue, Light Blue
#         for patch, color in zip(box['boxes'], colors):
#             patch.set_facecolor(color)
            
#         ax.set_title(f"{samples:,} Samples", fontsize=16, fontweight='bold')
#         ax.set_ylabel(y_label, fontsize=14) # Set label for each plot since scales differ
#         ax.grid(True, which='major', axis='y', linestyle='--', linewidth=0.7)
#         ax.yaxis.set_major_formatter(EngFormatter()) # Use engineering notation (e.g., 1k, 1M)
        
#         if use_log_scale:
#             ax.set_yscale('symlog', linthresh=1) # Use symlog for data that might include zero
#             # ax.set_yscale('log')

#     plt.tight_layout(rect=[0, 0.03, 1, 0.95])
#     plot_filename = f"comparison_{metric_col}.pdf"
#     plt.savefig(plot_filename, format="pdf", bbox_inches="tight")
#     print(f"-> Saved plot to '{plot_filename}'")
#     plt.show()

# def plot_time_series(trial_data, metric_col, title, y_label):
#     """
#     Generates a time-series plot with individual trials and a smoothed rolling median.
#     """
#     plot_groups = {}
#     for (planner, samples), list_of_dfs in trial_data.items():
#         if samples not in plot_groups:
#             plot_groups[samples] = {}
#         plot_groups[samples][planner] = list_of_dfs

#     if not plot_groups:
#         print(f"No data to plot for time-series '{metric_col}'.")
#         return

#     num_samples = len(plot_groups)
#     fig, axes = plt.subplots(1, num_samples, figsize=(8 * num_samples, 7), sharey=True, squeeze=False)
#     fig.suptitle(title, fontsize=22, fontweight='bold')

#     for i, (samples, planners_trial_data) in enumerate(sorted(plot_groups.items())):
#         ax = axes[0, i]
#         max_y = 0
#         colors = {'FMTX': '#EDB120', 'RRTX': '#77AC30'} # Gold, Green

#         for planner_name in sorted(planners_trial_data.keys()):
#             list_of_dfs = planners_trial_data[planner_name]
#             color = colors.get(planner_name, 'gray')

#             # 1. Plot individual trials with low opacity
#             for trial_df in list_of_dfs:
#                 trial_df_sorted = trial_df.sort_values('elapsed_s')
#                 ax.plot(trial_df_sorted['elapsed_s'], trial_df_sorted[metric_col], '-', color=color, linewidth=0.5, alpha=0.1)

#             # 2. Plot a smoothed rolling median over all concatenated trials
#             all_trials_df = pd.concat(list_of_dfs, ignore_index=True).sort_values('elapsed_s')
#             if not all_trials_df.empty:
#                 window_size = max(10, int(len(all_trials_df) * 0.05))
#                 rolling_median = all_trials_df[metric_col].rolling(window=window_size, center=True, min_periods=5).median()
#                 ax.plot(all_trials_df['elapsed_s'], rolling_median, color=color, linestyle='-', linewidth=3, label=f'{planner_name} (Median)')
#                 max_y = max(max_y, all_trials_df[metric_col].quantile(0.99)) # Use 99th percentile for robust y-limit
        
#         ax.set_title(f"{samples:,} Samples", fontsize=16, fontweight='bold')
#         ax.set_xlabel("Simulation Time (s)", fontsize=14)
#         if i == 0:
#             ax.set_ylabel(y_label, fontsize=14)
#         ax.grid(True, which='major', linestyle='--', linewidth=0.7)
#         ax.legend(loc='best', fontsize='large')
#         if max_y > 0:
#             ax.set_ylim(0, max_y * 1.1)

#     plt.tight_layout(rect=[0, 0.03, 1, 0.95])
#     plot_filename = f"timeseries_{metric_col}.pdf"
#     plt.savefig(plot_filename, format="pdf", bbox_inches="tight")
#     print(f"-> Saved plot to '{plot_filename}'")
#     plt.show()

# def perform_statistical_analysis(aggregated_data):
#     """Performs and prints a detailed statistical analysis for each metric."""
#     metrics_to_analyze = [
#         ("Update Duration (ms)", "duration_ms", "ms"),
#         ("Final Path Cost", "path_cost", ""),
#         ("Time to Goal", "time_to_goal", "s"),
#         ("Obstacle Checks", "obstacle_checks", "checks"),
#         ("Rewire Searches", "rewire_neighbor_searches", "searches"),
#         ("Orphaned Nodes", "orphaned_nodes", "nodes")
#     ]
    
#     # Group data by sample count
#     analysis_groups = {}
#     for (planner, samples), df_list in aggregated_data.items():
#         if samples not in analysis_groups:
#             analysis_groups[samples] = {}
#         analysis_groups[samples][planner] = pd.concat(df_list, ignore_index=True)

#     for samples, planners_data in sorted(analysis_groups.items()):
#         print(f"\n{'='*70}")
#         print(f" STATISTICAL ANALYSIS: {samples:,} SAMPLES")
#         print(f"{'='*70}")
        
#         if 'FMTX' not in planners_data or 'RRTX' not in planners_data:
#             print("Skipping analysis, missing data for one or more planners.")
#             continue
        
#         df_fmtx = planners_data['FMTX']
#         df_rrtx = planners_data['RRTX']

#         for name, col, unit in metrics_to_analyze:
#             print(f"\n--- Metric: {name} ---\n")
#             print(f"{'Statistic':<15} {'FMTx':<20} {'RRTx':<20}")
#             print("-" * 55)
            
#             data_fmtx = df_fmtx[col].dropna()
#             data_rrtx = df_rrtx[col].dropna()

#             if data_fmtx.empty or data_rrtx.empty:
#                 print("Not enough data for comparison.")
#                 continue

#             stats_to_run = [("Mean", np.mean), ("Median", np.median), ("Std Dev", np.std), ("95th %ile", lambda x: np.percentile(x, 95))]
#             for stat_name, func in stats_to_run:
#                 val_a = f"{func(data_fmtx):.2f} {unit}"
#                 val_b = f"{func(data_rrtx):.2f} {unit}"
#                 print(f"{stat_name:<15} {val_a:<20} {val_b:<20}")

#             # Mann-Whitney U test (non-parametric, good for non-normal data)
#             stat, p = stats.mannwhitneyu(data_fmtx, data_rrtx, alternative='two-sided')
#             print(f"\n  Mann-Whitney U-test p-value: {p:.4g}")
#             if p < 0.05:
#                 median_fmtx = np.median(data_fmtx)
#                 median_rrtx = np.median(data_rrtx)
#                 winner = "FMTx" if median_fmtx < median_rrtx else "RRTx"
#                 print(f"  Conclusion: Statistically significant difference found ( favoring {winner}).")
#             else:
#                 print("  Conclusion: No statistically significant difference detected.")
#             print("-" * 55)


# def main():
#     """Main function to load data, generate plots, and run analysis."""
#     # The script should be run from the parent directory of 'build'
#     search_path = "../build/sim_*_metrics.csv"
#     files = sorted(glob.glob(search_path))
    
#     if not files:
#         print(f"Error: No data files found matching '{os.path.abspath(search_path)}'")
#         print("Please ensure your C++ benchmarks have been run and saved CSV files.")
#         return
    
#     # 1. Load and aggregate all data from all trial runs
#     trial_data = aggregate_run_data(files)
#     if not trial_data:
#         print("No valid data could be aggregated. Exiting.")
#         return

#     # 2. Generate plots
#     print("\n--- Generating Plots ---")
    
#     # Plot distributions for all key metrics
#     plot_metric_distribution(trial_data, 'duration_ms', "Planner Update Time Distribution", "Update Duration (ms)", use_log_scale=True)
#     plot_metric_distribution(trial_data, 'path_cost', "Final Path Cost Distribution", "Path Cost")
#     plot_metric_distribution(trial_data, 'obstacle_checks', "Obstacle Check Distribution", "Collision Checks per Replan", use_log_scale=True)
#     plot_metric_distribution(trial_data, 'rewire_neighbor_searches', "Rewire Search Distribution", "Rewire Searches per Replan", use_log_scale=True)
#     plot_metric_distribution(trial_data, 'orphaned_nodes', "Orphaned Node Distribution", "Orphaned Nodes per Replan", use_log_scale=True)
    
#     # Plot time-series for efficiency and solution quality
#     plot_time_series(trial_data, 'duration_ms', "Planner Update Time Over Simulation", "Update Duration (ms)")
#     plot_time_series(trial_data, 'path_cost', "Path Cost Over Simulation", "Path Cost")

#     # 3. Perform and print statistical analysis
#     perform_statistical_analysis(trial_data)

# if __name__ == "__main__":
#     plt.style.use('seaborn-v0_8-whitegrid')
#     main()




############################################################

import pandas as pd
import matplotlib.pyplot as plt
import glob
import os
import re
import numpy as np
from scipy import stats
from matplotlib.ticker import EngFormatter

def get_planner_info(filename):
    """
    Extracts planner type and sample count from a filename.
    """
    base = os.path.basename(filename)
    planner_match = re.search(r'sim_(fmtx|rrtx)', base, re.IGNORECASE)
    samples_match = re.search(r'(\d+)samples', base, re.IGNORECASE)
    
    planner = planner_match.group(1).upper() if planner_match else 'Unknown'
    samples = int(samples_match.group(1)) if samples_match else 0
    
    return planner, samples

def aggregate_run_data(files):
    """
    Loads all metric CSV files and groups them by planner and sample count.
    Returns a dictionary: {(planner, samples): [list_of_trial_dataframes]}
    """
    aggregated_data = {}
    print("--- Loading and Aggregating Data ---")
    required_columns = ['elapsed_s', 'duration_ms', 'path_cost', 'obstacle_checks', 'rewire_neighbor_searches', 'orphaned_nodes']
    
    for filename in files:
        try:
            planner, samples = get_planner_info(filename)
            if samples == 0: continue

            df = pd.read_csv(filename)
            if not all(col in df.columns for col in required_columns):
                print(f"Skipping {filename}: missing one or more required columns.")
                continue
            
            df.replace([np.inf, -np.inf], np.nan, inplace=True)
            df.dropna(subset=['path_cost'], inplace=True)
            
            config_key = (planner, samples)
            if config_key not in aggregated_data:
                aggregated_data[config_key] = []
            
            aggregated_data[config_key].append(df)
            
        except Exception as e:
            print(f"Error processing {filename}: {e}")
    
    print(f"Successfully aggregated data for {len(aggregated_data)} configurations across {len(files)} files.")
    return aggregated_data

def plot_metric_distribution(aggregated_data, metric_col, title, y_label):
    """
    Generates a comparative box plot for a given metric.
    """
    plot_groups = {}
    for (planner, samples), df_list in aggregated_data.items():
        if samples not in plot_groups: plot_groups[samples] = {}
        plot_groups[samples][planner] = pd.concat(df_list, ignore_index=True)[metric_col]

    if not plot_groups: return

    num_samples = len(plot_groups)
    fig, axes = plt.subplots(1, num_samples, figsize=(6 * num_samples, 7), sharey=False, squeeze=False)
    fig.suptitle(title, fontsize=22, fontweight='bold')

    for i, (samples, planners_data) in enumerate(sorted(plot_groups.items())):
        ax = axes[0, i]
        plot_data, labels = [], []
        # Fixed order for consistent coloring
        for planner_name in ['RRTX', 'FMTX']:
            if planner_name in planners_data:
                plot_data.append(planners_data[planner_name].dropna())
                labels.append(planner_name)
        if not plot_data: continue

        box = ax.boxplot(plot_data, labels=labels, patch_artist=True, showfliers=False, medianprops={'color':'#A2142F', 'linewidth':2.5})
        
        # === COLOR CORRECTION ===
        # RRTx (Red), FMTx (Blue)
        colors = ['#d62728', '#1f77b4'] 
        for patch, color in zip(box['boxes'], colors):
            patch.set_facecolor(color)
            
        ax.set_title(f"{samples:,} Samples", fontsize=16, fontweight='bold')
        ax.set_ylabel(y_label, fontsize=14)
        ax.grid(True, which='major', axis='y', linestyle='--', linewidth=0.7)
        ax.yaxis.set_major_formatter(EngFormatter())
        
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plot_filename = f"comparison_{metric_col}.pdf"
    plt.savefig(plot_filename, format="pdf", bbox_inches="tight")
    print(f"-> Saved Box Plot to '{plot_filename}'")
    plt.show()

def plot_ecdf(aggregated_data, metric_col, title, x_label):
    """
    Generates a comparative Empirical Cumulative Distribution Function (ECDF) plot.
    """
    plot_groups = {}
    for (planner, samples), df_list in aggregated_data.items():
        if samples not in plot_groups: plot_groups[samples] = {}
        plot_groups[samples][planner] = pd.concat(df_list, ignore_index=True)[metric_col]

    if not plot_groups: return

    num_samples = len(plot_groups)
    fig, axes = plt.subplots(1, num_samples, figsize=(8 * num_samples, 6), sharey=True, squeeze=False)
    fig.suptitle(title, fontsize=22, fontweight='bold')
    
    # === COLOR CORRECTION ===
    colors = {'FMTX': '#1f77b4', 'RRTX': '#d62728'} # Blue, Red

    for i, (samples, planners_data) in enumerate(sorted(plot_groups.items())):
        ax = axes[0, i]
        
        for planner_name in sorted(planners_data.keys()):
            data = planners_data[planner_name].dropna()
            if data.empty: continue
            
            x_sorted = np.sort(data)
            y_ecdf = np.arange(1, len(x_sorted) + 1) / len(x_sorted)
            ax.plot(x_sorted, y_ecdf, linestyle='-', linewidth=2.5, color=colors.get(planner_name, 'gray'), label=planner_name)

        ax.set_title(f"{samples:,} Samples", fontsize=16, fontweight='bold')
        ax.set_xlabel(x_label, fontsize=14)
        if i == 0: ax.set_ylabel("Proportion of Replans", fontsize=14)
        ax.grid(True, which='both', linestyle='--', linewidth=0.5)
        ax.legend(loc='best', fontsize='large')
        ax.set_xscale('log')

    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plot_filename = f"ecdf_{metric_col}.pdf"
    plt.savefig(plot_filename, format="pdf", bbox_inches="tight")
    print(f"-> Saved ECDF Plot to '{plot_filename}'")
    plt.show()

def plot_time_series(trial_data, metric_col, title, y_label):
    """
    Generates a time-series plot with individual trials and a smoothed rolling median.
    """
    plot_groups = {}
    for (planner, samples), list_of_dfs in trial_data.items():
        if samples not in plot_groups: plot_groups[samples] = {}
        plot_groups[samples][planner] = list_of_dfs

    if not plot_groups: return

    num_samples = len(plot_groups)
    fig, axes = plt.subplots(1, num_samples, figsize=(8 * num_samples, 7), sharey=False, squeeze=False)
    fig.suptitle(title, fontsize=22, fontweight='bold')
    
    # === COLOR CORRECTION ===
    colors = {'FMTX': '#1f77b4', 'RRTX': '#d62728'} # Blue, Red

    for i, (samples, planners_trial_data) in enumerate(sorted(plot_groups.items())):
        ax = axes[0, i]
        max_y = 0

        for planner_name in sorted(planners_trial_data.keys()):
            list_of_dfs = planners_trial_data[planner_name]
            color = colors.get(planner_name, 'gray')

            # Plot individual trials with low opacity
            for trial_df in list_of_dfs:
                trial_df_sorted = trial_df.sort_values('elapsed_s')
                ax.plot(trial_df_sorted['elapsed_s'], trial_df_sorted[metric_col], '-', color=color, linewidth=0.5, alpha=0.1)

            # Plot a smoothed rolling median over all concatenated trials
            all_trials_df = pd.concat(list_of_dfs, ignore_index=True).sort_values('elapsed_s')
            if not all_trials_df.empty:
                window_size = max(10, int(len(all_trials_df) * 0.05))
                rolling_median = all_trials_df[metric_col].rolling(window=window_size, center=True, min_periods=5).median()
                ax.plot(all_trials_df['elapsed_s'], rolling_median, color=color, linestyle='-', linewidth=3, label=f'{planner_name} (Median)')
                if not all_trials_df[metric_col].empty:
                    max_y = max(max_y, all_trials_df[metric_col].quantile(0.99))
        
        ax.set_title(f"{samples:,} Samples", fontsize=16, fontweight='bold')
        ax.set_xlabel("Simulation Time (s)", fontsize=14)
        ax.set_ylabel(y_label, fontsize=14)
        ax.grid(True, which='major', linestyle='--', linewidth=0.7)
        ax.legend(loc='best', fontsize='large')
        if max_y > 0:
            ax.set_ylim(0, max_y * 1.1)

    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plot_filename = f"timeseries_{metric_col}.pdf"
    plt.savefig(plot_filename, format="pdf", bbox_inches="tight")
    print(f"-> Saved Time-Series Plot to '{plot_filename}'")
    plt.show()

def perform_statistical_analysis(aggregated_data):
    """
    Performs and prints a detailed statistical analysis for each metric.
    """
    metrics_to_analyze = [
        ("Update Duration (ms)", "duration_ms", "ms"),
        ("Final Path Cost", "path_cost", ""),
        ("Time to Goal", "time_to_goal", "s"),
        ("Obstacle Checks", "obstacle_checks", "checks"),
        ("Rewire Searches", "rewire_neighbor_searches", "searches"),
        ("Orphaned Nodes", "orphaned_nodes", "nodes")
    ]
    
    analysis_groups = {}
    for (planner, samples), df_list in aggregated_data.items():
        if samples not in analysis_groups: analysis_groups[samples] = {}
        analysis_groups[samples][planner] = pd.concat(df_list, ignore_index=True)

    for samples, planners_data in sorted(analysis_groups.items()):
        print(f"\n{'='*70}")
        print(f" STATISTICAL ANALYSIS: {samples:,} SAMPLES")
        print(f"{'='*70}")
        
        if 'FMTX' not in planners_data or 'RRTX' not in planners_data:
            print("Skipping analysis, missing data for one or more planners.")
            continue
        
        df_fmtx = planners_data['FMTX']
        df_rrtx = planners_data['RRTX']

        for name, col, unit in metrics_to_analyze:
            print(f"\n--- Metric: {name} ---\n")
            print(f"{'Statistic':<15} {'FMTx':<20} {'RRTx':<20}")
            print("-" * 55)
            
            data_fmtx = df_fmtx[col].dropna()
            data_rrtx = df_rrtx[col].dropna()

            if data_fmtx.empty or data_rrtx.empty:
                print("Not enough data for comparison.")
                continue

            stats_to_run = [("Mean", np.mean), ("Median", np.median), ("Std Dev", np.std), ("95th %ile", lambda x: np.percentile(x, 95))]
            for stat_name, func in stats_to_run:
                val_a = f"{func(data_fmtx):.2f} {unit}"
                val_b = f"{func(data_rrtx):.2f} {unit}"
                print(f"{stat_name:<15} {val_a:<20} {val_b:<20}")

            stat, p = stats.mannwhitneyu(data_fmtx, data_rrtx, alternative='two-sided')
            print(f"\n  Mann-Whitney U-test p-value: {p:.4g}")
            if p < 0.05:
                median_fmtx = np.median(data_fmtx)
                median_rrtx = np.median(data_rrtx)
                winner = "FMTx" if median_fmtx < median_rrtx else "RRTx"
                print(f"  Conclusion: Statistically significant difference found (favoring {winner}).")
            else:
                print("  Conclusion: No statistically significant difference detected.")
            print("-" * 55)

def main():
    """Main function to load data, generate all plots, and run analysis."""
    search_path = "../build/sim_*_metrics.csv"
    files = sorted(glob.glob(search_path))
    
    if not files:
        print(f"Error: No data files found matching '{os.path.abspath(search_path)}'")
        return
    
    trial_data = aggregate_run_data(files)
    if not trial_data: return

    print("\n--- Generating Plots ---")
    
    plot_metric_distribution(trial_data, 'duration_ms', "Planner Update Time Distribution", "Update Duration (ms)")
    plot_metric_distribution(trial_data, 'path_cost', "Final Path Cost Distribution", "Path Cost")
    
    plot_ecdf(trial_data, 'obstacle_checks', "ECDF of Obstacle Checks", "Number of Obstacle Checks (log scale)")
    plot_ecdf(trial_data, 'rewire_neighbor_searches', "ECDF of Rewire Searches", "Number of Rewire Searches (log scale)")
    plot_ecdf(trial_data, 'orphaned_nodes', "ECDF of Orphaned Nodes", "Number of Orphaned Nodes (log scale)")
    
    plot_time_series(trial_data, 'duration_ms', "Planner Update Time Over Simulation", "Update Duration (ms)")
    plot_time_series(trial_data, 'path_cost', "Path Cost Over Simulation", "Path Cost")
    
    perform_statistical_analysis(trial_data)

if __name__ == "__main__":
    plt.style.use('seaborn-v0_8-whitegrid')
    main()