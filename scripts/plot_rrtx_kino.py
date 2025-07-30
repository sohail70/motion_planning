import pandas as pd
import matplotlib.pyplot as plt
import glob
import os
import re
import numpy as np
from scipy import stats
from matplotlib.ticker import EngFormatter
import itertools
from matplotlib import cm

def get_planner_info(filename):
    """
    Extracts RRTx mode and sample count from a filename.
    Correctly parses names like: 'sim_rrtx_3000samples_29_7_2025_16_21_40_mode1.csv'
    """
    base = os.path.basename(filename)
    # This regex is specifically designed to find the sample count and the 'mode' identifier.
    match = re.search(r'sim_rrtx_(\d+)samples_.*_(mode\d+)\.csv', base, re.IGNORECASE)
    
    if match:
        # Group 1: sample count, Group 2: mode identifier
        samples = int(match.group(1))
        planner_mode = match.group(2).upper()  # This will be 'MODE1', 'MODE2', etc.
        return planner_mode, samples
    
    # Return None if the filename doesn't match the expected pattern
    return None, None

def aggregate_run_data(files):
    """
    Loads all RRTx metric CSV files and groups them by mode and sample count.
    Returns a dictionary: {(planner_mode, samples): [list_of_trial_dataframes]}
    """
    aggregated_data = {}
    print("--- Loading and Aggregating RRTx Data by Mode ---")
    required_columns = ['elapsed_s', 'duration_ms', 'path_cost', 'obstacle_checks', 'rewire_neighbor_searches', 'orphaned_nodes']
    
    for filename in files:
        try:
            planner_mode, samples = get_planner_info(filename)
            # Skip files that don't match the pattern or have no sample info
            if not planner_mode or not samples:
                continue

            df = pd.read_csv(filename)
            if not all(col in df.columns for col in required_columns):
                print(f"Skipping {filename}: missing one or more required columns.")
                continue
            
            df.replace([np.inf, -np.inf], np.nan, inplace=True)
            df.dropna(subset=['path_cost'], inplace=True)
            
            config_key = (planner_mode, samples)
            if config_key not in aggregated_data:
                aggregated_data[config_key] = []
            
            aggregated_data[config_key].append(df)
            
        except Exception as e:
            print(f"Error processing {filename}: {e}")
    
    print(f"Successfully aggregated data for {len(aggregated_data)} configurations.")
    return aggregated_data

def get_planner_colors(aggregated_data):
    """Creates a consistent color mapping for all planners (modes) found in the data."""
    all_planners = sorted(list(set(planner for planner, _ in aggregated_data.keys())))
    # Use a colormap that provides distinct colors
    colors = cm.get_cmap('viridis', len(all_planners))
    return {planner: colors(i) for i, planner in enumerate(all_planners)}

def plot_metric_distribution(aggregated_data, color_map, metric_col, title, y_label):
    """
    Generates a comparative box plot for a given metric across RRTx modes.
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
        
        sorted_planners = sorted(planners_data.keys())
        plot_data = [planners_data[p].dropna() for p in sorted_planners]
        
        if not any(len(d) > 0 for d in plot_data): continue

        box = ax.boxplot(plot_data, labels=sorted_planners, patch_artist=True, showfliers=False, medianprops={'color':'#A2142F', 'linewidth':2.5})
        
        for patch, planner_name in zip(box['boxes'], sorted_planners):
            patch.set_facecolor(color_map.get(planner_name, 'gray'))
            
        ax.set_title(f"{samples:,} Samples", fontsize=16, fontweight='bold')
        ax.set_ylabel(y_label, fontsize=14)
        ax.grid(True, which='major', axis='y', linestyle='--', linewidth=0.7)
        ax.yaxis.set_major_formatter(EngFormatter())
        
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plot_filename = f"comparison_{metric_col}.pdf"
    plt.savefig(plot_filename, format="pdf", bbox_inches="tight")
    print(f"-> Saved Box Plot to '{plot_filename}'")
    plt.show()

def plot_ecdf(aggregated_data, color_map, metric_col, title, x_label):
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
    
    for i, (samples, planners_data) in enumerate(sorted(plot_groups.items())):
        ax = axes[0, i]
        
        for planner_name in sorted(planners_data.keys()):
            data = planners_data[planner_name].dropna()
            if data.empty: continue
            
            x_sorted = np.sort(data)
            y_ecdf = np.arange(1, len(x_sorted) + 1) / len(x_sorted)
            ax.plot(x_sorted, y_ecdf, linestyle='-', linewidth=2.5, color=color_map.get(planner_name, 'gray'), label=planner_name)

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

def plot_time_series(trial_data, color_map, metric_col, title, y_label):
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
    
    for i, (samples, planners_trial_data) in enumerate(sorted(plot_groups.items())):
        ax = axes[0, i]
        max_y = 0

        for planner_name in sorted(planners_trial_data.keys()):
            list_of_dfs = planners_trial_data[planner_name]
            color = color_map.get(planner_name, 'gray')

            for trial_df in list_of_dfs:
                trial_df_sorted = trial_df.sort_values('elapsed_s')
                ax.plot(trial_df_sorted['elapsed_s'], trial_df_sorted[metric_col], '-', color=color, linewidth=0.5, alpha=0.1)

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
    Performs and prints a detailed statistical analysis for each metric,
    comparing all RRTx modes with each other.
    """
    metrics_to_analyze = [
        ("Update Duration (ms)", "duration_ms", "ms"),
        ("Final Path Cost", "path_cost", ""),
        ("Obstacle Checks", "obstacle_checks", "checks"),
        ("Rewire Searches", "rewire_neighbor_searches", "searches"),
        ("Orphaned Nodes", "orphaned_nodes", "nodes")
    ]
    
    analysis_groups = {}
    for (planner, samples), df_list in aggregated_data.items():
        if samples not in analysis_groups: analysis_groups[samples] = {}
        analysis_groups[samples][planner] = pd.concat(df_list, ignore_index=True)

    for samples, planners_data in sorted(analysis_groups.items()):
        planners = sorted(planners_data.keys())
        
        print(f"\n{'='*80}")
        print(f" STATISTICAL ANALYSIS: {samples:,} SAMPLES")
        print(f"{'='*80}")
        
        if len(planners) < 2:
            print("Skipping analysis, need at least two modes to compare.")
            continue
        
        for name, col, unit in metrics_to_analyze:
            all_planner_data = {p: planners_data[p][col].dropna() for p in planners if col in planners_data[p].columns}
            if len(all_planner_data) < len(planners):
                print(f"\n--- Metric: {name} (Skipped: Column not found for all planners) ---")
                continue

            print(f"\n--- Metric: {name} ---")
            
            header = f"{'Statistic':<15}" + "".join([f"{p:<20}" for p in planners])
            print(header)
            print("-" * len(header))
            
            stats_to_run = [("Mean", np.mean), ("Median", np.median), ("Std Dev", np.std), ("95th %ile", lambda x: np.percentile(x, 95))]
            for stat_name, func in stats_to_run:
                row = f"{stat_name:<15}"
                for p in planners:
                    data = all_planner_data.get(p, pd.Series(dtype='float64'))
                    val_str = "N/A" if data.empty else f"{func(data):.2f} {unit}"
                    row += f"{val_str:<20}"
                print(row)
            
            print("\n  Pairwise Mann-Whitney U-tests (p-values):")
            planner_pairs = list(itertools.combinations(planners, 2))
            
            for p1, p2 in planner_pairs:
                data1 = all_planner_data[p1]
                data2 = all_planner_data[p2]

                if data1.empty or data2.empty:
                    print(f"  - {p1} vs {p2}: Not enough data for comparison.")
                    continue
                
                _, p = stats.mannwhitneyu(data1, data2, alternative='two-sided')
                
                result_str = f"p = {p:.4g}"
                if p < 0.05:
                    median1, median2 = np.median(data1), np.median(data2)
                    winner = p1 if median1 < median2 else p2
                    result_str += f" (Significant, {winner} is better)"
                else:
                    result_str += " (Not significant)"
                
                print(f"  - {p1:<15} vs {p2:<15}: {result_str}")
            
            print("-" * len(header))

def main():
    """Main function to load data, generate all plots, and run analysis."""
    # CORRECTED: Removed "_metrics" from the search pattern
    search_path = "../build/sim_*.csv"
    files = sorted(glob.glob(search_path))
    
    if not files:
        print(f"Error: No data files found matching '{os.path.abspath(search_path)}'")
        return
    
    trial_data = aggregate_run_data(files)
    if not trial_data:
        print("No valid RRTx data was loaded. Exiting.")
        return

    color_map = get_planner_colors(trial_data)
    print(f"\n--- Planner Mode Color Map ---")
    for planner, color in color_map.items():
        print(f"- {planner}: {color}")

    print("\n--- Generating Plots ---")
    
    plot_metric_distribution(trial_data, color_map, 'duration_ms', "Planner Update Time Distribution by Mode", "Update Duration (ms)")
    plot_metric_distribution(trial_data, color_map, 'path_cost', "Final Path Cost Distribution by Mode", "Path Cost")
    
    plot_ecdf(trial_data, color_map, 'obstacle_checks', "ECDF of Obstacle Checks by Mode", "Number of Obstacle Checks (log scale)")
    plot_ecdf(trial_data, color_map, 'rewire_neighbor_searches', "ECDF of Rewire Searches by Mode", "Number of Rewire Searches (log scale)")
    plot_ecdf(trial_data, color_map, 'orphaned_nodes', "ECDF of Orphaned Nodes by Mode", "Number of Orphaned Nodes (log scale)")
    
    plot_time_series(trial_data, color_map, 'duration_ms', "Planner Update Time Over Simulation by Mode", "Update Duration (ms)")
    plot_time_series(trial_data, color_map, 'path_cost', "Path Cost Over Simulation by Mode", "Path Cost")
    
    perform_statistical_analysis(trial_data)

if __name__ == "__main__":
    plt.style.use('seaborn-v0_8-whitegrid')
    main()