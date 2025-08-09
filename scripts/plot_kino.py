import os
import re
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import numpy as np
import warnings

# Suppress future warnings from pandas to keep the output clean
warnings.simplefilter(action='ignore', category=FutureWarning)

def analyze_data(base_dir):
    """
    Analyzes simulation data from CSV files, assuming a directory structure like:
    base_dir/10obs/dubins/dubins_2500_2/sim_fmtx_..._metrics.csv

    Args:
        base_dir (str): The base directory containing the simulation data.

    Returns:
        tuple: A tuple containing two DataFrames:
               1. A DataFrame where each row is the median of one trial.
               A DataFrame with all the raw data from all trials combined.
    """
    state_spaces = ['R2T', 'dubins', 'thruster']
    obstacle_levels = ['10obs', '20obs']
    
    trial_records = []
    all_full_dfs = []

    for space in state_spaces:
        for obs_level in obstacle_levels:
            space_dir_path = os.path.join(base_dir, obs_level, space)
            if not os.path.exists(space_dir_path):
                continue

            for run_config_dir in os.listdir(space_dir_path):
                match = re.match(rf"{re.escape(space)}_(\d+)_(\d+)", run_config_dir)
                if match:
                    sample_size = int(match.group(1))
                    c_value = int(match.group(2))
                    
                    data_dir = os.path.join(space_dir_path, run_config_dir)
                    
                    for filename in os.listdir(data_dir):
                        if filename.endswith("_metrics.csv"):
                            file_path = os.path.join(data_dir, filename)
                            
                            planner_match = re.search(r"sim_(rrtx|fmtx)_", filename)
                            if not planner_match:
                                continue
                            planner = planner_match.group(1).upper()

                            try:
                                temp_df = pd.read_csv(file_path)
                                if not temp_df.empty:
                                    # Calculate median for this trial for the boxplot
                                    if 'duration_ms' in temp_df.columns:
                                        median_duration = temp_df['duration_ms'].median()
                                        if pd.notna(median_duration):
                                            trial_records.append({
                                                'state_space': space,
                                                'obstacles': obs_level,
                                                'planner': planner,
                                                'sample_size': sample_size,
                                                'c_value': c_value,
                                                'median_duration_ms': median_duration
                                            })
                                    
                                    # Store all data for summary stats
                                    full_df_for_append = temp_df.copy()
                                    full_df_for_append['state_space'] = space
                                    full_df_for_append['obstacles'] = obs_level
                                    full_df_for_append['planner'] = planner
                                    all_full_dfs.append(full_df_for_append)
                                    
                            except pd.errors.EmptyDataError:
                                print(f"Warning: CSV file is empty: {file_path}")
                            except Exception as e:
                                print(f"Error reading {file_path}: {e}")

    if not trial_records:
        return pd.DataFrame(), pd.DataFrame()

    full_df_final = pd.concat(all_full_dfs, ignore_index=True) if all_full_dfs else pd.DataFrame()
    
    return pd.DataFrame(trial_records), full_df_final

def create_enhanced_boxplot(df, state_space, plots_dir):
    """
    Creates a single, enhanced boxplot for the 'median_duration_ms' metric
    for a given state space and saves it as a PNG file.

    Args:
        df (pandas.DataFrame): The DataFrame containing the median-per-trial data.
        state_space (str): The state space to plot (e.g., 'R2T').
        plots_dir (str): The directory to save the plot in.
    """
    space_df = df[df['state_space'] == state_space].copy()
    if space_df.empty:
        print(f"No data to plot for state space: {state_space}")
        return

    # Create a combined category for plotting and ordering
    space_df['category'] = space_df['obstacles'] + ' ' + space_df['planner']
    plot_order = sorted(
        space_df['category'].unique(),
        key=lambda x: (int(x.split('obs')[0]), x.split(' ')[1])
    )

    plt.style.use('seaborn-v0_8-whitegrid')
    fig, ax = plt.subplots(figsize=(8, 8))

    # base colors by planner
    planner_colors = {'FMTX': '#1f77b4', 'RRTX': '#ff7f0e'}
    # map every category to its planner's color
    palette = {cat: planner_colors[cat.split()[1]] for cat in plot_order}

    # draw the boxplot
    sns.boxplot(
        data=space_df,
        x='category',
        y='median_duration_ms',
        order=plot_order,
        palette=palette,
        ax=ax,
        showfliers=False,
        boxprops=dict(edgecolor='black', linewidth=1.5),
        whiskerprops=dict(color='black', linewidth=1.5),
        capprops=dict(color='black', linewidth=1.5),
        medianprops=dict(color='yellow', linewidth=2.5)
    )

    # --- Formatting the plot ---
    ax.set_ylabel('Median Replanning Time per Trial (ms)', fontsize=14, weight='bold')
    ax.tick_params(axis='y', which='major', labelsize=12)

    # Force the y-axis to start at zero
    ax.set_ylim(bottom=0)

    # Legend with superscript x, larger font
    planner_order = [r'$FMT^{\mathrm{x}}$', r'$RRT^{\mathrm{x}}$']
    handles = [plt.Line2D([], [], color=planner_colors[p], lw=4) for p in ['FMTX', 'RRTX']]
    ax.legend(handles, planner_order, fontsize=14, title_fontsize=15)

    # Custom, inclined x-tick labels — bumped up font size
    new_labels = []
    for label in ax.get_xticklabels():
        text = label.get_text()  # e.g., "10obs FMTX"
        obs_str, planner = text.split()

        subset = space_df[
            (space_df['obstacles'] == obs_str) &
            (space_df['planner']   == planner)
        ]
        if not subset.empty:
            planner_fmt = r'$FMT^{\mathrm{x}}$' if planner == 'FMTX' else r'$RRT^{\mathrm{x}}$'
            sample_size = subset['sample_size'].iloc[0]
            samples_k   = f"{sample_size/1000:g}k"
            obs_count   = obs_str.replace('obs','')
            new_labels.append(f"{planner_fmt}, {samples_k}, {obs_count}")
        else:
            new_labels.append(text)

    ax.set_xticklabels(new_labels, rotation=45, ha='right', fontsize=13)
    ax.set_xlabel('Planner, Sample Size, # Obs.', fontsize=14, weight='bold', labelpad=20)
    ax.grid(True, which='major', linestyle='--', linewidth=0.7, color='grey', alpha=0.7)

    plt.tight_layout()
    plot_filename = f"{state_space}_duration_comparison.pdf"
    plot_path = os.path.join(plots_dir, plot_filename)
    plt.savefig(plot_path, format='pdf', bbox_inches='tight')
    plt.close(fig)
    print(f"Saved plot: {plot_path}")

def print_summary_statistics(df):
    """
    Calculates and prints summary statistics for various metrics to the console.
    """
    if df.empty:
        return
        
    metrics_to_summarize = [
        'duration_ms', 'path_cost', 'time_to_goal', 'obstacle_checks', 
        'rewire_neighbor_searches', 'orphaned_nodes'
    ]
    
    print("\n" + "="*80)
    print("STATISTICAL ANALYSIS")
    print("="*80)

    # Replace inf with NaN to be excluded from calculations
    df.replace([np.inf, -np.inf], np.nan, inplace=True)

    for (space, obs), group in df.groupby(['state_space', 'obstacles']):
        print(f"\n--- State Space: {space} | Obstacles: {obs} ---")
        
        planners_data = {
            'FMTX': group[group['planner'] == 'FMTX'],
            'RRTX': group[group['planner'] == 'RRTX']
        }

        for name in metrics_to_summarize:
            # Check if column exists in both planner dataframes for a fair comparison
            if name not in planners_data.get('FMTX', pd.DataFrame()).columns or \
               name not in planners_data.get('RRTX', pd.DataFrame()).columns:
                continue

            print(f"\n  Metric: {name}")
            print(f"  {'-'*65}")
            print(f"  {'Statistic':<15} {'FMTx':<20} {'RRTx':<20}")
            print(f"  {'-'*65}")

            data_fmtx = planners_data['FMTX'][name].dropna()
            data_rrtx = planners_data['RRTX'][name].dropna()

            if data_fmtx.empty or data_rrtx.empty:
                print("  Not enough data for full comparison.")
                continue

            stats_to_run = [("Mean", np.mean), ("Median", np.median), ("Std Dev", np.std)]
            for stat_name, func in stats_to_run:
                val_a = f"{func(data_fmtx):.2f}"
                val_b = f"{func(data_rrtx):.2f}"
                print(f"  {stat_name:<15} {val_a:<20} {val_b:<20}")
            print(f"  {'-'*65}")


if __name__ == '__main__':
    # --- Configuration ---
    BASE_DIRECTORY = '../build/new/kinodynamic'
    PLOTS_DIR = 'kinodynamic_plots'

    # --- Analysis ---
    median_df, full_df = analyze_data(BASE_DIRECTORY)

    if not median_df.empty:
        # --- Create Plots Directory ---
        os.makedirs(PLOTS_DIR, exist_ok=True)
        
        # --- Print Summary Statistics to Terminal ---
        print_summary_statistics(full_df)
        
        # --- Create and Save Plots ---
        print(f"\nGenerating plots and saving to '{PLOTS_DIR}' directory...")
        state_spaces_to_plot = sorted(median_df['state_space'].unique())
        for space in state_spaces_to_plot:
            create_enhanced_boxplot(median_df, space, PLOTS_DIR)
        print("...Done.")

    else:
        print("\nNo data found to analyze.")