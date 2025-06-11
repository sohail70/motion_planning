import os
import glob
import re
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns # Used for creating the heatmaps

# --- Configuration ---
FMTX_DIR = "../build/new/new_names/full_fmtx" # USER: Verify this path
RRTX_DIR = "../build/new/new_names/full_rrtx" # USER: Verify this path 
OUTPUT_FIGURES_DIR = "./figures_summary_heatmap"

# --- Data Loading and Processing Functions ---
def parse_filename_params(filename):
    base = os.path.basename(filename)
    match = re.match(r"(\d+)_(\d+)_(\d+)_sim_(fmtx|rrtx)_(\d+)samples", base.lower())
    if match:
        c_major, c_minor, obst_str, planner_str, samples_str = match.groups()
        C = float(f"{c_major}.{c_minor}")
        obstacle_count = int(obst_str)
        planner = 'FMTx' if planner_str == 'fmtx' else 'RRTx'
        samples = int(samples_str)
        return C, obstacle_count, planner, samples
    else:
        match_fallback = re.search(r"(\d+)obs_C(\d+p\d+)_sim_(fmtx|rrtx)_(\d+)samples", base.lower())
        if match_fallback:
            obst_str, c_str, planner_str, samples_str = match_fallback.groups()
            C = float(c_str.replace('p', '.'))
            obstacle_count = int(obst_str)
            planner = 'FMTx' if planner_str == 'fmtx' else 'RRTx'
            samples = int(samples_str)
            return C, obstacle_count, planner, samples
        raise ValueError(f"Cannot parse parameters from filename '{base}'.")

def load_all_data(fmtx_data_dir, rrtx_data_dir):
    data = {}
    for directory in (fmtx_data_dir, rrtx_data_dir):
        if not os.path.isdir(directory):
            print(f"Warning: Directory not found - {directory}")
            continue
        
        pattern = os.path.join(directory, "*.csv")
        filepaths_found = glob.glob(pattern)
        if not filepaths_found:
            print(f"No CSV files found in {directory}")
            continue

        for filepath in filepaths_found:
            try:
                C, obst, planner, samples = parse_filename_params(filepath)
            except ValueError as e:
                continue
            
            try:
                df = pd.read_csv(filepath)
                if df.empty or 'duration_ms' not in df.columns:
                    continue
                data.setdefault(samples, {}).setdefault(C, {}).setdefault(obst, {}).setdefault(planner, []).append(df)
            except Exception as e:
                print(f"Error reading '{filepath}': {e}")
                continue
    return data

def calculate_summary_metrics_from_trials(df_list):
    """
    FIXED: Calculates statistics based on per-trial medians, matching the table logic.
    """
    # 1. Get the median replanning time for each of the 30 trials.
    per_trial_medians = [df['duration_ms'].median() for df in df_list if not df.empty]

    if not per_trial_medians:
        return {'median': np.nan, 'std': np.nan}
        
    # 2. The final metrics are stats of this list of per-trial medians.
    return {
        'median': np.median(per_trial_medians), 
        'std': np.std(per_trial_medians)
    }

# --- PLOTTING FUNCTION ---

def plot_performance_ratio_heatmap(ratio_df, output_dir):
    """
    Creates a heatmap for each C-value, ensuring the color scale is consistent across all subplots.
    """
    if ratio_df.empty:
        print("Ratio DataFrame is empty. Skipping heatmap.")
        return
        
    c_values = sorted(ratio_df['C'].unique())
    fig, axes = plt.subplots(1, len(c_values), figsize=(len(c_values) * 6, 5), sharey=True)
    if len(c_values) == 1: axes = [axes]

    vmin = ratio_df['performance_ratio'].min()
    vmax = ratio_df['performance_ratio'].max()
    print(f"Global color range for heatmaps set to: [{vmin:.2f}, {vmax:.2f}]")

    for i, c_val in enumerate(c_values):
        ax = axes[i]
        subset = ratio_df[ratio_df['C'] == c_val]
        pivot_table = subset.pivot_table(index='obstacles', columns='samples', values='performance_ratio')
        
        sns.heatmap(
            pivot_table, 
            annot=True, fmt=".2f", cmap="viridis", ax=ax,
            linewidths=.5, cbar=(i == len(c_values) - 1),
            vmin=vmin, vmax=vmax
        )
        ax.set_title(f'Performance Ratio (C = {c_val})', fontsize=14)
        ax.set_xlabel('Sample Size (n)')
        ax.set_ylabel('Obstacle Count' if i == 0 else '')

    plt.tight_layout()
    
    output_path = os.path.join(output_dir, "performance_ratio_heatmaps.pdf")
    plt.savefig(output_path, format='pdf', bbox_inches='tight')
    plt.close()
    print(f"Saved: {output_path}")


def main():
    print(f"Loading data from directories: '{FMTX_DIR}', '{RRTX_DIR}'")
    data_loaded = load_all_data(FMTX_DIR, RRTX_DIR)

    if not data_loaded:
        print("No data was loaded. Exiting.")
        return

    os.makedirs(OUTPUT_FIGURES_DIR, exist_ok=True)
    print(f"Output figures will be saved to: {OUTPUT_FIGURES_DIR}")

    summary_data_list = []
    for samples, samples_data in data_loaded.items():
        for C_val, C_data in samples_data.items():
            for obst, obst_data in C_data.items():
                for planner, trials_list in obst_data.items():
                    # FIXED: Call the corrected summary function
                    metrics = calculate_summary_metrics_from_trials(trials_list)
                    summary_data_list.append({
                        'samples': samples, 'C': C_val, 'obstacles': obst, 'planner': planner, **metrics
                    })
    
    summary_df = pd.DataFrame(summary_data_list).dropna()
    if summary_df.empty:
        print("Summary DataFrame is empty after processing. Cannot generate plots.")
        return
        
    pivot_df = summary_df.pivot_table(
        index=['samples', 'C', 'obstacles'],
        columns='planner',
        # FIXED: Use the correct column name 'median'
        values='median' 
    ).reset_index()

    if 'FMTx' in pivot_df.columns and 'RRTx' in pivot_df.columns:
        pivot_df['performance_ratio'] = pivot_df['RRTx'] / pivot_df['FMTx']
        print("\n--- DataFrame with Performance Ratio ---")
        print(pivot_df.head().to_string())
        
        plot_performance_ratio_heatmap(pivot_df, OUTPUT_FIGURES_DIR)
    else:
        print("Could not find data for both 'FMTx' and 'RRTx' in the summary. Cannot calculate ratio.")

    print("\n--- Visualization complete ---")


if __name__ == "__main__":
    main()
