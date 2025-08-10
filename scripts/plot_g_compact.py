import glob
import os
import re
import math
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import AutoMinorLocator
# import seaborn as sns # Seaborn is not strictly needed for these specific plots

# --- Configuration ---
FMTX_DIR = "../build/new/new_names/full_fmtx" 
RRTX_DIR = "../build/new/new_names/full_rrtx"
OUTPUT_FIGURES_DIR = "./figures_summary_compact_sensitivity"
DEFAULT_METRIC = 'median_duration' 

# --- Helper Functions ---
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
        raise ValueError(f"Cannot parse parameters from filename '{base}' using primary regex.")

def load_all_data(fmtx_data_dir, rrtx_data_dir):
    data = {}
    for directory in (fmtx_data_dir, rrtx_data_dir):
        if not os.path.isdir(directory):
            print(f"Warning: Directory not found - {directory}")
            continue
        pattern = os.path.join(directory, "*.csv")
        filepaths_found = glob.glob(pattern)
        if not filepaths_found: print(f"No CSV files found in {directory}"); continue
        for filepath in filepaths_found:
            try: C, obst, planner, samples = parse_filename_params(filepath)
            except ValueError as e: print(e); continue
            try:
                df = pd.read_csv(filepath)
                if df.empty or 'elapsed_s' not in df.columns or 'duration_ms' not in df.columns: continue
            except Exception as e: print(f"Error reading '{filepath}': {e}"); continue
            data.setdefault(samples, {}).setdefault(C, {}).setdefault(obst, {}).setdefault(planner, []).append(df)
    return data

def interpolate_trials_to_grid(df_list, grid_points):
    interp_curves = []
    for df_trial in df_list:
        t, y = df_trial['elapsed_s'].values, df_trial['duration_ms'].values
        unique_t_indices = np.argsort(t)
        t_sorted, y_sorted = t[unique_t_indices], y[unique_t_indices]
        unique_t, unique_indices = np.unique(t_sorted, return_index=True)
        unique_y = y_sorted[unique_indices]
        if len(unique_t) < 2: interp_curves.append(np.full_like(grid_points, np.nan)); continue
        interp_curves.append(np.interp(grid_points, unique_t, unique_y, left=np.nan, right=np.nan))
    if not interp_curves: return None
    return np.vstack(interp_curves)

def calculate_summary_metrics_from_trials(df_list, grid_points):
    stacked_curves = interpolate_trials_to_grid(df_list, grid_points)
    if stacked_curves is None or np.all(np.isnan(stacked_curves)):
        return {'median_duration': np.nan, 'p95_duration': np.nan, 'mean_duration': np.nan}
    all_durations = stacked_curves.flatten()[~np.isnan(stacked_curves.flatten())]
    if not len(all_durations): return {'median_duration': np.nan, 'p95_duration': np.nan, 'mean_duration': np.nan}
    return {'median_duration': np.median(all_durations), 
            'p95_duration': np.percentile(all_durations, 95), 
            'mean_duration': np.mean(all_durations)}

# --- Plotting Functions ---
def plot_sensitivity_C_all_samples_grouped(summary_df, fixed_obstacle_count, 
                                           unique_C_values, unique_sample_values,
                                           output_dir, metric_to_plot=DEFAULT_METRIC):
    if summary_df.empty:
        print(f"Summary DataFrame is empty. Skipping plot for Obstacles={fixed_obstacle_count}.")
        return

    plt.figure(figsize=(10, 7)) # Adjusted for legend below
    ax = plt.gca()
    fmtx_cmap = plt.cm.Blues
    rrtx_cmap = plt.cm.Reds
    norm_samples = {s: 0.3 + 0.6 * (i / (len(unique_sample_values) - 1 if len(unique_sample_values) > 1 else 1)) 
                    for i, s in enumerate(unique_sample_values)}
    legend_handles = []

    for planner in ['FMTx', 'RRTx']:
        cmap = fmtx_cmap if planner == 'FMTx' else rrtx_cmap
        for samples_val in unique_sample_values:
            plot_data = summary_df[
                (summary_df['planner'] == planner) &
                (summary_df['obstacles'] == fixed_obstacle_count) &
                (summary_df['samples'] == samples_val)
            ].sort_values(by='C')
            if plot_data.empty: continue
            
            color_intensity = norm_samples.get(samples_val, 0.5)
            line_color = cmap(color_intensity)
            line, = ax.plot(plot_data['C'], plot_data[metric_to_plot],
                            marker='o', linestyle='-', linewidth=1.5,
                            label=f"{planner} ({samples_val} samples)", color=line_color)
            legend_handles.append(line)

    ax.set_xlabel("C-value (Scaling Factor)", fontsize=12)
    ax.set_ylabel(f"Overall {metric_to_plot.replace('_', ' ').title()} (ms)", fontsize=12)
    ax.set_title(f"Performance vs. C-value (Fixed Obstacles = {fixed_obstacle_count})\nLines show different sample sizes (darker = more samples)", fontsize=14)
    
    if unique_C_values is not None and len(unique_C_values) > 0:
        ax.set_xticks(unique_C_values)
        ax.set_xticklabels([str(xt) for xt in unique_C_values])

    def get_legend_sort_key(handle):
        label = handle.get_label()
        p_name = "FMTx" if "FMTx" in label else "RRTx"
        s_val = int(re.search(r'\((\d+) samples\)', label).group(1))
        return (p_name, s_val)
    
    if legend_handles:
        sorted_handles = sorted(legend_handles, key=get_legend_sort_key)
        # Adjust ncol for legend below plot, aim for 2-3 rows if possible
        num_legend_entries = len(sorted_handles)
        legend_ncol = math.ceil(num_legend_entries / 2) if num_legend_entries > 4 else num_legend_entries # Max 2 rows if more than 4 entries
        legend_ncol = min(legend_ncol, 4) # Cap at 4 columns to prevent excessive width

        ax.legend(handles=sorted_handles, title="Planner (Sample Size)", 
                  loc='lower center', bbox_to_anchor=(0.5, -0.25 - (0.05 * (math.ceil(num_legend_entries/legend_ncol)-1))), # Adjust y-offset based on rows
                  ncol=legend_ncol, fontsize='small', frameon=True)


    ax.grid(True, linestyle='--', alpha=0.7)
    plt.subplots_adjust(bottom=0.25 + (0.05 * (math.ceil(len(legend_handles)/(legend_ncol if legend_ncol > 0 else 1))-1))) # Make space for legend
    
    filename = f"sensitivity_C_all_samples_obs{fixed_obstacle_count}_{metric_to_plot}.pdf"
    output_path = os.path.join(output_dir, filename)
    plt.savefig(output_path, bbox_inches='tight', format='pdf')
    print(f"Saved: {output_path}")
    plt.close()

def plot_sensitivity_Obstacles_all_samples_grouped(summary_df, fixed_C_value,
                                                   unique_obstacle_values, unique_sample_values,
                                                   output_dir, metric_to_plot=DEFAULT_METRIC):
    if summary_df.empty:
        print(f"Summary DataFrame is empty. Skipping plot for C={fixed_C_value}.")
        return

    plt.figure(figsize=(10, 7)) # Adjusted for legend below
    ax = plt.gca()
    fmtx_cmap = plt.cm.Blues
    rrtx_cmap = plt.cm.Reds
    norm_samples = {s: 0.3 + 0.6 * (i / (len(unique_sample_values) - 1 if len(unique_sample_values) > 1 else 1))
                    for i, s in enumerate(unique_sample_values)}
    legend_handles = []

    for planner in ['FMTx', 'RRTx']:
        cmap = fmtx_cmap if planner == 'FMTx' else rrtx_cmap
        for samples_val in unique_sample_values:
            plot_data = summary_df[
                (summary_df['planner'] == planner) &
                (summary_df['C'] == fixed_C_value) &
                (summary_df['samples'] == samples_val)
            ].sort_values(by='obstacles')
            if plot_data.empty: continue

            color_intensity = norm_samples.get(samples_val, 0.5)
            line_color = cmap(color_intensity)
            line, = ax.plot(plot_data['obstacles'], plot_data[metric_to_plot],
                            marker='o', linestyle='-', linewidth=1.5,
                            label=f"{planner} ({samples_val} samples)", color=line_color)
            legend_handles.append(line)

    ax.set_xlabel("Number of Obstacles", fontsize=12)
    ax.set_ylabel(f"Overall {metric_to_plot.replace('_', ' ').title()} (ms)", fontsize=12)
    ax.set_title(f"Performance vs. Obstacle Count (Fixed C-value = {fixed_C_value:.1f})\nLines show different sample sizes (darker = more samples)", fontsize=14)

    if unique_obstacle_values is not None and len(unique_obstacle_values) > 0:
        ax.set_xticks(unique_obstacle_values)
        ax.set_xticklabels([str(xt) for xt in unique_obstacle_values])

    def get_legend_sort_key(handle):
        label = handle.get_label()
        p_name = "FMTx" if "FMTx" in label else "RRTx"
        s_val = int(re.search(r'\((\d+) samples\)', label).group(1))
        return (p_name, s_val)

    if legend_handles:
        sorted_handles = sorted(legend_handles, key=get_legend_sort_key)
        num_legend_entries = len(sorted_handles)
        legend_ncol = math.ceil(num_legend_entries / 2) if num_legend_entries > 4 else num_legend_entries
        legend_ncol = min(legend_ncol, 4)

        ax.legend(handles=sorted_handles, title="Planner (Sample Size)",
                  loc='lower center', bbox_to_anchor=(0.5, -0.25 - (0.05 * (math.ceil(num_legend_entries/legend_ncol)-1))),
                  ncol=legend_ncol, fontsize='small', frameon=True)

    ax.grid(True, linestyle='--', alpha=0.7)
    plt.subplots_adjust(bottom=0.25 + (0.05 * (math.ceil(len(legend_handles)/(legend_ncol if legend_ncol > 0 else 1))-1)))


    filename = f"sensitivity_Obst_all_samples_C{str(fixed_C_value).replace('.', 'p')}_{metric_to_plot}.pdf"
    output_path = os.path.join(output_dir, filename)
    plt.savefig(output_path, bbox_inches='tight', format='pdf')
    print(f"Saved: {output_path}")
    plt.close()


# --- Main Execution ---
def main():
    print(f"Loading data from FMTx dir: {FMTX_DIR}")
    print(f"Loading data from RRTx dir: {RRTX_DIR}")
    data_loaded = load_all_data(FMTX_DIR, RRTX_DIR)

    if not data_loaded:
        print("No data was loaded. Exiting."); return

    os.makedirs(OUTPUT_FIGURES_DIR, exist_ok=True)
    print(f"Output figures will be saved to: {OUTPUT_FIGURES_DIR}")

    dt = 1 / 30.0
    grid_time_points = np.arange(0, 30 + dt / 2, dt)
    summary_data_list = []

    for samples, samples_data in data_loaded.items():
        for C_val_loop, C_data in samples_data.items(): # Renamed C_val to C_val_loop to avoid conflict
            for obst, obst_data in C_data.items():
                for planner, trials_list in obst_data.items():
                    metrics = calculate_summary_metrics_from_trials(trials_list, grid_time_points)
                    summary_data_list.append({
                        'samples': samples, 'C': C_val_loop, 'obstacles': obst, 'planner': planner, **metrics
                    })
    
    summary_df = pd.DataFrame(summary_data_list).dropna()
    if summary_df.empty:
        print("Summary DataFrame is empty after processing/dropping NaNs. Exiting."); return
        
    print(f"\n--- Generated Summary DataFrame (NaNs dropped, {len(summary_df)} rows) ---")
    print(summary_df.head().to_string())
    summary_df.to_csv(os.path.join(OUTPUT_FIGURES_DIR, "summary_metrics_all_conditions.csv"), index=False)

    # --- Get unique sorted values for iteration and ticks ---
    # Ensure these are your specific tested values by taking uniques from the dataframe
    # These are the values that will appear on the x-axes or be iterated over.
    unique_samples_found = sorted(summary_df['samples'].unique())
    unique_C_values_found = sorted(summary_df['C'].unique())
    unique_obstacle_counts_found = sorted(summary_df['obstacles'].unique())
    
    # Explicitly define the values you want to see on axes if they differ from all found values
    # For example, if you only want to ensure 1.0, 1.5, 2.0 appear for C even if 1.1 was tested
    # For now, we'll use all unique values found in the data for ticks.
    # User wants specific ticks:
    # Samples: 2k, 5k, 10k, 20k (assuming these are in unique_samples_found)
    # Obstacles: 10, 20, 30 (assuming these are in unique_obstacle_counts_found)
    # C: 1.0, 1.5, 2.0 (assuming these are in unique_C_values_found)
    
    print(f"\nUnique values for plotting: Samples={unique_samples_found}, C={unique_C_values_found}, Obstacles={unique_obstacle_counts_found}")
    print(f"Generating plots using metric: {DEFAULT_METRIC}")

    # Sensitivity to C (All Samples Grouped per Obstacle Count)
    print("\n--- Generating Sensitivity to C (All Samples Grouped) Plots ---")
    for obst_count in unique_obstacle_counts_found:
        plot_sensitivity_C_all_samples_grouped(
            summary_df,
            fixed_obstacle_count=obst_count,
            unique_C_values=unique_C_values_found, 
            unique_sample_values=unique_samples_found, 
            output_dir=OUTPUT_FIGURES_DIR,
            metric_to_plot=DEFAULT_METRIC
        )
        
    # 2. Sensitivity to Obstacles (All Samples Grouped per C-value)
    print("\n--- Generating Sensitivity to Obstacles (All Samples Grouped) Plots ---")
    for C_value in unique_C_values_found:
        plot_sensitivity_Obstacles_all_samples_grouped(
            summary_df,
            fixed_C_value=C_value,
            unique_obstacle_values=unique_obstacle_counts_found,
            unique_sample_values=unique_samples_found,
            output_dir=OUTPUT_FIGURES_DIR,
            metric_to_plot=DEFAULT_METRIC
        )

    # --- Placeholder for other plot types from previous script if you want to re-integrate them ---
    # e.g., scatter plot, speedup bars, individual sensitivity plots

    print("\n--- Compact sensitivity plot generation complete ---")

if __name__ == "__main__":
    main()