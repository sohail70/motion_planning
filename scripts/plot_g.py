import glob
import os
import re
import math
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import AutoMinorLocator
import seaborn as sns

# --- Configuration ---
FMTX_DIR = "../build/new/new_names/full_fmtx" # USER: Verify this path
RRTX_DIR = "../build/new/new_names/full_rrtx" # USER: Verify this path
OUTPUT_FIGURES_DIR = "./figures_summary_all_combinations" # Output directory for plots
DEFAULT_METRIC = 'median_duration' # Metric to use for most plots

# --- Helper Functions ---
def parse_filename_params(filename):
    base = os.path.basename(filename)
    # Expects format like: 1_0_10_sim_fmtx_2000samples_....csv
    # Or:                  1_5_30_sim_rrtx_5000samples_....csv
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
        print(f"Searching for files in: {pattern}")
        filepaths_found = glob.glob(pattern)
        if not filepaths_found:
            print(f"No CSV files found in {directory}")
            continue
        print(f"Found {len(filepaths_found)} files in {directory}")

        for filepath in filepaths_found:
            try:
                C, obst, planner, samples = parse_filename_params(filepath)
            except ValueError as e:
                print(e)
                continue
            try:
                df = pd.read_csv(filepath)
                if df.empty or 'elapsed_s' not in df.columns or 'duration_ms' not in df.columns:
                    print(f"Warning: Skipping '{filepath}' (empty or missing required columns).")
                    continue
            except Exception as e:
                print(f"Error reading '{filepath}': {e}")
                continue
            
            data.setdefault(samples, {}) \
                .setdefault(C, {}) \
                .setdefault(obst, {}) \
                .setdefault(planner, []).append(df)
    return data

def interpolate_trials_to_grid(df_list, grid_points):
    interp_curves = []
    for df_trial in df_list:
        t = df_trial['elapsed_s'].values
        y = df_trial['duration_ms'].values
        
        unique_t_indices = np.argsort(t)
        t_sorted = t[unique_t_indices]
        y_sorted = y[unique_t_indices]
        unique_t, unique_indices = np.unique(t_sorted, return_index=True)
        unique_y = y_sorted[unique_indices]

        if len(unique_t) < 2:
            interp_curves.append(np.full_like(grid_points, np.nan))
            continue
        interp_curves.append(np.interp(grid_points, unique_t, unique_y, left=np.nan, right=np.nan))
    
    if not interp_curves: return None
    return np.vstack(interp_curves)

def calculate_summary_metrics_from_trials(df_list, grid_points):
    stacked_curves = interpolate_trials_to_grid(df_list, grid_points)
    if stacked_curves is None or np.all(np.isnan(stacked_curves)):
        return {'median_duration': np.nan, 'p95_duration': np.nan, 'mean_duration': np.nan}

    all_interpolated_durations = stacked_curves.flatten()
    all_interpolated_durations = all_interpolated_durations[~np.isnan(all_interpolated_durations)]

    if len(all_interpolated_durations) == 0:
        return {'median_duration': np.nan, 'p95_duration': np.nan, 'mean_duration': np.nan}

    return {
        'median_duration': np.median(all_interpolated_durations),
        'p95_duration': np.percentile(all_interpolated_durations, 95),
        'mean_duration': np.mean(all_interpolated_durations)
    }

# --- Plotting Functions ---
def plot_parameter_sensitivity(summary_df, fixed_params, varying_param_name, x_tick_values,
                               output_dir, metric_to_plot=DEFAULT_METRIC, palette=None):
    if summary_df.empty: return
    plt.figure(figsize=(8, 5.5)) # Slightly taller for title
    ax = plt.gca()

    query_parts = [f"`{k}` == {v}" for k, v in fixed_params.items()]
    query_str = " & ".join(query_parts)
    plot_data = summary_df.query(query_str) if query_str else summary_df.copy()

    if plot_data.empty:
        print(f"No data for sensitivity: fixed={fixed_params}, varying={varying_param_name}")
        plt.close(); return

    planners = ['FMTx', 'RRTx']
    colors = palette if palette else {'FMTx': 'royalblue', 'RRTx': 'orangered'}

    for planner in planners:
        planner_data = plot_data[plot_data['planner'] == planner].sort_values(by=varying_param_name)
        if not planner_data.empty:
            ax.plot(planner_data[varying_param_name], planner_data[metric_to_plot], 
                    marker='o', linestyle='-', label=planner, color=colors[planner])

    ax.set_xlabel(varying_param_name.replace("_", " ").title())
    ax.set_ylabel(f"Overall {metric_to_plot.replace('_', ' ').title()} (ms)")
    
    fixed_params_str = ", ".join([f"{k.replace('_',' ').title()}={v}" for k,v in fixed_params.items()])
    title_str = f"Performance vs. {varying_param_name.replace('_',' ').title()}"
    if fixed_params_str: title_str += f"\n(Fixed: {fixed_params_str})"
    ax.set_title(title_str)
    
    if x_tick_values is not None and len(x_tick_values) > 0:
        ax.set_xticks(x_tick_values)
        ax.set_xticklabels([str(xt) for xt in x_tick_values]) # Ensure labels are strings if needed

    ax.legend()
    ax.grid(True, linestyle='--', alpha=0.7)
    plt.tight_layout()
    
    filename_parts = [f"sens_vs_{varying_param_name}"]
    for k,v in sorted(fixed_params.items()):
        abbr_k = k.replace('samples','s').replace('obstacles','o').replace('C','c')
        val_str = str(v).replace('.','p')
        filename_parts.append(f"{abbr_k}{val_str}")
    filename = "_".join(filename_parts) + ".pdf"
    
    output_path = os.path.join(output_dir, filename)
    plt.savefig(output_path, bbox_inches='tight', format='pdf')
    print(f"Saved: {output_path}")
    plt.close()

def plot_performance_scatter(summary_df_pivot, output_dir, metric_to_plot=DEFAULT_METRIC):
    if summary_df_pivot.empty or 'FMTx' not in summary_df_pivot.columns or 'RRTx' not in summary_df_pivot.columns:
        print("Scatter: Pivoted summary DF empty or missing planner columns.")
        return

    plt.figure(figsize=(9, 8)) # Slightly larger for legend
    
    # Prepare hue and size data from the multi-index
    df_for_scatter = summary_df_pivot.reset_index()
    hue_col, size_col = 'C', 'samples' # Example encoding

    scatter_plot = sns.scatterplot(
        data=df_for_scatter, x='RRTx', y='FMTx',
        hue=hue_col, size=size_col,
        palette='viridis', sizes=(30, 200), 
        alpha=0.8, style='obstacles' # Use obstacles for style if it's in index
    )
    
    # Determine plot limits for y=x line
    all_values = pd.concat([summary_df_pivot['FMTx'], summary_df_pivot['RRTx']]).dropna()
    if all_values.empty: 
        min_val, max_val = 0, 100 # Fallback
    else:
        min_val, max_val = all_values.min(), all_values.max()

    plt.plot([min_val, max_val], [min_val, max_val], 'k--', lw=1.5, label='y=x (Equal Performance)')
    
    plt.xlabel(f"RRTx - {metric_to_plot.replace('_', ' ').title()} (ms)")
    plt.ylabel(f"FMTx - {metric_to_plot.replace('_', ' ').title()} (ms)")
    plt.title(f"FMTx vs. RRTx ({metric_to_plot.replace('_', ' ').title()})\nPoints below y=x: FMTx is faster")
    plt.axis('equal')
    plt.grid(True, linestyle='--', alpha=0.6)
    
    # Improve legend
    handles, labels = scatter_plot.get_legend_handles_labels()
    # Separate legend items if too many, or adjust placement
    plt.legend(handles, labels, title="Parameters", bbox_to_anchor=(1.02, 1), loc='upper left', borderaxespad=0.)

    plt.tight_layout(rect=[0,0,0.82,1]) # Adjust for external legend
    
    output_path = os.path.join(output_dir, f"scatter_fmtx_vs_rrtx_{metric_to_plot}.pdf")
    plt.savefig(output_path, bbox_inches='tight', format='pdf')
    print(f"Saved: {output_path}")
    plt.close()

def plot_speedup_bars(summary_df_pivot, fixed_params: dict, x_axis_param: str, hue_param: str, 
                      output_dir, metric_to_plot=DEFAULT_METRIC):
    if summary_df_pivot.empty or 'FMTx' not in summary_df_pivot.columns or 'RRTx' not in summary_df_pivot.columns:
        print(f"Speedup Bars: Pivoted summary DF empty or missing planners.")
        return

    query_parts = [f"`{k}` == {v}" for k, v in fixed_params.items()]
    query_str = " & ".join(query_parts)
    plot_data_orig = summary_df_pivot.query(query_str) if query_str else summary_df_pivot.copy()
    
    if plot_data_orig.empty:
        print(f"No data for speedup: fixed={fixed_params}, x={x_axis_param}, hue={hue_param}")
        plt.close(); return
        
    plot_data = plot_data_orig.copy()
    plot_data['Speedup (RRTx/FMTx)'] = plot_data['RRTx'] / plot_data['FMTx']
    plot_data_reset = plot_data.reset_index()

    if x_axis_param not in plot_data_reset.columns or hue_param not in plot_data_reset.columns:
        print(f"Speedup Bars: x_axis_param '{x_axis_param}' or hue_param '{hue_param}' not found in data columns.")
        plt.close(); return

    plt.figure(figsize=(10, 6))
    sns.barplot(data=plot_data_reset, x=x_axis_param, y='Speedup (RRTx/FMTx)', hue=hue_param,
                palette='viridis_r', errorbar=None) # errorbar=None if data is already means/medians

    plt.axhline(1.0, color='black', linestyle='--', lw=1.5, label='Speedup = 1 (Equal Performance)')
    plt.ylabel(f"Speedup (RRTx {metric_to_plot.split('_')[0]} / FMTx {metric_to_plot.split('_')[0]})")
    plt.xlabel(x_axis_param.replace("_", " ").title())
    fixed_params_str = ", ".join([f"{k.replace('_',' ').title()}={v}" for k,v in fixed_params.items()])
    plt.title(f"FMTx Speedup over RRTx (Fixed: {fixed_params_str})\nHigher is better for FMTx")
    plt.legend(title=hue_param.replace("_", " ").title(), bbox_to_anchor=(1.02, 1), loc='upper left')
    plt.grid(True, axis='y', linestyle='--', alpha=0.7)
    plt.tight_layout(rect=[0,0,0.85,1])

    filename_parts = [f"speedup_x_{x_axis_param}_hue_{hue_param}"]
    for k,v in sorted(fixed_params.items()):
        abbr_k = k.replace('samples','s').replace('obstacles','o').replace('C','c')
        val_str = str(v).replace('.','p')
        filename_parts.append(f"fix_{abbr_k}{val_str}")
    filename = "_".join(filename_parts) + ".pdf"

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
        for C_val, C_data in samples_data.items():
            for obst, obst_data in C_data.items():
                for planner, trials_list in obst_data.items():
                    metrics = calculate_summary_metrics_from_trials(trials_list, grid_time_points)
                    summary_data_list.append({
                        'samples': samples, 'C': C_val, 'obstacles': obst, 'planner': planner, **metrics
                    })
    
    summary_df = pd.DataFrame(summary_data_list).dropna() # Drop rows with NaN metrics
    if summary_df.empty:
        print("Summary DataFrame is empty after processing/dropping NaNs. Exiting."); return
        
    print(f"\n--- Generated Summary DataFrame (NaNs dropped, {len(summary_df)} rows) ---")
    print(summary_df.head().to_string())
    summary_df.to_csv(os.path.join(OUTPUT_FIGURES_DIR, "summary_metrics_all_conditions.csv"), index=False)

    # Define specific discrete values for parameters (as per user request)
    # These should ideally be derived from summary_df.unique() to ensure they exist
    unique_samples = sorted(summary_df['samples'].unique())
    unique_C_values = sorted(summary_df['C'].unique())
    unique_obstacle_counts = sorted(summary_df['obstacles'].unique())
    
    print(f"\nUnique values found: Samples={unique_samples}, C={unique_C_values}, Obstacles={unique_obstacle_counts}")
    print(f"Generating plots using metric: {DEFAULT_METRIC}")

    # 1. Parameter Sensitivity Plots (for all combinations)
    print("\n--- Generating Parameter Sensitivity Plots ---")
    # Varying C
    for s_val in unique_samples:
        for o_val in unique_obstacle_counts:
            plot_parameter_sensitivity(summary_df, fixed_params={'samples': s_val, 'obstacles': o_val}, 
                                       varying_param_name='C', x_tick_values=unique_C_values,
                                       output_dir=OUTPUT_FIGURES_DIR, metric_to_plot=DEFAULT_METRIC)
    # Varying Obstacles
    for s_val in unique_samples:
        for c_val in unique_C_values:
            plot_parameter_sensitivity(summary_df, fixed_params={'samples': s_val, 'C': c_val}, 
                                       varying_param_name='obstacles', x_tick_values=unique_obstacle_counts,
                                       output_dir=OUTPUT_FIGURES_DIR, metric_to_plot=DEFAULT_METRIC)
    # Varying Samples
    for c_val in unique_C_values:
        for o_val in unique_obstacle_counts:
            plot_parameter_sensitivity(summary_df, fixed_params={'C': c_val, 'obstacles': o_val}, 
                                       varying_param_name='samples', x_tick_values=unique_samples,
                                       output_dir=OUTPUT_FIGURES_DIR, metric_to_plot=DEFAULT_METRIC)

    # 2. Overall Performance Scatter Plot
    print("\n--- Generating Overall Performance Scatter Plot ---")
    summary_df_pivot = summary_df.pivot_table(
        index=['samples', 'C', 'obstacles'], columns='planner', values=DEFAULT_METRIC
    ) # .reset_index() is done inside plot_performance_scatter after potential query
    
    # Drop rows where either FMTx or RRTx metric is NaN for a fair comparison in scatter
    summary_df_pivot_cleaned = summary_df_pivot.dropna(subset=['FMTx', 'RRTx'])

    if not summary_df_pivot_cleaned.empty:
        plot_performance_scatter(summary_df_pivot_cleaned, output_dir=OUTPUT_FIGURES_DIR, metric_to_plot=DEFAULT_METRIC)
    else:
        print("Not enough data for scatter plot after pivoting and cleaning NaNs.")

    # 3. Speedup Bar Charts (for all combinations)
    print("\n--- Generating Speedup Bar Charts ---")
    # Use summary_df_pivot_cleaned for speedup plots too
    if not summary_df_pivot_cleaned.empty:
        param_combinations = [
            ({'samples': s}, 'C', 'obstacles') for s in unique_samples # Fix S, X=C, Hue=O
        ] + [
            ({'samples': s}, 'obstacles', 'C') for s in unique_samples # Fix S, X=O, Hue=C
        ] + [
            ({'C': c}, 'samples', 'obstacles') for c in unique_C_values # Fix C, X=S, Hue=O
        ] + [
            ({'C': c}, 'obstacles', 'samples') for c in unique_C_values # Fix C, X=O, Hue=S
        ] + [
            ({'obstacles': o}, 'samples', 'C') for o in unique_obstacle_counts # Fix O, X=S, Hue=C
        ] + [
            ({'obstacles': o}, 'C', 'samples') for o in unique_obstacle_counts # Fix O, X=C, Hue=S
        ]

        for fixed_p, x_p, hue_p in param_combinations:
            plot_speedup_bars(summary_df_pivot_cleaned, fixed_params=fixed_p,
                              x_axis_param=x_p, hue_param=hue_p,
                              output_dir=OUTPUT_FIGURES_DIR, metric_to_plot=DEFAULT_METRIC)
    else:
        print("Not enough data for speedup bar charts after pivoting and cleaning NaNs.")

    print("\n--- All plot generation tasks complete ---")

if __name__ == "__main__":
    main()