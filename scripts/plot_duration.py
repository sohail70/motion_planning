import os
import glob
import re
import itertools
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from matplotlib.patches import Patch

# --- Configuration ---
FMTX_DIR = "../build/new/new_names/full_fmtx"
RRTX_DIR = "../build/new/new_names/full_rrtx"
OUTPUT_FIGURES_DIR = "./figures_final_boxplots"

# --- Data Loading and Parsing Functions ---
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
    match_fb = re.search(r"(\d+)obs_C(\d+p\d+)_sim_(fmtx|rrtx)_(\d+)samples", base.lower())
    if match_fb:
        obst_str, c_str, planner_str, samples_str = match_fb.groups()
        C = float(c_str.replace('p', '.'))
        obstacle_count = int(obst_str)
        planner = 'FMTx' if planner_str == 'fmtx' else 'RRTx'
        samples = int(samples_str)
        return C, obstacle_count, planner, samples
    raise ValueError(f"Cannot parse parameters from filename '{base}'.")

def load_all_data(data_dirs):
    data = {}
    for directory in data_dirs:
        if not os.path.isdir(directory):
            print(f"Warning: Directory not found - {directory}")
            continue
        for filepath in glob.glob(os.path.join(directory, "*.csv")):
            try:
                C, obst, planner, samples = parse_filename_params(filepath)
                df = pd.read_csv(filepath)
                if df.empty or 'duration_ms' not in df.columns:
                    continue
                data.setdefault(C, {}) \
                    .setdefault(obst, {}) \
                    .setdefault(samples, {}) \
                    .setdefault(planner, []).append(df)
            except Exception:
                continue
    return data

def create_per_trial_median_df(data):
    """
    Creates a DataFrame where each row is the median of one trial.
    This is the basis for robust statistical plotting.
    """
    records = []
    for C, cdata in data.items():
        for obst, odata in cdata.items():
            for samples, sdata in odata.items():
                for planner, trials in sdata.items():
                    for df_trial in trials:
                        median_val = df_trial['duration_ms'].median()
                        if pd.notna(median_val):
                            records.append({
                                'C': C,
                                'obstacles': obst,
                                'samples': samples,
                                'planner': planner,
                                'median_duration': median_val
                            })
    return pd.DataFrame(records)

# --- CONSOLIDATED PLOTTING FUNCTION ---
def plot_combined_grid_boxplots(df, output_dir):
    """
    Generates a single 3x3 grid of box plots with detailed x-axis labels.
    """
    if df.empty:
        print("DataFrame is empty. Cannot generate combined plot.")
        return

    # --- Create a combined label for the x-axis and set a specific order ---
    df['samples_k'] = (df['samples'] / 1000).astype(str).str.replace(r'\.0$', '', regex=True) + 'k'
    df['x_axis_label'] = df['planner'] + ', ' + df['samples_k']
    
    sample_order = sorted(df['samples_k'].unique(), key=lambda s: float(s.replace('k','')))
    planner_order = ['FMTx', 'RRTx']
    xaxis_order = [f"{p}, {s}" for s in sample_order for p in planner_order]
    df['x_axis_label'] = pd.Categorical(df['x_axis_label'], categories=xaxis_order, ordered=True)

    # Define the order and values for rows and columns
    obstacle_order = sorted(df['obstacles'].unique())
    c_value_order = sorted(df['C'].unique())
    
    fig, axes = plt.subplots(3, 3, figsize=(20, 18), sharex=True)
    
    palette = {'FMTx': '#1f77b4', 'RRTx': '#ff7f0e'} # Blue for FMTx, Orange/Red for RRTx

    for i, obst in enumerate(obstacle_order):
        for j, c_val in enumerate(c_value_order):
            ax = axes[i, j]
            subset = df[(df['obstacles'] == obst) & (df['C'] == c_val)]
            
            if subset.empty:
                ax.text(0.5, 0.5, 'No Data', ha='center', va='center', fontsize=12, color='grey')
                ax.set_xticks([])
            else:
                sns.boxplot(
                    data=subset,
                    x='x_axis_label',
                    y='median_duration',
                    hue='planner',
                    palette=palette,
                    ax=ax,
                    showfliers=False,
                    dodge=False 
                )
            
            if i == 0:
                ax.set_title(f'C = {c_val}', fontsize=14, pad=10)
            if j == 0:
                ax.set_ylabel(f'{obst} Obstacles\nMedian Time (ms)', fontsize=12)
            else:
                ax.set_ylabel('')

            ax.grid(True, linestyle='--', alpha=0.6)
            ax.legend_.remove()
            
            # Rotate labels for readability
            ax.tick_params(axis='x', labelrotation=45)

    # Set shared labels on the outside
    for ax in axes[-1, :]:
         ax.set_xlabel('Planner & Sample Size', fontsize=12)
    
    # Create a single, shared legend for the entire figure
    handles = [Patch(facecolor=palette[p], label=p) for p in planner_order]
    fig.legend(
        handles=handles,
        title="Planner",
        loc='lower center',
        ncol=len(planner_order),
        bbox_to_anchor=(0.5, 0.01),
        frameon=False,
        fontsize=12
    )
    
    fig.tight_layout(rect=[0, 0.05, 1, 1]) 

    filename = "combined_median_performance_boxplots.pdf"
    output_path = os.path.join(output_dir, filename)
    plt.savefig(output_path, bbox_inches='tight')
    plt.close()
    print(f"Saved: {output_path}")

# --- Script Entry Point ---
def main():
    data_loaded = load_all_data([FMTX_DIR, RRTX_DIR])
    if not data_loaded:
        print("No data loaded; exiting.")
    else:
        median_df = create_per_trial_median_df(data_loaded)
        if median_df.empty:
            print("No valid performance data to plot; exiting.")
            return
        
        os.makedirs(OUTPUT_FIGURES_DIR, exist_ok=True)
        print(f"\nOutputting combined plot to: {OUTPUT_FIGURES_DIR}")

        plot_combined_grid_boxplots(median_df, OUTPUT_FIGURES_DIR)

if __name__ == "__main__":
    main()
