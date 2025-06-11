import os
import glob
import re
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from matplotlib.patches import Patch

# --- Configuration ---
# Please verify these paths are correct
FMTX_DIR = "../build/new/new_names/full_fmtx"
RRTX_DIR = "../build/new/new_names/full_rrtx"
OUTPUT_FIGURES_DIR = "./figures_median_performance" # Directory for the correct plots

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
    CHANGED: Creates a DataFrame where each row represents one trial,
    and the value is the median duration of that trial.
    """
    records = []
    for C, cdata in data.items():
        for obst, odata in cdata.items():
            for samples, sdata in odata.items():
                for planner, trials in sdata.items():
                    for df_trial in trials:
                        records.append({
                            'C': C,
                            'obstacles': obst,
                            'samples': samples,
                            'planner': planner,
                            'median_duration': df_trial['duration_ms'].median()
                        })
    return pd.DataFrame(records)

def plot_median_performance_boxplots(df, c_value, output_dir):
    """
    CHANGED: Generates a box plot for a specific C value, showing the 
    distribution of per-trial median replanning times.
    """
    if df.empty:
        print(f"DataFrame for C={c_value} plot is empty.")
        return

    palette = {'FMTx': '#1f77b4', 'RRTx': '#ff7f0e'}

    # Create a grid of plots, with each row representing an obstacle count
    g = sns.catplot(
        data=df,
        x='samples',
        y='median_duration', # CHANGED: Plotting the per-trial median duration
        hue='planner',
        row='obstacles',
        kind='box',
        height=4,
        aspect=1.5,
        legend=False,
        palette=palette,
        sharey=False # Allow each obstacle count to have its own y-axis scale
    )

    g.set_axis_labels("Sample Size (n)", "Median Replanning Time per Trial (ms)")
    g.set_titles("Obstacles: {row_name}")
    g.fig.suptitle(f'Median Replanning Performance (C = {c_value})', y=1.03)

    # Use your corrected legend method
    handles = [Patch(facecolor=palette[p], label=p) for p in ['FMTx', 'RRTx']]
    g.fig.legend(
        handles=handles,
        title="Planner",
        loc='lower center',
        ncol=2,
        bbox_to_anchor=(0.5, -0.05),
        frameon=False
    )

    g.fig.tight_layout(rect=[0, 0.05, 1, 0.97])

    filename = f"median_performance_C_{str(c_value).replace('.', 'p')}.pdf"
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
        # Create the DataFrame of per-trial medians
        median_df = create_per_trial_median_df(data_loaded)
        if median_df.empty:
            print("No performance data to plot; exiting.")
            return
        
        os.makedirs(OUTPUT_FIGURES_DIR, exist_ok=True)
        print(f"\nOutputting separate PDFs to: {OUTPUT_FIGURES_DIR}")

        # Loop through each C value and create a separate plot for it
        for c_val in sorted(median_df['C'].unique()):
            print(f"\n--- Generating plot for C = {c_val} ---")
            df_subset = median_df[median_df['C'] == c_val]
            plot_median_performance_boxplots(df_subset, c_val, OUTPUT_FIGURES_DIR)

if __name__ == "__main__":
    main()
