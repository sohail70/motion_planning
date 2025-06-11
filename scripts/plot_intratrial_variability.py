import os
import glob
import re
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from matplotlib.patches import Patch

# --- Configuration ---
FMTX_DIR = "../build/new/new_names/full_fmtx"
RRTX_DIR = "../build/new/new_names/full_rrtx"
OUTPUT_FIGURES_DIR = "./figures_variability"

# --- Data Loading and Parsing ---
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

def create_variability_df(data):
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
                            'intra_trial_std': df_trial['duration_ms'].std()
                        })
    return pd.DataFrame(records)

def plot_intra_trial_variability(df):
    if df.empty:
        print("DataFrame for variability plot is empty.")
        return

    # Use a fixed palette so colors are consistent:
    palette = {'FMTx': '#1f77b4', 'RRTx': '#ff7f0e'}

    g = sns.catplot(
        data=df,
        x='samples',
        y='intra_trial_std',
        hue='planner',
        col='C',
        row='obstacles',
        kind='box',
        height=4,
        aspect=1.2,
        legend=False,
        palette=palette,
        sharey=False
    )

    g.set_axis_labels("Sample Size (n)", "Intra-Trial Std Dev (ms)")
    g.set_titles("Obstacles: {row_name} | C: {col_name}")

    # Build legend handles manually
    handles = [Patch(facecolor=palette[p], label=p) for p in ['FMTx', 'RRTx']]
    g.fig.legend(
        handles=handles,
        title="Planner",
        loc='lower center',
        ncol=2,
        bbox_to_anchor=(0.5, -0.02),
        frameon=False
    )

    # Make room for the legend
    g.fig.tight_layout(rect=[0, 0.02, 1, 1])

    os.makedirs(OUTPUT_FIGURES_DIR, exist_ok=True)
    output_path = os.path.join(OUTPUT_FIGURES_DIR, "intra_trial_variability_boxplots.pdf")
    plt.savefig(output_path, bbox_inches='tight')
    plt.close()
    print(f"Saved: {output_path}")

# --- Script Entry Point ---
if __name__ == "__main__":
    data_loaded = load_all_data([FMTX_DIR, RRTX_DIR])
    if not data_loaded:
        print("No data loaded; exiting.")
    else:
        variability_df = create_variability_df(data_loaded)
        if variability_df.empty:
            print("No variability data; exiting.")
        else:
            plot_intra_trial_variability(variability_df)

