import glob
import os
import re
import math

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import AutoMinorLocator


def parse_filename_params(filename):
    """
    Given a filename like "1_5_30_sim_fmtx_2500samples_26_5_2025_17_3_6_timed.csv",
    parse out:
      - C (as float, e.g. 1.5)
      - obstacle_count (int, e.g. 30)
      - planner (string: "FMTx" or "RRTx")
      - samples (int, e.g. 2500)
    """
    base = os.path.basename(filename)
    tokens = base.split('_')
    try:
        C = float(tokens[0] + "." + tokens[1])
        obstacle_count = int(tokens[2])
        planner_token = tokens[4].lower()
        if planner_token == 'fmtx':
            planner = 'FMTx'
        elif planner_token == 'rrtx':
            planner = 'RRTx'
        else:
            planner = planner_token.upper()
        m = re.match(r"(\d+)samples", tokens[5].lower())
        samples = int(m.group(1)) if m else None
    except Exception as e:
        raise ValueError(f"Cannot parse parameters from filename '{filename}': {e}")
    return C, obstacle_count, planner, samples


def load_all_data(fmtx_dir, rrtx_dir):
    """
    Walks both directories, collects all CSVs, and groups them into a nested dict:
      data[samples][C][obstacle_count][planner] = list of DataFrames
    """
    data = {}
    for directory in (fmtx_dir, rrtx_dir):
        pattern = os.path.join(directory, "*.csv")
        for filepath in glob.glob(pattern):
            try:
                C, obst, planner, samples = parse_filename_params(filepath)
            except ValueError as e:
                print(e)
                continue
            try:
                df = pd.read_csv(filepath)
            except Exception as e:
                print(f"Error reading '{filepath}': {e}")
                continue
            data.setdefault(samples, {}) \
                .setdefault(C, {}) \
                .setdefault(obst, {}) \
                .setdefault(planner, []).append(df)
    return data


def interpolate_trials_to_grid(df_list, grid):
    """
    Given a list of DataFrames (each with 'elapsed_s' and 'duration_ms'),
    interpolate each trial onto the common 'grid' (1D numpy array).
    Returns a 2D numpy array of shape (n_trials, len(grid)).
    """
    interp_curves = []
    for df in df_list:
        t = df['elapsed_s'].values
        y = df['duration_ms'].values
        interp = np.interp(grid, t, y, left=np.nan, right=np.nan)
        interp_curves.append(interp)
    if len(interp_curves) == 0:
        return None
    return np.vstack(interp_curves)


def plot_envelopes_for_group(group_dict, title, output_path):
    """
    Given a dict of the form:
        group_dict[label] = list of DataFrames
    where each label might be e.g. "FMTx, C=1.0" or "RRTx, obst=20",
    compute the 5th-95th percentile envelope and median for each label, and plot them on one figure.
    FMTx uses shades of blue with increasing intensity; RRTx uses shades of red.
    The legend is placed just below the plot in exactly four rows.
    """
    # Build common time grid 0 → 30s at 1/30s resolution
    dt = 1 / 30.0
    grid = np.arange(0, 30 + dt / 2, dt)

    # Separate labels by planner, sorting numerically by parameter
    fmtx_labels = sorted(
        [lab for lab in group_dict if lab.startswith("FMTx")],
        key=lambda s: float(s.split("C=")[1]) if "C=" in s else float(s.split("obst=")[1])
    )
    rrtx_labels = sorted(
        [lab for lab in group_dict if lab.startswith("RRTx")],
        key=lambda s: float(s.split("C=")[1]) if "C=" in s else float(s.split("obst=")[1])
    )

    n_fmtx = len(fmtx_labels)
    n_rrtx = len(rrtx_labels)

    # Prepare colormaps
    fmtx_cmap = plt.cm.Blues
    rrtx_cmap = plt.cm.Reds

    fig, ax = plt.subplots(figsize=(10, 6))

    plotted_handles = []  # to collect handles in plotting order
    plotted_labels = []

    # Plot FMTx curves first
    for idx, label in enumerate(fmtx_labels):
        df_list = group_dict[label]
        stacked = interpolate_trials_to_grid(df_list, grid)
        if stacked is None:
            continue

        lo5 = np.nanpercentile(stacked, 5, axis=0)
        hi95 = np.nanpercentile(stacked, 95, axis=0)
        med = np.nanmedian(stacked, axis=0)

        # Choose a blue shade: from lighter (idx=0) to darker (idx=n_fmtx-1)
        if n_fmtx > 1:
            shade = 0.2 + 0.6 * (idx / (n_fmtx - 1))  # range [0.2, 0.8]
        else:
            shade = 0.5
        fill_color = fmtx_cmap(shade)
        line_color = fmtx_cmap(min(shade + 0.2, 1.0))

        h_fill = ax.fill_between(grid, lo5, hi95, color=fill_color, alpha=0.4)
        h_line, = ax.plot(grid, med, color=line_color, linewidth=2)

        # Store handles so median appears right after its envelope
        plotted_handles.extend([h_fill, h_line])
        plotted_labels.extend([f"{label} (5–95%)", f"{label} (median)"])

    # Plot RRTx curves next
    for idx, label in enumerate(rrtx_labels):
        df_list = group_dict[label]
        stacked = interpolate_trials_to_grid(df_list, grid)
        if stacked is None:
            continue

        lo5 = np.nanpercentile(stacked, 5, axis=0)
        hi95 = np.nanpercentile(stacked, 95, axis=0)
        med = np.nanmedian(stacked, axis=0)

        # Choose a red shade: from lighter (idx=0) to darker (idx=n_rrtx-1)
        if n_rrtx > 1:
            shade = 0.2 + 0.6 * (idx / (n_rrtx - 1))  # range [0.2, 0.8]
        else:
            shade = 0.5
        fill_color = rrtx_cmap(shade)
        line_color = rrtx_cmap(min(shade + 0.2, 1.0))

        h_fill = ax.fill_between(grid, lo5, hi95, color=fill_color, alpha=0.4)
        h_line, = ax.plot(grid, med, color=line_color, linewidth=2)

        plotted_handles.extend([h_fill, h_line])
        plotted_labels.extend([f"{label} (5–95%)", f"{label} (median)"])

    ax.set_xlabel("Elapsed time (s)")
    ax.set_ylabel("Replanning Duration (ms)")
    ax.set_title(title)
    ax.grid(True, which='both', linestyle='--', linewidth=0.5, alpha=0.7)
    ax.minorticks_on()
    ax.grid(True, which='minor', linestyle=':', linewidth=0.5, alpha=0.3)
    ax.xaxis.set_minor_locator(AutoMinorLocator(4))
    ax.yaxis.set_minor_locator(AutoMinorLocator(4))

    # Place legend below the plot in exactly four rows
    handles = plotted_handles
    labels = plotted_labels
    n_items = len(labels)
    # Determine number of columns to make exactly 4 rows:
    #   ncol = ceil(n_items / 4)
    ncol = math.ceil(n_items / 4)

    fig.legend(
        handles,
        labels,
        loc='lower center',
        ncol=ncol,
        fontsize='small',
        frameon=True,
        bbox_to_anchor=(0.5, -0.10),
        columnspacing=0.4,
        handletextpad=0.4
    )

    # Adjust bottom margin so legend is closer but still below plot
    plt.subplots_adjust(bottom=0.28)

    plt.tight_layout()
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    plt.savefig(output_path, bbox_inches='tight', format='pdf')
    plt.close()


def main():
    # Directories containing the renamed CSVs
    fmtx_dir = "../build/new/new_names/full_fmtx"
    rrtx_dir = "../build/new/new_names/full_rrtx"

    # Load all data into a nested dictionary
    data = load_all_data(fmtx_dir, rrtx_dir)

    # Gather lists of unique values for samples, C, and obstacle counts
    all_samples = sorted(data.keys())
    all_Cs = sorted({C for s in data for C in data[s].keys()})
    all_obsts = sorted({o for s in data for C in data[s] for o in data[s][C].keys()})

    figures_dir = "./figures"
    os.makedirs(figures_dir, exist_ok=True)

    # 1) For each sample size and each obstacle count, compare different C's (for both planners)
    for samples in all_samples:
        for obst in all_obsts:
            group_dict = {}
            for C in all_Cs:
                if samples in data and C in data[samples] and obst in data[samples][C]:
                    for planner in ("FMTx", "RRTx"):
                        trials = data[samples][C][obst].get(planner, [])
                        if not trials:
                            continue
                        label = f"{planner}, C={C:.1f}"
                        group_dict[label] = trials
            if not group_dict:
                continue

            title = (
                f"Sample = {samples}, Obstacles = {obst}\n"
                "Comparison across C-values (1.0, 1.5, 2.0), both planners"
            )
            out_file = os.path.join(figures_dir, f"s{samples}_o{obst}_compare_C.pdf")
            plot_envelopes_for_group(group_dict, title, out_file)

    # 2) For each sample size and each C, compare different obstacle counts (for both planners)
    for samples in all_samples:
        for C in all_Cs:
            group_dict = {}
            for obst in all_obsts:
                if samples in data and C in data[samples] and obst in data[samples][C]:
                    for planner in ("FMTx", "RRTx"):
                        trials = data[samples][C][obst].get(planner, [])
                        if not trials:
                            continue
                        label = f"{planner}, obst={obst}"
                        group_dict[label] = trials
            if not group_dict:
                continue

            title = (
                f"Sample = {samples}, C = {C:.1f}\n"
                "Comparison across obstacle counts (10, 20, 30), both planners"
            )
            out_file = os.path.join(figures_dir, f"s{samples}_C{C:.1f}_compare_obst.pdf")
            plot_envelopes_for_group(group_dict, title, out_file)

    print("All figures have been generated and saved to './figures'.")


if __name__ == "__main__":
    main()
