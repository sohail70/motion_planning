import glob
import os
import re

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import AutoMinorLocator


def get_planner_name(filename):
    base = os.path.basename(filename)
    if 'fmtx' in base.lower():
        planner = 'FMTx'
    elif 'rrtx' in base.lower():
        planner = 'RRTx'
    else:
        planner = os.path.splitext(base)[0]
    m = re.search(r"(\d+)samples_", base.lower())
    return planner, int(m.group(1)) if m else 0


def load_grouped_data(files):
    """
    Reads each CSV file and groups them by (samples → planner → list of DataFrames).
    """
    data = {}
    for f in files:
        planner, samples = get_planner_name(f)
        try:
            df = pd.read_csv(f)
            data.setdefault(samples, {}).setdefault(planner, []).append(df)
        except Exception as e:
            print(f"Error loading {f}: {e}")
    return data


def plot_combined_trajectory(data, samples_list, output_path="./figures/combined_replanning.pdf"):
    """
    Combines the trajectory+envelope plots for the specified sample sizes into a single figure.
    Each (samples, planner) pair will have:
      - a shaded envelope between the 5th and 95th percentiles of all trials, and
      - a median curve plotted on top.
    Uses distinct shades per sample size for each planner (blue shades for FMTx, red shades for RRTx).
    """

    # Build a common time grid (0 → 30 s at 1/30 s resolution)
    dt = 1/30  # ~0.03333 s
    grid = np.arange(0, 30 + dt/2, dt)  # ~901 points

    plt.figure(figsize=(12, 8))
    ax = plt.gca()

    # Sort sample sizes so shading goes from lighter (smallest) to darker (largest)
    samples_list_sorted = sorted(samples_list)

    # For FMTx: use Blues colormap; for RRTx: use Reds colormap
    fmtx_cmap = plt.cm.Blues
    rrtx_cmap = plt.cm.Reds

    n = len(samples_list_sorted)
    for idx, samples in enumerate(samples_list_sorted):
        if samples not in data:
            continue

        # Normalized index for colormap lookup
        t_norm = idx / float(max(n - 1, 1))

        for planner in ['FMTx', 'RRTx']:
            if planner not in data[samples]:
                continue

            # Choose color shade based on planner and index
            if planner == 'FMTx':
                line_color = fmtx_cmap(0.4 + 0.4 * t_norm)   # mid-to-dark blues
                fill_color = fmtx_cmap(0.2 + 0.4 * t_norm)   # lighter blues
            else:  # RRTx
                line_color = rrtx_cmap(0.4 + 0.4 * t_norm)   # mid-to-dark reds
                fill_color = rrtx_cmap(0.2 + 0.4 * t_norm)   # lighter reds

            # Collect all interpolated curves for this (samples, planner)
            interp_curves = []
            for df in data[samples][planner]:
                t = df['elapsed_s'].values
                y = df['duration_ms'].values
                # Interpolate durations onto the common grid; out-of-bounds → NaN
                interp = np.interp(grid, t, y, left=np.nan, right=np.nan)
                interp_curves.append(interp)

            stacked = np.vstack(interp_curves)  # shape = (n_trials, len(grid))

            # Percentiles and median across trials
            lo_05 = np.nanpercentile(stacked, 5, axis=0)
            hi_95 = np.nanpercentile(stacked, 95, axis=0)
            med = np.nanmedian(stacked, axis=0)

            # Label includes sample size + planner
            label_fill = f"{planner}, {samples} samples (5–95%)"
            label_median = f"{planner}, {samples} samples (median)"

            # Plot shaded envelope with higher opacity
            ax.fill_between(
                grid,
                lo_05,
                hi_95,
                facecolor=fill_color,
                alpha=0.5,
                label=label_fill
            )

            # Plot median line
            ax.plot(
                grid,
                med,
                color=line_color,
                linewidth=2,
                label=label_median
            )

    # Axis labels, title, and customizations
    ax.set_xlabel("Elapsed time (s)")
    ax.set_ylabel("Replanning Duration (ms)")
    ax.set_title("Combined Trajectories & 5th–95th Percentile Envelopes")
    ax.grid(True, which='both', linestyle='--', linewidth=0.5, alpha=0.7)
    ax.minorticks_on()
    ax.grid(True, which='minor', linestyle=':', linewidth=0.5, alpha=0.3)
    ax.xaxis.set_minor_locator(AutoMinorLocator(4))
    ax.yaxis.set_minor_locator(AutoMinorLocator(4))

    # Improved legend: more columns, smaller font, placed lower so it doesn't obstruct the x-axis
    plt.legend(
        loc='lower center',
        bbox_to_anchor=(0.5, -0.22),
        ncol=4,
        fontsize='x-small',
        columnspacing=0.5,
        handletextpad=0.5,
        frameon=True
    )

    plt.tight_layout()
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    plt.savefig(output_path, bbox_inches='tight', format='pdf')
    plt.show()


def main():
    # Adjust this glob path if needed to point to your CSV location
    files = sorted(glob.glob("../build/new/test/sim_*_*.csv"))
    if not files:
        print("No data files found in '../build/new/test/'.")
        return

    data = load_grouped_data(files)

    # Take the first four sample sizes (sorted ascending)
    samples_list = sorted(data.keys())[:4]

    # Only produce the combined figure
    plot_combined_trajectory(data, samples_list)


if __name__ == '__main__':
    main()
