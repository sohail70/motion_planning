#!/usr/bin/env python3

import glob
import os
import matplotlib.pyplot as plt
import numpy as np

def main():
    # 1) Find all CSV files matching sim_times_*.csv
    csv_files = sorted(glob.glob("../build/sim_times*.csv"))
    if not csv_files:
        print("No files matching 'sim_times_*.csv' found in the current directory.")
        return

    # 2) We can use a colormap to assign different colors
    #    e.g., 'viridis' or 'tab10'
    cmap = plt.cm.get_cmap("tab10", len(csv_files))  # or 'viridis'

    # 3) Read each file and plot
    for i, filename in enumerate(csv_files):
        durations = []
        with open(filename, 'r') as f:
            for line in f:
                line = line.strip()
                if line:
                    durations.append(float(line))

        # Use the file name (minus ".csv") as legend label
        label = os.path.splitext(os.path.basename(filename))[0]

        # Plot with a unique color from our colormap
        color = cmap(i)

        # We'll plot with a line + markers
        plt.plot(durations, marker='o', linestyle='-', color=color, label=label)

    # 4) Style and show
    plt.xlabel("Iteration")
    plt.ylabel("Duration (ms)")
    plt.title("Comparison of Planner Durations from Multiple CSV Files")
    plt.legend()
    plt.grid(True)
    plt.show()

if __name__ == "__main__":
    main()
