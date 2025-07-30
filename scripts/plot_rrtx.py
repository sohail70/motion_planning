import pandas as pd
import matplotlib.pyplot as plt
import os
import re
import glob
import numpy as np
from scipy import stats
from itertools import combinations

# --- CONFIGURATION ---
# Set the path to the directory where your CSV files are stored.
path_to_data = '../build/'
# --- You shouldn't need to edit anything below this line. ---

def run_automatic_comparison():
    """
    Automatically finds and compares all 'mode' files in the data path.
    """
    # Use glob to find all files that match the mode pattern
    search_pattern = os.path.join(path_to_data, 'sim_rrtx_*_mode*.csv')
    found_files = glob.glob(search_pattern)

    if not found_files:
        print(f"Error: No mode files found matching the pattern '{search_pattern}'.")
        print("Please check the 'path_to_data' and that your simulation files exist.")
        return

    print("--- Found Files for Comparison ---")
    for f in found_files:
        print(f"  - {os.path.basename(f)}")

    all_data = {}
    for full_path in found_files:
        # Use regex to extract the mode number from the filename
        match = re.search(r'_mode(\d+)\.csv', full_path)
        if not match:
            continue

        mode_num = match.group(1)
        mode_name = f'Mode {mode_num}'

        try:
            df = pd.read_csv(full_path)
            # Handle files with one or two columns
            if 'duration_ms' not in df.columns:
                if len(df.columns) == 1:
                    df.columns = ['duration_ms'] # Rename single column
                else:
                    print(f"Warning: Skipping '{os.path.basename(full_path)}' - 'duration_ms' column not found.")
                    continue
            
            all_data[mode_name] = df['duration_ms'].dropna()
        except Exception as e:
            print(f"Error processing {os.path.basename(full_path)}: {e}")

    if len(all_data) < 2:
        print("\nCould not successfully load at least two different mode files for comparison. Exiting.")
        return

    # --- Generate Box Plot ---
    plt.style.use('seaborn-v0_8-whitegrid')
    fig, ax = plt.subplots(figsize=(10, 7))

    # Sort the data by mode number for consistent plotting order
    sorted_modes = sorted(all_data.keys(), key=lambda x: int(x.split(' ')[1]))
    data_to_plot = [all_data[mode] for mode in sorted_modes]

    box = ax.boxplot(data_to_plot, labels=sorted_modes, patch_artist=True, showfliers=False, medianprops={'color':'#A2142F', 'linewidth':2.5})

    colors = plt.cm.get_cmap('viridis', len(data_to_plot))
    for patch, color in zip(box['boxes'], colors.colors):
        patch.set_facecolor(color)

    ax.set_title('Comparison of Planner Update Times by Mode', fontsize=18, fontweight='bold')
    ax.set_ylabel('Update Duration (ms)', fontsize=14)
    ax.grid(True, which='major', axis='y', linestyle='--', linewidth=0.7)
    
    plot_filename = "mode_comparison_duration.pdf"
    plt.savefig(plot_filename, format="pdf", bbox_inches="tight")
    print(f"\n-> Comparison plot saved to '{plot_filename}'")
    plt.show()

    # --- Print Basic Statistics ---
    print("\n--- Basic Statistics (duration_ms) ---")
    print(f"{'Mode':<10} {'Mean':<15} {'Median':<15}")
    print("-" * 40)
    for mode in sorted_modes:
        data = all_data[mode]
        mean_val = f"{np.mean(data):.2f} ms"
        median_val = f"{np.median(data):.2f} ms"
        print(f"{mode:<10} {mean_val:<15} {median_val:<15}")

    # --- Perform Statistical Test (Mann-Whitney U) on all pairs ---
    if len(sorted_modes) >= 2:
        print("\n--- Statistical Comparison (Mann-Whitney U-test) ---")
        # Create all unique pairs for comparison
        mode_pairs = combinations(sorted_modes, 2)
        for p1, p2 in mode_pairs:
            stat, p_value = stats.mannwhitneyu(all_data[p1], all_data[p2], alternative='two-sided')
            print(f"{p1} vs {p2}: p-value = {p_value:.4g}", end="")
            if p_value < 0.05:
                print(" (Statistically Significant Difference)")
            else:
                print(" (No Significant Difference)")

if __name__ == "__main__":
    run_automatic_comparison()