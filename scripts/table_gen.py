import os
import glob
import re
import pandas as pd
import numpy as np

# --- Configuration ---
# Please verify these paths are correct relative to where you run the script.
FMTX_DIR = "../build/new/new_names/full_fmtx" 
RRTX_DIR = "../build/new/new_names/full_rrtx" 

# --- Data Loading and Parsing Functions ---

def parse_filename_params(filename):
    """Extracts experimental parameters from a filename."""
    base = os.path.basename(filename)
    # This regex is designed to match your specific filename format.
    match = re.match(r"(\d+)_(\d+)_(\d+)_sim_(fmtx|rrtx)_(\d+)samples", base.lower())
    if match:
        c_major, c_minor, obst_str, planner_str, samples_str = match.groups()
        C = float(f"{c_major}.{c_minor}")
        obstacle_count = int(obst_str)
        planner = 'FMTx' if planner_str == 'fmtx' else 'RRTx'
        samples = int(samples_str)
        return C, obstacle_count, planner, samples
    raise ValueError(f"Filename {base} did not match expected format.")

def load_all_data(data_dirs):
    """Loads all CSV files from a list of directories into a nested dictionary."""
    data = {}
    for directory in data_dirs:
        if not os.path.isdir(directory):
            print(f"Warning: Directory not found - {directory}")
            continue
        
        pattern = os.path.join(directory, "*.csv")
        filepaths = glob.glob(pattern)
        print(f"Found {len(filepaths)} files in {directory}...")

        for filepath in filepaths:
            try:
                C, obst, planner, samples = parse_filename_params(filepath)
                df = pd.read_csv(filepath)
                if df.empty or 'duration_ms' not in df.columns: continue
                # Group data by parameters
                data.setdefault(samples, {}).setdefault(C, {}).setdefault(obst, {}).setdefault(planner, []).append(df)
            except Exception as e:
                # print(f"Could not process '{filepath}': {e}") # Enable for debugging
                continue
    return data

def calculate_per_trial_summary(df_list):
    """
    Correctly calculates summary stats based on per-trial medians.
    This is the standard, robust method.
    """
    # Get the median replanning time for each individual trial.
    per_trial_medians = [df['duration_ms'].median() for df in df_list if not df.empty]

    if not per_trial_medians:
        return {'median': np.nan, 'std': np.nan}
        
    # The final metrics are the median and std dev of that list of per-trial medians.
    return {
        'median': np.median(per_trial_medians), 
        'std': np.std(per_trial_medians)
    }

def generate_latex_table(summary_df):
    """
    Generates the full LaTeX code for the results table from the summary DataFrame.
    """
    # Reshape the data to match the table structure
    pivot = summary_df.pivot_table(
        index=['obstacles', 'samples'],
        columns=['C', 'planner'],
        values=['median', 'std']
    )
    
    # --- Start LaTeX String ---
    latex_string = r'''\begin{table*}[t]
	\centering
	\caption{Replanning Times for FMT\textsuperscript{x} and RRT\textsuperscript{x} (median and std.\ dev., ms; 30 trials)}
	\label{tab:replanning_times_subcols}
	\small
	\setlength{\tabcolsep}{3pt}
	\renewcommand{\arraystretch}{1.2}
	\begin{tabular}{|l|c|cc|cc|cc|cc|cc|cc|}
		\hline
		& & \multicolumn{2}{r@{\hspace{1pt}}}{$C=$} & \multicolumn{2}{l@{\hspace{1pt}}|}{$1.0$}
		& \multicolumn{2}{r@{\hspace{1pt}}}{$C=$} & \multicolumn{2}{l@{\hspace{1pt}}|}{$1.5$}
		& \multicolumn{2}{r@{\hspace{1pt}}}{$C=$} & \multicolumn{2}{l@{\hspace{1pt}}|}{$2.0$} \\
		\cline{3-14} 
		\#Obs. & Samples 
		& \multicolumn{2}{c}{FMT\textsuperscript{x}} & \multicolumn{2}{c|}{RRT\textsuperscript{x}}
		& \multicolumn{2}{c}{FMT\textsuperscript{x}} & \multicolumn{2}{c|}{RRT\textsuperscript{x}}
		& \multicolumn{2}{c}{FMT\textsuperscript{x}} & \multicolumn{2}{c|}{RRT\textsuperscript{x}} \\
		\cline{3-14} 
		& 
		& Med. & Std. & Med. & Std. 
		& Med. & Std. & Med. & Std. 
		& Med. & Std. & Med. & Std. \\
		\hline
'''
    # --- Generate Table Rows ---
    last_obs = None
    for (obs, samples), row_data in pivot.iterrows():
        if obs != last_obs:
            if last_obs is not None:
                latex_string += r'		\hline' + '\n'
            latex_string += f'		\\multirow{{4}}{{*}}{{{obs}}} \n'
        last_obs = obs
        
        # Format sample size (e.g., 2500 -> 2.5k)
        sample_str = f"{samples/1000:.1f}k".replace(".0", "")
        
        line = f'		& {sample_str} '
        for C in sorted(pivot.columns.get_level_values('C').unique()):
            for planner in ['FMTx', 'RRTx']:
                med = row_data.get(('median', C, planner), np.nan)
                std = row_data.get(('std', C, planner), np.nan)
                line += f'& {med:6.2f} & {std:4.2f} '
        line += r'\\' + '\n'
        latex_string += line

    # --- End LaTeX String ---
    latex_string += r'''		\hline
	\end{tabular}
\end{table*}
'''
    return latex_string

def main():
    print("--- Data Analysis and LaTeX Table Generation ---")
    data_loaded = load_all_data([FMTX_DIR, RRTX_DIR])

    if not data_loaded:
        print("No data was loaded. Exiting.")
        return

    # --- Calculate summary statistics using the correct per-trial method ---
    summary_data_list = []
    for samples, C_data in data_loaded.items():
        for C_val, obst_data in C_data.items():
            for obst, planner_data in obst_data.items():
                for planner, trials_list in planner_data.items():
                    metrics = calculate_per_trial_summary(trials_list)
                    summary_data_list.append({
                        'samples': samples, 'C': C_val, 'obstacles': obst, 
                        'planner': planner, **metrics
                    })
    
    summary_df = pd.DataFrame(summary_data_list).dropna()
    
    if summary_df.empty:
        print("Summary DataFrame is empty. Cannot generate table.")
        return

    print("\n--- Summary DataFrame (Median and Std Dev of Per-Trial Medians) ---")
    print(summary_df.sort_values(['obstacles', 'samples', 'C', 'planner']).to_string())
    
    # --- Generate and Print the LaTeX Table ---
    latex_code = generate_latex_table(summary_df)
    print("\n--- Generated LaTeX Code for Your Table ---")
    print("\nCopy and paste the following code into your .tex file:\n")
    print(latex_code)


if __name__ == "__main__":
    main()
