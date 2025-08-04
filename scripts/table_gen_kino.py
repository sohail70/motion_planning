import os
import re
import pandas as pd
import numpy as np

def analyze_kinodynamic_data(base_dir):
    """
    Analyzes kinodynamic simulation data and returns a summary DataFrame.
    This function calculates the median duration for each trial, then aggregates
    those medians to get a final median and standard deviation.

    Args:
        base_dir (str): The base directory for kinodynamic data.

    Returns:
        pandas.DataFrame: A DataFrame with summary statistics.
    """
    state_spaces = ['R2T', 'dubins', 'thruster']
    obstacle_levels = ['10obs', '20obs']
    
    # This dictionary will hold lists of per-trial medians
    # Structure: {(space, obs, planner, samples): [median1, median2, ...]}
    per_trial_medians = {}

    for space in state_spaces:
        for obs_level in obstacle_levels:
            space_dir_path = os.path.join(base_dir, obs_level, space)
            if not os.path.exists(space_dir_path):
                continue

            for run_config_dir in os.listdir(space_dir_path):
                match = re.match(rf"{re.escape(space)}_(\d+)_(\d+)", run_config_dir)
                if match:
                    sample_size = int(match.group(1))
                    data_dir = os.path.join(space_dir_path, run_config_dir)
                    
                    for filename in os.listdir(data_dir):
                        if filename.endswith("_metrics.csv"):
                            file_path = os.path.join(data_dir, filename)
                            
                            planner_match = re.search(r"sim_(rrtx|fmtx)_", filename)
                            if not planner_match:
                                continue
                            planner = planner_match.group(1).upper()

                            try:
                                temp_df = pd.read_csv(file_path)
                                if not temp_df.empty and 'duration_ms' in temp_df.columns:
                                    median_duration = temp_df['duration_ms'].median()
                                    if pd.notna(median_duration):
                                        key = (space, obs_level, planner, sample_size)
                                        per_trial_medians.setdefault(key, []).append(median_duration)
                            except Exception as e:
                                print(f"Warning: Could not process {file_path}: {e}")

    # --- Aggregate the per-trial medians into final stats ---
    summary_records = []
    for (space, obs, planner, samples), medians_list in per_trial_medians.items():
        if medians_list:
            final_median = np.median(medians_list)
            final_std = np.std(medians_list)
            summary_records.append({
                'state_space': space,
                'obstacles': obs,
                'planner': planner,
                'samples': samples,
                'median': final_median,
                'std': final_std
            })
            
    return pd.DataFrame(summary_records)

def generate_latex_table(summary_df):
    """
    Generates the full LaTeX code for the results table.
    """
    if summary_df.empty:
        return "No data available to generate table."

    # Reshape the data for easy iteration
    pivot = summary_df.pivot_table(
        index=['obstacles', 'state_space', 'samples'],
        columns=['planner'],
        values=['median', 'std']
    )
    
    # --- Mapping for state space names ---
    space_map = {
        'R2T': ('Holonomic', r'$\mathbb{R}^{2}\times\mathbb{T}$'),
        'dubins': ('Dubins', r'$\mathbb{R}^{2}\times\mathbb{S}^{1}\times\mathbb{T}$'),
        'thruster': ('Thruster', r'$\mathbb{R}_x^2 \times \mathbb{R}_{\dot{x}}^2 \times \mathbb{T}$')
    }

    # --- Start LaTeX String ---
    latex_string = r'''\begin{table}[t]
    \centering
    \caption{Kinodynamic Replanning Performance. Median and standard deviation of per-trial median replanning times (ms) across 30 trials.}
    \label{tab:kinodynamic_replanning_times}
    \small
    \setlength{\tabcolsep}{4pt}
    \renewcommand{\arraystretch}{1.25}
    \begin{tabular}{|c|l|l|c|cc|cc|}
        \hline
        \multirow{2}{*}{\textbf{\#Obs.}} & \multirow{2}{*}{\textbf{System}} & \multirow{2}{*}{\textbf{State Space}} & \multirow{2}{*}{\textbf{Samples}} & \multicolumn{2}{c|}{\textbf{FMT\textsuperscript{x}}} & \multicolumn{2}{c|}{\textbf{RRT\textsuperscript{x}}} \\
        \cline{5-8}
         & & & & \textbf{Med.} & \textbf{Std.} & \textbf{Med.} & \textbf{Std.} \\
        \hline
'''
    # --- Generate Table Rows ---
    last_obs = None
    # Sort by obstacle count then by a custom order for state space
    custom_space_order = ['R2T', 'dubins', 'thruster']
    pivot = pivot.reindex(custom_space_order, level='state_space')

    for (obs, space, samples), row_data in pivot.iterrows():
        # Add a midrule between obstacle groups
        if obs != last_obs:
            if last_obs is not None:
                latex_string += r'      \hline' + '\n'
            # Use multirow for the obstacle count
            num_spaces = len(pivot.loc[obs].index)
            latex_string += f'      \\multirow{{{num_spaces}}}{{*}}{{{obs.replace("obs", "")}}} \n'
        
        last_obs = obs
        
        system_name, space_symbol = space_map.get(space, (space, space))
        sample_str = f"{(samples / 1000):g}k"

        # Get stats for both planners
        fmtx_med = row_data.get(('median', 'FMTX'), np.nan)
        fmtx_std = row_data.get(('std', 'FMTX'), np.nan)
        rrtx_med = row_data.get(('median', 'RRTX'), np.nan)
        rrtx_std = row_data.get(('std', 'RRTX'), np.nan)

        # Format numbers and bold the winner (lower median)
        fmtx_med_str = f"{fmtx_med:.2f}"
        rrtx_med_str = f"{rrtx_med:.2f}"
        
        if pd.notna(fmtx_med) and pd.notna(rrtx_med):
            if fmtx_med < rrtx_med:
                fmtx_med_str = f"\\textbf{{{fmtx_med_str}}}"
            else:
                rrtx_med_str = f"\\textbf{{{rrtx_med_str}}}"

        line = f'       & {system_name} & {space_symbol} & {sample_str} & {fmtx_med_str} & {fmtx_std:.2f} & {rrtx_med_str} & {rrtx_std:.2f} \\\\\n'
        latex_string += line

    # --- End LaTeX String ---
    latex_string += r'''        \hline
    \end{tabular}
\end{table}
'''
    return latex_string

if __name__ == "__main__":
    BASE_DIRECTORY = '../build/new/kinodynamic'

    print("--- Analyzing Kinodynamic Data ---")
    summary_df = analyze_kinodynamic_data(BASE_DIRECTORY)

    if not summary_df.empty:
        print("\n--- Summary DataFrame (Median and Std Dev of Per-Trial Medians) ---")
        print(summary_df.sort_values(['obstacles', 'state_space', 'samples', 'planner']).to_string())
        
        latex_code = generate_latex_table(summary_df)
        print("\n--- Generated LaTeX Code for Your Table ---")
        print("\nCopy and paste the following code into your .tex file:\n")
        print(latex_code)
    else:
        print("\nNo data found. Please check the BASE_DIRECTORY path and the data structure.")