import pandas as pd
import glob
import os
import re
import numpy as np
import matplotlib.pyplot as plt
from pandas.errors import EmptyDataError

# Publication-ready plot styling
plt.rcParams.update({
    'font.family': 'serif',
    'font.serif': ['Times New Roman', 'DejaVu Serif', 'serif'],
    'axes.labelsize': 12,
    'axes.titlesize': 14,
    'legend.fontsize': 11,
    'xtick.labelsize': 11,
    'ytick.labelsize': 11,
    'figure.dpi': 300
})

BUILD_DIR = "."  

FILENAME_PATTERN = re.compile(r"sim_([A-Za-z0-9]+)_([A-Za-z0-9_]+)_seed_(\d+)_(\d{8}_\d{6})_metrics\.csv")
FALLBACK_PATTERN = re.compile(r"sim_([A-Za-z0-9]+)_([A-Za-z0-9_]+)_(\d{8}_\d{6})_metrics\.csv")

def load_data(directory):
    if not os.path.exists(directory): return {}
    files = glob.glob(os.path.join(directory, "sim_*_metrics.csv"))
    scenarios = {}

    for filepath in files:
        filename = os.path.basename(filepath)
        match = FILENAME_PATTERN.match(filename)
        scenario = "default"
        
        if match:
            planner_raw = match.group(1)
            scenario = match.group(2)
        else:
            fallback = FALLBACK_PATTERN.match(filename)
            if fallback:
                planner_raw = fallback.group(1)
                scenario = fallback.group(2)
            else:
                parts = filename.replace("sim_", "").replace("_metrics.csv", "").split("_")
                planner_raw = parts[0]
                if len(parts) > 2: scenario = "_".join(parts[1:-2])
        
        # Skip anytime algorithms
        if "ANY" in planner_raw: 
            continue 
        
        # Clean names for the paper
        planner_clean = planner_raw.replace("Kinodynamic", "").replace("PRMStarDStarLite", "DLITE").replace("FMTX", "FMTX")
        
        try:
            df = pd.read_csv(filepath)
            if df.empty: continue
            df.replace([np.inf, -np.inf], np.nan, inplace=True)
            
            if scenario not in scenarios: scenarios[scenario] = {}
            if planner_clean not in scenarios[scenario]: scenarios[scenario][planner_clean] = []
            scenarios[scenario][planner_clean].append(df)
        except EmptyDataError: pass
        except Exception as e: print(f"Warning: Could not load {filename}: {e}")
            
    return scenarios

# --- NEW FUNCTION: Safely remove the terminal log entry with the Dubins spike ---
def drop_terminal_goal_row(df):
    if len(df) > 1 and df['update_ms'].iloc[-1] == 0.0 and df['plan_ms'].iloc[-1] == 0.0 and df['elapsed_s'].iloc[-1] > 0:
        return df.iloc[:-1].copy()
    return df.copy()
# --------------------------------------------------------------------------------

def save_latency_plot(scenario_name, planners_data):
    if not planners_data: return
    planners, init_plan, repair_lat = [], [], []

    for planner, dfs in planners_data.items():
        valid_dfs = [df for df in dfs if not df.empty]
        if not valid_dfs: continue
        
        merged_df = pd.concat(valid_dfs, ignore_index=True)
        planners.append(planner)

        # Initial Plan time (from the cold-start frame)
        df_init = merged_df[merged_df['elapsed_s'] == 0.0]
        init_plan.append(df_init['plan_ms'].mean() if not df_init.empty else 0)

        # Repair latency (only when obstacles move)
        df_event = merged_df[(merged_df['elapsed_s'] > 0.0) & (merged_df['update_ms'] > 0.001)]
        if not df_event.empty:
            repair_lat.append(df_event['total_latency_ms'].mean())
        else:
            repair_lat.append(0)

    if not planners: return
    x = np.arange(len(planners))
    width = 0.4
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10, 5))

    c_init, c_repair = '#2ca02c', '#d95f02'

    # Plot 1: Initial Planning Time
    bars1 = ax1.bar(x, init_plan, width, color=c_init, edgecolor='black', hatch='\\\\')
    ax1.set_ylabel('Average Latency (ms)')
    ax1.set_title('Graph Construction ($T_{init}$)')
    ax1.set_xticks(x)
    ax1.set_xticklabels(planners)
    ax1.grid(axis='y', linestyle=':', alpha=0.7)
    for bar in bars1:
        val = bar.get_height()
        if val > 0: ax1.text(bar.get_x() + bar.get_width()/2., val * 1.05, f"{val:.1f}", ha='center', va='bottom', fontweight='bold')

    # Plot 2: Dynamic Repair Time
    bars2 = ax2.bar(x, repair_lat, width, color=c_repair, edgecolor='black', hatch='//')
    ax2.set_title('Dynamic Replan Event ($T_{repair}$)')
    ax2.set_xticks(x)
    ax2.set_xticklabels(planners)
    ax2.grid(axis='y', linestyle=':', alpha=0.7)
    for bar in bars2:
        val = bar.get_height()
        if val > 0: ax2.text(bar.get_x() + bar.get_width()/2., val * 1.05, f"{val:.1f}", ha='center', va='bottom', fontweight='bold')

    plt.suptitle(f"Fixed-Graph Latency Breakdown: {scenario_name}", y=1.05)
    plt.tight_layout()
    
    out_path = os.path.join(BUILD_DIR, f"plot_fixed_latency_{scenario_name}.png")
    plt.savefig(out_path, bbox_inches='tight')
    plt.close()
    print(f"[Saved Plot] {out_path}")
def analyze_group_statistics(scenario_name, planners_data):
    print(f"\n{'='*155}")
    print(f" FIXED-GRAPH ANALYSIS: {scenario_name} (Successful runs only)")
    print(f"{'='*155}")
    
    summary_data = []

    for planner, dfs in planners_data.items():
        valid_dfs = [df for df in dfs if not df.empty]
        if not valid_dfs: continue
        
        num_seeds = len(valid_dfs)
        successful_dfs = []
        success_count = 0
        
        for df in valid_dfs:
            if 'crashed' in df.columns and not (df['crashed'] == 1).any():
                success_count += 1; successful_dfs.append(drop_terminal_goal_row(df))
            elif 'collision_count' in df.columns and df['collision_count'].max() == 0:
                success_count += 1; successful_dfs.append(drop_terminal_goal_row(df))
            elif 'crashed' not in df.columns and 'collision_count' not in df.columns:
                success_count += 1; successful_dfs.append(drop_terminal_goal_row(df))
                
        succ_rate = (success_count / num_seeds) * 100 if num_seeds > 0 else 0.0
        if not successful_dfs:
            summary_data.append({"Planner": planner, "Succ(%)": f"{succ_rate:.0f}%", "Status": "All runs failed"})
            continue
            
        merged_df = pd.concat(successful_dfs, ignore_index=True)
        if 'update_ms' not in merged_df.columns: continue

        # Initial Phase (Graph Construction)
        df_init = merged_df[merged_df['elapsed_s'] == 0.0]
        t_set = df_init['setup_ms'].mean() if 'setup_ms' in df_init.columns else 0.0
        t_init = df_init['plan_ms'].mean() if not df_init.empty else 0.0

        # Dynamic Repair Phase (No new samples, only graph repair)
        df_event = merged_df[(merged_df['elapsed_s'] > 0.0) & (merged_df['update_ms'] > 0.001)]
        
        if not df_event.empty:
            t_repair_avg = df_event['total_latency_ms'].mean()
            t_repair_p99 = df_event['total_latency_ms'].quantile(0.99)
            obs_chk   = df_event['obstacle_checks'].mean()
            queue_ops = df_event['queue_operations'].mean() if 'queue_operations' in df_event.columns else 0.0
            
            # --- YOUR ORIGINAL LOGIC IS BACK ---
            # Averages the path cost across all replanning events
            avg_cost  = df_event['path_cost'].dropna().mean()
        else:
            t_repair_avg = t_repair_p99 = obs_chk = queue_ops = 0.0
            # Fallback to initial plan cost if no repair events happened
            avg_cost = df_init['path_cost'].dropna().mean() if not df_init.empty else 0.0

        tree_sz = merged_df['tree_size'].max()
        
        # Calculate isolated nodes string with percentage
        if 'isolated_nodes' in merged_df.columns:
            iso_count = merged_df['isolated_nodes'].max()
            iso_pct = (iso_count / tree_sz) * 100 if tree_sz > 0 else 0.0
            iso_str = f"{iso_count:.0f} ({iso_pct:.1f}%)"
        else:
            iso_str = "0 (0.0%)"

        deg_in  = merged_df['avg_deg_in'].mean() if 'avg_deg_in' in merged_df.columns else 0.0
        deg_out = merged_df['avg_deg_out'].mean() if 'avg_deg_out' in merged_df.columns else 0.0

        summary_data.append({
            "Planner": planner,
            "Succ(%)": f"{succ_rate:.0f}%",
            "T_set(ms)": f"{t_set:.1f}",
            "T_init(ms)": f"{t_init:.1f}",
            "T_repair(ms)": f"{t_repair_avg:.2f}",
            "Rep_p99(ms)": f"{t_repair_p99:.1f}",
            "Obs_Chk": f"{obs_chk:.0f}",
            "Q_Ops": f"{queue_ops:.0f}",
            "Path_Cost": f"{avg_cost:.2f}", # EXACTLY how you had it: e.g., 81.45
            "|V|": f"{tree_sz:.0f}",
            "Iso_Nodes": iso_str,
            "Deg(I/O)": f"{deg_in:.1f}/{deg_out:.1f}"
        })

    if summary_data:
        df_out = pd.DataFrame(summary_data)
        
        # EXACTLY your original table header
        header = (f"{'Planner':<12} | {'Succ%':<6} | {'T_set(ms)':<9} | {'T_init(ms)':<10} | {'T_repair':<8} | {'Rep_p99':<8} | "
                  f"{'Obs_Chk':<7} | {'Q_Ops':<7} | {'|V|':<6} | {'Iso_Nodes':<12} | {'Deg(I/O)':<9} | {'Path_Cost':<9}")
        print(header)
        print("-" * 155)
        
        for row in summary_data:
            if "Status" in row:
                print(f"{row['Planner']:<12} | {row['Succ(%)']:>4} | {row['Status']}")
            else:
                print(f"{row['Planner']:<12} | {row['Succ(%)']:>4} | \033[93m{row['T_set(ms)']:<9}\033[0m | \033[93m{row['T_init(ms)']:<10}\033[0m | "
                      f"\033[91m{row['T_repair(ms)']:<8}\033[0m | {row['Rep_p99(ms)']:<8} | "
                      f"{row['Obs_Chk']:<7} | {row['Q_Ops']:<7} | {row['|V|']:<6} | {row['Iso_Nodes']:<12} | "
                      f"{row['Deg(I/O)']:<9} | {row['Path_Cost']:<9}")

        print("-" * 155)

        latex_str = df_out.to_latex(
            index=False, 
            caption=f"Fixed-Graph Performance Metrics: {scenario_name}",
            label=f"tab:fixed_metrics_{scenario_name}",
            column_format="l" + "c"*(len(df_out.columns)-1),
            escape=False 
        )
        print("\n--- LaTeX Table Snippet ---")
        print(latex_str)
        print("---------------------------\n")

def save_comparative_plot(scenario_name, planners_data):
    if not planners_data: return
    plt.figure(figsize=(10, 6))

    for planner, dfs in planners_data.items():
        successful_dfs = []
        for df in dfs:
            if df.empty or 'path_cost' not in df.columns: continue
            if 'crashed' in df.columns and (df['crashed'] == 1).any(): continue
            if 'collision_count' in df.columns and df['collision_count'].max() > 0: continue
            successful_dfs.append(drop_terminal_goal_row(df)) # FIX: Drop terminal row here too
            
        if not successful_dfs: continue
        
        is_geometric = (successful_dfs[0]['time_to_goal'].sum() == 0) if 'time_to_goal' in successful_dfs[0].columns else True
        
        all_times, all_costs = [], []
        goal_reached_times = [] # To track exactly when the robot reached the goal
        
        for df in successful_dfs:
            df_clean = df[df['path_cost'].notna() & (df['path_cost'] != np.inf)].copy()
            if df_clean.empty: continue
            
            if not is_geometric and 'time_to_goal' in df_clean.columns:
                start_budget = df_clean['time_to_goal'].max()
                # Sim Time = 0 at start, increasing as the robot drives.
                df_clean['sim_time'] = start_budget - df_clean['time_to_goal']
                
                # The exact time the simulation terminated (robot reached goal)
                goal_reached_times.append(df_clean['sim_time'].max())
            else:
                df_clean['sim_time'] = df_clean['elapsed_s']
                goal_reached_times.append(df_clean['sim_time'].max())
                
            df_clean['sim_time_rounded'] = df_clean['sim_time'].round(1)
            all_times.extend(df_clean['sim_time_rounded'].tolist())
            all_costs.extend(df_clean['path_cost'].tolist())

        if not all_times: continue
        combined = pd.DataFrame({'time': all_times, 'cost': all_costs})
        combined.dropna(subset=['cost'], inplace=True)
        
        grouped = combined.groupby('time')['cost'].agg(['mean', 'std']).reset_index()
        
        # Plot the main curve
        line, = plt.plot(grouped['time'], grouped['mean'], label=planner, linewidth=2)
        color = line.get_color()
        plt.fill_between(grouped['time'], grouped['mean'] - grouped['std'], grouped['mean'] + grouped['std'], alpha=0.2, color=color)

        # Plot vertical line for Goal Reached Time
        avg_goal_reached = np.mean(goal_reached_times)
        plt.axvline(x=avg_goal_reached, color=color, linestyle='--', alpha=0.7, 
                    label=f'{planner} Goal Reached ({avg_goal_reached:.1f}s)')
        
        print(f"[{scenario_name}] {planner} Avg Goal-Reached Time: {avg_goal_reached:.2f} seconds")

    plt.xlabel("Simulation Time (s)")
    plt.ylabel("Path Cost")
    plt.title(f"Average Path Cost over Time (Successful Runs) - {scenario_name}")
    plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left') # Move legend outside to prevent clutter
    plt.grid(True, linestyle=':', alpha=0.7)
    
    out_path = os.path.join(BUILD_DIR, f"plot_fixed_anytime_{scenario_name}_average.png")
    plt.savefig(out_path, bbox_inches='tight')
    plt.close()
    print(f"[Saved Plot] {out_path}")

def main():
    scenarios = load_data(BUILD_DIR)
    if scenarios:
        for scenario_name, planners_data in scenarios.items():
            analyze_group_statistics(scenario_name, planners_data)
            save_latency_plot(scenario_name, planners_data)
            save_comparative_plot(scenario_name, planners_data)
    else:
        print("No NON-ANYTIME CSV files found.")

if __name__ == "__main__":
    main()





# #################################################################################
#     """
#     =====================================================================================
#     STATISTICAL ANALYSIS RATIONALE (FMTX vs PRM*-D*Lite)
#     =====================================================================================
    
#     1. THE "NO SIGNIFICANT DIFFERENCE" PARADOX IN PATH COST:
#     It is common to see a difference in raw averages (e.g., PRM* = 2.77 vs FMTX = 3.01) 
#     while the Wilcoxon signed-rank test yields a high p-value (e.g., p = 0.44). 
#     A p-value > 0.05 does NOT mean the averages are identical; it means the observed 
#     difference is not consistent enough to be distinguished from random environmental noise.
    
#     Because the obstacle trajectories and random node placements vary drastically per seed, 
#     the path cost fluctuates wildly. A p-value of 0.44 means there is a 44% chance that 
#     PRM*'s slight 0.24 advantage is purely due to the random seeds drawn, rather than a 
#     systematic algorithmic superiority.
    
#     2. WHY THE DIFFERENCE EXISTS DESPITE IDENTICAL SAMPLES:
#     Even with the same sample size (N=1000) and neighborhood radius, the algorithms build 
#     their graphs differently:
#     - PRM* exhaustively evaluates and rewires almost all valid connections in a neighborhood.
#     - FMT* uses a "lazy" dynamic programming approach (Open/Unvisited sets) to expand the 
#       tree faster, intentionally skipping some collision checks and exhaustive rewiring.
      
#     Because of this structural difference at finite sample sizes, FMT* may occasionally 
#     lock into a slightly suboptimal anchor parent compared to PRM*. (This suboptimality 
#     vanishes asymptotically as N -> infinity).
    
#     3. THE PAPER'S CORE ARGUMENT:
#     We use the Wilcoxon test to mathematically defend the algorithm against reviewers. 
#     By proving the cost difference yields p > 0.05, we establish "Statistical Non-Inferiority" 
#     in trajectory quality. We then contrast this with the Replanning Latency (p < 0.001), 
#     proving that FMTX achieves a massive (e.g., 5x) computational speedup without making a 
#     statistically meaningful sacrifice to path optimality.
#     =====================================================================================
#     """


# import warnings
# # Suppress the pandas bottleneck and matplotlib 3D warnings
# warnings.filterwarnings("ignore")

# import pandas as pd
# import glob
# import os
# import re
# import numpy as np
# import matplotlib.pyplot as plt
# from scipy.stats import wilcoxon
# from pandas.errors import EmptyDataError

# # Publication-ready plot styling
# plt.rcParams.update({
#     'font.family': 'serif',
#     'font.serif': ['Times New Roman', 'DejaVu Serif', 'serif'],
#     'axes.labelsize': 12,
#     'axes.titlesize': 14,
#     'legend.fontsize': 11,
#     'xtick.labelsize': 11,
#     'ytick.labelsize': 11,
#     'figure.dpi': 300
# })

# BUILD_DIR = "."  

# def load_and_pair_data(directory):
#     if not os.path.exists(directory): 
#         return {}
        
#     files = glob.glob(os.path.join(directory, "sim_*_metrics.csv"))
#     scenarios = {}

#     print(f"Found {len(files)} total metrics files. Filtering...")

#     for filepath in files:
#         filename = os.path.basename(filepath)
        
#         # Skip ANYTIME algorithms
#         if "ANY" in filename: 
#             continue
            
#         # 1. Extract Seed safely
#         seed_match = re.search(r"_seed_(\d+)", filename)
#         if not seed_match:
#             continue
#         seed = int(seed_match.group(1))
        
#         # 2. Extract Planner Name
#         if "FMTX" in filename:
#             planner = "FMTX"
#         elif "PRM" in filename or "DLITE" in filename or "DStar" in filename:
#             planner = "PRM*-D*Lite"
#         else:
#             planner = "Other"
            
#         # 3. Extract Scenario (State Space)
#         scenario = "Default"
#         if "_R2_" in filename: scenario = "R2"
#         elif "_R2T_" in filename: scenario = "R2T"
#         elif "_Dubins_" in filename: scenario = "Dubins"
#         elif "_Thruster_" in filename: scenario = "Thruster"
#         else:
#             parts = filename.split('_')
#             if len(parts) >= 3:
#                 scenario = parts[2]

#         try:
#             df = pd.read_csv(filepath)
#             if df.empty: continue
            
#             # Clean headers (removes accidental spaces from C++ output)
#             df.columns = [c.strip().replace('_', '') for c in df.columns]
            
#             # Standardize names
#             col_map = {
#                 'elapseds': 'elapsed_s', 'totallatencyms': 'total_latency_ms',
#                 'updatems': 'update_ms', 'planms': 'plan_ms', 
#                 'timetogoal': 'time_to_goal', 'pathcost': 'path_cost', 
#                 'collisioncount': 'collision_count'
#             }
#             df.rename(columns=col_map, inplace=True)
#             df.replace([np.inf, -np.inf], np.nan, inplace=True)
            
#             # Success logic (0 collisions)
#             succ = False
#             if 'collision_count' in df.columns and df['collision_count'].max() == 0:
#                 succ = True
                
#             # Repair latency (elapsed > 0 and update_ms > 0)
#             if 'elapsed_s' in df.columns and 'update_ms' in df.columns and 'total_latency_ms' in df.columns:
#                 repair_df = df[(df['elapsed_s'] > 0.0) & (df['update_ms'] > 0.001)]
#                 avg_repair = repair_df['total_latency_ms'].mean() if not repair_df.empty else 0.0
#             else:
#                 avg_repair = 0.0
                
#             # Final stats
#             arrival_time = df['time_to_goal'].dropna().iloc[-1] if 'time_to_goal' in df.columns else 0.0
#             final_cost = df['path_cost'].dropna().iloc[-1] if 'path_cost' in df.columns else 0.0
            
#             metrics = {
#                 'success': succ,
#                 'avg_repair_ms': avg_repair,
#                 'arrival_time': arrival_time,
#                 'final_cost': final_cost
#             }
            
#             if scenario not in scenarios: scenarios[scenario] = {}
#             if seed not in scenarios[scenario]: scenarios[scenario][seed] = {}
#             scenarios[scenario][seed][planner] = metrics
            
#         except Exception as e:
#             print(f"Warning: Could not process {filename}: {e}")
            
#     return scenarios

# def analyze_and_plot(scenarios):
#     for scenario, seeds_dict in scenarios.items():
#         print(f"\n{'='*80}\n SCENARIO: {scenario}\n{'='*80}")
        
#         # Identity planners
#         planners = set()
#         for s in seeds_dict.values(): planners.update(s.keys())
#         planners = list(planners)
        
#         if len(planners) < 2:
#             print(f"Only found {planners} in {scenario}. Need 2 planners to compare.")
#             continue
            
#         # Ensure FMTX is p1 if available for consistent coloring
#         if "FMTX" in planners:
#             p1 = "FMTX"
#             p2 = [p for p in planners if p != "FMTX"][0]
#         else:
#             p1, p2 = planners[0], planners[1]
        
#         p1_repair, p2_repair = [], []
#         p1_cost, p2_cost = [], []
#         p1_arrival, p2_arrival = [], []
#         paired_seeds = []
        
#         # Pair the seeds for statistical testing
#         for seed, runs in seeds_dict.items():
#             if p1 in runs and p2 in runs:
#                 # ONLY use seeds where BOTH planners successfully reached the goal without crashing
#                 if runs[p1]['success'] and runs[p2]['success']:
#                     paired_seeds.append(seed)
#                     p1_repair.append(runs[p1]['avg_repair_ms'])
#                     p2_repair.append(runs[p2]['avg_repair_ms'])
#                     p1_cost.append(runs[p1]['final_cost'])
#                     p2_cost.append(runs[p2]['final_cost'])
#                     p1_arrival.append(runs[p1]['arrival_time'])
#                     p2_arrival.append(runs[p2]['arrival_time'])
                    
#         num_pairs = len(paired_seeds)
#         print(f"Valid Paired Successful Seeds: {num_pairs}")
#         if num_pairs < 5:
#             print("Too few paired successes for statistical analysis.")
#             continue
            
#         # 1. Wilcoxon Signed-Rank Tests (Statistical Significance)
#         stat_rep, pval_rep = wilcoxon(p1_repair, p2_repair)
#         stat_cost, pval_cost = wilcoxon(p1_cost, p2_cost)
        
#         print(f"\n--- WILCOXON PAIRED STATS ({p1} vs {p2}) ---")
#         print(f"Average Replanning Latency (ms): {p1}: {np.mean(p1_repair):.2f} | {p2}: {np.mean(p2_repair):.2f} | p-value: {pval_rep:.4f}")
#         print(f"Final Path Cost:                 {p1}: {np.mean(p1_cost):.2f}   | {p2}: {np.mean(p2_cost):.2f}   | p-value: {pval_cost:.4f}")
        
#         sig_str = "SIGNIFICANT" if pval_rep < 0.05 else "NOT Significant"
#         print(f"-> Conclusion: The difference in Replanning Latency is {sig_str}.")

#         # 2. BOX PLOT: Replanning Latency
#         plt.figure(figsize=(6, 5))
#         plt.boxplot([p1_repair, p2_repair], labels=[p1, p2], patch_artist=True, 
#                     boxprops=dict(facecolor='lightblue', color='black'),
#                     medianprops=dict(color='red', linewidth=2))
#         plt.ylabel('Average Replanning Latency (ms)')
#         plt.title(f'Paired Replanning Latency ({num_pairs} paired seeds)')
#         plt.grid(axis='y', linestyle=':', alpha=0.7)
        
#         out_box = os.path.join(BUILD_DIR, f"boxplot_latency_{scenario}.png")
#         plt.savefig(out_box, bbox_inches='tight')
#         plt.close()

#         # 3. SCATTER PLOT: Pareto Front (Arrival Time vs Path Cost)
#         plt.figure(figsize=(8, 6))
#         plt.scatter(p1_arrival, p1_cost, label=p1, alpha=0.7, edgecolors='black', s=80, marker='o', c='#1f77b4')
#         plt.scatter(p2_arrival, p2_cost, label=p2, alpha=0.7, edgecolors='black', s=80, marker='s', c='#ff7f0e')
        
#         # Plot Means as large stars
#         plt.scatter(np.mean(p1_arrival), np.mean(p1_cost), color='blue', marker='*', s=400, edgecolors='black', label=f'{p1} Mean')
#         plt.scatter(np.mean(p2_arrival), np.mean(p2_cost), color='red', marker='*', s=400, edgecolors='black', label=f'{p2} Mean')

#         plt.xlabel('Arrival Time (s) [T_robot]')
#         plt.ylabel('Kinematic Path Cost')
#         plt.title(f'Time-Energy Pareto Distribution ({num_pairs} paired seeds)')
#         plt.legend()
#         plt.grid(True, linestyle=':', alpha=0.7)
        
#         out_scatter = os.path.join(BUILD_DIR, f"scatter_pareto_{scenario}.png")
#         plt.savefig(out_scatter, bbox_inches='tight')
#         plt.close()
        
#         print(f"[Saved] {out_box}")
#         print(f"[Saved] {out_scatter}")

# if __name__ == "__main__":
#     scenarios_data = load_and_pair_data(BUILD_DIR)
#     if scenarios_data:
#         analyze_and_plot(scenarios_data)
#     else:
#         print("No CSV metrics files found in the current directory.")