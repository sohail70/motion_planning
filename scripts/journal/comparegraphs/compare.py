# import warnings
# warnings.filterwarnings('ignore')

# import os
# import glob
# import math
# import pandas as pd
# import numpy as np

# BUILD_DIR = "../../../build/"
# TARGET_ENV = "R2T_Test"
# COST_TOLERANCE = 1e-4

# def parse_filename(filepath):
#     filename = os.path.basename(filepath)
#     base_name = filename[6:-4] 
#     parts = base_name.split('_')
#     if len(parts) >= 3 and parts[-2].isdigit() and parts[-1].isdigit():
#         timestamp = f"{parts[-2]}_{parts[-1]}"
#         planner_name = parts[0]
#         environment_name = "_".join(parts[1:-2])
#         return planner_name, environment_name, timestamp
#     return None, None, None

# def load_and_prep_data(filepath):
#     try:
#         df = pd.read_csv(filepath, skipinitialspace=True, index_col=False)
#     except Exception as e:
#         print(f"[ERROR] Failed to read {filepath}. Error: {e}")
#         return pd.DataFrame()

#     df.columns = [str(c).strip().lower() for c in df.columns]

#     cycle_col = next((c for c in df.columns if c in ['cycle_id', 'cycle', 'iteration']), None)
#     if not cycle_col:
#         return pd.DataFrame()

#     df[cycle_col] = pd.to_numeric(df[cycle_col], errors='coerce').fillna(-1).astype(int)
#     available_cycles = [c for c in df[cycle_col].unique() if c >= 0]
#     if not available_cycles: return pd.DataFrame()
#     target_cycle = 0 if 0 in available_cycles else min(available_cycles)

#     df_c = df[df[cycle_col] == target_cycle].copy()
    
#     # استخراج مقادیر g و rhs
#     g_col = next((c for c in df_c.columns if c in ['g', 'g_value', 'cost']), None)
#     lmc_col = next((c for c in df_c.columns if c in ['lmc', 'rhs', 'v_value', 'v']), None)
#     df_c['g_val'] = pd.to_numeric(df_c[g_col], errors='coerce') if g_col else np.nan
#     df_c['lmc_val'] = pd.to_numeric(df_c[lmc_col], errors='coerce') if lmc_col else np.nan

#     # استخراج مختصات مکانی
#     x_col = next((c for c in df_c.columns if c in ['x', 'coord_x', 'pos_x']), None)
#     y_col = next((c for c in df_c.columns if c in ['y', 'coord_y', 'pos_y']), None)
#     z_col = next((c for c in df_c.columns if c in ['z', 'coord_z', 'pos_z']), None)
#     df_c['x_val'] = pd.to_numeric(df_c[x_col], errors='coerce') if x_col else np.nan
#     df_c['y_val'] = pd.to_numeric(df_c[y_col], errors='coerce') if y_col else np.nan
#     df_c['z_val'] = pd.to_numeric(df_c[z_col], errors='coerce') if z_col else np.nan

#     # استخراج والد برای ساخت درخت
#     parent_col = next((c for c in df_c.columns if c in ['parent', 'parent_id', 'p_id']), None)
#     df_c['parent_id'] = pd.to_numeric(df_c[parent_col], errors='coerce').fillna(-1).astype(int)

#     if 'node_id' not in df_c.columns and 'id' in df_c.columns:
#         df_c['node_id'] = df_c['id']

#     df_c['node_id'] = pd.to_numeric(df_c['node_id'], errors='coerce').fillna(-1).astype(int)
#     return df_c.set_index('node_id')

# def fmt(val):
#     if pd.isna(val) or val == float('inf') or val > 1e10:
#         return "inf"
#     return f"{val:.4f}"

# def format_loc(row):
#     x, y, z = row.get('x_val', np.nan), row.get('y_val', np.nan), row.get('z_val', np.nan)
#     if pd.isna(x) and pd.isna(y): return "N/A"
#     if pd.isna(z): return f"({x:.2f}, {y:.2f})"
#     return f"({x:.2f}, {y:.2f}, {z:.2f})"

# def is_different(val1, val2):
#     is_inf1 = pd.isna(val1) or val1 == float('inf') or val1 > 1e10
#     is_inf2 = pd.isna(val2) or val2 == float('inf') or val2 > 1e10
#     if is_inf1 and is_inf2: return False
#     if is_inf1 != is_inf2: return True
#     return not math.isclose(val1, val2, rel_tol=COST_TOLERANCE, abs_tol=COST_TOLERANCE)

# def build_dfs_order(df, common_nodes):
#     """
#     درخت را بر اساس روابط والد-فرزندی می‌سازد و ترتیب گره‌ها را با DFS مشخص می‌کند.
#     گره‌های بدون والد به‌طور خودکار در این پیمایش نادیده گرفته می‌شوند.
#     """
#     parents = df['parent_id']
#     g_vals = df['g_val']
    
#     # 1. یافتن ریشه(ها): گره‌هایی که والد ندارند اما مقدار g آن‌ها متناهی است
#     roots = [n for n in df.index if (pd.isna(parents.get(n)) or parents.get(n) in [-1, n] or parents.get(n) not in df.index) and g_vals.get(n, float('inf')) < 1e10]
            
#     # 2. ساخت لیست مجاورت برای فرزندان
#     children = {n: [] for n in df.index}
#     for n in df.index:
#         p = parents.get(n, -1)
#         if not pd.isna(p) and p != -1 and p != n and p in children and n not in roots:
#             children[p].append(n)
            
#     # 3. پیمایش جستجوی عمق-اول (DFS Iterative) برای جلوگیری از خطای Recursion
#     ordered_nodes = []
#     for r in sorted(roots):
#         if r in common_nodes:
#             ordered_nodes.append({'node': r, 'depth': 0, 'branch': r})
        
#         # هر فرزند مستقیم ریشه، یک شاخه (Branch) جدید محسوب می‌شود
#         for branch_head in sorted(children.get(r, [])):
#             stack = [(branch_head, 1, branch_head)] 
#             while stack:
#                 curr, depth, b_id = stack.pop()
#                 if curr in common_nodes:
#                     ordered_nodes.append({'node': curr, 'depth': depth, 'branch': b_id})
                
#                 # فرزندان به‌صورت معکوس اضافه می‌شوند تا ترتیب پیمایش حفظ شود
#                 for child in reversed(sorted(children.get(curr, []))):
#                     stack.append((child, depth + 1, b_id))
                    
#     return ordered_nodes

# def compare_graphs():
#     search_pattern = os.path.join(BUILD_DIR, "graph_*.csv")
#     all_files = glob.glob(search_pattern)
#     latest_files = {}
#     max_timestamps = {}

#     for filepath in all_files:
#         planner, env, timestamp = parse_filename(filepath)
#         if env == TARGET_ENV and planner:
#             if planner not in max_timestamps or timestamp > max_timestamps[planner]:
#                 max_timestamps[planner] = timestamp
#                 latest_files[planner] = filepath

#     if len(latest_files) < 2:
#         print("[INFO] Not enough files found for comparison.")
#         return

#     planners_data = {}
#     for planner, filepath in latest_files.items():
#         df = load_and_prep_data(filepath)
#         if not df.empty:
#             planners_data[planner] = df

#     planners_list = list(planners_data.keys())
#     base_planner = planners_list[0]
#     base_df = planners_data[base_planner]

#     for target_planner in planners_list[1:]:
#         target_df = planners_data[target_planner]
#         common_nodes = sorted(list(set(base_df.index).intersection(set(target_df.index))))
#         common_nodes = [n for n in common_nodes if n >= 0]

#         # ساخت ساختار درختی و استخراج گره‌ها بر اساس گراف پایه (base_df)
#         dfs_ordered_nodes = build_dfs_order(base_df, common_nodes)

#         print(f"\n{'='*120}")
#         print(f"{f' TREE-STRUCTURED COMPARISON: [{base_planner}] vs [{target_planner}] ':^120}")
#         print(f"{'='*120}")
        
#         bp_name, tp_name = base_planner[:14], target_planner[:14]
#         bp_col, tp_col = f"G / RHS ({bp_name})", f"G / RHS ({tp_name})"
        
#         header = f"{'Node ID (Tree)':<18} | {'Location':<18} | {bp_col:<28} | {tp_col:<28} | {'Status'}"
#         print(header)
#         print("-" * 120)

#         mismatch_count = 0
#         current_branch = None

#         for item in dfs_ordered_nodes:
#             node_id = item['node']
#             depth = item['depth']
#             branch = item['branch']

#             # اعمال بخش‌بندی و چاپ جداکننده بین شاخه‌ها
#             if depth == 1 and branch != current_branch:
#                 print(f"{'-'*120}")
#                 print(f"{f' [ BRANCH STARTING AT NODE {branch} ] ':-^120}")
#                 print(f"{'-'*120}")
#                 current_branch = branch

#             b_row, t_row = base_df.loc[node_id], target_df.loc[node_id]

#             bg, brhs = b_row.get('g_val', float('inf')), b_row.get('lmc_val', float('inf'))
#             tg, trhs = t_row.get('g_val', float('inf')), t_row.get('lmc_val', float('inf'))

#             # فرمت‌دهی بصری ساختار درختی در ستون Node ID
#             if depth == 0:
#                 node_str = f"*{node_id} (Root)"
#             else:
#                 indent = "  " * (depth - 1)
#                 node_str = f"{indent}↳ {node_id}"
#                 # جلوگیری از به‌هم‌ریختگی جدول در صورت عمیق بودن بیش از حد درخت
#                 if len(node_str) > 17: 
#                     node_str = f"...↳ {node_id}" 

#             location_str = format_loc(b_row)

#             # بررسی تفاوت‌ها
#             flag = "<-- MISMATCH" if (is_different(bg, tg) or is_different(brhs, trhs)) else ""
#             if flag: mismatch_count += 1

#             b_combined = f"{fmt(bg)} / {fmt(brhs)}"
#             t_combined = f"{fmt(tg)} / {fmt(trhs)}"

#             print(f"{node_str:<18} | {location_str:<18} | {b_combined:<28} | {t_combined:<28} | {flag}")

#         print("-" * 120)
#         print(f"Total Connected Common Nodes: {len(dfs_ordered_nodes)}")
#         print(f"Nodes with mismatched G/RHS : {mismatch_count}")

# if __name__ == "__main__":
#     compare_graphs()


# ##############################################################################################################
# import warnings
# warnings.filterwarnings('ignore')

# import os
# import glob
# import math
# import pandas as pd
# import numpy as np

# BUILD_DIR = "../../../build/"
# COST_TOLERANCE = 1e-4

# def parse_filename(filepath):
#     filename = os.path.basename(filepath)
#     base_name = filename[6:-4] # Remove 'graph_' and '.csv'
#     parts = base_name.split('_')
#     if len(parts) >= 3 and parts[-2].isdigit() and parts[-1].isdigit():
#         timestamp = f"{parts[-2]}_{parts[-1]}"
#         planner_name = parts[0]
#         environment_name = "_".join(parts[1:-2])
#         return planner_name, environment_name, timestamp
#     return None, None, None

# def load_and_prep_all_data(filepath):
#     """
#     Returns the whole dataframe so we can compare all cycles together.
#     """
#     try:
#         df = pd.read_csv(filepath, skipinitialspace=True, index_col=False)
#     except Exception as e:
#         print(f"[ERROR] Failed to read {filepath}. Error: {e}")
#         return pd.DataFrame(), None

#     df.columns = [str(c).strip().lower() for c in df.columns]

#     cycle_col = next((c for c in df.columns if c in ['cycle_id', 'cycle', 'iteration']), None)
#     if not cycle_col:
#         return pd.DataFrame(), None

#     df[cycle_col] = pd.to_numeric(df[cycle_col], errors='coerce').fillna(-1).astype(int)
    
#     # Standardization
#     g_col = next((c for c in df.columns if c in ['g', 'g_value', 'cost']), None)
#     lmc_col = next((c for c in df.columns if c in ['lmc', 'rhs', 'v_value', 'v']), None)
    
#     df['g_val'] = pd.to_numeric(df[g_col], errors='coerce') if g_col else np.nan
#     df['lmc_val'] = pd.to_numeric(df[lmc_col], errors='coerce') if lmc_col else np.nan

#     if 'node_id' not in df.columns and 'id' in df.columns:
#         df['node_id'] = df['id']

#     df['node_id'] = pd.to_numeric(df['node_id'], errors='coerce').fillna(-1).astype(int)
    
#     return df, cycle_col

# def is_finite(val):
#     return not (pd.isna(val) or val == float('inf') or val > 1e10)

# def compare_graphs():
#     search_pattern = os.path.join(BUILD_DIR, "graph_*.csv")
#     all_files = glob.glob(search_pattern)
    
#     # Group by Environment -> Planner -> File details
#     env_latest_files = {}
#     env_max_timestamps = {}

#     # 1. Find the latest files for EACH planner within EACH environment
#     for filepath in all_files:
#         planner, env, timestamp = parse_filename(filepath)
#         if not planner or not env:
#             continue
            
#         if env not in env_max_timestamps:
#             env_max_timestamps[env] = {}
#             env_latest_files[env] = {}

#         if planner not in env_max_timestamps[env] or timestamp > env_max_timestamps[env][planner]:
#             env_max_timestamps[env][planner] = timestamp
#             env_latest_files[env][planner] = filepath

#     if not env_latest_files:
#         print("[INFO] No graph files found.")
#         return

#     # 2. Evaluate dynamically for every environment discovered
#     for env, latest_files in env_latest_files.items():
#         print(f"\n{'*'*90}")
#         print(f"{' ENVIRONMENT: ' + env + ' ':-^90}")
#         print(f"{'*'*90}")

#         if len(latest_files) < 2:
#             print(f"[INFO] Not enough planners found to compare in {env} (Found: {list(latest_files.keys())}).")
#             continue

#         planners_data = {}
#         common_cycles = set()
        
#         for planner, filepath in latest_files.items():
#             df, cycle_col = load_and_prep_all_data(filepath)
#             if not df.empty:
#                 planners_data[planner] = df
#                 cycles_in_df = set(df[cycle_col].unique())
#                 cycles_in_df.discard(-1)
                
#                 if not common_cycles:
#                     common_cycles = cycles_in_df
#                 else:
#                     common_cycles = common_cycles.intersection(cycles_in_df)

#         planners_list = list(planners_data.keys())
#         if len(planners_list) < 2:
#             print(f"[WARN] Not enough valid data to compare in {env}.")
#             continue

#         # Assuming pairwise comparison for the first two planners found
#         base_planner = planners_list[0]
#         target_planner = planners_list[1] 

#         print(f"\n{'='*90}")
#         print(f"{' DETAILED PERFORMANCE SUMMARY BY CYCLE ':-^90}")
#         print(f" BASE: {base_planner} | TARGET: {target_planner}")
#         print(f"{'='*90}")

#         if not common_cycles:
#             print("[WARN] No common cycles found between the algorithms.")
#             continue

#         # 3. Cycle-by-cycle Evaluation
#         for cycle in sorted(list(common_cycles)):
#             base_df = planners_data[base_planner]
#             target_df = planners_data[target_planner]
            
#             cycle_col_b = next((c for c in base_df.columns if c in ['cycle_id', 'cycle', 'iteration']), None)
#             cycle_col_t = next((c for c in target_df.columns if c in ['cycle_id', 'cycle', 'iteration']), None)

#             b_cycle_df = base_df[base_df[cycle_col_b] == cycle].set_index('node_id')
#             t_cycle_df = target_df[target_df[cycle_col_t] == cycle].set_index('node_id')

#             common_nodes = sorted(list(set(b_cycle_df.index).intersection(set(t_cycle_df.index))))
#             common_nodes = [n for n in common_nodes if n >= 0]

#             # --- Metrics Computations ---
#             b_finite_count = sum(is_finite(b_cycle_df.loc[n, 'g_val']) for n in b_cycle_df.index)
#             t_finite_count = sum(is_finite(t_cycle_df.loc[n, 'g_val']) for n in t_cycle_df.index)

#             b_consistent = sum(abs(b_cycle_df.loc[n, 'g_val'] - b_cycle_df.loc[n, 'lmc_val']) < COST_TOLERANCE for n in b_cycle_df.index if is_finite(b_cycle_df.loc[n, 'g_val']))
#             t_consistent = sum(abs(t_cycle_df.loc[n, 'g_val'] - t_cycle_df.loc[n, 'lmc_val']) < COST_TOLERANCE for n in t_cycle_df.index if is_finite(t_cycle_df.loc[n, 'g_val']))

#             base_wins = 0
#             target_wins = 0
#             ties = 0
#             cost_diffs = [] 

#             for n in common_nodes:
#                 bg = b_cycle_df.loc[n, 'g_val']
#                 tg = t_cycle_df.loc[n, 'g_val']
                
#                 b_fin = is_finite(bg)
#                 t_fin = is_finite(tg)

#                 if not b_fin and not t_fin:
#                     continue # Both blocked
#                 elif b_fin and not t_fin:
#                     base_wins += 1 
#                 elif not b_fin and t_fin:
#                     target_wins += 1 
#                 else:
#                     if abs(bg - tg) <= COST_TOLERANCE:
#                         ties += 1
#                     elif bg < tg:
#                         base_wins += 1
#                     else:
#                         target_wins += 1
                        
#                     cost_diffs.append(bg - tg)

#             avg_diff = sum(cost_diffs) / len(cost_diffs) if cost_diffs else 0.0

#             # --- Determine Verdict ---
#             if base_wins > target_wins:
#                 verdict = f"{base_planner} is DOMINATING (Found better paths for {base_wins} nodes)"
#             elif target_wins > base_wins:
#                 verdict = f"{target_planner} is DOMINATING (Found better paths for {target_wins} nodes)"
#             else:
#                 verdict = "Both algorithms performed EQUALLY in cost optimization."

#             # --- Formatted Output ---
#             print(f"\n[ CYCLE {cycle} ] ".ljust(90, '-'))
            
#             print("\n 1. Reachability (Nodes with finite G-cost):")
#             print(f"    - {base_planner:<20}: {b_finite_count} nodes")
#             print(f"    - {target_planner:<20}: {t_finite_count} nodes")

#             print("\n 2. Cost Optimality (Comparison on shared nodes):")
#             print(f"    - {base_planner} wins on : {base_wins} nodes")
#             print(f"    - {target_planner} wins on : {target_wins} nodes")
#             print(f"    - Tied on                  : {ties} nodes")
#             print(f"    - Mean Cost Difference     : {avg_diff:+.4f} (Positive means {target_planner} is cheaper)")

#             print("\n 3. Consistency Status (Nodes where G == RHS):")
#             b_pct = (b_consistent / b_finite_count * 100) if b_finite_count else 0
#             t_pct = (t_consistent / t_finite_count * 100) if t_finite_count else 0
#             print(f"    - {base_planner:<20}: {b_consistent} / {b_finite_count} ({b_pct:.1f}%)")
#             print(f"    - {target_planner:<20}: {t_consistent} / {t_finite_count} ({t_pct:.1f}%)")

#             print("\n 🎯 VERDICT:")
#             print(f"    >> {verdict}")

#         print(f"\n{'='*90}")
#         print(f" SUMMARY FOR {env} COMPLETE ".center(90, '='))

# if __name__ == "__main__":
#     compare_graphs()



import warnings
warnings.filterwarnings('ignore')

import os
import glob
import pandas as pd
import numpy as np

BUILD_DIR = '../../../build/'
COST_TOLERANCE = 1e-4
BASE_PLANNER = 'FMTX'
TARGET_PLANNER = 'PRMSTARDSTARLITE'


def parse_filename(filepath):
    filename = os.path.basename(filepath)
    if not (filename.startswith('graph_') and filename.endswith('.csv')):
        return None, None, None
    base_name = filename[6:-4]
    parts = base_name.split('_')
    if len(parts) >= 3 and parts[-2].isdigit() and parts[-1].isdigit():
        timestamp = f"{parts[-2]}_{parts[-1]}"
        planner_name = parts[0]
        environment_name = '_'.join(parts[1:-2])
        return planner_name, environment_name, timestamp
    return None, None, None


def is_finite(val):
    return pd.notna(val) and np.isfinite(val) and val <= 1e10


def load_and_prep_all_data(filepath):
    try:
        df = pd.read_csv(filepath, skipinitialspace=True, index_col=False)
    except Exception:
        return pd.DataFrame(), None

    df.columns = [str(c).strip().lower() for c in df.columns]
    cycle_col = next((c for c in df.columns if c in ['cycle_id', 'cycle', 'iteration']), None)
    if not cycle_col:
        return pd.DataFrame(), None

    df[cycle_col] = pd.to_numeric(df[cycle_col], errors='coerce').fillna(-1).astype(int)

    if 'node_id' not in df.columns and 'id' in df.columns:
        df['node_id'] = df['id']
    if 'node_id' not in df.columns:
        return pd.DataFrame(), None
    df['node_id'] = pd.to_numeric(df['node_id'], errors='coerce').fillna(-1).astype(int)

    for col in ['g', 'lmc', 'robot_g', 'robot_lmc', 'robot_total_cost', 'robot_time_to_goal']:
        if col not in df.columns:
            df[col] = np.nan
        df[col] = pd.to_numeric(df[col], errors='coerce')

    if 'is_robot_anchor' not in df.columns:
        df['is_robot_anchor'] = 0
    df['is_robot_anchor'] = pd.to_numeric(df['is_robot_anchor'], errors='coerce').fillna(0).astype(int)

    df['g_val'] = df['g']
    df['lmc_val'] = df['lmc']
    return df, cycle_col


def get_robot_cycle_stats(cycle_df):
    anchor_rows = cycle_df[cycle_df['is_robot_anchor'] == 1]
    row = anchor_rows.iloc[0] if not anchor_rows.empty else cycle_df.iloc[0]
    return {
        'anchor_id': int(row['node_id']) if pd.notna(row['node_id']) else -1,
        'robot_g': float(row['robot_g']) if pd.notna(row['robot_g']) else np.nan,
        'robot_lmc': float(row['robot_lmc']) if pd.notna(row['robot_lmc']) else np.nan,
        'robot_total_cost': float(row['robot_total_cost']) if pd.notna(row['robot_total_cost']) else np.nan,
        'robot_time_to_goal': float(row['robot_time_to_goal']) if pd.notna(row['robot_time_to_goal']) else np.nan,
    }


def consistency_gap(df):
    return (df['g_val'] - df['lmc_val']).abs()


def compare_graphs():
    search_pattern = os.path.join(BUILD_DIR, 'graph_*.csv')
    all_files = glob.glob(search_pattern)

    env_latest_files = {}
    env_max_timestamps = {}

    for filepath in all_files:
        planner, env, timestamp = parse_filename(filepath)
        if not planner or not env:
            continue
        if planner not in [BASE_PLANNER, TARGET_PLANNER]:
            continue
        env_latest_files.setdefault(env, {})
        env_max_timestamps.setdefault(env, {})
        if planner not in env_max_timestamps[env] or timestamp > env_max_timestamps[env][planner]:
            env_max_timestamps[env][planner] = timestamp
            env_latest_files[env][planner] = filepath

    if not env_latest_files:
        print('[INFO] No matching graph files found.')
        return

    for env, latest_files in env_latest_files.items():
        if BASE_PLANNER not in latest_files or TARGET_PLANNER not in latest_files:
            continue

        base_df, cycle_col_b = load_and_prep_all_data(latest_files[BASE_PLANNER])
        target_df, cycle_col_t = load_and_prep_all_data(latest_files[TARGET_PLANNER])
        if base_df.empty or target_df.empty:
            continue

        common_cycles = set(base_df[cycle_col_b].unique()).intersection(set(target_df[cycle_col_t].unique()))
        common_cycles.discard(-1)

        print(f"\n{'='*100}")
        print(f"{' FAIR NON-ANYTIME GRAPH COMPARISON BY CYCLE ':-^100}")
        print(f" ENVIRONMENT: {env}")
        print(f" BASE: {BASE_PLANNER} | TARGET: {TARGET_PLANNER}")
        print(f"{'='*100}")

        for cycle in sorted(common_cycles):
            b_cycle_df = base_df[base_df[cycle_col_b] == cycle].copy().set_index('node_id')
            t_cycle_df = target_df[target_df[cycle_col_t] == cycle].copy().set_index('node_id')

            common_nodes = sorted(n for n in set(b_cycle_df.index).intersection(set(t_cycle_df.index)) if n >= 0)
            if not common_nodes:
                continue

            b_stats = get_robot_cycle_stats(base_df[base_df[cycle_col_b] == cycle])
            t_stats = get_robot_cycle_stats(target_df[target_df[cycle_col_t] == cycle])

            b_anchor_cost = b_stats['robot_lmc'] if is_finite(b_stats['robot_lmc']) else np.nan
            t_anchor_cost = t_stats['robot_lmc'] if is_finite(t_stats['robot_lmc']) else np.nan
            if is_finite(b_anchor_cost) and is_finite(t_anchor_cost):
                fair_threshold = min(b_anchor_cost, t_anchor_cost)
            else:
                fair_threshold = np.nan


            partial_comparable = True
            partial_reason = ""

            if not is_finite(b_stats['robot_lmc']) or not is_finite(t_stats['robot_lmc']):
                partial_comparable = False
                partial_reason = "One planner has infinite robot_lmc, so the settled horizon is not fully comparable."
            elif abs(b_stats['anchor_id'] - t_stats['anchor_id']) > 0:
                partial_reason = "Anchor ids differ, but comparison is still allowed because the fair horizon is cost-based."
            elif len(common_nodes) < min(len(b_cycle_df), len(t_cycle_df)):
                partial_reason = "Graphs are not identical in support, but overlap is sufficient for snapshot comparison."
            else:
                partial_reason = "Fully comparable cycle."

            base_wins = target_wins = ties = 0
            cost_diffs = []
            fair_nodes_evaluated = 0
            b_consistency_viol = 0
            t_consistency_viol = 0

            for n in common_nodes:
                bg = b_cycle_df.loc[n, 'g_val']
                tg = t_cycle_df.loc[n, 'g_val']
                bl = b_cycle_df.loc[n, 'lmc_val']
                tl = t_cycle_df.loc[n, 'lmc_val']

                if is_finite(bl) and is_finite(bg) and abs(bg - bl) > COST_TOLERANCE:
                    b_consistency_viol += 1
                if is_finite(tl) and is_finite(tg) and abs(tg - tl) > COST_TOLERANCE:
                    t_consistency_viol += 1

                b_fin = is_finite(bg)
                t_fin = is_finite(tg)

                if is_finite(fair_threshold):
                    if b_fin and bg > fair_threshold + COST_TOLERANCE:
                        continue
                    if t_fin and tg > fair_threshold + COST_TOLERANCE:
                        continue

                fair_nodes_evaluated += 1

                if not b_fin and not t_fin:
                    continue
                elif b_fin and not t_fin:
                    base_wins += 1
                elif not b_fin and t_fin:
                    target_wins += 1
                else:
                    if abs(bg - tg) <= COST_TOLERANCE:
                        ties += 1
                    elif bg < tg:
                        base_wins += 1
                    else:
                        target_wins += 1
                    cost_diffs.append(bg - tg)

            avg_diff = sum(cost_diffs) / len(cost_diffs) if cost_diffs else 0.0
            overlap_ratio = len(common_nodes) / max(1, min(len(b_cycle_df), len(t_cycle_df)))

            if base_wins > target_wins:
                verdict = f'{BASE_PLANNER} DOMINATES inside the fair anchor horizon.'
            elif target_wins > base_wins:
                verdict = f'{TARGET_PLANNER} DOMINATES inside the fair anchor horizon.'
            else:
                verdict = 'Both algorithms are equally optimal inside the fair anchor horizon.'

            print(f"\n[ CYCLE {cycle} ] ".ljust(100, '-'))
            print('\n 0. Anchor / Robot Costs:')
            print(f"    - {BASE_PLANNER} anchor id        : {b_stats['anchor_id']}")
            print(f"    - {TARGET_PLANNER} anchor id      : {t_stats['anchor_id']}")
            print(f"    - {BASE_PLANNER} robot_lmc        : {b_stats['robot_lmc']:.4f}")
            print(f"    - {TARGET_PLANNER} robot_lmc      : {t_stats['robot_lmc']:.4f}")
            print(f"    - {BASE_PLANNER} total cost       : {b_stats['robot_total_cost']:.4f}")
            print(f"    - {TARGET_PLANNER} total cost     : {t_stats['robot_total_cost']:.4f}")
            print(f"    - Fair Common Threshold           : {fair_threshold:.4f}")

            print('\n 1. Comparability:')
            print(f"    - {BASE_PLANNER} nodes this cycle : {len(b_cycle_df)}")
            print(f"    - {TARGET_PLANNER} nodes this cycle: {len(t_cycle_df)}")
            print(f"    - Common nodes                    : {len(common_nodes)}")
            print(f"    - Overlap ratio                   : {overlap_ratio:.3f}")
            print(f"    - Nodes evaluated fairly          : {fair_nodes_evaluated}")

            print('\n 2. Consistency Check:')
            print(f"    - {BASE_PLANNER} |g-lmc| violations : {b_consistency_viol}")
            print(f"    - {TARGET_PLANNER} |g-lmc| violations: {t_consistency_viol}")

            print('\n 3. Cost Optimality (Inside Fair Anchor Horizon):')
            print(f"    - {BASE_PLANNER} wins on          : {base_wins} nodes")
            print(f"    - {TARGET_PLANNER} wins on        : {target_wins} nodes")
            print(f"    - Tied on                         : {ties} nodes")
            print(f"    - Mean Cost Difference            : {avg_diff:+.4f} (Positive means {TARGET_PLANNER} is cheaper)")

            print('\n 🎯 VERDICT:')
            print(f"    >> {verdict}")


if __name__ == '__main__':
    compare_graphs()