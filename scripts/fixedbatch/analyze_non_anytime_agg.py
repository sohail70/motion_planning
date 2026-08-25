"""
======================================================================================
BENCHMARKING METHODOLOGY & METRIC ISOLATION (NON-ANYTIME VERSION)
======================================================================================
This script processes high-resolution, event-based timeline metrics from the C++ planner.

Event semantics:
- initial_plan   : one-time initialization plan (Graph construction & setup)
- set_state      : robot-state snapshot; path_cost belongs here
- update         : obstacle update & repair triggered here
- slice_end      : end of real-time slice
- goal_reached   : terminal success snapshot
- time_limit     : terminal timeout snapshot

Metric isolation policy:
1. T_repair:
   Per repair event = update_ms
   Because plan() happens inside updateObstacles() in non-anytime planners.
   Aggregation: per-seed median of update_ms → cross-seed median of those per-seed medians.
   Using median inside each seed avoids skew from a few extremely slow repairs,
   while the global median summarises typical planner latency robustly.

2. Path cost (solution quality):
   Source: set_state / goal_reached rows only (instantaneous planned cost-to-go).
   Since the robot always finishes exactly at the time budget, total simulation time
   does not discriminate planner quality. Instead we measure how much suboptimal
   remaining cost the robot endured over the whole run.
   - Within one seed: we compute the **mean** of all valid path_cost values.
     The mean captures prolonged detours: if the robot stays at high cost for a
     long time, the mean rises. The median would hide that duration effect,
     and in our case high-cost episodes are exactly the behaviour we want to
     penalise – they are the signal, not outliers.
   - Across seeds: we report the **median** of the per-seed means.
     This provides an outlier-robust summary (one unusually hard seed won't
     inflate the central value) and corresponds to the "typical" mission quality.

3. Obstacle checks:
   Reported as Obs/Upd directly from the 'update' event.
   Per-seed: mean obstacle checks per update event.
   Cross-seed: mean of those per-seed means.

4. Fixed Graph Stats:
   Samples, r_n, Setup(ms), and Isolated nodes are parsed from the 'initial_plan' event.

======================================================================================
"""

import pandas as pd
import glob
import os
import re
import numpy as np
import matplotlib.pyplot as plt
from pandas.errors import EmptyDataError

# Publication-ready plot styling
plt.rcParams.update({
    "font.family": "serif",
    "font.serif": ["Times New Roman", "DejaVu Serif", "serif"],
    "axes.labelsize": 12,
    "axes.titlesize": 14,
    "legend.fontsize": 11,
    "xtick.labelsize": 11,
    "ytick.labelsize": 11,
    "figure.dpi": 300
})

PLANNER_ORDER = ["FMTX", "DLITE"]
color_map = {"FMTX": "#1f78b4", "DLITE": "#d62728"}
display_names = {
    "FMTX":    r"D-FMT$^*$",
    "DLITE":   r"D$^*$Lite"
}

def planner_sort_key(p):
    """Return sort key: FMTX first, then DLITE, then others alphabetically."""
    try:
        return (0, PLANNER_ORDER.index(p))
    except ValueError:
        return (1, p)

BUILD_DIR = "."
FILENAME_PATTERN = re.compile(
    r"sim_([A-Za-z0-9]+)_([A-Za-z0-9_]+)_seed_(\d+)_(\d{8}_\d{6})_metrics\.csv"
)

def safe_numeric(df, cols):
    for c in cols:
        if c in df.columns:
            df[c] = pd.to_numeric(df[c], errors="coerce")
    return df

def load_data(directory):
    if not os.path.exists(directory):
        return {}

    files = glob.glob(os.path.join(directory, "sim_*_metrics.csv"))
    scenarios = {}

    for filepath in files:
        filename = os.path.basename(filepath)
        match = FILENAME_PATTERN.match(filename)
        if not match:
            continue

        planner_raw = match.group(1)
        scenario = match.group(2)
        seed = int(match.group(3)) # Extract the seed

        planner_clean = (
            planner_raw
            .replace("Kinodynamic", "")
            .replace("PRMStarDStarLite", "DLITE")
            .replace("PRMStar", "")
            .replace("ANY", "")
        )

        try:
            df = pd.read_csv(filepath)
            if df.empty:
                continue

            df.replace([np.inf, -np.inf], np.nan, inplace=True)
            df['seed'] = seed # Inject seed into dataframe

            numeric_cols = [
                "row_id", "elapsed_s", "sim_time", "setup_ms", "total_latency_ms",
                "update_ms", "plan_ms", "time_to_goal", "path_cost", "obstacle_checks",
                "collision_count", "tree_size", "isolated_nodes", "avg_deg_out",
                "avg_deg_in", "neighborhood_radius"
            ]
            df = safe_numeric(df, numeric_cols)

            sort_cols = [c for c in ["row_id", "elapsed_s"] if c in df.columns]
            if sort_cols:
                df = df.sort_values(sort_cols).reset_index(drop=True)

            if scenario not in scenarios:
                scenarios[scenario] = {}
            if planner_clean not in scenarios[scenario]:
                scenarios[scenario][planner_clean] = []

            scenarios[scenario][planner_clean].append(df)
            print(f"Loaded {filename}: {len(df)} rows")

        except EmptyDataError:
            pass
        except Exception as e:
            print(f"Warning: Could not load {filename}: {e}")

    return scenarios

def is_successful_run(df):
    if df.empty:
        return False
    if "collision_count" in df.columns:
        cc = pd.to_numeric(df["collision_count"], errors="coerce").fillna(0)
        return cc.max() == 0
    if "crashed" in df.columns:
        cr = pd.to_numeric(df["crashed"], errors="coerce").fillna(0)
        return (cr == 1).sum() == 0
    return True

def extract_runtime_events(df):
    if df.empty or "event_type" not in df.columns:
        return pd.DataFrame()
    runtime = df.copy()
    if "elapsed_s" in runtime.columns:
        runtime = runtime[runtime["elapsed_s"].fillna(0) > 0.0].copy()
    runtime = runtime.sort_values(
        [c for c in ["row_id", "elapsed_s"] if c in runtime.columns]
    ).reset_index(drop=True)
    return runtime

def get_state_rows(df):
    if df.empty:
        return pd.DataFrame()
    rows = df[df["event_type"].isin(["set_state", "goal_reached"])].copy()
    rows = safe_numeric(rows, ["path_cost", "sim_time", "elapsed_s", "time_to_goal"])
    rows = rows[
        rows["path_cost"].notna() &
        np.isfinite(rows["path_cost"]) &
        (rows["path_cost"] > 0)
    ].copy()
    return rows

def get_update_rows(df):
    if df.empty:
        return pd.DataFrame()
    rows = df[df["event_type"] == "update"].copy()
    rows = safe_numeric(rows, ["update_ms", "obstacle_checks", "path_cost"])
    return rows

def get_terminal_rows(df):
    if df.empty:
        return pd.DataFrame()
    rows = df[df["event_type"].isin(["goal_reached", "time_limit"])].copy()
    return rows

def filter_paired_seeds(planners_data, target_count=30):
    """
    Finds the intersection of successful seeds across planners,
    and returns a filtered dictionary containing exactly 'target_count' paired seeds.
    """
    successful_seeds_per_planner = {}
    for planner, dfs in planners_data.items():
        successful_seeds = [df['seed'].iloc[0] for df in dfs if is_successful_run(df)]
        successful_seeds_per_planner[planner] = set(successful_seeds)
    
    if not successful_seeds_per_planner:
        return planners_data, 0
        
    common_seeds = set.intersection(*successful_seeds_per_planner.values())
    
    if len(common_seeds) < target_count:
        print(f"WARNING: Only found {len(common_seeds)} paired successful seeds, which is less than the target {target_count}.")
        selected_seeds = list(common_seeds)
    else:
        # Sort them to be deterministic, then take the first 30
        selected_seeds = sorted(list(common_seeds))[:target_count]
        
    filtered_data = {}
    for planner, dfs in planners_data.items():
        filtered_data[planner] = [df for df in dfs if df['seed'].iloc[0] in selected_seeds]
        
    return filtered_data, len(selected_seeds)




# ---------- DISTRIBUTION PLOTS (Non‑Anytime) ----------

def extract_non_anytime_per_seed_medians(planners_data):
    """
    Extract per-seed median repair time and mean obstacle checks per repair
    for non‑anytime planners (FMTX, DLITE, etc.).
    """
    paired_data, _ = filter_paired_seeds(planners_data, target_count=30)
    metrics = {}

    for planner, dfs in paired_data.items():
        valid_dfs = [df for df in dfs if not df.empty]
        if not valid_dfs:
            continue
        t_repair_list = []
        obs_repair_list = []   # per-seed mean obstacle checks per update event

        for df in valid_dfs:
            runtime_df = extract_runtime_events(df)
            run_update = get_update_rows(runtime_df)

            if not run_update.empty:
                # Median repair time
                t_repair_list.append(run_update["update_ms"].dropna().median())
                # Mean obstacle checks per repair event
                obs_checks = run_update["obstacle_checks"].dropna().mean()
                if pd.notna(obs_checks):
                    obs_repair_list.append(obs_checks)

        if t_repair_list:   # only include if there is at least one seed with repairs
            metrics[planner] = {
                't_repair': t_repair_list,
                'obs_repair': obs_repair_list
            }
    return metrics


# --- Separate boxplots for repair time and obstacle checks ---

def save_boxplot_repair_time_nonanytime(scenario_name, planners_data):
    metrics = extract_non_anytime_per_seed_medians(planners_data)
    if not metrics:
        return
    planners = sorted(metrics.keys(), key=planner_sort_key)
    color_mapping = {'FMTX': '#1f78b4', 'DLITE': '#d62728'}

    fig, ax = plt.subplots(figsize=(6, 5))
    data_to_plot = [metrics[p]['t_repair'] for p in planners]
    bp = ax.boxplot(data_to_plot,tick_labels=[display_names.get(p, p) for p in planners] , patch_artist=True,
                    widths=0.6, showfliers=True)
    for patch, planner in zip(bp['boxes'], planners):
        patch.set_facecolor(color_mapping.get(planner, '#AAAAAA'))
        patch.set_alpha(0.6)
    ax.set_ylabel('Repair Time Per Seed (ms)')
    # ax.grid(axis='y', linestyle=':', alpha=0.7)
    plt.tight_layout()
    out_path = os.path.join(BUILD_DIR, f"boxplot_repair_time_{scenario_name}.png")
    plt.savefig(out_path, bbox_inches="tight")
    plt.close()
    print(f"[Saved Plot] {out_path}")


def save_boxplot_obs_checks_nonanytime(scenario_name, planners_data):
    metrics = extract_non_anytime_per_seed_medians(planners_data)
    if not metrics:
        return
    planners = sorted(metrics.keys(), key=planner_sort_key)
    color_mapping = {'FMTX': '#1f78b4', 'DLITE': '#d62728'}

    fig, ax = plt.subplots(figsize=(6, 5))
    data_to_plot = [metrics[p]['obs_repair'] for p in planners]
    bp = ax.boxplot(data_to_plot, tick_labels=[display_names.get(p, p) for p in planners] , patch_artist=True,
                    widths=0.6, showfliers=True)
    for patch, planner in zip(bp['boxes'], planners):
        patch.set_facecolor(color_mapping.get(planner, '#AAAAAA'))
        patch.set_alpha(0.6)
    ax.set_ylabel('Obstacle Checks Per Event (Per Seed Mean)')
    # ax.grid(axis='y', linestyle=':', alpha=0.7)
    plt.tight_layout()
    out_path = os.path.join(BUILD_DIR, f"boxplot_obs_checks_{scenario_name}.png")
    plt.savefig(out_path, bbox_inches="tight")
    plt.close()
    print(f"[Saved Plot] {out_path}")


# --- Separate CDF plots for repair time and obstacle checks ---

def save_cdf_repair_time_nonanytime(scenario_name, planners_data):
    metrics = extract_non_anytime_per_seed_medians(planners_data)
    if not metrics:
        return
    planners = sorted(metrics.keys(), key=planner_sort_key)
    color_mapping = {'FMTX': '#1f78b4', 'DLITE': '#d62728'}

    fig, ax = plt.subplots(figsize=(6, 5))
    for planner in planners:
        vals = metrics[planner]['t_repair']
        if not vals:
            continue
        sorted_vals = np.sort(vals)
        cdf = np.arange(1, len(sorted_vals)+1) / len(sorted_vals)
        ax.step(sorted_vals, cdf, where='post',
                label=planner, color=color_mapping.get(planner, None),
                linestyle='-', linewidth=2)
    ax.set_xlabel('Repair Time Per Seed (ms)')
    ax.set_ylabel('CDF')
    ax.legend()
    # ax.grid(True, linestyle=':', alpha=0.7)
    plt.tight_layout()
    out_path = os.path.join(BUILD_DIR, f"cdf_repair_time_{scenario_name}.png")
    plt.savefig(out_path, bbox_inches="tight")
    plt.close()
    print(f"[Saved Plot] {out_path}")


def save_cdf_obs_checks_nonanytime(scenario_name, planners_data):
    metrics = extract_non_anytime_per_seed_medians(planners_data)
    if not metrics:
        return
    planners = sorted(metrics.keys(), key=planner_sort_key)
    color_mapping = {'FMTX': '#1f78b4', 'DLITE': '#d62728'}

    fig, ax = plt.subplots(figsize=(6, 5))
    for planner in planners:
        vals = metrics[planner]['obs_repair']
        if not vals:
            continue
        sorted_vals = np.sort(vals)
        cdf = np.arange(1, len(sorted_vals)+1) / len(sorted_vals)
        ax.step(sorted_vals, cdf, where='post',
                label=planner, color=color_mapping.get(planner, None),
                linestyle='-', linewidth=2)
    ax.set_xlabel('Obstacle Checks Per Event (Per Seed Mean)')
    ax.set_ylabel('CDF')
    ax.legend()
    # ax.grid(True, linestyle=':', alpha=0.7)
    plt.tight_layout()
    out_path = os.path.join(BUILD_DIR, f"cdf_obs_checks_{scenario_name}.png")
    plt.savefig(out_path, bbox_inches="tight")
    plt.close()
    print(f"[Saved Plot] {out_path}")


def extract_per_seed_mean_path_costs(planners_data):
    """
    Return a dict: planner -> list of per‑seed mean path costs
    (mean over all set_state / goal_reached events within one seed).
    Only uses paired successful seeds.
    """
    paired_data, _ = filter_paired_seeds(planners_data, target_count=30)
    metrics = {}
    for planner, dfs in paired_data.items():
        valid_dfs = [df for df in dfs if not df.empty]
        if not valid_dfs:
            continue
        avg_cost_list = []
        for df in valid_dfs:
            state_rows = get_state_rows(df)
            if not state_rows.empty:
                avg_cost_list.append(state_rows["path_cost"].dropna().mean())
        if avg_cost_list:
            metrics[planner] = avg_cost_list
    return metrics


def save_boxplot_path_cost_nonanytime(scenario_name, planners_data):
    """Boxplot of per‑seed mean path cost, one box per planner."""
    metrics = extract_per_seed_mean_path_costs(planners_data)
    if not metrics:
        return
    planners = sorted(metrics.keys(), key=planner_sort_key)
    color_mapping = {'FMTX': '#1f78b4', 'DLITE': '#d62728'}

    fig, ax = plt.subplots(figsize=(6, 5))
    data_to_plot = [metrics[p] for p in planners]
    bp = ax.boxplot(data_to_plot,
                    tick_labels=[display_names.get(p, p) for p in planners],
                    patch_artist=True, widths=0.6, showfliers=True)
    for patch, planner in zip(bp['boxes'], planners):
        patch.set_facecolor(color_mapping.get(planner, '#AAAAAA'))
        patch.set_alpha(0.6)
    ax.set_ylabel('Mean Path Cost Per Seed (m)')
    plt.tight_layout()
    out_path = os.path.join(BUILD_DIR, f"boxplot_path_cost_{scenario_name}.png")
    plt.savefig(out_path, bbox_inches="tight")
    plt.close()
    print(f"[Saved Plot] {out_path}")



"""
Combined path‑cost boxplots (per‑seed mean) across three configurations and three state spaces.
Reads sub‑folders: 1000_10obs, 3000_20obs_2, 3000_20obs_3.
For R2T, Dubins and Thruster, extracts the per‑seed mean path cost for each planner.
Produces:
  - one standalone boxplot per state space (grouped by configuration)
  - a combined figure with three vertical subplots (one per state space)
"""

import os, sys
import pandas as pd, numpy as np, matplotlib.pyplot as plt
from matplotlib.patches import Patch

# ======================================================================
#  Copy‑paste all the helper functions from your non‑anytime script here
#  (load_data, is_successful_run, extract_runtime_events,
#   get_state_rows, get_update_rows, filter_paired_seeds,
#   extract_per_seed_mean_path_costs, and the associated constants)
#  For brevity they are not repeated in this answer; assume they exist.
# ======================================================================

# ---------- Global styling ----------
plt.rcParams.update({
    "font.family": "serif",
    "font.serif": ["Times New Roman", "DejaVu Serif", "serif"],
    "axes.labelsize": 11,
    "legend.fontsize": 10,
    "xtick.labelsize": 9,
    "ytick.labelsize": 10,
    "figure.dpi": 300
})

PLANNER_ORDER = ["FMTX", "DLITE"]          # non‑anytime pair (change to ["FMTX","RRTX"] for anytime)
color_map = {"FMTX": "#1f78b4", "DLITE": "#d62728"}
display_names = {
    "FMTX":  r"D-FMT$^*$",
    "DLITE": r"D$^*$Lite"
}

# ---------- Configuration ----------
CONFIG_DIRS = {
    "10 obs / 1k":          "1000_10obs",
    "20 obs / 3k (f=2)":    "3000_20obs_2",
    "20 obs / 3k (f=3)":    "3000_20obs_3"
}
STATE_SPACES = ["R2T", "Dubins", "Thruster"]   # R2 has no path cost


def make_labels(ss, configs_present):
    """Return a list of x‑tick labels for the given state space and config list."""
    labels = []
    for cn in configs_present:
        if cn == "10 obs / 1k":
            f = C1_FACTOR.get(ss, '?')
            labels.append(f"10 Obstacles\nn = 1000, f = {f}")
        elif cn == "20 obs / 3k (f=2)":
            labels.append("20 Obstacles\nn = 3000, f = 2")
        elif cn == "20 obs / 3k (f=3)":
            labels.append("20 Obstacles\nn = 3000, f = 3")
        else:
            labels.append(cn)
    return labels

# ---------- Silent metric extractor (no warnings) ----------
def filter_paired_seeds_silent(planners_data, target_count=1):
    successful_seeds_per_planner = {}
    for planner, dfs in planners_data.items():
        successful_seeds = [df['seed'].iloc[0] for df in dfs if is_successful_run(df)]
        successful_seeds_per_planner[planner] = set(successful_seeds)
    if not successful_seeds_per_planner:
        return planners_data, 0
    common_seeds = set.intersection(*successful_seeds_per_planner.values())
    selected_seeds = list(common_seeds)
    filtered_data = {}
    for planner, dfs in planners_data.items():
        filtered_data[planner] = [df for df in dfs if df['seed'].iloc[0] in selected_seeds]
    return filtered_data, len(selected_seeds)

def extract_per_seed_mean_path_costs_silent(planners_data):
    """Return dict: planner -> list of per‑seed mean path costs."""
    paired_data, _ = filter_paired_seeds_silent(planners_data, target_count=1)
    metrics = {}
    for planner, dfs in paired_data.items():
        valid_dfs = [df for df in dfs if not df.empty]
        if not valid_dfs:
            continue
        avg_cost_list = []
        for df in valid_dfs:
            state_rows = get_state_rows(df)
            if not state_rows.empty:
                avg_cost_list.append(state_rows["path_cost"].dropna().mean())
        if avg_cost_list:
            metrics[planner] = avg_cost_list
    return metrics

# ======================================================================
#  Data collection
# ======================================================================
all_data = {ss: {cn: {} for cn in CONFIG_DIRS} for ss in STATE_SPACES}

for config_label, dir_name in CONFIG_DIRS.items():
    if not os.path.isdir(dir_name):
        print(f"Directory {dir_name} not found, skipping")
        continue
    print(f"\nProcessing {config_label} ({dir_name})")
    scenarios = load_data(dir_name)
    for ss in STATE_SPACES:
        if ss not in scenarios:
            continue
        planners_data = scenarios[ss]
        path_metrics = extract_per_seed_mean_path_costs_silent(planners_data)
        for planner in PLANNER_ORDER:
            all_data[ss][config_label][planner] = path_metrics.get(planner, [])

# ... (all imports and helpers unchanged) ...

C1_FACTOR = {
    "R2T": 2,
    "Dubins": 3,
    "Thruster": 3
}

# Display names for state spaces (titles)
SS_DISPLAY_NAMES = {
    "R2T":       "Holonomic",
    "Dubins":    "Dubins",
    "Thruster":  "Thruster"
}
def draw_grouped_boxplot(ax, ss, config_dirs, labels_for_configs):
    for ci, (cn, lbl) in enumerate(zip(config_dirs, labels_for_configs)):
        base_pos = ci * 2.5
        for pi, planner in enumerate(PLANNER_ORDER):
            vals = all_data[ss][cn].get(planner, [])
            if not vals:
                continue
            pos = base_pos + pi * 0.8
            bp = ax.boxplot([vals], positions=[pos], widths=0.6,
                            patch_artist=True, showfliers=True, manage_ticks=False)
            bp['boxes'][0].set_facecolor(color_map[planner])
            bp['boxes'][0].set_alpha(0.6)

    group_centers = [ci * 2.5 + 0.4 for ci in range(len(config_dirs))]
    ax.set_xticks(group_centers)
    ax.set_xticklabels(labels_for_configs, rotation=0, ha='center')
    ax.set_title(SS_DISPLAY_NAMES.get(ss, ss))
    ax.set_ylabel('Mean Path Cost Per Seed (m)')

# --- Individual figures ---
for ss in STATE_SPACES:
    configs_present = [cn for cn in CONFIG_DIRS
                       if any(len(all_data[ss][cn].get(p, [])) > 0 for p in PLANNER_ORDER)]
    if not configs_present:
        continue
    # labels = []
    # for cn in configs_present:
    #     if cn == "10 obs / 1k":
    #         f = C1_FACTOR.get(ss, '?')
    #         labels.append(f"10 Obstacles\nn = 1000, f={f}")
    #     elif cn == "20 obs / 3k (f=2)":
    #         labels.append("20 Obstacles\nn = 3000, f=2")
    #     elif cn == "20 obs / 3k (f=3)":
    #         labels.append("20 Obstacles\nn = 3000, f=3")
    #     else:
    #         labels.append(cn)

    labels = make_labels(ss, configs_present)


    fig, ax = plt.subplots(figsize=(8, 5))
    draw_grouped_boxplot(ax, ss, configs_present, labels)
    legend_elems = [Patch(facecolor=color_map[p], alpha=0.6,
                          label=display_names.get(p, p)) for p in PLANNER_ORDER]
    ax.legend(handles=legend_elems, loc='upper right')
    plt.tight_layout()
    out = f"path_cost_boxplot_{ss}.pdf"
    plt.savefig(out, bbox_inches='tight')
    plt.close()
    print(f"Saved {out}")

# --- Combined vertical figure ---
fig, axes = plt.subplots(len(STATE_SPACES), 1, figsize=(8, 12), sharex=False)
for ax, ss in zip(axes, STATE_SPACES):
    configs_present = [cn for cn in CONFIG_DIRS
                       if any(len(all_data[ss][cn].get(p, [])) > 0 for p in PLANNER_ORDER)]
    if not configs_present:
        ax.text(0.5, 0.5, 'No data', ha='center', va='center')
        ax.set_title(SS_DISPLAY_NAMES.get(ss, ss))
        continue
    # labels = []
    # for cn in configs_present:
    #     if cn == "10 obs / 1k":
    #         f = C1_FACTOR.get(ss, '?')
    #         labels.append(f"10 obs / 1k (f={f})")
    #     else:
    #         labels.append(cn)

    labels = make_labels(ss, configs_present)
    draw_grouped_boxplot(ax, ss, configs_present, labels)
    if ss == STATE_SPACES[0]:
        legend_elems = [Patch(facecolor=color_map[p], alpha=0.6,
                              label=display_names.get(p, p)) for p in PLANNER_ORDER]
        ax.legend(handles=legend_elems, loc='upper right')

# axes[-1].set_xlabel('Configuration')
plt.tight_layout()
out_combined = "path_cost_combined_vertical.pdf"
plt.savefig(out_combined, bbox_inches='tight')
plt.close()
print(f"Saved {out_combined}")

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