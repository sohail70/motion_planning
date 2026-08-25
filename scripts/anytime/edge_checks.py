#!/usr/bin/env python3
"""
======================================================================================
ANYTIME COLLISION-CHECK FIGURES  (Simulations I, II, III)
======================================================================================
Builds compact log-scale dot plots of collision-check ("edge check") cost for the
three anytime planners, as a drop-in visual replacement for the three anytime
edge-check tables.

Metric (identical to the tables, see analyze_anytime_agg.py):
  A "repair" is an `update` row together with the `plan` row that immediately
  follows it, if any. Checks per repair are counted as

      RRT^X          : update_obstacle_checks               (eager, all work in update)
      FMT^X / LLPT*  : update_obstacle_checks + plan_obstacle_checks   (lazy)

  - per seed : mean checks per repair
  - per group: median of those per-seed means, with Q1--Q3 as the error bar
  - seeds    : paired common-success only, i.e. the intersection of seeds on
               which EVERY compared planner succeeds. Success is collision-free
               plus goal-reaching, the latter required only in scenarios that
               actually terminate at the goal (see --success).

Data source:
  JUL6NEW_setrobotstate/
    SIM1_coneOFF/results_samples_{m}_{k}obs          (4 state spaces)
    SIM2_coneON/results_Thruster_samples_1_time_{T}  (Thruster)
    SIM3_partially_observable/results_samples_{m}_dubins

Outputs (into ./edge_check_figs/):
  fig_edge_checks_anytime.pdf/.png   combined 3-panel figure
  fig_edge_checks_sim{1,2,3}.pdf/.png  individual panels
  edge_checks_anytime.csv            the plotted numbers, incl. N_pair

Usage:
  python3 edge_checks.py
  python3 edge_checks.py --annotate --base /path/to/JUL6NEW_setrobotstate
======================================================================================
"""

import argparse
import os
import pickle
import re
import sys
from collections import defaultdict

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

# ----------------------------------------------------------------------------------
# Style
# ----------------------------------------------------------------------------------
plt.rcParams.update({
    "font.family": "serif",
    "font.serif": ["Times New Roman", "DejaVu Serif", "serif"],
    "mathtext.fontset": "dejavuserif",
    "axes.labelsize": 11,
    "axes.titlesize": 11,
    "xtick.labelsize": 9,
    "ytick.labelsize": 9,
    "legend.fontsize": 9,
    "axes.grid": True,
    "grid.alpha": 0.25,
    "grid.linestyle": "-",
    "grid.linewidth": 0.5,
    "axes.axisbelow": True,
    "figure.dpi": 130,
    "savefig.bbox": "tight",
})

# Planner display order, labels, colors, markers
PLANNERS = ["LLPTStar", "ANYFMTX", "ANYRRTX"]
PRETTY = {"ANYFMTX": r"FMT$^{\mathrm{X}}$",
          "ANYRRTX": r"RRT$^{\mathrm{X}}$",
          "LLPTStar": r"LLPT$^{*}$"}
COLOR = {"ANYFMTX": "#1b6ca8",   # blue
         "ANYRRTX": "#c0392b",   # red
         "LLPTStar": "#1e8449"}  # green
MARKER = {"ANYFMTX": "o", "ANYRRTX": "s", "LLPTStar": "^"}

SPACE_ORDER = ["R2", "R2T", "Dubins", "Thruster"]
SPACE_PRETTY = {
    "R2": "Geometric",
    "R2T": "Holonomic",
    "Dubins": "Dubins",
    "Thruster": "Thruster"
}

FNAME_RE = re.compile(
    r"sim_Kinodynamic(?P<planner>ANYFMTX|ANYRRTX|LLPTStar)_"
    r"(?P<space>R2T|R2|Dubins|Thruster)_seed_(?P<seed>\d+)_.*_metrics\.csv$")


# ----------------------------------------------------------------------------------
# Loading / metric extraction
# ----------------------------------------------------------------------------------
def scan_directory(path):
    """-> {(planner, space): {seed: csv_path}}"""
    out = defaultdict(dict)
    if not os.path.isdir(path):
        return out
    for fn in os.listdir(path):
        m = FNAME_RE.match(fn)
        if m:
            out[(m["planner"], m["space"])][int(m["seed"])] = os.path.join(path, fn)
    return out


USECOLS = ["row_id", "event_type", "obstacle_checks", "collision_count", "reached_goal"]


def read_run(path):
    """Read only the columns the metric needs, in logged order."""
    try:
        df = pd.read_csv(path, usecols=lambda c: c in USECOLS)
    except Exception:
        return None
    if df.empty or "event_type" not in df.columns:
        return None
    for c in ("row_id", "obstacle_checks", "collision_count", "reached_goal"):
        if c in df.columns:
            df[c] = pd.to_numeric(df[c], errors="coerce")
    if "row_id" in df.columns:
        df = df.sort_values("row_id").reset_index(drop=True)
    return df


def collision_free(df):
    if df is None or df.empty:
        return False
    if "collision_count" in df.columns:
        return df["collision_count"].fillna(0).max() == 0
    return True


def reached_goal(df):
    if df is None or df.empty or "reached_goal" not in df.columns:
        return False
    return df["reached_goal"].fillna(0).max() == 1


def checks_per_repair(df, planner):
    """
    Mean checks per repair. A repair is an `update` row plus the `plan` row
    immediately following it. RRT^X does all repair work inside `update`, so its
    trailing `plan` row is anytime expansion rather than repair and is excluded.
    """
    if df is None or "obstacle_checks" not in df.columns:
        return np.nan
    ev = df["event_type"].to_numpy()
    oc = df["obstacle_checks"].fillna(0).to_numpy()
    idx = np.flatnonzero(ev == "update")
    if idx.size == 0:
        return np.nan
    total = oc[idx].astype(float)
    if planner != "ANYRRTX":
        nxt = idx + 1
        valid = (nxt < len(ev)) & (ev[np.clip(nxt, 0, len(ev) - 1)] == "plan")
        total = total + np.where(valid, oc[np.clip(nxt, 0, len(ev) - 1)], 0.0)
    return float(total.mean())


def group_stats(runs_by_planner, success_mode="auto"):
    """
    runs_by_planner: {planner: {seed: path}} for ONE comparable group.
    Returns (stats, n_pair, rule) with stats = {planner: (median, q1, q3)}.
    """
    present = [p for p in PLANNERS if runs_by_planner.get(p)]
    if len(present) < 2:
        return {}, 0, "-"

    cache, cfree, goal = {}, {}, {}
    for p in present:
        cfree[p], goal[p] = set(), set()
        for seed, path in runs_by_planner[p].items():
            df = read_run(path)
            cache[(p, seed)] = df
            if collision_free(df):
                cfree[p].add(seed)
            if reached_goal(df):
                goal[p].add(seed)

    # "auto": require goal-reaching only where the scenario terminates at the goal
    any_goal = any(goal[p] for p in present)
    rule = success_mode
    if success_mode == "auto":
        rule = "goal" if any_goal else "collision"

    if rule == "goal":
        ok = {p: cfree[p] & goal[p] for p in present}
    else:
        ok = {p: cfree[p] for p in present}

    common = set.intersection(*(ok[p] for p in present))
    if not common:
        return {}, 0, rule

    stats = {}
    for p in present:
        vals = [checks_per_repair(cache[(p, s)], p) for s in sorted(common)]
        vals = np.array([v for v in vals if np.isfinite(v)], dtype=float)
        if vals.size:
            stats[p] = (float(np.median(vals)),
                        float(np.percentile(vals, 25)),
                        float(np.percentile(vals, 75)))
    return stats, len(common), rule


# ----------------------------------------------------------------------------------
# Simulation collectors -> list of (label, stats, n_pair)
# ----------------------------------------------------------------------------------
def collect_sim1(base, sm="auto"):
    root = os.path.join(base, "SIM1_coneOFF")
    cfgs = []
    for d in sorted(os.listdir(root)) if os.path.isdir(root) else []:
        m = re.match(r"results_samples_(\d+)_(\d+)obs$", d)
        if m:
            cfgs.append((int(m[1]), int(m[2]), os.path.join(root, d)))
    cfgs.sort(key=lambda t: (t[1], t[0]))

    groups = []
    for space in SPACE_ORDER:
        for m_s, n_obs, path in cfgs:
            found = scan_directory(path)
            rbp = {p: found.get((p, space), {}) for p in PLANNERS}
            if not any(rbp.values()):
                continue
            stats, npair, rule = group_stats(rbp, sm)
            if stats:
                groups.append((f"$m$={m_s}\n$|O|$={n_obs}", stats, npair, space, rule))
    return groups


def block_spans(groups):
    """Contiguous runs of the same state space -> (start, end_inclusive, space)."""
    spans, i = [], 0
    while i < len(groups):
        j = i
        while j + 1 < len(groups) and groups[j + 1][3] == groups[i][3]:
            j += 1
        spans.append((i, j, groups[i][3]))
        i = j + 1
    return spans


def collect_sim2(base, sm="auto"):
    root = os.path.join(base, "SIM2_coneON")
    out = []
    if not os.path.isdir(root):
        return out
    dirs = []
    for d in os.listdir(root):
        m = re.match(r"results_Thruster_samples_(\d+)_time_([\d.]+)$", d)
        if m:
            dirs.append((float(m[2]), os.path.join(root, d)))
    for T, path in sorted(dirs):
        found = scan_directory(path)
        rbp = {p: found.get((p, "Thruster"), {}) for p in PLANNERS}
        stats, npair, rule = group_stats(rbp, sm)
        if stats:
            out.append((f"$T$={T:g}s", stats, npair, "Thruster", rule))
    return out


def collect_sim3(base, sm="auto"):
    root = os.path.join(base, "SIM3_partially_observable")
    out = []
    if not os.path.isdir(root):
        return out
    dirs = []
    for d in os.listdir(root):
        m = re.match(r"results_samples_(\d+)_dubins$", d)
        if m:
            dirs.append((int(m[1]), os.path.join(root, d)))
    for m_s, path in sorted(dirs):
        found = scan_directory(path)
        rbp = {p: found.get((p, "Dubins"), {}) for p in PLANNERS}
        stats, npair, rule = group_stats(rbp, sm)
        if stats:
            out.append((f"$m$={m_s}", stats, npair, "Dubins", rule))
    return out


# ----------------------------------------------------------------------------------
# Plotting
# ----------------------------------------------------------------------------------
def draw_panel(ax, groups, title, annotate=False, show_npair=True, sep_on_space=False):
    """One panel: x = configuration groups, y = collision checks per repair (log scale), IQR bars."""
    n = len(groups)
    xs = np.arange(n)
    offs = {p: (i - (len(PLANNERS) - 1) / 2) * 0.22 for i, p in enumerate(PLANNERS)}

    for p in PLANNERS:
        x_pts, y_pts, lo, hi = [], [], [], []
        for gi, (_, stats, _, _, _) in enumerate(groups):
            if p not in stats:
                continue
            med, q1, q3 = stats[p]
            x_pts.append(xs[gi] + offs[p])
            y_pts.append(med)
            lo.append(max(med - q1, 0))
            hi.append(max(q3 - med, 0))
        if not x_pts:
            continue
        ax.errorbar(x_pts, y_pts, yerr=[lo, hi], fmt=MARKER[p], color=COLOR[p],
                    markersize=5.5, markeredgecolor="white", markeredgewidth=0.6,
                    elinewidth=1.4, capsize=2.6, linestyle="none", zorder=3,
                    label=PRETTY[p])
        if annotate:
            for x, y in zip(x_pts, y_pts):
                ax.annotate(f"{y:,.0f}", (x, y), textcoords="offset points",
                            xytext=(0, 7), ha="center", fontsize=6.2,
                            color=COLOR[p], rotation=90)

    # state space named once per block, above the axes (Simulation I)
    if sep_on_space:
        for start, end, space in block_spans(groups):
            if start:
                ax.axvline(start - 0.5, color="0.55", lw=0.8, ls=(0, (4, 3)), zorder=1)
            ax.annotate(SPACE_PRETTY[space], ((start + end) / 2, 1.015),
                        xycoords=("data", "axes fraction"), ha="center",
                        va="bottom", fontsize=9)

    ax.set_yscale("log")
    ax.set_xticks(xs)
    ax.set_xticklabels([g[0] for g in groups], fontsize=8)
    ax.set_xlim(-0.6, n - 0.4)
    if title:
        ax.set_title(title, pad=22 if sep_on_space else 8)
    ax.grid(True, which="major", axis="y")
    ax.grid(True, which="minor", axis="y", alpha=0.12, lw=0.4)
    ax.grid(False, axis="x")

    if show_npair:
        npairs = [g[2] for g in groups]

        for gi, np_ in enumerate(npairs):
            ax.annotate(
                rf"$N$={np_}",
                (xs[gi], 0.015),
                xycoords=("data", "axes fraction"),
                ha="center",
                va="bottom",
                fontsize=6.5,
                color="0.35"
            )
    return ax

def legend_handles():
    return [
        Line2D(
            [0], [0],
            marker=MARKER[p],
            linestyle="none",
            markersize=6,
            markerfacecolor=COLOR[p],
            markeredgecolor="white",
            markeredgewidth=0.6,
            color=COLOR[p],
            label=PRETTY[p]
        )
        for p in PLANNERS
    ]
def save(fig, outdir, stem):
    os.makedirs(outdir, exist_ok=True)
    for ext in ("pdf", "png"):
        fig.savefig(os.path.join(outdir, f"{stem}.{ext}"))
    print(f"  wrote {os.path.join(outdir, stem)}.pdf/.png")


def main():
    ap = argparse.ArgumentParser()
    here = os.path.dirname(os.path.abspath(__file__))
    ap.add_argument("--base", default=os.path.join(here, "JUL6NEW_setrobotstate"))
    ap.add_argument("--out", default=os.path.join(here, "edge_check_figs"))
    ap.add_argument("--annotate", action="store_true",
                    help="print median values above each marker")
    ap.add_argument("--no-sharey", action="store_true",
                    help="give each panel its own y-axis (default: shared)")
    ap.add_argument("--refresh", action="store_true",
                    help="recompute from CSVs instead of using the cache")
    ap.add_argument("--success", choices=["auto", "goal", "collision"], default="auto",
                    help="success rule for pairing; auto requires goal-reaching only "
                         "in scenarios that terminate at the goal")
    args = ap.parse_args()

    if not os.path.isdir(args.base):
        sys.exit(f"error: data directory not found: {args.base}")

    os.makedirs(args.out, exist_ok=True)
    cache_path = os.path.join(args.out, f"_cache_{args.success}.pkl")
    if os.path.exists(cache_path) and not args.refresh:
        print(f"Using cache {cache_path}  (--refresh to rescan)")
        with open(cache_path, "rb") as fh:
            sims = pickle.load(fh)
    else:
        print(f"Reading {args.base}  (this scans every run CSV)")

        sims = [
            ("Simulation I",   collect_sim1(args.base, args.success), True),
            ("Simulation II",  collect_sim2(args.base, args.success), False),
            ("Simulation III", collect_sim3(args.base, args.success), False),
        ]

        sims = [(t, g, sp) for t, g, sp in sims if g]

        with open(cache_path, "wb") as fh:
            pickle.dump(sims, fh)
    if not sims:
        sys.exit("error: no usable runs found")

    # ---- individual figures ------------------------------------------------------
    for i, (title, groups, sep) in enumerate(sims, start=1):

        # ----------------------------------------------------------------------
        # Simulation I: 2x2 layout, one state space per panel
        # ----------------------------------------------------------------------
        if i == 1:
            f, axes = plt.subplots(
                2, 2,
                figsize=(7.4, 6.0),
                sharey=not args.no_sharey
            )
            axes = axes.flatten()

            for ax, space in zip(axes, SPACE_ORDER):
                space_groups = [g for g in groups if g[3] == space]

                draw_panel(
                    ax,
                    space_groups,
                    SPACE_PRETTY[space],
                    annotate=args.annotate,
                    show_npair=True,
                    sep_on_space=False
                )

            # Y-axis labels only on left column.
            axes[0].set_ylabel("Collision checks per repair\n(log scale)")
            axes[2].set_ylabel("Collision checks per repair\n(log scale)")
            axes[1].set_ylabel("")
            axes[3].set_ylabel("")

            # Remove individual legends created by draw_panel().
            for ax in axes:
                leg = ax.get_legend()
                if leg is not None:
                    leg.remove()

            # One shared legend below the whole figure.
            handles = legend_handles()

            f.legend(
                handles=handles,
                frameon=False,
                ncol=3,
                loc="lower center",
                bbox_to_anchor=(0.5, 0.01)
            )

            f.subplots_adjust(
                left=0.10,
                right=0.98,
                top=0.95,
                bottom=0.14,
                wspace=0.18,
                hspace=0.34
            )

        # ----------------------------------------------------------------------
        # Simulations II and III: single-panel layout
        # ----------------------------------------------------------------------
        else:
            f, a = plt.subplots(
                figsize=(max(4.0, 0.95 * len(groups) + 1.8), 3.5)
            )

            draw_panel(
                a,
                groups,
                "",
                annotate=args.annotate,
                show_npair=True,
                sep_on_space=False
            )

            a.set_ylabel("Collision checks per repair\n(log scale)")

            a.legend(
                handles=legend_handles(),
                frameon=False,
                ncol=3,
                loc="upper center",
                bbox_to_anchor=(0.5, -0.14)
            )

            f.subplots_adjust(
                top=0.92,
                bottom=0.32
            )

        # IMPORTANT: save each figure INSIDE the loop.
        save(f, args.out, f"fig_edge_checks_sim{i}")
        plt.close(f)
    # --------------------------------------------------------------------------
    # Simulations II and III: keep single-panel layout
    # --------------------------------------------------------------------------
    else:
        f, a = plt.subplots(
            figsize=(max(4.0, 0.95 * len(groups) + 1.8), 3.5)
        )

        draw_panel(
            a,
            groups,
            "",
            annotate=args.annotate,
            show_npair=True,
            sep_on_space=False
        )

        a.set_ylabel("Collision checks per repair\n(log scale)")

        a.legend(
            handles=legend_handles(),
            frameon=False,
            ncol=3,
            loc="upper center",
            bbox_to_anchor=(0.5, -0.14)
        )

        f.subplots_adjust(
            top=0.92,
            bottom=0.32
        )

    save(f, args.out, f"fig_edge_checks_sim{i}")
    plt.close(f)
    # ---- numbers -----------------------------------------------------------------
    rows = []
    for title, groups, _ in sims:
        for label, stats, npair, space, rule in groups:
            for p in PLANNERS:
                if p in stats:
                    med, q1, q3 = stats[p]
                    rows.append({"simulation": title,
                                 "config": label.replace("\n", " "),
                                 "state_space": space,
                                 "planner": PRETTY[p],
                                 "median": round(med, 1),
                                 "q1": round(q1, 1),
                                 "q3": round(q3, 1),
                                 "n_pair": npair,
                                 "success_rule": rule})
    df = pd.DataFrame(rows)
    csv = os.path.join(args.out, "edge_checks_anytime.csv")
    df.to_csv(csv, index=False)
    print(f"  wrote {csv}")
    print()
    print(df.to_string(index=False))


if __name__ == "__main__":
    main()
