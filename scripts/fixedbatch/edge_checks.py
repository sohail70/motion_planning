#!/usr/bin/env python3
"""
======================================================================================
FIXED-GRAPH EDGE-CHECK FIGURE  (D-FMT* vs D* Lite)
======================================================================================
Builds a compact log-scale dot plot of collision-check ("edge check") cost for the
two fixed-graph planners, as a visual companion to the fixed-batch edge-check table.

Metric (identical to the table):
  - per seed : mean of `obstacle_checks` over `event_type == "update"` rows
  - per group: median of those per-seed means, with Q1--Q3 as the error bar
  - seeds    : paired common-success only, i.e. the intersection of seeds on
               which BOTH planners succeed (collision-free and goal-reaching)

Data source:
  JUL6_newSetROBOT/{1000_10obs, 1000_20obs, 3000_20obs}/
      sim_Kinodynamic{FMTX,PRMStarDStarLite}_{R2,R2T,Dubins,Thruster}_seed_*.csv

Outputs (into ./edge_check_figs/):
  fig_edge_checks_fixed_graph.pdf/.png
  edge_checks_fixed_graph.csv        the plotted numbers, incl. N_pair

Usage:
  python3 edge_checks.py
  python3 edge_checks.py --annotate --ratio
======================================================================================
"""

import argparse
import os
import re
import sys
from collections import defaultdict

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

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
    "grid.linewidth": 0.5,
    "axes.axisbelow": True,
    "figure.dpi": 130,
    "savefig.bbox": "tight",
})

PLANNERS = ["FMTX", "PRMStarDStarLite"]
PRETTY = {"FMTX": r"D-FMT$^{*}$", "PRMStarDStarLite": r"D$^{*}$ Lite"}
COLOR = {"FMTX": "#1b6ca8", "PRMStarDStarLite": "#b7791f"}
MARKER = {"FMTX": "o", "PRMStarDStarLite": "D"}

SPACE_ORDER = ["R2", "R2T", "Dubins", "Thruster"]
SPACE_PRETTY = {
    "R2": "Geometric",
    "R2T": "Holonomic",
    "Dubins": "Dubins",
    "Thruster": "Thruster"
}

# D* Lite must be matched before PRMStar so the alternation does not truncate
FNAME_RE = re.compile(
    r"sim_Kinodynamic(?P<planner>PRMStarDStarLite|FMTX)_"
    r"(?P<space>R2T|R2|Dubins|Thruster)_seed_(?P<seed>\d+)_.*_metrics\.csv$")

USECOLS = ["event_type", "obstacle_checks", "collision_count", "reached_goal"]


def scan_directory(path):
    """-> {(planner, space): {seed: [csv_path, ...]}}

    A seed may have been run more than once, so paths are collected per seed
    rather than overwritten.
    """
    out = defaultdict(lambda: defaultdict(list))
    if not os.path.isdir(path):
        return out
    for fn in sorted(os.listdir(path)):
        m = FNAME_RE.match(fn)
        if m:
            out[(m["planner"], m["space"])][int(m["seed"])].append(os.path.join(path, fn))
    return out


def read_run(path):
    """Read only the three columns the metric needs."""
    try:
        df = pd.read_csv(path, usecols=lambda c: c in USECOLS)
    except Exception:
        return None
    if df.empty or "event_type" not in df.columns:
        return None
    for c in ("obstacle_checks", "collision_count", "reached_goal"):
        if c in df.columns:
            df[c] = pd.to_numeric(df[c], errors="coerce")
    return df


def collision_free(df):
    if df is None or df.empty:
        return False
    if "collision_count" in df.columns:
        return df["collision_count"].fillna(0).max() == 0
    return True


def reached_goal(df):
    """Run terminated at the goal, by flag or by terminal event."""
    if df is None or df.empty:
        return False
    if "reached_goal" in df.columns and df["reached_goal"].fillna(0).max() == 1:
        return True
    return bool((df["event_type"] == "goal_reached").any())


def mean_checks_per_update(df):
    if df is None or "obstacle_checks" not in df.columns:
        return np.nan
    upd = df[df["event_type"] == "update"]
    if upd.empty:
        return np.nan
    v = upd["obstacle_checks"].dropna()
    return v.mean() if len(v) else np.nan


def group_stats(runs_by_planner, success_mode="auto"):
    """Success is collision-free plus goal-reaching, the latter required only in
    scenarios that actually terminate at the goal."""
    present = [p for p in PLANNERS if runs_by_planner.get(p)]
    if len(present) < 2:
        return {}, 0
    cache, cfree, goal = defaultdict(list), {}, {}
    for p in present:
        cfree[p], goal[p] = set(), set()
        for seed, paths in runs_by_planner[p].items():
            for path in paths:
                df = read_run(path)
                cache[(p, seed)].append(df)
                # a seed counts as successful if any of its runs succeeded
                if collision_free(df):
                    cfree[p].add(seed)
                if reached_goal(df):
                    goal[p].add(seed)
    rule = success_mode
    if success_mode == "auto":
        rule = "goal" if any(goal[p] for p in present) else "collision"
    ok = ({p: cfree[p] & goal[p] for p in present} if rule == "goal"
          else {p: cfree[p] for p in present})
    common = set.intersection(*(ok[p] for p in present))
    if not common:
        return {}, 0
    stats = {}
    for p in present:
        vals = [mean_checks_per_update(df)
                for s in sorted(common) for df in cache[(p, s)]]
        vals = np.array([v for v in vals if np.isfinite(v) and v > 0], dtype=float)
        if vals.size:
            stats[p] = (float(np.median(vals)),
                        float(np.percentile(vals, 25)),
                        float(np.percentile(vals, 75)))
    return stats, len(common)


def collect(base, sm="auto"):
    cfgs = []
    for d in sorted(os.listdir(base)):
        m = re.match(r"(\d+)_(\d+)obs$", d)
        if m and os.path.isdir(os.path.join(base, d)):
            cfgs.append((int(m[1]), int(m[2]), os.path.join(base, d)))
    cfgs.sort(key=lambda t: (t[0], t[1]))

    groups = []
    for space in SPACE_ORDER:
        for n, n_obs, path in cfgs:
            found = scan_directory(path)
            rbp = {p: found.get((p, space), {}) for p in PLANNERS}
            if not any(rbp.values()):
                continue
            stats, npair = group_stats(rbp, sm)
            if stats:
                groups.append((f"$n$={n}\n$|O|$={n_obs}", stats, npair, space))
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


def main():
    ap = argparse.ArgumentParser()
    here = os.path.dirname(os.path.abspath(__file__))
    ap.add_argument("--base", default=os.path.join(here, "JUL6_newSetROBOT"))
    ap.add_argument("--out", default=os.path.join(here, "edge_check_figs"))
    ap.add_argument("--annotate", action="store_true",
                    help="print median values above each marker")
    ap.add_argument("--success", choices=["auto", "goal", "collision"], default="auto",
                    help="success rule for pairing; auto requires goal-reaching only "
                         "in scenarios that terminate at the goal")
    ap.add_argument("--ratio", action="store_true",
                    help="add a lower panel with the D* Lite / D-FMT* check ratio")
    args = ap.parse_args()

    if not os.path.isdir(args.base):
        sys.exit(f"error: data directory not found: {args.base}")

    print(f"Reading {args.base}")
    groups = collect(args.base, args.success)
    if not groups:
        sys.exit("error: no usable runs found")

    # ------------------------------------------------------------------
    # 2x2 figure: one panel per state space
    # ------------------------------------------------------------------
    fig, axes = plt.subplots(
        2, 2,
        figsize=(7.5, 6.0),
        sharey=True
    )
    axes = axes.flatten()

    offs = {
        p: (i - 0.5) * 0.22
        for i, p in enumerate(PLANNERS)
    }

    for ax, space in zip(axes, SPACE_ORDER):

        # Only the three configurations belonging to this state space.
        space_groups = [g for g in groups if g[3] == space]

        xs = np.arange(len(space_groups))

        for p in PLANNERS:
            x_pts, y_pts, lo, hi = [], [], [], []

            for gi, (_, stats, _, _) in enumerate(space_groups):
                if p not in stats:
                    continue

                med, q1, q3 = stats[p]

                x_pts.append(xs[gi] + offs[p])
                y_pts.append(med)
                lo.append(max(med - q1, 0))
                hi.append(max(q3 - med, 0))

            ax.errorbar(
                x_pts,
                y_pts,
                yerr=[lo, hi],
                fmt=MARKER[p],
                color=COLOR[p],
                markersize=6,
                markeredgecolor="white",
                markeredgewidth=0.6,
                elinewidth=1.5,
                capsize=3,
                linestyle="none",
                zorder=3,
                label=PRETTY[p]
            )

            if args.annotate:
                for x, y in zip(x_pts, y_pts):
                    ax.annotate(
                        f"{y:,.0f}",
                        (x, y),
                        textcoords="offset points",
                        xytext=(0, 8),
                        ha="center",
                        fontsize=6.5,
                        color=COLOR[p],
                        rotation=90
                    )

        # Log collision-check axis.
        ax.set_yscale("log")
        ax.grid(True, which="minor", axis="y", alpha=0.12, lw=0.4)
        ax.grid(False, axis="x")

        # State-space title.
        ax.set_title(SPACE_PRETTY[space], fontsize=10)

        # Three n / |O| configurations.
        ax.set_xticks(xs)
        ax.set_xticklabels(
            [g[0] for g in space_groups],
            fontsize=8,
            ha="center",
            multialignment="center"
        )

        ax.set_xlim(-0.55, len(space_groups) - 0.45)

        # N = number of paired common-success seeds for each configuration.
        for gi, (_, _, npair, _) in enumerate(space_groups):
            ax.annotate(
                rf"$N$={npair}",
                (xs[gi], 0.018),
                xycoords=("data", "axes fraction"),
                ha="center",
                va="bottom",
                fontsize=6.5,
                color="0.35"
            )

    # Y-axis labels only on left column.
    axes[0].set_ylabel("Collision checks per update\n(log scale)")
    axes[2].set_ylabel("Collision checks per update\n(log scale)")
    axes[1].set_ylabel("")
    axes[3].set_ylabel("")

    # One shared legend below the full 2x2 figure.
    handles = [
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

    fig.legend(
        handles=handles,
        loc="lower center",
        ncol=2,
        frameon=False,
        bbox_to_anchor=(0.5, 0.01)
    )

    fig.subplots_adjust(
        left=0.10,
        right=0.98,
        top=0.95,
        bottom=0.14,
        wspace=0.16,
        hspace=0.34
    )

    os.makedirs(args.out, exist_ok=True)

    for ext in ("pdf", "png"):
        fig.savefig(
            os.path.join(
                args.out,
                f"fig_edge_checks_fixed_graph.{ext}"
            )
        )

    print(
        f"  wrote "
        f"{os.path.join(args.out, 'fig_edge_checks_fixed_graph')}.pdf/.png"
    )

    plt.close(fig)

    rows = []
    for label, stats, npair, space in groups:
        for p in PLANNERS:
            if p in stats:
                med, q1, q3 = stats[p]
                rows.append({"config": label.replace("\n", " "),
                             "state_space": space,
                             "planner": PRETTY[p],
                             "median": round(med, 1),
                             "q1": round(q1, 1),
                             "q3": round(q3, 1),
                             "n_pair": npair})
    df = pd.DataFrame(rows)
    csv = os.path.join(args.out, "edge_checks_fixed_graph.csv")
    df.to_csv(csv, index=False)
    print(f"  wrote {csv}")
    print()
    print(df.to_string(index=False))


if __name__ == "__main__":
    main()
