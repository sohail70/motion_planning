#!/usr/bin/env python3
"""
paper_report.py  --  cached, two-stage reporting for the anytime FMTX / RRTX / LLPT* ablation.

Why two stages?  Loading + synthesizing the raw sim_*_metrics.csv is the slow part. We do it
ONCE into a tidy per-seed cache (pickle); after that, tables and figures render instantly, so
you can tweak axes/labels/figures without ever re-reading the CSVs.

    python paper_report.py build      # (re)build the cache from the raw CSV directories  [SLOW]
    python paper_report.py render     # tables (terminal + LaTeX) + figures from cache    [FAST]
    python paper_report.py all        # build then render
    python paper_report.py            # render if cache exists, else build then render

It reuses the tuned loaders / planner-aware repair semantics from analyze_anytime2.py
(imported, never re-implemented), so the numbers match your existing per-sim tables exactly.
"""

import os
import sys
import pickle
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")  # headless: just write files
import matplotlib.pyplot as plt

import analyze_anytime2 as aa  # tuned loaders + repair pairing live here
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
# ===========================================================================================
# CONFIG  --  EDIT THESE THREE PATHS to point at your sim output directories.
# Each directory holds the sim_*_metrics.csv for ONE (m, #obstacles) setting (all 4 spaces).
# ===========================================================================================
_BASE = "."
CONFIGS = {
    "m=1, 10 obs": f"{_BASE}/results_samples_1_10obs",   # <-- EDIT  (sim 1)
    "m=1, 20 obs": f"{_BASE}/results_samples_1_20obs",   # <-- EDIT  (sim 2)
    "m=3, 20 obs": f"{_BASE}/results_samples_3_20obs",   # <-- EDIT  (sim 3)
}
CONFIG_ORDER = list(CONFIGS.keys())
# Display labels. Cache keys above stay ASCII (so no rebuild is needed); figures use mathtext.
CONFIG_FIG_LABEL = {
    "m=1, 10 obs": r"$m{=}1$" "\n" r"$|O|{=}10$",
    "m=1, 20 obs": r"$m{=}1$" "\n" r"$|O|{=}20$",
    "m=3, 20 obs": r"$m{=}3$" "\n" r"$|O|{=}20$",
}

CONFIG_TXT_LABEL = {
    "m=1, 10 obs": "m=1, |O|=10",
    "m=1, 20 obs": "m=1, |O|=20",
    "m=3, 20 obs": "m=3, |O|=20",
}
def _cfg_fig_labels():
    return [CONFIG_FIG_LABEL.get(c, c) for c in CONFIG_ORDER]

CACHE_PATH = "paper_cache.pkl"
FIG_DIR    = "paper_out"

# Raw scenario name (from filenames)  ->  paper label
SCENARIO_RENAME = {
    "R2": "Geometric",
    "R2T": "Holonomic",
    "Dubins": "Dubins",
    "Thruster": "Thruster"
}

SCENARIO_ORDER = ["Geometric", "Holonomic", "Dubins", "Thruster"]
# Spaces where the robot actually moves (success + executed quality are meaningful)
DYNAMIC_SCENARIOS = ["Holonomic", "Dubins", "Thruster"]

PLANNER_ORDER  = ["FMTX", "RRTX", "LLPTStar"]
PLANNER_LABEL  = {"FMTX": r"FMT$^{\mathrm{X}}$", "RRTX": r"RRT$^{\mathrm{X}}$", "LLPTStar": r"LLPT$^{*}$"}
PLANNER_COLORS = {
    "FMTX": "#1b6ca8",
    "RRTX": "#c0392b",
    "LLPTStar": "#1e8449"
}

# Per-scenario "domain" quality metric (path length is shown for all).
# effort = integral of |acceleration| over time  ->  m/s^2 * s = m/s.
DOMAIN_METRIC = {"Dubins":   ("exec_turn",   "Turn (rad)"),
                 "Thruster": ("exec_effort", r"Control effort $\int|a|\,dt$  (m/s)")}


# ===========================================================================================
# STAGE 1 — BUILD CACHE (slow)
# ===========================================================================================
def extract_seed_metrics(df, planner):
    """One row of per-seed metrics, mirroring analyze_group_statistics' inner loop, including
    the planner-aware repair semantics
        (RRTX repairs in update; FMTX and LLPT* use update+next-plan)."""
    m, pu = {}, planner.upper()
    rt = aa.extract_runtime_events(df)

    if "event_type" in df.columns:
        di = df[df["event_type"] == "initial_plan"]
        if not di.empty and "plan_ms" in di.columns:
            v = di["plan_ms"].dropna()
            if not v.empty:
                m["t_init"] = v.median()

    run_update      = aa.get_update_rows(rt)
    run_repair_plan = aa.get_first_repair_plan_rows(rt)
    run_steady      = aa.get_steady_plan_rows(rt)
    run_repairs     = aa.pair_repair_events(rt)

    # --- repair-time series (planner-aware) ---
    if pu == "RRTX":
        s = run_update["update_ms"].dropna() if not run_update.empty else pd.Series(dtype=float)
    else:
        s = run_repairs["t_repair_ms"].dropna() if not run_repairs.empty else pd.Series(dtype=float)
    if not s.empty:
        m["t_repair"]     = s.median()
        m["t_repair_p99"] = s.quantile(0.99)
        m["t_repair_max"] = s.max()

    # --- obstacle checks per repair (planner-aware) ---
    if not run_repairs.empty:
        if pu == "RRTX":
            tot = run_repairs["update_obstacle_checks"].fillna(0)
        else:
            tot = (run_repairs["update_obstacle_checks"].fillna(0)
                   + run_repairs["repair_plan_obstacle_checks"].fillna(0))
        m["obs_per_repair"] = tot.mean()

    if not run_update.empty:
        m["t_upd"] = run_update["update_ms"].dropna().median()
    if not run_repair_plan.empty:
        m["t_pln"]       = run_repair_plan["plan_ms"].dropna().median()
        m["obs_rep_pln"] = run_repair_plan["obstacle_checks"].dropna().mean()
    if not run_steady.empty:
        m["t_add"]      = run_steady["plan_ms"].dropna().median()
        m["obs_steady"] = run_steady["obstacle_checks"].dropna().mean()

    # --- executed solution quality at the terminal goal row ---
    term = aa.get_success_terminal_row(df)
    if not term.empty:
        r = term.iloc[-1]
        for col in ("exec_length", "exec_time", "exec_turn", "exec_effort"):
            if col in term.columns:
                v = pd.to_numeric(r.get(col), errors="coerce")
                if pd.notna(v) and np.isfinite(v) and v > 0:
                    m[col] = float(v)

    # --- graph stats ---
    if "tree_size" in df.columns and not df.empty:
        sv = pd.to_numeric(df.iloc[0]["tree_size"], errors="coerce")
        ev = pd.to_numeric(df.iloc[-1]["tree_size"], errors="coerce")
        if pd.notna(sv): m["start_v"] = sv
        if pd.notna(ev): m["end_v"] = ev
    rc = "neighborhood_radius" if "neighborhood_radius" in df.columns else (
         "radius" if "radius" in df.columns else None)
    if rc and not df.empty:
        sr = pd.to_numeric(df.iloc[0][rc], errors="coerce")
        er = pd.to_numeric(df.iloc[-1][rc], errors="coerce")
        if pd.notna(sr): m["start_r"] = sr
        if pd.notna(er): m["end_r"] = er
    # Dynamic runs use "goal_reached"; geometric R2 runs use "sim_end".
    rterm = aa.get_terminal_rows(df)

    if "event_type" in df.columns:
        event = df["event_type"].astype(str).str.strip().str.lower()
        sim_end = df[event.eq("sim_end")]

        # For R2, sim_end is the authoritative final graph snapshot.
        if not sim_end.empty:
            rterm = sim_end

    if not rterm.empty:
        rterm = rterm.tail(1)

        if "avg_deg_in" in rterm.columns:
            value = pd.to_numeric(
                rterm["avg_deg_in"].iloc[-1], errors="coerce"
            )
            if pd.notna(value) and np.isfinite(value):
                m["deg_in"] = float(value)

        if "avg_deg_out" in rterm.columns:
            value = pd.to_numeric(
                rterm["avg_deg_out"].iloc[-1], errors="coerce"
            )
            if pd.notna(value) and np.isfinite(value):
                m["deg_out"] = float(value)


    return m


def build_cache():
    rows = []
    for cfg in CONFIG_ORDER:
        d = CONFIGS[cfg]
        scenarios = aa.load_data(d)
        if not scenarios:
            print(f"[WARN] no sim_*_metrics.csv found in '{d}' for config '{cfg}'")
            continue
        for scen, planners_data in scenarios.items():
            aa.CURRENT_SCENARIO = scen  # so is_successful_run handles R2 (always-True) correctly
            for planner, dfs in planners_data.items():
                for df in dfs:
                    if df.empty:
                        continue
                    seed = int(df["seed"].iloc[0]) if "seed" in df.columns else -1
                    rec = {"config": cfg, "scenario": scen, "planner": planner,
                           "seed": seed, "success": bool(aa.is_successful_run(df))}
                    rec.update(extract_seed_metrics(df, planner))
                    rows.append(rec)
        print(f"[build] config '{cfg}': {sum(len(p) for s in scenarios.values() for p in s.values())} runs")

    tidy = pd.DataFrame(rows)
    if tidy.empty:
        print("[build] ERROR: no data extracted. Check the CONFIGS paths.")
        sys.exit(1)
    tidy["scenario"] = tidy["scenario"].map(lambda s: SCENARIO_RENAME.get(s, s))
    with open(CACHE_PATH, "wb") as f:
        pickle.dump(tidy, f)
    print(f"[build] cached {len(tidy)} per-seed rows -> {CACHE_PATH}")
    return tidy


def load_cache():
    if not os.path.exists(CACHE_PATH):
        print(f"[render] cache '{CACHE_PATH}' missing -> building first ...")
        return build_cache()

    with open(CACHE_PATH, "rb") as f:
        tidy = pickle.load(f)

    # Normalize old cached names.
    tidy["scenario"] = tidy["scenario"].replace(SCENARIO_RENAME)

    return tidy


# ===========================================================================================
# STAGE 2 — RENDER (fast): aggregation helpers
# ===========================================================================================
def _med_iqr(vals, fmt="{:.1f}"):
    v = pd.Series(vals, dtype=float).dropna()
    if v.empty:
        return "--"
    return f"{fmt.format(v.median())} ({fmt.format(v.quantile(.25))}-{fmt.format(v.quantile(.75))})"


def paired_seeds(tidy, cfg, scen):
    """Seeds where EVERY present planner succeeded (intersection)."""
    sub = tidy[(tidy.config == cfg) & (tidy.scenario == scen)]
    planners = [p for p in PLANNER_ORDER if p in sub.planner.unique()]
    if not planners:
        return set()
    sets = [set(sub[(sub.planner == p) & sub.success].seed) for p in planners]
    return set.intersection(*sets) if sets else set()


def success_rate(tidy, cfg, scen, planner):
    sub = tidy[(tidy.config == cfg) & (tidy.scenario == scen) & (tidy.planner == planner)]
    n = len(sub)
    return (100.0 * sub.success.sum() / n) if n else np.nan, int(sub.success.sum()), n


# ===========================================================================================
# STAGE 2 — TABLES (terminal + LaTeX).  Rich timing/graph stats; NOTHING that is plotted
# (executed length / time / turn / effort live in the figures, not here).
# ===========================================================================================
# (column key, header, latex header)
TABLE_COLS = [
    ("succ",         "Succ%",      r"Succ\%"),
    ("t_init",       "T_init(ms)", r"$T_{\mathrm{init}}$"),
    ("t_upd",        "T_upd(ms)",  r"$T_{\mathrm{upd}}$"),
    ("t_pln",        "T_pln(ms)",  r"$T_{\mathrm{pln}}$"),
    ("t_repair",     "T_repair(ms)", r"$T_{\mathrm{rep}}$"),
    ("t_repair_p99", "Rep_p99",    r"p99"),
    ("t_repair_max", "Rep_max",    r"max"),
    ("t_add",        "T_add(ms)",  r"$T_{\mathrm{add}}$"),
    ("obs_rep_pln", "ChecksRepPln", r"Checks$_{\mathrm{rep\text{-}plan}}$"),
    ("obs_steady",  "ChecksSteady", r"Checks$_{\mathrm{steady}}$"),
    ("obs_per_repair", "Checks/Repair", r"Checks/Rep."),
    ("start_v",      "Start|V|",   r"$|V|_0$"),
    ("end_v",        "End|V|",     r"$|V|_n$"),
    ("start_r",      "Start r_n",  r"$r_0$"),
    ("end_r",        "End r_n",    r"$r_n$"),
    ("deg",          "Deg(O/I)",   r"Deg"),
]


def aggregate_cell(tidy, cfg, scen, planner, seeds):
    """Rich per-cell aggregation over paired `seeds`, matching analyze_anytime2 formatting."""
    sub = tidy[(tidy.config == cfg) & (tidy.scenario == scen) & (tidy.planner == planner)]
    paired = sub[sub.seed.isin(seeds)]
    succ, _, _ = success_rate(tidy, cfg, scen, planner)
    is_rrtx = (planner.upper() == "RRTX")
    g = lambda c: (paired[c].dropna() if c in paired else pd.Series(dtype=float))
    one  = lambda c, f="{:.1f}": (f.format(g(c).median()) if not g(c).empty else "nan")
    mean = lambda c, f="{:.0f}": (f.format(g(c).mean())   if not g(c).empty else "nan")
    return {
        "succ":          f"{succ:.0f}%",
        "t_init":        one("t_init"),
        "t_upd":         one("t_upd"),
        "t_pln":         ("nan" if is_rrtx else one("t_pln")),   # RRTX: next plan is not repair
        "t_repair":      _med_iqr(g("t_repair")),
        "t_repair_p99":  one("t_repair_p99"),
        "t_repair_max":  (f"{g('t_repair_max').max():.1f}" if not g("t_repair_max").empty else "nan"),
        "t_add":         _med_iqr(g("t_add")),
        "obs_rep_pln":   ("nan" if is_rrtx else mean("obs_rep_pln")),
        "obs_steady":    _med_iqr(g("obs_steady"), "{:.0f}"),
        "obs_per_repair":_med_iqr(g("obs_per_repair"), "{:.0f}"),
        "start_v":       mean("start_v"),
        "end_v":         mean("end_v"),
        "start_r":       mean("start_r", "{:.2f}"),
        "end_r":         mean("end_r", "{:.2f}"),
        "deg":           (f"{g('deg_out').mean():.1f}/{g('deg_in').mean():.1f}"
                          if not g("deg_out").empty and not g("deg_in").empty else "nan"),
    }


def print_terminal_tables(tidy):
    for scen in SCENARIO_ORDER:
        if scen not in tidy.scenario.unique():
            continue
        rows = []
        for cfg in CONFIG_ORDER:
            seeds = paired_seeds(tidy, cfg, scen)
            for i, planner in enumerate(PLANNER_ORDER):
                if tidy[(tidy.config == cfg) & (tidy.scenario == scen) & (tidy.planner == planner)].empty:
                    continue
                a = aggregate_cell(tidy, cfg, scen, planner, seeds)
                ctxt = CONFIG_TXT_LABEL.get(cfg, cfg)
                row = {"Config": f"{ctxt} (N_pair={len(seeds)})" if i == 0 else "", "Planner": planner}
                row.update({hdr: a[key] for key, hdr, _ in TABLE_COLS})
                rows.append(row)
        print("\n" + "=" * 200)
        print(f" {scen}   (timing/graph over paired common-success seeds; Succ% over all 100)")
        print("=" * 200)
        print(pd.DataFrame(rows).to_string(index=False, justify="center"))
        print("-" * 200)


def write_latex_tables(tidy):
    os.makedirs(FIG_DIR, exist_ok=True)
    path = os.path.join(FIG_DIR, "tables.tex")
    latex_hdr = [lh for _, _, lh in TABLE_COLS]
    with open(path, "w") as f:
        for scen in SCENARIO_ORDER:
            if scen not in tidy.scenario.unique():
                continue
            f.write("\\begin{table*}[t]\\centering\n")
            f.write(
                f"\\caption{{{scen}: latency and graph statistics. "
                f"$T_{{\\mathrm{{rep}}}}$, $T_{{\\mathrm{{add}}}}$, and collision-check "
                f"statistics are summarized over paired common-success seeds. "
                f"Max denotes the maximum observed individual repair latency. "
                f"Success is computed over all 100 seeds.}}\n"
            )
            f.write("\\scriptsize\\setlength{\\tabcolsep}{3pt}\n")
            f.write("\\begin{tabular}{ll" + "r" * len(latex_hdr) + "}\n\\toprule\n")
            f.write("Config & Planner & " + " & ".join(latex_hdr) + " \\\\\n\\midrule\n")
            for cfg in CONFIG_ORDER:
                seeds = paired_seeds(tidy, cfg, scen)
                for i, planner in enumerate(PLANNER_ORDER):
                    if tidy[(tidy.config == cfg) & (tidy.scenario == scen) & (tidy.planner == planner)].empty:
                        continue
                    a = aggregate_cell(tidy, cfg, scen, planner, seeds)
                    tag = (CONFIG_FIG_LABEL.get(cfg, cfg) if i == 0 else "")  # mathtext renders in LaTeX
                    cells = [str(a[key]).replace("-", "--").replace("%", "\\%") for key, _, _ in TABLE_COLS]
                    f.write(f"{tag} & {PLANNER_LABEL[planner]} & " + " & ".join(cells) + " \\\\\n")
                f.write("\\midrule\n")
            f.write("\\bottomrule\n\\end{tabular}\n\\end{table*}\n\n")
    print(f"[render] LaTeX tables -> {path}")


# ===========================================================================================
# STAGE 2 — FIGURES (path quality only; success/repair live in the tables)
# ===========================================================================================
def _grouped_box(ax, tidy, scen, metric, ylabel):
    """Grouped box plot: x = config, hue = planner, over paired seeds. y = `metric`."""
    nP, nC = len(PLANNER_ORDER), len(CONFIG_ORDER)
    width = 0.8 / nP
    for jp, planner in enumerate(PLANNER_ORDER):
        positions, data = [], []
        for ic, cfg in enumerate(CONFIG_ORDER):
            seeds = paired_seeds(tidy, cfg, scen)
            sub = tidy[(tidy.config == cfg) & (tidy.scenario == scen)
                       & (tidy.planner == planner) & tidy.seed.isin(seeds)]
            vals = sub[metric].dropna().values if metric in sub else np.array([])
            positions.append(ic + (jp - (nP - 1) / 2.0) * width)
            data.append(vals if len(vals) else [np.nan])
        bp = ax.boxplot(data, positions=positions, widths=width * 0.9, patch_artist=True,
                        showfliers=False, medianprops=dict(color="black", lw=1.2),
                        whiskerprops=dict(color=PLANNER_COLORS[planner]),
                        capprops=dict(color=PLANNER_COLORS[planner]))
        for box in bp["boxes"]:
            box.set(facecolor=PLANNER_COLORS[planner], alpha=0.65, edgecolor="black", lw=0.6)
    ax.set_xticks(range(nC))
    ax.set_xticklabels(
        _cfg_fig_labels(),
        rotation=0,
        ha="center",
        multialignment="center",
        fontsize=8
    )
    ax.set_title(scen, fontsize=11)
    ax.set_ylabel(ylabel, fontsize=9)
    ax.grid(axis="y", ls=":", alpha=0.5)


def _legend(fig):
    handles = [
        plt.Line2D(
            [0], [0],
            marker="s",
            ls="",
            markersize=8,
            markerfacecolor=PLANNER_COLORS[p],
            markeredgecolor="black",
            label=PLANNER_LABEL[p]
        )
        for p in PLANNER_ORDER
    ]

    fig.legend(
        handles=handles,
        loc="lower center",
        ncol=len(PLANNER_ORDER),
        frameon=False,
        fontsize=9,
        bbox_to_anchor=(0.5, -0.01)
    )


def fig_path_length(tidy):
    scs = [s for s in DYNAMIC_SCENARIOS if s in tidy.scenario.unique()]
    if not scs:
        return
    fig, axes = plt.subplots(1, len(scs), figsize=(3.6 * len(scs), 3.4), squeeze=False)
    for ax, scen in zip(axes[0], scs):
        _grouped_box(ax, tidy, scen, "exec_length", "Executed path length (m)")
    _legend(fig)
    fig.tight_layout(rect=[0, 0.07, 1, 1])
    out = os.path.join(FIG_DIR, "fig_path_length.pdf")
    fig.savefig(out, bbox_inches="tight"); plt.close(fig)
    print(f"[render] figure -> {out}")


def fig_domain_quality(tidy):
    items = [(s, *DOMAIN_METRIC[s]) for s in DOMAIN_METRIC if s in tidy.scenario.unique()]
    if not items:
        return
    fig, axes = plt.subplots(1, len(items), figsize=(3.6 * len(items), 3.4), squeeze=False)
    for ax, (scen, metric, ylabel) in zip(axes[0], items):
        _grouped_box(ax, tidy, scen, metric, ylabel)
    _legend(fig)
    fig.tight_layout(rect=[0, 0.07, 1, 1])
    out = os.path.join(FIG_DIR, "fig_domain_quality.pdf")
    fig.savefig(out, bbox_inches="tight"); plt.close(fig)
    print(f"[render] figure -> {out}")


def _repair_panel(ax, tidy, scen):
    """Repair-latency box per (config, planner) on log-y, with the observed maximum marker (x)."""
    nP, nC = len(PLANNER_ORDER), len(CONFIG_ORDER)
    width = 0.8 / nP
    floor = 0.05  # ms: log-y floor so sub-millisecond (LLPT) boxes still render
    for jp, planner in enumerate(PLANNER_ORDER):
        for ic, cfg in enumerate(CONFIG_ORDER):
            seeds = paired_seeds(tidy, cfg, scen)
            sub = tidy[(tidy.config == cfg) & (tidy.scenario == scen)
                       & (tidy.planner == planner) & tidy.seed.isin(seeds)]
            if sub.empty:
                continue
            pos = ic + (jp - (nP - 1) / 2.0) * width
            vals = sub["t_repair"].dropna().clip(lower=floor).values
            if len(vals):
                bp = ax.boxplot([vals], positions=[pos], widths=width * 0.9, patch_artist=True,
                                showfliers=False, medianprops=dict(color="black", lw=1.2),
                                whiskerprops=dict(color=PLANNER_COLORS[planner]),
                                capprops=dict(color=PLANNER_COLORS[planner]))
                bp["boxes"][0].set(facecolor=PLANNER_COLORS[planner], alpha=0.65,
                                   edgecolor="black", lw=0.6)
            # observed maximum marker (maximum individual repair observed)
            mx = sub["t_repair_max"].dropna()
            if not mx.empty:
                ax.plot([pos], [max(mx.max(), floor)], marker="x", ms=6, mec=PLANNER_COLORS[planner], mew=1.6, ls="")
    ax.set_yscale("log")
    ax.set_xticks(range(nC))
    ax.set_xticklabels(
        _cfg_fig_labels(),
        rotation=0,
        ha="center",
        multialignment="center",
        fontsize=8
    )
    ax.set_title(scen, fontsize=11)
    ax.set_ylabel("Repair latency (ms, log scale)", fontsize=9)
    ax.grid(axis="y", which="both", ls=":", alpha=0.4)


def fig_repair_time(tidy):
    # NOTE: the 'x' markers denote the maximum observed individual repair latency.
    scs = [s for s in SCENARIO_ORDER if s in tidy.scenario.unique()]  # includes R2 (repair-only)
    if not scs:
        return
    fig, axes = plt.subplots(2, 2, figsize=(7.6, 7.2))
    axflat = axes.flatten()
    for i, ax in enumerate(axflat):
        if i < len(scs):
            _repair_panel(ax, tidy, scs[i])
        else:
            ax.set_visible(False)
    _legend(fig)
    fig.tight_layout(rect=[0, 0.05, 1, 1])
    out = os.path.join(FIG_DIR, "fig_repair_time.pdf")
    fig.savefig(out, bbox_inches="tight"); plt.close(fig)
    print(f"[render] figure -> {out}")


def fig_success(tidy):
    """Success-rate bars (over all 100 seeds) per config x planner; dynamic spaces only (no R2)."""
    scs = [s for s in DYNAMIC_SCENARIOS if s in tidy.scenario.unique()]
    if not scs:
        return
    nP, nC = len(PLANNER_ORDER), len(CONFIG_ORDER)
    width = 0.8 / nP
    fig, axes = plt.subplots(1, len(scs), figsize=(3.6 * len(scs), 3.4), squeeze=False)
    for ax, scen in zip(axes[0], scs):
        for jp, planner in enumerate(PLANNER_ORDER):
            xs, ys = [], []
            for ic, cfg in enumerate(CONFIG_ORDER):
                succ, _, _ = success_rate(tidy, cfg, scen, planner)
                xs.append(ic + (jp - (nP - 1) / 2.0) * width)
                ys.append(succ if np.isfinite(succ) else 0.0)
            ax.bar(xs, ys, width=width * 0.9, color=PLANNER_COLORS[planner],
                   edgecolor="black", lw=0.5, alpha=0.85)
            for x, y in zip(xs, ys):
                ax.annotate(f"{y:.0f}", (x, y), textcoords="offset points",
                            xytext=(0, 2), ha="center", fontsize=6)
        ax.set_xticks(range(nC))
        ax.set_xticklabels(
            _cfg_fig_labels(),
            rotation=0,
            ha="center",
            multialignment="center",
            fontsize=8
        )
        ax.set_title(scen, fontsize=11)
        ax.set_ylabel("Success rate (%)", fontsize=9)
        ax.set_ylim(0, 108)
        ax.grid(axis="y", ls=":", alpha=0.5)
    _legend(fig)
    fig.tight_layout(rect=[0, 0.07, 1, 1])
    out = os.path.join(FIG_DIR, "fig_success.pdf")
    fig.savefig(out, bbox_inches="tight"); plt.close(fig)
    print(f"[render] figure -> {out}")


def render():
    tidy = load_cache()
    os.makedirs(FIG_DIR, exist_ok=True)
    print_terminal_tables(tidy)
    write_latex_tables(tidy)
    fig_success(tidy)
    fig_repair_time(tidy)
    fig_path_length(tidy)
    fig_domain_quality(tidy)
    print("\n[render] done.")


# ===========================================================================================
def main():
    cmd = sys.argv[1] if len(sys.argv) > 1 else "auto"
    if cmd == "build":
        build_cache()
    elif cmd == "render":
        render()
    elif cmd == "all":
        build_cache(); render()
    else:  # auto: build if cache missing, then render
        render()


if __name__ == "__main__":
    main()
