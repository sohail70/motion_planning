#!/usr/bin/env python3
"""
paper_report_nonanytime.py  --  cached, two-stage reporting for the NON-ANYTIME
D-FMT* vs D*Lite ablation (mirror of the anytime paper_report.py).

Why two stages?  Loading + synthesizing the raw sim_*_metrics.csv is the slow part. We do it
ONCE into a tidy per-seed cache (pickle); after that, tables and figures render instantly, so
you can tweak axes/labels/figures without ever re-reading the CSVs.

    python paper_report_nonanytime.py build      # (re)build the cache from the raw CSV dirs  [SLOW]
    python paper_report_nonanytime.py render     # tables (terminal + LaTeX) + figures        [FAST]
    python paper_report_nonanytime.py all        # build then render
    python paper_report_nonanytime.py            # render if cache exists, else build then render

It reuses the tuned loaders / repair semantics from analyze_non_anytime2.py (imported, never
re-implemented), so the numbers match your existing per-sim tables exactly.

Non-anytime specifics vs. the anytime report:
  * Two planners only: FMTX == D-FMT*  (blue, hero) and DLITE == D*Lite (red).
  * Repair latency = update_ms  (plan() runs inside updateObstacles(); no update/plan split).
  * Graph is a one-shot batch of N samples (no per-slice sample addition), so the config axis
    is N (graph size) x |O| (obstacles), and the table carries Setup / Samples / Isolated.
"""

import os
import sys
import pickle
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")  # headless: just write files
import matplotlib.pyplot as plt

import analyze_non_anytime2 as aa  # tuned loaders + repair semantics live here

# ===========================================================================================
# CONFIG  --  the three sim output directories (each holds all 4 spaces for one (N, |O|) setting).
# ===========================================================================================
CONFIGS = {
    "N=1000, 10 obs": "1000_10obs",   # sim 1
    "N=1000, 20 obs": "1000_20obs",   # sim 2
    "N=3000, 20 obs": "3000_20obs",   # sim 3
}
CONFIG_ORDER = list(CONFIGS.keys())
# Display labels. Cache keys above stay ASCII (so no rebuild is needed); figures use mathtext.
CONFIG_FIG_LABEL = {
    "N=1000, 10 obs": r"$N{=}1000,\ |\mathcal{O}|{=}10$",
    "N=1000, 20 obs": r"$N{=}1000,\ |\mathcal{O}|{=}20$",
    "N=3000, 20 obs": r"$N{=}3000,\ |\mathcal{O}|{=}20$",
}
CONFIG_TXT_LABEL = {  # for the terminal/LaTeX tables
    "N=1000, 10 obs": "N=1000,|O|=10",
    "N=1000, 20 obs": "N=1000,|O|=20",
    "N=3000, 20 obs": "N=3000,|O|=20",
}
def _cfg_fig_labels():
    return [CONFIG_FIG_LABEL.get(c, c) for c in CONFIG_ORDER]

CACHE_PATH = "paper_cache_nonanytime.pkl"
FIG_DIR    = "paper_out_nonanytime"

# Raw scenario name (from filenames)  ->  paper label
SCENARIO_RENAME = {"R2": "R2", "R2T": "Holonomic", "Dubins": "Dubins", "Thruster": "Thruster"}
SCENARIO_ORDER  = ["R2", "Holonomic", "Dubins", "Thruster"]
# Spaces where the robot actually moves (success + executed quality are meaningful)
DYNAMIC_SCENARIOS = ["Holonomic", "Dubins", "Thruster"]

PLANNER_ORDER  = ["FMTX", "DLITE"]
PLANNER_LABEL  = {"FMTX": r"D-FMT$^{*}$", "DLITE": r"D$^{*}$Lite"}
PLANNER_COLORS = {"FMTX": "#1f77b4", "DLITE": "#d62728"}  # blue (hero) / red

# Per-scenario "domain" quality metric (path length is shown for all dynamic spaces).
# effort = integral of |acceleration| over time  ->  m/s^2 * s = m/s.
DOMAIN_METRIC = {"Dubins":   ("exec_turn",   "Turn (rad)"),
                 "Thruster": ("exec_effort", r"Control effort $\int|a|\,dt$  (m/s)")}


# ===========================================================================================
# STAGE 1 — BUILD CACHE (slow)
# ===========================================================================================
def extract_seed_metrics(df):
    """One row of per-seed metrics, mirroring analyze_non_anytime2.analyze_group_statistics'
    inner loop.  Repair latency = update_ms (plan() runs inside updateObstacles())."""
    m = {}
    rt = aa.extract_runtime_events(df)

    # --- one-shot graph build stats (initial_plan event) ---
    if "event_type" in df.columns:
        di = df[df["event_type"] == "initial_plan"]
        if not di.empty:
            if "plan_ms" in di.columns:
                v = di["plan_ms"].dropna()
                if not v.empty: m["t_init"] = v.median()
            if "setup_ms" in di.columns:
                v = di["setup_ms"].dropna()
                if not v.empty: m["setup"] = v.median()
            if "isolated_nodes" in di.columns:
                v = di["isolated_nodes"].dropna()
                if not v.empty: m["isolated"] = v.mean()
            if "tree_size" in di.columns:
                v = di["tree_size"].dropna()
                if not v.empty: m["samples"] = v.mean()
            rc = ("neighborhood_radius" if "neighborhood_radius" in di.columns
                  else ("radius" if "radius" in di.columns else None))
            if rc:
                v = di[rc].dropna()
                if not v.empty: m["r_n"] = v.mean()

            # Combined initial time = setup + initial planning time
            if "setup" in m and "t_init" in m:
                m["initial_time"] = m["setup"] + m["t_init"]


    # --- repair latency (update event) ---
    run_update = aa.get_update_rows(rt)
    if not run_update.empty:
        s = run_update["update_ms"].dropna()
        if not s.empty:
            m["t_repair"]     = s.median()
            m["t_repair_p99"] = s.quantile(0.99)
            m["t_repair_max"] = s.max()
        oc = run_update["obstacle_checks"].dropna()
        if not oc.empty:
            m["obs_per_upd"] = oc.mean()

    # --- executed solution quality at the terminal goal row ---
    term = aa.get_success_terminal_row(df)
    if not term.empty:
        r = term.iloc[-1]
        for col in ("exec_length", "exec_time", "exec_turn", "exec_effort"):
            if col in term.columns:
                v = pd.to_numeric(r.get(col), errors="coerce")
                if pd.notna(v) and np.isfinite(v) and v > 0:
                    m[col] = float(v)

    # --- graph degrees (terminal rows) ---
    # --- final graph snapshot ---
    # R2 ends with "sim_end".
    # Successful moving-robot runs end with "goal_reached".
    if "event_type" in df.columns:
        event = df["event_type"].astype(str).str.strip().str.lower()

        goal_rows = df[event.eq("goal_reached")]
        sim_end_rows = df[event.eq("sim_end")]

        # Prefer goal_reached for R2T/Dubins/Thruster; otherwise use
        # sim_end for the geometric R2 graph-repair benchmark.
        if not goal_rows.empty:
            rterm = goal_rows
        elif not sim_end_rows.empty:
            rterm = sim_end_rows
        else:
            rterm = df.iloc[0:0]
    else:
        rterm = df.iloc[0:0]

    if not rterm.empty:
        # Use the final terminal snapshot, not a mean over event rows.
        if "row_id" in rterm.columns:
            row_ids = pd.to_numeric(rterm["row_id"], errors="coerce")
            if row_ids.notna().any():
                rterm = rterm.loc[[row_ids.idxmax()]]
            else:
                rterm = rterm.tail(1)
        else:
            rterm = rterm.tail(1)

        for source, target in (
            ("avg_deg_in", "deg_in"),
            ("avg_deg_out", "deg_out"),
        ):
            if source in rterm.columns:
                value = pd.to_numeric(
                    rterm[source].iloc[-1], errors="coerce"
                )
                if pd.notna(value) and np.isfinite(value):
                    m[target] = float(value)


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
                    rec.update(extract_seed_metrics(df))
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
        return pickle.load(f)


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
# STAGE 2 — TABLES (terminal + LaTeX).  Timing/graph stats; NOTHING that is plotted
# (executed length / turn / effort live in the figures, not here).
# ===========================================================================================
# (column key, terminal header, latex header)
TABLE_COLS = [
    ("succ",         "Succ%",        r"Succ\%"),
    # ("setup",        "Setup(ms)",    r"$T_{\mathrm{setup}}$"),
    # ("t_init",       "T_init(ms)",   r"$T_{\mathrm{init}}$"),
    ("initial_time", "Initial(ms)",    r"$T_{\mathrm{initial}}$"),
    ("t_repair",     "T_repair(ms)", r"$T_{\mathrm{rep}}$"),
    ("t_repair_p99", "Rep_p99",      r"p99"),
    ("t_repair_max", "Rep_max",      r"max"),
    ("obs_per_upd",  "Obs/Upd",      r"Obs/Upd"),
    ("samples",      "Samples",      r"$N$"),
    ("isolated",     "Isolated",     r"Iso"),
    ("r_n",          "r_n",          r"$r_n$"),
    ("deg",          "Deg(I/O)",     r"Deg"),
]


def aggregate_cell(tidy, cfg, scen, planner, seeds):
    """Per-cell aggregation over paired `seeds`, matching analyze_non_anytime2 formatting."""
    sub = tidy[(tidy.config == cfg) & (tidy.scenario == scen) & (tidy.planner == planner)]
    paired = sub[sub.seed.isin(seeds)]
    succ, _, _ = success_rate(tidy, cfg, scen, planner)
    g = lambda c: (paired[c].dropna() if c in paired else pd.Series(dtype=float))
    one  = lambda c, f="{:.1f}": (f.format(g(c).median()) if not g(c).empty else "nan")
    mean = lambda c, f="{:.0f}": (f.format(g(c).mean())   if not g(c).empty else "nan")
    return {
        "succ":          f"{succ:.0f}%",
        # "setup":         one("setup"),
        # "t_init":        one("t_init"),
        "initial_time":  one("initial_time"),
        "t_repair":      _med_iqr(g("t_repair")),
        "t_repair_p99":  one("t_repair_p99"),                                  # median of per-seed p99
        "t_repair_max":  (f"{g('t_repair_max').max():.1f}" if not g("t_repair_max").empty else "nan"),
        "obs_per_upd":   _med_iqr(g("obs_per_upd"), "{:.0f}"),
        "samples":       mean("samples"),
        "isolated":      mean("isolated"),
        "r_n":           mean("r_n", "{:.2f}"),
        "deg":           (f"{g('deg_in').mean():.1f}/{g('deg_out').mean():.1f}"
                          if not g("deg_in").empty and not g("deg_out").empty else "nan"),
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
                row = {"Config": f"{ctxt} (n={len(seeds)})" if i == 0 else "", "Planner": planner}
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
            f.write(f"\\caption{{{scen} (non-anytime): latency and graph statistics (median; IQR for "
                    f"$T_{{\\mathrm{{rep}}}}$/Obs over paired common-success seeds; max is the "
                    f"worst single repair). Success is over all 100 seeds. Executed-path quality "
                    f"is in Fig.~X.}}\n")
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
# STAGE 2 — FIGURES
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
    ax.set_xticklabels(_cfg_fig_labels(), rotation=18, ha="right", fontsize=8)
    ax.set_title(scen, fontsize=11)
    ax.set_ylabel(ylabel, fontsize=9)
    ax.grid(axis="y", ls=":", alpha=0.5)


def _legend(fig):
    handles = [plt.Line2D([0], [0], marker="s", ls="", markersize=10,
                          markerfacecolor=PLANNER_COLORS[p], markeredgecolor="black",
                          label=PLANNER_LABEL[p]) for p in PLANNER_ORDER]
    fig.legend(handles=handles, loc="lower center", ncol=len(PLANNER_ORDER),
               frameon=False, fontsize=11, bbox_to_anchor=(0.5, -0.02))


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
    """Repair-latency box per (config, planner) on log-y, with the worst-case `max` marker (x)."""
    nP, nC = len(PLANNER_ORDER), len(CONFIG_ORDER)
    width = 0.8 / nP
    floor = 0.05  # ms: log-y floor so tiny boxes still render
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
            # worst-case marker, aggregated exactly like the table (max = max-of-per-seed-max)
            mx = sub["t_repair_max"].dropna()
            if not mx.empty:
                ax.plot([pos], [max(mx.max(), floor)], marker="x", ms=6,
                        mec=PLANNER_COLORS[planner], mew=1.6, ls="")
    ax.set_yscale("log")
    ax.set_xticks(range(nC))
    ax.set_xticklabels(_cfg_fig_labels(), rotation=18, ha="right", fontsize=8)
    ax.set_title(scen, fontsize=11)
    ax.set_ylabel("Repair latency (ms, log scale)", fontsize=9)
    ax.grid(axis="y", which="both", ls=":", alpha=0.4)


def fig_repair_time(tidy):
    # NOTE: the 'x' markers denote per-config worst-case (max) repair latency -- explain in caption.
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
        ax.set_xticklabels(_cfg_fig_labels(), rotation=18, ha="right", fontsize=8)
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
