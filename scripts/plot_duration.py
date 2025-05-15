import glob
import os
import re
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import AutoMinorLocator
from scipy.stats import linregress

def get_planner_name(filename):
    base = os.path.basename(filename)
    if 'fmtx'  in base.lower(): planner = 'FMTx'
    elif 'rrtx' in base.lower(): planner = 'RRTx'
    else: planner = os.path.splitext(base)[0]
    m = re.search(r"(\d+)samples_", base.lower())
    return planner, int(m.group(1)) if m else 0

def load_grouped_data(files):
    data = {}
    for f in files:
        planner, samples = get_planner_name(f)
        try:
            df = pd.read_csv(f)
            data.setdefault(samples, {}).setdefault(planner, []).append(df)
        except Exception as e:
            print(f"Error loading {f}: {e}")
    return data

def bootstrap_median_ci(data, n_boot=1000, ci=95):
    meds = []
    for _ in range(n_boot):
        sample = np.random.choice(data, size=len(data), replace=True)
        meds.append(np.median(sample))
    lo = np.percentile(meds, (100-ci)/2)
    hi = np.percentile(meds, 100-(100-ci)/2)
    return lo, hi

def summarize_planners(data):
    rows = []
    for samples, planners in data.items():
        for planner, dfs in planners.items():
            meds = [np.median(d['duration_ms']) for d in dfs]
            med = np.median(meds)
            lo, hi = bootstrap_median_ci(meds)
            rows.append({'samples': samples, 'planner': planner,
                         'median_ms': med, 'ci_lo': lo, 'ci_hi': hi})
    return pd.DataFrame(rows).sort_values(['samples','planner']).reset_index(drop=True)

def plot_ecdf(data, samples_list):
    plt.figure(figsize=(8,5))
    for samples in samples_list:
        for planner, dfs in data[samples].items():
            all_durs = np.concatenate([d['duration_ms'] for d in dfs])
            s = np.sort(all_durs)
            ecdf = np.arange(1, len(s)+1)/len(s)
            plt.step(s, ecdf, where='post', label=f"{planner} ({samples})")
    plt.xlabel("Duration (ms)")
    plt.ylabel("ECDF")
    plt.title("ECDF of replanning durations")
    plt.grid('--', lw=0.5)
    plt.legend(fontsize='small')
    plt.tight_layout()
    plt.show()

def plot_boxplot(data, samples_list):
    recs = []
    for samples in samples_list:
        for planner, dfs in data[samples].items():
            for med in [np.median(d['duration_ms']) for d in dfs]:
                recs.append({'samples': samples, 'planner': planner, 'med_ms': med})
    df = pd.DataFrame(recs)
    plt.figure(figsize=(8,5))
    ax = plt.subplot()
    df.boxplot(column='med_ms', by=['samples','planner'], ax=ax)
    plt.suptitle("")
    plt.xlabel("Sample size, Planner")
    plt.ylabel("Trial median (ms)")
    plt.xticks(rotation=45)
    plt.grid('--', lw=0.5)
    plt.tight_layout()
    plt.show()
    
def estimate_complexity(n_values, time_values):
    """
    Estimates the time complexity from empirical data.
    Returns the best-fit complexity class and R² score.
    """
    n = np.array(n_values)
    t = np.array(time_values)
    if len(n) < 2:
        return "Insufficient data", 0.0

    # Log-log regression to estimate exponent
    try:
        log_n = np.log(n)
        log_t = np.log(t)
        slope, _, r_value, _, _ = linregress(log_n, log_t)
        k = slope
    except:
        k = 1  # Fallback if regression fails

    # Define candidate complexity functions
    candidates = {
        'O(1)': np.ones_like(n),
        'O(n)': n,
        'O(n log n)': n * np.log(n),
        'O(n²)': n**2,
        'O(n³)': n**3,
        'O(log n)': np.log(n),
        'O(2ⁿ)': 2**n,
        f'O(n^{k:.2f})': n**k,
        f'O(n^{k:.2f} log n)': n**k * np.log(n)
    }

    best_fit = None
    best_r2 = -np.inf

    for name, f_n in candidates.items():
        if np.any(f_n <= 0):
            continue  # Skip invalid candidates
        try:
            # Fit T(n) = a * f(n)
            a = np.sum(t * f_n) / np.sum(f_n**2)
            t_pred = a * f_n
            ss_res = np.sum((t - t_pred)**2)
            ss_tot = np.sum((t - np.mean(t))**2)
            #ss_tot = np.sum(t**2)

            r2 = 1 - (ss_res / ss_tot) if ss_tot != 0 else 0
            if r2 > best_r2:
                best_r2 = r2
                best_fit = name
        except:
            continue

    return best_fit, best_r2

def plot_complexity(data):
    # gather summary
    df_sum = summarize_planners(data)
    fig, ax = plt.subplots(1,2, figsize=(12,4))

    for i, planner in enumerate(['FMTx','RRTx']):
        sub = df_sum[df_sum['planner']==planner]
        n = sub['samples'].values
        t = sub['median_ms'].values
        # log-log
        ax[0].loglog(n, t, '-o', label=planner)
        # normalized
        ax[1].plot(n, t/(n * np.log(n)), '-o', label=planner)

    # ——— quantitative complexity checks ———
    print("\nFitting log–log slope:")
    for planner in ['FMTx','RRTx']:
        sub = df_sum[df_sum['planner']==planner]
        x = np.log(sub['samples'])
        y = np.log(sub['median_ms'])
        slope, intercept, r, p, se = linregress(x, y)
        print(f"  {planner}: slope={slope:.3f}, R²={r**2:.3f}")

    print("\nChecking constancy of T/(n·log n):")
    for planner in ['FMTx','RRTx']:
        sub = df_sum[df_sum['planner']==planner]
        C = sub['median_ms'] / (sub['samples'] * np.log(sub['samples']))
        print(f"  {planner}: mean C={C.mean():.3e}, CV={C.std()/C.mean():.2f}")


    ax[0].set_xlabel("Samples (n)")
    ax[0].set_ylabel("Median duration (ms)")
    ax[0].set_title("Log-Log: runtime vs. n")
    ax[0].grid(True, which='both', linestyle='--', linewidth=0.5)
    ax[0].legend(fontsize='small')

    ax[1].set_xlabel("Samples (n)")
    ax[1].set_ylabel("median_ms / (n·log n)")
    ax[1].set_title("Normalized by n·log n")
    ax[1].grid(True, linestyle='--', linewidth=0.5)
    ax[1].legend(fontsize='small')

    plt.tight_layout()
    plt.show()
    
    # New: Estimate and print complexity
    print("\nEmpirical Complexity Estimation:")
    df_sum = summarize_planners(data)
    for planner in ['FMTx', 'RRTx']:
        sub = df_sum[df_sum['planner'] == planner]
        n = sub['samples'].values
        t = sub['median_ms'].values
        best_fit, r2 = estimate_complexity(n, t)
        print(f"  {planner}: {best_fit} (R²={r2:.2f})")



def plot_trajectory_subplots(data, samples_list):
    fig, axes = plt.subplots(2, 2, figsize=(12,8))
    grid = np.linspace(0, 30, 300)
    colors = {'FMTx':'#3060D0', 'RRTx':'#D03030'}  # median lines
    bg_colors = {'FMTx':'#DCE6F1','RRTx':'#F4CCCC'}   # envelopes

    for ax, samples in zip(axes.flat, samples_list):
        if samples not in data:
            ax.set_visible(False)
            continue
        for planner in ['FMTx','RRTx']:
            if planner not in data[samples]:
                continue
            curves = []
            for df in data[samples][planner]:
                t = df['elapsed_s'].values
                y = df['duration_ms'].values
                curves.append(np.interp(grid, t, y, np.nan, np.nan))
            stacked = np.vstack(curves)
            lo = np.nanpercentile(stacked,5, axis=0)
            hi = np.nanpercentile(stacked,95,axis=0)
            med = np.nanmedian(stacked,axis=0)
            ax.fill_between(grid, lo, hi,
                            facecolor=bg_colors[planner], alpha=0.3)
            ax.plot(grid, med, color=colors[planner], lw=2,
                    label=f"{planner} median")
        ax.set_title(f"{samples} samples")
        ax.set_xlabel("Elapsed time (s)")
        ax.set_ylabel("Duration (ms)")
        ax.grid(True, linestyle='--', linewidth=0.5)
        ax.legend(fontsize='small')

    plt.tight_layout()
    plt.show()

def main():
    files = sorted(glob.glob("../build/new/test/sim_*_*.csv"))
    if not files:
        print("No data files found.")
        return
    data = load_grouped_data(files)
    samples_list = sorted(data.keys())[:4]

    # 1) Summary table
    df_sum = summarize_planners(data)
    print("\nSummary (median ± 95% CI):")
    print(df_sum.pivot(index='samples', columns='planner',
                       values=['median_ms','ci_lo','ci_hi']))

    # 2) ECDF
    plot_ecdf(data, samples_list)
    # 3) Boxplot
    plot_boxplot(data, samples_list)
    # 4) Complexity check
    plot_complexity(data)
    # 5) 2×2 trajectory + envelope
    plot_trajectory_subplots(data, samples_list)

if __name__=='__main__':
    main()
