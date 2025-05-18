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

# # raw‑flattened summary
'''
The raw‑flattened summary tells you “among all replanning calls, here’s the overall distribution.”
'''
# def summarize_planners(data, n_boot=1000, ci=95):
#     rows = []
#     for samples, planners in sorted(data.items()):
#         for planner, dfs in planners.items():
#             durations = np.concatenate([d['duration_ms'].values for d in dfs])
#             med = np.median(durations)
#             lo, hi = bootstrap_median_ci(durations, n_boot, ci)
#             rows.append({
#                 'samples': samples,
#                 'planner': planner,
#                 'count': len(durations),
#                 'mean_ms': durations.mean(),
#                 'std_ms': durations.std(),
#                 'median_ms': med,
#                 'ci_lo': lo,
#                 'ci_hi': hi,
#                 'min_ms': durations.min(),
#                 'max_ms': durations.max()
#             })
#     # return pd.DataFrame(rows).sort_values(['samples','planner']).reset_index(drop=True)
#     return pd.DataFrame(rows)


# grid‑interpolated summary
'''
The grid‑interpolated summary tells you “at each aligned time stamp, here’s the performance trajectory,” then compresses that to overall stats.

Because RRTx runs more slowly than FMTx, in a fixed 30 s window it naturally completes fewer replanning loops. If you simply flatten all durations you end up with:

FMTx: ~27000 data points at 2.5k samples

RRTx: ~27000 as well at 2.5k (very similar speed there)

FMTx: ~6400 at 20k samples

RRTx: ~5000 at 20k samples

That difference in “count” is purely an artifact of how many loops each planner can squeeze into your time budget. It biases any raw‐flattened statistics toward the faster planner, because faster ⇒ more loops ⇒ more samples in your distribution.

Why the grid‐based approach fixes that
When you switch to a fixed time grid (rather than “one point per loop”), you:

Remove the loop‐count bias. Both planners get exactly one data point per grid timestamp, regardless of how many loops they actually ran.

Align “apples to apples.” You compare their performance at the same moments in time (and thus the same obstacle configurations), rather than comparing all FMTx’s loops to all RRTx’s loops.

'''
def summarize_planners(data, grid=None):
    """
    Apples-to-apples summary: interpolate each trial onto common time grid,
    then summarize the median trajectory across trials.
    """
    # define grid if not provided
    if grid is None:
        # grid = np.linspace(0, 30, 301)  # 0 to 30s at 0.1s steps

        dt = 1/30  # ≈0.03333 s
        grid = np.arange(0, 30+dt/2, dt)  # ~901 points


    rows = []
    for samples, planners in sorted(data.items()):
        for planner, dfs in planners.items():
            # interpolate each trial onto the common grid
            interp_curves = []
            for df in dfs:
                t = df['elapsed_s'].values
                d = df['duration_ms'].values
                interp = np.interp(grid, t, d, left=np.nan, right=np.nan)
                interp_curves.append(interp)
            stacked = np.vstack(interp_curves)  # shape (n_trials, n_grid)
            # median curve across trials
            median_curve = np.nanmedian(stacked, axis=0)
            # summarize statistics of that median_curve
            rows.append({
                'samples': samples,
                'planner': planner,
                'count': len(grid),
                'mean_ms': np.nanmean(median_curve),
                'std_ms': np.nanstd(median_curve),
                'median_ms': np.nanmedian(median_curve),
                'min_ms': np.nanmin(median_curve),
                'max_ms': np.nanmax(median_curve)
            })
    return pd.DataFrame(rows)


def plot_ecdf(data, samples_list):
    plt.figure(figsize=(8,5))
    for samples in samples_list:
        for planner, dfs in data[samples].items():
            all_durs = np.concatenate([d['duration_ms'] for d in dfs])
            s = np.sort(all_durs)
            ecdf = np.arange(1, len(s)+1)/len(s)
            plt.step(s, ecdf, where='post', label=f"{planner} ({samples})")
    plt.xlabel("Replanning Duration (ms)")
    plt.ylabel("Cumulative Probability (ECDF)")
    plt.title("ECDF of Replanning Durations Across Trials")
    plt.grid('--', lw=0.5)
    plt.legend(fontsize='small')
    plt.tight_layout()
    plt.show()

def plot_boxplot(data, samples_list):
    recs = []
    for samples in samples_list:
        for planner, dfs in data[samples].items():
            for med in [np.median(d['duration_ms']) for d in dfs]:
                recs.append({'samples': samples, 'planner': planner, 'median (ms)': med})
    df = pd.DataFrame(recs)
    
    plt.figure(figsize=(8, 5))
    ax = plt.subplot()
    
    # Generate boxplot - use this format for version compatibility
    df.boxplot(
        column='median (ms)',
        by=['samples', 'planner'],
        ax=ax
    )
    
    # Remove all auto-generated titles
    plt.suptitle("")  # Removes "Boxplot grouped by samples,planner"
    ax.set_title("")   # Removes the column name title
    
    # Custom labels
    ax.set_xlabel("Sample size, Planner")
    ax.set_ylabel("Median Replanning Time per Trial (ms)")
    plt.xticks(rotation=45)
    plt.grid('--', alpha=0.5)
    plt.tight_layout()
    
    os.makedirs("./figures", exist_ok=True)
    plt.savefig("./figures/boxplot.pdf", bbox_inches='tight', format='pdf')
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


    # Log-Log subplot
    ax[0].set_xlabel("Sample Size, $n$")
    ax[0].set_ylabel(r"Median Replanning Time, $T(n)$ (ms)")
    ax[0].set_title("Log-Log Scaling of Replanning Time vs. Sample Size")
    ax[0].grid(True, which='both', linestyle='--', linewidth=0.5)
    ax[0].legend(fontsize='small')



    # Normalized subplot
    ax[1].set_xlabel("Sample Size, $n$")
    ax[1].set_ylabel(r"$\frac{T(n)}{n \log n}$ (ms/node)")
    ax[1].set_title("Normalized Time Complexity")
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
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    grid = np.linspace(0, 30, 300)
    colors = {'FMTx': '#3060D0', 'RRTx': '#D03030'}  # Median lines
    bg_colors = {'FMTx': '#DCE6F1', 'RRTx': '#F4CCCC'}  # Envelopes

    for ax, samples in zip(axes.flat, samples_list):
        if samples not in data:
            ax.set_visible(False)
            continue
        for planner in ['FMTx', 'RRTx']:
            if planner not in data[samples]:
                continue
            curves = []
            for df in data[samples][planner]:
                t = df['elapsed_s'].values
                y = df['duration_ms'].values
                curves.append(np.interp(grid, t, y, np.nan, np.nan))
            stacked = np.vstack(curves)
            lo = np.nanpercentile(stacked, 5, axis=0)
            hi = np.nanpercentile(stacked, 95, axis=0)
            med = np.nanmedian(stacked, axis=0)
            label_fill = f"{planner} (5th–95th percentile)" if ax == axes.flat[0] else None
            label_line = f"{planner} median" if ax == axes.flat[0] else None

            ax.fill_between(
                grid, lo, hi,
                facecolor=bg_colors[planner],
                alpha=0.5,
                label=label_fill
            )
            ax.plot(
                grid, med,
                color=colors[planner],
                lw=2,
                label=label_line
            )
                    
        ax.set_title(f"{samples} samples")
        ax.set_xlabel("Elapsed time (s)")
        ax.set_ylabel("Replanning Duration (ms)")
        
        # ---- Enhanced Grid Customization ----
        ax.grid(True, which='both', linestyle='--', linewidth=0.5, alpha=0.7)  # Major grid
        ax.minorticks_on()  # Enable minor ticks
        ax.grid(True, which='minor', linestyle=':', linewidth=0.5, alpha=0.3)  # Minor grid
        
        # Set minor tick frequency (optional)
        ax.xaxis.set_minor_locator(AutoMinorLocator(4))  # 4 minor ticks per major division
        ax.yaxis.set_minor_locator(AutoMinorLocator(4))
        
        #ax.legend(fontsize='small', framealpha=1)
    # Add shared legend **below** subplots
    fig.legend(
        loc='lower center',          # below the plots
        bbox_to_anchor=(0.5, -0.05), # centered beneath
        ncol=2,                      # one entry per planner
        fontsize='medium'
    )
    plt.tight_layout()
    plt.savefig("./figures/replanning.pdf", bbox_inches='tight', format='pdf')
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
    # print(df_sum.pivot(index='samples', columns='planner',
    #                    values=['median_ms','ci_lo','ci_hi']))
    pd.set_option('display.float_format', '{:,.2f}'.format)
    print("\n=== Enriched Summary per Planner & Sample Size ===")
    print(df_sum.sort_values(['samples','planner'])
                    .reset_index(drop=True)
                    .to_string(index=False))
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
