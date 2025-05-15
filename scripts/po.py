import glob
import os
import pandas as pd
import matplotlib.pyplot as plt

# 1) Load all timed CSVs
csv_files = glob.glob('../build/*_timed.csv')  # adjust path if needed

runs = {}
for file in csv_files:
    df = pd.read_csv(file)
    run_id = os.path.splitext(os.path.basename(file))[0]
    runs[run_id] = df

# 2) Combine into one DataFrame with a run identifier
combined = pd.concat(
    [df.assign(run=run_id) for run_id, df in runs.items()],
    ignore_index=True
)

# 3) Overlay plot: all runs in gray + median across runs in color
plt.figure(figsize=(8, 4))
# plot individual runs
for df in runs.values():
    plt.plot(df['elapsed_s'], df['duration_ms'], color='gray', alpha=0.3)

# define time bins
bin_width = 0.5  # seconds
max_time = combined['elapsed_s'].max()
bins = pd.interval_range(start=0, end=max_time + bin_width, freq=bin_width, closed='left')

combined['time_bin'] = pd.cut(combined['elapsed_s'], bins)

# compute median per bin across all runs
median_by_bin = combined.groupby('time_bin')['duration_ms'].median()
times = [interval.left + bin_width/2 for interval in median_by_bin.index]
durations = median_by_bin.values

plt.plot(times, durations, color='C0', lw=2, label='Median across runs')
plt.xlabel('Elapsed time (s)')
plt.ylabel('Replanning duration (ms)')
plt.title('Replanning Duration over Time (All Runs + Median)')
plt.legend()
plt.tight_layout()
plt.show()

# 4) Bar chart: median replanning time per run
stats = combined.groupby('run')['duration_ms'].median().reset_index()
plt.figure(figsize=(8, 4))
plt.bar(stats['run'], stats['duration_ms'], color='C1')
plt.xlabel('Run ID')
plt.ylabel('Median replanning duration (ms)')
plt.title('Median Replanning Duration per Run')
plt.xticks(rotation=90)
plt.tight_layout()
plt.show()
