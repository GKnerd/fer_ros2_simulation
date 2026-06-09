"""
plot_mpc_coupled.py
====================

Plotting script for the coupled QP MPC log.

Reads mpc_full_log.csv (rename if your coupled node writes a different file
- see CSV_PATH below) and produces the figures that matter for the thesis
results chapter:

  1. Per-joint position tracking (actual vs reference vs goal).
  2. Per-joint tracking error with tolerance band.
  3. Per-joint torque with limits drawn (the constraint binding plot).
  4. Solver latency histogram + over time - the real-time benchmark.
  5. All-joints error overlay (one figure, all 7 joints).
  6. All-joints torque overlay against limits.
  7. MPC cost over time.

Run from the directory containing the CSV:
    python3 plot_mpc_coupled.py
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.ticker as mticker

# ── Config ─────────────────────────────────────────────────────────────────
CSV_PATH        = "mpc_coupled_qp_log.csv"
R2D             = 180.0 / np.pi
TRAJ_DURATION   = 5.0    # seconds — trajectory phase ends here
EFFORT_LIMITS   = {1: 87, 2: 87, 3: 87, 4: 87, 5: 12, 6: 12, 7: 12}
CYCLE_TARGET_MS = 10.0   # 100 Hz control loop target
CYCLE_STRETCH_MS = 1.0   # 1 kHz thesis target (for reference line)

df   = pd.read_csv(CSV_PATH)
time = df["time"]
tmax = float(time.iloc[-1])

# Tab10 colours so each joint has a consistent colour across all figures.
colors = plt.cm.tab10.colors


def _fmt_time_axis(ax, add_traj_line=True):
    """Set clean x-axis tick spacing and optionally mark trajectory end."""
    # Choose major tick interval so there are roughly 8-12 ticks
    raw_step = tmax / 10.0
    for candidate in [0.5, 1, 2, 5, 10, 15, 20, 30, 60]:
        if candidate >= raw_step:
            step = candidate
            break
    else:
        step = round(raw_step / 10) * 10

    ax.xaxis.set_major_locator(mticker.MultipleLocator(step))
    ax.xaxis.set_major_formatter(mticker.FuncFormatter(lambda x, _: f"{x:.0f} s"))
    ax.set_xlim(left=0, right=tmax * 1.02)
    if add_traj_line and TRAJ_DURATION <= tmax:
        ax.axvline(TRAJ_DURATION, color="gray", linestyle=":", linewidth=0.9,
                   label=f"traj end ({TRAJ_DURATION:.0f} s)")


def _fmt_deg_yaxis(ax):
    """Append ° to every y-axis tick label so degrees are unambiguous."""
    ax.yaxis.set_major_formatter(
        mticker.FuncFormatter(lambda x, _: f"{x:.3g}°")
    )


# =============================================================================
# Figure 1 - Per-joint diagnostics (5 panels each: q, error, tau, cost, v)
# =============================================================================
# This is the deep-dive figure for a single joint at a time. Use it when a
# specific joint misbehaves and you need to see everything about it.

for i in range(1, 8):
    fig, axes = plt.subplots(5, 1, figsize=(11, 16), sharex=True)
    fig.suptitle(f"Coupled QP MPC — Joint {i} Diagnostics",
                 fontsize=13, fontweight="bold")

    # Position — values already converted to degrees via * R2D
    ax = axes[0]
    ax.plot(time, df[f"q{i}"] * R2D,    label="actual",
            color="steelblue", linewidth=1.5)
    ax.plot(time, df[f"qref{i}"] * R2D, label="qref (traj)",
            color="tomato", linestyle="--", linewidth=1.2)
    ax.plot(time, df[f"qdes{i}"] * R2D, label="qdes (goal)",
            color="gray", linestyle=":", linewidth=0.9)
    ax.set_ylabel("Position [°]")
    _fmt_deg_yaxis(ax)   # adds ° to every tick so degrees are unambiguous
    ax.legend(loc="upper right")
    ax.grid(True, alpha=0.4)

    # Error with tolerance band — values in degrees
    ax = axes[1]
    ax.plot(time, df[f"error{i}"] * R2D, color="darkorange", linewidth=1.4,
            label="error = qref − q")
    ax.axhline( 0.01 * R2D, color="gray", linestyle=":", linewidth=0.8,
                label=f"±{0.01*R2D:.3f}°  (= ±0.01 rad)")
    ax.axhline(-0.01 * R2D, color="gray", linestyle=":", linewidth=0.8)
    ax.axhline(0, color="black", linewidth=0.5)
    ax.set_ylabel("Tracking error [°]")
    _fmt_deg_yaxis(ax)
    ax.legend(loc="upper right")
    ax.grid(True, alpha=0.4)

    # Torque with limits - critical plot for the coupled QP
    ax = axes[2]
    lim = EFFORT_LIMITS[i]
    ax.plot(time, df[f"tau_raw{i}"], label="tau (QP)",
            color="seagreen", linewidth=1.4)
    ax.plot(time, df[f"tau_sat{i}"], label="tau_sat (safety clamp)",
            color="darkviolet", linestyle="--", linewidth=1.0, alpha=0.7)
    ax.axhline( lim, color="red", linestyle="--", linewidth=0.8,
                label=f"±{lim} Nm limit")
    ax.axhline(-lim, color="red", linestyle="--", linewidth=0.8)
    ax.set_ylabel("Torque [Nm]")
    ax.legend(loc="upper right")
    ax.grid(True, alpha=0.4)

    # MPC cost
    ax = axes[3]
    ax.plot(time, df[f"best_cost{i}"], color="purple", linewidth=1.4,
            label="QP cost")
    ax.set_ylabel("MPC cost")
    ax.set_yscale("symlog", linthresh=1.0)
    ax.legend(loc="upper right")
    ax.grid(True, alpha=0.4)

    # Virtual acceleration — bottom panel carries the shared x-axis
    ax = axes[4]
    ax.plot(time, df[f"v{i}"], color="chocolate", linewidth=1.4,
            label="v (chosen by QP)")
    ax.axhline(0, color="black", linewidth=0.5)
    ax.set_ylabel("v [rad/s²]")
    _fmt_time_axis(ax, add_traj_line=True)
    ax.legend(loc="upper right")
    ax.grid(True, alpha=0.4)

    plt.tight_layout()


# =============================================================================
# Figure 2 - Solver latency (the real-time benchmark)
# =============================================================================
# This is the figure your thesis description explicitly asks for ("solver
# latency"). It only makes sense for the QP version - brute-force is too
# slow for real-time anyway.

if "solve_time_ms" in df.columns:
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    fig.suptitle("Coupled QP - Solver Latency", fontsize=13, fontweight="bold")

    solve_ms = df["solve_time_ms"]

    # Histogram
    ax = axes[0]
    ax.hist(solve_ms, bins=50, color="steelblue", edgecolor="black", alpha=0.75)
    ax.axvline(CYCLE_TARGET_MS, color="orange", linestyle="--", linewidth=1.2,
               label=f"100 Hz target ({CYCLE_TARGET_MS} ms)")
    ax.axvline(CYCLE_STRETCH_MS, color="red", linestyle="--", linewidth=1.2,
               label=f"1 kHz stretch goal ({CYCLE_STRETCH_MS} ms)")
    ax.axvline(solve_ms.mean(), color="green", linestyle="-", linewidth=1.4,
               label=f"mean = {solve_ms.mean():.2f} ms")
    ax.set_xlabel("Solve time [ms]")
    ax.set_ylabel("Number of cycles")
    ax.set_title("Solver Latency Distribution")
    ax.legend(loc="upper right")
    ax.grid(True, alpha=0.4)

    # Over time
    ax = axes[1]
    ax.plot(time, solve_ms, color="steelblue", linewidth=0.8, alpha=0.8)
    ax.axhline(CYCLE_TARGET_MS, color="orange", linestyle="--", linewidth=1.0)
    ax.axhline(CYCLE_STRETCH_MS, color="red",    linestyle="--", linewidth=1.0)
    ax.set_ylabel("Solve time [ms]")
    ax.set_title("Solver Latency Over Time")
    _fmt_time_axis(ax, add_traj_line=True)
    ax.grid(True, alpha=0.4)

    plt.tight_layout()

    # Quick stats to the terminal too.
    print()
    print("Solver latency stats:")
    print(f"  mean : {solve_ms.mean():.3f} ms")
    print(f"  p50  : {solve_ms.median():.3f} ms")
    print(f"  p95  : {solve_ms.quantile(0.95):.3f} ms")
    print(f"  p99  : {solve_ms.quantile(0.99):.3f} ms")
    print(f"  max  : {solve_ms.max():.3f} ms")
    print(f"  >10ms: {(solve_ms > 10).sum()} cycles ({(solve_ms > 10).mean()*100:.2f}%)")
else:
    print("Note: solve_time_ms column not present in CSV - skipping latency plot.")


# =============================================================================
# Figure 3 - All-joints torque overlay with limits
# =============================================================================
# The signature plot of the coupled QP. Shows which joints bind their
# constraints and how they coordinate.

fig, axes = plt.subplots(4, 2, figsize=(14, 14), sharex=True)
axes = axes.flatten()
fig.suptitle("Coupled QP — Torque vs Limits (all 7 joints)",
             fontsize=14, fontweight="bold")
for i in range(1, 8):
    ax = axes[i-1]
    lim = EFFORT_LIMITS[i]
    ax.plot(time, df[f"tau_raw{i}"], color=colors[i-1], linewidth=1.3)
    ax.axhline( lim, color="red", linestyle="--", linewidth=0.8)
    ax.axhline(-lim, color="red", linestyle="--", linewidth=0.8)
    ax.axhline(0, color="black", linewidth=0.4)
    ax.set_title(f"Joint {i}   (limit ±{lim} Nm)")
    ax.set_ylabel("Torque [Nm]")
    _fmt_time_axis(ax, add_traj_line=True)
    ax.grid(True, alpha=0.4)
axes[7].set_visible(False)
plt.tight_layout()


# =============================================================================
# Figure 4 - All-joints tracking error overlay
# =============================================================================

fig, ax = plt.subplots(figsize=(12, 6))
for i in range(1, 8):
    ax.plot(time, df[f"error{i}"] * R2D, label=f"Joint {i}",
            color=colors[i-1], linewidth=1.2)
ax.axhline( 0.01 * R2D, color="gray", linestyle=":", linewidth=0.8,
            label=f"±{0.01*R2D:.3f}°  (= ±0.01 rad)")
ax.axhline(-0.01 * R2D, color="gray", linestyle=":", linewidth=0.8)
ax.axhline(0, color="black", linewidth=0.5)
ax.set_ylabel("Tracking error [°]")
_fmt_deg_yaxis(ax)
_fmt_time_axis(ax, add_traj_line=True)
ax.set_title("Coupled QP — Tracking Error (all 7 joints)")
ax.legend(loc="upper right", ncol=2)
ax.grid(True, alpha=0.4)
plt.tight_layout()


# =============================================================================
# Figure 5 - MPC cost over time
# =============================================================================

fig, ax = plt.subplots(figsize=(12, 5))
# If there's a single "cost" column from the coupled QP, plot that.
# Otherwise, plot the sum across per-joint best_costN.
if "qp_cost" in df.columns:
    ax.plot(time, df["qp_cost"], color="purple", linewidth=1.4)
    label = "QP optimal cost"
else:
    total = sum(df[f"best_cost{i}"] for i in range(1, 8))
    ax.plot(time, total, color="purple", linewidth=1.4)
    label = "sum of per-joint costs"
ax.set_ylabel(label)
ax.set_yscale("symlog", linthresh=1.0)
_fmt_time_axis(ax, add_traj_line=True)
ax.set_title("Coupled QP — Optimizer Cost Over Time")
ax.grid(True, alpha=0.4)
plt.tight_layout()


# =============================================================================
# Summary printout
# =============================================================================

print()
print("=" * 70)
print("End-of-run summary:")
print("=" * 70)
for i in range(1, 8):
    final_err = df[f"error{i}"].iloc[-1]
    peak_tau  = df[f"tau_raw{i}"].abs().max()
    lim = EFFORT_LIMITS[i]
    pct = peak_tau / lim * 100
    binding = " ← BINDING" if pct > 99 else (" ← close" if pct > 90 else "")
    print(f"  J{i}: final err = {final_err*R2D:+.3f}°,  "
          f"peak |tau| = {peak_tau:6.2f}/{lim:3.0f} Nm  ({pct:5.1f}%){binding}")
print("=" * 70)

plt.show()