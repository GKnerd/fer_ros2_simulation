
import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.ticker as mticker

# ── Config ──────────────────────────────────────────────────────────────────
CSV_PATH      = "mpc_coupled_qp_log.csv"
R2D           = 180.0 / np.pi
EFFORT_LIMITS = {1: 87, 2: 87, 3: 87, 4: 87, 5: 12, 6: 12, 7: 12}
WP_LABELS     = ["A", "B", "C", "D", "E", "home"]
SAVE_DIR      = "figures/Waypoints_set4_t"
os.makedirs(SAVE_DIR, exist_ok=True)

df = pd.read_csv(CSV_PATH)

# ── Reconstruct absolute time ────────────────────────────────────────────────
# The 'time' column = t_now - start_time_, which resets to ~0 at each
# waypoint switch. Detect resets (large backward jumps) and add offsets.
t_raw  = df["time"].to_numpy(dtype=float)
abs_t  = np.empty_like(t_raw)
abs_t[0] = t_raw[0]
offset   = 0.0
for i in range(1, len(t_raw)):
    if t_raw[i] < t_raw[i - 1] - 1.0:   # backward jump → waypoint reset
        offset += t_raw[i - 1]
    abs_t[i] = t_raw[i] + offset

tmax   = abs_t[-1]
colors = plt.cm.tab10.colors

# ── Waypoint boundary times (absolute) ──────────────────────────────────────
wp_arr = df["current_waypoint"].to_numpy(dtype=int)
boundaries = []   # list of (abs_time, label) for each transition
for w in range(1, int(wp_arr.max()) + 1):
    idx = np.where(wp_arr == w)[0]
    if len(idx):
        boundaries.append((abs_t[idx[0]], WP_LABELS[w]))

# ── Helpers ──────────────────────────────────────────────────────────────────
def _fmt_time_axis(ax):
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


def _add_wp_lines(ax, alpha=0.35):
    """Grey vertical lines + waypoint label at each transition."""
    ylim = ax.get_ylim()
    ypos = ylim[1] - (ylim[1] - ylim[0]) * 0.05
    for t_b, lbl in boundaries:
        ax.axvline(t_b, color="grey", linestyle="--", linewidth=0.8, alpha=alpha)
        ax.text(t_b + tmax * 0.005, ypos, lbl,
                fontsize=7, color="dimgrey", va="top")


def _add_wp_segment_labels(ax):
    """Label each segment (A, B, … home) in the middle of its time window."""
    ylim = ax.get_ylim()
    ypos = ylim[1] - (ylim[1] - ylim[0]) * 0.07

    segs = [0.0] + [b[0] for b in boundaries] + [tmax]
    for k, lbl in enumerate(WP_LABELS[:len(segs) - 1]):
        mid = (segs[k] + segs[k + 1]) / 2.0
        ax.text(mid, ypos, lbl, fontsize=8, color="dimgrey",
                ha="center", va="top", style="italic")


def _fmt_deg_yaxis(ax):
    ax.yaxis.set_major_formatter(
        mticker.FuncFormatter(lambda x, _: f"{x:.3g}°"))


def _autoscale_torque(ax, *series):
    all_vals = np.concatenate([s for s in series])
    lo, hi = all_vals.min(), all_vals.max()
    span = max(hi - lo, 1.0)
    ax.set_ylim(lo - 0.15 * span, hi + 0.15 * span)


# =============================================================================
# Figure 1 — Per-joint diagnostics (one figure per joint, 5 panels)
# =============================================================================
for i in range(1, 8):
    fig, axes = plt.subplots(4, 1, figsize=(13, 14), sharex=True)
    fig.suptitle(f"Coupled QP MPC — Joint {i} Full Run Diagnostics",
                 fontsize=13, fontweight="bold")

    # Position
    ax = axes[0]
    ax.plot(abs_t, df[f"q{i}"]    * R2D, label="actual",      color="steelblue",  lw=1.5)
    ax.plot(abs_t, df[f"qref{i}"] * R2D, label="qref (traj)", color="tomato",     lw=1.2, ls="--")
    ax.plot(abs_t, df[f"qdes{i}"] * R2D, label="qdes (goal)", color="gray",       lw=0.9, ls=":")
    ax.set_ylabel("Position [°]")
    _fmt_deg_yaxis(ax)
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, alpha=0.35)
    _add_wp_lines(ax)
    _add_wp_segment_labels(ax)

    # Tracking error
    ax = axes[1]
    ax.plot(abs_t, df[f"error{i}"] * R2D, color="darkorange", lw=1.4,
            label="error = qref − q")
    ax.axhline( 0.01 * R2D, color="gray", ls=":", lw=0.8,
                label=f"±{0.01*R2D:.3f}°  (= ±0.01 rad)")
    ax.axhline(-0.01 * R2D, color="gray", ls=":", lw=0.8)
    ax.axhline(0, color="black", lw=0.5)
    ax.set_ylabel("Tracking error [°]")
    _fmt_deg_yaxis(ax)
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, alpha=0.35)
    _add_wp_lines(ax)

    # Torque
    ax = axes[2]
    lim = EFFORT_LIMITS[i]
    ax.plot(abs_t, df[f"tau_raw{i}"], label="tau (QP)",            color="seagreen",   lw=1.4)
    ax.plot(abs_t, df[f"tau_sat{i}"], label="tau_sat (clamp)",     color="darkviolet", lw=1.0, ls="--", alpha=0.7)
    ax.axhline( lim, color="red", ls="--", lw=0.8, label=f"±{lim} Nm limit")
    ax.axhline(-lim, color="red", ls="--", lw=0.8)
    _autoscale_torque(ax, df[f"tau_raw{i}"].values, df[f"tau_sat{i}"].values)
    ax.set_ylabel("Torque [Nm]")
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, alpha=0.35)
    _add_wp_lines(ax)

    # Virtual acceleration
    ax = axes[3]
    ax.plot(abs_t, df[f"v{i}"], color="chocolate", lw=1.4, label="virtual acceleration (QP output)")
    ax.axhline(0, color="black", lw=0.5)
    ax.set_ylabel("Virtual acceleration [rad/s²]")
    ax.set_xlabel("Time [s]")
    _fmt_time_axis(ax)
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, alpha=0.35)
    _add_wp_lines(ax)

    plt.tight_layout()
    plt.savefig(os.path.join(SAVE_DIR, f"Figure{i} qpc1n.png"), dpi=150)


# =============================================================================
# Figure 2 — Solver latency
# =============================================================================
if "solve_time_ms" in df.columns:
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    fig.suptitle("Coupled QP — Solver Latency", fontsize=13, fontweight="bold")

    solve_ms = df["solve_time_ms"]

    ax = axes[0]
    ax.hist(solve_ms, bins=60, color="steelblue", edgecolor="black", alpha=0.75)
    ax.axvline(10.0,              color="orange", ls="--", lw=1.2,
               label="100 Hz target (10 ms)")
    ax.axvline(solve_ms.mean(),   color="green",  ls="-",  lw=1.4,
               label=f"mean = {solve_ms.mean():.2f} ms")
    ax.set_xlabel("Solve time [ms]")
    ax.set_ylabel("Number of cycles")
    ax.set_title("Solver Latency Distribution")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.35)

    ax = axes[1]
    # Rolling median + IQR band over time — keeps the time axis (unlike a
    # per-segment boxplot) while smoothing out the raw per-cycle spike-noise
    # that made a plain line plot unreadable.
    win = max(int(round(1.0 / 0.01)), 5)  # ~1 second rolling window at ~100 Hz
    roll = solve_ms.rolling(win, center=True, min_periods=1)
    med  = roll.median()
    q25  = roll.quantile(0.25)
    q75  = roll.quantile(0.75)
    ax.fill_between(abs_t, q25, q75, color="steelblue", alpha=0.25, label="IQR (rolling 1 s)")
    ax.plot(abs_t, med, color="steelblue", lw=1.6, label="rolling median (1 s)")
    ax.axhline(10.0, color="orange", ls="--", lw=1.0, label="100 Hz target")
    ax.set_ylabel("Solve time [ms]")
    ax.set_xlabel("Time [s]")
    ax.set_title("Solver Latency Over Time (smoothed)")
    _fmt_time_axis(ax)
    _add_wp_lines(ax)
    _add_wp_segment_labels(ax)
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.35)

    plt.tight_layout()
    plt.savefig(os.path.join(SAVE_DIR, "Figure8 qpc1n.png"), dpi=150)

    print("\nSolver latency stats:")
    print(f"  mean : {solve_ms.mean():.3f} ms")
    print(f"  p50  : {solve_ms.median():.3f} ms")
    print(f"  p95  : {solve_ms.quantile(0.95):.3f} ms")
    print(f"  p99  : {solve_ms.quantile(0.99):.3f} ms")
    print(f"  max  : {solve_ms.max():.3f} ms")
    print(f"  >10ms: {(solve_ms > 10).sum()} cycles ({(solve_ms > 10).mean()*100:.2f}%)")


# =============================================================================
# Figure 3 — All-joints torque (data-scaled y-axis)
# =============================================================================
fig, axes = plt.subplots(4, 2, figsize=(14, 14), sharex=True)
axes = axes.flatten()
fig.suptitle("Coupled QP — Torque vs Limits (all 7 joints)",
             fontsize=14, fontweight="bold")
for i in range(1, 8):
    ax = axes[i - 1]
    lim = EFFORT_LIMITS[i]
    ax.plot(abs_t, df[f"tau_raw{i}"], color=colors[i - 1], lw=1.3)
    ax.axhline( lim, color="red", ls="--", lw=0.8, label=f"±{lim} Nm")
    ax.axhline(-lim, color="red", ls="--", lw=0.8)
    ax.axhline(0, color="black", lw=0.4)
    _autoscale_torque(ax, df[f"tau_raw{i}"].values)
    ax.set_title(f"Joint {i}  (limit ±{lim} Nm)")
    ax.set_ylabel("Torque [Nm]")
    ax.legend(loc="upper right", fontsize=7)
    _fmt_time_axis(ax)
    _add_wp_lines(ax)
    _add_wp_segment_labels(ax)
    ax.grid(True, alpha=0.35)
axes[7].set_visible(False)
plt.tight_layout()
plt.savefig(os.path.join(SAVE_DIR, "Figure9 qpc1n.png"), dpi=150)


# =============================================================================
# Figure 4 — All-joints tracking error overlay
# =============================================================================
fig, ax = plt.subplots(figsize=(13, 6))
for i in range(1, 8):
    ax.plot(abs_t, df[f"error{i}"] * R2D, label=f"J{i}",
            color=colors[i - 1], lw=1.2)
ax.axhline( 0.01 * R2D, color="gray", ls=":", lw=0.8,
            label=f"±{0.01*R2D:.3f}°")
ax.axhline(-0.01 * R2D, color="gray", ls=":", lw=0.8)
ax.axhline(0, color="black", lw=0.5)
ax.set_ylabel("Tracking error [°]")
_fmt_deg_yaxis(ax)
_fmt_time_axis(ax)
ax.set_title("Coupled QP — Tracking Error (all 7 joints)")
ax.legend(loc="upper right", ncol=4, fontsize=8)
ax.grid(True, alpha=0.35)
_add_wp_lines(ax)
_add_wp_segment_labels(ax)
plt.tight_layout()
plt.savefig(os.path.join(SAVE_DIR, "Figure10 qpc1n.png"), dpi=150)


# =============================================================================
# Figure 5 — MPC cost over time
# =============================================================================
fig, ax = plt.subplots(figsize=(13, 5))
if "qp_cost" in df.columns:
    ax.plot(abs_t, df["qp_cost"], color="purple", lw=1.4, label="QP optimal cost")
else:
    total = sum(df[f"best_cost{i}"] for i in range(1, 8))
    ax.plot(abs_t, total, color="purple", lw=1.4, label="sum of per-joint costs")
ax.set_ylabel("MPC cost")
ax.set_yscale("symlog", linthresh=1.0)
_fmt_time_axis(ax)
ax.set_title("Coupled QP — Optimizer Cost Over Time")
ax.legend(fontsize=8)
ax.grid(True, alpha=0.35)
_add_wp_lines(ax)
_add_wp_segment_labels(ax)
plt.tight_layout()
plt.savefig(os.path.join(SAVE_DIR, "Figure11 qpc1n.png"), dpi=150)


# =============================================================================
# Summary printout
# =============================================================================
print()
print("=" * 72)
print("End-of-run summary:")
print("=" * 72)
for i in range(1, 8):
    final_err = df[f"error{i}"].iloc[-1]
    peak_tau  = df[f"tau_raw{i}"].abs().max()
    lim = EFFORT_LIMITS[i]
    pct = peak_tau / lim * 100
    flag = " ← BINDING" if pct > 99 else (" ← close" if pct > 90 else "")
    print(f"  J{i}: final err={final_err*R2D:+.3f}°  "
          f"peak|tau|={peak_tau:6.2f}/{lim:3.0f} Nm ({pct:5.1f}%){flag}")
print("=" * 72)
print(f"\nTotal run: {tmax:.1f} s   Waypoints: {WP_LABELS}")

plt.show()
