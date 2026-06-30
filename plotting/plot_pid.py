import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.ticker as mticker

# ── Config ──────────────────────────────────────────────────────────────────
CSV_PATH  = "pid_log.csv"
R2D       = 180.0 / np.pi
WP_LABELS = ["A", "B", "C", "D", "E", "home"]
SAVE_DIR  = "figures/pid_waypoints_set4"
os.makedirs(SAVE_DIR, exist_ok=True)

df = pd.read_csv(CSV_PATH)

# ── Reconstruct absolute time (per-segment 'time' resets at each waypoint) ──
t_raw  = df["time"].to_numpy(dtype=float)
abs_t  = np.empty_like(t_raw)
abs_t[0] = t_raw[0]
offset   = 0.0
for i in range(1, len(t_raw)):
    if t_raw[i] < t_raw[i - 1] - 1.0:
        offset += t_raw[i - 1]
    abs_t[i] = t_raw[i] + offset

tmax   = abs_t[-1]
colors = plt.cm.tab10.colors

# ── Waypoint boundary times (absolute) ──────────────────────────────────────
wp_arr = df["current_waypoint"].to_numpy(dtype=int)
boundaries = []
for w in range(1, int(wp_arr.max()) + 1):
    idx = np.where(wp_arr == w)[0]
    if len(idx):
        boundaries.append((abs_t[idx[0]], WP_LABELS[w]))

# ── Helpers (mirrors plot_mpc_coupled.py) ───────────────────────────────────
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
    ylim = ax.get_ylim()
    ypos = ylim[1] - (ylim[1] - ylim[0]) * 0.05
    for t_b, lbl in boundaries:
        ax.axvline(t_b, color="grey", linestyle="--", linewidth=0.8, alpha=alpha)
        ax.text(t_b + tmax * 0.005, ypos, lbl,
                fontsize=7, color="dimgrey", va="top")


def _add_wp_segment_labels(ax):
    ylim = ax.get_ylim()
    ypos = ylim[1] - (ylim[1] - ylim[0]) * 0.07
    segs = [0.0] + [b[0] for b in boundaries] + [tmax]
    for k, lbl in enumerate(WP_LABELS[:len(segs) - 1]):
        mid = (segs[k] + segs[k + 1]) / 2.0
        ax.text(mid, ypos, lbl, fontsize=8, color="dimgrey",
                ha="center", va="top", style="italic")


def _fmt_deg_yaxis(ax):
    ax.yaxis.set_major_formatter(mticker.FuncFormatter(lambda x, _: f"{x:.3g}°"))


# =============================================================================
# Figure 1 — Per-joint diagnostics (one figure per joint, 3 panels)
#            Position / Tracking error / Velocity — matches the MPC layout
#            for the panels that PID logging actually has (no tau/v/cost).
# =============================================================================
for i in range(1, 8):
    fig, axes = plt.subplots(2, 1, figsize=(13, 8), sharex=True)
    fig.suptitle(f"PID — Joint {i} Full Run Diagnostics", fontsize=13, fontweight="bold")

    # Position
    ax = axes[0]
    ax.plot(abs_t, df[f"q{i}"]    * R2D, label="actual",      color="steelblue", lw=1.5)
    ax.plot(abs_t, df[f"qref{i}"] * R2D, label="qref (traj)", color="tomato",    lw=1.2, ls="--")
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
    ax.set_xlabel("Time [s]")
    _fmt_deg_yaxis(ax)
    _fmt_time_axis(ax)
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, alpha=0.35)
    _add_wp_lines(ax)

    plt.tight_layout()
    plt.savefig(os.path.join(SAVE_DIR, f"pid_joint{i}_diagnostics.png"), dpi=120)


# =============================================================================
# Figure 2 — All-joints tracking error overlay
# =============================================================================
fig, ax = plt.subplots(figsize=(13, 6))
for i in range(1, 8):
    ax.plot(abs_t, df[f"error{i}"] * R2D, label=f"J{i}", color=colors[i - 1], lw=1.2)
ax.axhline( 0.01 * R2D, color="gray", ls=":", lw=0.8, label=f"±{0.01*R2D:.3f}°")
ax.axhline(-0.01 * R2D, color="gray", ls=":", lw=0.8)
ax.axhline(0, color="black", lw=0.5)
ax.set_ylabel("Tracking error [°]")
_fmt_deg_yaxis(ax)
_fmt_time_axis(ax)
ax.set_title("PID — Tracking Error (all 7 joints)")
ax.legend(loc="upper right", ncol=4, fontsize=8)
ax.grid(True, alpha=0.35)
_add_wp_lines(ax)
_add_wp_segment_labels(ax)
plt.tight_layout()
plt.savefig(os.path.join(SAVE_DIR, "pid_all_joints_error.png"), dpi=120)


# =============================================================================
# Summary stats — same format used for the MPC runs, for direct comparison
# =============================================================================
def rms(x):
    return np.sqrt(np.mean(x ** 2))

dq = df[[f"dq{j}" for j in range(1, 8)]].to_numpy()
err = df[[f"error{j}" for j in range(1, 8)]].to_numpy()
motion_mask = np.max(np.abs(dq), axis=1) > 0.05

print(f"rows: {len(df)}   total_time: {tmax:.1f} s   motion_rows: {motion_mask.sum()}")
print(f"ALL RMS err:  {np.degrees(rms(err[motion_mask].flatten())):.4f} deg")
print(f"ALL diff(dq): {rms(np.diff(dq[motion_mask], axis=0).flatten()):.6f}")
print()
print(f"{'J':<4} {'RMS err':>10} {'peak err':>10} {'diff(dq)':>10}")
for j in range(7):
    e = np.degrees(err[motion_mask, j])
    print(f"J{j+1:<3} {rms(e):>10.4f} {np.max(np.abs(e)):>10.4f} {rms(np.diff(dq[motion_mask, j])):>10.6f}")

print("\nSaved: pid_joint1..7_diagnostics.png, pid_all_joints_error.png")
plt.show()

