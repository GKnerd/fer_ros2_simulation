import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

R2D = 180.0 / np.pi

EFFORT_LIMITS = {1: 87, 2: 87, 3: 87, 4: 87, 5: 12, 6: 12, 7: 12}

df = pd.read_csv("mpc_full_log.csv")
time = df["time"]

# ── Per-joint figures (5 panels each) ─────────────────────────────────────────
for i in range(1, 8):
    fig, axes = plt.subplots(5, 1, figsize=(11, 16), sharex=True)
    fig.suptitle(f"MPC — Joint {i} Diagnostics", fontsize=13, fontweight="bold")

    # 1. Position tracking: actual vs trajectory reference vs fixed goal
    ax = axes[0]
    ax.plot(time, df[f"q{i}"]    * R2D, label="actual",    color="steelblue",  linewidth=1.5)
    ax.plot(time, df[f"qref{i}"] * R2D, label="qref (traj)", color="tomato",   linestyle="--", linewidth=1.2)
    ax.plot(time, df[f"qdes{i}"] * R2D, label="qdes (goal)", color="gray",     linestyle=":",  linewidth=0.9)
    ax.set_ylabel("Position [deg]")
    ax.legend(loc="upper right")
    ax.grid(True, alpha=0.4)

    # 2. Tracking error (q_ref - q) with ±0.01 rad tolerance band
    ax = axes[1]
    ax.plot(time, df[f"error{i}"] * R2D, color="darkorange", linewidth=1.4, label="error = qref − q")
    ax.axhline( 0.01 * R2D, color="gray", linestyle=":", linewidth=0.8, label="±0.01 rad")
    ax.axhline(-0.01 * R2D, color="gray", linestyle=":", linewidth=0.8)
    ax.axhline(0, color="black", linewidth=0.5)
    ax.set_ylabel("Error [deg]")
    ax.legend(loc="upper right")
    ax.grid(True, alpha=0.4)

    # 3. Torque: raw vs saturated with joint limits
    ax = axes[2]
    lim = EFFORT_LIMITS[i]
    ax.plot(time, df[f"tau_raw{i}"], label="tau_raw", color="mediumpurple", alpha=0.7, linewidth=1.2)
    ax.plot(time, df[f"tau_sat{i}"], label="tau_sat", color="darkviolet",   linestyle="--", linewidth=1.4)
    ax.axhline( lim, color="red", linestyle="--", linewidth=0.8, label=f"±{lim} Nm limit")
    ax.axhline(-lim, color="red", linestyle="--", linewidth=0.8)
    ax.set_ylabel("Torque [Nm]")
    ax.legend(loc="upper right")
    ax.grid(True, alpha=0.4)

    # 4. MPC optimizer cost — should decrease as robot approaches target
    ax = axes[3]
    ax.plot(time, df[f"best_cost{i}"], color="seagreen", linewidth=1.4, label="best_cost")
    ax.set_ylabel("MPC Cost")
    ax.set_yscale("symlog", linthresh=1.0)   # log scale handles large early values
    ax.legend(loc="upper right")
    ax.grid(True, alpha=0.4)

    # 5. Virtual acceleration chosen by MPC
    ax = axes[4]
    ax.plot(time, df[f"v{i}"], color="chocolate", linewidth=1.4, label="v (virtual accel)")
    ax.axhline(0, color="black", linewidth=0.5)
    ax.set_ylabel("v [rad/s²]")
    ax.set_xlabel("Time [s]")
    ax.legend(loc="upper right")
    ax.grid(True, alpha=0.4)

    plt.tight_layout()

# ── Velocity tracking overlay (all joints) ────────────────────────────────────
# Added: dq_ref is now part of the MPC cost, so verifying velocity tracking
# shows whether the optimizer is actually matching the reference velocity.
fig, axes = plt.subplots(4, 2, figsize=(14, 14), sharex=True)
axes = axes.flatten()
colors = plt.cm.tab10.colors
for i in range(1, 8):
    ax = axes[i - 1]
    ax.plot(time, df[f"dq{i}"],    label="actual dq",  color="steelblue",  linewidth=1.2)
    ax.plot(time, df[f"dqref{i}"], label="dqref",       color="tomato",    linestyle="--", linewidth=1.1)
    ax.axhline(0, color="black", linewidth=0.4)
    ax.set_title(f"Joint {i} — Velocity Tracking")
    ax.set_ylabel("dq [rad/s]")
    ax.set_xlabel("Time [s]")
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, alpha=0.4)
axes[7].set_visible(False)
fig.suptitle("MPC — Velocity Tracking (dq vs dqref)", fontsize=13, fontweight="bold")
plt.tight_layout()

# ── All-joints error overlay ───────────────────────────────────────────────────
fig, ax = plt.subplots(figsize=(11, 5))
for i in range(1, 8):
    ax.plot(time, df[f"error{i}"] * R2D, label=f"joint {i}", color=colors[i - 1])
ax.axhline( 0.01 * R2D, color="gray", linestyle=":", linewidth=0.8, label="±0.01 rad")
ax.axhline(-0.01 * R2D, color="gray", linestyle=":", linewidth=0.8)
ax.axhline(0, color="black", linewidth=0.5)
ax.set_xlabel("Time [s]")
ax.set_ylabel("Error [deg]")
ax.set_title("MPC — All Joints Tracking Error")
ax.legend(loc="upper right", ncol=2)
ax.grid(True, alpha=0.4)
plt.tight_layout()

# ── All-joints MPC cost overlay ───────────────────────────────────────────────
# Added: seeing all costs in one plot reveals which joints are hardest to control.
fig, ax = plt.subplots(figsize=(11, 5))
for i in range(1, 8):
    ax.plot(time, df[f"best_cost{i}"], label=f"joint {i}", color=colors[i - 1])
ax.set_xlabel("Time [s]")
ax.set_ylabel("MPC Best Cost")
ax.set_yscale("symlog", linthresh=1.0)
ax.set_title("MPC — All Joints Optimizer Cost")
ax.legend(loc="upper right", ncol=2)
ax.grid(True, alpha=0.4)
plt.tight_layout()

plt.show()
