import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

R2D = 180.0 / np.pi

EFFORT_LIMITS = {1: 87, 2: 87, 3: 87, 4: 87, 5: 12, 6: 12, 7: 12}

df = pd.read_csv("fl_full_log.csv")
time = df["time"]

for i in range(1, 8):
    fig, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
    fig.suptitle(f"Joint {i} Diagnostics", fontsize=13, fontweight="bold")

    # --- Position tracking ---
    ax = axes[0]
    ax.plot(time, df[f"q{i}"] * R2D, label="actual", color="steelblue")
    ax.plot(time, df[f"qdes{i}"] * R2D, label="desired", color="tomato", linestyle="--")
    ax.set_ylabel("Position [deg]")
    ax.legend(loc="upper right")
    ax.grid(True, alpha=0.4)

    # --- Tracking error with ±0.01 rad band ---
    ax = axes[1]
    ax.plot(time, df[f"error{i}"] * R2D, color="darkorange", label="error")
    ax.axhline( 0.01 * R2D, color="gray", linestyle=":", linewidth=0.8, label="±0.01 rad")
    ax.axhline(-0.01 * R2D, color="gray", linestyle=":",  linewidth=0.8)
    ax.axhline(0, color="black", linewidth=0.5)
    ax.set_ylabel("Error [deg]")
    ax.legend(loc="upper right")
    ax.grid(True, alpha=0.4)

    # --- Torque with saturation limits ---
    ax = axes[2]
    lim = EFFORT_LIMITS[i]
    ax.plot(time, df[f"tau_raw{i}"], label="tau_raw", color="mediumpurple", alpha=0.6)
    ax.plot(time, df[f"tau_sat{i}"], label="tau_sat", color="darkviolet", linestyle="--")
    ax.axhline( lim, color="red", linestyle="--", linewidth=0.8, label=f"±{lim} Nm limit")
    ax.axhline(-lim, color="red", linestyle="--", linewidth=0.8)
    ax.set_ylabel("Torque [Nm]")
    ax.set_xlabel("Time [s]")
    ax.legend(loc="upper right")
    ax.grid(True, alpha=0.4)

    plt.tight_layout()

# --- All joints error overlay ---
fig, ax = plt.subplots(figsize=(11, 5))
colors = plt.cm.tab10.colors
for i in range(1, 8):
    ax.plot(time, df[f"error{i}"] * R2D, label=f"joint {i}", color=colors[i-1])
ax.axhline( 0.01 * R2D, color="gray", linestyle=":", linewidth=0.8, label="±0.01 rad")
ax.axhline(-0.01 * R2D, color="gray", linestyle=":", linewidth=0.8)
ax.axhline(0, color="black", linewidth=0.5)
ax.set_xlabel("Time [s]")
ax.set_ylabel("Error [deg]")
ax.set_title("All Joints — Tracking Error")
ax.legend(loc="upper right", ncol=2)
ax.grid(True, alpha=0.4)
plt.tight_layout()

plt.show()
