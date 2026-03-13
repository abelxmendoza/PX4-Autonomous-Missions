import pandas as pd
import matplotlib.pyplot as plt
import glob
import os

# Find the most recent flight log
logs = sorted(glob.glob("/home/azrael/Desktop/px4-autonomous-mission/flight_log_*.csv"))
if not logs:
    print("No flight logs found.")
    exit()

latest = logs[-1]
print(f"Plotting: {latest}")

df = pd.read_csv(latest)

fig, axes = plt.subplots(2, 2, figsize=(12, 8))
fig.suptitle("Lawnmower Scan — Flight Telemetry", fontsize=14, fontweight="bold")

# --- Plot 1: Flight path (lat/lon) ---
ax1 = axes[0, 0]
ax1.plot(df["longitude"], df["latitude"], "b-o", markersize=3, linewidth=1.5)
ax1.scatter(df["longitude"].iloc[0], df["latitude"].iloc[0], color="green", s=100, zorder=5, label="Start")
ax1.scatter(df["longitude"].iloc[-1], df["latitude"].iloc[-1], color="red", s=100, zorder=5, label="End")
ax1.set_title("Flight Path (Lat/Lon)")
ax1.set_xlabel("Longitude")
ax1.set_ylabel("Latitude")
ax1.legend()
ax1.grid(True)

# --- Plot 2: Altitude over time ---
ax2 = axes[0, 1]
ax2.plot(range(len(df)), df["alt_rel_m"], "r-", linewidth=1.5)
ax2.set_title("Relative Altitude Over Time")
ax2.set_xlabel("Sample")
ax2.set_ylabel("Altitude (m)")
ax2.grid(True)

# --- Plot 3: Latitude over time ---
ax3 = axes[1, 0]
ax3.plot(range(len(df)), df["latitude"], "g-", linewidth=1.5)
ax3.set_title("Latitude Over Time")
ax3.set_xlabel("Sample")
ax3.set_ylabel("Latitude (deg)")
ax3.grid(True)

# --- Plot 4: Longitude over time ---
ax4 = axes[1, 1]
ax4.plot(range(len(df)), df["longitude"], "m-", linewidth=1.5)
ax4.set_title("Longitude Over Time")
ax4.set_xlabel("Sample")
ax4.set_ylabel("Longitude (deg)")
ax4.grid(True)

plt.tight_layout()

# Save to file
output = "/home/azrael/Desktop/px4-autonomous-mission/flight_plot.png"
plt.savefig(output, dpi=150)
print(f"Plot saved to: {output}")
plt.show()
