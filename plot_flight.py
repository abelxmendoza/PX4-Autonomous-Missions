"""Plot the most recent flight log (MAVSDK GPS or ROS mission NED)."""

from __future__ import annotations

import glob
import os

import matplotlib.pyplot as plt
import pandas as pd

ROOT = os.path.dirname(os.path.abspath(__file__))
logs = sorted(glob.glob(os.path.join(ROOT, "flight_log_*.csv")))
if not logs:
    print("No flight logs found.")
    raise SystemExit(1)

latest = logs[-1]
print(f"Plotting: {latest}")
df = pd.read_csv(latest)
is_mission = "north" in df.columns

fig, axes = plt.subplots(2, 2, figsize=(12, 8))
title = "Offboard Mission — NED Telemetry" if is_mission else "Flight Telemetry"
fig.suptitle(title, fontsize=14, fontweight="bold")

if is_mission:
    ax1 = axes[0, 0]
    ax1.plot(df["east"], df["north"], "b-", linewidth=1.2, label="path")
    ax1.scatter(df["east"].iloc[0], df["north"].iloc[0], c="green", s=80, zorder=5, label="Start")
    ax1.scatter(df["east"].iloc[-1], df["north"].iloc[-1], c="red", s=80, zorder=5, label="End")
    if "tgt_e" in df.columns:
        ax1.plot(df["tgt_e"], df["tgt_n"], "c--", alpha=0.5, linewidth=1, label="setpoint")
    ax1.set_title("Path (East / North)")
    ax1.set_xlabel("East (m)")
    ax1.set_ylabel("North (m)")
    ax1.axis("equal")
    ax1.legend()
    ax1.grid(True)

    ax2 = axes[0, 1]
    ax2.plot(-df["down"], "r-", linewidth=1.5, label="alt AGL")
    if "tgt_d" in df.columns:
        ax2.plot(-df["tgt_d"], "c--", alpha=0.6, label="setpoint alt")
    ax2.set_title("Altitude Over Time")
    ax2.set_xlabel("Sample")
    ax2.set_ylabel("Altitude (m)")
    ax2.legend()
    ax2.grid(True)

    ax3 = axes[1, 0]
    ax3.plot(df["north"], "g-", label="north")
    ax3.plot(df["east"], "m-", label="east")
    ax3.set_title("NED Horizontal")
    ax3.set_xlabel("Sample")
    ax3.set_ylabel("metres")
    ax3.legend()
    ax3.grid(True)

    ax4 = axes[1, 1]
    if "obstacle" in df.columns:
        flags = (df["obstacle"].fillna("") != "").astype(int)
        ax4.fill_between(range(len(flags)), flags, step="mid", alpha=0.5, label="avoiding")
        ax4.set_ylim(-0.1, 1.2)
        ax4.set_title("Avoidance Active")
        ax4.set_xlabel("Sample")
        ax4.set_ylabel("flag")
        ax4.legend()
        ax4.grid(True)
    elif "state" in df.columns:
        ax4.plot(df["wp_index"], "k-", linewidth=1.2)
        ax4.set_title("Waypoint Index")
        ax4.set_xlabel("Sample")
        ax4.grid(True)
else:
    ax1 = axes[0, 0]
    ax1.plot(df["longitude"], df["latitude"], "b-o", markersize=3, linewidth=1.5)
    ax1.scatter(df["longitude"].iloc[0], df["latitude"].iloc[0], color="green", s=100, zorder=5, label="Start")
    ax1.scatter(df["longitude"].iloc[-1], df["latitude"].iloc[-1], color="red", s=100, zorder=5, label="End")
    ax1.set_title("Flight Path (Lat/Lon)")
    ax1.set_xlabel("Longitude")
    ax1.set_ylabel("Latitude")
    ax1.legend()
    ax1.grid(True)

    ax2 = axes[0, 1]
    ax2.plot(range(len(df)), df["alt_rel_m"], "r-", linewidth=1.5)
    ax2.set_title("Relative Altitude Over Time")
    ax2.set_xlabel("Sample")
    ax2.set_ylabel("Altitude (m)")
    ax2.grid(True)

    ax3 = axes[1, 0]
    ax3.plot(range(len(df)), df["latitude"], "g-", linewidth=1.5)
    ax3.set_title("Latitude Over Time")
    ax3.set_xlabel("Sample")
    ax3.set_ylabel("Latitude (deg)")
    ax3.grid(True)

    ax4 = axes[1, 1]
    ax4.plot(range(len(df)), df["longitude"], "m-", linewidth=1.5)
    ax4.set_title("Longitude Over Time")
    ax4.set_xlabel("Sample")
    ax4.set_ylabel("Longitude (deg)")
    ax4.grid(True)

plt.tight_layout()
output = os.path.join(ROOT, "flight_plot.png")
plt.savefig(output, dpi=150)
print(f"Plot saved to: {output}")
plt.show()
