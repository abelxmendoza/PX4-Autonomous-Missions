"""
obstacle_avoidance.py — PX4 obstacle avoidance mission via MAVSDK

Navigates around 5 static obstacles defined in worlds/obstacle_world.sdf
using pre-planned GPS waypoints with a 6 m lateral clearance buffer.

Obstacle positions (ENU, meters from spawn at 47.397742°N 8.545594°E):
  OB1 — (east=12, north=10) — 3×3×4 m  — red building
  OB2 — (east=28, north=10) — 3×3×6 m  — orange tower
  OB3 — (east=10, north=24) — 4×3×4 m  — green block
  OB4 — (east=24, north=24) — 2×2×5 m  — blue pillar
  OB5 — (east=18, north=38) — 5×3×4 m  — purple wall

Avoidance strategy (pre-planned waypoints, 20 m altitude):
  WP1 → approach north, east of spawn          (e= 5, n= 5)
  WP2 → pass OB1 on the WEST side              (e= 5, n=15)  OB1 west edge @ e=10.5 → 5.5 m buffer ✓
  WP3 → fly east above OB1/OB2 latitude        (e=18, n=15)  clears both tops at 20 m ✓
  WP4 → thread between OB3 (west) & OB4 (east) (e=18, n=24)  OB3 east edge @12, OB4 west edge @23 → 6 m each ✓
  WP5 → continue north past OB3/OB4            (e=18, n=32)
  WP6 → dodge east of OB5 wall                 (e=26, n=36)  OB5 east edge @20.5 → 5.5 m buffer ✓
  WP7 → north of all obstacles                 (e=26, n=46)
  WP8 → final destination                      (e= 0, n=50)
  RTL  → return to launch

Run:
  # Terminal 1 — PX4 SITL + Gazebo Harmonic
  cd ~/PX4-Autopilot
  PX4_GZ_WORLD=obstacle_world make px4_sitl gz_x500

  # Terminal 2 — MAVSDK server
  mavsdk_server udpin://0.0.0.0:14540

  # Terminal 3 — this script
  python3 obstacle_avoidance.py
"""

import asyncio
import csv
import math
import time

from mavsdk import System
from mavsdk.mission import MissionItem, MissionPlan

# ── GPS origin (PX4 Gazebo Harmonic default spawn) ─────────────────────────
ORIGIN_LAT = 47.397742
ORIGIN_LON = 8.545594
ALTITUDE_M = 20.0   # fly well above tallest obstacle (OB2 = 6 m)
SPEED_MS   = 4.0    # m/s — moderate speed for avoidance clarity

# Unit conversion at 47.4°N
_LAT_PER_M = 1.0 / 111_000              # °/m northing
_LON_PER_M = 1.0 / (111_000 * math.cos(math.radians(ORIGIN_LAT)))  # °/m easting


def enu_to_gps(east_m: float, north_m: float):
    """Convert local ENU offset (meters) to GPS (lat, lon)."""
    return (
        ORIGIN_LAT + north_m * _LAT_PER_M,
        ORIGIN_LON + east_m  * _LON_PER_M,
    )


def make_waypoint(lat, lon, alt=ALTITUDE_M, fly_through=True, loiter=0):
    return MissionItem(
        latitude_deg=lat,
        longitude_deg=lon,
        relative_altitude_m=alt,
        speed_m_s=SPEED_MS,
        is_fly_through=fly_through,
        gimbal_pitch_deg=0,
        gimbal_yaw_deg=0,
        camera_action=MissionItem.CameraAction.NONE,
        loiter_time_s=loiter,
        camera_photo_interval_s=0,
        acceptance_radius_m=2,
        yaw_deg=float("nan"),
        camera_photo_distance_m=0,
        vehicle_action=MissionItem.VehicleAction.NONE,
    )


# ── Obstacle-aware waypoint path ────────────────────────────────────────────
#
#  Each tuple: (east_m, north_m, label)
#  Clearance annotations are in the module docstring above.
#
AVOIDANCE_PATH = [
    ( 5,  5,  "WP1 — initial heading NE"),
    ( 5, 15,  "WP2 — west of OB1 (OB1 @ e=12)"),
    (18, 15,  "WP3 — east above OB1/OB2 row"),
    (18, 24,  "WP4 — between OB3 (e=10) and OB4 (e=24)"),
    (18, 32,  "WP5 — clear of OB3/OB4"),
    (26, 36,  "WP6 — east of OB5 (OB5 @ e=18, east edge 20.5)"),
    (26, 46,  "WP7 — north of all obstacles"),
    ( 0, 50,  "WP8 — final destination"),
]


async def log_telemetry(drone, stop_event, log_path):
    with open(log_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["time", "latitude", "longitude", "alt_abs_m", "alt_rel_m"])
        async for pos in drone.telemetry.position():
            if stop_event.is_set():
                break
            row = [
                time.strftime("%H:%M:%S"),
                round(pos.latitude_deg, 7),
                round(pos.longitude_deg, 7),
                round(pos.absolute_altitude_m, 2),
                round(pos.relative_altitude_m, 2),
            ]
            writer.writerow(row)
            f.flush()
            print(f"  LOG | {row[0]} | lat={row[1]} lon={row[2]} | alt={row[4]}m")
            await asyncio.sleep(1)
    print(f"Flight log saved to: {log_path}")


async def main():
    drone = System(mavsdk_server_address="localhost", port=50051)
    await drone.connect()

    print("Connecting to MAVSDK server (localhost:50051)...")
    try:
        async with asyncio.timeout(10):
            async for state in drone.core.connection_state():
                if state.is_connected:
                    print("Drone connected via MAVSDK server!")
                    break
    except asyncio.TimeoutError:
        print("ERROR: Could not reach MAVSDK server. Is 'mavsdk_server udpout://127.0.0.1:14580' running?")
        return

    print("Waiting for global position lock...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("Global position OK")
            break

    print("Clearing any existing mission...")
    await drone.mission.clear_mission()
    await asyncio.sleep(1)

    # Build mission items from the avoidance path
    print(f"\nObstacle avoidance path ({len(AVOIDANCE_PATH)} waypoints @ {ALTITUDE_M}m):")
    mission_items = []
    for east, north, label in AVOIDANCE_PATH:
        lat, lon = enu_to_gps(east, north)
        print(f"  {label}")
        print(f"    lat={lat:.7f}  lon={lon:.7f}  (e={east}m, n={north}m)")
        mission_items.append(make_waypoint(lat, lon))

    # Loiter 3 s at the final waypoint
    last_lat, last_lon = enu_to_gps(*AVOIDANCE_PATH[-1][:2])
    mission_items[-1] = make_waypoint(last_lat, last_lon, fly_through=False, loiter=3)

    print("\nUploading mission...")
    await drone.mission.upload_mission(MissionPlan(mission_items))

    log_path = f"flight_log_avoidance_{time.strftime('%Y%m%d_%H%M%S')}.csv"
    stop_event = asyncio.Event()
    log_task = asyncio.ensure_future(log_telemetry(drone, stop_event, log_path))

    print("Arming...")
    await drone.action.arm()

    print("Starting obstacle avoidance mission...\n")
    await drone.mission.start_mission()

    async for progress in drone.mission.mission_progress():
        pct = int(progress.current / progress.total * 100) if progress.total else 0
        label = AVOIDANCE_PATH[progress.current - 1][2] if progress.current > 0 else "—"
        print(f"  [{pct:3d}%] WP {progress.current}/{progress.total}  {label}")
        if progress.current == progress.total:
            print("\nAll waypoints reached — obstacles cleared!")
            break

    print("Returning to launch...")
    await drone.action.return_to_launch()

    async for is_armed in drone.telemetry.armed():
        if not is_armed:
            print("Disarmed — mission complete.")
            break

    stop_event.set()
    await log_task


if __name__ == "__main__":
    asyncio.run(main())
