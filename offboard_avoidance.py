"""
offboard_avoidance.py — PX4 OFFBOARD reactive obstacle avoidance via MAVSDK

Features:
- Local NED control (no GPS drift)
- Smooth motion (no teleport stepping)
- Yaw alignment (faces direction of travel)
- Boundary clamp (prevents flying off sim)
- Watchdog (prevents offboard dropout)
- Sim-aware obstacle detection (matches worlds/obstacle_world.sdf)

Loop = sense → decide → smooth → act → repeat

Architecture:
  Python → gRPC (localhost:50051) → MAVSDK server → MAVLink → PX4

Run:
  # Terminal 1 — PX4 SITL + Gazebo Harmonic
  cd ~/PX4-Autopilot
  PX4_GZ_WORLD=obstacle_world make px4_sitl gz_x500

  # Terminal 2 — MAVSDK server
  mavsdk_server udpout://127.0.0.1:14580

  # Terminal 3 — this script
  python3 offboard_avoidance.py
"""

import asyncio
import math

from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw

# ── Config ────────────────────────────────────────────────────────────────────
TAKEOFF_ALT = 5.0
STEP_SIZE   = 1.0
AVOID_DIST  = 2.0
LOOP_DT     = 0.1

SMOOTHING   = 0.2
MAX_RANGE   = 20.0

DETECTION_RADIUS = 4.0   # meters — obstacle triggers within this radius
FRONT_ANGLE      = 30    # degrees — cone in front classified as "front"
SIDE_ANGLE       = 60    # degrees — flanks classified as "left"/"right"

VEL_EPS = 0.1            # m — minimum displacement before updating yaw


# ── Obstacle map (matches worlds/obstacle_world.sdf) ─────────────────────────
# (east_m, north_m) — ENU from spawn origin
OBSTACLES = [
    (12, 10),   # OB1 — red building
    (28, 10),   # OB2 — orange tower
    (10, 24),   # OB3 — green block
    (24, 24),   # OB4 — blue pillar
    (18, 38),   # OB5 — purple wall
]


# ── Sim-aware sensor ──────────────────────────────────────────────────────────
def detect_obstacle(north: float, east: float, yaw_deg: float) -> str | None:
    """
    Geometry-based obstacle detection against the known SDF obstacle map.
    Replace OBSTACLES lookup with real LiDAR sectors or a ROS 2 subscriber
    when running on hardware or a sensor-equipped sim.

    Returns: "front", "left", "right", or None
    """
    for obs_e, obs_n in OBSTACLES:
        dn = obs_n - north
        de = obs_e - east
        distance = math.hypot(dn, de)

        if distance > DETECTION_RADIUS:
            continue

        angle = math.degrees(math.atan2(de, dn))
        rel_angle = (angle - yaw_deg + 180) % 360 - 180  # normalize to [-180, 180]

        if abs(rel_angle) < FRONT_ANGLE:
            return "front"
        elif 0 < rel_angle < SIDE_ANGLE:
            return "left"
        elif -SIDE_ANGLE < rel_angle < 0:
            return "right"

    return None


# ── Main ──────────────────────────────────────────────────────────────────────
async def main():
    drone = System(mavsdk_server_address="localhost", port=50051)
    await drone.connect()

    print("Connecting to MAVSDK server (localhost:50051)...")

    async def _wait_conn():
        async for state in drone.core.connection_state():
            if state.is_connected:
                return

    try:
        await asyncio.wait_for(_wait_conn(), timeout=10)
        print("Drone connected via MAVSDK server!")
    except asyncio.TimeoutError:
        print("ERROR: Could not reach MAVSDK server. Is 'mavsdk_server udpout://127.0.0.1:14580' running?")
        return

    print("Waiting for local position lock (EKF)...")
    async for health in drone.telemetry.health():
        if health.is_local_position_ok:
            break
    print("Position OK")

    # ── Arm + takeoff ─────────────────────────────────────────────────────────
    print("Arming...")
    await drone.action.arm()
    await drone.action.set_takeoff_altitude(TAKEOFF_ALT)
    await drone.action.takeoff()
    await asyncio.sleep(5)

    # ── Init OFFBOARD ─────────────────────────────────────────────────────────
    await drone.offboard.set_position_ned(
        PositionNedYaw(0.0, 0.0, -TAKEOFF_ALT, 0.0)
    )

    try:
        await drone.offboard.start()
    except OffboardError as e:
        print(f"ERROR: Failed to start offboard: {e}")
        await drone.action.disarm()
        return

    print("Offboard started\n")

    # ── State ─────────────────────────────────────────────────────────────────
    north = 0.0
    east  = 0.0

    target_north = 0.0
    target_east  = 0.0

    yaw = 0.0
    last_setpoint_time = asyncio.get_event_loop().time()

    # ── Control loop ──────────────────────────────────────────────────────────
    print("Control loop running. Ctrl-C to stop.")
    try:
        while True:
            obstacle = detect_obstacle(north, east, yaw)

            # Decide
            if obstacle == "front":
                target_north -= 1.0
                target_east  += AVOID_DIST
            elif obstacle == "left":
                target_east  += AVOID_DIST
            elif obstacle == "right":
                target_east  -= AVOID_DIST
            else:
                target_north += STEP_SIZE

            # Clamp
            target_north = max(min(target_north, MAX_RANGE), -MAX_RANGE)
            target_east  = max(min(target_east,  MAX_RANGE), -MAX_RANGE)

            # Smooth
            north += (target_north - north) * SMOOTHING
            east  += (target_east  - east)  * SMOOTHING

            # Yaw — only update when moving to avoid spin-in-place
            if math.hypot(north, east) > VEL_EPS:
                yaw = math.degrees(math.atan2(east, north))

            # Watchdog — re-hold position if loop fell behind
            now = asyncio.get_event_loop().time()
            if now - last_setpoint_time > 0.2:
                print("[WARN] Setpoint delay — re-hold position")
                await drone.offboard.set_position_ned(
                    PositionNedYaw(north, east, -TAKEOFF_ALT, yaw)
                )
            last_setpoint_time = now

            # Act
            await drone.offboard.set_position_ned(
                PositionNedYaw(north, east, -TAKEOFF_ALT, yaw)
            )

            print(f"[STATE] obs={str(obstacle):5} | N={north:6.2f}  E={east:6.2f}  yaw={yaw:6.1f}°")

            await asyncio.sleep(LOOP_DT)

    except KeyboardInterrupt:
        print("\nInterrupt received — stopping offboard and landing...")

    # ── Shutdown ──────────────────────────────────────────────────────────────
    await drone.offboard.stop()
    await drone.action.land()

    async for armed in drone.telemetry.armed():
        if not armed:
            break

    print("Disarmed — complete.")


if __name__ == "__main__":
    asyncio.run(main())
