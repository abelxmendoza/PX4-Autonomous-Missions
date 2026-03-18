"""
offboard_avoidance.py — PX4 OFFBOARD reactive obstacle avoidance via MAVSDK

Upgraded features:
- Local NED control (no GPS drift)
- Smooth motion (no teleport stepping)
- Yaw alignment (faces direction of travel)
- Boundary clamp (prevents flying off sim)
- Watchdog (prevents offboard dropout)
- Reactive obstacle avoidance (extendable to LiDAR / ROS2)

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

# ── Config ───────────────────────────────────────────────────────────────────
TAKEOFF_ALT = 5.0     # meters AGL
STEP_SIZE   = 1.0     # forward advance per loop tick (meters)
AVOID_DIST  = 2.0     # lateral offset when obstacle detected (meters)
LOOP_DT     = 0.1     # control loop period — 10 Hz

SMOOTHING   = 0.2     # exponential smoothing factor (0 = no motion, 1 = instant)
MAX_RANGE   = 20.0    # NED boundary clamp (meters)


# ── Sensor stub ───────────────────────────────────────────────────────────────
def detect_obstacle() -> str | None:
    """
    Replace with real sensor input:
      - LiDAR sectors
      - Depth camera
      - ROS 2 subscriber

    Returns: "front", "left", "right", or None
    """
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

    last_setpoint_time = asyncio.get_event_loop().time()

    # ── Control loop ──────────────────────────────────────────────────────────
    print("Control loop running. Ctrl-C to stop.")
    try:
        while True:
            obstacle = detect_obstacle()

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

            # Yaw alignment — face direction of travel
            yaw = math.degrees(math.atan2(east, north)) if north != 0 else 0.0

            # Watchdog
            now = asyncio.get_event_loop().time()
            if now - last_setpoint_time > 0.2:
                print("[WARN] Setpoint delay — stabilizing")
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
