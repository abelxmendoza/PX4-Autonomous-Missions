"""
offboard_avoidance.py — PX4 OFFBOARD reactive obstacle avoidance via MAVSDK

Uses LOCAL NED coordinates and a continuous setpoint loop (OFFBOARD mode).
Reacts to obstacles dynamically via a sensor stub — replace detect_obstacle()
with real LiDAR, depth camera, or ROS subscriber output.

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

from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw

# ── Config ──────────────────────────────────────────────────────────────────
TAKEOFF_ALT_M = 5.0    # meters AGL
STEP_SIZE_M   = 1.0    # forward advance per loop tick (north, meters)
AVOID_DIST_M  = 2.0    # lateral offset when obstacle detected (meters)
LOOP_DT_S     = 0.1    # control loop period (10 Hz)


# ── Sensor stub ─────────────────────────────────────────────────────────────
def detect_obstacle() -> str | None:
    """
    Replace with real sensor input: LiDAR, depth camera, or ROS topic subscriber.

    Returns:
        "front"  — obstacle directly ahead
        "left"   — obstacle on the left side
        "right"  — obstacle on the right side
        None     — path clear
    """
    return None


# ── Main ─────────────────────────────────────────────────────────────────────
async def main():
    drone = System(mavsdk_server_address="localhost", port=50051)
    await drone.connect()

    print("Connecting to MAVSDK server (localhost:50051)...")

    async def _wait_for_connection():
        async for state in drone.core.connection_state():
            if state.is_connected:
                return

    try:
        await asyncio.wait_for(_wait_for_connection(), timeout=10)
        print("Drone connected via MAVSDK server!")
    except asyncio.TimeoutError:
        print("ERROR: Could not reach MAVSDK server. Is 'mavsdk_server udpout://127.0.0.1:14580' running?")
        return

    print("Waiting for local position lock (EKF)...")
    async for health in drone.telemetry.health():
        if health.is_local_position_ok and health.is_global_position_ok:
            print("Position OK")
            break

    # ── Arm + takeoff ────────────────────────────────────────────────────────
    print("Arming...")
    await drone.action.arm()

    print(f"Taking off to {TAKEOFF_ALT_M} m...")
    await drone.action.set_takeoff_altitude(TAKEOFF_ALT_M)
    await drone.action.takeoff()
    await asyncio.sleep(5)

    # ── OFFBOARD mode ─────────────────────────────────────────────────────────
    # PX4 requires at least one setpoint before offboard can be activated.
    print("Setting initial setpoint...")
    await drone.offboard.set_position_ned(
        PositionNedYaw(0.0, 0.0, -TAKEOFF_ALT_M, 0.0)
    )

    print("Starting OFFBOARD mode...")
    try:
        await drone.offboard.start()
    except OffboardError as e:
        print(f"ERROR: Failed to start offboard: {e._result.result}")
        await drone.action.disarm()
        return

    # ── Reactive control loop ─────────────────────────────────────────────────
    north = 0.0
    east  = 0.0
    yaw   = 0.0

    print("Offboard control loop running. Ctrl-C to stop.\n")
    try:
        while True:
            obstacle = detect_obstacle()

            if obstacle == "front":
                print("  [AVOID] Obstacle ahead → sidestep east")
                east += AVOID_DIST_M
            elif obstacle == "left":
                print("  [AVOID] Obstacle left → sidestep east")
                east += AVOID_DIST_M
            elif obstacle == "right":
                print("  [AVOID] Obstacle right → sidestep west")
                east -= AVOID_DIST_M
            else:
                north += STEP_SIZE_M

            await drone.offboard.set_position_ned(
                PositionNedYaw(north, east, -TAKEOFF_ALT_M, yaw)
            )
            print(f"  NED → N:{north:6.1f}  E:{east:6.1f}  D:{-TAKEOFF_ALT_M:.1f}")
            await asyncio.sleep(LOOP_DT_S)

    except KeyboardInterrupt:
        print("\nInterrupt received — stopping offboard and landing...")

    # ── Shutdown ──────────────────────────────────────────────────────────────
    await drone.offboard.stop()
    await drone.action.land()

    async for is_armed in drone.telemetry.armed():
        if not is_armed:
            print("Disarmed — complete.")
            break


if __name__ == "__main__":
    asyncio.run(main())
