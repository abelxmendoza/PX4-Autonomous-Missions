import asyncio
import csv
import time
from mavsdk import System
from mavsdk.mission import MissionItem, MissionPlan

LOG_FILE = f"/home/azrael/Desktop/mavsdk_scripts/flight_log_{time.strftime('%Y%m%d_%H%M%S')}.csv"

def make_waypoint(lat, lon, alt, fly_through=True, loiter=0):
    return MissionItem(
        latitude_deg=lat,
        longitude_deg=lon,
        relative_altitude_m=alt,
        speed_m_s=5,
        is_fly_through=fly_through,
        gimbal_pitch_deg=0,
        gimbal_yaw_deg=0,
        camera_action=MissionItem.CameraAction.NONE,
        loiter_time_s=loiter,
        camera_photo_interval_s=0,
        acceptance_radius_m=1,
        yaw_deg=float("nan"),
        camera_photo_distance_m=0,
        vehicle_action=MissionItem.VehicleAction.NONE
    )

async def log_telemetry(drone, stop_event):
    with open(LOG_FILE, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["time", "latitude", "longitude", "alt_abs_m", "alt_rel_m"])
        async for position in drone.telemetry.position():
            if stop_event.is_set():
                break
            row = [
                time.strftime("%H:%M:%S"),
                round(position.latitude_deg, 7),
                round(position.longitude_deg, 7),
                round(position.absolute_altitude_m, 2),
                round(position.relative_altitude_m, 2),
            ]
            writer.writerow(row)
            f.flush()
            print(f"  LOG | {row[0]} | lat={row[1]} lon={row[2]} | alt={row[4]}m")
            await asyncio.sleep(1)
    print(f"Flight log saved to: {LOG_FILE}")

async def main():
    drone = System(mavsdk_server_address="localhost", port=50051)
    await drone.connect(system_address="udpin://0.0.0.0:14540")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone connected!")
            break

    print("Waiting for global position...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("Global position OK")
            break

    print("Clearing any existing mission...")
    await drone.mission.clear_mission()
    await asyncio.sleep(1)

    mission_items = [
        make_waypoint(47.398039859999997, 8.5455725400000002, 10),
        make_waypoint(47.398036222362471, 8.5450146439425085, 15),
        make_waypoint(47.397825620791885, 8.5450092830163271, 10, fly_through=False, loiter=2),
    ]

    print("Uploading mission...")
    await drone.mission.upload_mission(MissionPlan(mission_items))

    stop_event = asyncio.Event()
    log_task = asyncio.ensure_future(log_telemetry(drone, stop_event))

    print("Arming...")
    await drone.action.arm()

    print("Starting mission...")
    await drone.mission.start_mission()

    async for progress in drone.mission.mission_progress():
        print(f"Mission progress: {progress.current}/{progress.total}")
        if progress.current == progress.total:
            print("Mission complete!")
            break

    print("Returning to launch...")
    await drone.action.return_to_launch()

    # Wait for disarm instead of landed state — more reliable in SITL
    async for is_armed in drone.telemetry.armed():
        if not is_armed:
            print("Disarmed — flight complete.")
            break

    stop_event.set()
    await log_task

asyncio.run(main())
