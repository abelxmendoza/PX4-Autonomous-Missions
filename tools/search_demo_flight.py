#!/usr/bin/env python3
"""Scripted search flight over the search field: takeoff, a lawnmower sweep, land,
while geolocating every ArUco marker the downward camera sees.

    GZ_IP=127.0.0.1 python tools/search_demo_flight.py [--altitude 8] [--speed 4] [--report out.json]

The lanes are fixed in advance (they cover the field outline from worlds/search_field.sdf)
and the flight does not react to detections. Each detection is turned into a ground
position from PX4's OWN position and attitude estimate (MAVSDK telemetry) and the pixel
(search_geolocate.py); the ground-truth target file is never read here. Score the report
afterwards with tools/score_search.py. Talks to PX4 SITL with MAVSDK on UDP 14540.
"""
from __future__ import annotations

import os

# Must precede every protobuf import (MAVSDK's included): Gazebo's generated messages only
# load with the pure-Python protobuf backend, and the backend is fixed at first import.
os.environ.setdefault("PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION", "python")
os.environ.setdefault("GZ_IP", "127.0.0.1")

import argparse  # noqa: E402
import asyncio  # noqa: E402
import json  # noqa: E402
import math  # noqa: E402
import sys  # noqa: E402
import time  # noqa: E402
from pathlib import Path  # noqa: E402

import cv2  # noqa: E402
from mavsdk import System  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "px4_offboard"))
from px4_offboard.gz_camera_feed import CameraFeed  # noqa: E402
from px4_offboard.search_geolocate import SEARCH_CAMERA, TargetTracker, geolocate  # noqa: E402
from px4_offboard.search_trace import TraceRecorder  # noqa: E402
from px4_offboard.gz_trail import GazeboTrail  # noqa: E402
from px4_offboard.vision_marker_detect import detect_markers  # noqa: E402

# Lanes over the 60 x 60 m field (NED metres from home). At 8 m the camera footprint is
# about 19 m across, so 12 m lane spacing overlaps; east +-24 m plus that reaches the edges.
LANE_NORTH_M = (14.0, 26.0, 38.0, 50.0)
LANE_EAST_M = 24.0
EARTH_M_PER_DEG = 111_320.0


def lawnmower() -> list[tuple[float, float]]:
    points = []
    for i, n in enumerate(LANE_NORTH_M):
        a, b = (-LANE_EAST_M, LANE_EAST_M) if i % 2 == 0 else (LANE_EAST_M, -LANE_EAST_M)
        points += [(n, a), (n, b)]
    return points


def offset(lat: float, lon: float, north: float, east: float) -> tuple[float, float]:
    return (lat + north / EARTH_M_PER_DEG,
            lon + east / (EARTH_M_PER_DEG * math.cos(math.radians(lat))))


class Pose:
    """PX4's latest estimate, as MAVSDK reports it (NED from the EKF origin, FRD->NED quaternion)."""
    position_ned: tuple[float, float, float] | None = None
    q_wxyz: tuple[float, float, float, float] | None = None


async def _follow_pose(drone: System, pose: Pose, stop: asyncio.Event, trace: TraceRecorder, t0: float,
                       trail: GazeboTrail | None) -> None:
    # Each loop breaks out on its own: leaving `async for` closes the MAVSDK stream
    # properly. Cancelling these tasks from outside instead left a stream thread blocked
    # and the process never exited after landing.
    async def position():
        async for pv in drone.telemetry.position_velocity_ned():
            if stop.is_set():
                break
            pose.position_ned = (pv.position.north_m, pv.position.east_m, pv.position.down_m)
            if pose.q_wxyz is not None:
                trace.pose(time.monotonic() - t0, pose.position_ned, pose.q_wxyz)
            if trail is not None:
                trail.points.add(pose.position_ned)

    async def attitude():
        async for q in drone.telemetry.attitude_quaternion():
            if stop.is_set():
                break
            pose.q_wxyz = (q.w, q.x, q.y, q.z)

    await asyncio.gather(position(), attitude())


async def _geolocate_detections(feed: CameraFeed, pose: Pose, tracker: TargetTracker, active: asyncio.Event,
                                stats: dict, stop: asyncio.Event, trace: TraceRecorder, t0: float) -> None:
    last = 0
    while not stop.is_set():
        await asyncio.sleep(0.05)
        frame, count = feed.latest()
        if frame is None or count == last or not active.is_set() or pose.position_ned is None or pose.q_wxyz is None:
            continue
        last = count
        stats["frames"] += 1
        for d in detect_markers(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)):
            stats["detections"] += 1
            hit = geolocate(SEARCH_CAMERA, pose.position_ned, pose.q_wxyz, d.center_x_px, d.center_y_px)
            if hit is None:
                continue
            now = time.monotonic() - t0
            trace.sighting(now, d.marker_id, hit[0], hit[1])
            for mid in tracker.add(d.marker_id, hit[0], hit[1], now):
                trace.confirmed(now, mid)
                est = tracker.estimates()[mid]
                print(f"  found marker {mid} at north {est.north:+.1f} m, east {est.east:+.1f} m", flush=True)


async def _draw_trail(trail: GazeboTrail, stop: asyncio.Event) -> None:
    """Refresh the pink path in the Gazebo window (GUI-only marker) twice a second."""
    while not stop.is_set():
        await asyncio.to_thread(trail.publish)  # blocking gz request, off the event loop
        await asyncio.sleep(0.5)


async def fly(altitude: float, speed: float, report_path: Path, trace_path: Path | None) -> None:
    feed = CameraFeed()
    trace = TraceRecorder(rate_hz=10.0)
    try:
        trail = GazeboTrail(min_step_m=0.5)
    except Exception as exc:  # a missing marker service must never stop the flight
        print(f"(no Gazebo trail: {exc})", flush=True)
        trail = None
    tracker = TargetTracker(min_detections=3)
    pose, active, stop = Pose(), asyncio.Event(), asyncio.Event()
    stats = {"frames": 0, "detections": 0}
    drone = System()
    await drone.connect(system_address="udpin://0.0.0.0:14540")
    print("waiting for PX4 ...", flush=True)
    async for state in drone.core.connection_state():
        if state.is_connected:
            break
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            break
    home = await anext(drone.telemetry.home())
    print(f"home {home.latitude_deg:.6f}, {home.longitude_deg:.6f}", flush=True)
    for setter, hz in ((drone.telemetry.set_rate_position_velocity_ned, 30.0),
                       (drone.telemetry.set_rate_attitude_quaternion, 50.0)):
        try:
            await setter(hz)
        except Exception as exc:  # not every PX4 build accepts every rate; defaults still work
            print(f"(telemetry rate request refused: {exc})", flush=True)
    t0 = time.monotonic()  # trace time zero: telemetry ready, just before arming
    tasks = [asyncio.create_task(_follow_pose(drone, pose, stop, trace, t0, trail)),
             asyncio.create_task(_geolocate_detections(feed, pose, tracker, active, stats, stop, trace, t0))]
    if trail is not None:
        tasks.append(asyncio.create_task(_draw_trail(trail, stop)))

    await drone.action.set_takeoff_altitude(altitude)
    await drone.action.arm()
    await drone.action.takeoff()
    async for pos in drone.telemetry.position():
        if pos.relative_altitude_m > altitude - 0.5:
            break
    await drone.action.set_current_speed(speed)
    active.set()  # geolocate only at survey altitude, not during takeoff/landing

    target_amsl = home.absolute_altitude_m + altitude
    for k, (n, e) in enumerate(lawnmower(), start=1):
        lat, lon = offset(home.latitude_deg, home.longitude_deg, n, e)
        yaw = 90.0 if e > 0 else 270.0
        print(f"leg {k}: to north {n:+.0f} m, east {e:+.0f} m", flush=True)
        await drone.action.goto_location(lat, lon, target_amsl, yaw)
        async for pos in drone.telemetry.position():
            dn = (pos.latitude_deg - lat) * EARTH_M_PER_DEG
            de = (pos.longitude_deg - lon) * EARTH_M_PER_DEG * math.cos(math.radians(lat))
            if math.hypot(dn, de) < 1.5:
                break

    active.clear()
    print("sweep done, landing", flush=True)
    await drone.action.land()
    async for in_air in drone.telemetry.in_air():
        if not in_air:
            break
    print("landed", flush=True)
    stop.set()
    await asyncio.wait_for(asyncio.gather(*tasks), timeout=10.0)
    feed.close()
    confirmed = tracker.confirmed()
    report = {
        "world": "search_field",
        "position_source": "PX4 local position + attitude estimate (MAVSDK telemetry); ground truth not used",
        "camera": "x500_search_cam downward camera, 640x480, hfov 1.74 rad",
        "altitude_m": altitude, "speed_mps": speed,
        "frames_processed": stats["frames"], "marker_detections": stats["detections"],
        "targets": [{"id": e.marker_id, "north": round(e.north, 3), "east": round(e.east, 3),
                     "detections": e.detections, "spread_m": round(e.spread_m, 3)}
                    for e in sorted(confirmed.values(), key=lambda e: e.marker_id)],
        "unconfirmed": [{"id": e.marker_id, "detections": e.detections}
                        for e in tracker.estimates().values() if e.marker_id not in confirmed],
    }
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(json.dumps(report, indent=2) + "\n")
    print(f"report: {report_path} ({len(confirmed)} targets)", flush=True)
    if trace_path is not None:
        trace_path.parent.mkdir(parents=True, exist_ok=True)
        trace_path.write_text(json.dumps(trace.to_dict(
            altitude_m=altitude, speed_mps=speed,
            position_source="PX4 local position + attitude estimate (MAVSDK telemetry)",
            camera={"width_px": SEARCH_CAMERA.width_px, "height_px": SEARCH_CAMERA.height_px,
                    "hfov_rad": SEARCH_CAMERA.horizontal_fov_rad},
            final_targets=report["targets"],
        ), separators=(",", ":")) + "\n")
        print(f"trace: {trace_path} ({len(trace.frames)} poses, {len(trace.sightings)} sightings)", flush=True)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--altitude", type=float, default=8.0)
    parser.add_argument("--speed", type=float, default=4.0)
    parser.add_argument("--report", type=Path, default=Path("/tmp/search_demo/search_report.json"))
    parser.add_argument("--trace", type=Path, help="also save the flight for the browser replay")
    args = parser.parse_args()
    asyncio.run(fly(args.altitude, args.speed, args.report, args.trace))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
