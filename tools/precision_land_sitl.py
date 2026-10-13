#!/usr/bin/env python3
"""Fly a precision landing in PX4 SITL and record touchdown metrics.

PX4 must already be running in ``search_field`` with ``x500_search_cam``
(see scripts/run_precision_land_sitl.sh). The vehicle takes off, flies north
until the downward camera sees an ArUco marker, then ``PrecisionLand`` guides
the rest of the descent. Guidance uses PX4's own position and attitude plus
the camera. ``worlds/search_field_targets.json`` is read only after landing,
to score the touchdown.

    GZ_IP=127.0.0.1 python3 tools/precision_land_sitl.py --report /tmp/precision_land_sitl/report.json
"""

from __future__ import annotations

import os

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
from mavsdk.offboard import OffboardError, PositionNedYaw, VelocityNedYaw  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "px4_offboard"))
from px4_offboard.gz_camera_feed import CameraFeed  # noqa: E402
from px4_offboard.precision_land import (  # noqa: E402
    CameraTag,
    CommandMode,
    LandState,
    PrecisionLand,
    VehicleSnapshot,
    offboard_setpoint,
)
from px4_offboard.search_geolocate import SEARCH_CAMERA  # noqa: E402
from px4_offboard.vision_marker_detect import camera_matrix, estimate_marker_poses  # noqa: E402

MARKER_M = 1.0
TRANSIT_SPEED_MPS = 1.5
NORTH_LIMIT_M = 28.0
LOOP_S = 0.05


class Pose:
    position_ned: tuple[float, float, float] | None = None
    velocity_ned: tuple[float, float, float] = (0.0, 0.0, 0.0)
    q_wxyz: tuple[float, float, float, float] | None = None
    in_air: bool = False


def _yaw_rad(q: tuple[float, float, float, float]) -> float:
    w, x, y, z = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


async def _follow(drone: System, pose: Pose, stop: asyncio.Event) -> None:
    async def position():
        async for pv in drone.telemetry.position_velocity_ned():
            if stop.is_set():
                break
            pose.position_ned = (pv.position.north_m, pv.position.east_m, pv.position.down_m)
            pose.velocity_ned = (pv.velocity.north_m_s, pv.velocity.east_m_s, pv.velocity.down_m_s)

    async def attitude():
        async for q in drone.telemetry.attitude_quaternion():
            if stop.is_set():
                break
            pose.q_wxyz = (q.w, q.x, q.y, q.z)

    async def air():
        async for flag in drone.telemetry.in_air():
            if stop.is_set():
                break
            pose.in_air = bool(flag)

    await asyncio.gather(position(), attitude(), air())


def _snapshot(pose: Pose, now_s: float, landed: bool) -> VehicleSnapshot | None:
    if pose.position_ned is None or pose.q_wxyz is None:
        return None
    return VehicleSnapshot(
        position_ned=pose.position_ned,
        velocity_ned=pose.velocity_ned,
        attitude_wxyz=pose.q_wxyz,
        landed=landed,
        time_s=now_s,
    )


def _see_marker(frame, pose: Pose, now_s: float) -> tuple[CameraTag, int] | None:
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    intrinsics = camera_matrix(SEARCH_CAMERA.width_px, SEARCH_CAMERA.height_px, SEARCH_CAMERA.horizontal_fov_rad)
    poses = estimate_marker_poses(gray, intrinsics, MARKER_M)
    if not poses:
        return None
    chosen = max(poses, key=lambda item: item.side_px)
    return CameraTag(chosen.position_xyz, chosen.orientation_xyzw, now_s), chosen.marker_id


async def _send(drone: System, command, yaw_deg: float) -> None:
    fields = offboard_setpoint(command)
    if fields is None:
        return
    yaw = yaw_deg if math.isnan(fields["yaw"]) else math.degrees(fields["yaw"])
    if fields["use_position"]:
        north, east, down = fields["position_ned"]
        await drone.offboard.set_position_ned(PositionNedYaw(north, east, down, yaw))
        return
    vn, ve, vd = fields["velocity_ned"]
    await drone.offboard.set_velocity_ned(VelocityNedYaw(vn, ve, vd, yaw))


async def fly(report_path: Path, altitude_m: float, timeout_s: float) -> dict:
    feed = CameraFeed()
    pose, stop = Pose(), asyncio.Event()
    drone = System()
    await drone.connect(system_address="udpin://0.0.0.0:14540")
    print("waiting for PX4 ...", flush=True)
    async for state in drone.core.connection_state():
        if state.is_connected:
            break
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            break
    for setter, hz in (
        (drone.telemetry.set_rate_position_velocity_ned, 30.0),
        (drone.telemetry.set_rate_attitude_quaternion, 50.0),
    ):
        try:
            await setter(hz)
        except Exception as exc:
            print(f"(telemetry rate request refused: {exc})", flush=True)

    t0 = time.monotonic()
    follower = asyncio.create_task(_follow(drone, pose, stop))
    await drone.action.set_takeoff_altitude(altitude_m)
    await drone.action.arm()
    await drone.action.takeoff()
    print(f"takeoff to {altitude_m:.1f} m", flush=True)
    while pose.position_ned is None or -pose.position_ned[2] < altitude_m - 0.6:
        if time.monotonic() - t0 > 60.0:
            raise TimeoutError("takeoff did not reach altitude")
        await asyncio.sleep(0.1)

    await drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
    try:
        await drone.offboard.start()
    except OffboardError as exc:
        raise RuntimeError(f"offboard start failed: {exc}") from exc
    print("offboard, flying north until a marker is in view", flush=True)

    lander = PrecisionLand()
    marker_id: int | None = None
    last_frame = 0
    frames = 0
    detections = 0
    loss_events = 0
    was_failed = False
    descend_t0: float | None = None
    descend_t1: float | None = None
    ages: list[float] = []
    last_state = None
    result = "timeout"
    deadline = time.monotonic() + timeout_s

    last_tag = None
    hold_since: float | None = None

    while time.monotonic() < deadline:
        await asyncio.sleep(LOOP_S)
        now = time.monotonic() - t0
        snap = _snapshot(pose, now, landed=not pose.in_air and lander.state == LandState.DESCEND)
        if snap is None:
            continue
        frame, count = feed.latest()
        if frame is not None and count != last_frame:
            last_frame = count
            frames += 1
            seen = _see_marker(frame, pose, now)
            if seen is not None:
                tag, mid = seen
                detections += 1
                marker_id = mid
                if not lander.active and not lander.failed:
                    lander.activate(snap.position_ned)
                    print(
                        f"marker {mid} at t={now:.1f}s, north {snap.position_ned[0]:.1f} m — precision land",
                        flush=True,
                    )
                lander.observe_tag(tag, snap)

        if not lander.active and not lander.failed:
            if snap.position_ned[0] > NORTH_LIMIT_M:
                result = "no_marker"
                break
            await drone.offboard.set_velocity_ned(VelocityNedYaw(TRANSIT_SPEED_MPS, 0.0, 0.0, 0.0))
            continue

        command = lander.step(snap)
        if command.state != last_state:
            print(f"  {command.state.value} at t={now:.1f}s age={command.tag_age_s}", flush=True)
            last_state = command.state
        if lander.failed and not was_failed:
            loss_events += 1
            was_failed = True
        if command.state == LandState.DESCEND:
            if descend_t0 is None:
                descend_t0 = now
            if command.tag_age_s is not None:
                ages.append(command.tag_age_s)
        if command.mode == CommandMode.VELOCITY and lander.failed:
            raise RuntimeError("velocity command published after marker loss")
        await _send(drone, command, math.degrees(_yaw_rad(snap.attitude_wxyz)))
        if command.tag_ned is not None:
            last_tag = command.tag_ned
        if command.state == LandState.FINISHED or (lander.failed and not pose.in_air):
            descend_t1 = now
            result = "landed" if command.state == LandState.FINISHED else "held_on_ground"
            break
        if lander.failed:
            if hold_since is None:
                hold_since = now
            elif now - hold_since > 5.0:
                descend_t1 = now
                result = "marker_lost"
                break
        else:
            hold_since = None

    if result == "timeout":
        print("timeout", flush=True)
        if descend_t0 is not None:
            descend_t1 = time.monotonic() - t0
    guidance_end = pose.position_ned

    try:
        await drone.offboard.stop()
    except OffboardError:
        pass
    if pose.in_air:
        print("landing via PX4 land mode", flush=True)
        await drone.action.land()
        async for in_air in drone.telemetry.in_air():
            if not in_air:
                break
    final = pose.position_ned or guidance_end
    stop.set()
    try:
        await asyncio.wait_for(follower, timeout=5.0)
    except asyncio.TimeoutError:
        follower.cancel()
    feed.close()

    truth_error = None
    truth_target = None
    if marker_id is not None and final is not None:
        truth = json.loads((ROOT / "worlds" / "search_field_targets.json").read_text())
        match = next((item for item in truth["targets"] if item["id"] == marker_id), None)
        if match is not None:
            truth_target = {"id": match["id"], "north": match["north"], "east": match["east"]}
            truth_error = math.hypot(final[0] - match["north"], final[1] - match["east"])

    vision_error = None
    if last_tag is not None and final is not None:
        vision_error = math.hypot(final[0] - last_tag[0], final[1] - last_tag[1])

    report = {
        "world": "search_field",
        "vehicle": "x500_search_cam",
        "guidance": "PrecisionLand offboard setpoints; camera + PX4 estimate only",
        "truth_used_for_guidance": False,
        "result": result,
        "marker_id": marker_id,
        "altitude_m": altitude_m,
        "guidance_end_ned": None if guidance_end is None else [round(v, 3) for v in guidance_end],
        "touchdown_ned": None if final is None else [round(v, 3) for v in final],
        "last_tag_ned": None if last_tag is None else [round(v, 3) for v in last_tag],
        "horizontal_error_vs_vision_m": None if vision_error is None else round(vision_error, 3),
        "horizontal_error_vs_truth_m": None if truth_error is None else round(truth_error, 3),
        "truth_target": truth_target,
        "descent_duration_s": None if descend_t0 is None or descend_t1 is None else round(descend_t1 - descend_t0, 2),
        "marker_loss_events": loss_events,
        "min_tag_age_during_descent_s": None if not ages else round(min(ages), 3),
        "max_tag_age_during_descent_s": None if not ages else round(max(ages), 3),
        "frames": frames,
        "detections": detections,
    }
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2), flush=True)
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", type=Path, default=Path("/tmp/precision_land_sitl/report.json"))
    parser.add_argument("--altitude", type=float, default=6.0)
    parser.add_argument("--timeout", type=float, default=150.0)
    args = parser.parse_args()
    try:
        report = asyncio.run(fly(args.report, args.altitude, args.timeout))
    except Exception as exc:
        print(f"precision land sitl failed: {exc}", flush=True)
        return 1
    return 0 if report["result"] == "landed" else 2


if __name__ == "__main__":
    raise SystemExit(main())
