#!/usr/bin/env python3
"""Fly one integrated seeding sortie in PX4 SITL, then precision-land.

PX4 must already be running in ``search_field`` with ``x500_search_cam``
(see scripts/run_integrated_seeding_sitl.sh). This process is the only
trajectory publisher. Do not launch the ROS ``precision_land`` node with it.

The low-altitude seeding profile is used. The 15,000 ft profile is not.
PX4's action takeoff climbs clear of the ground; the orchestrator then owns
OFFBOARD setpoints for the climb, release, return, and ArUco descent.
``worlds/search_field_targets.json`` is read only after the flight, to score
the touchdown. It is not a guidance target.

    GZ_IP=127.0.0.1 python3 tools/integrated_seeding_sitl.py --report /tmp/integrated_seeding_sitl/report.json
"""

from __future__ import annotations

import os

os.environ.setdefault("PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION", "python")
os.environ.setdefault("GZ_IP", "127.0.0.1")

import argparse  # noqa: E402
import asyncio  # noqa: E402
import json  # noqa: E402
import math  # noqa: E402
import subprocess  # noqa: E402
import sys  # noqa: E402
import time  # noqa: E402
from dataclasses import replace  # noqa: E402
from datetime import datetime, timezone  # noqa: E402
from pathlib import Path  # noqa: E402

import cv2  # noqa: E402
from mavsdk import System  # noqa: E402
from mavsdk.offboard import OffboardError, PositionNedYaw, VelocityNedYaw  # noqa: E402
from mavsdk.telemetry import FlightMode, LandedState  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "px4_offboard"))
from px4_offboard.airspace import simulation_authorizer  # noqa: E402
from px4_offboard.altitude_reference import (  # noqa: E402
    AltitudeBand,
    AltitudeFrame,
    AltitudeMonitor,
    Px4AltitudeSnapshot,
)
from px4_offboard.gz_camera_feed import CameraFeed  # noqa: E402
from px4_offboard.mission_orchestrator import (  # noqa: E402
    MissionOrchestrator,
    MissionState,
    OrchestratorConfig,
    StagingConditions,
    VehicleInput,
)
from px4_offboard.precision_land import CameraTag, PrecisionLand  # noqa: E402
from px4_offboard.search_geolocate import SEARCH_CAMERA  # noqa: E402
from px4_offboard.seeding_mission import SeedingMission, sim_profile  # noqa: E402
from px4_offboard.seeding_supervisor import SeedingSupervisor  # noqa: E402
from px4_offboard.vision_marker_detect import camera_matrix, estimate_marker_poses  # noqa: E402
from px4_offboard.weather import WeatherAuthorizer, simulation_observation, simulation_policy  # noqa: E402
from px4_offboard.weather_provider import StaticWeatherProvider  # noqa: E402

MARKER_M = 1.0
LOOP_S = 0.05
# Matches simulation_observation()'s station elevation. The low-altitude
# release is 15 m above the local origin. Pairing that profile with PX4
# SITL's Zurich AMSL would make this surface observation inapplicable.
SIM_SURFACE_ELEVATION_M = 20.0


class Pose:
    position_ned: tuple[float, float, float] | None = None
    velocity_ned: tuple[float, float, float] = (0.0, 0.0, 0.0)
    q_wxyz: tuple[float, float, float, float] | None = None
    in_air: bool = False
    armed: bool = False
    on_ground: bool = False
    offboard: bool = False


def _yaw_deg(q: tuple[float, float, float, float]) -> float:
    w, x, y, z = q
    return math.degrees(math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))


def _px4_version() -> str:
    px4 = os.environ.get("PX4_DIR", str(Path.home() / "PX4-Autopilot"))
    try:
        return subprocess.check_output(
            ["git", "-C", px4, "describe", "--always", "--dirty"],
            text=True,
            stderr=subprocess.DEVNULL,
        ).strip()
    except (OSError, subprocess.CalledProcessError):
        return "unknown"


def _profile():
    """sim_profile geometry with a looser arrival and a SITL-length timeout.

    sim_profile() itself stays at arrival 0.5 m and timeout 60 s. Those
    numbers are what the unit tests fly. A real PX4 approach does not settle
    inside 0.5 m on the first pass, and the sortie is longer than 60 s.
    """
    return replace(sim_profile(), arrival_m=4.0, timeout_s=300.0)


def _orchestrator() -> MissionOrchestrator:
    observation = simulation_observation(elevation_m=SIM_SURFACE_ELEVATION_M)
    supervisor = SeedingSupervisor(
        SeedingMission(_profile()),
        AltitudeMonitor(max_age_s=1.0),
        AltitudeBand(AltitudeFrame.LOCAL, 12.0, 18.0),
        WeatherAuthorizer(StaticWeatherProvider(observation), simulation_policy()),
        simulation_authorizer(),
    )
    # Staging is 18 m so the downward camera's footprint can include marker 4
    # near the origin. The marker coordinate is not a setpoint.
    staging = StagingConditions(
        max_horizontal_m=5.0,
        min_altitude_m=16.0,
        max_altitude_m=20.0,
        max_horizontal_speed_mps=1.5,
        staging_altitude_m=18.0,
    )
    return MissionOrchestrator(
        supervisor,
        PrecisionLand(),
        OrchestratorConfig(require_weather=True, landing_timeout_s=150.0, staging=staging),
    )


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

    async def armed():
        async for flag in drone.telemetry.armed():
            if stop.is_set():
                break
            pose.armed = bool(flag)

    async def ground():
        async for state in drone.telemetry.landed_state():
            if stop.is_set():
                break
            pose.on_ground = state == LandedState.ON_GROUND

    async def mode():
        async for item in drone.telemetry.flight_mode():
            if stop.is_set():
                break
            pose.offboard = item == FlightMode.OFFBOARD

    await asyncio.gather(position(), attitude(), air(), armed(), ground(), mode())


def _see_marker(frame, now_s: float) -> tuple[CameraTag, int] | None:
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    intrinsics = camera_matrix(SEARCH_CAMERA.width_px, SEARCH_CAMERA.height_px, SEARCH_CAMERA.horizontal_fov_rad)
    poses = estimate_marker_poses(gray, intrinsics, MARKER_M)
    if not poses:
        return None
    chosen = max(poses, key=lambda item: item.side_px)
    return CameraTag(chosen.position_xyz, chosen.orientation_xyzw, now_s), chosen.marker_id


def _vehicle(pose: Pose, now_s: float, dt_s: float, landed: bool, tag: CameraTag | None) -> VehicleInput | None:
    if pose.q_wxyz is None:
        return None
    down = 0.0 if pose.position_ned is None else pose.position_ned[2]
    north_east_down = pose.velocity_ned
    return VehicleInput(
        time_s=now_s,
        position_ned=pose.position_ned,
        velocity_ned=north_east_down,
        attitude_wxyz=pose.q_wxyz,
        landed=landed,
        armed=pose.armed,
        offboard=pose.offboard,
        altitude=Px4AltitudeSnapshot(
            time_s=now_s,
            baro_alt_m=None if pose.position_ned is None else -down,
            local_z_m=None if pose.position_ned is None else down,
            local_vz_mps=north_east_down[2],
            z_valid=pose.position_ned is not None,
            vz_valid=True,
            ref_alt_m=SIM_SURFACE_ELEVATION_M,
            baro_fault=False,
        ),
        tag=tag,
        dt_s=dt_s,
    )


async def _publish(drone: System, output, pose: Pose) -> None:
    if not output.heartbeat or output.request_nav_land:
        return
    yaw = 0.0 if pose.q_wxyz is None else _yaw_deg(pose.q_wxyz)
    if output.use_position and output.position_ned is not None:
        north, east, down = output.position_ned
        await drone.offboard.set_position_ned(PositionNedYaw(north, east, down, yaw))
        return
    velocity = output.velocity_ned
    if velocity is not None and all(math.isfinite(v) for v in velocity):
        await drone.offboard.set_velocity_ned(VelocityNedYaw(velocity[0], velocity[1], velocity[2], yaw))
        return
    if pose.position_ned is not None:
        north, east, down = pose.position_ned
        await drone.offboard.set_position_ned(PositionNedYaw(north, east, down, yaw))


def _blank_report(started: str) -> dict:
    return {
        "mission_id": "integrated-seeding-sitl",
        "scenario_id": "search_field_low_altitude",
        "simulation_environment": {
            "world": "search_field",
            "vehicle": "x500_search_cam",
            "headless": True,
            "sitl": "px4_sitl_gz",
        },
        "px4_version": _px4_version(),
        "ros_distribution": "not_used",
        "ros_note": "MAVSDK offboard is the only setpoint publisher. The ROS precision_land node is not started.",
        "start_time": started,
        "end_time": None,
        "mission_result": "in_progress",
        "truth_used_for_guidance": False,
        "validation": "PX4 SITL attempted",
        "physical_flight_validated": False,
    }


def _write(path: Path, report: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(report, indent=2) + "\n")


async def fly(report_path: Path, timeout_s: float) -> dict:
    started = datetime.now(timezone.utc).isoformat()
    report = _blank_report(started)
    feed = CameraFeed()
    pose, stop = Pose(), asyncio.Event()
    drone = System()
    follower = None
    orch = _orchestrator()
    marker_id = None
    frames = 0
    detections = 0
    last_frame = 0
    reached_release = False
    try:
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

        wall0 = time.monotonic()
        follower = asyncio.create_task(_follow(drone, pose, stop))
        await drone.action.set_takeoff_altitude(8.0)
        await drone.action.arm()
        await drone.action.takeoff()
        print("takeoff to 8 m, then the orchestrator owns OFFBOARD", flush=True)
        while pose.position_ned is None or -pose.position_ned[2] < 7.4 or pose.q_wxyz is None:
            if time.monotonic() - wall0 > 90.0:
                raise TimeoutError("takeoff did not reach 8 m")
            await asyncio.sleep(0.1)

        north, east, down = pose.position_ned
        await drone.offboard.set_position_ned(PositionNedYaw(north, east, down, _yaw_deg(pose.q_wxyz)))
        try:
            await drone.offboard.start()
        except OffboardError as exc:
            raise RuntimeError(f"offboard start failed: {exc}") from exc
        mode_deadline = time.monotonic() + 2.0
        while not pose.offboard and time.monotonic() < mode_deadline:
            await asyncio.sleep(0.05)

        mission_t0 = time.monotonic()
        orch.start(0.0)
        last_state = orch.state
        last_print = 0.0
        previous_time = 0.0
        deadline = time.monotonic() + timeout_s

        while time.monotonic() < deadline:
            await asyncio.sleep(LOOP_S)
            now = time.monotonic() - mission_t0
            dt = max(0.0, now - previous_time)
            previous_time = now
            altitude = 0.0 if pose.position_ned is None else -pose.position_ned[2]
            landed = bool(pose.on_ground or (not pose.in_air and altitude < 1.0 and now > 5.0))
            tag = None
            if orch.state is MissionState.PRECISION_LANDING:
                frame, count = feed.latest()
                if frame is not None and count != last_frame:
                    last_frame = count
                    frames += 1
                    seen = _see_marker(frame, now)
                    if seen is not None:
                        tag, marker_id = seen
                        detections += 1
            vehicle = _vehicle(pose, now, dt, landed, tag)
            if vehicle is None:
                continue
            output = orch.step(vehicle)
            if output.state is not last_state:
                print(f"  {last_state.value} -> {output.state.value} at t={now:.1f}s owner={output.owner.value}", flush=True)
                last_state = output.state
            if output.state in (MissionState.HOLD_FOR_RELEASE, MissionState.DISPENSING):
                reached_release = True
            if now - last_print > 5.0 and pose.position_ned is not None:
                last_print = now
                speed = math.hypot(pose.velocity_ned[0], pose.velocity_ned[1])
                print(
                    f"  t={now:.0f}s {output.state.value} n={pose.position_ned[0]:.1f} e={pose.position_ned[1]:.1f} "
                    f"alt={altitude:.1f} vxy={speed:.2f} dispense={output.dispense} marker={marker_id}",
                    flush=True,
                )
            if output.request_nav_land:
                print("orchestrator requested PX4 land mode", flush=True)
                try:
                    await drone.offboard.stop()
                except OffboardError:
                    pass
                await drone.action.land()
                break
            await _publish(drone, output, pose)
            if output.state in (MissionState.LANDED, MissionState.ABORTED, MissionState.FAILED):
                break
        else:
            print("wall-clock timeout", flush=True)
            report["mission_result"] = "simulation_terminated"

        try:
            await drone.offboard.stop()
        except OffboardError:
            pass
        if pose.in_air and orch.state is not MissionState.LANDED:
            print("still airborne; commanding PX4 land", flush=True)
            await drone.action.land()
            land_deadline = time.monotonic() + 60.0
            while pose.in_air and time.monotonic() < land_deadline:
                await asyncio.sleep(0.2)

        final = pose.position_ned
        evidence = orch.evidence()
        if report["mission_result"] != "simulation_terminated":
            report["mission_result"] = evidence["mission_result"]
        if report["mission_result"] == "completed":
            report["validation"] = "PX4 SITL VALIDATED"
        elif report["mission_result"] == "aborted_recovered":
            report["validation"] = "PX4 SITL aborted and recovered"
        report.update(_score(final, marker_id, evidence, reached_release, frames, detections))
        report["end_time"] = datetime.now(timezone.utc).isoformat()
        report["mission_duration_s"] = round(time.monotonic() - mission_t0, 2)
        report["orchestrator_evidence"] = evidence
    except Exception as exc:
        report["mission_result"] = "simulation_terminated"
        report["failure"] = str(exc)
        report["end_time"] = datetime.now(timezone.utc).isoformat()
        print(f"integrated seeding sitl failed: {exc}", flush=True)
        raise
    finally:
        report["end_time"] = report["end_time"] or datetime.now(timezone.utc).isoformat()
        _write(report_path, report)
        print(json.dumps({k: report[k] for k in report if k != "orchestrator_evidence"}, indent=2), flush=True)
        stop.set()
        if follower is not None:
            try:
                await asyncio.wait_for(follower, timeout=5.0)
            except asyncio.TimeoutError:
                follower.cancel()
        feed.close()
    return report


def _score(final, marker_id, evidence, reached_release, frames, detections) -> dict:
    truth_target = None
    truth_error = None
    if marker_id is not None and final is not None:
        truth = json.loads((ROOT / "worlds" / "search_field_targets.json").read_text())
        match = next((item for item in truth["targets"] if item["id"] == marker_id), None)
        if match is not None:
            truth_target = {
                "id": match["id"],
                "north_m": match["north"],
                "east_m": match["east"],
                "frame": "local_ned",
                "source": "worlds/search_field_targets.json",
                "estimated": False,
                "simulated": True,
                "used_for_guidance": False,
            }
            truth_error = math.hypot(final[0] - match["north"], final[1] - match["east"])
    mission = evidence.get("mission") or {}
    ages = (evidence.get("precision_landing_evidence") or {}).get("descent_tag_age_s") or {}
    return {
        "reached_release_point": reached_release,
        "release_authorized": mission.get("sample_count", 0) > 0 and not evidence.get("release_aborted"),
        "release_sample_count": mission.get("sample_count", 0),
        "release_amount_kg": mission.get("amount_kg"),
        "dispenser_closed": evidence.get("dispenser_closed"),
        "return_completed": any(
            item.get("destination") == "precision_landing" for item in evidence.get("mission_state_history", [])
        ),
        "marker_detection_succeeded": detections > 0,
        "marker_id": marker_id,
        "precision_landing_completed": evidence.get("mission_result") in ("completed", "aborted_recovered"),
        "touchdown_position": None
        if final is None
        else {
            "frame": "local_ned",
            "north_m": round(final[0], 3),
            "east_m": round(final[1], 3),
            "down_m": round(final[2], 3),
            "source": "px4_local_position",
            "estimated": True,
            "simulated": True,
            "units": "m",
        },
        "marker_ground_truth": truth_target,
        "touchdown_horizontal_error_m": None if truth_error is None else round(truth_error, 3),
        "camera_frames": frames,
        "marker_detections": detections,
        "descent_tag_age_s": ages,
        "fault_codes": [item.get("code") for item in evidence.get("fault_events", [])],
        "ref_alt_m": {
            "value": SIM_SURFACE_ELEVATION_M,
            "units": "m",
            "source": "simulated weather station elevation",
            "estimated": False,
            "simulated": True,
            "note": "Not PX4 GPS AMSL. Local NED still comes from PX4. This keeps the surface observation applicable to the 15 m release.",
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", type=Path, default=Path("/tmp/integrated_seeding_sitl/report.json"))
    parser.add_argument("--timeout", type=float, default=360.0)
    args = parser.parse_args()
    try:
        report = asyncio.run(fly(args.report, args.timeout))
    except Exception:
        return 1
    return 0 if report.get("mission_result") == "completed" else 2


if __name__ == "__main__":
    raise SystemExit(main())
