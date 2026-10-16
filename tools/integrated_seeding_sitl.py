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
from px4_offboard.airspace import (  # noqa: E402
    AirspaceAuthorizer,
    AirspaceKind,
    AirspaceVolume,
    simulation_authorizer,
    simulation_dataset,
)
from px4_offboard.mission_faults import (  # noqa: E402
    AIRSPACE_FAULT,
    WEATHER_FAULT,
    AirspaceFaultSwitch,
    FaultSpec,
    MissionFaultPlan,
    WeatherFaultSwitch,
)
from px4_offboard.mission_geofence import GeofenceConfig, _contains  # noqa: E402
from px4_offboard.mission_logic import Fence  # noqa: E402
from px4_offboard.precision_land import (CameraTag, LandState, PrecisionLand, PrecisionLandParams,  # noqa: E402
                                         VehicleSnapshot, tag_to_ned)
from px4_offboard.seeding_trace import SeedingTraceRecorder  # noqa: E402
from px4_offboard import seeding_trace  # noqa: E402
from px4_offboard.simulated_comms import simulation_clearance, simulation_comms_authorizer  # noqa: E402
from px4_offboard.traffic import simulation_snapshot, simulation_traffic_authorizer, track_at_ned  # noqa: E402
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
from px4_offboard.search_geolocate import SEARCH_CAMERA  # noqa: E402
from px4_offboard.seeding_mission import SeedingMission, sim_profile  # noqa: E402
from px4_offboard.seeding_supervisor import SeedingSupervisor  # noqa: E402
from px4_offboard.vision_marker_detect import camera_matrix, estimate_marker_poses, track_landing_marker  # noqa: E402
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


def _recovery_site(scenario: str = "nominal") -> tuple[float, float, float]:
    """The planned landing site: the centre of the scenario's recovery area (a mission-plan
    coordinate from the geofence, not a marker position)."""
    polygon = _sitl_geofence(tight=scenario == "geofence_boundary").recovery_polygon_ned
    norths = [p[0] for p in polygon]
    easts = [p[1] for p in polygon]
    return ((min(norths) + max(norths)) / 2.0, (min(easts) + max(easts)) / 2.0, 0.0)


def _profile(scenario: str = "nominal"):
    """sim_profile geometry with a looser arrival, a SITL-length timeout, and the return
    going to the planned landing site instead of the takeoff point.

    sim_profile() itself stays at arrival 0.5 m and timeout 60 s. Those
    numbers are what the unit tests fly. A real PX4 approach does not settle
    inside 0.5 m on the first pass, and the sortie is longer than 60 s.

    Precision landing starts from wherever the return stages (18 m above home_ned). Staged
    over the takeoff point, the pad markers were 15-21 m off to the side, about 40 degrees
    off the camera axis and ~20 px wide: whichever was detected first got locked, and a
    first fix of marker 4 from there put it ~9 m short of where it is. The drone parked at
    the wrong spot, lost the marker and timed out (3 of 7 SITL runs, 2026-10-16). A real
    precision landing flies to the planned landing site first and refines with vision.
    """
    return replace(sim_profile(), arrival_m=4.0, timeout_s=300.0, home_ned=_recovery_site(scenario))


SCENARIOS = (
    "nominal",
    "altitude_denial",
    "weather_denial",
    "airspace_denial",
    "traffic_conflict",
    "clearance_expired",
    "weather_during_dispense",
    "airspace_during_dispense",
    "geofence_boundary",
    "marker_loss",
)

# Mission result the runner treats as the designed outcome. geofence_boundary
# holds inside the fence until the wall clock ends; the evidence is the
# rejected setpoint, not a completed landing.
EXPECTED_RESULT = {
    "nominal": "completed",
    "altitude_denial": "aborted_recovered",
    "weather_denial": "aborted_recovered",
    "airspace_denial": "aborted_recovered",
    "traffic_conflict": "aborted_recovered",
    "clearance_expired": "aborted_recovered",
    "weather_during_dispense": "aborted_recovered",
    "airspace_during_dispense": "aborted_recovered",
    "geofence_boundary": "simulation_terminated",
    "marker_loss": "failed",
}


def _sitl_geofence(tight: bool) -> GeofenceConfig:
    """Local NED keep-in for search_field.

    The nominal box contains home, the release point, and marker 4. The
    recovery polygon is where descent below the 2 m cruise floor is allowed.
    Its floor is -1 m so a PX4 touchdown estimate slightly below the local
    origin stays inside the recovery exception. The tight box stops at
    north 8 m, so the release setpoint is inadmissible.
    """
    north_max = 8.0 if tight else 50.0
    recovery_north = 6.0 if tight else 25.0
    source = "sitl_search_field_tight" if tight else "sitl_search_field"
    config = GeofenceConfig.from_fence(
        Fence(-10.0, north_max, -20.0, 30.0, 30.0),
        source=source,
        min_altitude_m=2.0,
        margin_m=1.0,
        approach_distance_m=3.0,
        recovery_polygon_ned=(
            (-5.0, -15.0),
            (recovery_north, -15.0),
            (recovery_north, 15.0),
            (-5.0, 15.0),
        ),
        recovery_min_altitude_m=-1.0,
    )
    problem = config.problems()
    if problem:
        raise ValueError(problem)
    return config


def _orchestrator(scenario: str) -> MissionOrchestrator:
    observation_overrides = {"elevation_m": SIM_SURFACE_ELEVATION_M}
    if scenario == "weather_denial":
        observation_overrides["wind_speed_mps"] = 20.0
        observation_overrides["wind_gust_mps"] = 25.0
    observation = simulation_observation(**observation_overrides)
    band = AltitudeBand(AltitudeFrame.LOCAL, 1000.0, 1100.0) if scenario == "altitude_denial" else AltitudeBand(
        AltitudeFrame.LOCAL, 12.0, 18.0
    )
    if scenario == "airspace_denial":
        volume = AirspaceVolume("P-SITL", AirspaceKind.PROHIBITED, 46.0, 48.0, -123.0, -121.0, 0.0, 20_000.0)
        airspace = simulation_authorizer(simulation_dataset(volumes=(volume,)))
    else:
        airspace = simulation_authorizer()
    if scenario == "traffic_conflict":
        track = track_at_ned(30.0, 10.0, 15.0, ref_alt_m=SIM_SURFACE_ELEVATION_M)
        traffic = simulation_traffic_authorizer(simulation_snapshot(tracks=(track,)))
    else:
        traffic = simulation_traffic_authorizer()
    if scenario == "clearance_expired":
        comms = simulation_comms_authorizer(clearance=simulation_clearance(expires_at_s=1.0))
    else:
        comms = simulation_comms_authorizer()
    faults = None
    weather_provider = StaticWeatherProvider(observation)
    if scenario == "weather_during_dispense":
        faults = MissionFaultPlan((FaultSpec("wx-mid-dispense", WEATHER_FAULT, at_state="dispensing", duration_s=0.0),))
        weather_provider = WeatherFaultSwitch(weather_provider, faults)
    if scenario == "airspace_during_dispense":
        faults = MissionFaultPlan((FaultSpec("air-mid-dispense", AIRSPACE_FAULT, at_state="dispensing", duration_s=0.0),))
        airspace = AirspaceAuthorizer(AirspaceFaultSwitch(airspace.provider, faults), airspace.policy)
    supervisor = SeedingSupervisor(
        SeedingMission(_profile(scenario)),
        AltitudeMonitor(max_age_s=1.0),
        band,
        WeatherAuthorizer(weather_provider, simulation_policy()),
        airspace,
        traffic,
        comms,
    )
    # Staging is 18 m above the planned landing site (the recovery area's centre, see
    # _profile), so the pad marker is near the middle of the downward camera's view.
    # The marker coordinate is not a setpoint.
    staging = StagingConditions(
        max_horizontal_m=5.0,
        min_altitude_m=16.0,
        max_altitude_m=20.0,
        max_horizontal_speed_mps=1.5,
        staging_altitude_m=18.0,
    )
    return MissionOrchestrator(
        supervisor,
        # Marker 4 is ~15 m from the staging point. An uncapped approach crossed that at
        # cruise speed and tilted the marker out of the downward camera (marker_stale).
        PrecisionLand(PrecisionLandParams(approach_max_speed_mps=2.0)),
        OrchestratorConfig(
            require_weather=True,
            landing_timeout_s=150.0,
            staging=staging,
            geofence=_sitl_geofence(tight=scenario == "geofence_boundary"),
        ),
        faults=faults,
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


def _see_marker(frame, now_s: float, pose: Pose, fence: GeofenceConfig | None,
                locked_id: int | None = None) -> tuple[CameraTag, int] | None:
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    intrinsics = camera_matrix(SEARCH_CAMERA.width_px, SEARCH_CAMERA.height_px, SEARCH_CAMERA.horizontal_fov_rad)
    poses = estimate_marker_poses(gray, intrinsics, MARKER_M)
    if not poses:
        return None
    center_x = SEARCH_CAMERA.width_px / 2.0
    center_y = SEARCH_CAMERA.height_px / 2.0
    inside: dict[int, bool] | None = None
    polygon = None if fence is None else (fence.recovery_polygon_ned or fence.polygon_ned)
    if polygon is not None and pose.position_ned is not None and pose.q_wxyz is not None:
        snap = VehicleSnapshot(
            position_ned=pose.position_ned,
            velocity_ned=pose.velocity_ned,
            attitude_wxyz=pose.q_wxyz,
            landed=False,
            time_s=now_s,
        )
        inside = {}
        for item in poses:
            tag = CameraTag(item.position_xyz, item.orientation_xyzw, now_s)
            ned, _yaw = tag_to_ned(tag, snap)
            inside[item.marker_id] = _contains(polygon, ned[0], ned[1])
    chosen = track_landing_marker(poses, center_x, center_y, inside, locked_id)
    if chosen is None:
        return None
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
        position_uncertainty_m=1.0,
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


def _blank_report(started: str, scenario: str) -> dict:
    return {
        "mission_id": "integrated-seeding-sitl",
        "scenario_id": scenario,
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
        "simulation_seed": 0,
        "truth_used_for_guidance": False,
        "validation": "PX4 SITL attempted",
        "physical_flight_validated": False,
    }


def _write(path: Path, report: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(report, indent=2) + "\n")


async def _record(recorder: SeedingTraceRecorder, pose: "Pose", live: dict, stop: asyncio.Event) -> None:
    """Sample PX4's estimate at the recorder's rate, tagged with the orchestrator's live state."""
    while not stop.is_set():
        if pose.position_ned is not None and pose.q_wxyz is not None:
            recorder.sample(time.monotonic(), pose.position_ned, pose.q_wxyz, live["state"], live["owner"], live["dispense"])
        await asyncio.sleep(0.02)


async def fly(report_path: Path, timeout_s: float, scenario: str, trace_path: Path | None = None) -> dict:
    started = datetime.now(timezone.utc).isoformat()
    report = _blank_report(started, scenario)
    feed = CameraFeed()
    pose, stop = Pose(), asyncio.Event()
    drone = System()
    follower = None
    orch = _orchestrator(scenario)
    marker_id = None
    frames = 0
    detections = 0
    last_frame = 0
    reached_release = False
    recorder = SeedingTraceRecorder(rate_hz=10.0)
    live = {"state": "takeoff", "owner": "px4", "dispense": False}
    record_stop = asyncio.Event()
    recorder_task = None
    mission_t0 = None
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
        recorder_task = asyncio.create_task(_record(recorder, pose, live, record_stop))
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
                    # Lock onto the first marker acquired: from the staging height more than
                    # one pad is in view, and re-picking each frame made the target jump.
                    seen = _see_marker(frame, now, pose, orch.config.geofence, locked_id=marker_id)
                    if seen is not None:
                        tag, marker_id = seen
                        detections += 1
                        recorder.detection(time.monotonic(), marker_id)
                if scenario == "marker_loss" and orch.lander.state is LandState.DESCEND:
                    tag = None
            vehicle = _vehicle(pose, now, dt, landed, tag)
            if vehicle is None:
                continue
            output = orch.step(vehicle)
            live.update(state=output.state.value, owner=output.owner.value, dispense=bool(output.dispense))
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
                live.update(owner="px4_land")
                try:
                    await drone.offboard.stop()
                except OffboardError:
                    pass
                try:
                    await asyncio.wait_for(drone.action.land(), timeout=20.0)
                except Exception as exc:
                    report["land_command_error"] = str(exc)
                land_deadline = time.monotonic() + 20.0
                while pose.in_air and time.monotonic() < land_deadline:
                    await asyncio.sleep(0.2)
                if not pose.in_air and orch.state is MissionState.PRECISION_LANDING:
                    settled = _vehicle(pose, time.monotonic() - mission_t0, LOOP_S, True, None)
                    if settled is not None:
                        orch.step(settled)
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
            try:
                await asyncio.wait_for(drone.action.land(), timeout=20.0)
            except Exception as exc:
                report["land_command_error"] = str(exc)
            land_deadline = time.monotonic() + 20.0
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
        try:
            evidence = orch.evidence()
            report["orchestrator_evidence"] = evidence
            report.update(_score(pose.position_ned, marker_id, evidence, reached_release, frames, detections))
        except Exception as inner:
            report["evidence_error"] = str(inner)
    finally:
        report["end_time"] = report["end_time"] or datetime.now(timezone.utc).isoformat()
        _write(report_path, report)
        record_stop.set()
        if recorder_task is not None:
            await asyncio.wait_for(recorder_task, timeout=2.0)
        if trace_path is not None and mission_t0 is not None:
            camera = {"width_px": SEARCH_CAMERA.width_px, "height_px": SEARCH_CAMERA.height_px,
                      "hfov_rad": SEARCH_CAMERA.horizontal_fov_rad}
            trace = seeding_trace.build(report, recorder, mission_t0, release_ned=_profile(scenario).release_ned,
                                        expected_result=EXPECTED_RESULT[scenario], camera=camera)
            trace_path.parent.mkdir(parents=True, exist_ok=True)
            trace_path.write_text(json.dumps(trace, separators=(",", ":")) + "\n")
            print(f"trace: {trace_path} ({len(trace['frames'])} poses)", flush=True)
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
    parser.add_argument("--scenario", choices=SCENARIOS, default="nominal")
    parser.add_argument("--trace", type=Path, default=None, help="also write the browser replay trace here")
    args = parser.parse_args()
    expected = EXPECTED_RESULT[args.scenario]
    try:
        report = asyncio.run(fly(args.report, args.timeout, args.scenario, args.trace))
    except Exception:
        return 1
    return 0 if report.get("mission_result") == expected else 2


if __name__ == "__main__":
    raise SystemExit(main())
