"""Geofence geometry, setpoint admission, and mission fault response."""

from __future__ import annotations

import math

import pytest

from px4_offboard.airspace import AirspaceAuthorizer, simulation_authorizer
from px4_offboard.altitude_reference import AltitudeBand, AltitudeFrame, AltitudeMonitor, Px4AltitudeSnapshot
from px4_offboard.mission_faults import (
    AIRSPACE_FAULT,
    ALTITUDE_FAULT,
    DISPENSER_FAULT,
    OFFBOARD_FAULT,
    POSITION_FAULT,
    VISION_FAULT,
    WEATHER_FAULT,
    AirspaceFaultSwitch,
    FaultSpec,
    MissionFaultPlan,
    WeatherFaultSwitch,
)
from px4_offboard.mission_geofence import (
    APPROACHING,
    INSIDE,
    INSUFFICIENT,
    INVALID,
    OUTSIDE,
    GeofenceConfig,
    command_admissible,
    evaluate_vehicle,
)
from px4_offboard.mission_logic import Fence
from px4_offboard.mission_orchestrator import MissionOrchestrator, MissionState, OrchestratorConfig, Owner, VehicleInput
from px4_offboard.precision_land import CameraTag, PrecisionLand, PrecisionLandParams
from px4_offboard.seeding_mission import SeedingMission, sim_profile
from px4_offboard.seeding_supervisor import SeedingSupervisor
from px4_offboard.simulated_comms import simulation_comms_authorizer
from px4_offboard.traffic import simulation_traffic_authorizer
from px4_offboard.weather import WeatherAuthorizer, simulation_observation, simulation_policy
from px4_offboard.weather_provider import StaticWeatherProvider


SQUARE = ((0.0, 0.0), (4.0, 0.0), (4.0, 4.0), (0.0, 4.0))


PX4_LAND_SPEED_MPS = 0.7  # PX4 default MPC_LAND_SPEED


def _config(**overrides) -> GeofenceConfig:
    values = dict(
        source="unit-test",
        polygon_ned=SQUARE,
        altitude_frame=AltitudeFrame.LOCAL,
        min_altitude_m=1.0,
        max_altitude_m=20.0,
        margin_m=0.0,
        approach_distance_m=0.4,
        uncertainty_sigma=1.0,
        prediction_horizon_s=2.0,
        recovery_polygon_ned=((0.0, 0.0), (1.5, 0.0), (1.5, 1.5), (0.0, 1.5)),
        recovery_min_altitude_m=0.0,
    )
    values.update(overrides)
    return GeofenceConfig(**values)


def _eval(config, north, east, altitude=10.0, uncertainty=0.0, velocity=(0.0, 0.0, 0.0), velocity_uncertainty=None, valid=True, frame=None):
    return evaluate_vehicle(
        config,
        north_m=north,
        east_m=east,
        altitude_m=altitude,
        altitude_valid=valid,
        altitude_frame=config.altitude_frame if frame is None else frame,
        position_uncertainty_m=uncertainty,
        velocity_ned=velocity,
        velocity_uncertainty_mps=velocity_uncertainty,
        time_s=1.0,
        mission_state="transit_to_release",
    )


def test_point_inside_outside_and_on_the_boundary():
    config = _config()
    assert _eval(config, 2.0, 2.0).status == INSIDE
    assert _eval(config, 6.0, 2.0).status == OUTSIDE
    assert _eval(config, 6.0, 2.0).reason_code == "outside_horizontal"
    edge = _eval(config, 0.0, 2.0, uncertainty=0.0)
    assert edge.status == INSIDE
    assert edge.reason_code == "on_boundary"


def test_invalid_polygons_and_coordinates_are_rejected():
    bowtie = _config(polygon_ned=((0.0, 0.0), (1.0, 1.0), (0.0, 1.0), (1.0, 0.0)))
    assert "self-intersecting" in bowtie.problems()
    assert _eval(bowtie, 0.2, 0.2).status == INVALID
    assert "nonfinite" in _config(polygon_ned=((0.0, 0.0), (math.nan, 0.0), (1.0, 1.0))).problems()
    assert "geographic" in _config(frame="geographic").problems()
    assert "below minimum" in _config(min_altitude_m=10.0, max_altitude_m=2.0).problems()
    assert "altitude frame" in _config(altitude_frame=None).problems() or _eval(
        _config(), 2.0, 2.0, frame=AltitudeFrame.AGL
    ).reason_code == "altitude_frame_mismatch"


def test_altitude_limits_use_one_frame_and_uncertainty_is_not_a_false_inside():
    config = _config()
    assert _eval(config, 2.0, 2.0, altitude=0.2).reason_code == "below_min_altitude"
    assert _eval(config, 2.0, 2.0, altitude=25.0).reason_code == "above_max_altitude"
    low = _eval(config, 2.0, 2.0, altitude=0.2)
    assert low.frame == "local"
    near = _eval(config, 0.2, 2.0, uncertainty=0.5)
    assert near.status == APPROACHING
    assert near.reason_code == "uncertain_proximity"
    assert near.status != INSIDE
    missing = _eval(config, None, None)
    assert missing.status == INSUFFICIENT
    assert missing.reason_code == "position_invalid"


def test_rectangle_matches_the_existing_fence_box():
    fence = Fence(0.0, 4.0, 0.0, 4.0, 20.0)
    config = GeofenceConfig.from_fence(fence, source="mission_logic.Fence", min_altitude_m=0.0, margin_m=0.0, approach_distance_m=0.0)
    assert config.problems() == ""
    for north, east in ((1.0, 1.0), (5.0, 1.0)):
        legacy = fence.contains((north, east, -10.0))
        decision = _eval(config, north, east, altitude=10.0, uncertainty=0.0)
        assert (decision.status == INSIDE) is legacy


def test_prediction_toward_away_and_uncertain():
    config = _config(approach_distance_m=0.2)
    toward = _eval(config, 3.6, 2.0, velocity=(1.0, 0.0, 0.0))
    assert toward.predicted_reason == "predicted_boundary_violation"
    assert toward.approach_time_s is not None
    away = _eval(config, 3.6, 2.0, velocity=(-1.0, 0.0, 0.0))
    assert away.predicted_reason == "prediction_clear"
    still = _eval(config, 3.7, 2.0, velocity=(0.0, 0.0, 0.0))
    assert still.predicted_reason == "prediction_clear"
    uncertain = _eval(config, 3.2, 2.0, velocity=(0.2, 0.0, 0.0), uncertainty=0.1, velocity_uncertainty=1.0)
    assert uncertain.predicted_reason == "uncertain_predicted_crossing"
    invalid = _eval(config, 2.0, 2.0, velocity=(math.nan, 0.0, 0.0))
    assert invalid.predicted_reason == "velocity_invalid"


def test_setpoint_admission_rejects_unsafe_commands_and_keeps_nan_velocity():
    config = _config(margin_m=0.0, approach_distance_m=0.2)
    vehicle = _eval(config, 2.0, 2.0, uncertainty=0.0)
    assert command_admissible(
        config, vehicle, use_position=True, position_ned=(2.0, 2.0, -10.0), velocity_ned=(math.nan, math.nan, math.nan),
        north_m=2.0, east_m=2.0, altitude_m=10.0,
    ).admissible
    rejected = command_admissible(
        config, vehicle, use_position=True, position_ned=(8.0, 2.0, -10.0), velocity_ned=(math.nan, math.nan, math.nan),
        north_m=2.0, east_m=2.0, altitude_m=10.0,
    )
    assert not rejected.admissible
    assert rejected.reason_code == "setpoint_outside"
    broken = command_admissible(
        config, vehicle, use_position=True, position_ned=(math.nan, 2.0, -10.0), velocity_ned=None,
        north_m=2.0, east_m=2.0, altitude_m=10.0,
    )
    assert broken.reason_code == "setpoint_position_invalid"
    velocity = command_admissible(
        config, vehicle, use_position=False, position_ned=None, velocity_ned=(0.1, 0.0, 0.0),
        north_m=2.0, east_m=2.0, altitude_m=10.0,
    )
    assert velocity.admissible


def test_notched_return_route_is_blocked_and_recovery_altitude_is_local():
    notch = (
        (0.0, -5.0), (15.0, -5.0), (15.0, 5.0), (25.0, 5.0), (25.0, -5.0),
        (40.0, -5.0), (40.0, 20.0), (0.0, 20.0),
    )
    config = _config(polygon_ned=notch, min_altitude_m=4.0, margin_m=0.0, approach_distance_m=0.2, recovery_polygon_ned=((0.0, 0.0), (4.0, 0.0), (4.0, 4.0), (0.0, 4.0)))
    assert config.problems() == ""
    vehicle = _eval(config, 30.0, 2.0, altitude=10.0, uncertainty=0.0)
    blocked = command_admissible(
        config, vehicle, use_position=True, position_ned=(5.0, 2.0, -10.0), velocity_ned=(math.nan,) * 3,
        north_m=30.0, east_m=2.0, altitude_m=10.0,
    )
    assert blocked.reason_code == "return_route_blocked"
    touchdown = _eval(config, 1.0, 1.0, altitude=0.2, uncertainty=0.0)
    assert touchdown.status == INSIDE
    outside_pad = _eval(config, 10.0, 10.0, altitude=0.2, uncertainty=0.0)
    assert outside_pad.reason_code == "below_min_altitude"


def _supervisor(provider=None):
    weather = WeatherAuthorizer(provider or StaticWeatherProvider(simulation_observation()), simulation_policy())
    return SeedingSupervisor(
        SeedingMission(sim_profile()),
        AltitudeMonitor(max_age_s=1.0),
        AltitudeBand(AltitudeFrame.LOCAL, 12.0, 18.0),
        weather,
        simulation_authorizer(),
        simulation_traffic_authorizer(),
        simulation_comms_authorizer(),
    )


def _snapshot(time_s, position, **overrides):
    down = 0.0 if position is None else position[2]
    values = dict(
        time_s=time_s, baro_alt_m=-down, local_z_m=down, local_vz_mps=0.0,
        z_valid=position is not None, vz_valid=True, ref_alt_m=0.0, baro_fault=False,
    )
    values.update(overrides)
    return Px4AltitudeSnapshot(**values)


def _vehicle(time_s, position, velocity=(0.0, 0.0, 0.0), **kwargs):
    altitude = kwargs.pop("altitude", None) or _snapshot(time_s, position if position is not None else (0.0, 0.0, 0.0))
    return VehicleInput(
        time_s=time_s,
        position_ned=None if position is None else tuple(position),
        velocity_ned=tuple(velocity),
        attitude_wxyz=kwargs.pop("attitude", (1.0, 0.0, 0.0, 0.0)),
        landed=kwargs.pop("landed", False),
        armed=kwargs.pop("armed", True),
        offboard=kwargs.pop("offboard", True),
        altitude=altitude,
        tag=kwargs.pop("tag", None),
        dt_s=kwargs.pop("dt", 0.1),
        position_uncertainty_m=kwargs.pop("uncertainty", 0.0),
    )


def _wide_fence():
    return GeofenceConfig.from_fence(
        Fence(-30.0, 60.0, -30.0, 40.0, 40.0),
        source="sortie-test",
        min_altitude_m=0.0,
        margin_m=0.0,
        approach_distance_m=0.5,
        recovery_polygon_ned=((-5.0, -5.0), (20.0, -5.0), (20.0, 10.0), (-5.0, 10.0)),
        recovery_min_altitude_m=0.0,
    )


def _orch(provider=None, fence=None, faults=None, **config):
    settings = {"require_weather": True, "geofence": fence if fence is not None else _wide_fence()}
    settings.update(config)
    return MissionOrchestrator(
        _supervisor(provider),
        PrecisionLand(PrecisionLandParams(delta_velocity=0.5)),
        OrchestratorConfig(**settings),
        faults=faults,
    )


def _toward(position, target, speed, dt):
    dist = math.dist(position, target)
    if dist <= speed * dt or dist == 0.0:
        return [target[0], target[1], target[2]], (0.0, 0.0, 0.0)
    scale = speed * dt / dist
    new = [position[i] + (target[i] - position[i]) * scale for i in range(3)]
    return new, tuple((new[i] - position[i]) / dt for i in range(3))


def _states_seen(orch: MissionOrchestrator) -> set[str]:
    return {item["mission_state"] for item in orch.geofence_decisions}


def test_geofence_is_recorded_through_the_sortie_and_a_marker_outside_recovery_is_rejected():
    orch = _orch()
    profile = orch.mission.profile
    position = list(profile.home_ned)
    velocity = (0.0, 0.0, 0.0)
    orch.start(0.0)
    time_s = 0.0
    dt = 0.1
    landed = False
    for _ in range(900):
        tag = None
        if orch.state is MissionState.PRECISION_LANDING:
            tag = CameraTag((0.0, 0.0, max(0.2, -position[2] - 0.05)), time_s=time_s)
        output = orch.step(_vehicle(time_s, position, velocity, landed=landed, tag=tag, dt=dt, uncertainty=0.0))
        if output.state in (MissionState.LANDED, MissionState.ABORTED, MissionState.FAILED):
            break
        landed = False
        if output.use_position and output.position_ned is not None:
            position, velocity = _toward(position, output.position_ned, 8.0, dt)
        elif output.request_nav_land:
            # PX4 land mode (requested on pad contact): straight down at MPC_LAND_SPEED.
            velocity = (0.0, 0.0, PX4_LAND_SPEED_MPS)
            position = [position[0], position[1], position[2] + PX4_LAND_SPEED_MPS * dt]
        elif output.velocity_ned is not None and all(math.isfinite(v) for v in output.velocity_ned):
            velocity = output.velocity_ned
            position = [position[i] + velocity[i] * dt for i in range(3)]
        if position[2] >= -0.05:
            position[2] = 0.0
            landed = True
            velocity = (0.0, 0.0, 0.0)
        time_s += dt
    seen = _states_seen(orch)
    for name in ("takeoff", "transit_to_release", "hold_for_release", "dispensing", "return_to_recovery", "precision_landing"):
        assert name in seen
    assert orch.state is MissionState.LANDED
    assert orch.evidence()["schema"] == 2
    assert orch.evidence()["geofence_configuration"]["clamp_setpoints"] is False

    tight = _orch(fence=_config(approach_distance_m=0.2, margin_m=0.0, min_altitude_m=0.0, max_altitude_m=30.0))
    tight.start(0.0)
    tight.state = MissionState.PRECISION_LANDING
    home = (1.0, 1.0, -2.0)
    outside = CameraTag((0.0, -8.0, 10.0), time_s=5.0)
    tight.step(_vehicle(5.0, home, tag=outside, dt=0.1, uncertainty=0.0))
    assert any(item["reason_code"] == "marker_outside_recovery" for item in tight.rejected_setpoints)


def test_unsafe_setpoint_is_rejected_without_giving_up_the_only_publisher():
    fence = _config(margin_m=0.0, approach_distance_m=0.2, min_altitude_m=0.0, max_altitude_m=30.0)
    orch = _orch(fence=fence)
    orch.start(0.0)
    output = orch.step(_vehicle(0.0, (2.0, 2.0, -10.0), dt=0.1, uncertainty=0.0))
    assert output.owner in (Owner.SEEDING, Owner.HOLD)
    assert output.position_ned is not None
    assert math.dist(output.position_ned, (2.0, 2.0, -10.0)) < 1.0 or any(
        item["reason_code"] == "setpoint_outside" for item in orch.rejected_setpoints
    )
    assert orch.submit("precision_land", 0.2) is False
    assert orch.faults[-1]["code"] == "ownership_conflict"
    assert orch.evidence()["rejected_setpoints"]


def test_release_faults_close_the_dispenser_and_do_not_restart_it():
    plan = MissionFaultPlan((FaultSpec("wx", WEATHER_FAULT, at_time_s=3.0, duration_s=5.0),))
    provider = WeatherFaultSwitch(StaticWeatherProvider(simulation_observation()), plan)
    orch = _orch(provider=provider, faults=plan, fence=None)
    orch.start(0.0)
    release = orch.mission.profile.release_ned
    for k in range(25):
        orch.step(_vehicle(k * 0.1, release, dt=0.1))
    assert orch.mission.samples
    frozen = len(orch.mission.samples)
    orch.step(_vehicle(3.0, release, dt=0.1))
    assert orch.mission.reason == "provider_failure"
    assert len(orch.mission.samples) == frozen
    orch.step(_vehicle(8.5, release, dt=0.1))
    assert len(orch.mission.samples) == frozen
    assert orch.fault_plan.injections[0]["fault_type"] == WEATHER_FAULT

    air_plan = MissionFaultPlan((FaultSpec("air", AIRSPACE_FAULT, at_time_s=3.0, duration_s=5.0),))
    air = AirspaceFaultSwitch(simulation_authorizer().provider, air_plan)
    air_orch = _orch(
        faults=air_plan,
        fence=None,
        provider=StaticWeatherProvider(simulation_observation()),
    )
    air_orch.supervisor.airspace = AirspaceAuthorizer(air, air_orch.supervisor.airspace.policy)
    air_orch.start(0.0)
    for k in range(25):
        air_orch.step(_vehicle(k * 0.1, release, dt=0.1))
    kept = len(air_orch.mission.samples)
    assert kept > 0
    air_orch.step(_vehicle(3.0, release, dt=0.1))
    assert air_orch.mission.reason == "provider_failure"
    assert len(air_orch.mission.samples) == kept
    air_orch.step(_vehicle(8.5, release, dt=0.1))
    assert len(air_orch.mission.samples) == kept

    alt_plan = MissionFaultPlan((FaultSpec("alt", ALTITUDE_FAULT, at_time_s=3.0, duration_s=2.0),))
    alt = _orch(faults=alt_plan, fence=None)
    alt.start(0.0)
    for k in range(25):
        alt.step(_vehicle(k * 0.1, release, dt=0.1))
    assert alt.mission.samples
    alt.step(_vehicle(3.0, release, dt=0.1))
    assert alt.mission.reason == "estimated altitude invalid"


def test_position_loss_marker_loss_offboard_and_dispenser_faults():
    missing = _orch(faults=MissionFaultPlan((FaultSpec("pos", POSITION_FAULT, at_time_s=0.2, duration_s=1.0),)), fence=_wide_fence())
    missing.start(0.0)
    missing.step(_vehicle(0.0, (0.0, 0.0, -5.0), dt=0.1, uncertainty=0.0))
    blind = missing.step(_vehicle(0.2, (0.0, 0.0, -5.0), dt=0.1, uncertainty=0.0))
    assert blind.request_nav_land is True
    assert all(item["status"] != INSIDE for item in missing.geofence_decisions if item["time_s"] >= 0.2)

    offboard = _orch(faults=MissionFaultPlan((FaultSpec("ob", OFFBOARD_FAULT, at_time_s=0.2, duration_s=1.0),)), fence=None)
    offboard.start(0.0)
    offboard.step(_vehicle(0.0, (0.0, 0.0, -5.0), dt=0.1))
    kept = offboard.step(_vehicle(0.2, (0.0, 0.0, -5.0), dt=0.1))
    assert kept.heartbeat is True
    assert any(item["code"] == "offboard_lost" for item in offboard.faults)

    dispenser = _orch(faults=MissionFaultPlan((FaultSpec("door", DISPENSER_FAULT, at_state="dispensing"),)), fence=None)
    dispenser.start(0.0)
    release = dispenser.mission.profile.release_ned
    for k in range(80):
        out = dispenser.step(_vehicle(k * 0.1, release, dt=0.1))
        if out.state is MissionState.FAILED:
            break
    assert any(item["code"] == "dispenser_closure_failed" for item in dispenser.faults)

    with pytest.raises(ValueError):
        MissionFaultPlan((
            FaultSpec("dup", VISION_FAULT, at_time_s=1.0),
            FaultSpec("dup", VISION_FAULT, at_time_s=2.0),
        ))


def test_campaign_summary_keeps_failed_runs_in_the_rate():
    from px4_offboard.mission_verification import summarize_campaign

    reports = [
        {"scenario_id": "nominal", "mission_result": "completed", "release_sample_count": 10, "touchdown_horizontal_error_m": 0.1, "fault_codes": []},
        {"scenario_id": "nominal", "mission_result": "failed", "release_sample_count": 0, "touchdown_horizontal_error_m": None, "fault_codes": []},
        {
            "scenario_id": "geofence_boundary",
            "mission_result": "simulation_terminated",
            "orchestrator_evidence": {"rejected_setpoints": [{"reason_code": "setpoint_outside"}]},
            "fault_codes": [],
        },
    ]
    summary = summarize_campaign(reports, {"nominal": 5, "marker_loss": 3})
    assert summary["nominal_completion_rate"] == 0.5
    assert summary["nominal_run_count"] == 2
    assert summary["touchdown_horizontal_error_m"]["sample_count"] == 1
    assert summary["touchdown_horizontal_error_m"]["p95"] is None
    assert summary["unsafe_setpoint_rejection_count"] == 1
    assert summary["physical_flight_validated"] is False
    assert summary["runs"][1]["expectation_met"] is False


def test_requirements_do_not_count_a_skip_or_a_failure_as_verified():
    from px4_offboard.mission_traceability import TraceItem, summarize

    items = (
        TraceItem("REQ-GEOFENCE-001", "UNIT_VERIFIED", "test_point_inside_outside_and_on_the_boundary", ""),
        TraceItem("REQ-GEOFENCE-002", "NOT_TESTED", "", "skipped"),
        TraceItem("REQ-GEOFENCE-003", "FAILED", "test_altitude_limits_use_one_frame_and_uncertainty_is_not_a_false_inside", ""),
        TraceItem("REQ-VERIFY-001", "SITL_VERIFIED", "campaign", "evidence/mission_verification/campaign_summary.json"),
    )
    report = summarize(items)
    assert report["requirements"][1]["status"] == "NOT_TESTED"
    assert report["requirements"][2]["status"] == "FAILED"
    verified = [item["id"] for item in report["requirements"] if item["counts_as_verified"]]
    assert verified == ["REQ-GEOFENCE-001", "REQ-VERIFY-001"]
    assert len({item["id"] for item in report["requirements"]}) == 4
