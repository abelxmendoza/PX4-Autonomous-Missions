"""Integrated seeding sortie: one setpoint owner, then precision landing."""

from __future__ import annotations

import math

import pytest

from px4_offboard.altitude_reference import (
    AltitudeBand,
    AltitudeFrame,
    AltitudeMonitor,
    Px4AltitudeSnapshot,
)
from px4_offboard.mission_orchestrator import (
    ALLOWED,
    MissionOrchestrator,
    MissionState,
    OrchestratorConfig,
    Owner,
    StagingConditions,
    VehicleInput,
)
from px4_offboard.precision_land import CameraTag, PrecisionLand, PrecisionLandParams
from px4_offboard.seeding_mission import SeedingMission, sim_profile
from px4_offboard.seeding_supervisor import SeedingSupervisor
from px4_offboard.weather import Reason, WeatherAuthorizer, simulation_observation, simulation_policy
from px4_offboard.weather_provider import StaticWeatherProvider


def _supervisor(profile=None, observation=None):
    profile = profile or sim_profile()
    weather = WeatherAuthorizer(
        StaticWeatherProvider(simulation_observation() if observation is None else observation),
        simulation_policy(),
    )
    return SeedingSupervisor(
        SeedingMission(profile),
        AltitudeMonitor(max_age_s=1.0),
        AltitudeBand(AltitudeFrame.LOCAL, 12.0, 18.0),
        weather,
    )


def _snapshot(time_s, position, **overrides):
    down = 0.0 if position is None else position[2]
    values = dict(
        time_s=time_s,
        baro_alt_m=-down,
        local_z_m=down,
        local_vz_mps=0.0,
        z_valid=True,
        vz_valid=True,
        ref_alt_m=0.0,
        baro_fault=False,
    )
    values.update(overrides)
    return Px4AltitudeSnapshot(**values)


def _vehicle(time_s, position, velocity=(0.0, 0.0, 0.0), *, landed=False, armed=True,
             offboard=True, tag=None, dt=0.1, attitude=(1.0, 0.0, 0.0, 0.0), altitude=None):
    return VehicleInput(
        time_s=time_s,
        position_ned=None if position is None else tuple(position),
        velocity_ned=tuple(velocity),
        attitude_wxyz=attitude,
        landed=landed,
        armed=armed,
        offboard=offboard,
        altitude=altitude or _snapshot(time_s, position or (0.0, 0.0, 0.0)),
        tag=tag,
        dt_s=dt,
    )


def _orch(observation=None, **config):
    settings = {"require_weather": True}
    settings.update(config)
    return MissionOrchestrator(
        _supervisor(observation=observation),
        PrecisionLand(PrecisionLandParams(delta_velocity=0.5)),
        OrchestratorConfig(**settings),
    )


def _toward(position, target, speed, dt):
    dist = math.dist(position, target)
    if dist <= speed * dt or dist == 0.0:
        return [target[0], target[1], target[2]], (0.0, 0.0, 0.0)
    scale = speed * dt / dist
    new = [position[i] + (target[i] - position[i]) * scale for i in range(3)]
    vel = tuple((new[i] - position[i]) / dt for i in range(3))
    return new, vel


def _tag_below(position, time_s):
    # Straight down the lens. Camera sits 5 cm below the body origin.
    return CameraTag((0.0, 0.0, max(0.2, -position[2] - 0.05)), time_s=time_s)


def _fly(orch=None, speed=8.0, dt=0.1, ticks=800):
    orch = orch or _orch()
    profile = orch.mission.profile
    position = list(profile.home_ned)
    velocity = (0.0, 0.0, 0.0)
    landed = False
    orch.start(0.0)
    outputs = []
    time_s = 0.0
    for _ in range(ticks):
        tag = _tag_below(position, time_s) if orch.state is MissionState.PRECISION_LANDING else None
        output = orch.step(_vehicle(
            time_s, position, velocity, landed=landed, tag=tag, dt=dt,
        ))
        outputs.append(output)
        if output.state in (MissionState.LANDED, MissionState.ABORTED, MissionState.FAILED):
            return orch, outputs
        landed = False
        if output.use_position and output.position_ned is not None:
            position, velocity = _toward(position, output.position_ned, speed, dt)
            if math.dist(position, output.position_ned) < 0.05:
                velocity = (0.0, 0.0, 0.0)
        elif output.velocity_ned is not None and all(math.isfinite(v) for v in output.velocity_ned):
            velocity = output.velocity_ned
            position = [position[i] + velocity[i] * dt for i in range(3)]
        if position[2] >= -0.05:
            position[2] = 0.0
            landed = True
            velocity = (0.0, 0.0, 0.0)
        time_s += dt
    raise AssertionError(f"did not finish: {orch.state} {orch.evidence()['mission']}")


def test_valid_transitions_complete_a_sortie_and_invalid_ones_do_not():
    assert (MissionState.IDLE, MissionState.PREFLIGHT) in ALLOWED
    assert (MissionState.RETURN_TO_RECOVERY, MissionState.PRECISION_LANDING) in ALLOWED
    assert (MissionState.LANDED, MissionState.TAKEOFF) not in ALLOWED
    assert (MissionState.DISPENSING, MissionState.PRECISION_LANDING) not in ALLOWED

    orch, outputs = _fly()
    assert orch.state is MissionState.LANDED
    assert orch.evidence()["mission_result"] == "completed"
    names = [item["destination"] for item in orch.transitions]
    assert names[:6] == [
        "preflight",
        "takeoff",
        "transit_to_release",
        "hold_for_release",
        "dispensing",
        "return_to_recovery",
    ]
    assert "precision_landing" in names and names[-1] == "landed"
    orch.start(10_000.0)
    assert orch.state is MissionState.LANDED
    assert orch.faults[-1]["code"] == "invalid_transition"


def test_duplicate_enable_does_not_restart_and_repeats_do_not_duplicate_events():
    orch = _orch()
    orch.start(0.0)
    orch.start(1.0)
    assert orch.state is MissionState.PREFLIGHT
    assert [item["destination"] for item in orch.transitions] == ["preflight"]
    assert orch.faults[-1]["code"] == "duplicate_enable"

    position = (0.0, 0.0, -1.0)
    first = orch.step(_vehicle(1.0, position, dt=0.1))
    before = len(orch.transitions)
    samples = len(orch.mission.samples)
    again = orch.step(_vehicle(1.0, position, dt=0.0))
    assert again.state is first.state
    assert len(orch.transitions) == before
    assert len(orch.mission.samples) == samples


def test_one_owner_and_the_handoff_keeps_a_heartbeat():
    orch, outputs = _fly()
    owners = [item["owner"] for item in orch.ownership]
    assert owners.count("precision_land") >= 1
    assert "seeding" in owners
    # A competing publisher is refused and does not become the owner.
    assert orch.submit("precision_land", 1.0) is False or orch.owner is Owner.NONE
    refused = orch.submit("seeding", 2.0)
    assert refused is False
    assert orch.faults[-1]["code"] == "ownership_conflict"

    heartbeats = [out.heartbeat for out in outputs if out.state not in (MissionState.LANDED, MissionState.FAILED)]
    assert heartbeats and all(heartbeats)
    handoff = next(i for i, out in enumerate(outputs) if out.owner is Owner.PRECISION_LAND)
    assert outputs[handoff - 1].heartbeat is True
    assert outputs[handoff].heartbeat is True
    assert outputs[handoff - 1].owner is not Owner.PRECISION_LAND


def test_release_finishes_and_the_dispenser_is_closed_before_return():
    orch, _outputs = _fly()
    returned = next(item for item in orch.transitions if item["destination"] == "return_to_recovery")
    assert returned["context"]["dispense"] is False
    assert returned["reason"] == "release complete"
    assert orch.mission.report()["release_s"] == pytest.approx(sim_profile().dispense_s)
    assert orch.evidence()["dispenser_closed"] is True
    assert returned["time_s"] < next(
        item["time_s"] for item in orch.transitions if item["destination"] == "precision_landing"
    )


def test_weather_and_altitude_denials_cannot_be_bypassed():
    windy = simulation_observation(wind_speed_mps=13.0, wind_gust_mps=14.0)
    weather = _orch(observation=windy)
    weather.start(0.0)
    _drive_to_release(weather)
    assert weather.mission.samples == ()
    assert weather.mission.reason == Reason.WIND_ABOVE_LIMIT
    assert weather.evidence()["weather_evidence"]["consulted"] is True
    assert weather.evidence()["weather_evidence"]["authorized"] is False
    assert weather.evidence()["weather_evidence"]["provenance"] == "simulated"
    assert weather.state is MissionState.RETURN_TO_RECOVERY
    held = len(weather.mission.samples)
    for k in range(5):
        weather.step(_vehicle(40.0 + k * 0.1, weather.mission.profile.release_ned, dt=0.1))
    assert len(weather.mission.samples) == held
    assert weather.evidence()["mission_result"] != "completed"

    disabled = _orch(require_weather=False)
    disabled.start(0.0)
    assert disabled.state is MissionState.FAILED
    assert disabled.transitions[-1]["reason"] == "invalid_configuration"
    assert disabled.mission.samples == ()

    altitude = _orch()
    altitude.start(0.0)
    _drive_to_release(altitude)
    assert altitude.state is MissionState.DISPENSING
    assert altitude.mission.samples
    frozen = len(altitude.mission.samples)
    low = altitude.mission.profile.release_ned
    denied = altitude.step(_vehicle(
        50.0, low, dt=0.1,
        altitude=_snapshot(50.0, low, local_z_m=-10.0, baro_alt_m=10.0),
    ))
    assert denied.dispense is False
    assert "below" in altitude.mission.reason
    assert len(altitude.mission.samples) == frozen
    altitude.step(_vehicle(51.0, low, dt=0.1))
    assert len(altitude.mission.samples) == frozen


def test_dispenser_closure_failure_blocks_the_return():
    orch = _orch(dispenser_ack=False)
    orch.start(0.0)
    time_s = _drive_to_release(orch)
    assert orch.state is MissionState.DISPENSING
    out = None
    for k in range(80):
        position = orch.mission.profile.release_ned
        time_s += 0.1
        out = orch.step(_vehicle(time_s, position, dt=0.1))
        if out.state is MissionState.FAILED:
            break
    assert out is not None and out.state is MissionState.FAILED
    assert any(item["code"] == "dispenser_closure_failed" for item in orch.faults)
    assert orch.evidence()["dispenser_closed"] is False
    assert orch.state is not MissionState.RETURN_TO_RECOVERY


def test_precision_landing_waits_for_staging_and_marker_loss_holds():
    orch = _orch()
    orch.start(0.0)
    _drive_until(orch, MissionState.RETURN_TO_RECOVERY)
    # Still at the release point: too far, too high, not a reason to land.
    far = orch.step(_vehicle(40.0, orch.mission.profile.release_ned, velocity=(2.0, 0.0, 0.0), dt=0.1))
    assert far.state is MissionState.RETURN_TO_RECOVERY
    assert far.owner is Owner.SEEDING
    home = orch.mission.profile.home_ned
    staged = (home[0], home[1], -orch.config.staging.staging_altitude_m)
    ready = orch.step(_vehicle(41.0, staged, velocity=(0.0, 0.0, 0.0), dt=0.1, tag=_tag_below(staged, 41.0)))
    assert ready.state is MissionState.PRECISION_LANDING
    assert ready.owner is Owner.PRECISION_LAND
    assert ready.heartbeat is True

    # Lose the marker past the 5 s timeout.
    lost = None
    for k in range(1, 80):
        lost = orch.step(_vehicle(41.0 + k * 0.1, staged, velocity=(0.0, 0.0, 0.0), dt=0.1, tag=None))
        if lost.state is MissionState.FAILED:
            break
    assert lost is not None and lost.state is MissionState.FAILED
    assert lost.use_position is True
    assert lost.velocity_ned is not None and all(math.isnan(v) for v in lost.velocity_ned)
    assert any(item["code"] == "marker_lost" for item in orch.faults)
    # A new detection does not resume the descent.
    resumed = orch.step(_vehicle(60.0, staged, dt=0.1, tag=_tag_below(staged, 60.0)))
    assert resumed.state is MissionState.FAILED
    assert resumed.use_position is True
    assert all(math.isnan(v) for v in resumed.velocity_ned)


def test_landing_timeout_missing_position_and_disarm():
    timeout = _orch(landing_timeout_s=1.0)
    timeout.start(0.0)
    _drive_until(timeout, MissionState.RETURN_TO_RECOVERY)
    home = timeout.mission.profile.home_ned
    staged = (home[0], home[1], -6.0)
    timeout.step(_vehicle(10.0, staged, dt=0.1))
    assert timeout.state is MissionState.PRECISION_LANDING
    late = timeout.step(_vehicle(12.0, staged, dt=0.1))
    assert late.state is MissionState.FAILED
    assert any(item["code"] == "landing_timeout" for item in timeout.faults)
    assert late.use_position is True

    missing = _orch()
    missing.start(0.0)
    missing.step(_vehicle(0.0, (0.0, 0.0, 0.0), dt=0.1))
    assert missing.state is MissionState.TAKEOFF
    blind = missing.step(_vehicle(0.2, None, dt=0.1))
    assert blind.state is MissionState.FAILED
    assert blind.request_nav_land is True
    assert blind.position_ned is None
    assert blind.heartbeat is False
    assert any(item["code"] == "position_unavailable" for item in missing.faults)

    disarmed = _orch()
    disarmed.start(0.0)
    disarmed.step(_vehicle(0.0, (0.0, 0.0, -3.0), dt=0.1))
    stopped = disarmed.step(_vehicle(0.2, (0.0, 0.0, -3.0), armed=False, dt=0.1))
    assert stopped.state is MissionState.FAILED
    assert stopped.owner is Owner.NONE
    assert any(item["code"] == "unexpected_disarm" for item in disarmed.faults)


def test_offboard_loss_and_stream_gap_are_recorded_without_dropping_the_setpoint():
    orch = _orch()
    orch.start(0.0)
    first = orch.step(_vehicle(0.0, (0.0, 0.0, -3.0), dt=0.1))
    assert first.heartbeat is True
    lost = orch.step(_vehicle(0.2, (0.0, 0.0, -3.0), offboard=False, dt=0.2))
    assert lost.heartbeat is True
    assert lost.position_ned is not None
    assert any(item["code"] == "offboard_lost" for item in orch.faults)
    gapped = orch.step(_vehicle(2.0, (0.0, 0.0, -3.0), dt=1.8))
    assert gapped.heartbeat is True
    assert any(item["code"] == "offboard_stream_gap" for item in orch.faults)


def test_invalid_configuration_fails_closed():
    orch = MissionOrchestrator(
        _supervisor(),
        config=OrchestratorConfig(
            require_weather=True,
            staging=StagingConditions(min_altitude_m=10.0, max_altitude_m=4.0, staging_altitude_m=6.0),
        ),
    )
    orch.start(0.0)
    assert orch.state is MissionState.FAILED
    assert orch.transitions[-1]["reason"] == "invalid_configuration"
    assert orch.evidence()["mission_result"] == "failed"


def test_evidence_is_deterministic():
    def run():
        orch, _ = _fly()
        evidence = orch.evidence()
        evidence.pop("precision_landing_evidence")
        return evidence

    assert run() == run()


def _drive_to_release(orch: MissionOrchestrator) -> float:
    """Park the kinematic vehicle on the release point and step into DISPENSING or an abort."""
    profile = orch.mission.profile
    position = list(profile.home_ned)
    velocity = (0.0, 0.0, 0.0)
    time_s = 0.0
    dt = 0.1
    for _ in range(400):
        out = orch.step(_vehicle(time_s, position, velocity, dt=dt))
        if out.state in (MissionState.DISPENSING, MissionState.RETURN_TO_RECOVERY, MissionState.FAILED):
            return time_s
        if out.use_position and out.position_ned is not None:
            position, velocity = _toward(position, out.position_ned, 12.0, dt)
            if math.dist(position, out.position_ned) < 0.05:
                velocity = (0.0, 0.0, 0.0)
        time_s += dt
    raise AssertionError(orch.state)


def _drive_until(orch: MissionOrchestrator, state: MissionState) -> None:
    profile = orch.mission.profile
    position = list(profile.home_ned)
    velocity = (0.0, 0.0, 0.0)
    time_s = 0.0
    dt = 0.1
    for _ in range(600):
        out = orch.step(_vehicle(time_s, position, velocity, dt=dt))
        if out.state is state:
            return
        if out.state is MissionState.FAILED:
            raise AssertionError(orch.faults[-1])
        if out.use_position and out.position_ned is not None:
            position, velocity = _toward(position, out.position_ned, 12.0, dt)
            if math.dist(position, out.position_ned) < 0.05:
                velocity = (0.0, 0.0, 0.0)
        time_s += dt
    raise AssertionError(orch.state)
