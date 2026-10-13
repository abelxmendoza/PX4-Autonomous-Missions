"""The seeding dispenser opens only while the named altitude frame authorizes it."""

from __future__ import annotations

import math

import pytest

from px4_offboard.altitude_reference import (
    AltitudeBand,
    AltitudeFrame,
    AltitudeMonitor,
    Px4AltitudeSnapshot,
)
from px4_offboard.seeding_mission import (
    CommandMode,
    SeedingMission,
    SeedingPhase,
    sim_profile,
)
from px4_offboard.airspace import simulation_authorizer
from px4_offboard.seeding_supervisor import SeedingSupervisor
from px4_offboard.weather import WeatherAuthorizer, simulation_observation, simulation_policy
from px4_offboard.weather_provider import StaticWeatherProvider


def _supervisor(frame, floor, ceiling, tolerance=30.0):
    weather = WeatherAuthorizer(StaticWeatherProvider(simulation_observation()), simulation_policy())
    return SeedingSupervisor(
        SeedingMission(sim_profile()),
        AltitudeMonitor(max_age_s=0.5),
        AltitudeBand(frame, floor, ceiling, tolerance),
        weather,
        simulation_authorizer(),
    )


def _snapshot(time_s, position, **overrides):
    values = dict(
        time_s=time_s,
        baro_alt_m=-position[2],
        local_z_m=position[2],
        local_vz_mps=0.0,
        z_valid=True,
        vz_valid=True,
        ref_alt_m=0.0,
        baro_fault=False,
    )
    values.update(overrides)
    return Px4AltitudeSnapshot(**values)


def _toward(position, target, speed, dt):
    dist = math.dist(position, target)
    if dist <= speed * dt:
        return [target[0], target[1], target[2]]
    scale = speed * dt / dist
    return [position[i] + (target[i] - position[i]) * scale for i in range(3)]


def test_a_matching_local_reference_lets_the_sortie_finish():
    profile = sim_profile()
    supervisor = _supervisor(AltitudeFrame.LOCAL, 12.0, 18.0)
    position = list(profile.home_ned)
    landed = True
    time_s = 0.0
    dispense_times = []
    for _ in range(600):
        command = supervisor.step(
            tuple(position),
            time_s,
            _snapshot(time_s, position),
            landed=landed,
            dt_s=0.1,
        )
        if command.dispense:
            dispense_times.append(time_s)
            assert supervisor.decisions[-1].authorized is True
            assert supervisor.decisions[-1].frame == "local"
        if command.phase in (SeedingPhase.COMPLETE, SeedingPhase.ABORT):
            break
        landed = False
        if command.mode is CommandMode.LAND:
            position[2] = min(0.0, position[2] + 0.5)
            if position[2] >= -0.05:
                position[2] = 0.0
                landed = True
        else:
            position[:] = _toward(position, command.position_ned, 5.0, 0.1)
        time_s = round(time_s + 0.1, 10)
    else:
        raise AssertionError(supervisor.evidence()["mission"])

    evidence = supervisor.evidence()
    assert evidence["mission"]["phase"] == "complete"
    assert evidence["mission"]["violations"] == []
    assert evidence["mission"]["amount_kg"] == pytest.approx(profile.rate_kg_s * profile.dispense_s)
    assert dispense_times
    authorized_at = {item["time_s"] for item in evidence["decisions"] if item["authorized"]}
    assert set(dispense_times) <= authorized_at


def test_estimator_altitude_outside_the_band_aborts_before_any_release():
    profile = sim_profile()
    supervisor = _supervisor(AltitudeFrame.LOCAL, 12.0, 18.0)
    release = profile.release_ned
    # The geometric position is the release point, 15 m up. The estimator says 10 m.
    low = _snapshot(0.0, release, local_z_m=-10.0, baro_alt_m=10.0)
    supervisor.step(release, 0.0, low, dt_s=0.1)
    command = supervisor.step(
        release,
        profile.hold_s,
        _snapshot(profile.hold_s, release, local_z_m=-10.0, baro_alt_m=10.0),
        dt_s=0.1,
    )

    assert command.dispense is False
    assert command.phase is SeedingPhase.RETURN
    assert supervisor.mission.reason == "local altitude 10.0 m is below the 12.0 m floor"
    assert supervisor.mission.samples == ()
    evidence = supervisor.evidence()
    assert evidence["decisions"][-1]["authorized"] is False
    assert evidence["weather_decisions"][-1]["authorized"] is True
    assert evidence["weather_decisions"][-1]["gates_authorized"] is False
    assert supervisor.evidence()["mission"]["sample_count"] == 0


def test_a_barometer_fault_during_release_closes_the_valve_and_is_recorded():
    profile = sim_profile()
    supervisor = _supervisor(AltitudeFrame.BAROMETRIC, 12.0, 18.0)
    release = profile.release_ned
    opened = supervisor.step(release, 0.0, _snapshot(0.0, release), dt_s=0.1)
    assert opened.phase is SeedingPhase.HOLD
    command = supervisor.step(release, profile.hold_s, _snapshot(profile.hold_s, release), dt_s=0.1)
    assert command.dispense is True
    assert len(supervisor.mission.samples) == 1

    fault = _snapshot(profile.hold_s + 0.1, release, baro_fault=True, baro_alt_m=None)
    command = supervisor.step(release, profile.hold_s + 0.1, fault, dt_s=0.1)
    assert command.dispense is False
    assert command.phase is SeedingPhase.RETURN
    assert supervisor.mission.reason == "barometer fault"
    assert len(supervisor.mission.samples) == 1

    done = supervisor.step(
        profile.home_ned,
        profile.hold_s + 0.2,
        _snapshot(profile.hold_s + 0.2, profile.home_ned, baro_fault=True, baro_alt_m=None),
        landed=True,
        dt_s=0.1,
    )
    evidence = supervisor.evidence()
    assert done.phase is SeedingPhase.ABORT
    assert evidence["mission"]["phase"] == "abort"
    assert evidence["mission"]["reason"] == "barometer fault"
    assert evidence["mission"]["violations"] == []
    assert evidence["decisions"][-2]["reason"] == "barometer fault"
    assert evidence["decisions"][-2]["authorized"] is False
    assert [sample.time_s for sample in supervisor.mission.samples] == [profile.hold_s]


def test_evidence_for_the_same_inputs_matches():
    profile = sim_profile()
    release = profile.release_ned

    def run():
        supervisor = _supervisor(AltitudeFrame.BAROMETRIC, 12.0, 18.0)
        supervisor.step(release, 0.0, _snapshot(0.0, release), dt_s=0.1)
        supervisor.step(release, 2.0, _snapshot(2.0, release), dt_s=0.1)
        supervisor.step(release, 2.1, _snapshot(2.1, release, baro_fault=True, baro_alt_m=None), dt_s=0.1)
        return supervisor.evidence()

    assert run() == run()
