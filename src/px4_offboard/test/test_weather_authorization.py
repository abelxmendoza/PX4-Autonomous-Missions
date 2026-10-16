"""Weather can authorize or deny a seeding release. It cannot override altitude."""

from __future__ import annotations

import json
import math

import pytest

from px4_offboard.airspace import simulation_authorizer
from px4_offboard.simulated_comms import simulation_comms_authorizer
from px4_offboard.traffic import simulation_traffic_authorizer
from px4_offboard.altitude_reference import AltitudeBand, AltitudeFrame, AltitudeMonitor, Px4AltitudeSnapshot
from px4_offboard.seeding_mission import SeedingMission, SeedingPhase, sim_profile
from px4_offboard.seeding_supervisor import SeedingSupervisor
from px4_offboard.weather import (
    Applicability,
    ClockDomain,
    Reason,
    Status,
    WeatherAuthorizer,
    WeatherConstraints,
    simulation_constraints,
    simulation_observation,
    simulation_policy,
)
from px4_offboard.weather_provider import StaticWeatherProvider


def _authorizer(observation=None, **policy):
    return WeatherAuthorizer(
        StaticWeatherProvider(simulation_observation() if observation is None else observation),
        simulation_policy(**policy),
    )


def _decide(observation=None, now_s=10.0, release_m=15.0, **policy):
    return _authorizer(observation, **policy).evaluate(now_s, release_m)


def _supervisor(frame, floor, ceiling, observation=None, **policy):
    return SeedingSupervisor(
        SeedingMission(sim_profile()),
        AltitudeMonitor(max_age_s=0.5),
        AltitudeBand(frame, floor, ceiling),
        _authorizer(observation, **policy),
        simulation_authorizer(),
        simulation_traffic_authorizer(),
        simulation_comms_authorizer(),
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


def test_valid_weather_and_altitude_authorize_release():
    profile = sim_profile()
    supervisor = _supervisor(AltitudeFrame.LOCAL, 12.0, 18.0)
    release = profile.release_ned
    supervisor.step(release, 0.0, _snapshot(0.0, release), dt_s=0.1)
    command = supervisor.step(release, profile.hold_s, _snapshot(profile.hold_s, release), dt_s=0.1)
    assert command.dispense is True
    assert command.phase is SeedingPhase.DISPENSE
    record = supervisor.evidence()["weather_decisions"][-1]
    assert record["authorized"] is True
    assert record["gates_authorized"] is True
    assert record["dispenser_open"] is True
    assert record["provenance"] == "simulated"
    assert record["operational_approval"] is False
    assert supervisor.evidence()["operational_approval"] is False


def test_excessive_sustained_wind_denies_release():
    decision = _decide(simulation_observation(wind_speed_mps=13.0, wind_gust_mps=14.0))
    assert decision.authorized is False
    assert decision.reason_code == Reason.WIND_ABOVE_LIMIT
    assert decision.status == Status.VALID.value


def test_excessive_gust_denies_release():
    decision = _decide(simulation_observation(wind_speed_mps=3.0, wind_gust_mps=19.0))
    assert decision.reason_code == Reason.GUST_ABOVE_LIMIT
    assert decision.authorized is False


def test_missing_weather_denies_release():
    authorizer = WeatherAuthorizer(StaticWeatherProvider(None), simulation_policy())
    decision = authorizer.evaluate(10.0, 15.0)
    assert decision.reason_code == Reason.MISSING_OBSERVATION
    assert decision.status == Status.MISSING.value
    assert decision.measurements == {}


def test_stale_weather_denies_release():
    decision = _decide(now_s=10.0, max_age_s=5.0)
    assert decision.reason_code == Reason.STALE_OBSERVATION
    assert decision.status == Status.STALE.value
    assert decision.age_s == pytest.approx(10.0)


def test_a_stale_windy_report_is_rejected_for_age():
    observation = simulation_observation(wind_speed_mps=30.0, wind_gust_mps=35.0)
    decision = _decide(observation, now_s=10.0, max_age_s=5.0)
    assert decision.reason_code == Reason.STALE_OBSERVATION


def test_future_dated_observation_is_rejected():
    observation = simulation_observation(observed_at_s=50.0, received_at_s=50.0)
    decision = _decide(observation, now_s=10.0)
    assert decision.reason_code == Reason.FUTURE_OBSERVATION
    assert decision.status == Status.INVALID.value


def test_invalid_temperature_is_rejected():
    decision = _decide(simulation_observation(temperature_c=200.0))
    assert decision.reason_code == Reason.INVALID_TEMPERATURE
    assert decision.status == Status.INVALID.value


def test_nan_wind_speed_is_rejected_and_stays_out_of_the_record():
    decision = _decide(simulation_observation(wind_speed_mps=math.nan))
    assert decision.reason_code == Reason.INVALID_WIND_SPEED
    assert decision.measurements["wind_speed_mps"] is None
    json.dumps(decision.as_dict(), allow_nan=False)


def test_missing_wind_is_not_treated_as_calm():
    missing = _decide(simulation_observation(wind_speed_mps=None))
    calm = _decide(simulation_observation(wind_speed_mps=0.0, wind_gust_mps=0.0))
    assert missing.reason_code == Reason.MISSING_WIND_SPEED
    assert calm.authorized is True


def test_missing_coordinates_deny_geographic_applicability():
    decision = _decide(simulation_observation(latitude_deg=None))
    assert decision.reason_code == Reason.MISSING_COORDINATES
    assert decision.status == Status.MISSING.value


def test_weather_outside_the_radius_denies_release():
    decision = _decide(simulation_observation(longitude_deg=-100.0))
    assert decision.reason_code == Reason.OUTSIDE_AREA


def test_a_unix_observation_is_not_compared_with_mission_time_implicitly():
    observation = simulation_observation(
        clock=ClockDomain.UNIX,
        observed_at_s=1_700_000_000.0,
        received_at_s=1_700_000_000.0,
    )
    mismatched = _decide(observation, now_s=10.0)
    assert mismatched.reason_code == Reason.CLOCK_DOMAIN_MISMATCH

    mapped = _decide(observation, now_s=10.0, unix_origin_s=1_700_000_000.0)
    assert mapped.authorized is True
    assert mapped.age_s == pytest.approx(10.0)


def test_surface_weather_does_not_authorize_the_15000_ft_release():
    decision = _decide(release_m=4572.0)
    assert decision.reason_code == Reason.SURFACE_OBSERVATION_INSUFFICIENT
    assert decision.applicability == "surface"

    sounding = simulation_observation(
        applicability=Applicability.ALTITUDE,
        altitude_min_m=4_000.0,
        altitude_max_m=5_000.0,
    )
    covered = _decide(sounding, release_m=4572.0)
    assert covered.authorized is True
    assert covered.applicability == "altitude"


def test_provider_failure_has_an_explicit_reason():
    authorizer = WeatherAuthorizer(StaticWeatherProvider(failure="timeout"), simulation_policy())
    decision = authorizer.evaluate(10.0, 15.0)
    assert decision.reason_code == Reason.PROVIDER_FAILURE
    assert decision.authorized is False


def test_optional_pressure_and_humidity_limits():
    constraints = WeatherConstraints(
        max_wind_mps=12.0,
        max_gust_mps=18.0,
        min_temperature_c=-20.0,
        max_temperature_c=40.0,
        min_pressure_pa=90_000.0,
        max_pressure_pa=105_000.0,
        min_humidity_pct=10.0,
        max_humidity_pct=90.0,
    )
    low = simulation_observation(pressure_pa=80_000.0)
    decision = WeatherAuthorizer(
        StaticWeatherProvider(low),
        simulation_policy(constraints=constraints),
    ).evaluate(10.0, 15.0)
    assert decision.reason_code == Reason.PRESSURE_OUT_OF_RANGE

    absent = simulation_observation(pressure_pa=None, relative_humidity_pct=None)
    assert _decide(absent).authorized is True


def test_identical_inputs_produce_identical_evidence():
    def run():
        supervisor = _supervisor(AltitudeFrame.LOCAL, 12.0, 18.0)
        release = sim_profile().release_ned
        supervisor.step(release, 0.0, _snapshot(0.0, release), dt_s=0.1)
        supervisor.step(release, 2.0, _snapshot(2.0, release), dt_s=0.1)
        return supervisor.evidence()

    first = run()
    assert first == run()
    json.dumps(first, allow_nan=False)


def test_altitude_denial_blocks_release_when_weather_is_favorable():
    profile = sim_profile()
    supervisor = _supervisor(AltitudeFrame.LOCAL, 12.0, 18.0)
    release = profile.release_ned
    low = _snapshot(0.0, release, local_z_m=-10.0, baro_alt_m=10.0)
    supervisor.step(release, 0.0, low, dt_s=0.1)
    command = supervisor.step(release, profile.hold_s, _snapshot(profile.hold_s, release, local_z_m=-10.0, baro_alt_m=10.0), dt_s=0.1)
    record = supervisor.evidence()["weather_decisions"][-1]
    assert command.dispense is False
    assert record["authorized"] is True
    assert record["altitude_authorized"] is False
    assert record["gates_authorized"] is False
    assert supervisor.mission.reason == "local altitude 10.0 m is below the 12.0 m floor"
    assert supervisor.mission.samples == ()


def test_weather_denial_blocks_release_when_altitude_is_favorable():
    profile = sim_profile()
    windy = simulation_observation(wind_speed_mps=20.0, wind_gust_mps=25.0)
    supervisor = _supervisor(AltitudeFrame.LOCAL, 12.0, 18.0, windy)
    release = profile.release_ned
    supervisor.step(release, 0.0, _snapshot(0.0, release), dt_s=0.1)
    command = supervisor.step(release, profile.hold_s, _snapshot(profile.hold_s, release), dt_s=0.1)
    record = supervisor.evidence()["weather_decisions"][-1]
    assert command.dispense is False
    assert record["altitude_authorized"] is True
    assert record["authorized"] is False
    assert record["reason_code"] == Reason.WIND_ABOVE_LIMIT
    assert supervisor.mission.reason == Reason.WIND_ABOVE_LIMIT
    assert supervisor.evidence()["decisions"][-1]["authorized"] is True


def test_weather_recovery_does_not_restart_dispensing():
    from px4_offboard.weather_provider import SimulatedWeatherProvider

    profile = sim_profile()
    calm = simulation_observation()
    windy = simulation_observation(wind_speed_mps=20.0, wind_gust_mps=25.0)
    provider = SimulatedWeatherProvider(
        [(0.0, calm), (profile.hold_s + 0.05, windy), (profile.hold_s + 1.0, calm)]
    )
    supervisor = SeedingSupervisor(
        SeedingMission(profile),
        AltitudeMonitor(max_age_s=0.5),
        AltitudeBand(AltitudeFrame.LOCAL, 12.0, 18.0),
        WeatherAuthorizer(provider, simulation_policy()),
        simulation_authorizer(),
        simulation_traffic_authorizer(),
        simulation_comms_authorizer(),
    )
    release = profile.release_ned
    supervisor.step(release, 0.0, _snapshot(0.0, release), dt_s=0.1)
    opened = supervisor.step(release, profile.hold_s, _snapshot(profile.hold_s, release), dt_s=0.1)
    assert opened.dispense is True
    closed = supervisor.step(release, profile.hold_s + 0.1, _snapshot(profile.hold_s + 0.1, release), dt_s=0.1)
    assert closed.dispense is False
    assert len(supervisor.mission.samples) == 1

    recovered = supervisor.step(release, profile.hold_s + 1.0, _snapshot(profile.hold_s + 1.0, release), dt_s=0.1)
    assert recovered.dispense is False
    assert recovered.phase is SeedingPhase.RETURN
    assert len(supervisor.mission.samples) == 1
    assert supervisor.weather_decisions[-1]["authorized"] is True
    assert supervisor.weather_decisions[-1]["dispenser_open"] is False

    done = supervisor.step(
        profile.home_ned,
        profile.hold_s + 1.1,
        _snapshot(profile.hold_s + 1.1, profile.home_ned),
        landed=True,
        dt_s=0.1,
    )
    assert done.phase is SeedingPhase.ABORT
    assert len(supervisor.mission.samples) == 1


def test_step_reads_the_cache_and_does_not_call_fetch():
    class Probe:
        def __init__(self):
            self.fetch_calls = 0
            self.latest_calls = 0
            self._inner = StaticWeatherProvider(simulation_observation())

        def latest(self, now_s):
            self.latest_calls += 1
            return self._inner.latest(now_s)

        def fetch(self):
            self.fetch_calls += 1
            raise AssertionError("network I/O in the supervisor step")

    probe = Probe()
    supervisor = SeedingSupervisor(
        SeedingMission(sim_profile()),
        AltitudeMonitor(),
        AltitudeBand(AltitudeFrame.LOCAL, 12.0, 18.0),
        WeatherAuthorizer(probe, simulation_policy()),
        simulation_authorizer(),
        simulation_traffic_authorizer(),
        simulation_comms_authorizer(),
    )
    release = sim_profile().release_ned
    supervisor.step(release, 0.0, _snapshot(0.0, release), dt_s=0.1)
    assert probe.latest_calls == 1
    assert probe.fetch_calls == 0


def test_simulation_limits_are_the_labeled_stand_in():
    limits = simulation_constraints()
    assert limits.simulation_only is True
    assert limits.max_wind_mps == 12.0
    assert limits.max_gust_mps == 18.0
