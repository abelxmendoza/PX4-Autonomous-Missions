"""A charted volume can deny a seeding release. A clear chart is not a flight approval."""

from __future__ import annotations

import json

from px4_offboard.simulated_comms import simulation_comms_authorizer
from px4_offboard.traffic import simulation_traffic_authorizer
from px4_offboard.airspace import (
    AirspaceAuthorizer,
    AirspaceKind,
    AirspacePolicy,
    AirspaceVolume,
    Reason,
    SimulatedAirspaceProvider,
    StaticAirspaceProvider,
    simulation_authorizer,
    simulation_dataset,
)
from px4_offboard.altitude_reference import (
    AltitudeBand,
    AltitudeFrame,
    AltitudeMonitor,
    Px4AltitudeSnapshot,
)
from px4_offboard.seeding_mission import SeedingMission, SeedingPhase, sim_profile
from px4_offboard.seeding_supervisor import SeedingSupervisor
from px4_offboard.weather import WeatherAuthorizer, simulation_observation, simulation_policy
from px4_offboard.weather_provider import StaticWeatherProvider


def _volume(kind, floor, ceiling, name="AREA", **times) -> AirspaceVolume:
    return AirspaceVolume(
        name,
        kind,
        46.9,
        47.1,
        -122.2,
        -121.8,
        floor,
        ceiling,
        **times,
    )


def _chart(*volumes, **overrides):
    return simulation_dataset(volumes=volumes, **overrides)


def _supervisor(airspace=None, observation=None):
    return SeedingSupervisor(
        SeedingMission(sim_profile()),
        AltitudeMonitor(max_age_s=0.5),
        AltitudeBand(AltitudeFrame.LOCAL, 12.0, 18.0),
        WeatherAuthorizer(StaticWeatherProvider(observation or simulation_observation()), simulation_policy()),
        airspace or simulation_authorizer(),
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


def _release_attempt(supervisor):
    profile = sim_profile()
    release = profile.release_ned
    supervisor.step(release, 0.0, _snapshot(0.0, release), dt_s=0.1)
    return supervisor.step(release, profile.hold_s, _snapshot(profile.hold_s, release), dt_s=0.1)


def test_a_covering_chart_with_no_volume_authorizes_and_is_not_an_approval():
    supervisor = _supervisor()
    command = _release_attempt(supervisor)
    record = supervisor.evidence()["airspace_decisions"][-1]
    assert command.dispense is True
    assert record["authorized"] is True
    assert record["provenance"] == "simulated"
    assert record["operational_approval"] is False
    assert supervisor.evidence()["operational_approval"] is False
    json.dumps(supervisor.evidence(), allow_nan=False)


def test_prohibited_airspace_denies_release_even_with_an_authorization_token():
    volume = _volume(AirspaceKind.PROHIBITED, 0.0, 20_000.0, name="P-1")
    authorizer = simulation_authorizer(_chart(volume), AirspacePolicy(authorization_id="SIM-AUTH-1"))
    decision = authorizer.evaluate(10.0, 30.0, 10.0, 15.0)
    assert decision.reason_code == Reason.INSIDE_PROHIBITED
    assert decision.volume_name == "P-1"
    assert decision.operational_approval is False

    supervisor = _supervisor(simulation_authorizer(_chart(volume)))
    command = _release_attempt(supervisor)
    assert command.dispense is False
    assert supervisor.mission.reason == Reason.INSIDE_PROHIBITED
    assert supervisor.mission.samples == ()


def test_restricted_airspace_and_an_active_tfr_deny_release():
    restricted = simulation_authorizer(_chart(_volume(AirspaceKind.RESTRICTED, 0.0, 100.0)))
    assert restricted.evaluate(10.0, 0.0, 0.0, 15.0).reason_code == Reason.INSIDE_RESTRICTED
    assert restricted.evaluate(10.0, 0.0, 0.0, 4572.0).authorized is True

    active = _volume(AirspaceKind.TFR, 4_000.0, 5_000.0, name="TFR-1", effective_from_s=0.0, effective_until_s=100.0)
    later = _volume(AirspaceKind.TFR, 4_000.0, 5_000.0, effective_from_s=50.0, effective_until_s=80.0)
    assert simulation_authorizer(_chart(active)).evaluate(10.0, 0.0, 0.0, 4572.0).reason_code == Reason.INSIDE_TFR
    assert simulation_authorizer(_chart(later)).evaluate(10.0, 0.0, 0.0, 4572.0).authorized is True


def test_a_tfr_without_a_window_is_not_treated_as_inactive():
    volume = _volume(AirspaceKind.TFR, 0.0, 100.0)
    decision = simulation_authorizer(_chart(volume)).evaluate(10.0, 0.0, 0.0, 15.0)
    assert decision.reason_code == Reason.TFR_WINDOW_MISSING


def test_controlled_airspace_needs_a_recorded_token_and_the_token_is_not_approval():
    volume = _volume(AirspaceKind.CONTROLLED, 0.0, 20_000.0, name="CLASS-D")
    denied = simulation_authorizer(_chart(volume)).evaluate(10.0, 0.0, 0.0, 15.0)
    assert denied.reason_code == Reason.INSIDE_CONTROLLED
    allowed = simulation_authorizer(_chart(volume), AirspacePolicy(authorization_id="SIM-AUTH-9"))
    decision = allowed.evaluate(10.0, 0.0, 0.0, 15.0)
    assert decision.authorized is True
    assert decision.authorization_id == "SIM-AUTH-9"
    assert decision.operational_approval is False


def test_missing_stale_and_uncovered_charts_do_not_count_as_clear_air():
    assert AirspaceAuthorizer(StaticAirspaceProvider(None)).evaluate(10.0, 0.0, 0.0, 15.0).reason_code == (
        Reason.MISSING_AIRSPACE
    )
    assert AirspaceAuthorizer(StaticAirspaceProvider(failure="timeout")).evaluate(10.0, 0.0, 0.0, 15.0).reason_code == (
        Reason.PROVIDER_FAILURE
    )
    stale = simulation_authorizer(policy=AirspacePolicy(max_age_s=5.0))
    assert stale.evaluate(10.0, 0.0, 0.0, 15.0).reason_code == Reason.STALE_AIRSPACE
    uncovered = simulation_authorizer(_chart(coverage_radius_m=1.0))
    assert uncovered.evaluate(10.0, 5_000.0, 0.0, 15.0).reason_code == Reason.OUTSIDE_CHART_COVERAGE
    no_coverage = simulation_dataset(coverage_radius_m=None)
    assert simulation_authorizer(no_coverage).evaluate(10.0, 0.0, 0.0, 15.0).reason_code == Reason.COVERAGE_MISSING


def test_airspace_failure_during_release_closes_the_valve_and_does_not_resume():
    profile = sim_profile()
    clear = simulation_dataset()
    blocked = simulation_dataset(volumes=(_volume(AirspaceKind.PROHIBITED, 0.0, 100.0, name="P-9"),))
    provider = SimulatedAirspaceProvider([(0.0, clear), (profile.hold_s + 0.05, blocked), (profile.hold_s + 1.0, clear)])
    supervisor = _supervisor(AirspaceAuthorizer(provider))
    release = profile.release_ned
    supervisor.step(release, 0.0, _snapshot(0.0, release), dt_s=0.1)
    opened = supervisor.step(release, profile.hold_s, _snapshot(profile.hold_s, release), dt_s=0.1)
    assert opened.dispense is True
    closed = supervisor.step(release, profile.hold_s + 0.1, _snapshot(profile.hold_s + 0.1, release), dt_s=0.1)
    assert closed.dispense is False
    assert supervisor.mission.reason == Reason.INSIDE_PROHIBITED
    assert len(supervisor.mission.samples) == 1
    again = supervisor.step(release, profile.hold_s + 1.0, _snapshot(profile.hold_s + 1.0, release), dt_s=0.1)
    assert again.phase is SeedingPhase.RETURN
    assert again.dispense is False
    assert len(supervisor.mission.samples) == 1
    assert supervisor.airspace_decisions[-1]["authorized"] is True


def test_altitude_and_weather_denials_outrank_a_clear_chart():
    low = _supervisor()
    profile = sim_profile()
    release = profile.release_ned
    snap = _snapshot(0.0, release, local_z_m=-10.0, baro_alt_m=10.0)
    low.step(release, 0.0, snap, dt_s=0.1)
    command = low.step(release, profile.hold_s, _snapshot(profile.hold_s, release, local_z_m=-10.0, baro_alt_m=10.0), dt_s=0.1)
    assert command.dispense is False
    assert low.mission.reason.startswith("local altitude")
    assert low.airspace_decisions[-1]["authorized"] is True

    windy = simulation_observation(wind_speed_mps=20.0, wind_gust_mps=25.0)
    blocked_weather = _supervisor(observation=windy)
    _release_attempt(blocked_weather)
    assert blocked_weather.mission.reason == "wind_above_limit"
    assert blocked_weather.airspace_decisions[-1]["authorized"] is True
    assert blocked_weather.evidence()["decisions"][-1]["authorized"] is True


def test_identical_airspace_inputs_produce_identical_evidence():
    def run():
        supervisor = _supervisor(simulation_authorizer(_chart(_volume(AirspaceKind.RESTRICTED, 0.0, 100.0))))
        _release_attempt(supervisor)
        return supervisor.evidence()

    assert run() == run()
