"""Traffic snapshot checks and the release gate. Thresholds are simulation-only."""

from __future__ import annotations

import json
import urllib.request

import pytest

from px4_offboard.airspace import AirspaceKind, AirspaceVolume, simulation_authorizer, simulation_dataset
from px4_offboard.altitude_reference import AltitudeBand, AltitudeFrame, AltitudeMonitor, Px4AltitudeSnapshot
from px4_offboard.simulated_comms import simulation_comms_authorizer
from px4_offboard.seeding_mission import SeedingMission, SeedingPhase, sim_profile
from px4_offboard.seeding_supervisor import SeedingSupervisor
from px4_offboard.traffic import (
    ConflictStatus,
    Reason,
    TrafficAltitudeFrame,
    closest_approach,
    simulation_snapshot,
    simulation_traffic_authorizer,
    simulation_traffic_policy,
    track_at_ned,
)
from px4_offboard.traffic_provider import RecordedTrafficReplayProvider, SimulatedTrafficProvider, StaticTrafficProvider
from px4_offboard.weather import Provenance, WeatherAuthorizer, simulation_observation, simulation_policy
from px4_offboard.weather_provider import StaticWeatherProvider


def _authorizer(snapshot=None, **policy):
    return simulation_traffic_authorizer(
        simulation_snapshot() if snapshot is None else snapshot,
        simulation_traffic_policy(**policy) if policy else None,
    )


def _at_release(authorizer, north=30.0, east=10.0, down=-15.0, now=10.0, velocity=(0.0, 0.0, 0.0)):
    return authorizer.evaluate(now, north, east, down, velocity, 0.0)


def _supervisor(**overrides):
    profile = sim_profile()
    return SeedingSupervisor(
        SeedingMission(profile),
        AltitudeMonitor(max_age_s=0.5),
        AltitudeBand(AltitudeFrame.LOCAL, 12.0, 18.0),
        WeatherAuthorizer(StaticWeatherProvider(overrides.get("observation") or simulation_observation()), simulation_policy()),
        overrides.get("airspace") or simulation_authorizer(),
        overrides.get("traffic") or simulation_traffic_authorizer(),
        overrides.get("comms") or simulation_comms_authorizer(),
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


def _release(supervisor, at=None):
    profile = sim_profile()
    release = at or profile.release_ned
    supervisor.step(release, 0.0, _snapshot(0.0, release), dt_s=0.1)
    return supervisor.step(release, profile.hold_s, _snapshot(profile.hold_s, release), dt_s=0.1)


def test_valid_simulated_snapshot_has_no_reported_conflict():
    decision = _at_release(_authorizer())
    assert decision.authorized is True
    assert decision.conflict_status == ConflictStatus.NO_REPORTED_CONFLICT
    assert decision.surveillance_complete is False
    assert decision.operational_approval is False
    assert decision.provenance == "simulated"


def test_missing_snapshot_denies_release():
    decision = _at_release(simulation_traffic_authorizer_from(StaticTrafficProvider(None)))
    assert decision.reason_code == Reason.TRAFFIC_DATA_MISSING
    assert decision.authorized is False


def test_stale_snapshot_denies_release():
    authorizer = simulation_traffic_authorizer(policy=simulation_traffic_policy(max_age_s=5.0))
    decision = authorizer.evaluate(10.0, 30.0, 10.0, -15.0, (0.0, 0.0, 0.0), 0.0)
    assert decision.reason_code == Reason.TRAFFIC_DATA_STALE


def test_empty_snapshot_inside_coverage_is_not_proof_the_sky_is_empty():
    decision = _at_release(_authorizer(simulation_snapshot(tracks=())))
    assert decision.authorized is True
    assert decision.as_dict()["surveillance_complete"] is False
    assert decision.track_count == 0


def test_empty_snapshot_without_coverage_denies_release():
    snapshot = simulation_snapshot(coverage_radius_m=None)
    decision = _at_release(_authorizer(snapshot))
    assert decision.reason_code == Reason.TRAFFIC_COVERAGE_INSUFFICIENT


def test_invalid_coordinates_are_rejected():
    track = track_at_ned(30.0, 10.0, 15.0, latitude_deg=95.0)
    decision = _at_release(_authorizer(simulation_snapshot(tracks=(track,))))
    assert decision.reason_code == Reason.TRAFFIC_TRACK_INVALID
    assert decision.conflict_status == ConflictStatus.INVALID_TRAFFIC_DATA


def test_unsupported_altitude_frame_is_rejected():
    track = track_at_ned(30.0, 10.0, 15.0, altitude_frame=TrafficAltitudeFrame.BAROMETRIC)
    decision = _at_release(_authorizer(simulation_snapshot(tracks=(track,))))
    assert decision.reason_code == Reason.TRAFFIC_ALTITUDE_FRAME_UNKNOWN


def test_duplicate_track_identifiers_are_rejected():
    first = track_at_ned(0.0, 0.0, 15.0, track_id="AC1")
    second = track_at_ned(80.0, 0.0, 15.0, track_id="AC1")
    decision = _at_release(_authorizer(simulation_snapshot(tracks=(first, second))))
    assert decision.reason_code == Reason.TRAFFIC_DUPLICATE_TRACK


def test_future_observation_is_rejected():
    snapshot = simulation_snapshot(observed_at_s=100.0, received_at_s=100.0)
    decision = _at_release(_authorizer(snapshot), now=10.0)
    assert decision.reason_code == Reason.TRAFFIC_FUTURE_OBSERVATION


def test_invalid_velocity_is_rejected():
    track = track_at_ned(30.0, 10.0, 15.0, ground_speed_mps=-1.0)
    decision = _at_release(_authorizer(simulation_snapshot(tracks=(track,))))
    assert decision.reason_code == Reason.TRAFFIC_INVALID_VELOCITY


def test_missing_source_metadata_is_rejected():
    decision = _at_release(_authorizer(simulation_snapshot(source="")))
    assert decision.reason_code == Reason.TRAFFIC_SOURCE_UNAVAILABLE


def test_aircraft_outside_the_simulation_bubble_does_not_deny_release():
    track = track_at_ned(5030.0, 10.0, 15.0, track_id="FAR")
    decision = _at_release(_authorizer(simulation_snapshot(tracks=(track,))))
    assert decision.authorized is True
    assert decision.conflict_status == ConflictStatus.NO_REPORTED_CONFLICT


def test_aircraft_inside_the_simulation_bubble_denies_release():
    track = track_at_ned(30.0, 10.0, 15.0, track_id="NEAR")
    decision = _at_release(_authorizer(simulation_snapshot(tracks=(track,))))
    assert decision.reason_code == Reason.TRAFFIC_CONFLICT_DETECTED
    assert decision.conflict_status == ConflictStatus.CONFLICT_DETECTED
    assert decision.limits["simulation_only"] is True


def test_predicted_converging_track_denies_release():
    track = track_at_ned(230.0, 10.0, 15.0, ground_speed_mps=20.0, heading_deg=180.0, track_id="IN")
    decision = _at_release(_authorizer(simulation_snapshot(tracks=(track,))))
    assert decision.reason_code == Reason.TRAFFIC_PREDICTED_CONFLICT
    assert decision.conflict_status == ConflictStatus.POTENTIAL_CONFLICT
    assert decision.conflicts[0]["t_cpa_s"] == pytest.approx(10.0, abs=0.05)


def test_diverging_track_does_not_deny_release():
    track = track_at_ned(230.0, 10.0, 15.0, ground_speed_mps=20.0, heading_deg=0.0, track_id="OUT")
    decision = _at_release(_authorizer(simulation_snapshot(tracks=(track,))))
    assert decision.authorized is True


def test_stationary_aircraft_denies_only_when_it_is_inside_the_bubble():
    far = track_at_ned(530.0, 10.0, 15.0, ground_speed_mps=0.0, track_id="PARKED")
    near = track_at_ned(30.0, 10.0, 15.0, ground_speed_mps=0.0, track_id="PARKED")
    assert _at_release(_authorizer(simulation_snapshot(tracks=(far,)))).authorized is True
    assert _at_release(_authorizer(simulation_snapshot(tracks=(near,)))).reason_code == Reason.TRAFFIC_CONFLICT_DETECTED


def test_high_uncertainty_that_reaches_the_bubble_denies_release():
    track = track_at_ned(110.0, 10.0, 15.0, position_uncertainty_m=200.0, track_id="UNCERTAIN")
    decision = _at_release(_authorizer(simulation_snapshot(tracks=(track,))))
    assert decision.reason_code == Reason.TRAFFIC_UNCERTAINTY_EXCEEDS_LIMIT
    assert decision.conflict_status == ConflictStatus.INSUFFICIENT_DATA


def test_stale_track_denies_release():
    track = track_at_ned(530.0, 10.0, 15.0, observed_at_s=0.0, received_at_s=0.0)
    snapshot = simulation_snapshot(observed_at_s=10.0, received_at_s=10.0, tracks=(track,))
    authorizer = simulation_traffic_authorizer(snapshot, simulation_traffic_policy(max_track_age_s=5.0))
    decision = authorizer.evaluate(10.0, 30.0, 10.0, -15.0, (0.0, 0.0, 0.0), 0.0)
    assert decision.reason_code == Reason.TRAFFIC_TRACK_STALE


def test_several_aircraft_keep_the_current_conflict_ahead_of_a_prediction():
    near = track_at_ned(30.0, 10.0, 15.0, track_id="NEAR")
    inbound = track_at_ned(230.0, 10.0, 15.0, ground_speed_mps=20.0, heading_deg=180.0, track_id="IN")
    decision = _at_release(_authorizer(simulation_snapshot(tracks=(inbound, near))))
    assert decision.reason_code == Reason.TRAFFIC_CONFLICT_DETECTED
    assert {item["track_id"] for item in decision.conflicts} == {"NEAR", "IN"}


def test_closest_approach_for_a_constant_velocity_track():
    report = closest_approach(0.0, 0.0, 15.0, 0.0, 0.0, 0.0, 100.0, 0.0, 15.0, -10.0, 0.0, 0.0)
    assert report["t_cpa_s"] == pytest.approx(10.0)
    assert report["horizontal_separation_m"] == pytest.approx(100.0)
    assert report["cpa_horizontal_m"] == pytest.approx(0.0)
    assert report["cpa_vertical_m"] == pytest.approx(0.0)
    assert report["units"]["horizontal_separation"] == "m"
    assert report["units"]["time"] == "s"


def test_recorded_replay_rejects_a_simulated_snapshot():
    provider = RecordedTrafficReplayProvider([(0.0, simulation_snapshot())])
    fetch = provider.latest(1.0)
    assert fetch.reason_code == Reason.TRAFFIC_SOURCE_UNAVAILABLE
    recorded = simulation_snapshot(provenance=Provenance.RECORDED)
    assert RecordedTrafficReplayProvider([(0.0, recorded)]).latest(1.0).snapshot.provenance is Provenance.RECORDED


def test_all_gates_authorize_release_without_operational_approval():
    supervisor = _supervisor()
    command = _release(supervisor)
    evidence = supervisor.evidence()
    assert command.dispense is True
    assert evidence["schema"] == 4
    assert evidence["operational_approval"] is False
    assert evidence["authorization_summary"]["gates_authorized"] is True
    assert evidence["decisions"][-1]["authorized"] is True
    assert evidence["weather_decisions"][-1]["authorized"] is True
    assert evidence["airspace_decisions"][-1]["authorized"] is True
    assert evidence["traffic_decisions"][-1]["authorized"] is True
    assert evidence["communications_decisions"][-1]["authorized"] is True
    json.dumps(evidence, allow_nan=False)


def test_traffic_conflict_denies_release_and_a_later_clear_snapshot_does_not_resume():
    profile = sim_profile()
    clear = simulation_snapshot()
    blocked = simulation_snapshot(tracks=(track_at_ned(30.0, 10.0, 15.0, track_id="AC-NEAR"),))
    provider = SimulatedTrafficProvider([(0.0, clear), (profile.hold_s + 0.05, blocked), (profile.hold_s + 1.0, clear)])
    supervisor = _supervisor(traffic=simulation_traffic_authorizer_from(provider))
    release = profile.release_ned
    supervisor.step(release, 0.0, _snapshot(0.0, release), dt_s=0.1)
    opened = supervisor.step(release, profile.hold_s, _snapshot(profile.hold_s, release), dt_s=0.1)
    assert opened.dispense is True
    samples = supervisor.mission.samples
    closed = supervisor.step(release, profile.hold_s + 0.1, _snapshot(profile.hold_s + 0.1, release), dt_s=0.1)
    assert closed.dispense is False
    assert supervisor.mission.reason == Reason.TRAFFIC_CONFLICT_DETECTED
    assert supervisor.mission.samples == samples
    assert len(samples) == 1
    again = supervisor.step(release, profile.hold_s + 1.0, _snapshot(profile.hold_s + 1.0, release), dt_s=0.1)
    assert again.phase is SeedingPhase.RETURN
    assert again.dispense is False
    assert supervisor.mission.samples == samples
    assert supervisor.traffic_decisions[-1]["authorized"] is True
    assert supervisor.traffic_decisions[-1]["dispenser_open"] is False


def test_altitude_weather_and_airspace_denials_outrank_traffic():
    conflict = simulation_traffic_authorizer(simulation_snapshot(tracks=(track_at_ned(30.0, 10.0, 15.0),)))
    low = _supervisor(traffic=conflict)
    profile = sim_profile()
    release = profile.release_ned
    low.step(release, 0.0, _snapshot(0.0, release, local_z_m=-10.0, baro_alt_m=10.0), dt_s=0.1)
    low.step(release, profile.hold_s, _snapshot(profile.hold_s, release, local_z_m=-10.0, baro_alt_m=10.0), dt_s=0.1)
    assert low.mission.reason.startswith("local altitude")
    assert low.traffic_decisions[-1]["authorized"] is False

    windy = simulation_observation(wind_speed_mps=20.0, wind_gust_mps=25.0)
    weather = _supervisor(traffic=conflict, observation=windy)
    _release(weather)
    assert weather.mission.reason == "wind_above_limit"
    assert weather.evidence()["decisions"][-1]["authorized"] is True

    volume = AirspaceVolume("P-1", AirspaceKind.PROHIBITED, 46.9, 47.1, -122.2, -121.8, 0.0, 20_000.0)
    airspace = _supervisor(
        traffic=conflict,
        airspace=simulation_authorizer(simulation_dataset(volumes=(volume,))),
    )
    _release(airspace)
    assert airspace.mission.reason == "inside_prohibited_airspace"
    assert airspace.evidence()["weather_decisions"][-1]["authorized"] is True
    assert airspace.evidence()["airspace_decisions"][-1]["authorized"] is False


def test_missing_required_traffic_denies_release_before_a_clearance_can_matter():
    supervisor = _supervisor(traffic=simulation_traffic_authorizer_from(StaticTrafficProvider(None)))
    command = _release(supervisor)
    assert command.dispense is False
    assert supervisor.mission.reason == Reason.TRAFFIC_DATA_MISSING
    assert supervisor.comms_decisions[-1]["authorized"] is True
    assert supervisor.evidence()["authorization_summary"]["primary_reason"] == Reason.TRAFFIC_DATA_MISSING


def test_identical_traffic_inputs_produce_identical_evidence():
    def run():
        track = track_at_ned(30.0, 10.0, 15.0, track_id="AC-SAME")
        supervisor = _supervisor(traffic=simulation_traffic_authorizer(simulation_snapshot(tracks=(track,))))
        _release(supervisor)
        return supervisor.evidence()

    assert run() == run()
    assert run()["traffic_conflicts"][-1]["units"]["vertical_separation"] == "m"
    assert run()["traffic_source_metadata"]["provenance"] == "simulated"


def test_provider_latest_does_not_use_the_network_or_grow_history(monkeypatch):
    def boom(*_args, **_kwargs):
        raise AssertionError("network")

    monkeypatch.setattr(urllib.request, "urlopen", boom)
    provider = SimulatedTrafficProvider([(0.0, simulation_snapshot())])
    before = len(provider._segments)
    for _ in range(50):
        provider.latest(1.0)
    assert len(provider._segments) == before


def simulation_traffic_authorizer_from(provider):
    from px4_offboard.traffic import TrafficAuthorizer

    return TrafficAuthorizer(provider, simulation_traffic_policy())
