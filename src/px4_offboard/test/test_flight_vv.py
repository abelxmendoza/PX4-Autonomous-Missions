"""Unit tests for flight-log replay and V&V requirement checks."""

from pathlib import Path

from px4_offboard.flight_replay import FlightSample, FlightTrace, load_flight_log, write_flight_log
from px4_offboard.vv_harness import (
    check_executive_abort,
    check_geofence_response,
    check_legal_state_sequence,
    check_wp_monotonic,
    run_vv,
)


def _sample(**overrides) -> FlightSample:
    values = dict(
        t_s=0.0,
        state="MOVE",
        north=10.0,
        east=0.0,
        down=-5.0,
        tgt_n=12.0,
        tgt_e=0.0,
        tgt_d=-5.0,
        obstacle="",
        wp_index=1,
        geocage=True,
        geofence=True,
        inside=True,
        caged=False,
        executive_mode="NOMINAL",
        battery_frac=0.9,
        link_quality=1.0,
        propellant_s=150.0,
    )
    values.update(overrides)
    return FlightSample(**values)


def test_load_roundtrip(tmp_path: Path):
    samples = [
        _sample(t_s=0.0, state="MOVE", wp_index=0),
        _sample(t_s=0.1, state="MOVE", wp_index=1, north=11.0),
        _sample(t_s=0.2, state="LANDING", wp_index=1),
    ]
    path = tmp_path / "flight.csv"
    write_flight_log(path, samples)
    trace = load_flight_log(path)
    assert len(trace) == 3
    assert trace.unique_state_sequence() == ["MOVE", "LANDING"]
    assert abs(trace.duration_s - 0.2) < 1e-6
    assert trace.samples[1].north == 11.0


def test_nominal_trace_passes_vv():
    samples = [
        _sample(t_s=0.0, state="MOVE", wp_index=0),
        _sample(t_s=0.5, state="MOVE", wp_index=1, obstacle="front"),
        _sample(t_s=1.0, state="MOVE", wp_index=2, caged=True, tgt_n=10.0),
        _sample(t_s=1.5, state="LANDING", wp_index=2),
    ]
    report = run_vv(FlightTrace(samples=samples, source="synthetic-nominal"))
    assert report.passed, report.format_text()


def test_illegal_state_transition_fails():
    samples = [
        _sample(t_s=0.0, state="MOVE"),
        _sample(t_s=0.1, state="PREFLIGHT"),  # illegal
    ]
    result = check_legal_state_sequence(FlightTrace(samples=samples, source="bad"))
    assert not result.passed
    assert result.evidence


def test_wp_index_decrease_fails():
    samples = [
        _sample(t_s=0.0, wp_index=3),
        _sample(t_s=0.1, wp_index=2),
    ]
    result = check_wp_monotonic(FlightTrace(samples=samples, source="bad"))
    assert not result.passed


def test_geofence_breach_without_failsafe_fails():
    samples = [
        _sample(t_s=0.0, inside=True),
        _sample(t_s=0.2, inside=False, north=99.0),
        _sample(t_s=0.4, inside=False, north=99.0),
        _sample(t_s=2.0, inside=False, north=99.0),  # past response window
    ]
    result = check_geofence_response(
        FlightTrace(samples=samples, source="breach"), response_window_s=1.0
    )
    assert not result.passed


def test_geofence_breach_then_failsafe_passes():
    samples = [
        _sample(t_s=0.0, inside=True),
        _sample(t_s=0.2, inside=False, north=99.0),
        _sample(t_s=0.5, state="FAILSAFE", inside=False, north=99.0),
    ]
    result = check_geofence_response(
        FlightTrace(samples=samples, source="ok"), response_window_s=1.0
    )
    assert result.passed


def test_executive_abort_should_reach_terminal():
    samples = [
        _sample(t_s=0.0, executive_mode="NOMINAL"),
        _sample(t_s=0.2, executive_mode="ABORT"),
        _sample(t_s=0.5, state="FAILSAFE", executive_mode="ABORT"),
    ]
    result = check_executive_abort(FlightTrace(samples=samples, source="exec"))
    assert result.passed
    assert not result.skipped


def test_caged_setpoint_outside_fails_vv():
    samples = [
        _sample(
            t_s=0.0,
            caged=True,
            tgt_n=100.0,  # outside default fence
            tgt_e=0.0,
            tgt_d=-5.0,
        )
    ]
    report = run_vv(FlightTrace(samples=samples, source="cage-bad"))
    geocage = next(r for r in report.results if r.requirement_id == "REQ-GEOCAGE-01")
    assert not geocage.passed
    assert not report.passed


def test_vv_replay_cli(tmp_path: Path):
    from px4_offboard.vv_replay import main

    samples = [
        _sample(t_s=0.0, state="MOVE", wp_index=0),
        _sample(t_s=0.5, state="LANDING", wp_index=0),
    ]
    path = tmp_path / "ok.csv"
    write_flight_log(path, samples)
    assert main([str(path)]) == 0

    bad = [
        _sample(t_s=0.0, state="MOVE", wp_index=5),
        _sample(t_s=0.1, state="MOVE", wp_index=1),
    ]
    bad_path = tmp_path / "bad.csv"
    write_flight_log(bad_path, bad)
    assert main([str(bad_path)]) == 1
