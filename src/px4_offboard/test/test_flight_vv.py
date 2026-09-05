"""Unit tests for flight-log replay and V&V requirement checks."""

from pathlib import Path

from px4_offboard.flight_replay import FlightSample, FlightTrace, load_flight_log, write_flight_log
from px4_offboard.vv_harness import (
    check_obstacle_clearance,
    check_sensor_backed_avoidance,
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


def test_extended_sensor_evidence_roundtrip(tmp_path: Path):
    sample = _sample(
        obstacle="left",
        obstacle_source="sensor_only",
        sensor_fresh=True,
        lidar_front_m=5.2,
        lidar_left_m=2.1,
        lidar_right_m=6.4,
        mapped_clearance_m=1.25,
        nominal_n=15.0,
        nominal_e=-6.0,
        nominal_d=-5.0,
    )
    path = tmp_path / "evidence.csv"
    write_flight_log(path, [sample])
    loaded = load_flight_log(path).samples[0]
    assert loaded.obstacle_source == "sensor_only"
    assert loaded.sensor_fresh
    assert loaded.lidar_left_m == 2.1
    assert loaded.mapped_clearance_m == 1.25
    assert loaded.nominal_e == -6.0


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


def test_sensor_backed_avoidance_passes_with_matching_range():
    sample = _sample(
        obstacle="front",
        obstacle_source="sensor_only",
        sensor_fresh=True,
        lidar_front_m=2.4,
    )
    trace = FlightTrace(samples=[sample], columns=("sensor_fresh",))
    assert check_sensor_backed_avoidance(trace).passed


def test_sensor_only_avoidance_fails_without_fresh_lidar():
    sample = _sample(obstacle_source="sensor_only", sensor_fresh=False)
    trace = FlightTrace(samples=[sample], columns=("sensor_fresh",))
    assert not check_sensor_backed_avoidance(trace).passed


def test_sensor_backed_avoidance_passes_with_confirmed_clear_sector():
    # lidar_sectors.py publishes -1.0 to mean "nothing detected in this
    # sector" — a fresh, valid reading that the direction is clear, not a
    # missing/stale one. Steering toward a confirmed-clear sector is the
    # single safest case and must not be flagged as unverified (regression
    # for the false-positive this used to produce on real flight logs).
    sample = _sample(
        obstacle="right",
        obstacle_source="sensor_only",
        sensor_fresh=True,
        lidar_right_m=-1.0,
    )
    trace = FlightTrace(samples=[sample], columns=("sensor_fresh",))
    assert check_sensor_backed_avoidance(trace).passed


def test_zero_mapped_clearance_fails_collision_requirement():
    sample = _sample(mapped_clearance_m=0.0)
    trace = FlightTrace(samples=[sample], columns=("mapped_clearance_m",))
    assert not check_obstacle_clearance(trace).passed


def test_gps_columns_skipped_on_legacy_logs():
    from px4_offboard.vv_harness import (
        check_gps_denied_zone,
        check_gps_inject_source,
        check_gps_policy_response,
    )

    trace = FlightTrace(samples=[_sample()], source="legacy", columns=())
    assert check_gps_denied_zone(trace).skipped
    assert check_gps_inject_source(trace).skipped
    assert check_gps_policy_response(trace).skipped


def test_gps_zone_flag_matches_default_aabb():
    from px4_offboard.vv_harness import check_gps_denied_zone

    cols = ("in_gps_denied_zone",)
    inside = _sample(
        t_s=0.0, north=23.0, east=0.0, down=-5.0, in_gps_denied_zone=True
    )
    outside = _sample(
        t_s=0.1, north=5.0, east=-13.0, down=-5.0, in_gps_denied_zone=False
    )
    ok = FlightTrace(samples=[inside, outside], columns=cols, source="gps-zone-ok")
    assert check_gps_denied_zone(ok).passed

    bad = FlightTrace(
        samples=[
            _sample(north=23.0, east=0.0, down=-5.0, in_gps_denied_zone=False)
        ],
        columns=cols,
        source="gps-zone-bad",
    )
    assert not check_gps_denied_zone(bad).passed


def test_gps_inject_rejects_healthy_gps_claim():
    from px4_offboard.vv_harness import check_gps_inject_source

    cols = ("gps_injected_deny", "loc_source")
    ok = FlightTrace(
        samples=[
            _sample(
                gps_injected_deny=True,
                loc_source="GPS_DENIED_INJECTED",
                in_gps_denied_zone=True,
            )
        ],
        columns=cols,
    )
    assert check_gps_inject_source(ok).passed

    bad = FlightTrace(
        samples=[_sample(gps_injected_deny=True, loc_source="GPS")],
        columns=cols,
    )
    assert not check_gps_inject_source(bad).passed


def test_gps_policy_requires_failsafe_after_loc_failsafe_event():
    from px4_offboard.vv_harness import check_gps_policy_response

    cols = ("loc_event",)
    ok = FlightTrace(
        samples=[
            _sample(t_s=0.0, loc_event="LOC_FAILSAFE", state="MOVE"),
            _sample(t_s=0.5, loc_event="", state="FAILSAFE"),
        ],
        columns=cols,
    )
    assert check_gps_policy_response(ok).passed

    bad = FlightTrace(
        samples=[
            _sample(t_s=0.0, loc_event="LOC_FAILSAFE", state="MOVE"),
            _sample(t_s=3.0, loc_event="", state="MOVE"),
        ],
        columns=cols,
    )
    assert not check_gps_policy_response(bad, response_window_s=2.0).passed


def test_gps_fields_roundtrip_csv(tmp_path: Path):
    sample = _sample(
        in_gps_denied_zone=True,
        gps_xy_valid=False,
        gps_injected_deny=True,
        loc_source="GPS_DENIED_INJECTED",
        loc_event="GPS_INVALID",
        dead_reckoning=False,
        eph_m=2.5,
    )
    path = tmp_path / "gps.csv"
    write_flight_log(path, [sample])
    loaded = load_flight_log(path).samples[0]
    assert loaded.in_gps_denied_zone
    assert loaded.gps_injected_deny
    assert loaded.gps_xy_valid is False
    assert loaded.loc_source == "GPS_DENIED_INJECTED"
    assert loaded.eph_m == 2.5
