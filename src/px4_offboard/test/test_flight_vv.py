"""Unit tests for flight-log replay and V&V requirement checks."""

import json
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


def test_report_to_dict_is_json_serializable_and_preserves_skip_vs_pass():
    # The web viewer's "Requirements" panel is built entirely from this
    # dict (scripts/gen_vv_reports.py ships it as JSON next to each mission
    # CSV) — it must faithfully distinguish "skipped" (not applicable to
    # this recording's schema) from "passed", not collapse them, since a
    # legacy recording showing all-green would misrepresent what was
    # actually verified.
    samples = [
        _sample(t_s=0.0, state="MOVE", wp_index=0),
        _sample(t_s=0.5, state="LANDING", wp_index=1),
    ]
    report = run_vv(FlightTrace(samples=samples, source="synthetic-dict-test"))

    d = report.to_dict()
    json.dumps(d)  # must not raise — every value has to be JSON-safe

    assert d["source"] == "synthetic-dict-test"
    assert d["passed"] == report.passed
    assert len(d["results"]) == len(report.results)

    by_id = {r["requirement_id"]: r for r in d["results"]}
    live = by_id["REQ-STATE-01"]
    assert live["skipped"] is False
    assert live["passed"] is True
    assert isinstance(live["severity"], str)  # enum serialized to its .value, not the enum object

    legacy = by_id["REQ-CLEARANCE-01"]  # no mapped_clearance_m column in these samples
    assert legacy["skipped"] is True
    assert legacy["passed"] is True  # skipped checks report passed=True; UI must check skipped first


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


# ── Stereo/IMU fusion, velocity PID, attitude and contact requirements ───────

import pytest  # noqa: E402

from px4_offboard.vv_harness import (  # noqa: E402
    check_airframe_contact,
    check_attitude_envelope,
    check_velocity_pid_envelope,
    check_vo_availability,
    check_vo_drift,
)


def _trace_of(tmp_path, samples) -> FlightTrace:
    path = tmp_path / "t.csv"
    write_flight_log(path, samples)
    return load_flight_log(path)


def _vo_flight(drift_at_end: float, healthy: bool = True, path_end: float = 60.0):
    """A MOVE flight whose fusion drift grows linearly to ``drift_at_end``."""
    out = []
    for i in range(61):
        path = path_end * i / 60
        frac = drift_at_end * i / 60
        out.append(
            _sample(
                t_s=i * 0.5,
                vo_healthy=healthy,
                vo_n=path,
                vo_e=0.0,
                vo_d=-5.0,
                vo_path_m=path,
                vo_drift_frac=frac,
                vo_err_m=frac * path,
                vo_inliers=40,
            )
        )
    return out


def test_vo_drift_passes_when_error_is_small_relative_to_distance(tmp_path):
    result = check_vo_drift(_trace_of(tmp_path, _vo_flight(0.04)))
    assert result.passed and not result.skipped
    assert "4.0%" in result.detail


def test_vo_drift_fails_when_error_is_a_large_fraction_of_distance(tmp_path):
    result = check_vo_drift(_trace_of(tmp_path, _vo_flight(0.15)))
    assert not result.passed
    assert any("final drift 15.0%" in e for e in result.evidence)


def test_vo_drift_peak_limit_catches_a_transient_even_if_it_recovers(tmp_path):
    flight = _vo_flight(0.04)
    flight[40] = _sample(
        t_s=20.0, vo_healthy=True, vo_n=40.0, vo_e=0.0, vo_d=-5.0,
        vo_path_m=40.0, vo_drift_frac=0.35, vo_err_m=14.0, vo_inliers=40,
    )
    result = check_vo_drift(_trace_of(tmp_path, flight))
    assert not result.passed
    assert any("peak drift 35.0%" in e for e in result.evidence)


def test_vo_drift_is_skipped_not_passed_when_too_little_distance(tmp_path):
    result = check_vo_drift(_trace_of(tmp_path, _vo_flight(0.5, path_end=5.0)))
    assert result.skipped


def test_vo_drift_ignores_unhealthy_estimates(tmp_path):
    # A wildly drifting estimate that the node itself flagged unhealthy is
    # not evidence about the healthy estimate.
    result = check_vo_drift(_trace_of(tmp_path, _vo_flight(0.9, healthy=False)))
    assert result.skipped


def test_vo_availability_fails_when_the_cameras_go_dark(tmp_path):
    # Fusion node publishing (vo_n present) but never healthy: the BUG-017
    # signature -- cameras advertised, no frames.
    result = check_vo_availability(_trace_of(tmp_path, _vo_flight(0.0, healthy=False)))
    assert not result.passed and not result.skipped
    assert "0/61" in result.detail


def test_vo_availability_passes_when_healthy_most_of_the_flight(tmp_path):
    flight = _vo_flight(0.02)
    for i in range(0, 61, 10):  # 7 of 61 dropouts -> 88% available
        flight[i] = _sample(t_s=i * 0.5, vo_healthy=False, vo_n=1.0, vo_e=0.0, vo_d=-5.0)
    assert check_vo_availability(_trace_of(tmp_path, flight)).passed


def test_vo_availability_is_skipped_when_fusion_node_was_not_running(tmp_path):
    flight = [_sample(t_s=i * 0.5) for i in range(20)]
    assert check_vo_availability(_trace_of(tmp_path, flight)).skipped


def test_velocity_pid_envelope_passes_for_bounded_commands(tmp_path):
    flight = [
        _sample(t_s=i * 0.1, ctrl_mode="velocity_pid",
                vel_cmd_n=2.0, vel_cmd_e=-1.5, vel_cmd_d=-0.5)
        for i in range(10)
    ]
    assert check_velocity_pid_envelope(_trace_of(tmp_path, flight)).passed


@pytest.mark.parametrize(
    "overrides, needle",
    [
        (dict(vel_cmd_n=3.0, vel_cmd_e=3.0, vel_cmd_d=0.0), "horiz"),
        (dict(vel_cmd_n=1.0, vel_cmd_e=0.0, vel_cmd_d=2.5), "vert"),
        (dict(vel_cmd_n=None, vel_cmd_e=None, vel_cmd_d=None), "no command"),
        (dict(state="HOVER", vel_cmd_n=1.0, vel_cmd_e=0.0, vel_cmd_d=0.0), "HOVER"),
    ],
)
def test_velocity_pid_envelope_flags_out_of_bounds_samples(tmp_path, overrides, needle):
    base = dict(t_s=0.0, ctrl_mode="velocity_pid")
    base.update(overrides)
    result = check_velocity_pid_envelope(_trace_of(tmp_path, [_sample(**base)]))
    assert not result.passed
    assert any(needle in e for e in result.evidence)


def test_velocity_pid_envelope_skipped_in_position_mode(tmp_path):
    assert check_velocity_pid_envelope(_trace_of(tmp_path, [_sample(ctrl_mode="position")])).skipped


def test_attitude_envelope_flags_a_tumble_before_the_failsafe(tmp_path):
    # Shape of the BUG-016 crashes: normal flight, then roll/pitch swinging
    # past 60 deg in MOVE, and only later a geofence FAILSAFE.
    flight = [
        _sample(t_s=0.0, roll_deg=5.0, pitch_deg=-3.0),
        _sample(t_s=0.1, roll_deg=-44.0, pitch_deg=16.0),
        _sample(t_s=0.2, roll_deg=71.0, pitch_deg=-24.0),
        _sample(t_s=0.3, state="FAILSAFE", roll_deg=84.0, pitch_deg=-28.0),
    ]
    result = check_attitude_envelope(_trace_of(tmp_path, flight))
    assert not result.passed
    assert len(result.evidence) == 1 and "roll=71" in result.evidence[0]


def test_attitude_envelope_passes_for_aggressive_but_controlled_flight(tmp_path):
    flight = [_sample(t_s=i * 0.1, roll_deg=40.0, pitch_deg=-35.0) for i in range(5)]
    assert check_attitude_envelope(_trace_of(tmp_path, flight)).passed


def test_airframe_contact_catches_a_graze_that_clearance_01_passes(tmp_path):
    # The 1.3 cm approach from the first BUG-016 log: positive clearance, so
    # REQ-CLEARANCE-01 is satisfied, but the airframe is touching the wall.
    flight = [_sample(t_s=0.0, mapped_clearance_m=0.013)]
    trace = _trace_of(tmp_path, flight)
    assert check_obstacle_clearance(trace).passed
    contact = check_airframe_contact(trace)
    assert not contact.passed
    assert "0.013" in contact.evidence[0]


def test_airframe_contact_passes_with_real_standoff(tmp_path):
    flight = [_sample(t_s=i * 0.1, mapped_clearance_m=1.2 + i * 0.1) for i in range(5)]
    assert check_airframe_contact(_trace_of(tmp_path, flight)).passed


def test_run_vv_reports_the_new_requirements(tmp_path):
    report = run_vv(_trace_of(tmp_path, _vo_flight(0.04)))
    ids = {r.requirement_id for r in report.results}
    assert {"REQ-VO-DRIFT-01", "REQ-VO-AVAIL-01", "REQ-CTRL-01",
            "REQ-ATT-01", "REQ-CLEARANCE-02"} <= ids


def test_fusion_columns_roundtrip_through_the_csv(tmp_path):
    sample = _sample(
        ctrl_mode="velocity_pid", pos_source="fusion",
        vel_cmd_n=1.0, vel_cmd_e=-0.5, vel_cmd_d=0.1, vo_healthy=True,
        vo_n=4.0, vo_e=5.0, vo_d=-3.0, vo_err_m=0.7, vo_err_down_m=0.1,
        vo_path_m=25.0, vo_drift_frac=0.028, vo_inliers=42,
    )
    back = load_flight_log(_write(tmp_path, [sample])).samples[0]
    assert (back.ctrl_mode, back.pos_source) == ("velocity_pid", "fusion")
    assert back.vo_healthy and back.vo_inliers == 42
    assert back.vo_drift_frac == pytest.approx(0.028)


def _write(tmp_path, samples):
    path = tmp_path / "rt.csv"
    write_flight_log(path, samples)
    return path


def test_velocity_pid_envelope_allows_only_the_move_to_landing_handover_tick(tmp_path):
    # The row is logged after the state flips, so the hand-over tick carries
    # velocity_pid with state LANDING. That single tick is fine; a second
    # consecutive velocity_pid row in LANDING is not.
    def pid(t, state):
        return _sample(t_s=t, state=state, ctrl_mode="velocity_pid",
                       vel_cmd_n=1.0, vel_cmd_e=0.0, vel_cmd_d=0.0)

    ok = [pid(0.0, "MOVE"), pid(0.1, "MOVE"), pid(0.2, "LANDING")]
    assert check_velocity_pid_envelope(_trace_of(tmp_path, ok)).passed

    bad = ok + [pid(0.3, "LANDING")]
    result = check_velocity_pid_envelope(_trace_of(tmp_path, bad))
    assert not result.passed and "t=0.30s" in result.evidence[0]
