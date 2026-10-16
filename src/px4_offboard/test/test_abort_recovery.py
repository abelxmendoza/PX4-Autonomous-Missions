"""Abort-to-recovery contracts: startup identity, touchdown lag, and split outcomes."""

from __future__ import annotations

from px4_offboard.altitude_reference import AltitudeFrame
from px4_offboard.mission_geofence import (
    RECOVERY_TOUCHDOWN_LAG_M,
    GeofenceConfig,
    command_admissible,
    evaluate_vehicle,
)
from px4_offboard.mission_orchestrator import MissionState
from px4_offboard.precision_land import CameraTag, CommandMode, PrecisionLand, PrecisionLandParams, VehicleSnapshot
from px4_offboard.px4_startup import CONNECTED, FAILED, NOT_STARTED, READY, STARTING, assess, process_gate
from px4_offboard.vision_marker_detect import MarkerPose, select_landing_marker

from test_mission_orchestrator import _orch, _tag_below, _vehicle


SQUARE = ((-10.0, -20.0), (50.0, -20.0), (50.0, 30.0), (-10.0, 30.0))
RECOVERY = ((-5.0, -15.0), (25.0, -15.0), (25.0, 15.0), (-5.0, 15.0))
SUCCESS = "Startup script returned successfully\n"


def _fence() -> GeofenceConfig:
    return GeofenceConfig(
        source="unit-test",
        polygon_ned=SQUARE,
        altitude_frame=AltitudeFrame.LOCAL,
        min_altitude_m=2.0,
        max_altitude_m=30.0,
        margin_m=1.0,
        approach_distance_m=3.0,
        uncertainty_sigma=1.0,
        prediction_horizon_s=2.0,
        recovery_polygon_ned=RECOVERY,
        recovery_min_altitude_m=0.0,
    )


def _vehicle_decision(altitude: float):
    return evaluate_vehicle(
        _fence(),
        north_m=14.17,
        east_m=-3.93,
        altitude_m=altitude,
        altitude_valid=True,
        altitude_frame=AltitudeFrame.LOCAL,
        position_uncertainty_m=0.0,
        velocity_ned=(0.0, 0.0, 0.6),
        velocity_uncertainty_mps=None,
        time_s=50.0,
        mission_state="precision_landing",
    )


def test_touchdown_lag_inside_recovery_is_admitted():
    verdict = command_admissible(
        _fence(),
        _vehicle_decision(-0.4),
        use_position=False,
        position_ned=None,
        velocity_ned=(0.0, 0.0, 0.6),
        north_m=14.17,
        east_m=-3.93,
        altitude_m=-0.4,
    )
    assert verdict.admissible
    assert -0.4 >= -RECOVERY_TOUCHDOWN_LAG_M


def test_descent_beyond_touchdown_lag_is_rejected():
    verdict = command_admissible(
        _fence(),
        _vehicle_decision(-3.0),
        use_position=False,
        position_ned=None,
        velocity_ned=(0.0, 0.0, 0.6),
        north_m=14.17,
        east_m=-3.93,
        altitude_m=-3.0,
    )
    assert not verdict.admissible
    assert verdict.reason_code == "predicted_command_exit"


def test_descent_that_leaves_the_recovery_polygon_is_rejected():
    verdict = command_admissible(
        _fence(),
        _vehicle_decision(1.0),
        use_position=False,
        position_ned=None,
        velocity_ned=(20.0, 0.0, 0.6),
        north_m=14.17,
        east_m=-3.93,
        altitude_m=1.0,
    )
    assert not verdict.admissible


def test_stale_success_line_is_not_a_fresh_startup():
    state = process_gate(
        pid_alive=False,
        log_text=SUCCESS,
        log_mtime_s=100.0,
        launched_at_s=200.0,
    )
    assert state == NOT_STARTED


def test_live_process_with_a_fresh_success_line_is_ready():
    state = process_gate(
        pid_alive=True,
        log_text=SUCCESS,
        log_mtime_s=201.0,
        launched_at_s=200.0,
    )
    assert state == READY


def test_process_exit_before_success_is_failed():
    state = process_gate(
        pid_alive=False,
        log_text="px4 starting\n",
        log_mtime_s=201.0,
        launched_at_s=200.0,
    )
    assert state == FAILED


def test_mavlink_without_telemetry_is_connected():
    state = assess(
        pid_alive=True,
        log_text=SUCCESS,
        log_mtime_s=201.0,
        launched_at_s=200.0,
        mavlink_connected=True,
        telemetry_fresh=False,
    )
    assert state == CONNECTED


def test_telemetry_after_connection_is_ready():
    state = assess(
        pid_alive=True,
        log_text=SUCCESS,
        log_mtime_s=201.0,
        launched_at_s=200.0,
        mavlink_connected=True,
        telemetry_fresh=True,
    )
    assert state == READY


def test_process_alive_without_success_is_starting():
    state = process_gate(pid_alive=True, log_text="", log_mtime_s=201.0, launched_at_s=200.0)
    assert state == STARTING


def test_in_recovery_marker_outranks_a_closer_marker_outside_it():
    off_axis = MarkerPose(4, (0.0, 0.0, 3.0), (0.0, 0.0, 0.0, 1.0), ((200.0, 200.0), (240.0, 200.0), (240.0, 240.0), (200.0, 240.0)), 40.0)
    centered = MarkerPose(41, (1.0, 0.0, 2.0), (0.0, 0.0, 0.0, 1.0), ((300.0, 220.0), (340.0, 220.0), (340.0, 260.0), (300.0, 260.0)), 40.0)
    chosen = select_landing_marker([off_axis, centered], 320.0, 240.0, {4: True, 41: False})
    assert chosen.marker_id == 4


def test_marker_timeout_on_the_pad_waits_for_the_land_detector():
    orch = _orch()
    orch.start(0.0)
    orch._handed_off = True
    orch.state = MissionState.PRECISION_LANDING
    orch._precision_t0 = 0.0
    pad = (14.17, -3.93, 0.4)
    orch.lander.activate(pad)
    for k in range(3):
        orch.step(_vehicle(1.0 + k * 0.1, pad, tag=_tag_below(pad, 1.0 + k * 0.1), dt=0.1))
    held = None
    for k in range(70):
        held = orch.step(_vehicle(1.3 + k * 0.1, pad, tag=None, dt=0.1))
    assert held is not None
    assert held.state is MissionState.PRECISION_LANDING
    assert not any(item["code"].startswith("marker_") for item in orch.faults)
    bounced = None
    aloft = (14.17, -3.93, -1.5)
    for k in range(60):
        bounced = orch.step(_vehicle(8.3 + k * 0.1, aloft, tag=None, dt=0.1))
    assert bounced.request_nav_land is True
    assert not any(item["code"].startswith("marker_") for item in orch.faults)
    done = orch.step(_vehicle(16.0, (14.17, -3.93, 0.0), landed=True, offboard=False, tag=None, dt=0.1))
    assert done.state is MissionState.LANDED
    assert orch.evidence()["outcomes"]["recovery_result"] == "landed"
    assert not any(item["code"] in ("offboard_lost", "offboard_stream_gap") for item in orch.faults)


def test_centered_marker_is_preferred_over_a_larger_off_axis_marker():
    off_axis = MarkerPose(41, (1.0, 0.0, 2.0), (0.0, 0.0, 0.0, 1.0), ((400.0, 10.0), (480.0, 10.0), (480.0, 90.0), (400.0, 90.0)), 80.0)
    centered = MarkerPose(4, (0.0, 0.0, 3.0), (0.0, 0.0, 0.0, 1.0), ((300.0, 220.0), (340.0, 220.0), (340.0, 260.0), (300.0, 260.0)), 40.0)
    chosen = select_landing_marker([off_axis, centered], 320.0, 240.0)
    assert chosen.marker_id == 4


def test_marker_jump_does_not_refresh_the_fix():
    lander = PrecisionLand(PrecisionLandParams(max_fix_jump_m=2.0))
    vehicle = VehicleSnapshot((0.0, 0.0, -6.0), (0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0), False, 1.0)
    lander.activate((0.0, 0.0, -6.0))
    first = CameraTag((0.0, 0.0, 6.0), (0.0, 0.0, 0.0, 1.0), 1.0)
    assert lander.observe_tag(first, vehicle) is not None
    jumped = CameraTag((0.0, 0.0, 6.0), (0.0, 0.0, 0.0, 1.0), 2.0)
    # Body-frame tag at a large horizontal offset from the accepted fix.
    far = CameraTag((5.0, 0.0, 6.0), (0.0, 0.0, 0.0, 1.0), 2.0)
    assert lander.observe_tag(far, vehicle) is None
    assert lander.last_observation_reject == "marker_outlier_rejected"
    assert lander._tag.time_s == 1.0
    assert jumped.time_s == 2.0


def test_release_denial_and_recovery_are_separate_results():
    orch = _orch()
    orch.start(0.0)
    orch._release_aborted = True
    orch.mission._reason = "traffic_conflict_detected"
    orch.state = MissionState.ABORTED
    outcomes = orch.evidence()["outcomes"]
    assert outcomes["release_result"] == "denied"
    assert outcomes["release_reason_code"] == "traffic_conflict_detected"
    assert outcomes["mission_execution_result"] == "aborted_release"
    assert outcomes["recovery_result"] == "landed"


def test_failed_lander_holds_and_does_not_command_descent():
    lander = PrecisionLand()
    staged = (0.0, 0.0, -6.0)
    vehicle = VehicleSnapshot(staged, (0.0, 0.0, 0.6), (1.0, 0.0, 0.0, 0.0), False, 10.0)
    lander.activate(staged)
    lander._fail(vehicle)
    command = lander.step(vehicle)
    assert lander.failed
    assert command.mode is CommandMode.HOLD
    assert command.velocity_ned is None
    assert lander.failure_reason == "marker_never_acquired"


def test_once_locked_the_landing_tracks_one_marker_and_ignores_a_better_centred_other():
    # At the 18 m staging height the camera sees several markers inside the recovery area
    # (4 and 9 on search_field). Re-selecting every frame flipped the target between them,
    # 11 m apart, and the lander descended on stale fixes until it timed out (SITL,
    # altitude_denial and marker_loss, 2026-10-13). After the first pick it must stay put.
    from px4_offboard.vision_marker_detect import track_landing_marker
    four = MarkerPose(4, (0.0, 0.0, 3.0), (0.0, 0.0, 0.0, 1.0), ((200.0, 200.0), (240.0, 200.0), (240.0, 240.0), (200.0, 240.0)), 40.0)
    nine = MarkerPose(9, (1.0, 0.0, 2.0), (0.0, 0.0, 0.0, 1.0), ((300.0, 220.0), (340.0, 220.0), (340.0, 260.0), (300.0, 260.0)), 40.0)
    first = track_landing_marker([four, nine], 320.0, 240.0, {4: True, 9: True}, locked_id=None)
    assert first.marker_id == 9                       # unlocked: the usual pick (nearest centre)
    locked = track_landing_marker([four, nine], 320.0, 240.0, {4: True, 9: True}, locked_id=4)
    assert locked.marker_id == 4                      # locked: the other is ignored
    assert track_landing_marker([nine], 320.0, 240.0, {9: True}, locked_id=4) is None   # no fix this frame
