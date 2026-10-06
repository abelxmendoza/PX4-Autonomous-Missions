"""Tests for ekf_fusion.py.

The zero-drift-at-rest test is the important one: strapdown INS
mechanization has an easy-to-get-backwards gravity sign, and getting it
wrong doesn't crash anything -- it just makes the filter silently think the
vehicle is accelerating at 2g while sitting still. That failure mode is
exactly what this test is designed to catch.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from px4_offboard.ekf_fusion import (
    GRAVITY_MPS2,
    PoseVelocityEKF,
    quat_from_rotvec,
    quat_multiply,
    quat_normalize,
    quat_slerp,
    quat_to_rotation_matrix,
    rotation_matrix_to_quat,
)


def test_quat_rotation_matrix_roundtrip_for_known_rotations():
    for axis, angle in [
        ((1, 0, 0), 0.3),
        ((0, 1, 0), -0.7),
        ((0, 0, 1), 1.2),
        ((1, 1, 1), 0.9),
    ]:
        axis = np.array(axis, dtype=float)
        axis = axis / np.linalg.norm(axis)
        q = quat_from_rotvec(axis * angle)
        r = quat_to_rotation_matrix(q)
        q_back = rotation_matrix_to_quat(r)
        # q and -q represent the same rotation.
        assert np.allclose(q, q_back, atol=1e-6) or np.allclose(q, -q_back, atol=1e-6)


def test_quat_slerp_endpoints():
    q0 = np.array([1.0, 0.0, 0.0, 0.0])
    q1 = quat_from_rotvec(np.array([0, 0, math.pi / 2]))
    assert np.allclose(quat_slerp(q0, q1, 0.0), q0, atol=1e-9)
    assert np.allclose(quat_slerp(q0, q1, 1.0), q1, atol=1e-9) or np.allclose(
        quat_slerp(q0, q1, 1.0), -q1, atol=1e-9
    )


def test_gyro_integration_recovers_a_known_yaw_rotation():
    ekf = PoseVelocityEKF()
    yaw_rate = 0.5  # rad/s
    dt = 0.01
    steps = 200  # 2 seconds -> 1.0 rad of yaw
    at_rest_accel = np.array([0.0, 0.0, -GRAVITY_MPS2])
    for _ in range(steps):
        ekf.predict(np.array([0.0, 0.0, yaw_rate]), at_rest_accel, dt)
    r = quat_to_rotation_matrix(ekf.quat)
    expected_yaw = yaw_rate * steps * dt
    recovered_yaw = math.atan2(r[1, 0], r[0, 0])
    assert recovered_yaw == pytest.approx(expected_yaw, abs=1e-2)


def test_stationary_level_vehicle_does_not_drift():
    # This is the sign-convention regression: if GRAVITY_WORLD or its
    # application in predict() had the wrong sign, "at rest" accelerometer
    # readings would integrate into large, growing velocity/position.
    ekf = PoseVelocityEKF()
    at_rest_accel = np.array([0.0, 0.0, -GRAVITY_MPS2])
    dt = 0.005
    for _ in range(2000):  # 10 simulated seconds
        ekf.predict(np.zeros(3), at_rest_accel, dt)
    assert np.allclose(ekf.velocity, np.zeros(3), atol=1e-6)
    assert np.allclose(ekf.position, np.zeros(3), atol=1e-6)


def test_constant_world_frame_acceleration_matches_kinematics():
    # Level attitude (identity quat) so body frame == world frame: constant
    # accel of 1 m/s^2 north for 2s -> v=2 m/s, p=2m (x = 0.5*a*t^2).
    ekf = PoseVelocityEKF()
    accel_body = np.array([1.0, 0.0, -GRAVITY_MPS2])  # 1 m/s^2 north + hover
    dt = 0.01
    for _ in range(200):
        ekf.predict(np.zeros(3), accel_body, dt)
    t = 200 * dt
    assert ekf.velocity[0] == pytest.approx(1.0 * t, abs=1e-3)
    assert ekf.position[0] == pytest.approx(0.5 * 1.0 * t**2, abs=1e-2)
    assert ekf.velocity[2] == pytest.approx(0.0, abs=1e-6)


def test_velocity_update_pulls_state_toward_measurement():
    ekf = PoseVelocityEKF()
    ekf.velocity = np.array([0.0, 0.0, 0.0])
    ekf.cov = np.eye(6) * 1.0
    measured = np.array([2.0, 0.0, 0.0])
    ekf.update_velocity(measured, measurement_std=0.1)
    # A confident (low-noise) measurement should move the estimate most of
    # the way toward it, not leave it near the prior.
    assert ekf.velocity[0] > 1.0
    assert ekf.velocity[0] <= 2.0 + 1e-9


def test_velocity_update_reduces_covariance():
    ekf = PoseVelocityEKF()
    ekf.cov = np.eye(6) * 1.0
    trace_before = np.trace(ekf.cov)
    ekf.update_velocity(np.array([1.0, 1.0, 1.0]), measurement_std=0.1)
    assert np.trace(ekf.cov) < trace_before


def test_vo_attitude_correction_pulls_toward_vo_implied_orientation():
    # Gyro says the vehicle yawed 1.0 deg since the last VO update; VO says
    # 1.4 deg. They agree within the consistency gate, so the estimate is
    # nudged from the gyro value toward VO's.
    ekf = PoseVelocityEKF(vo_attitude_blend=0.5)
    ekf.predict(np.array([0.0, 0.0, math.radians(1.0)]), np.array([0.0, 0.0, -9.80665]), 1.0)
    gyro_yaw = math.atan2(*quat_to_rotation_matrix(ekf.quat)[1::-1, 0])
    vo_rotation = quat_to_rotation_matrix(quat_from_rotvec(np.array([0, 0, math.radians(1.4)])))
    assert ekf.apply_vo_attitude_correction(vo_rotation) is True
    corrected_yaw = math.atan2(*quat_to_rotation_matrix(ekf.quat)[1::-1, 0])
    assert gyro_yaw < corrected_yaw < math.radians(1.4) + 1e-6


def test_vo_attitude_correction_is_rejected_when_it_contradicts_the_gyro():
    # A bad VO frame claiming 11 deg of yaw while the gyro saw ~0 must not
    # rotate the estimate: every later velocity would point the wrong way.
    ekf = PoseVelocityEKF(vo_attitude_blend=0.5)
    ekf.predict(np.zeros(3), np.array([0.0, 0.0, -9.80665]), 0.1)
    before = ekf.quat.copy()
    bad = quat_to_rotation_matrix(quat_from_rotvec(np.array([0, 0, math.radians(11.0)])))
    assert ekf.apply_vo_attitude_correction(bad) is False
    assert np.allclose(ekf.quat, before)


def test_initialize_seeds_pose_and_zeroes_velocity():
    from px4_offboard.ekf_fusion import PoseVelocityEKF

    ekf = PoseVelocityEKF()
    ekf.velocity = np.array([1.0, 2.0, 3.0])
    q_yaw90 = np.array([math.cos(math.pi / 4), 0.0, 0.0, math.sin(math.pi / 4)])
    ekf.initialize(np.array([1.0, -2.0, -0.1]), q_yaw90)
    assert np.allclose(ekf.position, [1.0, -2.0, -0.1])
    assert np.allclose(ekf.velocity, 0.0)
    assert np.allclose(ekf.quat, q_yaw90)


def test_forward_vo_velocity_moves_north_when_yawed_north():
    # End-to-end frame check across the fixed chain: VO says "0.5 m forward
    # in 0.1 s" in camera axes; at yaw 0 (body x == north) the fused
    # velocity must be +5 m/s north and ~0 east/down.
    from px4_offboard.ekf_fusion import PoseVelocityEKF, quat_to_rotation_matrix
    from px4_offboard.stereo_depth import RelativeMotion

    ekf = PoseVelocityEKF()
    motion = RelativeMotion(np.eye(3), np.array([0.0, 0.0, 0.5]), 30, dt_s=0.1)
    _, trans_body = motion.in_body_frame()
    v_world = quat_to_rotation_matrix(ekf.quat) @ (trans_body / motion.dt_s)
    for _ in range(50):
        ekf.update_velocity(v_world, 0.15)
    assert ekf.velocity[0] == pytest.approx(5.0, abs=0.1)
    assert abs(ekf.velocity[1]) < 0.1
    assert abs(ekf.velocity[2]) < 0.1


def test_drift_tracker_reports_error_relative_to_distance_travelled():
    from px4_offboard.ekf_fusion import DriftTracker

    tracker = DriftTracker()
    out = {}
    for i in range(0, 101):  # reference flies 100 m north
        ref = np.array([float(i), 0.0, -3.0])
        fused = np.array([float(i) * 0.95, 0.0, -3.0])  # fused undershoots 5%
        out = tracker.update(fused, ref)
    assert out["path_length_m"] == pytest.approx(100.0)
    assert out["err_horiz_m"] == pytest.approx(5.0)
    assert out["drift_frac"] == pytest.approx(0.05)


def test_drift_tracker_does_not_divide_by_tiny_path_length():
    from px4_offboard.ekf_fusion import DriftTracker

    tracker = DriftTracker()
    out = tracker.update(np.array([0.5, 0.0, 0.0]), np.zeros(3))
    assert out["drift_frac"] == 0.0  # not enough path to be meaningful yet


def test_absurd_vo_velocity_is_rejected_and_state_untouched():
    from px4_offboard.ekf_fusion import PoseVelocityEKF

    ekf = PoseVelocityEKF()
    ekf.velocity = np.array([1.0, 0.0, 0.0])
    # The kind of output live flight produced: (39, -77, -76) m/s.
    assert ekf.update_velocity(np.array([39.0, -77.0, -76.0]), 0.15) is False
    assert np.allclose(ekf.velocity, [1.0, 0.0, 0.0])
    assert ekf.rejected_updates == 1 and ekf.accepted_updates == 0
    assert ekf.update_velocity(np.array([np.nan, 0.0, 0.0]), 0.15) is False


def test_statistically_inconsistent_velocity_is_gated_but_consistent_one_is_not():
    from px4_offboard.ekf_fusion import PoseVelocityEKF

    ekf = PoseVelocityEKF()
    for _ in range(30):  # settle: confident, at rest
        ekf.predict(np.zeros(3), np.array([0.0, 0.0, -9.80665]), 0.1)
        ekf.update_velocity(np.zeros(3), 0.15)
    assert ekf.update_velocity(np.array([0.1, -0.05, 0.0]), 0.15) is True
    assert ekf.update_velocity(np.array([8.0, 0.0, 0.0]), 0.15) is False  # 8 m/s from rest


def test_filter_reanchors_after_persistent_rejection_instead_of_locking_out():
    from px4_offboard.ekf_fusion import PoseVelocityEKF

    ekf = PoseVelocityEKF()
    for _ in range(30):
        ekf.predict(np.zeros(3), np.array([0.0, 0.0, -9.80665]), 0.1)
        ekf.update_velocity(np.zeros(3), 0.15)
    # The vehicle genuinely starts moving at 4 m/s and VO keeps saying so.
    accepted = [ekf.update_velocity(np.array([4.0, 0.0, 0.0]), 0.15) for _ in range(40)]
    assert any(accepted), "filter locked itself out of a consistent measurement stream"
    assert ekf.velocity[0] == pytest.approx(4.0, abs=0.5)


def test_realistic_attitude_error_does_not_make_velocity_diverge():
    # Stationary vehicle, filter's attitude 5 deg off (so gravity leaks 0.85
    # m/s^2 into the horizontal acceleration), VO correctly reporting zero
    # velocity. With the original 0.05 m/s^2 process noise the filter ignored
    # VO and ran away; the default must keep it bounded.
    from px4_offboard.ekf_fusion import PoseVelocityEKF

    ekf = PoseVelocityEKF()
    tilt = math.radians(5.0)
    ekf.quat = quat_from_rotvec(np.array([tilt, 0.0, 0.0]))
    accel_body = np.array([0.0, 0.0, -9.80665])  # truly level and at rest
    for _ in range(300):  # 30 s at 10 Hz
        ekf.predict(np.zeros(3), accel_body, 0.1)
        ekf.update_velocity(np.zeros(3), 0.15)
    assert np.linalg.norm(ekf.velocity) < 0.3
    assert np.linalg.norm(ekf.position) < 10.0

    overconfident = PoseVelocityEKF(accel_noise_std=0.05)
    overconfident.quat = quat_from_rotvec(np.array([tilt, 0.0, 0.0]))
    for _ in range(300):
        overconfident.predict(np.zeros(3), accel_body, 0.1)
        overconfident.update_velocity(np.zeros(3), 0.15)
    assert np.linalg.norm(overconfident.velocity) > np.linalg.norm(ekf.velocity)


def test_message_dt_uses_the_vehicle_clock_and_rejects_bad_intervals():
    from px4_offboard.ekf_fusion import message_dt_s

    assert message_dt_s(None, 1_000_000, 0.5) is None  # first message: nothing to integrate
    assert message_dt_s(1_000_000, 1_004_000, 0.5) == pytest.approx(0.004)
    assert message_dt_s(1_000_000, 1_000_000, 0.5) is None  # duplicate
    assert message_dt_s(1_000_000, 900_000, 0.5) is None  # clock went backwards
    assert message_dt_s(1_000_000, 2_000_000, 0.5) is None  # 1 s stall: skip, don't integrate


def test_reference_attitude_pulls_a_drifted_heading_back_without_stepping():
    ekf = PoseVelocityEKF(reference_attitude_tau_s=1.0)
    ekf.initialize(np.zeros(3), np.array([1.0, 0.0, 0.0, 0.0]))
    # Gyro integration has wandered 17 degrees in yaw.
    ekf.quat = quat_from_rotvec(np.array([0.0, 0.0, math.radians(17.0)]))
    truth = np.array([1.0, 0.0, 0.0, 0.0])

    def yaw_error_deg() -> float:
        r = quat_to_rotation_matrix(ekf.quat)
        return abs(math.degrees(math.atan2(r[1, 0], r[0, 0])))

    ekf.apply_reference_attitude(truth, dt_s=0.02)
    assert 16.0 < yaw_error_deg() < 17.0  # a gentle nudge, not a snap
    for _ in range(500):  # 10 s at 50 Hz
        ekf.apply_reference_attitude(truth, dt_s=0.02)
    assert yaw_error_deg() < 0.5


def test_reference_attitude_is_off_when_tau_is_zero():
    ekf = PoseVelocityEKF(reference_attitude_tau_s=0.0)
    ekf.initialize(np.zeros(3), np.array([1.0, 0.0, 0.0, 0.0]))
    drifted = quat_from_rotvec(np.array([0.0, 0.0, 0.3]))
    ekf.quat = drifted.copy()
    ekf.apply_reference_attitude(np.array([1.0, 0.0, 0.0, 0.0]), dt_s=0.02)
    assert np.allclose(ekf.quat, drifted)
