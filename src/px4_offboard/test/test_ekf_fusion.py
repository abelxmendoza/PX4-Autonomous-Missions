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
    ekf = PoseVelocityEKF()
    # Simulate accumulated gyro drift: filter thinks it's still level.
    assert np.allclose(ekf.quat, [1, 0, 0, 0])
    # VO says the camera actually rotated 0.2 rad about yaw since the last
    # correction.
    true_rotation = quat_to_rotation_matrix(quat_from_rotvec(np.array([0, 0, 0.2])))
    for _ in range(10):
        ekf.apply_vo_attitude_correction(true_rotation)
    r = quat_to_rotation_matrix(ekf.quat)
    yaw = math.atan2(r[1, 0], r[0, 0])
    # Repeated correction toward the same relative rotation should converge
    # the estimate close to the VO-implied yaw, not leave it at zero.
    assert abs(yaw) > 0.05
