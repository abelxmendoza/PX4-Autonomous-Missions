"""ekf_fusion_node: IMU integration through the real callback, and stall accounting."""
from __future__ import annotations

import math

import numpy as np
import pytest

rclpy = pytest.importorskip("rclpy")
px4_msgs_msg = pytest.importorskip("px4_msgs.msg")

from px4_offboard.ekf_fusion import quat_to_rotation_matrix  # noqa: E402
from px4_offboard.ekf_fusion_node import EkfFusionNode  # noqa: E402

SensorCombined = px4_msgs_msg.SensorCombined


@pytest.fixture
def node():
    rclpy.init()
    n = EkfFusionNode()
    yield n
    n.destroy_node()
    rclpy.shutdown()


def _imu(timestamp_us: int, gyro_z: float = 0.0):
    msg = SensorCombined()
    msg.timestamp = timestamp_us
    msg.gyro_rad = [0.0, 0.0, gyro_z]
    msg.accelerometer_m_s2 = [0.0, 0.0, -9.80665]  # level and at rest
    return msg


def _yaw(n) -> float:
    r = quat_to_rotation_matrix(n._ekf.quat)
    return math.atan2(r[1, 0], r[0, 0])


def test_gyro_integration_uses_the_message_timestamps(node):
    # 100 messages 4 ms apart at 0.5 rad/s: 0.396 s of motion -> 0.198 rad.
    t = 1_000_000
    for i in range(100):
        node._imu_cb(_imu(t + i * 4000, gyro_z=0.5))
    assert _yaw(node) == pytest.approx(0.5 * 0.396, abs=1e-3)
    assert node._imu_integrated_s == pytest.approx(0.396, abs=1e-6)
    assert node._imu_skipped_s == 0.0


def test_a_stall_is_not_integrated_and_is_reported(node):
    node._imu_cb(_imu(1_000_000, gyro_z=0.5))
    node._imu_cb(_imu(1_004_000, gyro_z=0.5))  # 4 ms integrated
    yaw_before = _yaw(node)
    node._imu_cb(_imu(3_004_000, gyro_z=0.5))  # 2 s stall
    assert _yaw(node) == pytest.approx(yaw_before)  # no guessing across the gap
    assert node._imu_skipped_s == pytest.approx(2.0)
    node._imu_cb(_imu(3_008_000, gyro_z=0.5))  # integration resumes
    assert _yaw(node) > yaw_before


def test_imu_and_image_callbacks_run_in_separate_callback_groups(node):
    # The fix for yaw drift during heavy stereo processing: a shared group
    # would serialize the IMU behind image processing again.
    assert node._imu_group is not node._image_group


def test_attitude_messages_pull_a_drifted_heading_toward_the_autopilot(node):
    VehicleAttitude = px4_msgs_msg.VehicleAttitude
    t = 1_000_000
    for i in range(500):  # 2 s at 0.5 rad/s with no reference: 1 rad of yaw
        node._imu_cb(_imu(t + i * 4000, gyro_z=0.5))
    drifted = _yaw(node)
    assert drifted == pytest.approx(0.998, abs=0.01)
    # The autopilot says the heading is actually 0; attitude arrives at 100 Hz.
    node._vo_updates = 1  # VO has acquired: the filter is no longer re-seeded
    for i in range(1000):
        msg = VehicleAttitude()
        msg.timestamp = t + i * 10_000
        msg.q = [1.0, 0.0, 0.0, 0.0]
        node._px4_attitude_cb(msg)
    assert abs(_yaw(node)) < 0.05
