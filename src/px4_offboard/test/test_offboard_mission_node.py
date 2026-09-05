"""Node-level tests for OffboardMission — the one file in this package with
no prior test coverage, since it's a live rclpy.Node rather than pure logic.

Unlike the other test files, these need a sourced ROS 2 environment (rclpy +
the built px4_msgs package), so they are NOT part of the lightweight
`pure-python-tests` CI job (which deliberately has no ROS dependency — see
.github/workflows/unit-tests.yml). Run locally instead:

  source /opt/ros/humble/setup.bash
  source install/setup.bash   # from the repo root, after colcon build
  PYTHONPATH=src/px4_offboard python3 -m pytest src/px4_offboard/test/test_offboard_mission_node.py -v

`pytest.importorskip` makes this file a no-op (skipped, not failed) in any
environment without rclpy/px4_msgs, so it's safe even if picked up by a
broader `pytest test/` run.

Scope: node construction (parameter declaration/QoS/pub-sub wiring), the
position/attitude callbacks that turn PX4 messages into node state, and the
CSV-logger error handling added to survive a full disk or unwritable log
directory without crashing node startup or the control-loop timer callback.
"""

from __future__ import annotations

import math

import pytest

rclpy = pytest.importorskip("rclpy")
px4_msgs_msg = pytest.importorskip("px4_msgs.msg")

from px4_offboard.offboard_mission import OffboardMission  # noqa: E402

VehicleLocalPosition = px4_msgs_msg.VehicleLocalPosition
VehicleAttitude = px4_msgs_msg.VehicleAttitude


@pytest.fixture
def node(tmp_path):
    rclpy.init(args=["--ros-args", "-p", f"log_dir:={tmp_path}"])
    n = OffboardMission()
    yield n
    n.destroy_node()
    rclpy.shutdown()


def _quat_from_yaw(yaw_rad: float) -> list[float]:
    """[w, x, y, z] for a pure yaw rotation — matches PX4's Hamilton FRD->NED q."""
    half = yaw_rad / 2.0
    return [math.cos(half), 0.0, 0.0, math.sin(half)]


def test_node_constructs_and_declares_all_parameters(node):
    # A construction-time crash here is exactly the class of bug the
    # config/offboard_mission.yaml <-> declare_parameter sync fix targeted:
    # if the code ever declares a parameter with a type that conflicts with
    # a value supplied via a params file, rclpy raises during __init__.
    assert node.get_name() == "offboard_mission"
    assert node._log_writer is not None  # tmp_path is writable


def test_position_callback_updates_state(node):
    msg = VehicleLocalPosition()
    msg.x, msg.y, msg.z = 12.5, -3.25, -8.0
    msg.vx, msg.vy, msg.vz = 1.5, -0.5, 0.2

    node._position_callback(msg)

    assert node.current_x == 12.5
    assert node.current_y == -3.25
    assert node.current_z == -8.0
    assert node.current_vx == 1.5
    assert node.current_vy == -0.5
    assert node.current_vz == 0.2
    assert node._have_position is True


def test_attitude_callback_identity_quaternion_is_level(node):
    msg = VehicleAttitude()
    msg.q = [1.0, 0.0, 0.0, 0.0]  # no rotation

    node._attitude_callback(msg)

    assert node.current_roll == pytest.approx(0.0, abs=1e-9)
    assert node.current_pitch == pytest.approx(0.0, abs=1e-9)
    assert node.current_yaw == pytest.approx(0.0, abs=1e-9)
    assert node._have_attitude is True


def test_attitude_callback_recovers_known_yaw(node):
    msg = VehicleAttitude()
    msg.q = _quat_from_yaw(math.radians(90.0))

    node._attitude_callback(msg)

    # abs=1e-4 deg, not 1e-6: msg.q is a float32[4] field, so assigning a
    # float64-computed quaternion loses precision on the round-trip through
    # the message — this is float32 rounding, not a bug in the conversion.
    assert math.degrees(node.current_yaw) == pytest.approx(90.0, abs=1e-4)
    assert node.current_roll == pytest.approx(0.0, abs=1e-6)
    assert node.current_pitch == pytest.approx(0.0, abs=1e-6)


def test_log_row_disables_logging_on_write_failure(node):
    # Regression test for the OSError guard added around _log_writer.writerow
    # / _log_file.flush() — a full-disk mid-flight must not propagate out of
    # the control-loop timer callback (which would abort rclpy.spin() and
    # stop setpoint streaming).
    class ExplodingWriter:
        def writerow(self, row):
            raise OSError("disk full (simulated)")

    node._log_writer = ExplodingWriter()

    node._log_row([0.0, 0.0, -5.0], obstacle=None)  # must not raise

    assert node._log_writer is None
    assert node._log_file is None


def test_log_row_is_a_noop_once_logging_is_disabled(node):
    node._log_writer = None
    node._log_row([0.0, 0.0, -5.0], obstacle=None)  # must not raise


def test_open_log_survives_unwritable_directory(tmp_path):
    # A regular file where a directory is expected makes mkdir() fail with
    # OSError (FileExistsError, a subclass) — simulating an unwritable log
    # path without needing actual filesystem permission tricks.
    blocked = tmp_path / "blocked"
    blocked.write_text("not a directory")

    rclpy.init(args=["--ros-args", "-p", f"log_dir:={blocked}"])
    try:
        n = OffboardMission()
        assert n._log_writer is None
        assert n._log_file is None
        n.destroy_node()  # must not raise despite no open log file
    finally:
        rclpy.shutdown()
