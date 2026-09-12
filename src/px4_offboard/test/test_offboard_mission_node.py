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

import csv
import math
import time

import pytest

rclpy = pytest.importorskip("rclpy")
px4_msgs_msg = pytest.importorskip("px4_msgs.msg")

from px4_offboard.mission_state import State  # noqa: E402
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


def test_climb_avoidance_target_is_locked_not_recomputed_each_tick(node):
    # Regression for a real SITL bug: the climb-avoidance branch recomputed
    # blocking height and climb-vs-sidestep direction from the *current*
    # position on every tick. Since the vehicle's position that tick was
    # itself the result of chasing the previous tick's decision, a small
    # oscillation could feed back into the next tick's decision — observed
    # in a real flight as the setpoint zig-zagging (including a commanded
    # negative altitude) until the stuck-in-avoidance failsafe landed it.
    # The fix decides once per obstacle encounter and holds that decision.
    # Both current positions below share the same fixed target and "front"
    # classification, but the straight-line segment to it clips a
    # *different* real obstacle from OBSTACLE_BOXES in each case — OB1
    # (11.5 m tall, forces can_climb=False -> sidestep-right, since no live
    # sensor data makes _lateral_blocked always read clear) from due south,
    # vs. only OB2 (6 m tall, can_climb=True -> climb) from due east. An
    # unlocked recomputation genuinely picks a different response for each;
    # this is what makes the test able to fail.
    node.avoidance_strategy = "climb"
    node.obstacle_source = "hybrid"
    target = [20.0, -6.0, -3.0]

    node.current_x, node.current_y, node.current_z = 0.0, -6.0, -3.0
    node._apply_avoidance(list(target), "front")
    first_target = node._climb_target
    assert first_target is not None
    assert first_target[2] == -3.0, "expected the sidestep branch (no altitude change)"

    # Move the vehicle to where a fresh computation would choose climb
    # instead, and call again with the same obstacle classification: the
    # locked decision must not change.
    node.current_x, node.current_y, node.current_z = 0.0, 20.0, -3.4
    node._apply_avoidance(list(target), "front")
    assert node._climb_target == first_target

    # Once the obstacle clears, the lock releases for a future encounter.
    node._apply_avoidance(list(target), None)
    assert node._climb_target is None


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


def test_gps_denied_failsafe_event_is_captured_in_the_log(tmp_path):
    # End-to-end regression test for a real bug found while reviewing the
    # GPS-denied feature: LocalizationStateMachine's LOC_FAILSAFE event is
    # one-shot (sticky-to-None on every later tick), and _control_loop used
    # to `return` — past every state block that calls _log_row — the moment
    # it saw loc.failsafe, so no CSV row ever recorded the event that
    # actually triggered the failsafe. REQ-GPS-POLICY-01 (vv_harness.py)
    # then always passed via its "no events" early-out, even in the exact
    # deny-inject -> hold/land case it exists to verify. Drives the real
    # control loop (not just the pure LocalizationStateMachine) and reads
    # the real CSV back to prove the event now lands in the log.
    rclpy.init(args=[
        "--ros-args",
        "-p", f"log_dir:={tmp_path}",
        "-p", "gps_denied_enable:=true",
        "-p", "gps_deny_inject:=true",
        "-p", "gps_denied_action:=hold",
    ])
    try:
        n = OffboardMission()
        # Skip the arming sequence — jump straight to a flight state so the
        # localization failsafe branch in _control_loop is actually reached
        # (it's gated on state not in {PREFLIGHT, FAILSAFE, LANDING}).
        n._state_machine.state = State.MOVE
        n._have_position = True
        n._pos_stamp = time.monotonic()
        n._last_wp_progress_t = time.monotonic()
        # Inside the configured GPS-denied prism (default N[18,28] E[-5,5]
        # D[-12,0.5] — see config/offboard_mission.yaml).
        n.current_x, n.current_y, n.current_z = 23.0, 0.0, -5.0

        n._control_loop()  # should enter FAILSAFE and log this exact tick

        assert n._state == State.FAILSAFE

        log_files = list(tmp_path.glob("flight_log_mission_*.csv"))
        assert len(log_files) == 1
        with open(log_files[0], newline="") as f:
            rows = list(csv.DictReader(f))
        loc_events = [r["loc_event"] for r in rows if r["loc_event"]]
        assert "LOC_FAILSAFE" in loc_events

        n.destroy_node()
    finally:
        rclpy.shutdown()


def test_two_vehicle_topics_commands_and_logs_are_isolated(tmp_path):
    from rclpy.parameter import Parameter

    rclpy.init()
    nodes = []
    try:
        for instance in (1, 2):
            n = OffboardMission(
                namespace=f"px4_{instance}",
                parameter_overrides=[Parameter("target_system_id", value=instance + 1),
                                     Parameter("log_dir", value=str(tmp_path))],
            )
            nodes.append(n)
        first, second = nodes
        assert first._pub_cmd.topic_name == "/px4_1/fmu/in/vehicle_command"
        assert second._pub_cmd.topic_name == "/px4_2/fmu/in/vehicle_command"
        assert first._pub_sp.topic_name != second._pub_sp.topic_name
        for instance, n in enumerate(nodes, 1):
            assert n._pub_mission_status.topic_name == f"/px4_{instance}/px4_offboard/mission_status"
            assert all(s.topic_name.startswith(f"/px4_{instance}/") for s in n.subscriptions)
        assert first._log_file.name != second._log_file.name
        captured = []
        class Capture:
            def publish(self, message):
                captured.append(message)
        for n in nodes:
            n._pub_cmd = Capture()
            n._send_land()
        assert [m.target_system for m in captured] == [2, 3]
    finally:
        for n in nodes:
            n.destroy_node()
        rclpy.shutdown()


def test_zero_radius_mission_transitions_to_landing(tmp_path):
    rclpy.init(args=["--ros-args", "-p", f"log_dir:={tmp_path}",
                     "-p", "trajectory_mode:=circle", "-p", "circle_radius_m:=0.0",
                     "-p", "circle_period_s:=10.0", "-p", "max_orbits:=1.0"])
    try:
        n = OffboardMission()
        n._state_machine.state = State.MOVE
        assert n._next_target() == [0.0, 0.0, -n.hover_alt]
        n._mission_t = 10.0
        n._next_target()
        assert n._state == State.LANDING
        n.destroy_node()
    finally:
        rclpy.shutdown()
