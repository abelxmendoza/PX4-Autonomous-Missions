"""ROS adapter tests; command safety and local failure handling without arming."""
import json
import time

import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("px4_msgs.msg")
from rclpy.parameter import Parameter
from px4_msgs.msg import VehicleLocalPosition
from std_msgs.msg import String
from px4_offboard.swarm_vehicle import SwarmVehicle


@pytest.fixture
def node():
    rclpy.init()
    n = SwarmVehicle(namespace="px4_1")
    yield n
    n.destroy_node()
    rclpy.shutdown()


def command(node, seq=1, **kwargs):
    data = dict(vehicle="px4_1", vehicle_session=node.session, session="coordinator",
                seq=seq, sent=time.monotonic(), action="move", target=[4, 0, -3])
    data.update(kwargs)
    return String(data=json.dumps(data))


def ready(node):
    now = time.monotonic()
    node.state = "READY"
    node.position = (0, 0, -3)
    node.setpoint = node.position
    node.goal = node.position
    node.calibration = (0, 0, 0)
    node.position_valid = True
    node.position_stamp = node.mode_stamp = node.land_stamp = now
    node.armed = node.offboard = True
    node.landed = False


def test_namespaced_publishers_and_target_id(node):
    assert node.command_pub.topic_name == "/px4_1/fmu/in/vehicle_command"
    assert node.status_pub.topic_name == "/px4_1/swarm/telemetry"
    assert node.system_id == 2


@pytest.mark.parametrize("changes", [
    {"vehicle": "px4_2"}, {"vehicle_session": "old"}, {"sent": 0},
    {"target": [float("nan"), 0, -3]}, {"target": [4, 0, -30]},
    {"target": [100, 0, -3]}, {"action": "arm"}, {"seq": True},
])
def test_bad_commands_do_not_renew_watchdog(node, changes):
    ready(node)
    node._command(command(node, **changes))
    assert node.command_seq == -1
    assert node.goal == (0, 0, -3)


def test_replayed_command_cannot_move_or_renew(node):
    ready(node)
    node._command(command(node))
    stamp = node.command_stamp
    node._command(command(node, target=[8, 0, -3]))
    assert node.goal == (4, 0, -3)
    assert node.command_stamp == stamp


def test_coordinator_timeout_holds_then_lands(node):
    ready(node)
    node._command(command(node))
    node.command_stamp = time.monotonic() - 1
    node._tick()
    assert node.goal == node.position
    assert node.state != "LANDING"
    node.command_stamp = time.monotonic() - 3
    node._tick()
    assert node.state == "LANDING"
    assert "timeout" in node.fault
    node._command(command(node, seq=2))
    assert node.state == "LANDING"


def test_restart_cannot_resume_flight(node):
    ready(node)
    node._command(command(node))
    node._command(command(node, seq=2, session="new"))
    assert node.state == "LANDING"
    assert "restarted" in node.fault


def test_estimator_reset_invalidates_world_frame(node):
    ready(node)
    node.resets = (0, 0)
    msg = VehicleLocalPosition()
    msg.timestamp = 1
    msg.xy_valid = msg.z_valid = msg.v_xy_valid = msg.v_z_valid = True
    msg.x, msg.y, msg.z = 4.0, 0.0, -3.0
    msg.xy_reset_counter = 1
    node._position(msg)
    assert not node.frame_valid
    assert node.state == "LANDING"


def test_landing_needs_fresh_land_and_disarm(node):
    ready(node)
    node.state = "LANDING"
    node.landed = True
    node._tick()
    assert node.state == "LANDING"  # still armed
    node.armed = False
    node.land_stamp = time.monotonic() - 3
    node._tick()
    assert node.state == "LANDING"  # stale landing evidence
    node.land_stamp = time.monotonic()
    node._tick()
    assert node.state == "LANDED"
