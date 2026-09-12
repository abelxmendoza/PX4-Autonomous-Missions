"""Vehicle-isolated trail rendering and shared-world telemetry conversion."""
import json
import time
import pytest
rclpy = pytest.importorskip('rclpy')
pytest.importorskip('px4_msgs.msg')
from rclpy.parameter import Parameter
from std_msgs.msg import String
from px4_offboard.flight_trail import FlightTrail


def test_swarm_trails_use_shared_world_positions_and_isolated_names():
    rclpy.init()
    nodes = []
    try:
        for i in (1, 2):
            node = FlightTrail(namespace=f'px4_{i}', parameter_overrides=[
                Parameter('input_mode', value='swarm'), Parameter('gz_markers', value=False),
                Parameter('gz_crumbs', value=False)])
            nodes.append(node)
        first, second = nodes
        message = String(data=json.dumps(dict(vehicle='px4_2', session='run', seq=2,
            sent=time.monotonic(), position=[4, 14, -3], velocity=[0, 0, 0],
            state='MOVING', valid=True, armed=True, landed=False)))
        first._telemetry_cb(message)
        second._telemetry_cb(message)
        assert first._points_enu == []
        assert second._points_enu == [(14.0, 4.0, 3.0)]
        assert second._pub_path.topic_name == '/px4_2/px4_offboard/flight_path'
        assert first.visual_prefix != second.visual_prefix
        second._clear_trail()
        assert first._points_enu == []
        assert second._points_enu == []
    finally:
        for node in nodes:
            node.destroy_node()
        rclpy.shutdown()
