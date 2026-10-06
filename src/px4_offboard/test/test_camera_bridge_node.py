"""camera_bridge publishes each frame exactly once, stamped with sensor time.

Regression for the stereo-VO timing defect: the bridge used to republish its
latest frame on every timer tick with a fresh wall-clock stamp, so frames were
duplicated or skipped and the stamps no longer described the exposures.
"""
from __future__ import annotations

import os

import pytest

# Same setting the nodes apply before importing gz message modules.
os.environ.setdefault("PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION", "python")

rclpy = pytest.importorskip("rclpy")
gz_image = pytest.importorskip("gz.msgs10.image_pb2")

from px4_offboard.camera_bridge import CameraBridge  # noqa: E402


class _Capture:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


@pytest.fixture
def bridge():
    rclpy.init()
    node = CameraBridge()
    node._pub, node._healthy_pub = _Capture(), _Capture()
    yield node
    node.destroy_node()
    rclpy.shutdown()


def _frame(sec: int, nsec: int):
    msg = gz_image.Image()
    msg.width, msg.height = 4, 2
    msg.pixel_format_type = gz_image.Image.DESCRIPTOR.fields_by_name[
        "pixel_format_type"
    ].enum_type.values_by_name["RGB_INT8"].number
    msg.data = bytes(4 * 2 * 3)
    msg.header.stamp.sec, msg.header.stamp.nsec = sec, nsec
    return msg


def test_each_frame_is_published_once_with_the_sensor_timestamp(bridge):
    bridge._image_cb(_frame(7, 250_000_000))
    for _ in range(5):  # many timer ticks, one frame
        bridge._tick()
    assert len(bridge._pub.messages) == 1
    stamp = bridge._pub.messages[0].header.stamp
    assert (stamp.sec, stamp.nanosec) == (7, 250_000_000)

    bridge._image_cb(_frame(7, 350_000_000))
    bridge._tick()
    bridge._tick()
    assert len(bridge._pub.messages) == 2
    stamp = bridge._pub.messages[1].header.stamp
    assert (stamp.sec, stamp.nanosec) == (7, 350_000_000)


def test_a_frame_without_a_sensor_stamp_falls_back_to_receive_time(bridge):
    bridge._image_cb(_frame(0, 0))
    bridge._tick()
    assert len(bridge._pub.messages) == 1
    stamp = bridge._pub.messages[0].header.stamp
    assert stamp.sec > 0 or stamp.nanosec > 0
