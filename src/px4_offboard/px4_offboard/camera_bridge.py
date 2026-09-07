"""Bridge the Gazebo forward camera into ROS as sensor_msgs/Image.

x500_lidar_2d (models/x500_lidar_2d/model.sdf, this repo's tracked override)
merges in PX4's stock ``mono_cam`` sensor alongside the 2D LiDAR, giving the
vehicle a real forward-facing rendered camera in Gazebo. This node republishes
those frames as ROS ``sensor_msgs/Image`` on ``/px4_offboard/camera/image_raw``
so RViz, ``image_view``, or a future vision node can consume them — mirroring
how ``vio_bridge.py`` and ``lidar_sectors.py`` subscribe to Gazebo sensors
directly via gz-transport rather than depending on ``ros_gz_bridge``.
"""

from __future__ import annotations

import os
import threading
import time

os.environ.setdefault("PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION", "python")

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

from .camera_frame import UnsupportedPixelFormat, gz_pixel_format_to_ros_fields

try:
    from gz.msgs10.image_pb2 import Image as GzImage
    from gz.transport13 import Node as GzNode

    _GZ_OK = True
except Exception as exc:  # noqa: BLE001
    _GZ_OK = False
    _GZ_IMPORT_ERROR = exc


class CameraBridge(Node):
    def __init__(self):
        super().__init__("camera_bridge")
        self.declare_parameter(
            "gz_topic",
            "/world/obstacle_world/model/x500_lidar_2d_0/link/camera_link/sensor/imager/image",
        )
        self.declare_parameter("frame_id", "camera_link")
        self.declare_parameter("stale_timeout_s", 1.0)

        self.gz_topic = str(self.get_parameter("gz_topic").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.stale_timeout = float(self.get_parameter("stale_timeout_s").value)
        self._lock = threading.Lock()
        self._last_frame_t: float | None = None

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._pub = self.create_publisher(Image, "/px4_offboard/camera/image_raw", qos)
        self._healthy_pub = self.create_publisher(
            Bool, "/px4_offboard/camera_healthy", 10
        )
        self._frame_count = 0

        if not _GZ_OK:
            self.get_logger().error(f"Gazebo bindings unavailable: {_GZ_IMPORT_ERROR}")
            self._gz = None
        else:
            self._gz = GzNode()
            ok = self._gz.subscribe(GzImage, self.gz_topic, self._image_cb)
            self.get_logger().info(
                f"camera bridge subscribed={ok} topic={self.gz_topic}"
            )
        self._healthy_pub.publish(Bool(data=False))
        self.create_timer(0.2, self._check_staleness)

    def _image_cb(self, msg: GzImage):
        pixel_format_name = GzImage.DESCRIPTOR.fields_by_name[
            "pixel_format_type"
        ].enum_type.values_by_number[msg.pixel_format_type].name
        try:
            fields = gz_pixel_format_to_ros_fields(pixel_format_name, msg.width)
        except UnsupportedPixelFormat as exc:
            self.get_logger().error(str(exc))
            self._healthy_pub.publish(Bool(data=False))
            return

        out = Image()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.frame_id
        out.height = msg.height
        out.width = msg.width
        out.encoding = fields.encoding
        out.is_bigendian = fields.is_bigendian
        out.step = fields.step
        out.data = msg.data
        self._pub.publish(out)
        self._frame_count += 1
        with self._lock:
            self._last_frame_t = time.monotonic()
        self._healthy_pub.publish(Bool(data=True))

    def _check_staleness(self):
        with self._lock:
            last = self._last_frame_t
        if last is not None and time.monotonic() - last > self.stale_timeout:
            self._healthy_pub.publish(Bool(data=False))


def main(args=None):
    rclpy.init(args=args)
    node = CameraBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
