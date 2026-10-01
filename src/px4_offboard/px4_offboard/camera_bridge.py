"""Bridge a Gazebo camera into ROS as sensor_msgs/Image.

x500_lidar_2d (models/x500_lidar_2d/model.sdf, this repo's tracked override)
merges in this repo's ``mono_cam`` plus ``mono_cam_right`` (same
intrinsics, 0.06m baseline) alongside the 2D LiDAR, giving the vehicle a real
forward-facing stereo pair in Gazebo. This node republishes one camera's raw
frames as ROS ``sensor_msgs/Image`` — mirroring how ``vio_bridge.py`` and
``lidar_sectors.py`` subscribe to Gazebo sensors directly via gz-transport
rather than depending on ``ros_gz_bridge``. Two instances of this same node
(different ``gz_topic``/``output_topic`` parameters, see
``full_stack.launch.py``'s ``use_stereo`` flag) bridge the left and right
cameras separately; ``stereo_depth.py`` subscribes to both.

The Gazebo callback only stores the latest frame. ROS publish happens on a
timer, matching ``lidar_sectors.py`` — publishing ``sensor_msgs/Image`` from
the gz-transport thread is unsafe under rclpy.

Gazebo camera sensors skip rendering unless a C++ gz-transport subscriber
is connected (``Publisher::HasConnections()``). These Python bindings
return True from ``subscribe()`` but do not increment that count, so
``full_stack.launch.py`` also starts ``scripts/gz_cam_sub`` whenever
cameras are enabled. LiDAR does not need the extra process because PX4's
C++ ``gz_bridge`` is already subscribed to the scan topic.
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
        self.declare_parameter("output_topic", "/px4_offboard/camera/image_raw")
        self.declare_parameter("healthy_topic", "/px4_offboard/camera_healthy")
        self.declare_parameter("publish_hz", 10.0)

        self.gz_topic = str(self.get_parameter("gz_topic").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.stale_timeout = float(self.get_parameter("stale_timeout_s").value)
        output_topic = str(self.get_parameter("output_topic").value)
        healthy_topic = str(self.get_parameter("healthy_topic").value)
        publish_hz = float(self.get_parameter("publish_hz").value)
        self._lock = threading.Lock()
        self._latest: tuple[int, int, object, bytes] | None = None
        self._last_frame_t: float | None = None
        self._gz_cb_errors = 0

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._pub = self.create_publisher(Image, output_topic, qos)
        self._healthy_pub = self.create_publisher(Bool, healthy_topic, 10)
        self._frame_count = 0
        self._published_count = 0
        self._last_stats_t = time.monotonic()

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
        self.create_timer(1.0 / max(publish_hz, 1.0), self._tick)

    def _image_cb(self, msg: GzImage):
        try:
            pixel_format_name = GzImage.DESCRIPTOR.fields_by_name[
                "pixel_format_type"
            ].enum_type.values_by_number[msg.pixel_format_type].name
            fields = gz_pixel_format_to_ros_fields(pixel_format_name, msg.width)
        except (KeyError, UnsupportedPixelFormat) as exc:
            self._gz_cb_errors += 1
            self.get_logger().error(str(exc), throttle_duration_sec=5.0)
            return
        except Exception as exc:  # noqa: BLE001
            self._gz_cb_errors += 1
            self.get_logger().error(
                f"gz image callback failed: {exc}", throttle_duration_sec=5.0
            )
            return

        payload = bytes(msg.data)
        with self._lock:
            self._latest = (int(msg.width), int(msg.height), fields, payload)
            self._frame_count += 1
            self._last_frame_t = time.monotonic()

    def _tick(self):
        now = time.monotonic()
        with self._lock:
            latest = self._latest
            last = self._last_frame_t
            count = self._frame_count
            errors = self._gz_cb_errors
        healthy = last is not None and now - last <= self.stale_timeout
        self._healthy_pub.publish(Bool(data=healthy))

        if latest is not None:
            width, height, fields, payload = latest
            out = Image()
            out.header.stamp = self.get_clock().now().to_msg()
            out.header.frame_id = self.frame_id
            out.height = height
            out.width = width
            out.encoding = fields.encoding
            out.is_bigendian = fields.is_bigendian
            out.step = fields.step
            out.data = payload
            self._pub.publish(out)
            self._published_count += 1

        if now - self._last_stats_t >= 2.0:
            self._last_stats_t = now
            self.get_logger().info(
                f"camera frames gz={count} ros={self._published_count} "
                f"healthy={healthy} cb_errors={errors} topic={self.gz_topic}"
            )


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
