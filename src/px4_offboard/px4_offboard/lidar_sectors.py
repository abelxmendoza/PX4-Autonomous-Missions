"""
lidar_sectors.py — Gazebo GPU LiDAR → /px4_offboard/obstacle_dir

Reads gz.msgs.LaserScan from the x500 2D lidar, bins ranges into
front / left / right sectors, and publishes a std_msgs/String that
offboard_mission already consumes as a live sensor override.

Also republishes sensor_msgs/LaserScan on /px4_offboard/scan for RViz.

Run (after SITL with gz_x500_lidar_2d):
  ros2 run px4_offboard lidar_sectors
"""

from __future__ import annotations

import math
import os
import threading
import time

os.environ.setdefault("PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION", "python")

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan as RosLaserScan
from std_msgs.msg import Float32MultiArray, String

from .sensor_logic import sector_from_scan

try:
    from gz.msgs10.laserscan_pb2 import LaserScan as GzLaserScan
    from gz.transport13 import Node as GzNode

    _GZ_OK = True
except Exception as exc:  # noqa: BLE001
    _GZ_OK = False
    _GZ_IMPORT_ERROR = exc


def _sector_from_scan(
    ranges: list[float],
    angle_min: float,
    angle_step: float,
    range_min: float,
    range_max: float,
    trigger_m: float,
    front_deg: float,
    side_deg: float,
    side_trigger_m: float | None = None,
) -> tuple[str | None, dict[str, float]]:
    """Backward-compatible wrapper around the pure scan processor."""
    return sector_from_scan(
        ranges, angle_min, angle_step, range_min, range_max,
        trigger_m, front_deg, side_deg, side_trigger_m,
    )


class LidarSectors(Node):
    def __init__(self):
        super().__init__("lidar_sectors")

        self.declare_parameter(
            "gz_topic",
            "/world/obstacle_world/model/x500_lidar_2d_0/link/link/sensor/lidar_2d_v2/scan",
        )
        self.declare_parameter("trigger_m", 4.0)
        self.declare_parameter("side_trigger_m", 2.0)
        self.declare_parameter("ignore_inside_m", 0.6)
        self.declare_parameter("front_angle_deg", 35.0)
        self.declare_parameter("side_angle_deg", 90.0)
        self.declare_parameter("publish_hz", 20.0)
        self.declare_parameter("fallback_none", True)
        self.declare_parameter("confirm_ticks", 2)

        self.gz_topic = str(self.get_parameter("gz_topic").value)
        self.trigger_m = float(self.get_parameter("trigger_m").value)
        self.side_trigger_m = float(self.get_parameter("side_trigger_m").value)
        self.ignore_inside_m = float(self.get_parameter("ignore_inside_m").value)
        self.front_deg = float(self.get_parameter("front_angle_deg").value)
        self.side_deg = float(self.get_parameter("side_angle_deg").value)
        self.publish_hz = float(self.get_parameter("publish_hz").value)
        self.fallback_none = bool(self.get_parameter("fallback_none").value)
        self.confirm_ticks = max(1, int(self.get_parameter("confirm_ticks").value))

        self._lock = threading.Lock()
        self._latest: GzLaserScan | None = None
        self._last_label = "none"
        self._published_label = "none"
        self._pending_label = "none"
        self._pending_count = 0
        self._scan_count = 0
        self._gz = None

        self._pub_dir = self.create_publisher(String, "/px4_offboard/obstacle_dir", 10)
        self._pub_scan = self.create_publisher(RosLaserScan, "/px4_offboard/scan", 10)
        self._pub_mins = self.create_publisher(
            Float32MultiArray, "/px4_offboard/lidar_sector_mins", 10
        )

        if not _GZ_OK:
            self.get_logger().error(f"gz bindings unavailable: {_GZ_IMPORT_ERROR}")
        else:
            self._gz = GzNode()
            ok = self._gz.subscribe(GzLaserScan, self.gz_topic, self._gz_cb)
            self.get_logger().info(
                f"Subscribed gz LaserScan ({ok}) on {self.gz_topic}"
            )

        period = 1.0 / max(self.publish_hz, 1.0)
        self.create_timer(period, self._tick)
        self.get_logger().info(
            f"LidarSectors ready — front={self.trigger_m}m "
            f"sides={self.side_trigger_m}m "
            f"front±{self.front_deg}° sides±{self.side_deg}°"
        )

    def _gz_cb(self, msg: GzLaserScan):
        with self._lock:
            self._latest = msg
            self._scan_count += 1

    def _tick(self):
        with self._lock:
            msg = self._latest
            count = self._scan_count

        if msg is None:
            if self.fallback_none:
                out = String()
                out.data = "none"
                self._pub_dir.publish(out)
            return

        ranges = list(msg.ranges)
        # Ignore ultra-near returns (airframe / landing gear self-hits)
        effective_min = max(float(msg.range_min), self.ignore_inside_m)
        label, mins = _sector_from_scan(
            ranges,
            msg.angle_min,
            msg.angle_step,
            effective_min,
            msg.range_max,
            self.trigger_m,
            self.front_deg,
            self.side_deg,
            self.side_trigger_m,
        )

        candidate = label if label else "none"
        if candidate == self._pending_label:
            self._pending_count += 1
        else:
            self._pending_label = candidate
            self._pending_count = 1
        if self._pending_count >= self.confirm_ticks:
            self._published_label = candidate

        out = String()
        out.data = self._published_label
        self._pub_dir.publish(out)

        mins_msg = Float32MultiArray()
        mins_msg.data = [
            mins["front"] if mins["front"] < 1e8 else -1.0,
            mins["left"] if mins["left"] < 1e8 else -1.0,
            mins["right"] if mins["right"] < 1e8 else -1.0,
        ]
        self._pub_mins.publish(mins_msg)

        # ROS LaserScan for RViz
        ros = RosLaserScan()
        ros.header.stamp = self.get_clock().now().to_msg()
        ros.header.frame_id = "lidar_2d_link"
        ros.angle_min = float(msg.angle_min)
        ros.angle_max = float(msg.angle_max)
        ros.angle_increment = float(msg.angle_step)
        ros.range_min = float(msg.range_min)
        ros.range_max = float(msg.range_max)
        ros.ranges = [float(r) for r in ranges]
        self._pub_scan.publish(ros)

        if out.data != self._last_label:
            self.get_logger().info(
                f"LiDAR sector → {out.data}  "
                f"(F={mins_msg.data[0]:.1f} L={mins_msg.data[1]:.1f} "
                f"R={mins_msg.data[2]:.1f} scans={count})"
            )
            self._last_label = out.data


def main(args=None):
    rclpy.init(args=args)
    node = LidarSectors()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
