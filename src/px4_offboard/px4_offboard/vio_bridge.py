"""Gazebo pose-backed visual odometry for PX4 GPS-denied SITL demos.

The simulated LiDAR scan contains its Gazebo world pose.  This node converts
that independent simulator truth from ENU to a local NED frame and publishes
PX4 ``VehicleOdometry``.  PX4's EKF consumes it as external vision; the
mission controller never uses this truth topic directly for guidance.
"""

from __future__ import annotations

import os
import threading
import time

os.environ.setdefault("PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION", "python")

import rclpy
from px4_msgs.msg import VehicleOdometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

from .frame_transforms import enu_to_ned

try:
    from gz.msgs10.laserscan_pb2 import LaserScan as GzLaserScan
    from gz.transport13 import Node as GzNode

    _GZ_OK = True
except Exception as exc:  # noqa: BLE001
    _GZ_OK = False
    _GZ_IMPORT_ERROR = exc


class VioBridge(Node):
    def __init__(self):
        super().__init__("vio_bridge")
        self.declare_parameter(
            "gz_topic",
            "/world/obstacle_world/model/x500_lidar_2d_0/link/link/"
            "sensor/lidar_2d_v2/scan",
        )
        self.declare_parameter("publish_hz", 20.0)
        self.declare_parameter("position_std_m", 0.05)
        self.declare_parameter("velocity_std_mps", 0.08)
        self.declare_parameter("stale_timeout_s", 0.5)

        self.gz_topic = str(self.get_parameter("gz_topic").value)
        self.publish_hz = float(self.get_parameter("publish_hz").value)
        self.position_var = float(self.get_parameter("position_std_m").value) ** 2
        self.velocity_var = float(self.get_parameter("velocity_std_mps").value) ** 2
        self.stale_timeout = float(self.get_parameter("stale_timeout_s").value)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._pub = self.create_publisher(
            VehicleOdometry, "/fmu/in/vehicle_visual_odometry", qos
        )
        self._healthy_pub = self.create_publisher(Bool, "/px4_offboard/vio_healthy", 10)
        self._lock = threading.Lock()
        self._latest: tuple[float, list[float]] | None = None
        self._origin: list[float] | None = None
        self._previous: tuple[float, list[float]] | None = None

        if not _GZ_OK:
            self.get_logger().error(f"Gazebo bindings unavailable: {_GZ_IMPORT_ERROR}")
            self._gz = None
        else:
            self._gz = GzNode()
            ok = self._gz.subscribe(GzLaserScan, self.gz_topic, self._scan_cb)
            self.get_logger().info(f"VIO source subscribed={ok} topic={self.gz_topic}")

        self.create_timer(1.0 / max(self.publish_hz, 1.0), self._tick)

    def _scan_cb(self, scan: GzLaserScan):
        pose = scan.world_pose.position
        ned = enu_to_ned(float(pose.x), float(pose.y), float(pose.z))
        now = time.monotonic()
        with self._lock:
            if self._origin is None:
                self._origin = list(ned)
            local = [ned[i] - self._origin[i] for i in range(3)]
            self._latest = (now, local)

    def _tick(self):
        with self._lock:
            latest = self._latest
        healthy = latest is not None and time.monotonic() - latest[0] <= self.stale_timeout
        self._healthy_pub.publish(Bool(data=healthy))
        if not healthy or latest is None:
            return

        sample_t, position = latest
        velocity = [float("nan")] * 3
        if self._previous is not None:
            previous_t, previous_position = self._previous
            dt = sample_t - previous_t
            if 0.01 <= dt <= self.stale_timeout:
                velocity = [
                    (position[i] - previous_position[i]) / dt for i in range(3)
                ]
        self._previous = (sample_t, list(position))

        timestamp = self.get_clock().now().nanoseconds // 1000
        msg = VehicleOdometry()
        msg.timestamp = timestamp
        msg.timestamp_sample = timestamp
        msg.pose_frame = VehicleOdometry.POSE_FRAME_NED
        msg.position = [float(value) for value in position]
        msg.q = [float("nan")] * 4
        msg.velocity_frame = VehicleOdometry.VELOCITY_FRAME_NED
        msg.velocity = velocity
        msg.angular_velocity = [float("nan")] * 3
        msg.position_variance = [self.position_var] * 3
        msg.orientation_variance = [float("nan")] * 3
        msg.velocity_variance = [self.velocity_var] * 3
        msg.reset_counter = 0
        msg.quality = 100
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = VioBridge()
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
