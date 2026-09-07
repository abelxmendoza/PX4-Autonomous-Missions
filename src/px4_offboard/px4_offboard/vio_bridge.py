"""Gazebo pose-backed visual odometry for PX4 GPS-denied SITL demos.

Gazebo's LiDAR ``LaserScan.world_pose`` is the sensor pose in the vehicle
frame (≈ origin), not the world.  This node reads the x500 model pose from
``/world/obstacle_world/pose/info``, converts ENU to local NED, and publishes
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

from .frame_transforms import named_enu_pose_to_ned
from .vio_noise import VioDriftConfig, VioDriftModel

try:
    from gz.msgs10.pose_v_pb2 import Pose_V as GzPoseV
    from gz.transport13 import Node as GzNode

    _GZ_OK = True
except Exception as exc:  # noqa: BLE001
    _GZ_OK = False
    _GZ_IMPORT_ERROR = exc


class VioBridge(Node):
    def __init__(self):
        super().__init__("vio_bridge")
        self.declare_parameter("gz_topic", "/world/obstacle_world/pose/info")
        self.declare_parameter("gz_model_name", "x500_lidar_2d_0")
        self.declare_parameter("publish_hz", 20.0)
        self.declare_parameter("position_std_m", 0.05)
        self.declare_parameter("velocity_std_mps", 0.08)
        self.declare_parameter("stale_timeout_s", 0.5)
        self.declare_parameter("drift_std_m_per_sqrt_s", 0.02)
        self.declare_parameter("drift_revert_rate_hz", 0.05)
        self.declare_parameter("max_drift_bias_m", 1.5)

        self.gz_topic = str(self.get_parameter("gz_topic").value)
        self.gz_model_name = str(self.get_parameter("gz_model_name").value)
        self.publish_hz = float(self.get_parameter("publish_hz").value)
        self.velocity_var = float(self.get_parameter("velocity_std_mps").value) ** 2
        self.stale_timeout = float(self.get_parameter("stale_timeout_s").value)
        self._drift = VioDriftModel(
            VioDriftConfig(
                position_std_m=float(self.get_parameter("position_std_m").value),
                drift_std_m_per_sqrt_s=float(
                    self.get_parameter("drift_std_m_per_sqrt_s").value
                ),
                drift_revert_rate_hz=float(
                    self.get_parameter("drift_revert_rate_hz").value
                ),
                max_bias_m=float(self.get_parameter("max_drift_bias_m").value),
            )
        )
        self._last_drift_t: float | None = None

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
        self._previous: tuple[float, list[float]] | None = None

        if not _GZ_OK:
            self.get_logger().error(f"Gazebo bindings unavailable: {_GZ_IMPORT_ERROR}")
            self._gz = None
        else:
            self._gz = GzNode()
            ok = self._gz.subscribe(GzPoseV, self.gz_topic, self._pose_cb)
            self.get_logger().info(
                f"VIO source subscribed={ok} topic={self.gz_topic} "
                f"model={self.gz_model_name}"
            )

        self.create_timer(1.0 / max(self.publish_hz, 1.0), self._tick)

    def _pose_cb(self, msg: GzPoseV):
        poses = [
            (pose.name, pose.position.x, pose.position.y, pose.position.z)
            for pose in msg.pose
        ]
        ned = named_enu_pose_to_ned(poses, self.gz_model_name)
        if ned is None:
            return
        now = time.monotonic()
        with self._lock:
            self._latest = (now, ned)

    def _tick(self):
        with self._lock:
            latest = self._latest
        healthy = latest is not None and time.monotonic() - latest[0] <= self.stale_timeout
        self._healthy_pub.publish(Bool(data=healthy))
        if not healthy or latest is None:
            return

        sample_t, true_position = latest
        dt_drift = (
            sample_t - self._last_drift_t
            if self._last_drift_t is not None
            else 1.0 / max(self.publish_hz, 1.0)
        )
        self._last_drift_t = sample_t
        self._drift.step(dt_drift)
        position = self._drift.apply(true_position)

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
        msg.position_variance = [float(v) for v in self._drift.reported_variance()]
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
