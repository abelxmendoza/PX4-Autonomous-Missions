"""ROS adapter: real IMU + real stereo VO -> PX4 external-vision odometry.

Subscribes to PX4's own ``SensorCombined`` (real body-frame gyro/accel,
already flowing through uXRCE-DDS) and this package's two ``camera_bridge``
instances (real rendered stereo frames), runs ``stereo_depth.py`` +
``ekf_fusion.py``, and publishes ``VehicleOdometry`` on the exact same
topic/QoS ``vio_bridge.py`` used -- so it is a drop-in alternative PX4
consumes identically, but built entirely from sensor data, never from
Gazebo ground truth.

Not yet wired into full_stack.launch.py's ``use_vio`` flag (that still
launches vio_bridge.py, the ground-truth-based approach): this needs a live
SITL verification pass first, since real VO+IMU fusion will have real drift
that the noise-bounded ground-truth approach didn't -- see bugs/ discipline
of verifying before replacing working, evidence-backed behavior. Launch
opt-in for now via ``use_sensor_fusion_vio:=true``.
"""

from __future__ import annotations

import os
import threading
import time

os.environ.setdefault("PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION", "python")

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import SensorCombined, VehicleOdometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

from .ekf_fusion import PoseVelocityEKF, quat_to_rotation_matrix
from .stereo_depth import StereoOdometry

try:
    import cv2

    _CV2_OK = True
except Exception:  # noqa: BLE001
    _CV2_OK = False


def _image_to_gray(msg: Image) -> np.ndarray:
    arr = np.frombuffer(msg.data, dtype=np.uint8)
    if msg.encoding in ("mono8", "8UC1"):
        return arr.reshape(msg.height, msg.width).copy()
    if msg.encoding == "rgb8":
        rgb = arr.reshape(msg.height, msg.width, 3)
        return cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
    if msg.encoding == "bgr8":
        bgr = arr.reshape(msg.height, msg.width, 3)
        return cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    raise ValueError(f"unsupported image encoding for stereo processing: {msg.encoding}")


class EkfFusionNode(Node):
    def __init__(self):
        super().__init__("ekf_fusion_node")
        self.declare_parameter("left_image_topic", "/px4_offboard/camera/image_raw")
        self.declare_parameter(
            "right_image_topic", "/px4_offboard/camera_secondary/image_raw"
        )
        self.declare_parameter("stereo_sync_tolerance_s", 0.03)
        self.declare_parameter("publish_hz", 20.0)
        self.declare_parameter("vo_velocity_std_mps", 0.15)
        self.declare_parameter("min_vo_dt_s", 0.02)
        self.declare_parameter("stale_timeout_s", 0.5)
        self.declare_parameter("publish_to_px4", False)

        self._sync_tol = float(self.get_parameter("stereo_sync_tolerance_s").value)
        self._publish_hz = float(self.get_parameter("publish_hz").value)
        self._vo_velocity_std = float(self.get_parameter("vo_velocity_std_mps").value)
        self._min_vo_dt = float(self.get_parameter("min_vo_dt_s").value)
        self._stale_timeout = float(self.get_parameter("stale_timeout_s").value)
        self._publish_to_px4 = bool(self.get_parameter("publish_to_px4").value)

        if not _CV2_OK:
            self.get_logger().error("OpenCV unavailable; ekf_fusion_node cannot run")

        self._lock = threading.Lock()
        self._ekf = PoseVelocityEKF()
        self._odometry = StereoOdometry()
        self._last_imu_t: float | None = None
        self._last_vo_t: float | None = None
        self._last_left: tuple[float, np.ndarray] | None = None
        self._last_right: tuple[float, np.ndarray] | None = None
        self._last_vo_update_t: float | None = None
        self._vo_updates = 0
        self._last_inliers = 0
        self._last_stats_t = time.monotonic()

        qos_px4 = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        qos_image = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(
            SensorCombined, "fmu/out/sensor_combined", self._imu_cb, qos_px4
        )
        self.create_subscription(
            Image,
            str(self.get_parameter("left_image_topic").value),
            self._left_image_cb,
            qos_image,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter("right_image_topic").value),
            self._right_image_cb,
            qos_image,
        )

        self._pub = self.create_publisher(
            VehicleOdometry, "/fmu/in/vehicle_visual_odometry", qos_px4
        )
        self._vo_pose_pub = self.create_publisher(
            PoseStamped, "/px4_offboard/vo_pose", 10
        )
        self._healthy_pub = self.create_publisher(
            Bool, "/px4_offboard/sensor_fusion_healthy", 10
        )
        self.create_timer(1.0 / max(self._publish_hz, 1.0), self._publish_tick)
        self.get_logger().info(
            f"ekf_fusion_node ready — publish_to_px4={self._publish_to_px4}"
        )

    def _imu_cb(self, msg: SensorCombined) -> None:
        now = time.monotonic()
        gyro = np.array(msg.gyro_rad, dtype=float)
        accel = np.array(msg.accelerometer_m_s2, dtype=float)
        with self._lock:
            dt = now - self._last_imu_t if self._last_imu_t is not None else None
            self._last_imu_t = now
            if dt is not None and 0.0 < dt < self._stale_timeout:
                self._ekf.predict(gyro, accel, dt)

    def _left_image_cb(self, msg: Image) -> None:
        self._handle_image(msg, is_left=True)

    def _right_image_cb(self, msg: Image) -> None:
        self._handle_image(msg, is_left=False)

    def _handle_image(self, msg: Image, is_left: bool) -> None:
        if not _CV2_OK:
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        try:
            gray = _image_to_gray(msg)
        except ValueError as exc:
            self.get_logger().error(str(exc), throttle_duration_sec=5.0)
            return

        with self._lock:
            if is_left:
                self._last_left = (stamp, gray)
            else:
                self._last_right = (stamp, gray)
            left, right = self._last_left, self._last_right
            if left is None or right is None:
                return
            if abs(left[0] - right[0]) > self._sync_tol:
                return
            left_t, left_gray = left
            _, right_gray = right
            # Consume both so we don't re-process the same pair on the next
            # single-camera callback.
            self._last_left = None
            self._last_right = None

        motion = self._odometry.process(left_gray, right_gray, left_t)
        if motion is None:
            return
        now = time.monotonic()
        with self._lock:
            dt_vo = (
                now - self._last_vo_t if self._last_vo_t is not None else None
            )
            self._last_vo_t = now
            if dt_vo is None or dt_vo < self._min_vo_dt:
                return
            r_body_to_world = quat_to_rotation_matrix(self._ekf.quat)
            velocity_world = r_body_to_world @ (motion.translation_m / dt_vo)
            self._ekf.update_velocity(velocity_world, self._vo_velocity_std)
            self._ekf.apply_vo_attitude_correction(motion.rotation_matrix)
            self._last_vo_update_t = now
            self._vo_updates += 1
            self._last_inliers = int(motion.inlier_count)

    def _publish_tick(self) -> None:
        now = time.monotonic()
        with self._lock:
            healthy = (
                self._last_vo_update_t is not None
                and now - self._last_vo_update_t <= self._stale_timeout
                and self._last_imu_t is not None
                and now - self._last_imu_t <= self._stale_timeout
            )
            state = self._ekf.state()
            vo_updates = self._vo_updates
            inliers = self._last_inliers
            imu_age = None if self._last_imu_t is None else now - self._last_imu_t
            vo_age = (
                None
                if self._last_vo_update_t is None
                else now - self._last_vo_update_t
            )
        self._healthy_pub.publish(Bool(data=healthy))

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "map"
        pose.pose.position.x = float(state.position_m[0])
        pose.pose.position.y = float(state.position_m[1])
        pose.pose.position.z = float(state.position_m[2])
        pose.pose.orientation.w = float(state.quat_wxyz[0])
        pose.pose.orientation.x = float(state.quat_wxyz[1])
        pose.pose.orientation.y = float(state.quat_wxyz[2])
        pose.pose.orientation.z = float(state.quat_wxyz[3])
        self._vo_pose_pub.publish(pose)

        if now - self._last_stats_t >= 2.0:
            self._last_stats_t = now
            self.get_logger().info(
                "fusion "
                f"healthy={healthy} vo_updates={vo_updates} inliers={inliers} "
                f"imu_age={imu_age if imu_age is None else round(imu_age, 3)}s "
                f"vo_age={vo_age if vo_age is None else round(vo_age, 3)}s "
                f"ned=({state.position_m[0]:.2f},{state.position_m[1]:.2f},"
                f"{state.position_m[2]:.2f})"
            )

        if not healthy or not self._publish_to_px4:
            return

        timestamp = self.get_clock().now().nanoseconds // 1000
        msg = VehicleOdometry()
        msg.timestamp = timestamp
        msg.timestamp_sample = timestamp
        msg.pose_frame = VehicleOdometry.POSE_FRAME_NED
        msg.position = [float(v) for v in state.position_m]
        msg.q = [float(v) for v in state.quat_wxyz]
        msg.velocity_frame = VehicleOdometry.VELOCITY_FRAME_NED
        msg.velocity = [float(v) for v in state.velocity_mps]
        msg.angular_velocity = [float("nan")] * 3
        msg.position_variance = [
            float(state.position_cov[0, 0]),
            float(state.position_cov[1, 1]),
            float(state.position_cov[2, 2]),
        ]
        msg.orientation_variance = [float("nan")] * 3
        msg.velocity_variance = [
            float(state.position_cov[3, 3]),
            float(state.position_cov[4, 4]),
            float(state.position_cov[5, 5]),
        ]
        msg.reset_counter = 0
        msg.quality = 100
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = EkfFusionNode()
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
