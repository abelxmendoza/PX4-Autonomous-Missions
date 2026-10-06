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
from px4_msgs.msg import (
    SensorCombined,
    VehicleAttitude,
    VehicleLocalPosition,
    VehicleOdometry,
)
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32MultiArray

from .ekf_fusion import (
    DriftTracker,
    PoseVelocityEKF,
    message_dt_s,
    quat_to_rotation_matrix,
)
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
        # Process noise must cover attitude-error leakage, not just accelerometer
        # noise -- see PoseVelocityEKF.
        self.declare_parameter("accel_noise_std", 0.5)
        # Weight of VO rotation in the attitude estimate; 0 = gyro only. Off by
        # default: on the same mission, blending it in gave 10.6% final drift
        # (peak 14.5%) against 3.8% (peak 7.1%) gyro-only -- frame-to-frame PnP
        # rotation under-reads fast turns, and each gated frame pulls the
        # heading a little toward the under-read.
        self.declare_parameter("vo_attitude_blend", 0.0)
        # Time constant of the slow pull toward the autopilot's own attitude
        # estimate (IMU + magnetometer AHRS, not ground truth). 0 = pure gyro
        # integration, which wandered up to 17 deg of yaw in fast turns.
        self.declare_parameter("reference_attitude_tau_s", 2.0)
        # Extra floor on tracked features per VO update, on top of VoConfig's
        # own minimum; the Mahalanobis gate in the EKF handles the rest.
        self.declare_parameter("min_vo_inliers", 15)
        self.declare_parameter("stale_timeout_s", 0.5)
        self.declare_parameter("publish_to_px4", False)
        # Optional per-VO-update CSV (vo quality analysis); empty = off.
        self.declare_parameter("debug_csv_path", "")
        # Optional: save every stereo pair + PX4 pose/velocity for offline VO tuning.
        self.declare_parameter("debug_frames_dir", "")

        self._sync_tol = float(self.get_parameter("stereo_sync_tolerance_s").value)
        self._publish_hz = float(self.get_parameter("publish_hz").value)
        self._vo_velocity_std = float(self.get_parameter("vo_velocity_std_mps").value)
        self._min_vo_dt = float(self.get_parameter("min_vo_dt_s").value)
        self._min_vo_inliers = int(self.get_parameter("min_vo_inliers").value)
        self._stale_timeout = float(self.get_parameter("stale_timeout_s").value)
        self._publish_to_px4 = bool(self.get_parameter("publish_to_px4").value)

        if not _CV2_OK:
            self.get_logger().error("OpenCV unavailable; ekf_fusion_node cannot run")

        self._lock = threading.Lock()
        self._ekf = PoseVelocityEKF(
            accel_noise_std=float(self.get_parameter("accel_noise_std").value),
            vo_attitude_blend=float(self.get_parameter("vo_attitude_blend").value),
            reference_attitude_tau_s=float(
                self.get_parameter("reference_attitude_tau_s").value
            ),
        )
        self._odometry = StereoOdometry()
        self._last_imu_t: float | None = None
        self._last_imu_stamp_us: int | None = None
        self._imu_integrated_s = 0.0
        self._imu_skipped_s = 0.0
        self._imu_msgs = 0
        self._last_left: tuple[float, np.ndarray] | None = None
        self._last_right: tuple[float, np.ndarray] | None = None
        self._last_vo_update_t: float | None = None
        self._vo_updates = 0
        self._last_inliers = 0
        self._last_stats_t = time.monotonic()
        self._drift = DriftTracker()
        self._frames_dir = str(self.get_parameter("debug_frames_dir").value)
        self._frame_idx = 0
        self._frame_meta = None
        if self._frames_dir:
            import csv as _csv

            os.makedirs(self._frames_dir, exist_ok=True)
            self._frame_meta_file = open(os.path.join(self._frames_dir, "meta.csv"), "w", newline="")
            self._frame_meta = _csv.writer(self._frame_meta_file)
            self._frame_meta.writerow(
                ["idx", "stamp_s", "px_n", "px_e", "px_d", "pv_n", "pv_e", "pv_d",
                 "qw", "qx", "qy", "qz"]
            )
        self._debug_file = None
        self._debug_writer = None
        debug_path = str(self.get_parameter("debug_csv_path").value)
        if debug_path:
            import csv

            self._debug_file = open(debug_path, "w", newline="")
            self._debug_writer = csv.writer(self._debug_file)
            self._debug_writer.writerow(
                ["stamp_s", "dt_s", "inliers", "vo_vn", "vo_ve", "vo_vd",
                 "px4_vn", "px4_ve", "px4_vd", "ekf_vn", "ekf_ve", "ekf_vd",
                 "roll_deg", "pitch_deg", "ekf_yaw_deg", "px4_yaw_deg", "trans_body_x", "trans_body_y", "trans_body_z"]
            )
        self._px4_quat: np.ndarray | None = None
        self._last_attitude_stamp_us: int | None = None
        self._px4_pos: np.ndarray | None = None
        self._px4_vel: np.ndarray | None = None
        self._last_drift: dict[str, float] = {}
        self._dbg_vo_vel = np.zeros(3)
        self._dbg_vo_dt = 0.0

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

        # Stereo matching + PnP runs for ~0.1-0.3 s per frame. On a single
        # thread the IMU callback (and its depth-1 queue) starved behind it,
        # dropping gyro samples during manoeuvres -- the filter's yaw drifted
        # ~10 degrees exactly where VO was busiest. IMU and image callbacks
        # therefore live in separate callback groups on separate threads, and
        # the IMU queue is deep enough to ride out a scheduling hiccup.
        self._imu_group = MutuallyExclusiveCallbackGroup()
        self._image_group = MutuallyExclusiveCallbackGroup()
        qos_imu = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=200,
        )
        self.create_subscription(
            SensorCombined,
            "fmu/out/sensor_combined",
            self._imu_cb,
            qos_imu,
            callback_group=self._imu_group,
        )
        self.create_subscription(
            VehicleAttitude, "fmu/out/vehicle_attitude", self._px4_attitude_cb, qos_px4
        )
        self.create_subscription(
            VehicleLocalPosition,
            "fmu/out/vehicle_local_position",
            self._px4_position_cb,
            qos_px4,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter("left_image_topic").value),
            self._left_image_cb,
            qos_image,
            callback_group=self._image_group,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter("right_image_topic").value),
            self._right_image_cb,
            qos_image,
            callback_group=self._image_group,
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
        # [n, e, d, healthy, inliers, err_horiz_m, err_down_m, path_len_m,
        #  drift_frac, vo_updates] -- the mission logger records this.
        self._status_pub = self.create_publisher(
            Float32MultiArray, "/px4_offboard/vo_status", 10
        )
        self.create_timer(1.0 / max(self._publish_hz, 1.0), self._publish_tick)
        self.get_logger().info(
            f"ekf_fusion_node ready — publish_to_px4={self._publish_to_px4}"
        )

    def _px4_attitude_cb(self, msg: VehicleAttitude) -> None:
        with self._lock:
            self._px4_quat = np.array(msg.q, dtype=float)
            if self._vo_updates > 0 and np.all(np.isfinite(self._px4_quat)):
                dt = message_dt_s(
                    self._last_attitude_stamp_us, int(msg.timestamp), self._stale_timeout
                )
                if dt is not None:
                    self._ekf.apply_reference_attitude(self._px4_quat, dt)
            self._last_attitude_stamp_us = int(msg.timestamp)
            self._maybe_seed_from_px4()

    def _px4_position_cb(self, msg: VehicleLocalPosition) -> None:
        with self._lock:
            self._px4_pos = np.array([msg.x, msg.y, msg.z], dtype=float)
            self._px4_vel = np.array([msg.vx, msg.vy, msg.vz], dtype=float)
            self._maybe_seed_from_px4()

    def _maybe_seed_from_px4(self) -> None:
        # Until VO has acquired, track PX4's own estimate (a real vehicle
        # has this on the ground). Once the first VO update lands the filter
        # runs on IMU + VO alone; PX4's estimate is then used only to
        # *measure* drift, never to correct the filter.
        if (
            self._vo_updates == 0
            and self._px4_quat is not None
            and self._px4_pos is not None
            and np.all(np.isfinite(self._px4_pos))
            and np.all(np.isfinite(self._px4_vel))
        ):
            self._ekf.initialize(self._px4_pos, self._px4_quat, self._px4_vel)

    def _imu_cb(self, msg: SensorCombined) -> None:
        now = time.monotonic()
        gyro = np.array(msg.gyro_rad, dtype=float)
        accel = np.array(msg.accelerometer_m_s2, dtype=float)
        with self._lock:
            previous = self._last_imu_stamp_us
            dt = message_dt_s(previous, int(msg.timestamp), self._stale_timeout)
            self._last_imu_stamp_us = int(msg.timestamp)
            self._last_imu_t = now  # wall-clock arrival, used only for staleness
            self._imu_msgs += 1
            if dt is not None:
                self._imu_integrated_s += dt
                self._ekf.predict(gyro, accel, dt)
            elif previous is not None and int(msg.timestamp) > previous:
                # A stall longer than the staleness bound: the interval is not
                # integrated (guessing a rate across it is worse). Counted so a
                # starved IMU path shows up in the log instead of as yaw error.
                self._imu_skipped_s += (int(msg.timestamp) - previous) * 1e-6

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

        if self._frame_meta is not None:
            with self._lock:
                pp = self._px4_pos if self._px4_pos is not None else np.full(3, np.nan)
                pv = self._px4_vel if self._px4_vel is not None else np.full(3, np.nan)
                pq = self._px4_quat if self._px4_quat is not None else np.full(4, np.nan)
            cv2.imwrite(os.path.join(self._frames_dir, f"left_{self._frame_idx:05d}.png"), left_gray,
                        [cv2.IMWRITE_PNG_COMPRESSION, 1])
            cv2.imwrite(os.path.join(self._frames_dir, f"right_{self._frame_idx:05d}.png"), right_gray,
                        [cv2.IMWRITE_PNG_COMPRESSION, 1])
            self._frame_meta.writerow([self._frame_idx, f"{left_t:.4f}", *np.round(pp, 4),
                                       *np.round(pv, 4), *np.round(pq, 5)])
            self._frame_meta_file.flush()
            self._frame_idx += 1
        motion = self._odometry.process(left_gray, right_gray, left_t)
        if (
            motion is None
            or motion.dt_s < self._min_vo_dt
            or motion.inlier_count < self._min_vo_inliers
        ):
            return
        # Velocity = displacement over the *image-stamp* interval the motion
        # actually spans (frames can be dropped, so processing time is not
        # that interval), re-expressed from camera optical axes into body FRD
        # before rotating into the world frame.
        rot_body, trans_body = motion.in_body_frame()
        now = time.monotonic()
        with self._lock:
            r_body_to_world = quat_to_rotation_matrix(self._ekf.quat)
            velocity_world = r_body_to_world @ (trans_body / motion.dt_s)
            if self._debug_writer is not None:
                q = self._ekf.quat
                roll = np.degrees(np.arctan2(2 * (q[0] * q[1] + q[2] * q[3]), 1 - 2 * (q[1] ** 2 + q[2] ** 2)))
                pitch = np.degrees(np.arcsin(max(-1.0, min(1.0, 2 * (q[0] * q[2] - q[3] * q[1])))))
                pv = self._px4_vel if self._px4_vel is not None else np.full(3, np.nan)
                pq = self._px4_quat if self._px4_quat is not None else np.array([1.0, 0, 0, 0])
                ekf_yaw = np.degrees(np.arctan2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] ** 2 + q[3] ** 2)))
                px4_yaw = np.degrees(np.arctan2(2 * (pq[0] * pq[3] + pq[1] * pq[2]), 1 - 2 * (pq[2] ** 2 + pq[3] ** 2)))
                self._debug_writer.writerow(
                    [round(left_t, 3), round(motion.dt_s, 4), int(motion.inlier_count),
                     *np.round(velocity_world, 3), *np.round(pv, 3),
                     *np.round(self._ekf.velocity, 3), round(float(roll), 1),
                     round(float(pitch), 1), round(float(ekf_yaw), 1), round(float(px4_yaw), 1),
                     *np.round(trans_body, 4)]
                )
                self._debug_file.flush()
            self._dbg_vo_vel = velocity_world
            self._dbg_vo_dt = motion.dt_s
            accepted = self._ekf.update_velocity(velocity_world, self._vo_velocity_std)
            if accepted:
                if self._ekf.vo_attitude_blend > 0.0:
                    self._ekf.apply_vo_attitude_correction(rot_body)
                # "Healthy" means VO is currently *contributing*: rejected
                # measurements must not keep the estimate looking fresh.
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
            dbg_v, dbg_dt = self._dbg_vo_vel.copy(), self._dbg_vo_dt
            imu_msgs, imu_int, imu_skip = self._imu_msgs, self._imu_integrated_s, self._imu_skipped_s
            px4_v = np.zeros(3) if self._px4_vel is None else self._px4_vel.copy()
            vo_age = (
                None
                if self._last_vo_update_t is None
                else now - self._last_vo_update_t
            )
        self._healthy_pub.publish(Bool(data=healthy))

        with self._lock:
            if self._px4_pos is not None and vo_updates > 0:
                self._last_drift = self._drift.update(state.position_m, self._px4_pos)
            drift = dict(self._last_drift)
        status = Float32MultiArray()
        status.data = [
            float(state.position_m[0]),
            float(state.position_m[1]),
            float(state.position_m[2]),
            1.0 if healthy else 0.0,
            float(inliers),
            float(drift.get("err_horiz_m", 0.0)),
            float(drift.get("err_down_m", 0.0)),
            float(drift.get("path_length_m", 0.0)),
            float(drift.get("drift_frac", 0.0)),
            float(vo_updates),
        ]
        self._status_pub.publish(status)

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
                f"{state.position_m[2]:.2f}) "
                f"err={drift.get('err_horiz_m', 0.0):.2f}m "
                f"path={drift.get('path_length_m', 0.0):.1f}m "
                f"drift={100.0 * drift.get('drift_frac', 0.0):.1f}% "
                f"v_vo=({dbg_v[0]:.2f},{dbg_v[1]:.2f},{dbg_v[2]:.2f}) "
                f"v_ekf=({state.velocity_mps[0]:.2f},{state.velocity_mps[1]:.2f},"
                f"{state.velocity_mps[2]:.2f}) "
                f"v_px4=({px4_v[0]:.2f},{px4_v[1]:.2f},{px4_v[2]:.2f}) "
                f"vo_dt={dbg_dt:.3f}s "
                f"acc/rej={self._ekf.accepted_updates}/{self._ekf.rejected_updates} "
                f"imu_msgs={imu_msgs} imu_integrated={imu_int:.1f}s skipped={imu_skip:.1f}s"
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
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
