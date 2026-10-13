"""OFFBOARD precision landing from a downward camera and an ArUco marker.

Streams the ``precision_land.PrecisionLand`` setpoints on the same PX4 topics
``offboard_mission`` uses. It does not arm the vehicle. Publish ``true`` on
``/px4_offboard/precision_land/enable`` once the vehicle is already airborne;
the node then switches to OFFBOARD and runs search → approach → descend.

Do not run this at the same time as ``offboard_mission``: both publish
``fmu/in/trajectory_setpoint``.
"""

from __future__ import annotations

import math

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleAttitude,
    VehicleCommand,
    VehicleLandDetected,
    VehicleLocalPosition,
)
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, String

from .precision_land import (
    CameraTag,
    LandCommand,
    PrecisionLand,
    PrecisionLandParams,
    VehicleSnapshot,
    offboard_setpoint,
)
from .vision_marker_detect import camera_matrix, estimate_marker_poses


class PrecisionLandNode(Node):
    def __init__(self):
        super().__init__("precision_land")
        self._declare()
        self._controller = PrecisionLand(self._params_from_node())
        self._enabled = False
        self._offboard_sent = False
        self._ticks_since_activate = 0
        self._position: tuple[float, float, float] | None = None
        self._velocity = (0.0, 0.0, 0.0)
        self._attitude = (1.0, 0.0, 0.0, 0.0)
        self._landed = False
        self._pending_tag: CameraTag | None = None
        self._intrinsics: np.ndarray | None = None
        self._dist: np.ndarray | None = None
        self._logged_state: str | None = None

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._pub_ocm = self.create_publisher(OffboardControlMode, "fmu/in/offboard_control_mode", qos)
        self._pub_sp = self.create_publisher(TrajectorySetpoint, "fmu/in/trajectory_setpoint", qos)
        self._pub_cmd = self.create_publisher(VehicleCommand, "fmu/in/vehicle_command", qos)
        self._pub_pose = self.create_publisher(PoseStamped, "/target_pose", qos)
        self._pub_world = self.create_publisher(PoseStamped, "/target_pose_world", qos)
        self._pub_image = self.create_publisher(Image, "/image_proc", qos)
        self._pub_status = self.create_publisher(String, "/px4_offboard/precision_land/status", 10)

        self.create_subscription(Image, self._image_topic, self._image_cb, qos)
        self.create_subscription(CameraInfo, self._camera_info_topic, self._info_cb, qos)
        self.create_subscription(VehicleLocalPosition, "fmu/out/vehicle_local_position", self._position_cb, qos)
        self.create_subscription(VehicleAttitude, "fmu/out/vehicle_attitude", self._attitude_cb, qos)
        self.create_subscription(VehicleLandDetected, "fmu/out/vehicle_land_detected", self._land_cb, qos)
        self.create_subscription(Bool, "/px4_offboard/precision_land/enable", self._enable_cb, 10)

        period = 1.0 / self._control_hz
        self.create_timer(period, self._tick)
        self.get_logger().info(
            "precision land idle; publish true on /px4_offboard/precision_land/enable once airborne"
        )

    def _declare(self) -> None:
        self.declare_parameter("descent_vel", 0.6)
        self.declare_parameter("vel_p_gain", 1.7)
        self.declare_parameter("vel_i_gain", 0.0)
        self.declare_parameter("max_velocity", 3.0)
        self.declare_parameter("target_timeout", 5.0)
        self.declare_parameter("delta_position", 0.2)
        self.declare_parameter("delta_velocity", 0.2)
        self.declare_parameter("horizontal_fov_rad", 1.74)
        self.declare_parameter("marker_size_m", 1.0)
        self.declare_parameter("aruco_dictionary", "DICT_4X4_50")
        self.declare_parameter("aruco_id", -1)  # negative: largest marker in the frame
        self.declare_parameter("camera_offset_body_ned", [0.0, 0.0, 0.05])
        self.declare_parameter("image_topic", "/px4_offboard/camera/image_raw")
        self.declare_parameter("camera_info_topic", "/px4_offboard/camera/camera_info")
        self.declare_parameter("control_hz", 20.0)
        self.declare_parameter("command_offboard", True)
        self.declare_parameter("target_system_id", 1)
        self._image_topic = str(self.get_parameter("image_topic").value)
        self._camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        self._control_hz = float(self.get_parameter("control_hz").value)
        self._command_offboard = bool(self.get_parameter("command_offboard").value)
        self._target_system = int(self.get_parameter("target_system_id").value)
        self._fov = float(self.get_parameter("horizontal_fov_rad").value)
        self._marker_size = float(self.get_parameter("marker_size_m").value)
        self._dictionary = str(self.get_parameter("aruco_dictionary").value)
        self._marker_id = int(self.get_parameter("aruco_id").value)

    def _params_from_node(self) -> PrecisionLandParams:
        offset = [float(v) for v in self.get_parameter("camera_offset_body_ned").value]
        return PrecisionLandParams(
            descent_vel=float(self.get_parameter("descent_vel").value),
            vel_p_gain=float(self.get_parameter("vel_p_gain").value),
            vel_i_gain=float(self.get_parameter("vel_i_gain").value),
            max_velocity=float(self.get_parameter("max_velocity").value),
            target_timeout_s=float(self.get_parameter("target_timeout").value),
            delta_position=float(self.get_parameter("delta_position").value),
            delta_velocity=float(self.get_parameter("delta_velocity").value),
            camera_offset_body_ned=(offset[0], offset[1], offset[2]),
        )

    def _position_cb(self, msg: VehicleLocalPosition) -> None:
        self._position = (float(msg.x), float(msg.y), float(msg.z))
        self._velocity = (float(msg.vx), float(msg.vy), float(msg.vz))
        if self._enabled and not self._controller.active and not self._controller.failed:
            self._start()

    def _attitude_cb(self, msg: VehicleAttitude) -> None:
        q = msg.q
        self._attitude = (float(q[0]), float(q[1]), float(q[2]), float(q[3]))

    def _land_cb(self, msg: VehicleLandDetected) -> None:
        self._landed = bool(msg.landed)

    def _info_cb(self, msg: CameraInfo) -> None:
        k = msg.k
        self._intrinsics = np.array(
            [[k[0], k[1], k[2]], [k[3], k[4], k[5]], [k[6], k[7], k[8]]],
            dtype=np.float64,
        )
        self._dist = np.array(msg.d, dtype=np.float64) if len(msg.d) else np.zeros(5)

    def _enable_cb(self, msg: Bool) -> None:
        rising = bool(msg.data) and not self._enabled
        self._enabled = bool(msg.data)
        if not msg.data:
            self._controller.deactivate()
            self._offboard_sent = False
            self._publish_status()
            self.get_logger().info("precision land disabled")
            return
        if self._position is None:
            self.get_logger().warn("precision land armed; waiting for vehicle_local_position")
            return
        if rising or self._controller.failed or not self._controller.active:
            self._start()

    def _start(self) -> None:
        assert self._position is not None
        self._controller.activate(self._position)
        self._offboard_sent = False
        self._ticks_since_activate = 0
        self._logged_state = None
        self.get_logger().info("precision land searching")

    def _image_cb(self, msg: Image) -> None:
        gray, color = _decode(msg)
        if gray is None or color is None:
            self.get_logger().warn(f"unsupported image encoding {msg.encoding!r}", once=True)
            return
        intrinsics = self._intrinsics
        if intrinsics is None:
            intrinsics = camera_matrix(msg.width, msg.height, self._fov)
        poses = estimate_marker_poses(
            gray, intrinsics, self._marker_size, self._dictionary, self._dist
        )
        if self._marker_id >= 0:
            poses = [pose for pose in poses if pose.marker_id == self._marker_id]
        chosen = max(poses, key=lambda pose: pose.side_px) if poses else None
        _draw(color, poses, chosen)
        self._pub_image.publish(_rgb_image(msg, color))
        if chosen is None:
            return
        self._pending_tag = CameraTag(
            position_xyz=chosen.position_xyz,
            orientation_xyzw=chosen.orientation_xyzw,
            time_s=self._now_s(),
        )
        self._pub_pose.publish(
            _pose(chosen.position_xyz, chosen.orientation_xyzw, "camera_frame", msg.header.stamp)
        )

    def _tick(self) -> None:
        self._publish_status()
        if not self._controller.active or self._position is None:
            return
        snap = self._snapshot()
        if self._pending_tag is not None:
            self._controller.observe_tag(self._pending_tag, snap)
        command = self._controller.step(snap)
        self._publish_setpoint(command)
        self._maybe_command_offboard()
        label = "failed" if self._controller.failed else command.state.value
        if label != self._logged_state:
            self.get_logger().info(f"precision land → {label}")
            self._logged_state = label

    def _snapshot(self) -> VehicleSnapshot:
        assert self._position is not None
        return VehicleSnapshot(
            position_ned=self._position,
            velocity_ned=self._velocity,
            attitude_wxyz=self._attitude,
            landed=self._landed,
            time_s=self._now_s(),
        )

    def _publish_setpoint(self, command: LandCommand) -> None:
        fields = offboard_setpoint(command)
        if fields is None:
            return
        mode = OffboardControlMode()
        mode.timestamp = self._ts()
        mode.position = fields["use_position"]
        mode.velocity = not fields["use_position"]
        mode.acceleration = False
        mode.attitude = False
        mode.body_rate = False
        self._pub_ocm.publish(mode)

        setpoint = TrajectorySetpoint()
        setpoint.timestamp = mode.timestamp
        setpoint.position = [float(v) for v in fields["position_ned"]]
        setpoint.velocity = [float(v) for v in fields["velocity_ned"]]
        setpoint.acceleration = [float("nan")] * 3
        setpoint.jerk = [float("nan")] * 3
        setpoint.yaw = float(fields["yaw"])
        setpoint.yawspeed = float("nan")
        self._pub_sp.publish(setpoint)
        if command.tag_ned is not None:
            quat = _yaw_quat(command.yaw if command.yaw is not None else 0.0)
            self._pub_world.publish(_pose(command.tag_ned, quat, "map", self.get_clock().now().to_msg()))

    def _maybe_command_offboard(self) -> None:
        if not self._command_offboard or self._offboard_sent:
            return
        # PX4 rejects OFFBOARD until setpoints have been streaming.
        self._ticks_since_activate += 1
        if self._ticks_since_activate < int(self._control_hz):
            return
        msg = VehicleCommand()
        msg.timestamp = self._ts()
        msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        msg.param1 = 1.0
        msg.param2 = 6.0
        msg.target_system = self._target_system
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self._pub_cmd.publish(msg)
        self._offboard_sent = True
        self.get_logger().info("Sent: DO_SET_MODE → OFFBOARD")

    def _publish_status(self) -> None:
        if self._controller.failed:
            text = "failed"
        elif not self._controller.active:
            text = "idle"
        else:
            text = self._controller.state.value
        self._pub_status.publish(String(data=text))

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _ts(self) -> int:
        return self.get_clock().now().nanoseconds // 1000


def _yaw_quat(yaw: float) -> tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def _pose(xyz, xyzw, frame_id: str, stamp) -> PoseStamped:
    msg = PoseStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.pose.position.x = float(xyz[0])
    msg.pose.position.y = float(xyz[1])
    msg.pose.position.z = float(xyz[2])
    msg.pose.orientation.x = float(xyzw[0])
    msg.pose.orientation.y = float(xyzw[1])
    msg.pose.orientation.z = float(xyzw[2])
    msg.pose.orientation.w = float(xyzw[3])
    return msg


def _decode(msg: Image) -> tuple[np.ndarray | None, np.ndarray | None]:
    if msg.encoding == "mono8":
        gray = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)
        return gray, cv2.cvtColor(gray, cv2.COLOR_GRAY2RGB)
    channels = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4}.get(msg.encoding)
    if channels is None:
        return None, None
    frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, channels)
    if msg.encoding == "rgb8":
        return cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY), frame[:, :, :3].copy()
    if msg.encoding == "bgr8":
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        return cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), rgb
    if msg.encoding == "rgba8":
        rgb = frame[:, :, :3].copy()
        return cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY), rgb
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGRA2RGB)
    return cv2.cvtColor(frame, cv2.COLOR_BGRA2GRAY), rgb


def _draw(color: np.ndarray, poses, chosen) -> None:
    if not poses:
        return
    corners = [np.array(pose.corners_px, dtype=np.float32).reshape(1, 4, 2) for pose in poses]
    ids = np.array([[pose.marker_id] for pose in poses], dtype=np.int32)
    cv2.aruco.drawDetectedMarkers(color, corners, ids)
    if chosen is None:
        return
    x, y, z = chosen.position_xyz
    text = f"id {chosen.marker_id}  X:{x:.2f} Y:{y:.2f} Z:{z:.2f}"
    cv2.putText(color, text, (10, color.shape[0] - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)


def _rgb_image(src: Image, rgb: np.ndarray) -> Image:
    out = Image()
    out.header = src.header
    out.height = rgb.shape[0]
    out.width = rgb.shape[1]
    out.encoding = "rgb8"
    out.is_bigendian = 0
    out.step = rgb.shape[1] * 3
    out.data = rgb.tobytes()
    return out


def main(args=None):
    rclpy.init(args=args)
    node = PrecisionLandNode()
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
