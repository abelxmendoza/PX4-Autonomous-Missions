"""
offboard_mission.py — PX4 state-driven autonomous flight controller
                       (ROS 2 + px4_msgs + Micro XRCE-DDS)

State machine:
  PREFLIGHT → ARMING → TAKEOFF → HOVER → MOVE → LANDING
                                    ↘ FAILSAFE ↗

Architecture:
  ├── Publishers  — offboard_control_mode, trajectory_setpoint, vehicle_command
  │                 /px4_offboard/fence_status
  ├── Subscribers — vehicle_local_position, vehicle_status, vehicle_control_mode
  │                 /px4_offboard/obstacle_dir
  │                 /px4_offboard/geocage_enable  (std_msgs/Bool) — soft keep-in
  │                 /px4_offboard/geofence_enable (std_msgs/Bool) — hard breach → failsafe
  └── Core logic  — state machine, trajectory, AABB avoidance, geo-cage, geofence,
                    mission executive (resources + NOMINAL/DEGRADED/SAFE/ABORT), CSV log

Run:
  ros2 run px4_offboard offboard_mission
  ros2 run px4_offboard offboard_mission --ros-args --params-file config/offboard_mission.yaml

Toggle at runtime:
  ros2 topic pub --once /px4_offboard/geocage_enable std_msgs/msg/Bool "{data: true}"
  ros2 topic pub --once /px4_offboard/geofence_enable std_msgs/msg/Bool "{data: false}"
"""

from __future__ import annotations

import csv
import json
import math
import time
from enum import Enum
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32MultiArray, String

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleAttitude,
    VehicleCommand,
    VehicleControlMode,
    VehicleLocalPosition,
    VehicleStatus,
)
from px4_offboard.mission_executive import (
    ExecutiveAction,
    MissionExecutive,
    MissionExecutiveConfig,
    ResourceBudgets,
)
from px4_offboard.mission_logic import (
    Fence,
    Obstacle,
    blocking_height,
    circle_target,
    detect_obstacle,
    distance_3d,
    obstacle_clearance,
    sensor_bypass_plan,
    sensor_hit_within_segment,
    yaw_toward,
)
from px4_offboard.mission_state import (
    FailsafeInputs,
    MissionStateMachine,
    State,
    arming_complete,
    failsafe_reason,
    hover_complete,
    should_start_arming,
    takeoff_complete,
)


# ── Enums ─────────────────────────────────────────────────────────────────────

class TrajectoryMode(Enum):
    WAYPOINTS = "waypoints"   # reactive path through obstacle field
    COURSE = "course"         # pre-planned clearance path (matches MAVSDK script)
    CIRCLE = "circle"         # orbit, then land after max_orbits


# Obstacle AABB: (east_m, north_m, size_e, size_n, height_m) — matches obstacle_world.sdf
OBSTACLE_BOXES = (
    Obstacle(-6.0, 10.0, 3.0, 3.0, 11.5),  # OB1 red — taller than fence ceiling, climb not an option
    Obstacle(10.0, 10.0, 3.0, 3.0, 6.0),   # OB2 orange — off to the side, rarely on the flight path
    Obstacle(-8.0, 24.0, 5.0, 3.0, 4.0),   # OB3 green — widened, tighter corridor with OB4
    Obstacle(6.0, 24.0, 3.0, 2.0, 5.0),    # OB4 blue — widened, tighter corridor with OB3
    Obstacle(0.0, 38.0, 5.0, 3.0, 11.5),   # OB5 purple — taller than fence ceiling, climb not an option
)

# Reactive path: approaches obstacles so AABB avoidance must engage.
# NED [north, east, down] — z negative = up.
DEFAULT_WAYPOINTS = [
    [0.0, 0.0, -5.0],
    [3.0, -6.0, -5.0],    # stage 5.5 m before OB1 so yaw/LiDAR align before entry
    [15.0, -6.0, -5.0],   # beyond OB1; reactive avoidance curves around it
    [18.0, 0.0, -5.0],
    [24.0, 0.0, -5.0],    # OB3 / OB4 corridor
    [32.0, 0.0, -5.0],    # toward OB5
    [43.0, 0.0, -5.0],    # beyond OB5; reactive avoidance curves around it
    [46.0, 8.0, -5.0],
    [50.0, 0.0, -5.0],
]

# Pre-planned clearance path (east, north) → NED, 8 m AGL — clears OB2 (6 m).
DEFAULT_COURSE = [
    (-13.0, 5.0),
    (-13.0, 15.0),
    (0.0, 15.0),
    (0.0, 24.0),
    (0.0, 32.0),
    (8.0, 36.0),
    (8.0, 46.0),
    (0.0, 50.0),
]


class OffboardMission(Node):

    def __init__(self):
        super().__init__("offboard_mission")

        self._declare_params()
        self._load_params()

        qos_pub = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        qos_sub = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._pub_ocm = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", qos_pub
        )
        self._pub_sp = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", qos_pub
        )
        self._pub_cmd = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", qos_pub
        )
        self._pub_fence_status = self.create_publisher(
            String, "/px4_offboard/fence_status", 10
        )
        self._pub_avoiding = self.create_publisher(Bool, "/px4_offboard/avoiding", 10)
        self._pub_mission_status = self.create_publisher(
            String, "/px4_offboard/mission_status", 10
        )
        self._pub_executive_status = self.create_publisher(
            String, "/px4_offboard/executive_status", 10
        )

        self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position",
            self._position_callback,
            qos_sub,
        )
        self.create_subscription(
            VehicleAttitude,
            "/fmu/out/vehicle_attitude",
            self._attitude_callback,
            qos_sub,
        )
        # PX4 v1.15+ publishes the live topic as vehicle_status_v1
        self.create_subscription(
            VehicleStatus,
            "/fmu/out/vehicle_status_v1",
            self._status_callback,
            qos_sub,
        )
        # Reliable arm/offboard flags (works even when VehicleStatus DDS type drifts)
        self.create_subscription(
            VehicleControlMode,
            "/fmu/out/vehicle_control_mode",
            self._control_mode_callback,
            qos_sub,
        )
        # Optional sensor override: publish "front" | "left" | "right" | "none"
        self.create_subscription(
            String,
            "/px4_offboard/obstacle_dir",
            self._sensor_callback,
            10,
        )
        # Per-sector minimum ranges [front, left, right] — used to check
        # whether a lateral escape direction is also blocked before
        # committing to it (a single "obstacle_dir" label can't tell us
        # whether the *other* side is clear too).
        self.create_subscription(
            Float32MultiArray,
            "/px4_offboard/lidar_sector_mins",
            self._sensor_mins_callback,
            10,
        )
        self.create_subscription(
            Bool, "/px4_offboard/geocage_enable", self._geocage_toggle_cb, 10
        )
        self.create_subscription(
            Bool, "/px4_offboard/geofence_enable", self._geofence_toggle_cb, 10
        )

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_vz = 0.0
        self._pos_stamp = 0.0
        self._have_position = False

        # Attitude — roll/pitch/yaw (radians), converted from the FRD-body→NED
        # quaternion PX4 publishes on vehicle_attitude. Logged and used for
        # real drone orientation in the web replay viewer (previously the
        # viewer inferred heading from direction of travel only).
        self.current_roll = 0.0
        self.current_pitch = 0.0
        self.current_yaw = 0.0
        self._have_attitude = False

        self._nav_state = -1
        self._arming_state = -1
        self._flag_armed = False
        self._flag_offboard = False
        self._last_obstacle = None
        self._bypass_target = None
        self._bypass_obstacle = None
        self._sensor_advance_target = None
        self._bypassed_obstacles = set()
        self._last_bypass_distance = float("inf")

        self._state_machine = MissionStateMachine()
        self._counter = 0
        self._hover_timer = 0.0
        self._mission_t = 0.0
        self._wp_index = 0
        self._last_wp_progress_t = 0.0
        self._avoid_active_t = 0.0
        self._smooth_target = [0.0, 0.0, -self.hover_alt]
        self._sensor_dir: str | None = None
        self._sensor_stamp = 0.0
        self._sensor_mins: tuple[float, float, float] | None = None
        self._sensor_mins_stamp = 0.0
        self._failsafe_reason = ""
        self._land_sent = False
        self._geocage_hit = False
        self._geofence_breach = False
        self._executive_hold = False
        self._last_exec_mode = None
        self._last_loop_t = time.monotonic()

        self._executive = MissionExecutive(self._executive_config)

        self._log_file = None
        self._log_writer = None
        self._open_log()

        self.create_timer(0.1, self._control_loop)
        self.get_logger().info(
            f"OffboardMission ready — mode={self.trajectory_mode.value} "
            f"alt={self.hover_alt}m waypoints={len(self.waypoints)} "
            f"geocage={'ON' if self.geocage_enable else 'OFF'} "
            f"geofence={'ON' if self.geofence_enable else 'OFF'} "
            f"obstacles={self.obstacle_source} "
            f"executive={'ON' if self._executive.config.enable else 'OFF'} "
            f"box N[{self.fence_n_min},{self.fence_n_max}] "
            f"E[{self.fence_e_min},{self.fence_e_max}] "
            f"alt≤{self.fence_alt_max}m"
        )

    # ── Parameters ────────────────────────────────────────────────────────────

    def _declare_params(self):
        self.declare_parameter("trajectory_mode", "waypoints")
        self.declare_parameter("hover_alt_m", 5.0)
        self.declare_parameter("preflight_cycles", 20)
        self.declare_parameter("hover_hold_s", 3.0)
        self.declare_parameter("takeoff_z_tol_m", 0.25)
        self.declare_parameter("wp_accept_m", 0.6)
        self.declare_parameter("detection_margin_m", 2.5)
        self.declare_parameter("front_angle_deg", 35.0)
        self.declare_parameter("side_angle_deg", 70.0)
        self.declare_parameter("avoid_smooth", 0.25)
        self.declare_parameter("sidestep_m", 2.0)
        self.declare_parameter("avoidance_strategy", "climb")
        self.declare_parameter("climb_clearance_m", 1.5)
        self.declare_parameter("max_alt_m", 12.0)
        self.declare_parameter("circle_radius_m", 8.0)
        self.declare_parameter("circle_period_s", 20.0)
        self.declare_parameter("max_orbits", 2.0)
        self.declare_parameter("position_timeout_s", 1.5)
        self.declare_parameter("mission_timeout_s", 180.0)
        self.declare_parameter("avoid_stuck_s", 12.0)
        self.declare_parameter("sensor_timeout_s", 0.5)
        self.declare_parameter("obstacle_source", "hybrid")
        self.declare_parameter("log_dir", ".")

        # Geo-cage (soft keep-in: clamp setpoints) / geofence (hard: breach → action)
        self.declare_parameter("geocage_enable", True)
        self.declare_parameter("geofence_enable", True)
        self.declare_parameter("fence_north_min_m", -5.0)
        self.declare_parameter("fence_north_max_m", 55.0)
        self.declare_parameter("fence_east_min_m", -23.0)
        self.declare_parameter("fence_east_max_m", 17.0)
        self.declare_parameter("fence_alt_max_m", 12.0)
        self.declare_parameter("geocage_margin_m", 1.0)
        self.declare_parameter("geofence_action", "land")  # land | hold | rtl
        self.declare_parameter("px4_fence_cmd", False)  # also send VEHICLE_CMD_DO_FENCE_ENABLE

        # Mission executive / resource manager
        self.declare_parameter("executive_enable", True)
        self.declare_parameter("science_waypoints", [2, 4, 6])
        self.declare_parameter("initial_battery", 1.0)
        self.declare_parameter("initial_propellant_s", 180.0)
        self.declare_parameter("battery_degraded", 0.45)
        self.declare_parameter("battery_safe", 0.25)
        self.declare_parameter("battery_abort", 0.12)
        self.declare_parameter("link_degraded", 0.55)
        self.declare_parameter("link_safe", 0.30)
        self.declare_parameter("link_abort", 0.10)
        self.declare_parameter("link_loss_abort_s", 8.0)
        self.declare_parameter("propellant_degraded_s", 90.0)
        self.declare_parameter("propellant_safe_s", 45.0)
        self.declare_parameter("propellant_abort_s", 20.0)

    def _load_params(self):
        mode = self.get_parameter("trajectory_mode").value.lower()
        try:
            self.trajectory_mode = TrajectoryMode(mode)
        except ValueError:
            self.get_logger().warn(f"Unknown trajectory_mode={mode!r}, using waypoints")
            self.trajectory_mode = TrajectoryMode.WAYPOINTS

        self.hover_alt = float(self.get_parameter("hover_alt_m").value)
        self.preflight_cycles = int(self.get_parameter("preflight_cycles").value)
        self.hover_hold_s = float(self.get_parameter("hover_hold_s").value)
        self.takeoff_z_tol = float(self.get_parameter("takeoff_z_tol_m").value)
        self.wp_accept = float(self.get_parameter("wp_accept_m").value)
        self.detection_margin = float(self.get_parameter("detection_margin_m").value)
        self.front_angle = float(self.get_parameter("front_angle_deg").value)
        self.side_angle = float(self.get_parameter("side_angle_deg").value)
        self.avoid_smooth = float(self.get_parameter("avoid_smooth").value)
        self.sidestep_m = float(self.get_parameter("sidestep_m").value)
        strategy = str(self.get_parameter("avoidance_strategy").value).lower()
        self.avoidance_strategy = (
            strategy if strategy in ("climb", "sidestep") else "climb"
        )
        self.climb_clearance = float(self.get_parameter("climb_clearance_m").value)
        self.max_alt = float(self.get_parameter("max_alt_m").value)
        self.circle_radius = float(self.get_parameter("circle_radius_m").value)
        self.circle_period = float(self.get_parameter("circle_period_s").value)
        self.max_orbits = float(self.get_parameter("max_orbits").value)
        self.position_timeout = float(self.get_parameter("position_timeout_s").value)
        self.mission_timeout = float(self.get_parameter("mission_timeout_s").value)
        self.avoid_stuck_s = float(self.get_parameter("avoid_stuck_s").value)
        self.sensor_timeout = float(self.get_parameter("sensor_timeout_s").value)
        obstacle_source = str(self.get_parameter("obstacle_source").value).lower()
        self.obstacle_source = (
            obstacle_source
            if obstacle_source in ("hybrid", "sensor_only", "map_only")
            else "hybrid"
        )
        self.log_dir = Path(self.get_parameter("log_dir").value)

        self.geocage_enable = bool(self.get_parameter("geocage_enable").value)
        self.geofence_enable = bool(self.get_parameter("geofence_enable").value)
        self.fence_n_min = float(self.get_parameter("fence_north_min_m").value)
        self.fence_n_max = float(self.get_parameter("fence_north_max_m").value)
        self.fence_e_min = float(self.get_parameter("fence_east_min_m").value)
        self.fence_e_max = float(self.get_parameter("fence_east_max_m").value)
        self.fence_alt_max = float(self.get_parameter("fence_alt_max_m").value)
        self.geocage_margin = max(0.0, float(self.get_parameter("geocage_margin_m").value))
        action = str(self.get_parameter("geofence_action").value).lower()
        self.geofence_action = action if action in ("land", "hold", "rtl") else "land"
        self.px4_fence_cmd = bool(self.get_parameter("px4_fence_cmd").value)
        self.fence = Fence(
            self.fence_n_min,
            self.fence_n_max,
            self.fence_e_min,
            self.fence_e_max,
            self.fence_alt_max,
        )

        if self.trajectory_mode == TrajectoryMode.COURSE:
            # Clearance path flies above tallest obstacle (OB2 = 6 m)
            self.hover_alt = max(self.hover_alt, 8.0)
            z = -self.hover_alt
            self.waypoints = [[n, e, z] for e, n in DEFAULT_COURSE]
        else:
            self.waypoints = [
                [wp[0], wp[1], -self.hover_alt] for wp in DEFAULT_WAYPOINTS
            ]

        science_raw = self.get_parameter("science_waypoints").value
        science_wps = tuple(int(i) for i in science_raw) if science_raw else ()
        self._executive_config = MissionExecutiveConfig(
            enable=bool(self.get_parameter("executive_enable").value),
            science_waypoints=science_wps,
            initial_battery=float(self.get_parameter("initial_battery").value),
            initial_propellant_s=float(
                self.get_parameter("initial_propellant_s").value
            ),
            budgets=ResourceBudgets(
                battery_degraded=float(self.get_parameter("battery_degraded").value),
                battery_safe=float(self.get_parameter("battery_safe").value),
                battery_abort=float(self.get_parameter("battery_abort").value),
                link_degraded=float(self.get_parameter("link_degraded").value),
                link_safe=float(self.get_parameter("link_safe").value),
                link_abort=float(self.get_parameter("link_abort").value),
                link_loss_abort_s=float(
                    self.get_parameter("link_loss_abort_s").value
                ),
                propellant_degraded_s=float(
                    self.get_parameter("propellant_degraded_s").value
                ),
                propellant_safe_s=float(
                    self.get_parameter("propellant_safe_s").value
                ),
                propellant_abort_s=float(
                    self.get_parameter("propellant_abort_s").value
                ),
            ),
        )

    # ── Telemetry log ─────────────────────────────────────────────────────────

    def _open_log(self):
        stamp = time.strftime("%Y%m%d_%H%M%S")
        try:
            self.log_dir.mkdir(parents=True, exist_ok=True)
            path = self.log_dir / f"flight_log_mission_{stamp}.csv"
            self._log_file = open(path, "w", newline="")
        except OSError as exc:
            # Unwritable log dir / full disk / permissions — fly without a
            # log rather than fail node startup over telemetry.
            self.get_logger().error(
                f"Could not open flight log ({exc}); continuing without logging"
            )
            self._log_file = None
            self._log_writer = None
            return
        self._log_writer = csv.writer(self._log_file)
        self._log_writer.writerow(
            [
                "time",
                "state",
                "north",
                "east",
                "down",
                "tgt_n",
                "tgt_e",
                "tgt_d",
                "obstacle",
                "obstacle_source",
                "sensor_fresh",
                "lidar_front_m",
                "lidar_left_m",
                "lidar_right_m",
                "mapped_clearance_m",
                "nominal_n",
                "nominal_e",
                "nominal_d",
                "wp_index",
                "geocage",
                "geofence",
                "inside",
                "caged",
                "roll_deg",
                "pitch_deg",
                "yaw_deg",
                "vn",
                "ve",
                "vd",
                "executive_mode",
                "battery_frac",
                "link_quality",
                "propellant_s",
            ]
        )
        self.get_logger().info(f"Logging to {path}")

    def _log_row(
        self,
        target: list,
        obstacle: str | None,
        nominal_target: list | None = None,
    ):
        if self._log_writer is None:
            return
        inside = self._inside_fence(self.current_x, self.current_y, self.current_z)
        nominal = nominal_target if nominal_target is not None else target
        sensor_fresh = self._sensor_data_fresh()
        sector_mins = self._sensor_mins if sensor_fresh and self._sensor_mins else (-1.0, -1.0, -1.0)
        clearance = min(
            obstacle_clearance(
                [self.current_x, self.current_y, self.current_z], obstacle_box
            )
            for obstacle_box in OBSTACLE_BOXES
        )
        row = [
            time.strftime("%H:%M:%S.%f")[:-3],
            self._state.name,
            round(self.current_x, 3),
            round(self.current_y, 3),
            round(self.current_z, 3),
            round(target[0], 3),
            round(target[1], 3),
            round(target[2], 3),
            obstacle or "",
            self.obstacle_source,
            int(sensor_fresh),
            round(sector_mins[0], 3),
            round(sector_mins[1], 3),
            round(sector_mins[2], 3),
            round(clearance, 3),
            round(nominal[0], 3),
            round(nominal[1], 3),
            round(nominal[2], 3),
            self._wp_index,
            int(self.geocage_enable),
            int(self.geofence_enable),
            int(inside),
            int(self._geocage_hit),
            round(math.degrees(self.current_roll), 2),
            round(math.degrees(self.current_pitch), 2),
            round(math.degrees(self.current_yaw), 2),
            round(self.current_vx, 3),
            round(self.current_vy, 3),
            round(self.current_vz, 3),
            self._executive.mode.name,
            round(self._executive.resources.battery_frac, 4),
            round(self._executive.resources.link_quality, 4),
            round(self._executive.resources.propellant_time_s, 2),
        ]
        try:
            self._log_writer.writerow(row)
            self._log_file.flush()
        except OSError as exc:
            # Disk full / handle error mid-flight — drop logging rather than
            # let this propagate out of the control-loop timer callback and
            # abort rclpy.spin(), which would stop setpoint streaming.
            self.get_logger().error(
                f"Flight log write failed ({exc}); disabling further logging",
                throttle_duration_sec=5.0,
            )
            try:
                self._log_file.close()
            except OSError:
                pass
            self._log_writer = None
            self._log_file = None

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _position_callback(self, msg: VehicleLocalPosition):
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_z = msg.z
        self.current_vx = msg.vx
        self.current_vy = msg.vy
        self.current_vz = msg.vz
        self._pos_stamp = time.monotonic()
        self._have_position = True

    def _attitude_callback(self, msg: VehicleAttitude):
        # q is [w, x, y, z], Hamilton convention, FRD body → NED earth frame.
        # Standard aerospace ZYX Euler extraction (same formula PX4/MAVLink use).
        qw, qx, qy, qz = msg.q[0], msg.q[1], msg.q[2], msg.q[3]
        self.current_roll = math.atan2(
            2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy)
        )
        sinp = 2.0 * (qw * qy - qz * qx)
        sinp = max(-1.0, min(1.0, sinp))
        self.current_pitch = math.asin(sinp)
        self.current_yaw = math.atan2(
            2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz)
        )
        self._have_attitude = True

    def _status_callback(self, msg: VehicleStatus):
        self._nav_state = msg.nav_state
        self._arming_state = msg.arming_state

    def _control_mode_callback(self, msg: VehicleControlMode):
        self._flag_armed = bool(msg.flag_armed)
        self._flag_offboard = bool(msg.flag_control_offboard_enabled)

    def _sensor_callback(self, msg: String):
        value = msg.data.strip().lower()
        if value in ("front", "left", "right", "none", ""):
            self._sensor_dir = None if value in ("none", "") else value
            self._sensor_stamp = time.monotonic()

    def _sensor_mins_callback(self, msg: Float32MultiArray):
        if len(msg.data) == 3:
            self._sensor_mins = (msg.data[0], msg.data[1], msg.data[2])
            self._sensor_mins_stamp = time.monotonic()

    def _sensor_data_fresh(self) -> bool:
        now = time.monotonic()
        return bool(
            self._sensor_stamp
            and self._sensor_mins_stamp
            and (now - self._sensor_stamp) < self.sensor_timeout
            and (now - self._sensor_mins_stamp) < self.sensor_timeout
        )

    def _lateral_blocked(self, side: str) -> bool:
        """True if the live LiDAR shows `side` (left|right) also within
        the danger margin — i.e. sidestepping that way would not clear
        the obstacle either. Unknown/stale sensor data reads as clear,
        preserving existing AABB-only behaviour."""
        if self._sensor_mins is None:
            return False
        if (time.monotonic() - self._sensor_mins_stamp) >= self.sensor_timeout:
            return False
        _, left_m, right_m = self._sensor_mins
        value = left_m if side == "left" else right_m
        return 0.0 <= value < self.detection_margin

    def _geocage_toggle_cb(self, msg: Bool):
        self.geocage_enable = bool(msg.data)
        self.get_logger().info(f"Geo-cage {'ENABLED' if self.geocage_enable else 'DISABLED'}")
        self._publish_fence_status()

    def _geofence_toggle_cb(self, msg: Bool):
        self.geofence_enable = bool(msg.data)
        self.get_logger().info(
            f"Geofence {'ENABLED' if self.geofence_enable else 'DISABLED'} "
            f"(action={self.geofence_action})"
        )
        if self.px4_fence_cmd:
            self._send_px4_fence_enable(self.geofence_enable)
        self._publish_fence_status()

    # ── Control loop ──────────────────────────────────────────────────────────

    def _control_loop(self):
        self._publish_offboard_control_mode()
        self._tick_executive()

        if self._state not in (State.PREFLIGHT, State.FAILSAFE, State.LANDING):
            if self._check_failsafes():
                return

        if self._state == State.PREFLIGHT:
            target = [0.0, 0.0, -self.hover_alt]
            target = self._apply_geocage(target)
            self._publish_setpoint(target)
            if self._counter == self.preflight_cycles:
                self._send_offboard_mode()
            if should_start_arming(self._counter, self.preflight_cycles):
                self._send_arm()
                if self.px4_fence_cmd and self.geofence_enable:
                    self._send_px4_fence_enable(True)
                self._transition(State.ARMING)

        elif self._state == State.ARMING:
            target = [0.0, 0.0, -self.hover_alt]
            target = self._apply_geocage(target)
            self._publish_setpoint(target)
            # Re-assert OFFBOARD + ARM until PX4 confirms (EKF / GCS can lag)
            if self._counter % 20 == 0:  # every 2 s
                self._send_offboard_mode()
                self._send_arm()
            if arming_complete(
                self._flag_armed,
                self._arming_state,
                self._flag_offboard,
                self._nav_state,
                self._have_position,
                self.current_z,
                self._counter,
            ):
                self._transition(State.TAKEOFF)

        elif self._state == State.TAKEOFF:
            target = [0.0, 0.0, -self.hover_alt]
            target = self._apply_geocage(target)
            self._publish_setpoint(target)
            if takeoff_complete(self.current_z, self.hover_alt, self.takeoff_z_tol):
                self._transition(State.HOVER)

        elif self._state == State.HOVER:
            target = [0.0, 0.0, -self.hover_alt]
            target = self._apply_geocage(target)
            self._publish_setpoint(target)
            self._hover_timer += 0.1
            if hover_complete(self._hover_timer, self.hover_hold_s):
                self._last_wp_progress_t = time.monotonic()
                self._transition(State.MOVE)

        elif self._state == State.MOVE:
            if self._apply_executive_decision():
                return  # ABORT → FAILSAFE entered
            if self._state != State.MOVE:
                pass  # e.g. skip last science WP → LANDING
            elif self._executive_hold:
                hold = [self.current_x, self.current_y, self.current_z]
                hold = self._apply_geocage(hold)
                self._publish_setpoint(hold)
                self._log_row(hold, self._last_obstacle)
            else:
                if self.obstacle_source == "sensor_only" and not self._sensor_data_fresh():
                    self._enter_failsafe("LiDAR timeout in sensor-only mode")
                    return
                nominal_target = self._next_target()
                target = list(nominal_target)
                obs = self._detect_obstacle(target)
                self._last_obstacle = obs
                target = self._apply_avoidance(target, obs)
                target = self._apply_geocage(target)
                self._publish_setpoint(target)
                self._log_row(target, self._last_obstacle, nominal_target)
                self._mission_t += 0.1

        elif self._state == State.LANDING:
            hold = [self.current_x, self.current_y, 0.0]
            hold = self._apply_geocage(hold)
            self._publish_setpoint(hold)
            if self._counter % 5 == 0:
                self._log_row(hold, self._last_obstacle)
            if not self._land_sent:
                self._send_land()
                self._land_sent = True
            # Prefer control_mode; fall back to near-ground after land command
            if (not self._flag_armed and self._arming_state != 2) or (
                abs(self.current_z) < 0.35 and self._counter % 30 == 0
            ):
                if not self._flag_armed or abs(self.current_z) < 0.35:
                    self.get_logger().info(
                        "Landed — mission complete", throttle_duration_sec=5.0
                    )

        elif self._state == State.FAILSAFE:
            if self.geofence_action == "hold" and "geofence" in self._failsafe_reason:
                # Soft hold: clamp inside cage and keep streaming setpoints
                hold = [
                    min(max(self.current_x, self.fence_n_min + self.geocage_margin),
                        self.fence_n_max - self.geocage_margin),
                    min(max(self.current_y, self.fence_e_min + self.geocage_margin),
                        self.fence_e_max - self.geocage_margin),
                    max(self.current_z, -self.fence_alt_max),
                ]
                self._publish_setpoint(hold)
                self._log_row(hold, self._last_obstacle)
            else:
                hold = [self.current_x, self.current_y, min(self.current_z, -2.0)]
                self._publish_setpoint(hold)
                self._log_row(hold, self._last_obstacle)
                if not self._land_sent:
                    self.get_logger().error(
                        f"FAILSAFE: {self._failsafe_reason} — {self.geofence_action}"
                    )
                    if self.geofence_action == "rtl":
                        self._send_rtl()
                    else:
                        self._send_land()
                    self._land_sent = True

        if self._counter % 10 == 0:  # 1 Hz status
            self._publish_fence_status()
            self._publish_mission_status()
            self._publish_executive_status()

        self._counter += 1

    def _tick_executive(self) -> None:
        now = time.monotonic()
        dt = max(0.0, min(0.5, now - self._last_loop_t))
        self._last_loop_t = now
        airborne = self._state in (
            State.TAKEOFF,
            State.HOVER,
            State.MOVE,
            State.LANDING,
            State.FAILSAFE,
        )
        link_ok = self._have_position and (
            now - self._pos_stamp
        ) < self.position_timeout
        lidar_active = bool(self._sensor_stamp) and (
            now - self._sensor_stamp
        ) < self.sensor_timeout
        self._executive.update(
            dt,
            airborne=airborne and self._flag_armed,
            moving=self._state == State.MOVE and not self._executive_hold,
            avoiding=self._last_obstacle is not None,
            link_ok=link_ok,
            lidar_active=lidar_active,
        )

    def _apply_executive_decision(self) -> bool:
        """Evaluate executive; return True if MOVE loop should stop advancing."""
        remaining = max(0, len(self.waypoints) - self._wp_index)
        decision = self._executive.evaluate(
            current_wp_index=self._wp_index,
            remaining_waypoints=remaining,
        )
        if decision.mode is not self._last_exec_mode:
            self.get_logger().warn(
                f"EXECUTIVE {decision.mode.name}: {decision.reason}"
            )
            self._last_exec_mode = decision.mode

        if decision.action is ExecutiveAction.ABORT_LAND:
            self._executive_hold = False
            return self._enter_failsafe(f"executive abort: {decision.reason}")

        if decision.action is ExecutiveAction.HOLD_SAFE:
            self._executive_hold = True
            return False

        self._executive_hold = False

        if decision.action is ExecutiveAction.SKIP_SCIENCE:
            for wp_i in decision.skipped_waypoints:
                self._executive.mark_skipped(wp_i)
                self.get_logger().warn(
                    f"EXECUTIVE skip science WP {wp_i} ({decision.reason})"
                )
                if self._wp_index == wp_i:
                    self._wp_index += 1
                    self._last_wp_progress_t = time.monotonic()
                    if self._wp_index >= len(self.waypoints):
                        self._transition(State.LANDING)
            return False

        return False

    def _check_failsafes(self) -> bool:
        now = time.monotonic()
        reason = failsafe_reason(
            FailsafeInputs(
                state=self._state,
                now_s=now,
                have_position=self._have_position,
                position_stamp_s=self._pos_stamp,
                position_timeout_s=self.position_timeout,
                mission_elapsed_s=self._mission_t,
                mission_timeout_s=self.mission_timeout,
                avoidance_started_s=self._avoid_active_t,
                waypoint_progress_s=self._last_wp_progress_t,
                avoidance_stuck_s=self.avoid_stuck_s,
                geofence_enabled=self.geofence_enable,
                inside_fence=self._inside_fence(
                    self.current_x, self.current_y, self.current_z
                ),
                north_m=self.current_x,
                east_m=self.current_y,
                down_m=self.current_z,
            )
        )
        if reason is None:
            return False
        if reason.startswith("geofence breach"):
            self._geofence_breach = True
        return self._enter_failsafe(reason)

    def _enter_failsafe(self, reason: str) -> bool:
        self._failsafe_reason = reason
        self._transition(State.FAILSAFE)
        return True

    # ── Trajectory ────────────────────────────────────────────────────────────

    def _next_target(self) -> list:
        if self.trajectory_mode == TrajectoryMode.CIRCLE:
            return self._circle_target()
        return self._waypoint_target()

    def _waypoint_target(self) -> list:
        if self._wp_index >= len(self.waypoints):
            self._transition(State.LANDING)
            return list(self.waypoints[-1])

        target = list(self.waypoints[self._wp_index])
        if self._distance_to_wp(target) < self.wp_accept:
            self.get_logger().info(f"WP {self._wp_index} reached  {target}")
            self._wp_index += 1
            self._last_wp_progress_t = time.monotonic()
            self._avoid_active_t = 0.0
            if self._wp_index >= len(self.waypoints):
                self._transition(State.LANDING)
                return list(self.waypoints[-1])
            target = list(self.waypoints[self._wp_index])
        return target

    def _circle_target(self) -> list:
        if self._mission_t >= self.max_orbits * self.circle_period:
            self._transition(State.LANDING)
            return [0.0, 0.0, -self.hover_alt]
        return circle_target(
            self._mission_t, self.circle_radius, self.circle_period, self.hover_alt
        )

    def _distance_to_wp(self, target: list) -> float:
        return distance_3d([self.current_x, self.current_y, self.current_z], target)

    # ── Obstacle detection (AABB + optional sensor) ───────────────────────────

    def _detect_obstacle(self, target: list) -> str | None:
        """
        Select live LiDAR and/or mapped AABB geometry according to
        ``obstacle_source`` (hybrid | sensor_only | map_only).
        """
        now = time.monotonic()
        if self.obstacle_source != "map_only" and self._sensor_stamp and (
            now - self._sensor_stamp
        ) < self.sensor_timeout:
            if self._counter % 20 == 0 and self._sensor_dir:
                self.get_logger().info(
                    f"LiDAR sector active → {self._sensor_dir}",
                    throttle_duration_sec=2.0,
                )
            if self._sensor_dir and self._sensor_mins:
                if not sensor_hit_within_segment(
                    [self.current_x, self.current_y, self.current_z],
                    target,
                    self._sensor_dir,
                    *self._sensor_mins,
                    endpoint_margin_m=max(0.5, self.wp_accept),
                ):
                    return None
            return self._sensor_dir

        if self.obstacle_source == "sensor_only":
            return None

        obstacles = (
            tuple(
                obstacle
                for obstacle in OBSTACLE_BOXES
                if obstacle not in self._bypassed_obstacles
            )
            if self.avoidance_strategy == "sidestep"
            else OBSTACLE_BOXES
        )
        return detect_obstacle(
            [self.current_x, self.current_y, self.current_z],
            target,
            obstacles,
            self.detection_margin,
            self.front_angle,
            self.side_angle,
            self.avoidance_strategy == "sidestep",
        )

    def _apply_avoidance(self, target: list, obs: str | None) -> list:
        adjusted = list(target)

        if self.avoidance_strategy == "sidestep" or self.obstacle_source == "sensor_only":
            if self._bypass_target is None and obs is not None:
                remaining = (
                    []
                    if self.obstacle_source == "sensor_only"
                    else [
                        obstacle
                        for obstacle in OBSTACLE_BOXES
                        if obstacle not in self._bypassed_obstacles
                    ]
                )
                if remaining:
                    nearest = min(
                        remaining,
                        key=lambda obstacle: math.hypot(
                            obstacle.north - self.current_x,
                            obstacle.east - self.current_y,
                        ),
                    )
                    self._bypass_target = [
                        nearest.north
                        + nearest.size_north / 2.0
                        + min(self.detection_margin, 2.5),
                        nearest.east + self.sidestep_m,
                        target[2],
                    ]
                    self._bypass_obstacle = nearest
                else:
                    # Live LiDAR sees something ahead that isn't in the
                    # known AABB map (or every mapped obstacle is already
                    # bypassed) — sidestep from the current position instead
                    # of relying on obstacle geometry.
                    _, left_m, right_m = self._sensor_mins or (-1.0, -1.0, -1.0)
                    # A sensor-only plan has no obstacle depth to consult.  The
                    # advance leg must therefore carry the aircraft from the
                    # trigger envelope, past the unseen obstacle thickness,
                    # and out the far-side safety margin.  Likewise, a bare
                    # ``sidestep_m`` can put the vehicle centre just outside a
                    # wall while its arms still intersect it.
                    sensor_forward_m = max(8.0, 2.0 * self.detection_margin + 3.0)
                    sensor_lateral_m = max(
                        4.0, self.sidestep_m, self.detection_margin + 1.5
                    )
                    self._bypass_target, self._sensor_advance_target = sensor_bypass_plan(
                        [self.current_x, self.current_y, self.current_z],
                        target,
                        obs,
                        sensor_forward_m,
                        sensor_lateral_m,
                        left_m,
                        right_m,
                    )
                    self._bypass_obstacle = None
                if self.geocage_enable:
                    self._bypass_target, _ = self.fence.clamp(
                        self._bypass_target, self.geocage_margin
                    )
                    if self._sensor_advance_target is not None:
                        self._sensor_advance_target, _ = self.fence.clamp(
                            self._sensor_advance_target, self.geocage_margin
                        )
                self._last_bypass_distance = self._distance_to_wp(
                    self._bypass_target
                )
                self.get_logger().warn(
                    "BYPASS created -> "
                    f"N={self._bypass_target[0]:.1f} "
                    f"E={self._bypass_target[1]:.1f}"
                )

            if self._bypass_target is not None:
                bypass_distance = self._distance_to_wp(self._bypass_target)
                if bypass_distance <= max(1.0, self.wp_accept):
                    if self._sensor_advance_target is not None:
                        self._bypass_target = self._sensor_advance_target
                        self._sensor_advance_target = None
                        self._last_bypass_distance = self._distance_to_wp(
                            self._bypass_target
                        )
                        self.get_logger().warn(
                            "BYPASS lateral clear -> advancing past obstacle"
                        )
                        return list(self._bypass_target)
                    self.get_logger().info("BYPASS complete -> resuming mission")
                    if self._bypass_obstacle is not None:
                        self._bypassed_obstacles.add(self._bypass_obstacle)
                    self._bypass_target = None
                    self._bypass_obstacle = None
                    self._sensor_advance_target = None
                    self._last_bypass_distance = float("inf")
                    self._avoid_active_t = 0.0
                    self._last_wp_progress_t = time.monotonic()
                    self._smooth_target = list(adjusted)
                    self._pub_avoiding.publish(Bool(data=False))
                    return adjusted

                if bypass_distance < self._last_bypass_distance - 0.25:
                    self._last_bypass_distance = bypass_distance
                    self._last_wp_progress_t = time.monotonic()

                adjusted = list(self._bypass_target)
                self._last_obstacle = obs or "bypass"
                if self._avoid_active_t == 0.0:
                    self._avoid_active_t = time.monotonic()
                self._pub_avoiding.publish(Bool(data=True))
                self.get_logger().warn(
                    "AVOID sidestep",
                    throttle_duration_sec=1.0,
                )
                a = self.avoid_smooth
                self._smooth_target[0] = self.current_x + a * (
                    adjusted[0] - self.current_x
                )
                self._smooth_target[1] = self.current_y + a * (
                    adjusted[1] - self.current_y
                )
                self._smooth_target[2] = self.current_z + a * (
                    adjusted[2] - self.current_z
                )
                return list(self._smooth_target)

        if obs is None:
            self._avoid_active_t = 0.0
            self._smooth_target = list(adjusted)
            self._pub_avoiding.publish(Bool(data=False))
            return adjusted

        if self._avoid_active_t == 0.0:
            self._avoid_active_t = time.monotonic()
        self._pub_avoiding.publish(Bool(data=True))
        self.get_logger().warn(f"AVOID {obs}", throttle_duration_sec=1.0)

        blocking_h = (
            self.max_alt
            if self.obstacle_source == "sensor_only"
            else self._blocking_height(target)
        )
        climb_alt = blocking_h + self.climb_clearance
        can_climb = climb_alt <= self.max_alt

        if obs == "front":
            if self.avoidance_strategy == "climb":
                if can_climb:
                    adjusted[2] = -climb_alt
                else:
                    adjusted[1] += self.sidestep_m
                    if self._lateral_blocked("right"):
                        self.get_logger().error(
                            "AVOID front: boxed in (too tall to climb, "
                            "right side also blocked)",
                            throttle_duration_sec=2.0,
                        )
        elif obs == "left":
            # Escaping "left" means steering east; if the right side is
            # also blocked, that escape would clip a second obstacle —
            # climb over instead, when there's altitude room to do so.
            if can_climb and self._lateral_blocked("right"):
                adjusted[2] = -climb_alt
            else:
                adjusted[1] += self.sidestep_m
        elif obs == "right":
            if can_climb and self._lateral_blocked("left"):
                adjusted[2] = -climb_alt
            else:
                adjusted[1] -= self.sidestep_m

        # Smooth only while avoiding — prevents setpoint step jumps
        a = self.avoid_smooth
        self._smooth_target[0] = self.current_x + a * (adjusted[0] - self.current_x)
        self._smooth_target[1] = self.current_y + a * (adjusted[1] - self.current_y)
        self._smooth_target[2] = self.current_z + a * (adjusted[2] - self.current_z)
        return list(self._smooth_target)

    def _blocking_height(self, target: list) -> float:
        """Tallest obstacle near the travel corridor."""
        return blocking_height(
            [self.current_x, self.current_y, self.current_z],
            target,
            OBSTACLE_BOXES,
            self.detection_margin,
            self.side_angle,
        )

    # ── Geo-cage / geofence ───────────────────────────────────────────────────

    def _inside_fence(self, north: float, east: float, down: float) -> bool:
        return self.fence.contains([north, east, down])

    def _apply_geocage(self, target: list) -> list:
        """
        Soft keep-in: clamp setpoints inside the fence, inset by geocage_margin.
        Disabled when geocage_enable is False.
        """
        if not self.geocage_enable:
            self._geocage_hit = False
            return target

        caged, self._geocage_hit = self.fence.clamp(target, self.geocage_margin)
        if self._geocage_hit:
            self.get_logger().warn(
                f"GEO-CAGE clamp → N={caged[0]:.1f} E={caged[1]:.1f} "
                f"alt={-caged[2]:.1f}",
                throttle_duration_sec=1.0,
            )
        return caged

    def _publish_fence_status(self):
        inside = (
            self._inside_fence(self.current_x, self.current_y, self.current_z)
            if self._have_position
            else True
        )
        msg = String()
        msg.data = (
            f"geocage={'on' if self.geocage_enable else 'off'} "
            f"geofence={'on' if self.geofence_enable else 'off'} "
            f"inside={int(inside)} caged={int(self._geocage_hit)} "
            f"breach={int(self._geofence_breach)} "
            f"action={self.geofence_action}"
        )
        self._pub_fence_status.publish(msg)

    def _publish_mission_status(self):
        """Publish a stable, presentation-friendly snapshot for demo tooling."""
        inside = (
            self._inside_fence(self.current_x, self.current_y, self.current_z)
            if self._have_position
            else True
        )
        payload = {
            "state": self._state.name,
            "mode": self.trajectory_mode.value,
            "avoidance_strategy": self.avoidance_strategy,
            "obstacle_source": self.obstacle_source,
            "sensor_fresh": self._sensor_data_fresh(),
            "lidar_sector_mins": list(self._sensor_mins or (-1.0, -1.0, -1.0)),
            "north_m": round(self.current_x, 2),
            "east_m": round(self.current_y, 2),
            "altitude_m": round(-self.current_z, 2),
            "waypoint": min(self._wp_index + 1, len(self.waypoints)),
            "waypoints_total": len(self.waypoints),
            "obstacle": self._last_obstacle or "none",
            "armed": self._flag_armed,
            "offboard": self._flag_offboard,
            "inside_geofence": inside,
            "failsafe": self._failsafe_reason or "none",
            "executive_mode": self._executive.mode.name,
            "executive_reason": self._executive.last_reason,
            "battery_frac": self._executive.resources.battery_frac,
            "link_quality": self._executive.resources.link_quality,
            "propellant_time_s": self._executive.resources.propellant_time_s,
            "skipped_waypoints": list(self._executive.skipped_waypoints),
        }
        self._pub_mission_status.publish(String(data=json.dumps(payload)))

    def _publish_executive_status(self):
        self._pub_executive_status.publish(
            String(data=json.dumps(self._executive.status_dict()))
        )

    def _send_px4_fence_enable(self, enable: bool):
        """Best-effort: enable/disable PX4 onboard geofence module (if configured)."""
        self._cmd(
            VehicleCommand.VEHICLE_CMD_DO_FENCE_ENABLE,
            param1=1.0 if enable else 0.0,
        )
        self.get_logger().info(
            f"Sent: DO_FENCE_ENABLE → {'ON' if enable else 'OFF'}"
        )

    # ── Publishers ────────────────────────────────────────────────────────────

    def _publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.timestamp = self._ts()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self._pub_ocm.publish(msg)

    def _publish_setpoint(self, ned: list):
        msg = TrajectorySetpoint()
        msg.timestamp = self._ts()
        msg.position = [float(v) for v in ned]
        msg.velocity = [float("nan")] * 3
        msg.acceleration = [float("nan")] * 3
        msg.jerk = [float("nan")] * 3
        msg.yaw = self._yaw_toward(ned)
        msg.yawspeed = float("nan")
        self._pub_sp.publish(msg)

    def _send_offboard_mode(self):
        self._cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Sent: DO_SET_MODE → OFFBOARD")

    def _send_arm(self):
        self._cmd(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=float(VehicleCommand.ARMING_ACTION_ARM),
        )
        self.get_logger().info("Sent: ARM")

    def _send_land(self):
        self._cmd(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Sent: NAV_LAND")

    def _send_rtl(self):
        self._cmd(VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH)
        self.get_logger().info("Sent: NAV_RETURN_TO_LAUNCH")

    def _cmd(self, command: int, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.timestamp = self._ts()
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self._pub_cmd.publish(msg)

    def _transition(self, new_state: State):
        self.get_logger().info(f"  {self._state.name} → {new_state.name}")
        self._state_machine.transition(new_state)

    @property
    def _state(self) -> State:
        return self._state_machine.state

    def _ts(self) -> int:
        return self.get_clock().now().nanoseconds // 1000

    def _yaw_toward(self, target: list) -> float:
        return yaw_toward([self.current_x, self.current_y, self.current_z], target)

    def destroy_node(self):
        if self._log_file:
            try:
                self._log_file.close()
            except OSError:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OffboardMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
