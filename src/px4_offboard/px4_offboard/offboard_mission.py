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
  └── Core logic  — state machine, trajectory, AABB avoidance, geo-cage, geofence, CSV log

Run:
  ros2 run px4_offboard offboard_mission
  ros2 run px4_offboard offboard_mission --ros-args --params-file config/offboard_mission.yaml

Toggle at runtime:
  ros2 topic pub --once /px4_offboard/geocage_enable std_msgs/msg/Bool "{data: true}"
  ros2 topic pub --once /px4_offboard/geofence_enable std_msgs/msg/Bool "{data: false}"
"""

from __future__ import annotations

import csv
import math
import time
from enum import Enum, auto
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleControlMode,
    VehicleLocalPosition,
    VehicleStatus,
)


# ── Enums ─────────────────────────────────────────────────────────────────────

class TrajectoryMode(Enum):
    WAYPOINTS = "waypoints"   # reactive path through obstacle field
    COURSE = "course"         # pre-planned clearance path (matches MAVSDK script)
    CIRCLE = "circle"         # orbit, then land after max_orbits


class State(Enum):
    PREFLIGHT = auto()
    ARMING = auto()
    TAKEOFF = auto()
    HOVER = auto()
    MOVE = auto()
    LANDING = auto()
    FAILSAFE = auto()


# Obstacle AABB: (east_m, north_m, size_e, size_n, height_m) — matches obstacle_world.sdf
OBSTACLE_BOXES = [
    (-6.0, 10.0, 3.0, 3.0, 4.0),   # OB1 red
    (10.0, 10.0, 3.0, 3.0, 6.0),   # OB2 orange
    (-8.0, 24.0, 4.0, 3.0, 4.0),   # OB3 green
    (6.0, 24.0, 2.0, 2.0, 5.0),    # OB4 blue
    (0.0, 38.0, 5.0, 3.0, 4.0),    # OB5 purple
]

# Reactive path: approaches obstacles so AABB avoidance must engage.
# NED [north, east, down] — z negative = up.
DEFAULT_WAYPOINTS = [
    [0.0, 0.0, -5.0],
    [5.0, 5.0, -5.0],
    [10.0, -6.0, -5.0],   # toward OB1
    [18.0, 0.0, -5.0],
    [24.0, 0.0, -5.0],    # OB3 / OB4 corridor
    [32.0, 0.0, -5.0],    # toward OB5
    [38.0, 0.0, -5.0],
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

        self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position",
            self._position_callback,
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
        self.create_subscription(
            Bool, "/px4_offboard/geocage_enable", self._geocage_toggle_cb, 10
        )
        self.create_subscription(
            Bool, "/px4_offboard/geofence_enable", self._geofence_toggle_cb, 10
        )

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self._pos_stamp = 0.0
        self._have_position = False

        self._nav_state = -1
        self._arming_state = -1
        self._flag_armed = False
        self._flag_offboard = False

        self._state = State.PREFLIGHT
        self._counter = 0
        self._hover_timer = 0.0
        self._mission_t = 0.0
        self._wp_index = 0
        self._last_wp_progress_t = 0.0
        self._avoid_active_t = 0.0
        self._smooth_target = [0.0, 0.0, -self.hover_alt]
        self._sensor_dir: str | None = None
        self._sensor_stamp = 0.0
        self._failsafe_reason = ""
        self._land_sent = False
        self._geocage_hit = False
        self._geofence_breach = False

        self._log_file = None
        self._log_writer = None
        self._open_log()

        self.create_timer(0.1, self._control_loop)
        self.get_logger().info(
            f"OffboardMission ready — mode={self.trajectory_mode.value} "
            f"alt={self.hover_alt}m waypoints={len(self.waypoints)} "
            f"geocage={'ON' if self.geocage_enable else 'OFF'} "
            f"geofence={'ON' if self.geofence_enable else 'OFF'} "
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
        self.declare_parameter("climb_clearance_m", 1.5)
        self.declare_parameter("max_alt_m", 12.0)
        self.declare_parameter("circle_radius_m", 8.0)
        self.declare_parameter("circle_period_s", 20.0)
        self.declare_parameter("max_orbits", 2.0)
        self.declare_parameter("position_timeout_s", 1.5)
        self.declare_parameter("mission_timeout_s", 180.0)
        self.declare_parameter("avoid_stuck_s", 12.0)
        self.declare_parameter("sensor_timeout_s", 0.5)
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
        self.climb_clearance = float(self.get_parameter("climb_clearance_m").value)
        self.max_alt = float(self.get_parameter("max_alt_m").value)
        self.circle_radius = float(self.get_parameter("circle_radius_m").value)
        self.circle_period = float(self.get_parameter("circle_period_s").value)
        self.max_orbits = float(self.get_parameter("max_orbits").value)
        self.position_timeout = float(self.get_parameter("position_timeout_s").value)
        self.mission_timeout = float(self.get_parameter("mission_timeout_s").value)
        self.avoid_stuck_s = float(self.get_parameter("avoid_stuck_s").value)
        self.sensor_timeout = float(self.get_parameter("sensor_timeout_s").value)
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

        if self.trajectory_mode == TrajectoryMode.COURSE:
            # Clearance path flies above tallest obstacle (OB2 = 6 m)
            self.hover_alt = max(self.hover_alt, 8.0)
            z = -self.hover_alt
            self.waypoints = [[n, e, z] for e, n in DEFAULT_COURSE]
        else:
            self.waypoints = [
                [wp[0], wp[1], -self.hover_alt] for wp in DEFAULT_WAYPOINTS
            ]

    # ── Telemetry log ─────────────────────────────────────────────────────────

    def _open_log(self):
        self.log_dir.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        path = self.log_dir / f"flight_log_mission_{stamp}.csv"
        self._log_file = open(path, "w", newline="")
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
                "wp_index",
                "geocage",
                "geofence",
                "inside",
                "caged",
            ]
        )
        self.get_logger().info(f"Logging to {path}")

    def _log_row(self, target: list, obstacle: str | None):
        if self._log_writer is None:
            return
        inside = self._inside_fence(self.current_x, self.current_y, self.current_z)
        self._log_writer.writerow(
            [
                time.strftime("%H:%M:%S"),
                self._state.name,
                round(self.current_x, 3),
                round(self.current_y, 3),
                round(self.current_z, 3),
                round(target[0], 3),
                round(target[1], 3),
                round(target[2], 3),
                obstacle or "",
                self._wp_index,
                int(self.geocage_enable),
                int(self.geofence_enable),
                int(inside),
                int(self._geocage_hit),
            ]
        )
        self._log_file.flush()

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _position_callback(self, msg: VehicleLocalPosition):
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_z = msg.z
        self._pos_stamp = time.monotonic()
        self._have_position = True

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

        if self._state not in (State.PREFLIGHT, State.FAILSAFE, State.LANDING):
            if self._check_failsafes():
                return

        if self._state == State.PREFLIGHT:
            target = [0.0, 0.0, -self.hover_alt]
            target = self._apply_geocage(target)
            self._publish_setpoint(target)
            if self._counter == self.preflight_cycles:
                self._send_offboard_mode()
            if self._counter == self.preflight_cycles + 5:
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
            armed = self._flag_armed or self._arming_state == 2
            offboard = self._flag_offboard or self._nav_state == 14
            if armed and offboard:
                self._transition(State.TAKEOFF)
            # Fallback: already airborne from prior arm (stale sim session)
            elif self._have_position and abs(self.current_z) > 1.0 and self._counter > 50:
                self.get_logger().warn(
                    "ARMING fallback — position indicates airborne; advancing"
                )
                self._transition(State.TAKEOFF)

        elif self._state == State.TAKEOFF:
            target = [0.0, 0.0, -self.hover_alt]
            target = self._apply_geocage(target)
            self._publish_setpoint(target)
            if abs(self.current_z - (-self.hover_alt)) < self.takeoff_z_tol:
                self._transition(State.HOVER)

        elif self._state == State.HOVER:
            target = [0.0, 0.0, -self.hover_alt]
            target = self._apply_geocage(target)
            self._publish_setpoint(target)
            self._hover_timer += 0.1
            if self._hover_timer >= self.hover_hold_s:
                self._last_wp_progress_t = time.monotonic()
                self._transition(State.MOVE)

        elif self._state == State.MOVE:
            target = self._next_target()
            obs = self._detect_obstacle(target)
            target = self._apply_avoidance(target, obs)
            target = self._apply_geocage(target)
            self._publish_setpoint(target)
            self._log_row(target, obs)
            self._mission_t += 0.1

        elif self._state == State.LANDING:
            hold = [self.current_x, self.current_y, 0.0]
            hold = self._apply_geocage(hold)
            self._publish_setpoint(hold)
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
            else:
                hold = [self.current_x, self.current_y, min(self.current_z, -2.0)]
                self._publish_setpoint(hold)
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

        self._counter += 1

    def _check_failsafes(self) -> bool:
        now = time.monotonic()

        if self._have_position and (now - self._pos_stamp) > self.position_timeout:
            return self._enter_failsafe("position timeout (XRCE / EKF)")

        if self._state == State.MOVE and self._mission_t > self.mission_timeout:
            return self._enter_failsafe("mission timeout")

        if (
            self._state == State.MOVE
            and self._avoid_active_t > 0
            and (now - self._avoid_active_t) > self.avoid_stuck_s
            and (now - self._last_wp_progress_t) > self.avoid_stuck_s
        ):
            return self._enter_failsafe("stuck in avoidance without waypoint progress")

        if self.geofence_enable and self._have_position:
            # Only enforce once airborne — avoids false trip if restarting mid-field
            if self._state in (State.TAKEOFF, State.HOVER, State.MOVE):
                if not self._inside_fence(self.current_x, self.current_y, self.current_z):
                    self._geofence_breach = True
                    return self._enter_failsafe(
                        f"geofence breach at N={self.current_x:.1f} E={self.current_y:.1f} "
                        f"alt={-self.current_z:.1f}"
                    )

        return False

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
        angle = (2 * math.pi / self.circle_period) * self._mission_t
        return [
            self.circle_radius * math.cos(angle),
            self.circle_radius * math.sin(angle),
            -self.hover_alt,
        ]

    def _distance_to_wp(self, target: list) -> float:
        dx = self.current_x - target[0]
        dy = self.current_y - target[1]
        dz = self.current_z - target[2]
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    # ── Obstacle detection (AABB + optional sensor) ───────────────────────────

    def _detect_obstacle(self, target: list) -> str | None:
        """
        Prefer live sensor on /px4_offboard/obstacle_dir when fresh.
        Otherwise use AABB geometry against obstacle_world.sdf.
        """
        now = time.monotonic()
        if self._sensor_stamp and (now - self._sensor_stamp) < self.sensor_timeout:
            return self._sensor_dir

        dx = target[0] - self.current_x
        dy = target[1] - self.current_y
        if abs(dx) < 1e-3 and abs(dy) < 1e-3:
            return None
        travel_yaw = math.degrees(math.atan2(dy, dx))

        best: tuple[float, str] | None = None
        alt_agl = -self.current_z

        for east, north, size_e, size_n, height in OBSTACLE_BOXES:
            if alt_agl > height + 0.5:
                continue  # already above this obstacle

            # Vehicle in NED (x=north, y=east); box center in ENU
            cx, cy = north, east
            half_n, half_e = size_n / 2.0, size_e / 2.0
            # Distance from point to AABB in horizontal plane
            nearest_n = min(max(self.current_x, cx - half_n), cx + half_n)
            nearest_e = min(max(self.current_y, cy - half_e), cy + half_e)
            dn = nearest_n - self.current_x
            de = nearest_e - self.current_y
            dist = math.hypot(dn, de)

            # Also consider distance to box center for bearing
            bearing_n = cx - self.current_x
            bearing_e = cy - self.current_y
            center_dist = math.hypot(bearing_n, bearing_e)
            trigger = half_n + half_e  # rough half-diagonal proxy
            trigger = max(half_n, half_e) + self.detection_margin

            if dist > trigger and center_dist > trigger:
                continue

            angle = math.degrees(math.atan2(bearing_e, bearing_n))
            rel = (angle - travel_yaw + 180.0) % 360.0 - 180.0

            if abs(rel) < self.front_angle:
                label = "front"
            elif 0 < rel < self.side_angle:
                label = "left"
            elif -self.side_angle < rel < 0:
                label = "right"
            else:
                continue

            if best is None or dist < best[0]:
                best = (dist, label)

        return best[1] if best else None

    def _apply_avoidance(self, target: list, obs: str | None) -> list:
        adjusted = list(target)

        if obs is None:
            self._avoid_active_t = 0.0
            self._smooth_target = list(adjusted)
            self._pub_avoiding.publish(Bool(data=False))
            return adjusted

        if self._avoid_active_t == 0.0:
            self._avoid_active_t = time.monotonic()
        self._pub_avoiding.publish(Bool(data=True))
        self.get_logger().warn(f"AVOID {obs}", throttle_duration_sec=1.0)

        if obs == "front":
            # Prefer climb if obstacle is short enough; else sidestep east
            blocking_h = self._blocking_height(target)
            climb_alt = blocking_h + self.climb_clearance
            if climb_alt <= self.max_alt:
                adjusted[2] = -climb_alt
            else:
                adjusted[1] += self.sidestep_m
        elif obs == "left":
            adjusted[1] += self.sidestep_m
        elif obs == "right":
            adjusted[1] -= self.sidestep_m

        # Smooth only while avoiding — prevents setpoint step jumps
        a = self.avoid_smooth
        self._smooth_target[0] = self.current_x + a * (adjusted[0] - self.current_x)
        self._smooth_target[1] = self.current_y + a * (adjusted[1] - self.current_y)
        self._smooth_target[2] = self.current_z + a * (adjusted[2] - self.current_z)
        return list(self._smooth_target)

    def _blocking_height(self, target: list) -> float:
        """Tallest obstacle near the travel corridor."""
        dx = target[0] - self.current_x
        dy = target[1] - self.current_y
        travel_yaw = math.degrees(math.atan2(dy, dx)) if (abs(dx) + abs(dy)) > 1e-3 else 0.0
        tallest = 0.0
        for east, north, size_e, size_n, height in OBSTACLE_BOXES:
            cx, cy = north, east
            bearing_n = cx - self.current_x
            bearing_e = cy - self.current_y
            dist = math.hypot(bearing_n, bearing_e)
            if dist > max(size_n, size_e) + self.detection_margin + 1.0:
                continue
            angle = math.degrees(math.atan2(bearing_e, bearing_n))
            rel = (angle - travel_yaw + 180.0) % 360.0 - 180.0
            if abs(rel) < self.side_angle:
                tallest = max(tallest, height)
        return tallest

    # ── Geo-cage / geofence ───────────────────────────────────────────────────

    def _inside_fence(self, north: float, east: float, down: float) -> bool:
        alt = -down
        return (
            self.fence_n_min <= north <= self.fence_n_max
            and self.fence_e_min <= east <= self.fence_e_max
            and alt <= self.fence_alt_max + 0.05
        )

    def _apply_geocage(self, target: list) -> list:
        """
        Soft keep-in: clamp setpoints inside the fence, inset by geocage_margin.
        Disabled when geocage_enable is False.
        """
        if not self.geocage_enable:
            self._geocage_hit = False
            return target

        m = self.geocage_margin
        n_lo, n_hi = self.fence_n_min + m, self.fence_n_max - m
        e_lo, e_hi = self.fence_e_min + m, self.fence_e_max - m
        if n_lo >= n_hi or e_lo >= e_hi:
            n_lo, n_hi = self.fence_n_min, self.fence_n_max
            e_lo, e_hi = self.fence_e_min, self.fence_e_max

        caged = list(target)
        caged[0] = min(max(caged[0], n_lo), n_hi)
        caged[1] = min(max(caged[1], e_lo), e_hi)
        # Clamp altitude AGL (NED down is negative up)
        min_down = -self.fence_alt_max
        if caged[2] < min_down:
            caged[2] = min_down

        self._geocage_hit = caged != list(target)
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
        self._state = new_state

    def _ts(self) -> int:
        return self.get_clock().now().nanoseconds // 1000

    def _yaw_toward(self, target: list) -> float:
        dx = target[0] - self.current_x
        dy = target[1] - self.current_y
        if abs(dx) < 0.1 and abs(dy) < 0.1:
            return float("nan")
        return math.atan2(dy, dx)

    def destroy_node(self):
        if self._log_file:
            self._log_file.close()
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
