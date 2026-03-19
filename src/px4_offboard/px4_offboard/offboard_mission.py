"""
offboard_mission.py — PX4 state-driven autonomous flight controller
                       (ROS 2 + px4_msgs + Micro XRCE-DDS)

State machine:
  PREFLIGHT → ARMING → TAKEOFF → HOVER → MOVE → LANDING

Transitions:
  PREFLIGHT → ARMING  : after PREFLIGHT_CYCLES setpoint stream (2 s)
  ARMING    → TAKEOFF : vehicle_status confirms nav_state=14, arming_state=2
  TAKEOFF   → HOVER   : |current_z - target_z| < 0.2 m
  HOVER     → MOVE    : time_in_state > 3 s
  MOVE      → LANDING : all waypoints reached

Architecture:
  ├── Publishers
  │   ├── /fmu/in/offboard_control_mode
  │   ├── /fmu/in/trajectory_setpoint
  │   └── /fmu/in/vehicle_command
  ├── Subscribers
  │   ├── /fmu/out/vehicle_local_position   ← position feedback
  │   └── /fmu/out/vehicle_status           ← arming / nav state
  └── Core logic
      ├── State machine
      ├── Feedback controller
      └── Trajectory generator  (WAYPOINTS | CIRCLE)

Run:
  ros2 run px4_offboard offboard_mission

Switch trajectory:
  Set TRAJECTORY_MODE to TrajectoryMode.WAYPOINTS or TrajectoryMode.CIRCLE
"""

import math
import rclpy
from enum import Enum, auto
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)


# ── Trajectory mode ───────────────────────────────────────────────────────────

class TrajectoryMode(Enum):
    WAYPOINTS = auto()
    CIRCLE    = auto()

TRAJECTORY_MODE = TrajectoryMode.WAYPOINTS


# ── Mission parameters ────────────────────────────────────────────────────────

HOVER_ALT_M       = 3.0    # meters AGL  (NED z = -3.0)
PREFLIGHT_CYCLES  = 20     # 20 × 100 ms = 2 s pre-stream before arm
HOVER_HOLD_S      = 3.0    # seconds to hold HOVER before advancing to MOVE
TAKEOFF_Z_TOL_M   = 0.2    # TAKEOFF → HOVER acceptance: |z_err| < this
WP_ACCEPT_M       = 0.3    # MOVE waypoint acceptance radius (3-D)

# Waypoints  [north_m, east_m, down_m]   NED frame, z negative = up
WAYPOINTS = [
    [0.0, 0.0, -HOVER_ALT_M],   # WP0 — home / initial hover
    [2.0, 0.0, -HOVER_ALT_M],   # WP1 — north 2 m
    [2.0, 2.0, -HOVER_ALT_M],   # WP2 — NE corner
    [0.0, 2.0, -HOVER_ALT_M],   # WP3 — east 2 m
]

# Circular orbit
CIRCLE_RADIUS_M = 8.0     # metres
CIRCLE_PERIOD_S = 20.0    # seconds per full 2π orbit


# ── States ────────────────────────────────────────────────────────────────────

class State(Enum):
    PREFLIGHT = auto()   # stream setpoints before arming
    ARMING    = auto()   # mode switch + arm commands sent
    TAKEOFF   = auto()   # climb to HOVER_ALT; wait for z convergence
    HOVER     = auto()   # stabilise; advance after HOVER_HOLD_S
    MOVE      = auto()   # navigate through waypoints / orbit
    LANDING   = auto()   # descend and disarm


# ── Node ──────────────────────────────────────────────────────────────────────

class OffboardMission(Node):

    def __init__(self):
        super().__init__("offboard_mission")

        qos_pub = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        qos_sub = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ── Publishers ────────────────────────────────────────────────────
        self._pub_ocm = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", qos_pub
        )
        self._pub_sp = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", qos_pub
        )
        self._pub_cmd = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", qos_pub
        )

        # ── Subscribers ───────────────────────────────────────────────────
        self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position",
            self._position_callback,
            qos_sub,
        )
        self.create_subscription(
            VehicleStatus,
            "/fmu/out/vehicle_status",
            self._status_callback,
            qos_sub,
        )

        # ── Feedback state ────────────────────────────────────────────────
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0

        self._nav_state    = -1
        self._arming_state = -1

        # ── Mission state ─────────────────────────────────────────────────
        self._state        = State.PREFLIGHT
        self._counter      = 0
        self._hover_timer  = 0.0    # seconds spent in HOVER
        self._mission_t    = 0.0    # seconds spent in MOVE (for circle)
        self._wp_index     = 0

        self.create_timer(0.1, self._control_loop)  # 10 Hz
        self.get_logger().info(
            f"OffboardMission ready — mode={TRAJECTORY_MODE.name}"
        )

    # ── Subscribers ───────────────────────────────────────────────────────────

    def _position_callback(self, msg: VehicleLocalPosition):
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_z = msg.z

    def _status_callback(self, msg: VehicleStatus):
        self._nav_state    = msg.nav_state
        self._arming_state = msg.arming_state

    # ── 10 Hz control loop ────────────────────────────────────────────────────

    def _control_loop(self):
        # Heartbeat — must NEVER stop while in OFFBOARD mode
        self._publish_offboard_control_mode()

        # ── PREFLIGHT ──────────────────────────────────────────────────────
        if self._state == State.PREFLIGHT:
            self._publish_setpoint([0.0, 0.0, -HOVER_ALT_M])

            if self._counter == PREFLIGHT_CYCLES:
                self._send_offboard_mode()
            if self._counter == PREFLIGHT_CYCLES + 5:
                self._send_arm()
                self._transition(State.ARMING)

        # ── ARMING ────────────────────────────────────────────────────────
        elif self._state == State.ARMING:
            self._publish_setpoint([0.0, 0.0, -HOVER_ALT_M])
            # Wait for PX4 confirmation before commanding motion
            if self._nav_state == 14 and self._arming_state == 2:
                self._transition(State.TAKEOFF)

        # ── TAKEOFF ───────────────────────────────────────────────────────
        elif self._state == State.TAKEOFF:
            target = [0.0, 0.0, -HOVER_ALT_M]
            self._publish_setpoint(target)
            if abs(self.current_z - (-HOVER_ALT_M)) < TAKEOFF_Z_TOL_M:
                self._transition(State.HOVER)

        # ── HOVER ─────────────────────────────────────────────────────────
        elif self._state == State.HOVER:
            self._publish_setpoint([0.0, 0.0, -HOVER_ALT_M])
            self._hover_timer += 0.1
            if self._hover_timer >= HOVER_HOLD_S:
                self._transition(State.MOVE)

        # ── MOVE ──────────────────────────────────────────────────────────
        elif self._state == State.MOVE:
            target = self._next_target()
            target = self._apply_avoidance(target)
            self._publish_setpoint(target)
            self._mission_t += 0.1

        # ── LANDING ───────────────────────────────────────────────────────
        elif self._state == State.LANDING:
            self._publish_setpoint([0.0, 0.0, 0.0])
            self._send_land()

        self._counter += 1

    # ── Trajectory generator ──────────────────────────────────────────────────

    def _next_target(self) -> list:
        if TRAJECTORY_MODE == TrajectoryMode.CIRCLE:
            return self._circle_target()
        return self._waypoint_target()

    def _waypoint_target(self) -> list:
        if self._wp_index >= len(WAYPOINTS):
            self._transition(State.LANDING)
            return WAYPOINTS[-1]

        target = WAYPOINTS[self._wp_index]
        if self._distance_to_wp(target) < WP_ACCEPT_M:
            self.get_logger().info(
                f"WP {self._wp_index} reached  {target}"
            )
            self._wp_index += 1

        return target

    def _circle_target(self) -> list:
        angle = (2 * math.pi / CIRCLE_PERIOD_S) * self._mission_t
        return [
            CIRCLE_RADIUS_M * math.cos(angle),
            CIRCLE_RADIUS_M * math.sin(angle),
            -HOVER_ALT_M,
        ]

    # ── Feedback helpers ──────────────────────────────────────────────────────

    def _distance_to_wp(self, target: list) -> float:
        dx = self.current_x - target[0]
        dy = self.current_y - target[1]
        dz = self.current_z - target[2]
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    # ── Obstacle avoidance hook ───────────────────────────────────────────────

    def _apply_avoidance(self, target: list) -> list:
        """
        Hook for reactive obstacle avoidance.

        Replace obstacle_detected() with real sensor input:
          - ROS 2 subscriber (LiDAR, depth camera)
          - offboard_avoidance.detect_obstacle()

        Example — sidestep east if obstacle ahead:
            if obstacle_detected == "front":
                target[1] += 1.0
        """
        return target   # pass-through until sensor is connected

    # ── Publishers ────────────────────────────────────────────────────────────

    def _publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.timestamp    = self._ts()
        msg.position     = True
        msg.velocity     = False
        msg.acceleration = False
        msg.attitude     = False
        msg.body_rate    = False
        self._pub_ocm.publish(msg)

    def _publish_setpoint(self, ned: list):
        msg = TrajectorySetpoint()
        msg.timestamp    = self._ts()
        msg.position     = [float(v) for v in ned]
        msg.velocity     = [float("nan")] * 3
        msg.acceleration = [float("nan")] * 3
        msg.jerk         = [float("nan")] * 3
        msg.yaw          = self._yaw_toward(ned)
        msg.yawspeed     = float("nan")
        self._pub_sp.publish(msg)

    # ── VehicleCommand ────────────────────────────────────────────────────────

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

    def _cmd(self, command: int, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.timestamp        = self._ts()
        msg.command          = command
        msg.param1           = param1
        msg.param2           = param2
        msg.target_system    = 1
        msg.target_component = 1
        msg.source_system    = 1
        msg.source_component = 1
        msg.from_external    = True
        self._pub_cmd.publish(msg)

    # ── Utilities ─────────────────────────────────────────────────────────────

    def _transition(self, new_state: State):
        self.get_logger().info(f"  {self._state.name} → {new_state.name}")
        self._state = new_state

    def _ts(self) -> int:
        return self.get_clock().now().nanoseconds // 1000

    def _yaw_toward(self, target: list) -> float:
        """Face direction of travel; hold current yaw when near target."""
        dx = target[0] - self.current_x
        dy = target[1] - self.current_y
        if abs(dx) < 0.1 and abs(dy) < 0.1:
            return float("nan")
        return math.atan2(dy, dx)


# ── Entry point ───────────────────────────────────────────────────────────────

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
