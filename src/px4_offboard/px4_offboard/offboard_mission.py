"""
offboard_mission.py — PX4 OFFBOARD mission node (ROS 2 + px4_msgs + XRCE-DDS)

State machine:
  PREFLIGHT → ARMING → TAKEOFF → MISSION → LANDING

Trajectory modes (set TRAJECTORY_MODE at top of file):
  TrajectoryMode.WAYPOINTS — fly through a list of NED positions
  TrajectoryMode.CIRCLE    — continuous circular orbit

Pipeline:
  ROS 2 node → XRCE-DDS → PX4 uORB

Topics published:
  /fmu/in/offboard_control_mode
  /fmu/in/trajectory_setpoint
  /fmu/in/vehicle_command

Topics subscribed:
  /fmu/out/vehicle_local_position   — current NED position (waypoint acceptance)
  /fmu/out/vehicle_status           — arming / nav state confirmation

Run:
  ros2 run px4_offboard offboard_mission

Change trajectory:
  Edit TRAJECTORY_MODE and WAYPOINTS / CIRCLE_* constants below.
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


# ── Mission config ────────────────────────────────────────────────────────────

HOVER_ALT_M      = 5.0    # meters AGL (NED z = -5.0)
ACCEPT_RADIUS_M  = 0.4    # waypoint acceptance radius (meters)
PREFLIGHT_CYCLES = 20     # setpoint cycles before arming (20 × 100ms = 2s)

# Waypoints in NED frame: [north_m, east_m, down_m]
# down_m is negative (z = -altitude)
WAYPOINTS = [
    [ 0.0,   0.0, -HOVER_ALT_M],   # WP0 — home / takeoff hover
    [10.0,   0.0, -HOVER_ALT_M],   # WP1 — north  10m
    [10.0,  10.0, -HOVER_ALT_M],   # WP2 — NE corner
    [ 0.0,  10.0, -HOVER_ALT_M],   # WP3 — east   10m
    [ 0.0,   0.0, -HOVER_ALT_M],   # WP4 — return home
]

# Circular orbit
CIRCLE_RADIUS_M  = 8.0    # meters
CIRCLE_PERIOD_S  = 20.0   # seconds per full orbit (2π rad)


# ── State machine ─────────────────────────────────────────────────────────────

class State(Enum):
    PREFLIGHT = auto()   # streaming setpoints, not yet armed
    ARMING    = auto()   # mode + arm commands sent, waiting for confirmation
    TAKEOFF   = auto()   # climbing to HOVER_ALT, waiting for altitude lock
    MISSION   = auto()   # executing trajectory
    LANDING   = auto()   # descending and disarming


# ── Node ──────────────────────────────────────────────────────────────────────

class OffboardMission(Node):

    def __init__(self):
        super().__init__("offboard_mission")

        # QoS for PX4 inbound topics (publish to FMU)
        qos_pub = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        # QoS for PX4 outbound topics (subscribe from FMU)
        qos_sub = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Publishers
        self._pub_ocm = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", qos_pub
        )
        self._pub_sp = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", qos_pub
        )
        self._pub_cmd = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", qos_pub
        )

        # Subscribers
        self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position",
            self._cb_local_pos,
            qos_sub,
        )
        self.create_subscription(
            VehicleStatus,
            "/fmu/out/vehicle_status",
            self._cb_vehicle_status,
            qos_sub,
        )

        # State
        self._state         = State.PREFLIGHT
        self._counter       = 0
        self._wp_index      = 0
        self._mission_t     = 0.0        # seconds into MISSION phase
        self._pos           = [0.0, 0.0, 0.0]  # current NED (x, y, z)
        self._nav_state     = -1
        self._arming_state  = -1

        self.create_timer(0.1, self._timer_cb)   # 10 Hz
        self.get_logger().info(
            f"OffboardMission started — mode={TRAJECTORY_MODE.name}"
        )

    # ── Subscriptions ─────────────────────────────────────────────────────────

    def _cb_local_pos(self, msg: VehicleLocalPosition):
        self._pos = [msg.x, msg.y, msg.z]

    def _cb_vehicle_status(self, msg: VehicleStatus):
        self._nav_state    = msg.nav_state
        self._arming_state = msg.arming_state

    # ── 10 Hz timer ───────────────────────────────────────────────────────────

    def _timer_cb(self):
        self._publish_offboard_control_mode()

        # ── PREFLIGHT ──────────────────────────────────────────────────────
        if self._state == State.PREFLIGHT:
            self._publish_setpoint([0.0, 0.0, -HOVER_ALT_M])
            if self._counter == PREFLIGHT_CYCLES:
                self._send_offboard_mode()
            if self._counter == PREFLIGHT_CYCLES + 5:
                self._send_arm()
                self._state = State.ARMING
                self.get_logger().info("→ ARMING")

        # ── ARMING ────────────────────────────────────────────────────────
        elif self._state == State.ARMING:
            self._publish_setpoint([0.0, 0.0, -HOVER_ALT_M])
            # nav_state 14 = OFFBOARD, arming_state 2 = ARMED
            if self._nav_state == 14 and self._arming_state == 2:
                self._state = State.TAKEOFF
                self.get_logger().info("→ TAKEOFF")

        # ── TAKEOFF ───────────────────────────────────────────────────────
        elif self._state == State.TAKEOFF:
            self._publish_setpoint([0.0, 0.0, -HOVER_ALT_M])
            if abs(self._pos[2] - (-HOVER_ALT_M)) < ACCEPT_RADIUS_M:
                self._state     = State.MISSION
                self._wp_index  = 0
                self._mission_t = 0.0
                self.get_logger().info("→ MISSION")

        # ── MISSION ───────────────────────────────────────────────────────
        elif self._state == State.MISSION:
            sp = self._next_setpoint()
            self._publish_setpoint(sp)
            self._mission_t += 0.1

        # ── LANDING ───────────────────────────────────────────────────────
        elif self._state == State.LANDING:
            self._publish_setpoint([0.0, 0.0, 0.0])   # descend to ground
            self._send_land()

        self._counter += 1

    # ── Trajectory generators ─────────────────────────────────────────────────

    def _next_setpoint(self) -> list:
        if TRAJECTORY_MODE == TrajectoryMode.WAYPOINTS:
            return self._waypoint_setpoint()
        return self._circle_setpoint()

    def _waypoint_setpoint(self) -> list:
        if self._wp_index >= len(WAYPOINTS):
            self._state = State.LANDING
            self.get_logger().info("→ LANDING (waypoints complete)")
            return WAYPOINTS[-1]

        target = WAYPOINTS[self._wp_index]
        dist = math.sqrt(
            (self._pos[0] - target[0]) ** 2
            + (self._pos[1] - target[1]) ** 2
            + (self._pos[2] - target[2]) ** 2
        )

        if dist < ACCEPT_RADIUS_M:
            self.get_logger().info(
                f"Waypoint {self._wp_index} reached — {target}"
            )
            self._wp_index += 1

        return target

    def _circle_setpoint(self) -> list:
        angle = (2 * math.pi / CIRCLE_PERIOD_S) * self._mission_t
        x = CIRCLE_RADIUS_M * math.cos(angle)
        y = CIRCLE_RADIUS_M * math.sin(angle)
        z = -HOVER_ALT_M
        return [x, y, z]

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
        msg.yaw          = self._yaw_to_target(ned)
        msg.yawspeed     = float("nan")
        self._pub_sp.publish(msg)

    # ── VehicleCommand helpers ────────────────────────────────────────────────

    def _send_offboard_mode(self):
        self._vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0,   # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
            param2=6.0,   # PX4_CUSTOM_MAIN_MODE_OFFBOARD
        )
        self.get_logger().info("Sent: DO_SET_MODE → OFFBOARD")

    def _send_arm(self):
        self._vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=float(VehicleCommand.ARMING_ACTION_ARM),
        )
        self.get_logger().info("Sent: ARM")

    def _send_land(self):
        self._vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Sent: LAND")

    def _vehicle_command(self, command: int, param1=0.0, param2=0.0):
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

    def _ts(self) -> int:
        return self.get_clock().now().nanoseconds // 1000

    def _yaw_to_target(self, target: list) -> float:
        """Face the direction of travel (yaw toward next waypoint)."""
        dx = target[0] - self._pos[0]
        dy = target[1] - self._pos[1]
        if abs(dx) < 0.1 and abs(dy) < 0.1:
            return float("nan")   # hold current yaw when nearly at target
        return math.atan2(dy, dx)


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
