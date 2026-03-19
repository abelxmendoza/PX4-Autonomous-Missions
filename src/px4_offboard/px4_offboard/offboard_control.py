"""
offboard_control.py — PX4 OFFBOARD hover node (ROS 2 + px4_msgs + XRCE-DDS)

Pipeline:
  ROS 2 node → XRCE-DDS → PX4 uORB

Topics published:
  /fmu/in/offboard_control_mode   — heartbeat (position=True)
  /fmu/in/trajectory_setpoint     — target: x=0, y=0, z=-5 NED
  /fmu/in/vehicle_command         — arm + OFFBOARD mode switch

Sequence:
  t=0s   start streaming setpoints (PX4 requires stream BEFORE offboard)
  t=2s   send VEHICLE_CMD_DO_SET_MODE → OFFBOARD
  t=2.5s send VEHICLE_CMD_COMPONENT_ARM_DISARM → ARM

Run:
  ros2 run px4_offboard offboard_control
"""

import rclpy
import math
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand


class OffboardControl(Node):

    # How many setpoint cycles to stream before switching to OFFBOARD
    _SETPOINT_PREFLIGHT_COUNT = 20   # 20 × 100ms = 2s

    def __init__(self):
        super().__init__("offboard_control")

        # PX4 uORB bridge requires BEST_EFFORT + TRANSIENT_LOCAL
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._pub_ocm = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", qos
        )
        self._pub_sp = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", qos
        )
        self._pub_cmd = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", qos
        )

        self._counter = 0
        self._armed   = False

        # 10 Hz control loop
        self.create_timer(0.1, self._timer_cb)
        self.get_logger().info("OffboardControl node started — streaming setpoints...")

    # ── Timer callback (10 Hz) ───────────────────────────────────────────────

    def _timer_cb(self):
        self._publish_offboard_control_mode()
        self._publish_trajectory_setpoint()

        if self._counter == self._SETPOINT_PREFLIGHT_COUNT:
            self._send_offboard_mode()
            self.get_logger().info("Sent: OFFBOARD mode")

        if self._counter == self._SETPOINT_PREFLIGHT_COUNT + 5:
            self._send_arm()
            self.get_logger().info("Sent: ARM command")
            self._armed = True

        if self._counter % 10 == 0:
            self.get_logger().info(f"[{self._counter}] Streaming setpoints — armed={self._armed}")

        self._counter += 1

    # ── Publishers ───────────────────────────────────────────────────────────

    def _publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.timestamp    = self._timestamp()
        msg.position     = True
        msg.velocity     = False
        msg.acceleration = False
        msg.attitude     = False
        msg.body_rate    = False
        self._pub_ocm.publish(msg)

    def _publish_trajectory_setpoint(self):
        msg = TrajectorySetpoint()
        msg.timestamp = self._timestamp()
        # NED frame: x=north, y=east, z=down (negative = up)
        msg.position     = [0.0, 0.0, -5.0]
        msg.velocity     = [float("nan")] * 3
        msg.acceleration = [float("nan")] * 3
        msg.jerk         = [float("nan")] * 3
        msg.yaw          = 0.0        # face north
        msg.yawspeed     = float("nan")
        self._pub_sp.publish(msg)

    # ── VehicleCommand helpers ────────────────────────────────────────────────

    def _send_offboard_mode(self):
        msg = VehicleCommand()
        msg.timestamp        = self._timestamp()
        msg.command          = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        msg.param1           = 1.0   # base mode: MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        msg.param2           = 6.0   # custom main mode: PX4_CUSTOM_MAIN_MODE_OFFBOARD
        msg.target_system    = 1
        msg.target_component = 1
        msg.source_system    = 1
        msg.source_component = 1
        msg.from_external    = True
        self._pub_cmd.publish(msg)

    def _send_arm(self):
        msg = VehicleCommand()
        msg.timestamp        = self._timestamp()
        msg.command          = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        msg.param1           = float(VehicleCommand.ARMING_ACTION_ARM)
        msg.target_system    = 1
        msg.target_component = 1
        msg.source_system    = 1
        msg.source_component = 1
        msg.from_external    = True
        self._pub_cmd.publish(msg)

    # ── Utility ───────────────────────────────────────────────────────────────

    def _timestamp(self) -> int:
        """PX4 expects microseconds since system boot."""
        return self.get_clock().now().nanoseconds // 1000


def main(args=None):
    rclpy.init(args=args)
    node = OffboardControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
