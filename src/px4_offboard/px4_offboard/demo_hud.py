"""Compact console presentation for the PX4 autonomous mission demo."""

from __future__ import annotations

import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class DemoHud(Node):
    def __init__(self) -> None:
        super().__init__("demo_hud")
        self._last_state = None
        self._last_obstacle = "none"
        self._last_summary = 0.0
        self.create_subscription(
            String, "/px4_offboard/mission_status", self._status_callback, 10
        )
        self.get_logger().info("DEMO READY | waiting for PX4 mission telemetry")

    def _status_callback(self, message: String) -> None:
        try:
            status = json.loads(message.data)
        except (TypeError, ValueError):
            self.get_logger().warning("Ignored malformed mission status")
            return

        state = status.get("state", "UNKNOWN")
        obstacle = status.get("obstacle", "none")
        now = time.monotonic()

        if state != self._last_state:
            self.get_logger().info(
                f"MISSION PHASE | {self._last_state or 'START'} -> {state}"
            )
            self._last_state = state

        if obstacle != self._last_obstacle:
            if obstacle == "none":
                self.get_logger().info("AVOIDANCE     | corridor clear")
            else:
                self.get_logger().warning(
                    f"AVOIDANCE     | obstacle {obstacle}; maneuver active"
                )
            self._last_obstacle = obstacle

        if status.get("failsafe", "none") != "none":
            self.get_logger().error(
                f"SAFETY        | {status['failsafe']}"
            )

        if now - self._last_summary >= 2.0:
            self.get_logger().info(
                "FLIGHT        | "
                f"N {status.get('north_m', 0):>6.1f} m  "
                f"E {status.get('east_m', 0):>6.1f} m  "
                f"ALT {status.get('altitude_m', 0):>5.1f} m  | "
                f"WP {status.get('waypoint', 0)}/{status.get('waypoints_total', 0)}  | "
                f"GEOFENCE {'OK' if status.get('inside_geofence') else 'BREACH'}"
            )
            self._last_summary = now


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DemoHud()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
