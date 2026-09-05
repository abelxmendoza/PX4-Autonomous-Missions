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
        self._last_exec_mode = None
        self._last_loc_source = None
        self._last_in_denied = None
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
        exec_mode = status.get("executive_mode", "NOMINAL")
        now = time.monotonic()

        if state != self._last_state:
            self.get_logger().info(
                f"MISSION PHASE | {self._last_state or 'START'} -> {state}"
            )
            self._last_state = state

        if exec_mode != self._last_exec_mode:
            reason = status.get("executive_reason", "")
            level = self.get_logger().info
            if exec_mode in ("SAFE", "ABORT"):
                level = self.get_logger().error
            elif exec_mode == "DEGRADED":
                level = self.get_logger().warning
            level(f"EXECUTIVE    | {exec_mode}" + (f" — {reason}" if reason else ""))
            self._last_exec_mode = exec_mode

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

        in_denied = bool(status.get("in_gps_denied_zone", False))
        loc_source = status.get("loc_source", "UNKNOWN")
        loc_event = status.get("loc_event") or ""
        if in_denied != self._last_in_denied or loc_source != self._last_loc_source:
            zone = "IN DENIED ZONE" if in_denied else "GPS zone clear"
            inject = " inject" if status.get("gps_injected_deny") else ""
            event = f" event={loc_event}" if loc_event else ""
            level = self.get_logger().warning if in_denied else self.get_logger().info
            level(f"LOCALIZATION  | {zone} | source={loc_source}{inject}{event}")
            self._last_in_denied = in_denied
            self._last_loc_source = loc_source

        if now - self._last_summary >= 2.0:
            skipped = status.get("skipped_waypoints") or []
            skip_txt = f"  skip={skipped}" if skipped else ""
            loc_txt = (
                f"  LOC {status.get('loc_source', '?')}"
                f"{'/ZONE' if status.get('in_gps_denied_zone') else ''}"
            )
            self.get_logger().info(
                "FLIGHT        | "
                f"N {status.get('north_m', 0):>6.1f} m  "
                f"E {status.get('east_m', 0):>6.1f} m  "
                f"ALT {status.get('altitude_m', 0):>5.1f} m  | "
                f"WP {status.get('waypoint', 0)}/{status.get('waypoints_total', 0)}  | "
                f"GEOFENCE {'OK' if status.get('inside_geofence') else 'BREACH'}  | "
                f"BATT {float(status.get('battery_frac', 1.0)):.0%}  "
                f"LINK {float(status.get('link_quality', 1.0)):.0%}  "
                f"PROP {float(status.get('propellant_time_s', 0)):.0f}s"
                f"{loc_txt}{skip_txt}"
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
