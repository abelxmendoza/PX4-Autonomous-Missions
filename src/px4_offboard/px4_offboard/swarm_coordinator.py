"""ROS adapter and synchronized evidence log for SurveyCoordinator."""
from dataclasses import asdict
import json
from pathlib import Path
import time
import uuid

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Path as RosPath
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, DurabilityPolicy

from .swarm_logic import HOMES, SurveyCoordinator, Telemetry


class SwarmCoordinator(Node):
    def __init__(self, **kwargs):
        super().__init__("swarm_coordinator", **kwargs)
        self.declare_parameter("log_dir", "demo_artifacts/swarm")
        self.logic = SurveyCoordinator()
        self.session = uuid.uuid4().hex
        self.seq = 0
        self.started = time.monotonic()
        self.last_phase = None
        log_dir = Path(str(self.get_parameter("log_dir").value)).expanduser()
        log_dir.mkdir(parents=True, exist_ok=True)
        self.log_path = log_dir / f"swarm_{self.session}.jsonl"
        self.log = self.log_path.open("x")
        self.log.write(json.dumps({"type": "header", "schema": 1, "session": self.session,
                                   "homes_ned": HOMES, "minimum_separation_m": self.logic.minimum_separation,
                                   "expected_tasks": [t.name for t in self.logic.tasks]}) + "\n")
        self.publishers_by_vehicle = {}
        self.route_publishers = {}
        self.route_signatures = {}
        route_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for v in HOMES:
            self.route_publishers[v] = self.create_publisher(RosPath, f"/{v}/px4_offboard/planned_path", route_qos)
            self.publishers_by_vehicle[v] = self.create_publisher(String, f"/{v}/swarm/command", 1)
            self.create_subscription(String, f"/{v}/swarm/telemetry",
                                     lambda msg, vehicle=v: self._telemetry(vehicle, msg), 1)
        self.status = self.create_publisher(String, "/swarm/status", 1)
        self.timer = self.create_timer(0.1, self._tick)
        self.get_logger().info(f"Swarm evidence: {self.log_path}")

    def _telemetry(self, vehicle, msg):
        try:
            sample = Telemetry.parse(json.loads(msg.data))
            if sample.vehicle == vehicle:
                self.logic.update(sample, time.monotonic())
        except (ValueError, KeyError, TypeError, OverflowError):
            pass  # Malformed packets cannot renew telemetry freshness.

    def _tick(self):
        now = time.monotonic()
        if self.logic.phase == "WAITING" and now - self.started > 45.0:
            self.logic._abort("startup telemetry timeout")
        commands = self.logic.step(now)
        self.seq += 1
        for vehicle, command in commands.items():
            sample = self.logic.telemetry.get(vehicle)
            if sample is None:
                continue
            envelope = dict(command, vehicle=vehicle, vehicle_session=sample.session,
                            session=self.session, seq=self.seq, sent=now)
            self.publishers_by_vehicle[vehicle].publish(String(data=json.dumps(envelope)))
        self._publish_routes()
        report = self.logic.report()
        self.status.publish(String(data=json.dumps(report)))
        if report["phase"] != self.last_phase:
            self.get_logger().info(json.dumps(report))
            self.last_phase = report["phase"]
        try:
            self.log.write(json.dumps({"type": "sample", "time": now, **report,
                                       "vehicles": {v: asdict(t) for v, t in self.logic.telemetry.items()},
                                       "commands": commands}, allow_nan=False) + "\n")
            self.log.flush()
        except OSError:
            self.logic._abort("evidence log write failed")

    def _publish_routes(self):
        for vehicle, route in self.logic.routes.items():
            signature = tuple(route)
            if self.logic.phase in {"ABORTED", "COMPLETE"}:
                signature = ()
            sample = self.logic.telemetry.get(vehicle)
            if sample is None or self.route_signatures.get(vehicle) == signature:
                continue
            self.route_signatures[vehicle] = signature
            path = RosPath()
            path.header.frame_id = "map"
            path.header.stamp = self.get_clock().now().to_msg()
            for north, east, down in ([sample.position, *signature] if signature else []):
                pose = PoseStamped()
                pose.header = path.header
                pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = east, north, -down
                pose.pose.orientation.w = 1.0
                path.poses.append(pose)
            self.route_publishers[vehicle].publish(path)

    def destroy_node(self):
        self.log.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SwarmCoordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
