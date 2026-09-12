"""
flight_trail.py — Leave a visible path behind the drone in Gazebo (+ ROS viz).

Spawns small emissive spheres along the flight path (Gazebo ENU) so you can
see where the vehicle came from and how it threaded obstacles.

Colors:
  cyan  — actual flight trail
  orange — actively avoiding
  green — A* preflight route
  gold  — planned turn / goal markers

Also publishes:
  /px4_offboard/flight_path   (nav_msgs/Path)
  /px4_offboard/flight_marker (visualization_msgs/Marker LINE_STRIP)

Run:
  ros2 run px4_offboard flight_trail
"""

from __future__ import annotations

import os

# Gazebo protobuf bindings need the pure-Python impl with newer pip protobuf
os.environ.setdefault("PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION", "python")

import math
import json
import time
import threading
from collections import deque

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, ColorRGBA, String
from visualization_msgs.msg import Marker

from px4_msgs.msg import VehicleLocalPosition

try:
    from gz.msgs10.boolean_pb2 import Boolean
    from gz.msgs10.entity_pb2 import Entity
    from gz.msgs10.entity_factory_pb2 import EntityFactory
    from gz.msgs10.marker_pb2 import Marker as GzMarker
    from gz.transport13 import Node as GzNode

    _GZ_OK = True
except Exception as exc:  # noqa: BLE001
    _GZ_OK = False
    _GZ_IMPORT_ERROR = exc


class FlightTrail(Node):
    def __init__(self, **kwargs):
        super().__init__("flight_trail", **kwargs)

        self.declare_parameter("input_mode", "px4")
        self.declare_parameter("trail_color", [0.1, 0.95, 1.0, 0.9])
        self.declare_parameter("route_color", [0.15, 1.0, 0.35, 0.85])
        self.declare_parameter("enable", True)
        self.declare_parameter("min_spacing_m", 0.4)
        self.declare_parameter("sphere_radius_m", 0.12)
        self.declare_parameter("max_crumbs", 400)
        self.declare_parameter("world_name", "obstacle_world")
        self.declare_parameter("gz_markers", True)
        self.declare_parameter("gz_crumbs", True)
        self.declare_parameter("line_width_m", 0.08)

        self.enable = bool(self.get_parameter("enable").value)
        self.min_spacing = float(self.get_parameter("min_spacing_m").value)
        self.sphere_r = float(self.get_parameter("sphere_radius_m").value)
        self.max_crumbs = int(self.get_parameter("max_crumbs").value)
        self.world = str(self.get_parameter("world_name").value)
        self.use_gz_markers = bool(self.get_parameter("gz_markers").value)
        self.use_gz_crumbs = bool(self.get_parameter("gz_crumbs").value)
        self.line_width = float(self.get_parameter("line_width_m").value)

        self.visual_prefix = self.get_namespace().strip("/").replace("/", "_") or "single"
        self.trail_color = tuple(self.get_parameter("trail_color").value)
        self.route_color = tuple(self.get_parameter("route_color").value)
        self._telemetry_session = None
        self._avoiding = False
        self._last_enu: tuple[float, float, float] | None = None
        self._points_enu: list[tuple[float, float, float]] = []
        self._planned_enu: list[tuple[float, float, float]] = []
        self._crumb_names: deque[str] = deque()
        self._planned_model_names: list[str] = []
        self._planned_lock = threading.Lock()
        self._crumb_id = 0

        self._gz = None
        self._gz_marker_pub = None
        if _GZ_OK and (self.use_gz_crumbs or self.use_gz_markers):
            try:
                self._gz = GzNode()
                if self.use_gz_markers:
                    self._gz_marker_pub = self._gz.advertise("/marker", GzMarker)
                self.get_logger().info("Gazebo transport ready for trail visuals")
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f"Gazebo transport unavailable: {exc}")
                self._gz = None
        elif not _GZ_OK:
            self.get_logger().warn(f"gz python bindings unavailable: {_GZ_IMPORT_ERROR}")

        qos_sub = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        if self.get_parameter("input_mode").value == "swarm":
            self.create_subscription(String, "swarm/telemetry", self._telemetry_cb, 1)
        else:
            self.create_subscription(VehicleLocalPosition, "fmu/out/vehicle_local_position",
                                     self._position_cb, qos_sub)
        self.create_subscription(Bool, "px4_offboard/avoiding", self._avoid_cb, 10)
        self.create_subscription(Bool, "px4_offboard/trail_clear", self._clear_cb, 10)
        self.create_subscription(
            Path, "px4_offboard/planned_path", self._planned_path_cb, qos_sub
        )

        self._pub_path = self.create_publisher(Path, "px4_offboard/flight_path", 10)
        self._pub_marker = self.create_publisher(
            Marker, "px4_offboard/flight_marker", 10
        )

        self.get_logger().info(
            f"FlightTrail ready — crumbs={self.use_gz_crumbs} "
            f"markers={self.use_gz_markers} spacing={self.min_spacing}m"
        )

    def _avoid_cb(self, msg: Bool):
        self._avoiding = bool(msg.data)

    def _clear_cb(self, msg: Bool):
        if msg.data:
            self._clear_trail()

    def _position_cb(self, msg: VehicleLocalPosition):
        if not self.enable:
            return
        # PX4 NED → Gazebo / ROS ENU
        east = float(msg.y)
        north = float(msg.x)
        up = float(-msg.z)
        enu = (east, north, up)

        self._record_enu(enu)

    def _telemetry_cb(self, msg):
        try:
            from .swarm_logic import Telemetry
            sample = Telemetry.parse(json.loads(msg.data))
            if not sample.valid or not 0 <= time.monotonic() - sample.sent <= 0.75:
                return
            if sample.vehicle != self.visual_prefix:
                return
            if self._telemetry_session not in (None, sample.session):
                self._clear_trail()
            self._telemetry_session = sample.session
            north, east, down = sample.position
            self._record_enu((east, north, -down))
        except (ValueError, TypeError, KeyError):
            return

    def _record_enu(self, enu):
        if not self.enable:
            return
        if self._last_enu is not None:
            dn = enu[1] - self._last_enu[1]
            de = enu[0] - self._last_enu[0]
            du = enu[2] - self._last_enu[2]
            if math.sqrt(dn * dn + de * de + du * du) < self.min_spacing:
                return

        self._last_enu = enu
        self._points_enu.append(enu)
        if len(self._points_enu) > self.max_crumbs * 2:
            self._points_enu = self._points_enu[-self.max_crumbs :]

        if self.use_gz_crumbs:
            self._spawn_crumb(enu)
        if self.use_gz_markers:
            self._publish_gz_line()
        self._publish_ros_viz()

    def _planned_path_cb(self, msg: Path):
        planned = [
            (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)
            for pose in msg.poses
        ]
        if planned == self._planned_enu:
            return
        self._planned_enu = planned
        self.get_logger().info(
            f"PLANNED ROUTE | {max(0, len(planned) - 1)} legs for {self.visual_prefix}"
        )
        self._publish_planned_ros()
        # GZ model spawn uses blocking service calls; keep it off the ROS
        # executor or crumbs/setpoints stall and the route never appears.
        threading.Thread(target=self._render_planned_gz, daemon=True).start()

    def _publish_planned_gz(self):
        if self._gz_marker_pub is None or len(self._planned_enu) < 2:
            return
        line = GzMarker()
        line.action = GzMarker.ADD_MODIFY
        line.type = GzMarker.LINE_STRIP
        line.id = 2
        line.ns = f"{self.visual_prefix}/planned_route"
        line.scale.x = self.line_width * 2.0
        line.material.ambient.r = 0.15
        line.material.ambient.g = 1.0
        line.material.ambient.b = 0.35
        line.material.ambient.a = 0.85
        line.material.diffuse.r = 0.15
        line.material.diffuse.g = 1.0
        line.material.diffuse.b = 0.35
        line.material.diffuse.a = 0.85
        if hasattr(line, "visibility"):
            line.visibility = 0xFFFFFFFF
        for east, north, up in self._planned_enu:
            point = line.point.add()
            point.x, point.y, point.z = east, north, up
        try:
            self._gz_marker_pub.publish(line)
            for index, (east, north, up) in enumerate(self._planned_enu[1:], start=1):
                marker = GzMarker()
                marker.action = GzMarker.ADD_MODIFY
                marker.type = GzMarker.SPHERE
                marker.id = 100 + index
                marker.ns = f"{self.visual_prefix}/planned_waypoints"
                marker.pose.position.x = east
                marker.pose.position.y = north
                marker.pose.position.z = up
                marker.scale.x = marker.scale.y = marker.scale.z = 0.42
                marker.material.ambient.r = 1.0
                marker.material.ambient.g = 0.7
                marker.material.ambient.b = 0.05
                marker.material.ambient.a = 0.95
                marker.material.diffuse.r = 1.0
                marker.material.diffuse.g = 0.7
                marker.material.diffuse.b = 0.05
                marker.material.diffuse.a = 0.95
                if hasattr(marker, "visibility"):
                    marker.visibility = 0xFFFFFFFF
                self._gz_marker_pub.publish(marker)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(
                f"planned-route marker failed: {exc}", throttle_duration_sec=2.0
            )

    def _render_planned_gz(self):
        with self._planned_lock:
            self._spawn_planned_models()

    def _spawn_planned_models(self):
        """Spawn the A* route as static Gazebo models (markers are easy to miss)."""
        if self._gz is None:
            return
        self._clear_planned_models()
        if len(self._planned_enu) < 2:
            return
        r, g, b, a = self.route_color
        for index, (east, north, up) in enumerate(self._planned_enu):
            name = f"{self.visual_prefix}_planned_wp_{index}"
            sdf = f"""<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <visual name="v">
        <geometry><sphere><radius>0.38</radius></sphere></geometry>
        <material>
          <ambient>1.0 0.72 0.05 1</ambient>
          <diffuse>1.0 0.72 0.05 1</diffuse>
          <emissive>0.55 0.32 0.0 1</emissive>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""
            if self._spawn_static(name, sdf, east, north, up):
                self._planned_model_names.append(name)
        for index, ((e1, n1, u1), (e2, n2, u2)) in enumerate(
            zip(self._planned_enu, self._planned_enu[1:])
        ):
            length = math.sqrt((e2 - e1) ** 2 + (n2 - n1) ** 2 + (u2 - u1) ** 2)
            if length < 0.25:
                continue
            name = f"{self.visual_prefix}_planned_leg_{index}"
            sdf = f"""<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <visual name="v">
        <geometry><box><size>{length:.3f} 0.16 0.16</size></box></geometry>
        <material>
          <ambient>{r} {g} {b} {a}</ambient>
          <diffuse>{r} {g} {b} {a}</diffuse>
          <emissive>{r*0.4} {g*0.4} {b*0.4} 1</emissive>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""
            yaw = math.atan2(n2 - n1, e2 - e1)
            pitch = -math.atan2(u2 - u1, math.hypot(e2 - e1, n2 - n1))
            if self._spawn_static(
                name, sdf, (e1 + e2) / 2.0, (n1 + n2) / 2.0, (u1 + u2) / 2.0, yaw, pitch
            ):
                self._planned_model_names.append(name)
        self.get_logger().info(
            f"PLANNED ROUTE | spawned {len(self._planned_model_names)} Gazebo models"
        )

    def _spawn_static(
        self,
        name: str,
        sdf: str,
        x: float,
        y: float,
        z: float,
        yaw: float = 0.0,
        pitch: float = 0.0,
    ) -> bool:
        req = EntityFactory()
        req.name = name
        req.allow_renaming = True
        req.sdf = sdf
        req.pose.position.x = float(x)
        req.pose.position.y = float(y)
        req.pose.position.z = float(z)
        req.pose.orientation.x = -math.sin(yaw / 2.0) * math.sin(pitch / 2.0)
        req.pose.orientation.y = math.cos(yaw / 2.0) * math.sin(pitch / 2.0)
        req.pose.orientation.z = math.sin(yaw / 2.0) * math.cos(pitch / 2.0)
        req.pose.orientation.w = math.cos(yaw / 2.0) * math.cos(pitch / 2.0)
        try:
            ok, response = self._gz.request(
                f"/world/{self.world}/create",
                req,
                EntityFactory,
                Boolean,
                800,
            )
            return bool(ok and response.data)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(
                f"planned model spawn failed ({name}): {exc}",
                throttle_duration_sec=2.0,
            )
            return False

    def _clear_planned_models(self):
        while self._planned_model_names:
            self._remove_crumb(self._planned_model_names.pop())

    def _publish_planned_ros(self):
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = "map"
        marker.ns = f"{self.visual_prefix}/planned_route"
        marker.id = 2
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD if len(self._planned_enu) >= 2 else Marker.DELETE
        marker.scale.x = self.line_width * 2.0
        marker.color = ColorRGBA(**dict(zip(("r", "g", "b", "a"), self.route_color)))
        marker.pose.orientation.w = 1.0
        from geometry_msgs.msg import Point

        for east, north, up in self._planned_enu:
            point = Point()
            point.x, point.y, point.z = east, north, up
            marker.points.append(point)
        self._pub_marker.publish(marker)

    def _crumb_color(self) -> tuple[float, float, float, float]:
        if self._avoiding:
            return (1.0, 0.45, 0.05, 0.95)  # orange
        return self.trail_color

    def _spawn_crumb(self, enu: tuple[float, float, float]):
        if self._gz is None:
            return
        self._crumb_id += 1
        name = f"{self.visual_prefix}_trail_crumb_{self._crumb_id}"
        r, g, b, a = self._crumb_color()
        rad = self.sphere_r * (1.35 if self._avoiding else 1.0)

        req = EntityFactory()
        req.name = name
        req.allow_renaming = True
        req.sdf = f"""<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <visual name="v">
        <geometry><sphere><radius>{rad:.3f}</radius></sphere></geometry>
        <material>
          <ambient>{r:.3f} {g:.3f} {b:.3f} {a:.3f}</ambient>
          <diffuse>{r:.3f} {g:.3f} {b:.3f} {a:.3f}</diffuse>
          <emissive>{r*0.6:.3f} {g*0.6:.3f} {b*0.6:.3f} 1</emissive>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""
        req.pose.position.x = enu[0]
        req.pose.position.y = enu[1]
        req.pose.position.z = enu[2]

        try:
            ok, response = self._gz.request(
                f"/world/{self.world}/create",
                req,
                EntityFactory,
                Boolean,
                500,
            )
            if ok and response.data:
                self._crumb_names.append(name)
                while len(self._crumb_names) > self.max_crumbs:
                    self._remove_crumb(self._crumb_names.popleft())
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"crumb spawn failed: {exc}", throttle_duration_sec=2.0)

    def _remove_crumb(self, name: str):
        if self._gz is None:
            return
        ent = Entity()
        ent.name = name
        ent.type = Entity.MODEL
        try:
            self._gz.request(
                f"/world/{self.world}/remove",
                ent,
                Entity,
                Boolean,
                300,
            )
        except Exception:
            pass

    def _publish_gz_line(self):
        if self._gz_marker_pub is None or len(self._points_enu) < 2:
            return
        r, g, b, a = self._crumb_color()
        m = GzMarker()
        m.action = GzMarker.ADD_MODIFY
        m.type = GzMarker.LINE_STRIP
        m.id = 1
        m.ns = f"{self.visual_prefix}/flight_trail"
        m.scale.x = self.line_width
        m.material.ambient.r = r
        m.material.ambient.g = g
        m.material.ambient.b = b
        m.material.ambient.a = a
        m.material.diffuse.r = r
        m.material.diffuse.g = g
        m.material.diffuse.b = b
        m.material.diffuse.a = a
        del m.point[:]
        for e, n, u in self._points_enu[-self.max_crumbs :]:
            pt = m.point.add()
            pt.x = e
            pt.y = n
            pt.z = u
        try:
            self._gz_marker_pub.publish(m)
        except Exception:
            pass

    def _publish_ros_viz(self):
        now = self.get_clock().now().to_msg()
        path = Path()
        path.header.stamp = now
        path.header.frame_id = "map"
        for e, n, u in self._points_enu[-self.max_crumbs :]:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = e
            ps.pose.position.y = n
            ps.pose.position.z = u
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self._pub_path.publish(path)

        r, g, b, a = self._crumb_color()
        marker = Marker()
        marker.header = path.header
        marker.ns = f"{self.visual_prefix}/flight_trail"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = self.line_width
        marker.color = ColorRGBA(r=r, g=g, b=b, a=a)
        marker.pose.orientation.w = 1.0
        for e, n, u in self._points_enu[-self.max_crumbs :]:
            from geometry_msgs.msg import Point

            p = Point()
            p.x = e
            p.y = n
            p.z = u
            marker.points.append(p)
        self._pub_marker.publish(marker)

    def _clear_trail(self):
        self.get_logger().info("Clearing flight trail")
        self._points_enu.clear()
        self._last_enu = None
        while self._crumb_names:
            self._remove_crumb(self._crumb_names.popleft())
        if self._gz_marker_pub is not None:
            clr = GzMarker()
            clr.action = GzMarker.DELETE_MARKER
            clr.ns = f"{self.visual_prefix}/flight_trail"
            clr.id = 1
            try:
                self._gz_marker_pub.publish(clr)
            except Exception:
                pass
        # clear ROS marker
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = f"{self.visual_prefix}/flight_trail"
        marker.id = 0
        marker.action = Marker.DELETE
        self._pub_marker.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = FlightTrail()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
