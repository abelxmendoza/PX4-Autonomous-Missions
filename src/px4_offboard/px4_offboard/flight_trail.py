"""
flight_trail.py — Leave a visible path behind the drone in Gazebo (+ ROS viz).

Spawns small emissive spheres along the flight path (Gazebo ENU) so you can
see where the vehicle came from and how it threaded obstacles.

Colors:
  cyan  — normal flight
  orange — actively avoiding

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
from collections import deque

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, ColorRGBA
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
    def __init__(self):
        super().__init__("flight_trail")

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

        self._avoiding = False
        self._last_enu: tuple[float, float, float] | None = None
        self._points_enu: list[tuple[float, float, float]] = []
        self._crumb_names: deque[str] = deque()
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

        self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position",
            self._position_cb,
            qos_sub,
        )
        self.create_subscription(Bool, "/px4_offboard/avoiding", self._avoid_cb, 10)
        self.create_subscription(Bool, "/px4_offboard/trail_clear", self._clear_cb, 10)

        self._pub_path = self.create_publisher(Path, "/px4_offboard/flight_path", 10)
        self._pub_marker = self.create_publisher(
            Marker, "/px4_offboard/flight_marker", 10
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

    def _crumb_color(self) -> tuple[float, float, float, float]:
        if self._avoiding:
            return (1.0, 0.45, 0.05, 0.95)  # orange
        return (0.1, 0.95, 1.0, 0.9)  # cyan

    def _spawn_crumb(self, enu: tuple[float, float, float]):
        if self._gz is None:
            return
        self._crumb_id += 1
        name = f"trail_crumb_{self._crumb_id}"
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
            ok, _ = self._gz.request(
                f"/world/{self.world}/create",
                req,
                EntityFactory,
                Boolean,
                500,
            )
            if ok:
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
        m.ns = "flight_trail"
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
        marker.ns = "flight_trail"
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
            clr.action = GzMarker.DELETE_ALL
            try:
                self._gz_marker_pub.publish(clr)
            except Exception:
                pass
        # clear ROS marker
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "flight_trail"
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
        rclpy.shutdown()


if __name__ == "__main__":
    main()
