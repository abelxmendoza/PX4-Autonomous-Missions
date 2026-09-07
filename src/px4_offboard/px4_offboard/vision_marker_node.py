"""Detect an ArUco fiducial in the camera_bridge image stream and publish
bearing/elevation/range to it.

Consumes /px4_offboard/camera/image_raw (published by camera_bridge.py) —
no direct Gazebo/gz-transport dependency, unlike the other bridges, since
the image is already real ROS data by the time this node sees it. Detection
and geometry are delegated to the unit-tested vision_marker_detect.py and
vision_marker.py.
"""

from __future__ import annotations

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

from .vision_marker import PinholeCamera, bearing_elevation_deg, range_m
from .vision_marker_detect import detect_largest_marker


class VisionMarkerNode(Node):
    def __init__(self):
        super().__init__("vision_marker_node")
        self.declare_parameter("horizontal_fov_rad", 1.74)  # matches mono_cam SDF
        self.declare_parameter("marker_size_m", 0.3)
        self.declare_parameter("aruco_dictionary", "DICT_4X4_50")
        self.declare_parameter("min_marker_side_px", 8.0)

        self.horizontal_fov_rad = float(self.get_parameter("horizontal_fov_rad").value)
        self.marker_size_m = float(self.get_parameter("marker_size_m").value)
        self.aruco_dictionary = str(self.get_parameter("aruco_dictionary").value)
        self.min_marker_side_px = float(self.get_parameter("min_marker_side_px").value)

        self._visible_pub = self.create_publisher(Bool, "/px4_offboard/vision_marker/visible", 10)
        self._bearing_pub = self.create_publisher(
            PointStamped, "/px4_offboard/vision_marker/bearing", 10
        )
        self.create_subscription(Image, "/px4_offboard/camera/image_raw", self._image_cb, 10)

    def _image_cb(self, msg: Image):
        if msg.encoding != "rgb8":
            self.get_logger().warn(f"unsupported image encoding {msg.encoding!r}", once=True)
            return
        frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)

        detection = detect_largest_marker(gray, self.aruco_dictionary)
        if detection is None:
            self._visible_pub.publish(Bool(data=False))
            return

        self._visible_pub.publish(Bool(data=True))
        if detection.side_px < self.min_marker_side_px:
            return

        camera = PinholeCamera(
            width_px=msg.width, height_px=msg.height, horizontal_fov_rad=self.horizontal_fov_rad
        )
        bearing_deg, elevation_deg = bearing_elevation_deg(
            camera, detection.center_x_px, detection.center_y_px
        )
        distance_m = range_m(camera, self.marker_size_m, detection.side_px)

        out = PointStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = "camera_link"
        out.point.x = bearing_deg
        out.point.y = elevation_deg
        out.point.z = distance_m
        self._bearing_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = VisionMarkerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
