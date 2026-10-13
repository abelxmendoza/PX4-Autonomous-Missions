"""Precision landing onto an ArUco marker. The vehicle must already be airborne.

This node publishes fmu/in/trajectory_setpoint, so stop offboard_mission first.

  ros2 launch px4_offboard precision_land.launch.py
  ros2 topic pub --once /px4_offboard/precision_land/enable std_msgs/msg/Bool "{data: true}"

Annotated frames are on /image_proc. Marker pose in the camera optical frame is
/target_pose; the same pose in local NED is /target_pose_world.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("px4_offboard")
    return LaunchDescription(
        [
            Node(
                package="px4_offboard",
                executable="precision_land",
                name="precision_land",
                output="screen",
                parameters=[os.path.join(share, "config", "precision_land.yaml")],
            )
        ]
    )
