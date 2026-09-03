"""
ROS 2 launch file — PX4 SITL + Gazebo Harmonic (+ optional MAVROS2).

For the full autonomy stack (XRCE + offboard_mission), prefer:
  ros2 launch px4_offboard full_stack.launch.py

Usage:
  ros2 launch launch/simulation.launch.py
  ros2 launch launch/simulation.launch.py headless:=true
  ros2 launch launch/simulation.launch.py use_mavros:=true
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    px4_dir = os.path.expanduser(LaunchConfiguration("px4_dir").perform(context))
    headless = LaunchConfiguration("headless").perform(context).lower() == "true"
    repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    worlds_dir = os.path.join(repo_root, "worlds")

    headless_env = "HEADLESS=1 " if headless else ""
    px4_cmd = (
        f"cd {px4_dir} && "
        f"rm -f build/px4_sitl_default/dataman && "
        f"PX4_GZ_WORLD=obstacle_world "
        f"GZ_SIM_RESOURCE_PATH={worlds_dir}:${{GZ_SIM_RESOURCE_PATH}} "
        f"{headless_env}"
        f"make px4_sitl gz_x500"
    )

    px4_process = ExecuteProcess(
        cmd=["bash", "-c", px4_cmd],
        output="screen",
        name="px4_sitl",
    )

    mavros_node = Node(
        package="mavros",
        executable="mavros_node",
        name="mavros",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_mavros")),
        parameters=[
            {
                "fcu_url": "udp://:14540@",
                "gcs_url": "udp://@localhost:14550",
                "target_system_id": 1,
                "target_component_id": 1,
                "fcu_protocol": "v2.0",
                "system_id": 255,
                "component_id": 240,
            },
            os.path.join(repo_root, "config", "mavros_params.yaml"),
        ],
    )

    delayed_mavros = TimerAction(period=8.0, actions=[mavros_node])
    return [px4_process, delayed_mavros]


def generate_launch_description():
    repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    worlds_dir = os.path.join(repo_root, "worlds")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "px4_dir",
                default_value=os.path.expanduser("~/PX4-Autopilot"),
                description="Path to PX4-Autopilot checkout",
            ),
            DeclareLaunchArgument(
                "headless",
                default_value="false",
                description="Run Gazebo without GUI",
            ),
            DeclareLaunchArgument(
                "use_mavros",
                default_value="true",
                description="Start MAVROS2 (default true for this launch)",
            ),
            SetEnvironmentVariable(name="GZ_SIM_RESOURCE_PATH", value=worlds_dir),
            OpaqueFunction(function=_launch_setup),
        ]
    )
