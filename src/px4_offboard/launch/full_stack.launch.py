"""
Full-stack launch: PX4 SITL + Gazebo Harmonic + Micro XRCE-DDS + offboard_mission.

Optional MAVROS2 for GCS/topic relay.

Usage:
  source /opt/ros/humble/setup.bash
  source install/setup.bash
  ros2 launch px4_offboard full_stack.launch.py
  ros2 launch px4_offboard full_stack.launch.py headless:=true
  ros2 launch px4_offboard full_stack.launch.py trajectory_mode:=course
  ros2 launch px4_offboard full_stack.launch.py use_mavros:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _repo_root() -> str:
    # install/share/px4_offboard → walk up to workspace, else fall back to common path
    share = get_package_share_directory("px4_offboard")
    # share/.../install/px4_offboard/share/px4_offboard
    candidate = os.path.abspath(os.path.join(share, "..", "..", "..", ".."))
    if os.path.isdir(os.path.join(candidate, "worlds")):
        return candidate
    return os.path.expanduser("~/Desktop/px4-autonomous-mission")


def _launch_setup(context, *args, **kwargs):
    px4_dir = os.path.expanduser(
        LaunchConfiguration("px4_dir").perform(context)
    )
    headless = LaunchConfiguration("headless").perform(context).lower() == "true"
    trajectory_mode = LaunchConfiguration("trajectory_mode").perform(context)
    hover_alt = LaunchConfiguration("hover_alt_m").perform(context)
    worlds_dir = os.path.join(_repo_root(), "worlds")
    pkg_share = get_package_share_directory("px4_offboard")
    params_file = os.path.join(pkg_share, "config", "offboard_mission.yaml")

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

    xrce_agent = ExecuteProcess(
        cmd=["MicroXRCEAgent", "udp4", "-p", "8888"],
        output="screen",
        name="micro_xrce_agent",
    )

    mission_node = Node(
        package="px4_offboard",
        executable="offboard_mission",
        name="offboard_mission",
        output="screen",
        parameters=[
            params_file,
            {
                "trajectory_mode": trajectory_mode,
                "hover_alt_m": float(hover_alt),
                "log_dir": _repo_root(),
            },
        ],
    )

    trail_node = Node(
        package="px4_offboard",
        executable="flight_trail",
        name="flight_trail",
        output="screen",
        parameters=[
            {
                "enable": True,
                "min_spacing_m": 0.35,
                "sphere_radius_m": 0.12,
                "max_crumbs": 500,
                "world_name": "obstacle_world",
                "gz_crumbs": True,
                "gz_markers": True,
            }
        ],
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
            os.path.join(pkg_share, "config", "mavros_params.yaml"),
        ],
    )

    # Boot order: PX4/Gazebo → XRCE → fake GCS heartbeat → mission + trail
    # Delays assume a warm PX4 build (cold cmake can take minutes — use scripts/run_full_stack.sh)
    delayed_xrce = TimerAction(period=6.0, actions=[xrce_agent])
    delayed_gcs = TimerAction(
        period=10.0,
        actions=[
            ExecuteProcess(
                cmd=["python3", os.path.join(_repo_root(), "scripts", "gcs_heartbeat.py")],
                output="screen",
                name="gcs_heartbeat",
            )
        ],
    )
    delayed_mission = TimerAction(period=22.0, actions=[mission_node])
    delayed_trail = TimerAction(period=16.0, actions=[trail_node])
    delayed_mavros = TimerAction(period=12.0, actions=[mavros_node])

    return [
        px4_process,
        delayed_xrce,
        delayed_gcs,
        delayed_trail,
        delayed_mission,
        delayed_mavros,
    ]


def generate_launch_description():
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
                "trajectory_mode",
                default_value="waypoints",
                description="waypoints | course | circle",
            ),
            DeclareLaunchArgument(
                "hover_alt_m",
                default_value="5.0",
                description="Mission altitude AGL (metres)",
            ),
            DeclareLaunchArgument(
                "use_mavros",
                default_value="false",
                description="Also start MAVROS2",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
