"""
Full-stack launch: PX4 SITL + Gazebo Harmonic + Micro XRCE-DDS + offboard_mission.

Includes LiDAR sector sensing (gz GPU lidar → /px4_offboard/obstacle_dir),
flight trail crumbs, optional MAVROS2 / demo HUD.

Usage:
  source /opt/ros/humble/setup.bash
  source install/setup.bash
  ros2 launch px4_offboard full_stack.launch.py
  ros2 launch px4_offboard full_stack.launch.py headless:=true
  ros2 launch px4_offboard full_stack.launch.py vehicle:=gz_x500_lidar_2d
  ros2 launch px4_offboard full_stack.launch.py use_lidar:=false
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
    share = get_package_share_directory("px4_offboard")
    candidate = os.path.abspath(os.path.join(share, "..", "..", "..", ".."))
    if os.path.isdir(os.path.join(candidate, "worlds")):
        return candidate
    return os.path.expanduser("~/Desktop/px4-autonomous-mission")


def _launch_setup(context, *args, **kwargs):
    px4_dir = os.path.expanduser(LaunchConfiguration("px4_dir").perform(context))
    headless = LaunchConfiguration("headless").perform(context).lower() == "true"
    trajectory_mode = LaunchConfiguration("trajectory_mode").perform(context)
    hover_alt = LaunchConfiguration("hover_alt_m").perform(context)
    avoidance_strategy = LaunchConfiguration("avoidance_strategy").perform(context)
    sidestep_m = LaunchConfiguration("sidestep_m").perform(context)
    detection_margin = LaunchConfiguration("detection_margin_m").perform(context)
    vehicle = LaunchConfiguration("vehicle").perform(context)
    use_lidar = LaunchConfiguration("use_lidar").perform(context).lower() == "true"
    lidar_trigger = LaunchConfiguration("lidar_trigger_m").perform(context)

    root = _repo_root()
    worlds_dir = os.path.join(root, "worlds")
    models_dir = os.path.join(root, "models")
    pkg_share = get_package_share_directory("px4_offboard")
    params_file = os.path.join(pkg_share, "config", "offboard_mission.yaml")

    # Prefer local model overrides (always-on lidar) then worlds, then PX4 defaults
    gz_path = f"{models_dir}:{worlds_dir}:${{GZ_SIM_RESOURCE_PATH}}"
    headless_env = "HEADLESS=1 " if headless else ""
    px4_cmd = (
        f"cd {px4_dir} && "
        f"rm -f build/px4_sitl_default/dataman && "
        f"PX4_GZ_WORLD=obstacle_world "
        f"GZ_SIM_RESOURCE_PATH={gz_path} "
        f"{headless_env}"
        f"make px4_sitl {vehicle}"
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
                "avoidance_strategy": avoidance_strategy,
                "sidestep_m": float(sidestep_m),
                "detection_margin_m": float(detection_margin),
                "sensor_timeout_s": 0.75 if use_lidar else 0.5,
                "log_dir": root,
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

    lidar_node = Node(
        package="px4_offboard",
        executable="lidar_sectors",
        name="lidar_sectors",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_lidar")),
        parameters=[
            {
                "gz_topic": (
                    "/world/obstacle_world/model/x500_lidar_2d_0/link/link/"
                    "sensor/lidar_2d_v2/scan"
                ),
                "trigger_m": float(lidar_trigger),
                "front_angle_deg": 35.0,
                "side_angle_deg": 90.0,
                "publish_hz": 20.0,
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

    demo_hud_node = Node(
        package="px4_offboard",
        executable="demo_hud",
        name="demo_hud",
        output="screen",
        condition=IfCondition(LaunchConfiguration("demo_mode")),
    )

    delayed_xrce = TimerAction(period=6.0, actions=[xrce_agent])
    delayed_gcs = TimerAction(
        period=10.0,
        actions=[
            ExecuteProcess(
                cmd=["python3", os.path.join(root, "scripts", "gcs_heartbeat.py")],
                output="screen",
                name="gcs_heartbeat",
            )
        ],
    )
    delayed_lidar = TimerAction(period=14.0, actions=[lidar_node])
    delayed_trail = TimerAction(period=16.0, actions=[trail_node])
    delayed_mission = TimerAction(period=24.0, actions=[mission_node])
    delayed_mavros = TimerAction(period=12.0, actions=[mavros_node])

    return [
        px4_process,
        delayed_xrce,
        delayed_gcs,
        delayed_lidar,
        delayed_trail,
        delayed_mission,
        demo_hud_node,
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
                "vehicle",
                default_value="gz_x500_lidar_2d",
                description="PX4 SITL target (gz_x500_lidar_2d recommended)",
            ),
            DeclareLaunchArgument(
                "use_lidar",
                default_value="true",
                description="Start lidar_sectors → /px4_offboard/obstacle_dir",
            ),
            DeclareLaunchArgument(
                "lidar_trigger_m",
                default_value="4.0",
                description="LiDAR sector trigger distance (metres)",
            ),
            DeclareLaunchArgument(
                "trajectory_mode",
                default_value="waypoints",
                description="waypoints | course | circle",
            ),
            DeclareLaunchArgument(
                "hover_alt_m",
                default_value="3.5",
                description="Mission altitude AGL (metres)",
            ),
            DeclareLaunchArgument(
                "detection_margin_m",
                default_value="2.5",
                description="AABB fallback detection margin (metres)",
            ),
            DeclareLaunchArgument(
                "avoidance_strategy",
                default_value="sidestep",
                description="climb | sidestep",
            ),
            DeclareLaunchArgument(
                "sidestep_m",
                default_value="2.0",
                description="Lateral avoidance offset in metres",
            ),
            DeclareLaunchArgument(
                "demo_mode",
                default_value="false",
                description="Show concise recruiter-friendly mission telemetry",
            ),
            DeclareLaunchArgument(
                "use_mavros",
                default_value="false",
                description="Also start MAVROS2",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
