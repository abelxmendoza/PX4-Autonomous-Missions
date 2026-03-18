"""
ROS 2 launch file — PX4 SITL + Gazebo Harmonic + MAVROS2

Starts:
  1. PX4 SITL with Gazebo Harmonic using the obstacle_world
  2. MAVROS2 node (connects to PX4 via MAVLink UDP)

QGroundControl connects automatically to UDP 14550 (PX4 broadcasts GCS link).

Prerequisites:
  sudo apt install ros-humble-mavros ros-humble-mavros-extras
  wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
  sudo bash install_geographiclib_datasets.sh

  Copy worlds/obstacle_world.sdf to:
    ~/PX4-Autopilot/Tools/simulation/gz/worlds/

Usage:
  ros2 launch launch/simulation.launch.py
  ros2 launch launch/simulation.launch.py headless:=true
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    TimerAction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    px4_dir = os.path.expanduser("~/PX4-Autopilot")
    worlds_dir = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "worlds",
    )

    headless_arg = DeclareLaunchArgument(
        "headless",
        default_value="false",
        description="Run Gazebo without GUI (headless mode)",
    )

    vehicle_arg = DeclareLaunchArgument(
        "vehicle",
        default_value="gz_x500",
        description="PX4 SITL vehicle model (gz_x500, gz_iris, gz_rc_cessna…)",
    )

    # ── 1. Export GZ_SIM_RESOURCE_PATH so Gazebo finds our custom world ──
    set_gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=worlds_dir,
    )

    # ── 2. PX4 SITL + Gazebo Harmonic ────────────────────────────────────
    #   PX4_GZ_WORLD=obstacle_world selects our SDF world.
    #   HEADLESS=1 disables the Gazebo GUI when headless:=true.
    px4_sitl_cmd = (
        f"cd {px4_dir} && "
        "PX4_GZ_WORLD=obstacle_world "
        "$([ '$(var headless)' = 'true' ] && echo 'HEADLESS=1' || echo '') "
        "make px4_sitl $(var vehicle)"
    )

    # Simpler cross-compatible version using env vars
    px4_process = ExecuteProcess(
        cmd=[
            "bash",
            "-c",
            (
                f"cd {px4_dir} && "
                "PX4_GZ_WORLD=obstacle_world "
                f"GZ_SIM_RESOURCE_PATH={worlds_dir}:$GZ_SIM_RESOURCE_PATH "
                "make px4_sitl gz_x500"
            ),
        ],
        output="screen",
        name="px4_sitl",
    )

    # ── 3. MAVROS2 node (delayed 8s to let PX4 fully boot) ───────────────
    #   fcu_url:  PX4 SITL MAVLink offboard port (14540)
    #   gcs_url:  forward telemetry to QGroundControl on localhost:14550
    mavros_node = Node(
        package="mavros",
        executable="mavros_node",
        name="mavros",
        output="screen",
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
            os.path.join(
                os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                "config",
                "mavros_params.yaml",
            ),
        ],
    )

    delayed_mavros = TimerAction(period=8.0, actions=[mavros_node])

    return LaunchDescription(
        [
            headless_arg,
            vehicle_arg,
            set_gz_resource_path,
            px4_process,
            delayed_mavros,
        ]
    )
