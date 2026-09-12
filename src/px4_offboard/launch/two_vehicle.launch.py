"""Two independent hover missions connected to PX4 SITL instances 1 and 2.

Start Gazebo/PX4 as documented in docs/multi_vehicle.md first. This launch
owns the DDS agent and controllers, not the simulator. Each vehicle holds
its own local origin; this is an isolation smoke test, not swarm planning.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _controllers(context):
    log_dir = os.path.abspath(os.path.expanduser(
        LaunchConfiguration("log_dir").perform(context)
    ))
    nodes = []
    for instance in (1, 2):
        namespace = f"px4_{instance}"
        nodes.append(Node(
            package="px4_offboard",
            executable="offboard_mission",
            namespace=namespace,
            name="offboard_mission",
            output="screen",
            parameters=[{
                # PX4's rcS sets MAV_SYS_ID = instance + 1.
                "target_system_id": instance + 1,
                "local_frame_id": f"{namespace}/local_enu",
                "log_dir": os.path.join(log_dir, namespace),
                # A zero-radius orbit reuses the existing timed trajectory
                # and complete state sequence, holding the local origin.
                "trajectory_mode": "circle",
                "circle_radius_m": 0.0,
                "circle_period_s": 10.0,
                "max_orbits": 1.0,
                "hover_alt_m": 3.0,
                "hover_hold_s": 3.0,
                "mission_timeout_s": 90.0,
                "obstacle_source": "map_only",
                "executive_enable": False,
                "gps_denied_enable": False,
                "gps_px4_failure_inject": False,
            }],
        ))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("log_dir", default_value="demo_artifacts/two_vehicle"),
        DeclareLaunchArgument("start_agent", default_value="true"),
        ExecuteProcess(
            cmd=["MicroXRCEAgent", "udp4", "-p", "8888"],
            output="screen",
            condition=IfCondition(LaunchConfiguration("start_agent")),
        ),
        OpaqueFunction(function=_controllers),
    ])
