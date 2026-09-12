"""Cooperative survey for the fixed two-vehicle obstacle_world SITL scenario."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    nodes = []
    for instance in (1, 2):
        vehicle = f"px4_{instance}"
        nodes.append(Node(
            package="px4_offboard", executable="swarm_vehicle", namespace=vehicle,
            output="screen", parameters=[{
                "vehicle_id": vehicle, "target_system_id": instance + 1,
                "abort_after_ready_s": ParameterValue(LaunchConfiguration("abort_vehicle_2_after_s"), value_type=float) if instance == 2 else 0.0,
            }],
        ))
        nodes.append(Node(
            package="px4_offboard", executable="flight_trail", namespace=vehicle,
            output="screen", parameters=[{
                "input_mode": "swarm", "world_name": "obstacle_world",
                "trail_color": [0.1, 0.95, 1.0, 1.0] if instance == 1 else [1.0, 0.3, 0.7, 1.0],
                "route_color": [0.2, 1.0, 0.45, 0.9] if instance == 1 else [1.0, 0.65, 0.15, 0.9],
            }],
        ))
    return LaunchDescription([
        DeclareLaunchArgument("log_dir", default_value="demo_artifacts/swarm"),
        DeclareLaunchArgument("abort_vehicle_2_after_s", default_value="0.0",
                              description="Simulation fault injection; 0 disables"),
        *nodes,
        Node(package="px4_offboard", executable="swarm_coordinator", output="screen",
             parameters=[{"log_dir": LaunchConfiguration("log_dir")}]),
    ])
