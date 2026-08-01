import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory("mtr_boat_core")
    default_params = os.path.join(
        package_share,
        "config",
        "boat.example.yaml",
    )
    params_file = LaunchConfiguration("params_file")
    enable_thruster = LaunchConfiguration("enable_thruster")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
            ),
            DeclareLaunchArgument(
                "enable_thruster",
                default_value="false",
                description="Allow the ROS thruster node to open the ESP32",
            ),
            Node(
                package="mtr_boat_core",
                executable="control_supervisor_node",
                name="control_supervisor_node",
                parameters=[params_file],
                output="screen",
                respawn=True,
                respawn_delay=2.0,
            ),
            Node(
                package="mtr_boat_core",
                executable="thruster_node",
                name="thruster_node",
                parameters=[params_file],
                output="screen",
                respawn=True,
                respawn_delay=2.0,
                condition=IfCondition(enable_thruster),
            ),
        ]
    )
