from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("seyond_mapping"))
    config = LaunchConfiguration("config")
    visualize = LaunchConfiguration("visualize")

    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=str(share / "config" / "mapping.yaml")),
            DeclareLaunchArgument("visualize", default_value="true"),
            Node(
                package="seyond_mapping",
                executable="seyond_pointcloud_node",
                output="screen",
                parameters=[config],
                remappings=[("points", "/seyond/points")],
            ),
            Node(
                package="kiss_icp",
                executable="kiss_icp_node",
                name="kiss_icp_node",
                output="screen",
                parameters=[config],
                remappings=[("pointcloud_topic", "/seyond/points")],
            ),
            Node(
                package="seyond_mapping",
                executable="map_accumulator_node",
                output="screen",
                parameters=[config],
                remappings=[
                    ("frame", "/kiss/frame"),
                    ("odometry", "/kiss/odometry"),
                    ("map", "/mapping/map"),
                    ("save_map", "/mapping/save_map"),
                    ("clear_map", "/mapping/clear_map"),
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                output="screen",
                arguments=["-d", str(share / "rviz" / "handheld_mapping.rviz")],
                condition=IfCondition(visualize),
            ),
        ]
    )
