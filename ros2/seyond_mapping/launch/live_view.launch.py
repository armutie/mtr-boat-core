from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("seyond_mapping"))
    config = LaunchConfiguration("config")
    port = LaunchConfiguration("port")
    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=str(share / "config" / "mapping.yaml")),
            DeclareLaunchArgument("port", default_value="8080"),
            Node(
                package="seyond_mapping",
                executable="seyond_pointcloud_node",
                output="screen",
                parameters=[config],
                remappings=[("points", "/seyond/points")],
            ),
            Node(
                package="seyond_mapping",
                executable="live_web_viewer_node",
                output="screen",
                parameters=[
                    config,
                    {
                        "port": port,
                        "html_path": str(share / "web" / "live.html"),
                    },
                ],
                remappings=[("points", "/seyond/points")],
            ),
        ]
    )
