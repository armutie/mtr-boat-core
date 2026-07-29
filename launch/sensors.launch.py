from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    default_params = PathJoinSubstitution(
        [FindPackageShare("mtr_boat_core"), "config", "boat.example.yaml"]
    )

    params_file = LaunchConfiguration("params_file")
    enable_gnss = LaunchConfiguration("enable_gnss")
    enable_imu = LaunchConfiguration("enable_imu")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="ROS parameter YAML for the boat sensor nodes",
            ),
            DeclareLaunchArgument("enable_gnss", default_value="true"),
            DeclareLaunchArgument("enable_imu", default_value="true"),
            Node(
                package="mtr_boat_core",
                executable="gnss_node",
                name="gnss_node",
                output="screen",
                parameters=[params_file],
                condition=IfCondition(enable_gnss),
            ),
            Node(
                package="mtr_boat_core",
                executable="imu_node",
                name="imu_node",
                output="screen",
                parameters=[params_file],
                condition=IfCondition(enable_imu),
            ),
        ]
    )
