from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    package_share = FindPackageShare("mtr_boat_core")
    params_file = LaunchConfiguration("params_file")
    dashboard_config = LaunchConfiguration("dashboard_config")

    enable_gnss = LaunchConfiguration("enable_gnss")
    enable_imu = LaunchConfiguration("enable_imu")
    imu_driver = LaunchConfiguration("imu_driver")
    enable_camera = LaunchConfiguration("enable_camera")
    enable_lidar = LaunchConfiguration("enable_lidar")
    enable_radar = LaunchConfiguration("enable_radar")
    enable_autonomy = LaunchConfiguration("enable_autonomy")
    enable_dashboard = LaunchConfiguration("enable_dashboard")
    enable_thruster = LaunchConfiguration("enable_thruster")

    sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [package_share, "launch", "sensors.launch.py"]
            )
        ),
        launch_arguments={
            "params_file": params_file,
            "enable_gnss": enable_gnss,
            "enable_imu": enable_imu,
            "imu_driver": imu_driver,
            "enable_camera": enable_camera,
            "enable_lidar": enable_lidar,
        }.items(),
    )
    control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [package_share, "launch", "control.launch.py"]
            )
        ),
        launch_arguments={
            "params_file": params_file,
            "enable_thruster": enable_thruster,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [package_share, "config", "boat.example.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "dashboard_config",
                default_value=PathJoinSubstitution(
                    [package_share, "config", "boat.example.json"]
                ),
            ),
            DeclareLaunchArgument("enable_gnss", default_value="true"),
            DeclareLaunchArgument("enable_imu", default_value="true"),
            DeclareLaunchArgument("imu_driver", default_value="bno055"),
            DeclareLaunchArgument("enable_camera", default_value="true"),
            DeclareLaunchArgument("enable_lidar", default_value="false"),
            DeclareLaunchArgument("enable_radar", default_value="false"),
            DeclareLaunchArgument("enable_autonomy", default_value="true"),
            DeclareLaunchArgument("enable_dashboard", default_value="true"),
            DeclareLaunchArgument(
                "enable_thruster",
                default_value="false",
                description="Allow the ROS thruster node to open the ESP32",
            ),
            sensors,
            control,
            Node(
                package="mtr_boat_core",
                executable="radar_uart_node",
                name="radar_uart_node",
                parameters=[params_file],
                output="screen",
                condition=IfCondition(enable_radar),
            ),
            Node(
                package="mtr_boat_core",
                executable="radar_nav_node",
                name="radar_nav_node",
                parameters=[params_file],
                output="screen",
                condition=IfCondition(enable_radar),
            ),
            Node(
                package="mtr_boat_core",
                executable="autonomy_node",
                name="autonomy_node",
                parameters=[params_file],
                output="screen",
                condition=IfCondition(enable_autonomy),
            ),
            ExecuteProcess(
                cmd=[
                    FindExecutable(name="web_dashboard"),
                    "--config",
                    dashboard_config,
                ],
                output="screen",
                condition=IfCondition(enable_dashboard),
            ),
        ]
    )
