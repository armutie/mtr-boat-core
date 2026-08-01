from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    default_params = PathJoinSubstitution(
        [FindPackageShare("mtr_boat_core"), "config", "boat.example.yaml"]
    )

    params_file = LaunchConfiguration("params_file")
    enable_gnss = LaunchConfiguration("enable_gnss")
    enable_imu = LaunchConfiguration("enable_imu")
    imu_driver = LaunchConfiguration("imu_driver")
    base_frame_id = LaunchConfiguration("base_frame_id")
    imu_frame_id = LaunchConfiguration("imu_frame_id")
    imu_x = LaunchConfiguration("imu_x")
    imu_y = LaunchConfiguration("imu_y")
    imu_z = LaunchConfiguration("imu_z")
    imu_roll = LaunchConfiguration("imu_roll")
    imu_pitch = LaunchConfiguration("imu_pitch")
    imu_yaw = LaunchConfiguration("imu_yaw")

    def imu_driver_enabled(driver: str) -> IfCondition:
        return IfCondition(
            PythonExpression(
                [
                    "'",
                    enable_imu,
                    "'.lower() == 'true' and '",
                    imu_driver,
                    f"' == '{driver}'",
                ]
            )
        )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="ROS parameter YAML for the boat sensor nodes",
            ),
            DeclareLaunchArgument("enable_gnss", default_value="true"),
            DeclareLaunchArgument("enable_imu", default_value="true"),
            DeclareLaunchArgument(
                "imu_driver",
                default_value="bno055",
                description="IMU implementation: bno055 or mpu6050",
            ),
            DeclareLaunchArgument("base_frame_id", default_value="base_link"),
            DeclareLaunchArgument("imu_frame_id", default_value="imu_link"),
            DeclareLaunchArgument("imu_x", default_value="0.0"),
            DeclareLaunchArgument("imu_y", default_value="0.0"),
            DeclareLaunchArgument("imu_z", default_value="0.0"),
            DeclareLaunchArgument("imu_roll", default_value="0.0"),
            DeclareLaunchArgument("imu_pitch", default_value="0.0"),
            DeclareLaunchArgument("imu_yaw", default_value="0.0"),
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
                parameters=[params_file, {"frame_id": imu_frame_id}],
                condition=imu_driver_enabled("mpu6050"),
            ),
            Node(
                package="mtr_boat_core",
                executable="bno055_node",
                name="bno055_node",
                output="screen",
                parameters=[params_file, {"frame_id": imu_frame_id}],
                condition=imu_driver_enabled("bno055"),
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="imu_static_transform",
                arguments=[
                    "--x",
                    imu_x,
                    "--y",
                    imu_y,
                    "--z",
                    imu_z,
                    "--roll",
                    imu_roll,
                    "--pitch",
                    imu_pitch,
                    "--yaw",
                    imu_yaw,
                    "--frame-id",
                    base_frame_id,
                    "--child-frame-id",
                    imu_frame_id,
                ],
                condition=IfCondition(enable_imu),
            ),
        ]
    )
