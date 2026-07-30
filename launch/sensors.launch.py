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
    enable_camera = LaunchConfiguration("enable_camera")
    camera_x = LaunchConfiguration("camera_x")
    camera_y = LaunchConfiguration("camera_y")
    camera_z = LaunchConfiguration("camera_z")
    camera_roll = LaunchConfiguration("camera_roll")
    camera_pitch = LaunchConfiguration("camera_pitch")
    camera_yaw = LaunchConfiguration("camera_yaw")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="ROS parameter YAML for the boat sensor nodes",
            ),
            DeclareLaunchArgument("enable_gnss", default_value="true"),
            DeclareLaunchArgument("enable_imu", default_value="true"),
            DeclareLaunchArgument("enable_camera", default_value="true"),
            DeclareLaunchArgument("camera_x", default_value="0.0"),
            DeclareLaunchArgument("camera_y", default_value="0.0"),
            DeclareLaunchArgument("camera_z", default_value="0.0"),
            DeclareLaunchArgument("camera_roll", default_value="0.0"),
            DeclareLaunchArgument("camera_pitch", default_value="0.0"),
            DeclareLaunchArgument("camera_yaw", default_value="0.0"),
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
            Node(
                package="mtr_boat_core",
                executable="camera_node",
                name="camera_node",
                output="screen",
                parameters=[params_file],
                additional_env={"PYTHONNOUSERSITE": "1"},
                condition=IfCondition(enable_camera),
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="camera_mount_transform",
                output="screen",
                arguments=[
                    "--x",
                    camera_x,
                    "--y",
                    camera_y,
                    "--z",
                    camera_z,
                    "--roll",
                    camera_roll,
                    "--pitch",
                    camera_pitch,
                    "--yaw",
                    camera_yaw,
                    "--frame-id",
                    "base_link",
                    "--child-frame-id",
                    "camera_link",
                ],
                condition=IfCondition(enable_camera),
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="camera_optical_transform",
                output="screen",
                arguments=[
                    "--x",
                    "0.0",
                    "--y",
                    "0.0",
                    "--z",
                    "0.0",
                    "--roll",
                    "-1.5707963267948966",
                    "--pitch",
                    "0.0",
                    "--yaw",
                    "-1.5707963267948966",
                    "--frame-id",
                    "camera_link",
                    "--child-frame-id",
                    "camera_optical_frame",
                ],
                condition=IfCondition(enable_camera),
            ),
        ]
    )
