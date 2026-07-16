from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = Path(get_package_share_directory("sphero_rvr_driver"))
    lidar_config = pkg_share / "config" / "lidar.yaml"

    serial_port = LaunchConfiguration("serial_port")
    serial_baudrate = LaunchConfiguration("serial_baudrate")
    frame_id = LaunchConfiguration("frame_id")
    base_frame = LaunchConfiguration("base_frame")
    laser_x = LaunchConfiguration("laser_x")
    laser_y = LaunchConfiguration("laser_y")
    laser_z = LaunchConfiguration("laser_z")
    laser_roll = LaunchConfiguration("laser_roll")
    laser_pitch = LaunchConfiguration("laser_pitch")
    laser_yaw = LaunchConfiguration("laser_yaw")

    return LaunchDescription([
        DeclareLaunchArgument("serial_port", default_value="/dev/rplidar"),
        DeclareLaunchArgument("serial_baudrate", default_value="460800"),
        DeclareLaunchArgument("frame_id", default_value="laser"),
        DeclareLaunchArgument("base_frame", default_value="base_link"),
        DeclareLaunchArgument(
            "laser_x",
            default_value="0.0",
            description="base_link -> laser x translation in meters; placeholder until measured.",
        ),
        DeclareLaunchArgument(
            "laser_y",
            default_value="0.0",
            description="base_link -> laser y translation in meters; placeholder until measured.",
        ),
        DeclareLaunchArgument(
            "laser_z",
            default_value="0.15",
            description="base_link -> laser z translation in meters; placeholder until measured.",
        ),
        DeclareLaunchArgument(
            "laser_roll",
            default_value="0.0",
            description="base_link -> laser roll in radians; placeholder until measured.",
        ),
        DeclareLaunchArgument(
            "laser_pitch",
            default_value="0.0",
            description="base_link -> laser pitch in radians; placeholder until measured.",
        ),
        DeclareLaunchArgument(
            "laser_yaw",
            default_value="0.0",
            description="base_link -> laser yaw in radians; placeholder until measured.",
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_to_laser_static_tf",
            output="screen",
            arguments=[
                "--x", laser_x,
                "--y", laser_y,
                "--z", laser_z,
                "--roll", laser_roll,
                "--pitch", laser_pitch,
                "--yaw", laser_yaw,
                "--frame-id", base_frame,
                "--child-frame-id", frame_id,
            ],
        ),
        Node(
            package="rplidar_ros",
            executable="rplidar_node",
            name="rplidar_node",
            output="screen",
            parameters=[
                str(lidar_config),
                {
                    "serial_port": serial_port,
                    "serial_baudrate": serial_baudrate,
                    "frame_id": frame_id,
                },
            ],
        ),
    ])
