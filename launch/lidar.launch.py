from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    serial_port = LaunchConfiguration("serial_port")
    serial_baudrate = LaunchConfiguration("serial_baudrate")
    frame_id = LaunchConfiguration("frame_id")
    base_frame = LaunchConfiguration("base_frame")
    laser_x = LaunchConfiguration("laser_x")
    laser_y = LaunchConfiguration("laser_y")
    laser_z = LaunchConfiguration("laser_z")
    laser_yaw = LaunchConfiguration("laser_yaw")

    return LaunchDescription([
        DeclareLaunchArgument("serial_port", default_value="/dev/rplidar"),
        DeclareLaunchArgument("serial_baudrate", default_value="460800"),
        DeclareLaunchArgument("frame_id", default_value="laser"),
        DeclareLaunchArgument("base_frame", default_value="base_link"),
        DeclareLaunchArgument("laser_x", default_value="0.0"),
        DeclareLaunchArgument("laser_y", default_value="0.0"),
        DeclareLaunchArgument("laser_z", default_value="0.15"),
        DeclareLaunchArgument("laser_yaw", default_value="0.0"),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_to_laser_static_tf",
            output="screen",
            arguments=[
                "--x", laser_x,
                "--y", laser_y,
                "--z", laser_z,
                "--roll", "0.0",
                "--pitch", "0.0",
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
            parameters=[{
                "serial_port": serial_port,
                "serial_baudrate": serial_baudrate,
                "frame_id": frame_id,
                "inverted": False,
                "angle_compensate": True,
                "scan_mode": "Standard",
            }],
        ),
    ])
