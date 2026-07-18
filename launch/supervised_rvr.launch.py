from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = Path(get_package_share_directory("sphero_rvr_driver"))
    rvr_config = pkg_share / "config" / "rvr.yaml"
    collision_stop_config = pkg_share / "config" / "collision_stop.yaml"

    serial_port = LaunchConfiguration("serial_port")
    start_supervisor = LaunchConfiguration("start_collision_stop")

    return LaunchDescription([
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyAMA0"),
        DeclareLaunchArgument(
            "start_collision_stop",
            default_value="true",
            description="Must remain true for operator motor-capable launches; false is development-only.",
        ),
        Node(
            package="sphero_rvr_driver",
            executable="rvr_node",
            name="sphero_rvr_driver",
            output="screen",
            parameters=[str(rvr_config), {"serial_port": serial_port}],
            remappings=[
                ("cmd_vel", "/cmd_vel_motor"),
                ("stop", "/rvr_driver/stop"),
                ("estop", "/rvr_driver/estop"),
                ("clear_estop", "/rvr_driver/clear_estop"),
            ],
        ),
        Node(
            package="sphero_rvr_driver",
            executable="lidar_collision_stop_supervisor",
            name="lidar_collision_stop_supervisor",
            output="screen",
            parameters=[str(collision_stop_config)],
            remappings=[
                ("cmd_vel", "/cmd_vel"),
                ("cmd_vel_motor", "/cmd_vel_motor"),
                ("stop", "/stop"),
                ("estop", "/estop"),
                ("clear_estop", "/clear_estop"),
            ],
        ),
    ])
