from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = Path(get_package_share_directory("sphero_rvr_driver"))
    rvr_config = pkg_share / "config" / "rvr.yaml"
    collision_stop_config = pkg_share / "config" / "collision_stop.yaml"
    range_motion_config = pkg_share / "config" / "range_motion.yaml"

    serial_port = LaunchConfiguration("serial_port")
    start_supervisor = LaunchConfiguration("start_collision_stop")
    start_range_motion = LaunchConfiguration("start_range_motion")

    rvr_node = Node(
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
    )
    collision_stop_node = Node(
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
        condition=IfCondition(start_supervisor),
    )
    range_motion_node = Node(
        package="sphero_rvr_driver",
        executable="range_motion_controller",
        name="range_motion_controller",
        output="screen",
        parameters=[str(range_motion_config)],
        remappings=[
            ("cmd_vel", "/cmd_vel"),
            ("scan", "/scan"),
            ("odom", "/odom"),
        ],
        condition=IfCondition(
            PythonExpression(["'", start_supervisor, "' == 'true' and '", start_range_motion, "' == 'true'"])
        ),
    )

    return LaunchDescription([
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyAMA0"),
        DeclareLaunchArgument(
            "start_collision_stop",
            default_value="true",
            description="Must remain true for operator motor-capable launches; false is development-only.",
        ),
        DeclareLaunchArgument(
            "start_range_motion",
            default_value="false",
            description="Start the optional closed-loop lidar range-motion controller above /cmd_vel.",
        ),
        rvr_node,
        range_motion_node,
        collision_stop_node,
        RegisterEventHandler(
            OnProcessExit(
                target_action=collision_stop_node,
                on_exit=[
                    EmitEvent(
                        event=Shutdown(
                            reason="lidar_collision_stop_supervisor exited; shutting down motor-capable launch"
                        )
                    )
                ],
            ),
            condition=IfCondition(start_supervisor),
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=rvr_node,
                on_exit=[EmitEvent(event=Shutdown(reason="sphero_rvr_driver exited; shutting down supervised launch"))],
            )
        ),
    ])
