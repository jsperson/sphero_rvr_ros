from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("sphero_rvr_driver"))
    mapping_launch = package_share / "launch" / "mapping.launch.py"

    start_rvr = LaunchConfiguration("start_rvr")
    start_live_route_runner = LaunchConfiguration("start_live_route_runner")
    serial_port = LaunchConfiguration("serial_port")
    lidar_serial_port = LaunchConfiguration("lidar_serial_port")
    enrollment_dir = LaunchConfiguration("enrollment_dir")
    evidence_dir = LaunchConfiguration("evidence_dir")
    camera_info_url = LaunchConfiguration("camera_info_url")

    semantic_perception = Node(
        package="sphero_rvr_driver",
        executable="stationary_perception",
        name="adaptive_mission_semantic_perception",
        output="screen",
        parameters=[
            {
                "stationary_session": False,
                "enrollment_dir": enrollment_dir,
                "evidence_dir": evidence_dir,
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "start_rvr",
                default_value="false",
                description=(
                    "MOTOR-CAPABLE only when explicitly true. The default starts "
                    "the Adaptive mission perception graph without a rover driver."
                ),
            ),
            DeclareLaunchArgument(
                "start_live_route_runner",
                default_value="false",
                description=(
                    "Typed Adaptive mission route executor; requires start_rvr:=true for "
                    "physical movement."
                ),
            ),
            DeclareLaunchArgument("serial_port", default_value="/dev/ttyAMA0"),
            DeclareLaunchArgument(
                "lidar_serial_port",
                default_value="/dev/rplidar",
                description=(
                    "Dedicated lidar device, intentionally distinct from "
                    "the RVR serial_port argument."
                ),
            ),
            DeclareLaunchArgument(
                "enrollment_dir",
                default_value=(
                    "/home/jsperson/.local/share/sphero_rvr/face-enrollment"
                ),
            ),
            DeclareLaunchArgument(
                "evidence_dir",
                default_value=(
                    "/home/jsperson/.local/state/sphero_rvr/adaptive-mission-perception"
                ),
            ),
            DeclareLaunchArgument(
                "camera_info_url",
                default_value=(
                    "file:///home/jsperson/.ros/camera_info/"
                    "rvr_pi_camera3_800x600.yaml"
                ),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(mapping_launch)),
                launch_arguments={
                    "start_rvr": start_rvr,
                    "start_collision_stop": "true",
                    "start_live_route_runner": start_live_route_runner,
                    "serial_port": serial_port,
                    "lidar_serial_port": lidar_serial_port,
                    "start_lidar": "true",
                    "start_camera": "true",
                    "camera_info_url": camera_info_url,
                    "start_slam": "true",
                    "use_sim_time": "false",
                }.items(),
            ),
            semantic_perception,
            EmitEvent(
                event=Shutdown(
                    reason=(
                        "start_live_route_runner requires start_rvr:=true in "
                        "the Adaptive mission perception launch"
                    )
                ),
                condition=IfCondition(
                    PythonExpression(
                        [
                            "'",
                            start_live_route_runner,
                            "' == 'true' and '",
                            start_rvr,
                            "' != 'true'",
                        ]
                    )
                ),
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=semantic_perception,
                    on_exit=[
                        EmitEvent(
                            event=Shutdown(
                                reason=(
                                    "Adaptive mission semantic perception exited; "
                                    "shutting down the supervised graph"
                                )
                            )
                        )
                    ],
                )
            ),
        ]
    )
