from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = Path(get_package_share_directory("sphero_rvr_driver"))
    config = pkg_share / "config" / "mission_service.yaml"

    service = Node(
        package="sphero_rvr_driver",
        executable="live_mission_service",
        name="live_mission_service",
        output="screen",
        parameters=[
            str(config),
            {
                "source_sha": LaunchConfiguration("source_sha"),
                "deployed_sha": LaunchConfiguration("deployed_sha"),
                "live_execution_enabled": LaunchConfiguration("live_execution_enabled"),
                "live_execution_reviewed_sha": LaunchConfiguration("live_execution_reviewed_sha"),
            },
        ],
        condition=IfCondition(LaunchConfiguration("start_service")),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "start_service",
                default_value="false",
                description="Explicitly enable the read-only persistent service; false starts no process.",
            ),
            DeclareLaunchArgument(
                "source_sha",
                default_value=EnvironmentVariable("RVR_SOURCE_SHA", default_value=""),
                description="Reviewed source commit; startup fails when absent.",
            ),
            DeclareLaunchArgument(
                "deployed_sha",
                default_value=EnvironmentVariable("RVR_DEPLOYED_SHA", default_value=""),
                description="Installed source commit; startup fails when absent.",
            ),
            DeclareLaunchArgument(
                "live_execution_enabled",
                default_value="false",
                description="Explicit reviewed physical route-authority gate; default false.",
            ),
            DeclareLaunchArgument(
                "live_execution_reviewed_sha",
                default_value="disabled",
                description="Must exactly match source_sha and deployed_sha when execution is enabled.",
            ),
            service,
        ]
    )
