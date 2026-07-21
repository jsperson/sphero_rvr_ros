from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    database = LaunchConfiguration("database")
    socket_path = LaunchConfiguration("socket")
    service = ExecuteProcess(
        cmd=[
            "rvr_mission_service",
            "--socket",
            socket_path,
            "--database",
            database,
        ],
        output="screen",
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "database",
                default_value=str(Path.home() / ".local/state/sphero-rvr/missions.sqlite3"),
            ),
            DeclareLaunchArgument(
                "socket",
                default_value=str(Path.home() / ".local/state/sphero-rvr/mission.sock"),
            ),
            service,
        ]
    )
