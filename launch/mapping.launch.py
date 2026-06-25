from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = Path(get_package_share_directory("sphero_rvr_driver"))
    rvr_launch = pkg_share / "launch" / "rvr.launch.py"
    lidar_launch = pkg_share / "launch" / "lidar.launch.py"
    slam_config = pkg_share / "config" / "slam_toolbox.yaml"

    start_rvr = LaunchConfiguration("start_rvr")
    start_lidar = LaunchConfiguration("start_lidar")
    start_slam = LaunchConfiguration("start_slam")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription([
        DeclareLaunchArgument(
            "start_rvr",
            default_value="false",
            description=(
                "Start the live RVR driver. MOTOR-CAPABLE: exposes /cmd_vel and can move the robot."
            ),
        ),
        DeclareLaunchArgument(
            "start_lidar",
            default_value="true",
            description="Start the lidar-only launch that publishes /scan and base_link -> laser.",
        ),
        DeclareLaunchArgument(
            "start_slam",
            default_value="true",
            description="Start slam_toolbox online async mapping node.",
        ),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(rvr_launch)),
            condition=IfCondition(start_rvr),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(lidar_launch)),
            condition=IfCondition(start_lidar),
        ),
        Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            output="screen",
            parameters=[str(slam_config), {"use_sim_time": use_sim_time}],
            condition=IfCondition(start_slam),
        ),
    ])
