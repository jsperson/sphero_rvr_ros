"""Default-off camera/lidar launch for the stationary M7.2 survey.

There is intentionally no RVR driver, command publisher, collision-to-motor
chain, Nav2 process, SLAM process, or physical-execution authority here.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("sphero_rvr_driver"))
    enabled = LaunchConfiguration("survey_session_enabled")
    camera_info_url = LaunchConfiguration("camera_info_url")

    stationary_inputs = GroupAction(
        condition=IfCondition(enabled),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(package_share / "launch" / "lidar.launch.py"))
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(package_share / "launch" / "camera.launch.py")),
                launch_arguments={"camera_info_url": camera_info_url}.items(),
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="m7_survey_map_to_base_tf",
                output="screen",
                arguments=[
                    "--x", "0.0",
                    "--y", "0.0",
                    "--z", "0.0",
                    "--roll", "0.0",
                    "--pitch", "0.0",
                    "--yaw", "0.0",
                    "--frame-id", "map",
                    "--child-frame-id", "base_link",
                ],
            ),
        ],
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "survey_session_enabled",
                default_value="false",
                description="Explicitly enable only stationary M7.2 camera/lidar inputs.",
            ),
            DeclareLaunchArgument(
                "camera_info_url",
                default_value=(
                    "file:///home/jsperson/.ros/camera_info/"
                    "rvr_pi_camera3_800x600.yaml"
                ),
            ),
            stationary_inputs,
        ]
    )
