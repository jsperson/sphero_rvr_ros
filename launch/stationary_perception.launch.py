from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("sphero_rvr_driver"))
    slam_share = Path(get_package_share_directory("slam_toolbox"))
    slam_config = package_share / "config" / "stationary_slam_toolbox.yaml"
    enrollment_dir = LaunchConfiguration("enrollment_dir")
    evidence_dir = LaunchConfiguration("evidence_dir")
    camera_info_url = LaunchConfiguration("camera_info_url")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "enrollment_dir",
                default_value="/home/jsperson/.local/share/sphero_rvr/face-enrollment",
                description=(
                    "Explicit identity subfolders containing enrollment-only face images."
                ),
            ),
            DeclareLaunchArgument(
                "evidence_dir",
                default_value="/home/jsperson/.local/state/sphero_rvr/stationary-evidence",
            ),
            DeclareLaunchArgument(
                "camera_info_url",
                default_value=(
                    "file:///home/jsperson/.ros/camera_info/"
                    "rvr_pi_camera3_stagec_800x600.yaml"
                ),
                description=(
                    "Stage C runtime copy of the measured calibration with the "
                    "current camera_ros device name."
                ),
            ),
            # These launches contain only sensor drivers and static transforms.
            # The rover driver, route runner, collision-to-motor graph, and UART
            # transport are intentionally absent.
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(package_share / "launch" / "lidar.launch.py")
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(package_share / "launch" / "camera.launch.py")
                ),
                launch_arguments={"camera_info_url": camera_info_url}.items(),
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="stationary_odom_to_base_tf",
                output="screen",
                arguments=[
                    "--x",
                    "0.0",
                    "--y",
                    "0.0",
                    "--z",
                    "0.0",
                    "--roll",
                    "0.0",
                    "--pitch",
                    "0.0",
                    "--yaw",
                    "0.0",
                    "--frame-id",
                    "odom",
                    "--child-frame-id",
                    "base_link",
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(slam_share / "launch" / "online_async_launch.py")
                ),
                launch_arguments={
                    "autostart": "true",
                    "use_lifecycle_manager": "false",
                    "use_sim_time": "false",
                    "slam_params_file": str(slam_config),
                }.items(),
            ),
            Node(
                package="sphero_rvr_driver",
                executable="stationary_perception",
                name="stationary_perception",
                output="screen",
                parameters=[
                    {
                        "enrollment_dir": enrollment_dir,
                        "evidence_dir": evidence_dir,
                    }
                ],
            ),
        ]
    )
