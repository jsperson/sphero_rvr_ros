from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


OPTICAL_FRAME_ROLL = "-1.57079632679"
OPTICAL_FRAME_YAW = "-1.57079632679"


def generate_launch_description():
    pkg_share = Path(get_package_share_directory("sphero_rvr_driver"))
    camera_config = pkg_share / "config" / "camera.yaml"

    camera_info_url = LaunchConfiguration("camera_info_url")
    width = LaunchConfiguration("width")
    height = LaunchConfiguration("height")
    format_name = LaunchConfiguration("format")
    camera = LaunchConfiguration("camera")
    role = LaunchConfiguration("role")
    base_frame = LaunchConfiguration("base_frame")
    camera_frame_id = LaunchConfiguration("camera_frame_id")
    camera_optical_frame_id = LaunchConfiguration("camera_optical_frame_id")
    camera_x = LaunchConfiguration("camera_x")
    camera_y = LaunchConfiguration("camera_y")
    camera_z = LaunchConfiguration("camera_z")
    camera_roll = LaunchConfiguration("camera_roll")
    camera_pitch = LaunchConfiguration("camera_pitch")
    camera_yaw = LaunchConfiguration("camera_yaw")

    return LaunchDescription([
        DeclareLaunchArgument("width", default_value="800"),
        DeclareLaunchArgument("height", default_value="600"),
        DeclareLaunchArgument("format", default_value="BGR888"),
        DeclareLaunchArgument("camera", default_value="0"),
        DeclareLaunchArgument("role", default_value="viewfinder"),
        DeclareLaunchArgument(
            "camera_info_url",
            default_value="file:///home/jsperson/.ros/camera_info/rvr_pi_camera3_800x600.yaml",
            description=(
                "Measured camera_calibration YAML URL for camera_ros. The default is "
                "intentionally invalid/unconfigured; do not use semantic localization "
                "until /camera/camera_info has nonzero width/height, K, D, and distortion_model."
            ),
        ),
        DeclareLaunchArgument("base_frame", default_value="base_link"),
        DeclareLaunchArgument("camera_frame_id", default_value="camera_link"),
        DeclareLaunchArgument("camera_optical_frame_id", default_value="camera_optical_frame"),
        DeclareLaunchArgument(
            "camera_x",
            default_value="0.0587375",
            description="Measured base_link -> camera_link x translation in meters.",
        ),
        DeclareLaunchArgument(
            "camera_y",
            default_value="-0.0301625",
            description="Measured base_link -> camera_link y translation in meters.",
        ),
        DeclareLaunchArgument(
            "camera_z",
            default_value="0.114300",
            description="Measured base_link -> camera_link z translation in meters.",
        ),
        DeclareLaunchArgument(
            "camera_roll",
            default_value="0.0",
            description="Measured base_link -> camera_link roll in radians.",
        ),
        DeclareLaunchArgument(
            "camera_pitch",
            default_value="0.0",
            description="Measured base_link -> camera_link pitch in radians.",
        ),
        DeclareLaunchArgument(
            "camera_yaw",
            default_value="0.0",
            description="Measured base_link -> camera_link yaw in radians.",
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_to_camera_static_tf",
            output="screen",
            arguments=[
                "--x", camera_x,
                "--y", camera_y,
                "--z", camera_z,
                "--roll", camera_roll,
                "--pitch", camera_pitch,
                "--yaw", camera_yaw,
                "--frame-id", base_frame,
                "--child-frame-id", camera_frame_id,
            ],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="camera_to_optical_static_tf",
            output="screen",
            arguments=[
                "--x", "0.0",
                "--y", "0.0",
                "--z", "0.0",
                "--roll", "-1.57079632679",
                "--pitch", "0.0",
                "--yaw", "-1.57079632679",
                "--frame-id", camera_frame_id,
                "--child-frame-id", camera_optical_frame_id,
            ],
        ),
        Node(
            package="camera_ros",
            executable="camera_node",
            name="camera_node",
            output="screen",
            parameters=[
                str(camera_config),
                {
                    "camera_info_url": camera_info_url,
                    "width": width,
                    "height": height,
                    "format": format_name,
                    "camera": camera,
                    "role": role,
                    "frame_id": camera_optical_frame_id,
                    "camera_frame_id": camera_frame_id,
                },
            ],
        ),
    ])
