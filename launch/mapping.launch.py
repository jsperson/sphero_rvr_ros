from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = Path(get_package_share_directory("sphero_rvr_driver"))
    rvr_launch = pkg_share / "launch" / "supervised_rvr.launch.py"
    lidar_launch = pkg_share / "launch" / "lidar.launch.py"
    camera_launch = pkg_share / "launch" / "camera.launch.py"
    slam_config = pkg_share / "config" / "slam_toolbox.yaml"

    start_rvr = LaunchConfiguration("start_rvr")
    start_collision_stop = LaunchConfiguration("start_collision_stop")
    allow_unsupervised_rvr = LaunchConfiguration("allow_unsupervised_rvr")
    start_lidar = LaunchConfiguration("start_lidar")
    start_camera = LaunchConfiguration("start_camera")
    start_slam = LaunchConfiguration("start_slam")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription([
        DeclareLaunchArgument(
            "start_rvr",
            default_value="false",
            description=(
                "Start the live RVR driver. MOTOR-CAPABLE supervised by lidar collision stop."
            ),
        ),
        DeclareLaunchArgument(
            "start_collision_stop",
            default_value="true",
            description="Keep true whenever start_rvr is true; false is a development-only override.",
        ),
        DeclareLaunchArgument(
            "allow_unsupervised_rvr",
            default_value="false",
            description="Development-only acknowledgement for bypassing supervised_rvr.launch.py; operator default forbids it.",
        ),
        DeclareLaunchArgument(
            "start_lidar",
            default_value="true",
            description="Start the lidar-only launch that publishes /scan and base_link -> laser.",
        ),
        DeclareLaunchArgument(
            "start_camera",
            default_value="false",
            description=(
                "Start the Pi Camera 3 camera_ros launch and camera TF. Safe/no-motor, "
                "but it requires the camera stack and a measured camera_info_url for semantic localization."
            ),
        ),
        DeclareLaunchArgument(
            "start_slam",
            default_value="true",
            description="Start slam_toolbox online async mapping node.",
        ),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(rvr_launch)),
            launch_arguments={"start_collision_stop": start_collision_stop}.items(),
            condition=IfCondition(start_rvr),
        ),
        EmitEvent(
            event=Shutdown(reason="start_rvr requires start_collision_stop:=true unless allow_unsupervised_rvr:=true"),
            condition=IfCondition(
                PythonExpression([
                    "'", start_rvr, "' == 'true' and '", start_collision_stop, "' != 'true' and '", allow_unsupervised_rvr, "' != 'true'"
                ])
            ),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(lidar_launch)),
            condition=IfCondition(start_lidar),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(camera_launch)),
            condition=IfCondition(start_camera),
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
