from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent
from launch.conditions import IfCondition, UnlessCondition
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    config_path = Path(get_package_share_directory("sphero_rvr_driver")) / "config" / "rvr.yaml"
    allow_unsupervised_rvr = LaunchConfiguration("allow_unsupervised_rvr")
    return LaunchDescription([
        DeclareLaunchArgument(
            "allow_unsupervised_rvr",
            default_value="false",
            description="Developer-only bypass acknowledgement. Production must use supervised_rvr.launch.py.",
        ),
        EmitEvent(
            event=Shutdown(reason="rvr.launch.py is developer-only; set allow_unsupervised_rvr:=true explicitly"),
            condition=UnlessCondition(allow_unsupervised_rvr),
        ),
        # UNSUPERVISED low-level driver launch. Operator/mapping workflows should use
        # supervised_rvr.launch.py so ordinary /cmd_vel sources cannot bypass lidar collision stop.
        Node(
            package="sphero_rvr_driver",
            executable="rvr_node",
            name="sphero_rvr_driver",
            output="screen",
            parameters=[str(config_path)],
            condition=IfCondition(allow_unsupervised_rvr),
        )
    ])
