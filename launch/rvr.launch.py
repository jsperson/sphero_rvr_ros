from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    config_path = Path(get_package_share_directory("sphero_rvr_driver")) / "config" / "rvr.yaml"
    return LaunchDescription([
        # UNSUPERVISED low-level driver launch. Operator/mapping workflows should use
        # supervised_rvr.launch.py so ordinary /cmd_vel sources cannot bypass lidar collision stop.
        Node(
            package="sphero_rvr_driver",
            executable="rvr_node",
            name="sphero_rvr_driver",
            output="screen",
            parameters=[str(config_path)],
        )
    ])
