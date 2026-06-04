from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="sphero_rvr_driver",
            executable="rvr_node",
            name="sphero_rvr_driver",
            output="screen",
            parameters=["config/rvr.yaml"],
        )
    ])
