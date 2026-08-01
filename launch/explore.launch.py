"""Minimal Get-Well explore graph with one supervised native-SI command path."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("sphero_rvr_driver"))
    nav2_bt_share = Path(get_package_share_directory("nav2_bt_navigator"))

    supervised_launch = share / "launch" / "supervised_rvr.launch.py"
    lidar_launch = share / "launch" / "lidar.launch.py"
    mapping_launch = share / "launch" / "mapping.launch.py"
    default_rvr_params = share / "config" / "lean_rvr_native_si.yaml"
    default_slam_params = share / "config" / "slam_toolbox.yaml"
    default_nav2_params = share / "config" / "lean_nav2.yaml"
    standard_nav_to_pose_bt = (
        nav2_bt_share
        / "behavior_trees"
        / "navigate_w_replanning_only_if_goal_is_updated.xml"
    )

    start_motion_stack = LaunchConfiguration("start_motion_stack")
    serial_port = LaunchConfiguration("serial_port")
    lidar_serial_port = LaunchConfiguration("lidar_serial_port")
    rvr_params_file = LaunchConfiguration("rvr_params_file")
    slam_params_file = LaunchConfiguration("slam_params_file")
    nav2_params_file = LaunchConfiguration("nav2_params_file")

    supervised = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(supervised_launch)),
        launch_arguments={
            "start_collision_stop": "true",
            "start_range_motion": "false",
            "start_live_route_runner": "false",
            "serial_port": serial_port,
            "rvr_params_file": rvr_params_file,
        }.items(),
        condition=IfCondition(start_motion_stack),
    )
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(lidar_launch)),
        launch_arguments={"serial_port": lidar_serial_port}.items(),
    )
    mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(mapping_launch)),
        launch_arguments={
            # Lidar and its measured base_link -> laser static transform are
            # included once above. The supervised driver is included once too.
            "start_rvr": "false",
            "start_lidar": "false",
            "start_camera": "false",
            "start_slam": "true",
            "slam_autostart": "true",
            "slam_params_file": slam_params_file,
            "use_sim_time": "false",
        }.items(),
    )

    nav2_nodes = [
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=[nav2_params_file],
        ),
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            parameters=[nav2_params_file],
            remappings=[("cmd_vel", "/cmd_vel")],
        ),
        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            output="screen",
            parameters=[
                nav2_params_file,
                {"default_nav_to_pose_bt_xml": str(standard_nav_to_pose_bt)},
            ],
        ),
        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            output="screen",
            parameters=[nav2_params_file],
            remappings=[("cmd_vel", "/cmd_vel")],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_explore",
            output="screen",
            parameters=[nav2_params_file],
        ),
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "start_motion_stack",
                default_value="false",
                description=(
                    "MOTOR-CAPABLE: start the collision supervisor and RVR driver. "
                    "Enable only for an attended level-room bringup."
                ),
            ),
            DeclareLaunchArgument("serial_port", default_value="/dev/ttyAMA0"),
            DeclareLaunchArgument(
                "lidar_serial_port", default_value="/dev/ttyUSB0"
            ),
            DeclareLaunchArgument(
                "rvr_params_file", default_value=str(default_rvr_params)
            ),
            DeclareLaunchArgument(
                "slam_params_file", default_value=str(default_slam_params)
            ),
            DeclareLaunchArgument(
                "nav2_params_file", default_value=str(default_nav2_params)
            ),
            supervised,
            lidar,
            mapping,
            *nav2_nodes,
        ]
    )
