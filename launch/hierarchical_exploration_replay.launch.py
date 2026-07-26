"""Default-off ROS 2 Jazzy/Nav2 graph for Phase 1 replay and simulation."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("sphero_rvr_driver"))
    nav2_params = str(share / "config" / "hierarchical_nav2.yaml")
    route_params = str(share / "config" / "live_route_runner.yaml")
    collision_params = str(share / "config" / "collision_stop.yaml")
    navigation_tree = str(
        share / "config" / "hierarchical_navigate_through_poses.xml"
    )
    default_map = str(
        share
        / "artifacts"
        / "phase1_recorded_slam_map"
        / "phase1_recorded_slam_map.yaml"
    )

    start_nav2 = LaunchConfiguration("start_nav2")
    start_bridge = LaunchConfiguration("start_bridge")
    start_supervisor = LaunchConfiguration("start_supervisor")
    start_loopback = LaunchConfiguration("start_loopback")
    map_yaml = LaunchConfiguration("map")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "start_nav2",
                default_value="false",
                description="Start replay-only Nav2 servers; no robot driver is launched.",
            ),
            DeclareLaunchArgument(
                "start_bridge",
                default_value="false",
                description="Start the replay-only private Nav2 command bridge.",
            ),
            DeclareLaunchArgument(
                "start_supervisor",
                default_value="false",
                description="Start collision_stop_node with no downstream RVR driver.",
            ),
            DeclareLaunchArgument(
                "start_loopback",
                default_value="false",
                description="Start Nav2 loopback simulation; no physical driver is launched.",
            ),
            DeclareLaunchArgument(
                "map",
                default_value=default_map,
                description="Saved trinary occupancy map for replay.",
            ),
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                parameters=[nav2_params, {"yaml_filename": map_yaml}],
                condition=IfCondition(start_nav2),
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                parameters=[nav2_params],
                condition=IfCondition(start_nav2),
            ),
            Node(
                package="nav2_controller",
                executable="controller_server",
                name="controller_server",
                parameters=[nav2_params],
                remappings=[("cmd_vel", "/nav2_cmd_vel_request")],
                condition=IfCondition(start_nav2),
            ),
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                parameters=[
                    nav2_params,
                    {"default_nav_through_poses_bt_xml": navigation_tree},
                ],
                condition=IfCondition(start_nav2),
            ),
            Node(
                package="nav2_behaviors",
                executable="behavior_server",
                name="behavior_server",
                parameters=[nav2_params],
                remappings=[("cmd_vel", "/nav2_cmd_vel_request")],
                condition=IfCondition(start_nav2),
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_hierarchical_navigation",
                parameters=[nav2_params],
                condition=IfCondition(start_nav2),
            ),
            Node(
                package="sphero_rvr_driver",
                executable="live_route_runner",
                name="live_route_runner",
                parameters=[
                    route_params,
                    {
                        "use_sim_time": True,
                        "hierarchical_mode_enabled": True,
                    },
                ],
                condition=IfCondition(start_bridge),
            ),
            Node(
                package="sphero_rvr_driver",
                executable="lidar_collision_stop_supervisor",
                name="lidar_collision_stop_supervisor",
                parameters=[collision_params, {"use_sim_time": True}],
                condition=IfCondition(start_supervisor),
            ),
            Node(
                package="sphero_rvr_driver",
                executable="rvr_nav2_loopback_sim",
                name="loopback_simulator",
                parameters=[
                    {
                        # The simulator must use wall time while it publishes
                        # /clock; consuming its own clock would leave every
                        # timestamp at zero.
                        "use_sim_time": False,
                        "base_frame_id": "base_link",
                        "odom_frame_id": "odom",
                        "map_frame_id": "map",
                        "scan_frame_id": "base_link",
                        "enable_stamped_cmd_vel": False,
                        "publish_map_odom_tf": True,
                        "publish_scan": True,
                        "publish_clock": True,
                        "scan_range_max": 5.0,
                        "scan_use_inf": False,
                    }
                ],
                remappings=[("cmd_vel", "/cmd_vel_motor")],
                condition=IfCondition(start_loopback),
            ),
            TimerAction(
                period=0.1,
                actions=[
                    ExecuteProcess(
                        cmd=[
                            "ros2",
                            "topic",
                            "pub",
                            "--once",
                            "/initialpose",
                            "geometry_msgs/msg/PoseWithCovarianceStamped",
                            (
                                "{header: {frame_id: map}, pose: {pose: "
                                "{position: {x: 0.12, y: -0.26}, "
                                "orientation: {z: 0.62932039, "
                                "w: 0.77714596}}}}"
                            ),
                        ],
                        output="screen",
                        condition=IfCondition(start_loopback),
                    )
                ],
            ),
        ]
    )
