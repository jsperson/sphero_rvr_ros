"""Default-off live physical graph for M7.5 hierarchical exploration."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("sphero_rvr_driver"))
    mapping_launch = share / "launch" / "mapping.launch.py"
    slam_params = str(
        share / "config" / "hierarchical_slam_toolbox.yaml"
    )
    nav2_params = str(share / "config" / "hierarchical_nav2_physical.yaml")
    route_params = str(share / "config" / "live_route_runner.yaml")
    navigation_tree = str(
        share / "config" / "hierarchical_navigate_through_poses.xml"
    )

    start_sensors = LaunchConfiguration("start_sensors")
    start_motion_stack = LaunchConfiguration("start_motion_stack")
    start_nav2 = LaunchConfiguration("start_nav2")
    start_authority = LaunchConfiguration("start_authority")
    start_semantic_adapter = LaunchConfiguration("start_semantic_adapter")
    camera_info_url = LaunchConfiguration("camera_info_url")
    reviewed_sha = LaunchConfiguration("reviewed_sha")
    source_sha = LaunchConfiguration("source_sha")
    deployed_sha = LaunchConfiguration("deployed_sha")
    approval_file = LaunchConfiguration("approval_file")
    proposal_file = LaunchConfiguration("proposal_file")
    graph_audit_file = LaunchConfiguration("graph_audit_file")

    exact_binding = PythonExpression(
        [
            "'",
            start_sensors,
            "' == 'true' and '",
            start_motion_stack,
            "' == 'true' and '",
            start_nav2,
            "' == 'true' and '",
            start_authority,
            "' == 'true' and '",
            start_semantic_adapter,
            "' == 'true'",
        ]
    )

    mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(mapping_launch)),
        launch_arguments={
            # An inconsistent invocation cannot briefly start the driver
            # before the fail-closed shutdown event is processed.
            "start_rvr": exact_binding,
            "start_collision_stop": "true",
            "allow_unsupervised_rvr": "false",
            "start_lidar": start_sensors,
            "start_camera": start_sensors,
            "camera_info_url": camera_info_url,
            "start_slam": start_sensors,
            "slam_autostart": start_sensors,
            "slam_params_file": slam_params,
            "start_live_route_runner": "false",
            "use_sim_time": "false",
        }.items(),
    )
    authority = Node(
        package="sphero_rvr_driver",
        executable="hierarchical_physical_authority",
        name="hierarchical_physical_authority",
        output="screen",
        parameters=[
            {
                "enabled": True,
                "use_sim_time": False,
                "source_sha": source_sha,
                "deployed_sha": deployed_sha,
                "reviewed_sha": reviewed_sha,
                "approval_file": approval_file,
            }
        ],
        condition=IfCondition(start_authority),
    )
    semantic_adapter = Node(
        package="sphero_rvr_driver",
        executable="hierarchical_nav2_adapter",
        name="hierarchical_nav2_adapter",
        output="screen",
        parameters=[
            {
                "enabled": True,
                "use_sim_time": False,
                "source_sha": source_sha,
                "deployed_sha": deployed_sha,
                "reviewed_sha": reviewed_sha,
            }
        ],
        condition=IfCondition(start_semantic_adapter),
    )
    semantic_perception = Node(
        package="sphero_rvr_driver",
        executable="stationary_perception",
        name="hierarchical_semantic_perception",
        output="screen",
        parameters=[
            {
                "stationary_session": False,
                "evidence_dir": (
                    "/home/jsperson/.local/state/sphero_rvr/"
                    "hierarchical-perception"
                ),
            }
        ],
        condition=IfCondition(exact_binding),
    )
    mission_controller = Node(
        package="sphero_rvr_driver",
        executable="hierarchical_mission_controller",
        name="hierarchical_mission_controller",
        output="screen",
        parameters=[
            {
                "enabled": True,
                "use_sim_time": False,
                "source_sha": source_sha,
                "deployed_sha": deployed_sha,
                "reviewed_sha": reviewed_sha,
                "proposal_file": proposal_file,
                "graph_audit_file": graph_audit_file,
            }
        ],
        condition=IfCondition(exact_binding),
    )
    route_bridge = Node(
        package="sphero_rvr_driver",
        executable="live_route_runner",
        name="live_route_runner",
        output="screen",
        parameters=[
            route_params,
            {
                "use_sim_time": False,
                "hierarchical_mode_enabled": True,
                "hierarchical_physical_binding_enabled": True,
                "nav2_cmd_lease_s": 0.50,
                "source_sha": source_sha,
                "deployed_sha": deployed_sha,
                "hierarchical_physical_reviewed_sha": reviewed_sha,
            },
        ],
        condition=IfCondition(exact_binding),
    )

    nodes = [
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
            name="lifecycle_manager_hierarchical_physical",
            parameters=[nav2_params],
            condition=IfCondition(start_nav2),
        ),
    ]

    actions = [
        DeclareLaunchArgument(
            "start_sensors",
            default_value="false",
            description="Start lidar, camera, and live SLAM; default is no sensors.",
        ),
        DeclareLaunchArgument(
            "start_motion_stack",
            default_value="false",
            description="MOTOR-CAPABLE driver and independent collision supervisor.",
        ),
        DeclareLaunchArgument(
            "start_nav2",
            default_value="false",
            description="Start live-time Nav2 servers; default is off.",
        ),
        DeclareLaunchArgument(
            "start_authority",
            default_value="false",
            description="Start exact-SHA M7.6 authority owner; default is off.",
        ),
        DeclareLaunchArgument(
            "start_semantic_adapter",
            default_value="false",
            description="Start deterministic semantic-goal Nav2 adapter; default is off.",
        ),
        DeclareLaunchArgument(
            "camera_info_url",
            default_value=(
                "file:///home/jsperson/.ros/camera_info/"
                "rvr_pi_imx708_calibrated_800x600.yaml"
            ),
            description=(
                "Measured CameraInfo whose camera_name matches the live imx708 "
                "device used for the canonical physical mission."
            ),
        ),
        DeclareLaunchArgument("source_sha", default_value=""),
        DeclareLaunchArgument("deployed_sha", default_value=""),
        DeclareLaunchArgument("reviewed_sha", default_value=""),
        DeclareLaunchArgument("approval_file", default_value=""),
        DeclareLaunchArgument("proposal_file", default_value=""),
        DeclareLaunchArgument("graph_audit_file", default_value=""),
        mapping,
        *nodes,
        authority,
        semantic_perception,
        mission_controller,
        semantic_adapter,
        route_bridge,
        EmitEvent(
            event=Shutdown(
                reason=(
                    "motor-capable hierarchical launch requires sensors, Nav2, "
                    "authority, and semantic adapter together"
                )
            ),
            condition=IfCondition(
                PythonExpression(
                    [
                        "'",
                        start_motion_stack,
                        "' == 'true' and not ('",
                        start_sensors,
                        "' == 'true' and '",
                        start_nav2,
                        "' == 'true' and '",
                        start_authority,
                        "' == 'true' and '",
                        start_semantic_adapter,
                        "' == 'true')",
                    ]
                )
            ),
        ),
    ]
    for critical in (
        *nodes,
        authority,
        semantic_perception,
        mission_controller,
        semantic_adapter,
        route_bridge,
    ):
        actions.append(
            RegisterEventHandler(
                OnProcessExit(
                    target_action=critical,
                    on_exit=[
                        EmitEvent(
                            event=Shutdown(
                                reason=(
                                    "hierarchical physical authority component exited"
                                )
                            )
                        )
                    ],
                ),
                condition=IfCondition(exact_binding),
            )
        )
    return LaunchDescription(actions)
