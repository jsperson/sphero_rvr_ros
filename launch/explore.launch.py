"""Quarantined Get-Well graph with a supervised native tank-SI path."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory("sphero_rvr_driver"))
    nav2_bt_share = Path(get_package_share_directory("nav2_bt_navigator"))

    supervised_launch = share / "launch" / "supervised_rvr.launch.py"
    lidar_launch = share / "launch" / "lidar.launch.py"
    mapping_launch = share / "launch" / "mapping.launch.py"
    default_rvr_params = share / "config" / "lean_rvr_tank_si.yaml"
    default_slam_params = share / "config" / "slam_toolbox.yaml"
    default_nav2_params = share / "config" / "lean_nav2.yaml"
    default_explore_lite_params = share / "config" / "lean_explore_lite.yaml"
    default_coverage_params = share / "config" / "coverage_explorer.yaml"
    standard_nav_to_pose_bt = (
        nav2_bt_share
        / "behavior_trees"
        / "navigate_to_pose_w_replanning_and_recovery.xml"
    )
    # Decisive mode drops controller_server (and with it Nav2's local costmap),
    # so the stock BT's local-costmap clears break bt_navigator bringup. Use a
    # variant whose costmap clears target the global costmap instead.
    decisive_nav_to_pose_bt = share / "behavior_trees" / "navigate_to_pose_decisive.xml"

    start_motion_stack = LaunchConfiguration("start_motion_stack")
    start_explore = LaunchConfiguration("start_explore")
    enable_imu_fusion = LaunchConfiguration("enable_imu_fusion")
    use_decisive_controller = LaunchConfiguration("use_decisive_controller")
    use_coverage_explorer = LaunchConfiguration("use_coverage_explorer")
    serial_port = LaunchConfiguration("serial_port")
    lidar_serial_port = LaunchConfiguration("lidar_serial_port")
    rvr_params_file = LaunchConfiguration("rvr_params_file")
    slam_params_file = LaunchConfiguration("slam_params_file")
    nav2_params_file = LaunchConfiguration("nav2_params_file")
    explore_lite_params_file = LaunchConfiguration("explore_lite_params_file")
    coverage_params_file = LaunchConfiguration("coverage_params_file")

    # start_explore picks ONE explorer: coverage+frontier (use_coverage_explorer)
    # or explore_lite's frontier-only (default). Both drive via NavigateToPose.
    explore_lite_active = PythonExpression(
        ["'", start_explore, "' == 'true' and '", use_coverage_explorer, "' == 'false'"]
    )
    coverage_active = PythonExpression(
        ["'", start_explore, "' == 'true' and '", use_coverage_explorer, "' == 'true'"]
    )

    supervised = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(supervised_launch)),
        launch_arguments={
            "start_collision_stop": "true",
            "enable_imu_fusion": enable_imu_fusion,
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

    # Pick the navigate-to-pose BT by mode: decisive mode uses the global-costmap
    # variant (no local costmap exists), RPP mode uses the stock tree.
    nav_to_pose_bt_xml = ParameterValue(
        PythonExpression(
            [
                "'",
                str(decisive_nav_to_pose_bt),
                "' if '",
                use_decisive_controller,
                "' == 'true' else '",
                str(standard_nav_to_pose_bt),
                "'",
            ]
        ),
        value_type=str,
    )

    nav2_nodes = [
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=[nav2_params_file],
        ),
        # Default controller: Nav2's RPP/RotationShim FollowPath.
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            parameters=[nav2_params_file],
            remappings=[("cmd_vel", "/cmd_vel")],
            condition=UnlessCondition(use_decisive_controller),
        ),
        # Opt-in replacement: our decisive FollowPath controller (drive straight
        # when aligned, arc while moving, pivot only when large) — same follow_path
        # action, not lifecycle-managed. Run instead of controller_server, not both.
        Node(
            package="sphero_rvr_driver",
            executable="decisive_controller",
            name="decisive_controller",
            output="screen",
            remappings=[("cmd_vel", "/cmd_vel")],
            condition=IfCondition(use_decisive_controller),
        ),
        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            output="screen",
            parameters=[
                nav2_params_file,
                {"default_nav_to_pose_bt_xml": nav_to_pose_bt_xml},
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
        # Lifecycle manager. Default manages controller_server too; in decisive
        # mode it must NOT (the decisive controller is a plain node), so it manages
        # only planner/behavior/bt.
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_explore",
            output="screen",
            parameters=[nav2_params_file],
            condition=UnlessCondition(use_decisive_controller),
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_explore",
            output="screen",
            parameters=[
                nav2_params_file,
                {"node_names": ["planner_server", "behavior_server", "bt_navigator"]},
            ],
            condition=IfCondition(use_decisive_controller),
        ),
    ]
    explore_lite = Node(
        package="explore_lite",
        executable="explore",
        name="explore_node",
        output="screen",
        parameters=[explore_lite_params_file],
        remappings=[("navigate_to_pose", "/navigate_to_pose")],
        condition=IfCondition(explore_lite_active),
    )
    # Coverage + frontier explorer: drives until every reachable free cell is both
    # seen AND approached within its coverage radius. Runs INSTEAD of explore_lite
    # (does not need the /explore/resume kick — it never quits on empty).
    coverage_explorer = Node(
        package="sphero_rvr_driver",
        executable="coverage_explorer",
        name="coverage_explorer",
        output="screen",
        parameters=[coverage_params_file],
        remappings=[("navigate_to_pose", "/navigate_to_pose")],
        condition=IfCondition(coverage_active),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "start_motion_stack",
                default_value="false",
                description=(
                    "MOTOR-CAPABLE: start the collision supervisor and RVR driver. "
                    "Do not enable until tank-SI mapping validation has passed."
                ),
            ),
            DeclareLaunchArgument(
                "start_explore",
                default_value="false",
                description=(
                    "Start autonomous frontier exploration (explore_lite + the "
                    "/explore/resume kick). Default false: bringup is INERT — "
                    "driver, lidar, SLAM, and Nav2 come up but nothing commands "
                    "motion. Send a NavigateToPose goal to drive directionally, "
                    "or set true to explore."
                ),
            ),
            DeclareLaunchArgument(
                "enable_imu_fusion",
                default_value="false",
                description=(
                    "Stage B: stream the RVR IMU and fuse it with wheel odom via "
                    "a robot_localization EKF (removes ~20 deg wheel-only yaw "
                    "drift). The driver yields odom -> base_link to the EKF."
                ),
            ),
            DeclareLaunchArgument(
                "use_decisive_controller",
                default_value="false",
                description=(
                    "Replace the RPP/RotationShim FollowPath controller with the "
                    "decisive controller (drive straight when aligned, arc while "
                    "moving, pivot only for large turns — no slow in-place pivots "
                    "to grind the motors). UNTESTED on hardware; RPP is the default."
                ),
            ),
            DeclareLaunchArgument(
                "use_coverage_explorer",
                default_value="false",
                description=(
                    "With start_explore, run the coverage+frontier explorer instead "
                    "of explore_lite: drives until every reachable free cell is both "
                    "SEEN and APPROACHED within coverage_radius_m (0.75 m). Default "
                    "false = explore_lite (frontier/see-only)."
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
            DeclareLaunchArgument(
                "explore_lite_params_file",
                default_value=str(default_explore_lite_params),
            ),
            DeclareLaunchArgument(
                "coverage_params_file",
                default_value=str(default_coverage_params),
            ),
            supervised,
            lidar,
            mapping,
            *nav2_nodes,
            # Autonomous exploration is OPT-IN (start_explore, default false) so
            # bringup is inert and the rover never moves on its own. When enabled:
            # explore_lite must SUBSCRIBE at startup (a late-started explore misses
            # the latched costmap and hangs on "waiting for costmap"), so it starts
            # with the graph but only drives once /explore/resume is kicked after
            # SLAM + costmaps warm up. (Diagnosed 2026-08-02: without the kick it
            # quits at ~t+1s.)
            explore_lite,
            coverage_explorer,
            # explore_lite quits permanently on ANY empty frontier search — the
            # cold-start race AND transient mid-run empties. Re-kick
            # /explore/resume periodically (every ~15 s) so it always restarts
            # and keeps exploring. Early kicks before warmup are harmless. Only
            # explore_lite needs this (coverage_explorer never quits on empty).
            ExecuteProcess(
                cmd=[
                    "ros2", "topic", "pub", "-r", "0.0667", "/explore/resume",
                    "std_msgs/msg/Bool", "{data: true}",
                ],
                output="screen",
                condition=IfCondition(explore_lite_active),
            ),
        ]
    )
