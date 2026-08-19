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
    start_tof = LaunchConfiguration("start_tof")
    start_semantic_map = LaunchConfiguration("start_semantic_map")
    start_vlm_scene = LaunchConfiguration("start_vlm_scene")
    serial_port = LaunchConfiguration("serial_port")
    lidar_serial_port = LaunchConfiguration("lidar_serial_port")
    rvr_params_file = LaunchConfiguration("rvr_params_file")
    slam_params_file = LaunchConfiguration("slam_params_file")
    nav2_params_file = LaunchConfiguration("nav2_params_file")
    explore_lite_params_file = LaunchConfiguration("explore_lite_params_file")
    coverage_params_file = LaunchConfiguration("coverage_params_file")
    mission_autostart = LaunchConfiguration("mission_autostart")

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

    # --- Camera CONSUMERS (all opt-in, all default off) ----------------------
    #
    # Until now no launch file started any of these, so every camera run was
    # assembled by hand -- which is how a run ends up differing from the last one in
    # ways nobody wrote down.
    #
    # The camera DRIVER is deliberately not started here; run camera.launch.py
    # separately. Two reasons, and the second is the load-bearing one:
    #   1. It prepends the pinned ~/.local/rpi-libcamera to LD_LIBRARY_PATH and
    #      AMENT_PREFIX_PATH, which has no business on nav2/slam.
    #   2. Kept separate, the camera SURVIVES a motion-stack restart, so a test that
    #      restarts the stack changes exactly one variable. On 2026-08-08 a restart
    #      destroyed a 34-minute SLAM map and invalidated both arms of an A/B; the
    #      camera staying up is what made the retry cheap.
    # These nodes tolerate the camera starting after them -- they subscribe and wait.
    #
    # THE CAMERA IS NOT A NAVIGATION SENSOR ON THIS ROBOT. Retired 2026-08-16 by
    # Scott's charter: "We want to use the camera to obtain intelligence about the
    # environment (object locations, faces, etc) but it should not be involved in the
    # safety stack or direct navigation."
    #
    # The monocular floor-boundary detector (`low_obstacle` -> /camera/low_obstacles)
    # used to be startable from here. It is gone, and so is its console entry point,
    # so it cannot be launched at all. What it cost while it ran: ~34% of a CPU on a
    # Pi that saturated at load 10.7 on gauntlet mission 1, starving the ToF to
    # 5.4 Hz -- below the rate its own staleness bound is derived from -- while
    # publishing to a topic the brake stopped reading long ago. Shedding it and the
    # camera together took load 10.7 -> 3.1 and the ToF 5.4 -> 6.9 Hz.
    #
    # The camera itself is started on demand by the Track 2 observe path, never here.

    # THE SUB-LIDAR BRAKE'S ACTUAL PRODUCER -> /tof/obstacles.
    #
    # DEFAULTS TRUE, and that is the whole point of putting it here. The supervisor
    # ships with `low_obstacle_brake_enable: true` pointed at `/tof/obstacles`, and a
    # brake with no producer FAILS OPEN silently -- `_apply_low_obstacle_brake`
    # returns the command unchanged when no cloud has ever arrived, which is correct
    # behaviour for a sensor that died and indistinguishable from one never started.
    # Until now this node was not in any launch file, so every ToF run was one
    # forgotten command away from believing it had a brake it did not have.
    #
    # NO PARAMETERS PASSED, deliberately and verified rather than assumed: the node's
    # declared defaults ARE the flying configuration. Checked against the robot's own
    # words in the 2026-08-15b bag -- `/tof/state` reported `stop_distance_m=0.45
    # rules=rule_a+b rule_b=pinned margin_m=0.06`, matching TofConfig's 0.45 / True /
    # 0.06. Gating on the state line rather than on a config file is the D-class
    # lesson from the night rule B was silently OFF with every test green, and
    # `tests/test_tof_launch_wiring.py` pins the agreement.
    tof = Node(
        package="sphero_rvr_driver",
        executable="tof",
        name="tof",
        output="screen",
        condition=IfCondition(start_tof),
    )
    # Map-frame object memory from VLM sightings -> /semantic_map/objects.
    # Costs one cloud VLM call per observation.
    semantic_map = Node(
        package="sphero_rvr_driver",
        executable="semantic_map",
        name="semantic_map",
        output="screen",
        condition=IfCondition(start_semantic_map),
    )
    # On-demand scene description service. Publishes nothing until called, so it is
    # safe to leave running.
    vlm_scene = Node(
        package="sphero_rvr_driver",
        executable="vlm_scene",
        name="vlm_scene",
        output="screen",
        condition=IfCondition(start_vlm_scene),
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
        parameters=[coverage_params_file, {"autostart": mission_autostart}],
        remappings=[("navigate_to_pose", "/navigate_to_pose")],
        condition=IfCondition(coverage_active),
    )
    # THE TOUCH PORT'S PRODUCER, in the launch at last. Both costmaps in
    # lean_nav2_stock.yaml subscribe /contact_marks, the node shipped as an entry
    # point -- and until 2026-08-18 NOTHING LAUNCHED IT (the never-launched-node
    # family; caught in pre-flight, flown by hand twice). Default TRUE because the
    # stock middle without it has no touch response at all: a contact plants no
    # mark and the planner never learns. The bespoke bringup passes false -- its
    # decisive controller carries its own freeze marks. (The old "~14% of a Pi
    # core when idle" claim here did not survive measurement: 0.2% over 60 s,
    # identity-verified PID, 2026-08-19 -- and the two bogus samples before that
    # measured the `ros2 run` wrapper and an ssh carrier, the
    # measure-the-right-population lesson in miniature.)
    contact_marker = Node(
        package="sphero_rvr_driver",
        executable="contact_marker",
        name="contact_marker",
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_contact_marker")),
    )
    # Option D's watcher. DEFAULT TRUE: rig certified all four arms (2026-08-18),
    # the flight ride-along cleared it (d45bd24, same evening), and Scott ratified
    # the flip on the morning of 2026-08-19 after a full walkthrough of
    # docs/watcher_default_decision_2026-08-19.md. Zero motion authority either
    # way; it only requests marks, and only contact_marker grants.
    refusal_watcher = Node(
        package="sphero_rvr_driver",
        executable="refusal_watcher",
        name="refusal_watcher",
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_refusal_watcher")),
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
                "mission_autostart",
                default_value="false",
                description=(
                    "Begin the coverage mission the moment the explorer comes up. "
                    "Default FALSE, and that default is the D29 fix: on 2026-08-10 "
                    "launching WAS liftoff, so a 53 s mission ran and died during "
                    "the bringup gate checks while the operator watched a stopped "
                    "rover with no idea a mission had happened. Leave false and "
                    "call `ros2 service call /coverage_explorer/mission/start "
                    "std_srvs/srv/Trigger` when the gates pass; "
                    "`.../mission/stop` ends it. Set true only for an unattended "
                    "run that genuinely wants motion at launch."
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
            DeclareLaunchArgument(
                "start_tof",
                default_value="true",
                description=(
                    "Start tof: the 8x8 time-of-flight sensor -> /tof/obstacles, "
                    "which IS what the collision brake reads. Defaults TRUE because "
                    "the brake ships enabled and fails OPEN with no producer, so a "
                    "run without this node believes it has a sub-lidar brake and "
                    "does not. Set false only when the I2C sensor is absent."
                ),
            ),
            DeclareLaunchArgument(
                "start_semantic_map",
                default_value="false",
                description=(
                    "Start semantic_map: VLM sightings -> map-frame object memory on "
                    "/semantic_map/objects. Needs a camera stream, TF to map, and a "
                    "Synthetic API key. Costs a cloud call per observation."
                ),
            ),
            DeclareLaunchArgument(
                "start_vlm_scene",
                default_value="false",
                description=(
                    "Start vlm_scene: on-demand describe_scene service. Idle until "
                    "called; one cloud VLM call per invocation."
                ),
            ),
            DeclareLaunchArgument(
                "start_contact_marker",
                default_value="true",
                description=(
                    "The touch port's producer (marks contacts into both costmaps "
                    "via /contact_marks). TRUE by default: the stock middle "
                    "without it has no touch response. The bespoke bringup sets "
                    "false (its controller carries its own freeze marks)."
                ),
            ),
            DeclareLaunchArgument(
                "start_refusal_watcher",
                default_value="true",
                description=(
                    "Option D (refusal-triggered mark promotion). TRUE per "
                    "Scott's ratification, morning of 2026-08-19, after the rig "
                    "campaign, the d45bd24 ride-along, and the certified-mission "
                    "negative test (docs/watcher_default_decision_2026-08-19.md). "
                    "Condition attached, verbatim: 'first default-ON flight is a "
                    "watch item -- every promotion inspected post-flight against "
                    "the room's ground truth; one false promotion reverts the "
                    "default the same day (one launch arg, no code).' The watcher "
                    "has no motion authority; it requests marks from "
                    "contact_marker when the livelock signature sustains."
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
            tof,
            semantic_map,
            vlm_scene,
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
            contact_marker,
            refusal_watcher,
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
