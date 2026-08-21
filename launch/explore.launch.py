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
    default_coverage_params = share / "config" / "coverage_explorer.yaml"
    standard_nav_to_pose_bt = (
        nav2_bt_share
        / "behavior_trees"
        / "navigate_to_pose_w_replanning_and_recovery.xml"
    )
    # Batch (a): the standard tree with ONE attribute changed -- Spin retargeted at
    # the supervisor's precise-turn gateway (firmware heading loop). Selected only
    # by use_precise_turn_spin; see the XML's own header for the provenance.
    precise_turn_nav_to_pose_bt = (
        share / "behavior_trees" / "navigate_to_pose_stock_precise_turn.xml"
    )

    start_motion_stack = LaunchConfiguration("start_motion_stack")
    start_explore = LaunchConfiguration("start_explore")
    enable_imu_fusion = LaunchConfiguration("enable_imu_fusion")
    use_precise_turn_spin = LaunchConfiguration("use_precise_turn_spin")
    use_coverage_explorer = LaunchConfiguration("use_coverage_explorer")
    start_tof = LaunchConfiguration("start_tof")
    start_semantic_map = LaunchConfiguration("start_semantic_map")
    start_vlm_scene = LaunchConfiguration("start_vlm_scene")
    serial_port = LaunchConfiguration("serial_port")
    lidar_serial_port = LaunchConfiguration("lidar_serial_port")
    rvr_params_file = LaunchConfiguration("rvr_params_file")
    slam_params_file = LaunchConfiguration("slam_params_file")
    nav2_params_file = LaunchConfiguration("nav2_params_file")
    coverage_params_file = LaunchConfiguration("coverage_params_file")
    mission_autostart = LaunchConfiguration("mission_autostart")

    # ONE explorer: the coverage explorer. (explore_lite and its selection
    # branch retired 2026-08-21 with Scott's word — superseded since 08-14 and
    # never flown since. use_coverage_explorer stays as an arg so existing
    # commands parse, but both values now mean the coverage explorer.)
    coverage_active = PythonExpression(
        ["'", start_explore, "' == 'true'"]
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

    # Pick the navigate-to-pose BT: the stock tree, or -- when
    # use_precise_turn_spin is true -- the stock tree with Spin retargeted at the
    # supervisor's precise-turn gateway. (The decisive-mode branch and its
    # global-costmap BT died with the bespoke controller, 2026-08-21 project
    # review -- five stock flights never reached it.)
    nav_to_pose_bt_xml = ParameterValue(
        PythonExpression(
            [
                "'",
                str(precise_turn_nav_to_pose_bt),
                "' if '",
                use_precise_turn_spin,
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
        # The controller: Nav2's RPP/RotationShim FollowPath.
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
        # Lifecycle manager (manages controller_server too; the decisive-mode
        # variant that could not died with the bespoke controller, 2026-08-21).
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_explore",
            output="screen",
            parameters=[nav2_params_file],
        ),
    ]
    # Coverage + frontier explorer: drives until every reachable free cell is both
    # seen AND approached within its coverage radius.
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
    # mark and the planner never learns. (The bespoke bringup that passed false
    # died 2026-08-21; its controller carried its own freeze marks.) (The old "~14% of a Pi
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
                    "Start the coverage explorer. Default false: bringup is "
                    "INERT — driver, lidar, SLAM, and Nav2 come up but nothing "
                    "commands motion. Send a NavigateToPose goal to drive "
                    "directionally, or set true to explore."
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
                "use_coverage_explorer",
                default_value="false",
                description=(
                    "VESTIGIAL since 2026-08-21 (explore_lite retired): the "
                    "coverage explorer is the only explorer, and start_explore "
                    "alone decides. Kept so existing commands parse; both values "
                    "behave identically."
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
                "use_precise_turn_spin",
                default_value="true",
                description=(
                    "Route the BT's Spin recovery through the supervisor's "
                    "precise-turn gateway (/collision_stop/precise_turn, the "
                    "firmware heading loop that manages torque as resistance "
                    "demands) instead of behavior_server's open-loop spin -- "
                    "the path that stalled three times on 2026-08-19. TRUE "
                    "per the (viii)-(ix) sitting, 2026-08-19 afternoon "
                    "(docs/bench_card_2026-08-19.md RESULTS): (viii) PASS -- "
                    "'precise turn SETTLED: target 150.4 deg, heading 151.7 "
                    "deg, err -1.3 deg, 1.0 s', a firmware 90 from rest on "
                    "the scrub that protect-trips the velocity family, with "
                    "behavior_server's spin silent; (ix) satisfied by live "
                    "event (admission refusal fell through loudly to "
                    "Wait/BackUp, PM-ruled); items (i)-(iii) passed "
                    "2026-08-18 night (f4c840a). Condition attached, "
                    "verbatim: 'first default-ON mission is a watch item -- "
                    "every gateway turn inspected post-flight against the "
                    "event lane and the bag; one wrong turn (fired when "
                    "inadmissible, or settled falsely) reverts the default "
                    "same-day (one launch arg, no code).'"
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
            # NO DEFAULT, deliberately (bespoke deletion, 2026-08-21): which
            # costmap config flies is a decision, and the old implicit default
            # (the bespoke-era lean_nav2.yaml) was a loaded footgun — a
            # hand-typed launch would silently run bespoke costmaps (no touch,
            # no tof) under the stock middle. The launch now REFUSES to come up
            # without an explicit choice; launch_and_arm.py passes the stock
            # file, as it always did.
            DeclareLaunchArgument(
                "nav2_params_file",
                description=("REQUIRED: absolute path to the Nav2 params file "
                             "(the deployed stock config is "
                             "lean_nav2_stock.yaml; no implicit default)"),
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
            # bringup is inert and the rover never moves on its own.
            coverage_explorer,
            contact_marker,
            refusal_watcher,
        ]
    )
