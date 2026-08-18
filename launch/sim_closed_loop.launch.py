"""CLOSED-LOOP PROOF AGAINST A SIMULATED CHASSIS. NOT A FLIGHT LAUNCH.

The whole stock middle — planner, controller_server + RPP, behavior_server, bt_navigator,
costmaps — plus our real reflex supervisor and the real `rvr_node`, driving a
curve-faithful model of this drivetrain instead of a serial port. Everything above the
transport is production code; only the wire is simulated.

WHAT THIS RIG IS FOR: the in-place rotation limit cycle. It runs with **no lidar and
therefore an empty costmap**, deliberately. The field aborts in §3a came from RPP's
collision checker, and reproducing those would need a scan stream consistent with a robot
the simulator is moving — which a parked lidar cannot provide, since the room would stay
still while the model drove away from it. **An empty costmap isolates the rotation
question and does not pretend to test collision behaviour.**

WHAT IT CANNOT SHOW, stated so no green result over-claims:
  * arcs (ideal kinematics, unmeasured regime — `docs/run_card_arc_rate_FUTURE.md`)
  * collision/inflation behaviour (no obstacles present at all here -- the supervisor is
    shown a permanently CLEAR scan so it will grant motion; its clamp is exercised, its
    obstacle logic is not)
  * anything about the real floor

`docs/velocity_adapter_design_note.md` holds the pre-registered acceptance criteria.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

from sphero_rvr_driver.rvr_node import SIMULATED_CHASSIS_PORT


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory("sphero_rvr_driver"))
    nav2_params = LaunchConfiguration("nav2_params_file")
    rvr_params = LaunchConfiguration("rvr_params_file")

    args = [
        DeclareLaunchArgument(
            "map_yaml",
            # The global costmap needs SOMETHING to plan in. With no map the planner
            # aborts before the robot moves at all (error 207) and the rig proves
            # nothing -- found the hard way on the first falsifier attempt. A recorded
            # mission map gives free space to plan through; the LOCAL costmap stays
            # empty regardless, since there is no lidar, so collision behaviour is still
            # deliberately out of scope.
            default_value="/home/jsperson/.ros/missions/mission_20260816_171934.yaml",
        ),
        DeclareLaunchArgument(
            "nav2_params_file",
            default_value=str(share / "config" / "lean_nav2_stock.yaml"),
        ),
        DeclareLaunchArgument(
            "rvr_params_file",
            default_value=str(share / "config" / "lean_rvr_tank_si.yaml"),
        ),
        DeclareLaunchArgument(
            "start_refusal_watcher",
            default_value="false",
            description=(
                "Option D's watcher (rig certification runs it; see "
                "docs/design_tof_planner_visibility.md). Default false so the "
                "FALSIFIER arm can prove the disease without the cure present."
            ),
        ),
        DeclareLaunchArgument(
            "map_tf_mode",
            default_value="static",
            description=(
                "static: timeless map->odom (exact-stamp TF lookups cannot fail -- "
                "vacuous for TF-timing code). laggy: sim_laggy_map_tf with SLAM's "
                "measured lag; REQUIRED for falsifying/certifying TF-timing code."
            ),
        ),
    ]

    # map -> odom: identity either way (no SLAM here; odom comes from the REAL
    # rvr_node odometry pipeline fed by the simulator's encoders). The MODE matters:
    #
    #   static (default)  tf2 treats a static transform as TIMELESS -- valid at every
    #                     stamp ever asked for -- so exact-stamp TF lookups are
    #                     UNFALSIFIABLE under it. Fine for rotation/costmap work;
    #                     VACUOUS for certifying any TF-timing code. contact_marker
    #                     v1's lookup-at-stamp shipped rig-green exactly this way and
    #                     lost 3/3 field contacts on 2026-08-18.
    #   laggy             sim_laggy_map_tf publishes the same identity with SLAM's
    #                     measured lag and stamp gaps. TF-timing code MUST be
    #                     falsified and certified in this mode.
    map_tf_mode = LaunchConfiguration("map_tf_mode")
    static_map = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="sim_static_map_to_odom",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        condition=UnlessCondition(
            PythonExpression(["'", map_tf_mode, "' == 'laggy'"])
        ),
    )
    laggy_map = Node(
        package="sphero_rvr_driver",
        executable="sim_laggy_map_tf",
        name="sim_laggy_map_tf",
        output="screen",
        condition=IfCondition(PythonExpression(["'", map_tf_mode, "' == 'laggy'"])),
    )
    refusal_watcher = Node(
        package="sphero_rvr_driver",
        executable="refusal_watcher",
        name="refusal_watcher",
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_refusal_watcher")),
    )
    # The certifier scene needs marks; the falsifier must run without the watcher.
    rig_contact_marker = Node(
        package="sphero_rvr_driver",
        executable="contact_marker",
        name="contact_marker",
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_refusal_watcher")),
    )
    static_laser = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="sim_static_base_to_laser",
        output="screen",
        # THE REAL MEASURED MOUNT, from lidar.launch.py -- including the ~179 deg yaw
        # that makes raw scan angle 0 point BEHIND the robot. A sim that used
        # identity here would exercise a robot we do not own.
        arguments=[
            "--x", "0.004500", "--y", "-0.011000", "--z", "0.190500",
            "--roll", "0.0", "--pitch", "0.0", "--yaw", "3.1239668018215028",
            "--frame-id", "base_link", "--child-frame-id", "laser",
        ],
    )

    driver = Node(
        package="sphero_rvr_driver",
        executable="rvr_node",
        name="sphero_rvr_driver",
        output="screen",
        parameters=[rvr_params, {"serial_port": SIMULATED_CHASSIS_PORT}],
        remappings=[("cmd_vel", "/cmd_vel_motor")],
    )
    supervisor = Node(
        package="sphero_rvr_driver",
        executable="lidar_collision_stop_supervisor",
        name="lidar_collision_stop_supervisor",
        output="screen",
        parameters=[str(share / "config" / "collision_stop.yaml")],
        remappings=[("cmd_vel", "/cmd_vel"), ("cmd_vel_motor", "/cmd_vel_motor")],
    )

    # The supervisor refuses everything without a scan (SENSOR_STALE / missing_scan) --
    # correctly, since moving blind is what it exists to prevent. With no lidar in this
    # rig that refusal zeroes /cmd_vel_motor and the loop never closes. An all-clear scan
    # keeps the supervisor's CLAMP in the path (the point) while its obstacle logic sees
    # an empty world (out of scope, and stated as such).
    # RAYCAST against the real recorded room, not an all-clear scan. The first falsifier
    # used all-clear and the BROKEN config passed everything -- with nothing to collide
    # with, RPP just arcs to the goal and never reverses, so the hunt cannot occur. The
    # beam pose comes from TF (map -> laser), so the real ~179 deg mount rotation is
    # inherited rather than reimplemented.
    scan = Node(
        package="sphero_rvr_driver",
        executable="sim_raycast_scan",
        name="sim_raycast_scan",
        output="screen",
        parameters=[{
            "i_understand_this_publishes_a_simulated_scan": True,
            "map_yaml": LaunchConfiguration("map_yaml"),
            "beams": 720,
            "rate_hz": 10.0,
        }],
    )

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[{"yaml_filename": LaunchConfiguration("map_yaml")}],
    )

    nav2 = [
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=[nav2_params],
        ),
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            parameters=[nav2_params],
            remappings=[("cmd_vel", "/cmd_vel")],
        ),
        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            output="screen",
            parameters=[nav2_params],
            remappings=[("cmd_vel", "/cmd_vel")],
        ),
        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            output="screen",
            parameters=[nav2_params],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_explore",
            output="screen",
            parameters=[
                nav2_params,
                # map_server is lifecycle-managed here too: launched unmanaged it sits
                # `unconfigured`, silent, and looking healthy in `ros2 node list` -- the
                # same costume slam_toolbox wears, and the third time this pattern has
                # cost a bringup.
                {"node_names": [
                    "map_server",
                    "controller_server",
                    "planner_server",
                    "behavior_server",
                    "bt_navigator",
                ]},
            ],
        ),
    ]

    return LaunchDescription(
        args
        + [static_map, laggy_map, refusal_watcher, rig_contact_marker,
           static_laser, scan, driver, supervisor, map_server]
        + nav2
    )
