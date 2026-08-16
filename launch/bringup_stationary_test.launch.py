"""STATIONARY GRAPH TEST — NOT A FLIGHT LAUNCH. The rover cannot move from this.

Purpose: prove the stock middle comes up, wires together, and populates a local costmap
from real floor-level lidar, on the robot's own Pi, with the CHASSIS OFF.

WHY A STATIC odom->base_link IS LEGITIMATE HERE, AND EXACTLY WHERE IT STOPS BEING SO:
with the chassis powered down the rover is parked and not moving, so a constant
odom->base_link IS the truth -- this publishes reality rather than fabricating input.
It becomes fabricated input the moment anything MOTION-SEMANTIC reads it. Therefore:

  PROVEN by this launch          : node bringup, lifecycle activation, topic wiring,
                                   local costmap POPULATING from real /scan returns,
                                   global costmap, planner producing plans, behaviors
                                   reaching the active state with a local costmap to
                                   collision-check against (the D36 refusal cause).
  NOT PROVEN, and never claim it : anything requiring motion -- progress-checker
                                   verdicts, recovery success/failure, RPP path
                                   tracking, goal completion.

THE FROZEN-FRAME TRAP, named so nobody reads a log wrong: with a static TF the robot's
pose never changes, so a progress checker WILL eventually declare no-progress on any goal
that is being pursued, and the BT will fire recoveries. Those recoveries' outcomes are
theatre -- they are artifacts of the frozen frame, not evidence about recoveries. Send
plan-only requests (compute_path_to_pose) rather than navigate_to_pose, or label any
outcome past the first progress timeout as an artifact.

There is a guard test (tests/test_stationary_test_launch_is_not_flyable.py) asserting no
FLIGHT launch references the static publisher this file uses. The camera charter's
un-startable-by-construction pattern: the safe thing is the one that cannot be started by
accident.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

#: The marker every guard test greps for. Its presence in a launch file means "this
#: launch cannot fly" -- it publishes a pose that is only true while the rover is parked.
STATIONARY_TEST_MARKER = "STATIONARY_TEST_STATIC_ODOM"


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory("sphero_rvr_driver"))
    default_nav2_params = share / "config" / "lean_nav2_stock.yaml"

    nav2_params_file = LaunchConfiguration("nav2_params_file")
    start_lidar = LaunchConfiguration("start_lidar")

    args = [
        DeclareLaunchArgument("nav2_params_file", default_value=str(default_nav2_params)),
        DeclareLaunchArgument("start_lidar", default_value="true"),
        # REPLAY MODE. With a recorded bag supplying /tf and /scan, the static publishers
        # must be OFF (the bag's own odom->base_link is the real, moving one) and the
        # clock must come from the bag. Publishing a static odom->base_link on top of a
        # replayed one is not a harmless duplicate -- it is two answers to "where is the
        # robot", which is the seam class this project keeps getting bitten by.
        DeclareLaunchArgument("static_odom", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
    ]
    use_sim_time = LaunchConfiguration("use_sim_time")
    sim_time_param = {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}

    # `condition=None if start_lidar is None else None` -- what this line said before --
    # is always None, so the argument did nothing and the lidar always started. Harmless
    # in live mode and WRONG in replay, where a live scanner would publish /scan on top of
    # the bag's recorded one and the costmap would silently mix a recording with the room
    # it is sitting in. A flag that does nothing is worse than no flag.
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / "launch" / "lidar.launch.py")),
        condition=IfCondition(start_lidar),
    )

    # THE STATIC TF. Named after the marker so a grep for the marker finds the node.
    # odom == base_link: the rover is parked with the chassis off. True while that holds,
    # false the instant anything drives.
    static_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=STATIONARY_TEST_MARKER.lower(),
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "odom", "base_link"],
        condition=IfCondition(LaunchConfiguration("static_odom")),
    )
    # map->odom, likewise: no SLAM in this test, so the global frame is pinned.
    static_map = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=f"{STATIONARY_TEST_MARKER.lower()}_map",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        condition=IfCondition(LaunchConfiguration("static_odom")),
    )

    nav2_nodes = [
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=[nav2_params_file, sim_time_param],
        ),
        # THE POINT OF THE WHOLE EXERCISE: controller_server runs, and with it the local
        # costmap that explore.launch.py:36 documents as absent -- the absence that made
        # Nav2's collision-checked recoveries refuse in 2 ms (D36).
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            parameters=[nav2_params_file, sim_time_param],
            remappings=[("cmd_vel", "/cmd_vel")],
        ),
        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            output="screen",
            parameters=[nav2_params_file, sim_time_param],
            remappings=[("cmd_vel", "/cmd_vel")],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_stationary_test",
            output="screen",
            parameters=[
                nav2_params_file,
                sim_time_param,
                {
                    "autostart": True,
                    "node_names": [
                        "controller_server",
                        "planner_server",
                        "behavior_server",
                    ],
                },
            ],
        ),
    ]

    return LaunchDescription(args + [lidar, static_odom, static_map] + nav2_nodes)
