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
  * collision/inflation behaviour (no obstacles present at all here)
  * anything about the real floor

`docs/velocity_adapter_design_note.md` holds the pre-registered acceptance criteria.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from sphero_rvr_driver.rvr_node import SIMULATED_CHASSIS_PORT


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory("sphero_rvr_driver"))
    nav2_params = LaunchConfiguration("nav2_params_file")
    rvr_params = LaunchConfiguration("rvr_params_file")

    args = [
        DeclareLaunchArgument(
            "nav2_params_file",
            default_value=str(share / "config" / "lean_nav2_stock.yaml"),
        ),
        DeclareLaunchArgument(
            "rvr_params_file",
            default_value=str(share / "config" / "lean_rvr_tank_si.yaml"),
        ),
    ]

    # map -> odom pinned: no SLAM here, and odom itself comes from the REAL rvr_node
    # odometry pipeline fed by the simulator's encoders.
    static_map = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="sim_static_map_to_odom",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
    )
    static_laser = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="sim_static_base_to_laser",
        output="screen",
        arguments=["0", "0", "0.19", "0", "0", "0", "base_link", "laser"],
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
            parameters=[nav2_params],
        ),
    ]

    return LaunchDescription(args + [static_map, static_laser, driver, supervisor] + nav2)
