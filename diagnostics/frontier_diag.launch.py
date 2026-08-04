"""Bench diagnostic: run the frontier pipeline (lidar -> SLAM -> global costmap)
with NO chassis powered. rvr_node's odom/IMU are replaced by a static, identity
odom->base_link TF (robot "parked at origin"). No controller, no rvr_node, no
motion — safe to run on the bench. Used to investigate open item #1b
("explore: No frontiers found").

Run:  ros2 launch ~/ros2_ws/src/sphero_rvr_ros/diagnostics/frontier_diag.launch.py
Then: python3 ~/ros2_ws/src/sphero_rvr_ros/diagnostics/costmap_analyze.py /map
      python3 .../costmap_analyze.py /global_costmap/costmap

The planner_server (which hosts the global costmap) starts on a TIMER, ~15 s in,
so SLAM has already published /map + map->odom. Starting it cold races SLAM and
the costmap wedges on the default-bounds "sensor out of bounds" at the origin.
NOTE: this spins the lidar. Stop it when done: ros2 service call /stop_motor
std_srvs/srv/Empty, then kill the stack.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("sphero_rvr_driver"))
    lidar_launch = share / "launch" / "lidar.launch.py"
    mapping_launch = share / "launch" / "mapping.launch.py"
    nav2_params = str(share / "config" / "lean_nav2.yaml")
    slam_params = str(share / "config" / "slam_toolbox.yaml")

    planner = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[nav2_params],
    )
    planner_lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_diag",
        output="screen",
        parameters=[{"autostart": True, "node_names": ["planner_server"]}],
    )

    return LaunchDescription(
        [
            # Real lidar (USB-powered off the Pi) -> /scan + base_link->laser TF.
            IncludeLaunchDescription(PythonLaunchDescriptionSource(str(lidar_launch))),
            # Stand in for rvr_node's odometry: static identity odom->base_link.
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="fake_odom_tf",
                arguments=["--frame-id", "odom", "--child-frame-id", "base_link"],
            ),
            # SLAM -> /map + map->odom (real scans, stationary viewpoint).
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(mapping_launch)),
                launch_arguments={
                    "start_rvr": "false",
                    "start_lidar": "false",
                    "start_camera": "false",
                    "start_slam": "true",
                    "slam_autostart": "true",
                    "slam_params_file": slam_params,
                    "use_sim_time": "false",
                }.items(),
            ),
            # Global costmap (in planner_server) starts AFTER SLAM is publishing, so
            # it initializes straight into the /map bounds instead of racing it.
            TimerAction(period=15.0, actions=[planner, planner_lifecycle]),
        ]
    )
