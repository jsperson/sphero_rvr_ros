from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = Path(get_package_share_directory("sphero_rvr_driver"))
    default_rvr_config = pkg_share / "config" / "rvr.yaml"
    collision_stop_config = pkg_share / "config" / "collision_stop.yaml"
    ekf_config = pkg_share / "config" / "ekf.yaml"

    serial_port = LaunchConfiguration("serial_port")
    rvr_params_file = LaunchConfiguration("rvr_params_file")
    start_supervisor = LaunchConfiguration("start_collision_stop")
    enable_imu_fusion = LaunchConfiguration("enable_imu_fusion")
    imu_x = LaunchConfiguration("imu_x")
    imu_y = LaunchConfiguration("imu_y")
    imu_z = LaunchConfiguration("imu_z")
    imu_yaw = LaunchConfiguration("imu_yaw")

    rvr_node = Node(
        package="sphero_rvr_driver",
        executable="rvr_node",
        name="sphero_rvr_driver",
        output="screen",
        parameters=[
            rvr_params_file,
            {"serial_port": serial_port},
            # Stage B IMU fusion: when on, the driver streams /imu and yields the
            # odom -> base_link TF to the EKF (odom_publish_tf off). When off,
            # these evaluate to the pre-fusion behavior (no /imu, driver owns TF).
            {
                "publish_imu": ParameterValue(
                    PythonExpression(["'", enable_imu_fusion, "' == 'true'"]),
                    value_type=bool,
                ),
                "odom_publish_tf": ParameterValue(
                    PythonExpression(["'", enable_imu_fusion, "' != 'true'"]),
                    value_type=bool,
                ),
            },
        ],
        remappings=[
            ("cmd_vel", "/cmd_vel_motor"),
            ("stop", "/rvr_driver/stop"),
            ("estop", "/rvr_driver/estop"),
            ("clear_estop", "/rvr_driver/clear_estop"),
        ],
        condition=IfCondition(start_supervisor),
    )
    collision_stop_node = Node(
        package="sphero_rvr_driver",
        executable="lidar_collision_stop_supervisor",
        name="lidar_collision_stop_supervisor",
        output="screen",
        # ONE source for this node's parameters. A second entry here shadows the
        # file (ROS 2 launch: the later entry wins), and a shadowed key makes the
        # YAML a lie that nothing can see -- +/-35 sat dead in the file for 23 days
        # behind a +/-45 launch default. tests/test_launch_does_not_shadow_config.py
        # fails if anything is ever layered on top again.
        #
        # A CAPABILITY WAS REMOVED HERE BY DECISION, NOT BY OVERSIGHT. 57e26be
        # (2026-07-23) added launch-time overrides for front_slow_min/max_angle_deg
        # and trajectory_clearance_margin_m on purpose -- "Make attended collision
        # corridor override explicit" -- so someone once wanted to tune the slow
        # corridor at launch. After 2026-08-25 there is no launch argument for it:
        # widening or narrowing the corridor means editing collision_stop.yaml.
        #
        # The evidence that this trade is right: on 2026-08-02, someone who wanted
        # exactly that change edited the YAML (4bb920d, "trim timid brake") rather
        # than passing the launch argument. That is which interface people actually
        # reach for -- and the override's only observable effect across those 23
        # days was to silently defeat them. A knob nobody turns, which quietly
        # overrides the file everybody edits, is worse than no knob.
        parameters=[str(collision_stop_config)],
        remappings=[
            ("cmd_vel", "/cmd_vel"),
            ("cmd_vel_motor", "/cmd_vel_motor"),
            ("stop", "/stop"),
            ("estop", "/estop"),
            ("clear_estop", "/clear_estop"),
        ],
        condition=IfCondition(start_supervisor),
    )

    # Stage B: EKF fusing wheel odom (forward velocity) + RVR IMU (yaw + yaw
    # rate). Sole publisher of odom -> base_link when fusion is enabled.
    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[str(ekf_config)],
        condition=IfCondition(enable_imu_fusion),
    )
    # base_link -> imu_link (RVR IMU pose on the chassis). Default identity;
    # override the offsets once measured. args: x y z yaw pitch roll parent child.
    imu_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_link_to_imu_link",
        arguments=[imu_x, imu_y, imu_z, imu_yaw, "0", "0", "base_link", "imu_link"],
        condition=IfCondition(enable_imu_fusion),
    )

    return LaunchDescription([
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyAMA0"),
        DeclareLaunchArgument(
            "enable_imu_fusion",
            default_value="false",
            description=(
                "Stage B: stream the RVR IMU and fuse it with wheel odom via a "
                "robot_localization EKF. When true the driver yields odom -> "
                "base_link to the EKF. Requires the robot_localization package."
            ),
        ),
        DeclareLaunchArgument("imu_x", default_value="0.0"),
        DeclareLaunchArgument("imu_y", default_value="0.0"),
        DeclareLaunchArgument("imu_z", default_value="0.0"),
        DeclareLaunchArgument("imu_yaw", default_value="0.0"),
        DeclareLaunchArgument(
            "rvr_params_file",
            default_value=str(default_rvr_config),
            description=(
                "RVR driver parameter file. Motor-capable parent launches must "
                "pass their reviewed backend explicitly."
            ),
        ),
        DeclareLaunchArgument(
            "start_collision_stop",
            default_value="true",
            description="Must remain true for motor-capable launches; false starts no driver and is test/development-only.",
        ),
        rvr_node,
        collision_stop_node,
        ekf_node,
        imu_static_tf,
        RegisterEventHandler(
            OnProcessExit(
                target_action=collision_stop_node,
                on_exit=[
                    EmitEvent(
                        event=Shutdown(
                            reason="lidar_collision_stop_supervisor exited; shutting down motor-capable launch"
                        )
                    )
                ],
            ),
            condition=IfCondition(start_supervisor),
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=rvr_node,
                on_exit=[EmitEvent(event=Shutdown(reason="sphero_rvr_driver exited; shutting down supervised launch"))],
            )
        ),
    ])
