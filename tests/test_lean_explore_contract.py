from __future__ import annotations

import ast
from pathlib import Path

import yaml


REPO_ROOT = Path(__file__).resolve().parents[1]


def _yaml(path: str) -> dict:
    return yaml.safe_load((REPO_ROOT / path).read_text(encoding="utf-8"))


def _node_calls(path: str) -> list[ast.Call]:
    module = ast.parse((REPO_ROOT / path).read_text(encoding="utf-8"))
    return [
        node
        for node in ast.walk(module)
        if isinstance(node, ast.Call)
        and getattr(node.func, "id", None) == "Node"
    ]


def _constant_keyword(call: ast.Call, name: str) -> str | None:
    for keyword in call.keywords:
        if keyword.arg == name and isinstance(keyword.value, ast.Constant):
            return str(keyword.value.value)
    return None


def _remappings(call: ast.Call) -> list[tuple[str, str]]:
    for keyword in call.keywords:
        if keyword.arg != "remappings" or not isinstance(keyword.value, ast.List):
            continue
        result = []
        for item in keyword.value.elts:
            if not isinstance(item, ast.Tuple) or len(item.elts) != 2:
                continue
            if all(isinstance(value, ast.Constant) for value in item.elts):
                result.append((str(item.elts[0].value), str(item.elts[1].value)))
        return result
    return []


def test_lean_driver_uses_native_tank_si_without_changing_deployed_rvr_config() -> None:
    deployed = _yaml("config/rvr.yaml")["sphero_rvr_driver"]["ros__parameters"]
    lean_path = REPO_ROOT / "config/lean_rvr_tank_si.yaml"
    lean = _yaml("config/lean_rvr_tank_si.yaml")["sphero_rvr_driver"]["ros__parameters"]

    assert deployed["velocity_control_mode"] == "raw_motor"
    assert lean["velocity_control_mode"] == "native_tank_si"
    assert lean["max_linear_mps"] == 0.12
    assert "native_rc_si" not in lean_path.read_text(encoding="utf-8")
    for name, expected in {
        "max_angular_rad_s": 0.4,
        "odom_counts_per_meter": 4337.768,
        "odom_wheel_track_m": 0.2507,
        "odom_frame_id": "odom",
        "base_frame_id": "base_link",
        "odom_publish_tf": True,
    }.items():
        assert lean[name] == expected
        assert lean[name] == deployed[name]


def test_supervised_launch_accepts_and_loads_an_explicit_rvr_parameter_file() -> None:
    source = (REPO_ROOT / "launch/supervised_rvr.launch.py").read_text(
        encoding="utf-8"
    )

    assert 'rvr_params_file = LaunchConfiguration("rvr_params_file")' in source
    # The driver still loads the explicit params file first (its reviewed backend);
    # the Stage B IMU-fusion overrides are layered after it, not instead of it.
    assert "rvr_params_file," in source
    assert '{"serial_port": serial_port}' in source
    assert '"rvr_params_file",' in source
    assert 'default_value=str(default_rvr_config)' in source


def test_explore_launch_is_the_minimal_supervised_composition() -> None:
    source = (REPO_ROOT / "launch/explore.launch.py").read_text(encoding="utf-8")

    ast.parse(source)
    assert "supervised_rvr.launch.py" in source
    assert '"start_range_motion": "false"' in source
    assert '"start_live_route_runner": "false"' in source
    assert '"rvr_params_file": rvr_params_file' in source
    assert "lean_rvr_tank_si.yaml" in source
    assert "lidar.launch.py" in source
    assert "mapping.launch.py" in source
    assert '"start_rvr": "false"' in source
    assert '"start_lidar": "false"' in source
    assert '"start_camera": "false"' in source
    assert '"start_slam": "true"' in source
    assert '"slam_autostart": "true"' in source
    assert "navigate_to_pose_w_replanning_and_recovery.xml" in source
    assert 'package="explore_lite"' in source
    assert 'executable="explore"' in source
    assert 'name="explore_node"' in source
    assert 'remappings=[("navigate_to_pose", "/navigate_to_pose")]' in source
    assert "lean_explore_lite.yaml" in source

    for excluded in (
        "mission_service",
        "hierarchical_mission",
        "hierarchical_nav2_adapter",
        "live_route_runner_node",
        "adaptive_mission",
        "camera.launch.py",
        "codex_app_server",
        "rolling_",
    ):
        assert excluded not in source


def test_explore_lite_dependency_and_small_room_parameters_are_pinned() -> None:
    repos = _yaml("workspace.repos")["repositories"]
    dependency = repos["m_explore_ros2"]
    params = _yaml("config/lean_explore_lite.yaml")["explore_node"][
        "ros__parameters"
    ]

    assert dependency == {
        "type": "git",
        "url": "https://github.com/robo-friends/m-explore-ros2.git",
        "version": "326cf8a0b487c34246bb8f3326afbcd69576dc60",
    }
    assert params == {
        "use_sim_time": False,
        "robot_base_frame": "base_link",
        "return_to_init": False,
        "costmap_topic": "/global_costmap/costmap",
        "costmap_updates_topic": "/global_costmap/costmap_updates",
        "visualize": True,
        "planner_frequency": 0.1,
        "progress_timeout": 90.0,
        "potential_scale": 3.0,
        "orientation_scale": 0.2,
        "gain_scale": 1.0,
        "transform_tolerance": 0.3,
        "min_frontier_size": 0.2,
    }

    nav2 = _yaml("config/lean_nav2.yaml")
    assert nav2["global_costmap"]["global_costmap"]["ros__parameters"][
        "static_layer"
    ]["plugin"] == "nav2_costmap_2d::StaticLayer"
    assert "mapping.launch.py" in (
        REPO_ROOT / "launch/explore.launch.py"
    ).read_text(encoding="utf-8")


def test_lean_nav2_has_explicit_unstamped_bounded_single_goal_contracts() -> None:
    nav2 = _yaml("config/lean_nav2.yaml")
    controller = nav2["controller_server"]["ros__parameters"]
    follow = controller["FollowPath"]
    behavior = nav2["behavior_server"]["ros__parameters"]
    bt = nav2["bt_navigator"]["ros__parameters"]

    assert controller["enable_stamped_cmd_vel"] is False
    assert behavior["enable_stamped_cmd_vel"] is False
    assert follow["plugin"] == (
        "nav2_rotation_shim_controller::RotationShimController"
    )
    assert follow["primary_controller"] == (
        "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
    )
    assert follow["desired_linear_vel"] == 0.12
    assert follow["rotate_to_heading_angular_vel"] == 0.4
    assert follow["min_approach_linear_velocity"] == 0.05
    assert follow["use_regulated_linear_velocity_scaling"] is False
    assert follow["use_cost_regulated_linear_velocity_scaling"] is False
    assert follow["allow_reversing"] is False
    assert nav2["planner_server"]["ros__parameters"]["GridBased"]["plugin"] == (
        "nav2_smac_planner::SmacPlanner2D"
    )
    assert bt["navigators"] == ["navigate_to_pose"]
    assert bt["navigate_to_pose"]["plugin"] == (
        "nav2_bt_navigator::NavigateToPoseNavigator"
    )

    text = (REPO_ROOT / "config/lean_nav2.yaml").read_text(encoding="utf-8")
    assert "DWBLocalPlanner" not in text
    assert "velocity_smoother" not in text
    assert "deadband" not in text.lower()


def test_lean_nav2_terminal_contract_counts_rotation_without_a_heading_gap() -> None:
    nav2 = _yaml("config/lean_nav2.yaml")
    controller = nav2["controller_server"]["ros__parameters"]
    progress = controller["progress_checker"]
    goal = controller["goal_checker"]
    follow = controller["FollowPath"]

    assert progress == {
        "plugin": "nav2_controller::PoseProgressChecker",
        "required_movement_radius": 0.02,
        "required_movement_angle": 0.10,
        "movement_time_allowance": 30.0,
    }
    assert goal["xy_goal_tolerance"] == 0.10
    assert goal["yaw_goal_tolerance"] == 0.35
    assert follow["rotate_to_heading_min_angle"] <= goal["yaw_goal_tolerance"]

    # Terminal acceptance is independent of cruise speed; the angular ceiling
    # remains fixed while linear cruise was raised to 0.12 m/s (wide-gap tuning).
    assert follow["desired_linear_vel"] == 0.12
    assert follow["rotate_to_heading_angular_vel"] == 0.4


def test_explore_graph_keeps_supervisor_as_sole_motor_command_publisher() -> None:
    """ROS-free ownership proof for every motor-capable node in explore.launch."""

    explore_source = (REPO_ROOT / "launch/explore.launch.py").read_text(
        encoding="utf-8"
    )
    supervised_source = (
        REPO_ROOT / "launch/supervised_rvr.launch.py"
    ).read_text(encoding="utf-8")
    collision_source = (
        REPO_ROOT / "src/sphero_rvr_driver/collision_stop_node.py"
    ).read_text(encoding="utf-8")

    nav2_motion_nodes = {
        _constant_keyword(call, "executable"): _remappings(call)
        for call in _node_calls("launch/explore.launch.py")
        if _constant_keyword(call, "executable")
        in {"controller_server", "behavior_server"}
    }
    assert nav2_motion_nodes == {
        "controller_server": [("cmd_vel", "/cmd_vel")],
        "behavior_server": [("cmd_vel", "/cmd_vel")],
    }
    assert "/cmd_vel_motor" not in explore_source

    # In the included supervised graph, the driver consumes the motor topic and
    # the collision supervisor alone publishes it.
    assert '("cmd_vel", "/cmd_vel_motor")' in supervised_source
    assert '("cmd_vel_motor", "/cmd_vel_motor")' in supervised_source
    assert "self._cmd_pub = self.create_publisher(Twist, motor_cmd_topic, 10)" in (
        collision_source
    )
    assert "self.create_subscription(Twist, requested_cmd_topic" in collision_source
    assert '"motor_cmd_topic": "/cmd_vel_motor"' in collision_source


def test_lean_explore_surfaces_are_installed() -> None:
    setup = (REPO_ROOT / "setup.py").read_text(encoding="utf-8")
    package = (REPO_ROOT / "package.xml").read_text(encoding="utf-8")

    for path in (
        "launch/explore.launch.py",
        "config/lean_rvr_tank_si.yaml",
        "config/lean_nav2.yaml",
        "config/lean_explore_lite.yaml",
        "docs/lean_explore_run_guide.md",
    ):
        assert f'"{path}"' in setup
        assert (REPO_ROOT / path).is_file()
    assert "<exec_depend>nav2_regulated_pure_pursuit_controller</exec_depend>" in (
        package
    )
    assert "<exec_depend>explore_lite</exec_depend>" in package
