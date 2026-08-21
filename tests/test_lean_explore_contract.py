from __future__ import annotations

import ast
import io
import tokenize
from pathlib import Path

import yaml


REPO_ROOT = Path(__file__).resolve().parents[1]


def _yaml(path: str) -> dict:
    return yaml.safe_load((REPO_ROOT / path).read_text(encoding="utf-8"))


def _strip_comments(source: str) -> str:
    """Source with `#` comments removed. String literals are kept, so a path or a
    node name written in code is still caught."""
    out = []
    reader = io.StringIO(source).readline
    for token in tokenize.generate_tokens(reader):
        if token.type != tokenize.COMMENT:
            out.append(token.string)
    return "\n".join(out)


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
    # 0.35 since the 2026-08-19 linear speed raise (Scott's directive, consensus
    # + derivations in docs/design_linear_speed_raise_2026-08-19.md; the three-
    # gates equality is pinned in tests/test_speed_raise.py).
    assert lean["max_linear_mps"] == 0.35
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
        # Scan CODE, not prose. These names must not be wired into the graph; a
        # comment explaining why one of them is absent is the opposite of a
        # violation, and a guard that punishes documenting its own reason gets
        # satisfied by deleting the explanation.
        assert excluded not in _strip_comments(source)


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

    assert "mapping.launch.py" in (
        REPO_ROOT / "launch/explore.launch.py"
    ).read_text(encoding="utf-8")


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
    # Whitespace-tolerant since 2026-08-18: the subscription call went multi-line
    # when /cmd_vel gained its own callback group (the stop-race fix). Same
    # assertion, same intent -- the supervisor consumes the requested commands.
    import re as _re
    assert _re.search(
        r"self\.create_subscription\(\s*Twist,\s*requested_cmd_topic", collision_source
    )
    assert '"motor_cmd_topic": "/cmd_vel_motor"' in collision_source


def test_lean_explore_surfaces_are_installed() -> None:
    setup = (REPO_ROOT / "setup.py").read_text(encoding="utf-8")
    package = (REPO_ROOT / "package.xml").read_text(encoding="utf-8")

    for path in (
        "launch/explore.launch.py",
        "config/lean_rvr_tank_si.yaml",
        "config/lean_explore_lite.yaml",
        "docs/lean_explore_run_guide.md",
    ):
        assert f'"{path}"' in setup
        assert (REPO_ROOT / path).is_file()
    assert "<exec_depend>nav2_regulated_pure_pursuit_controller</exec_depend>" in (
        package
    )
    assert "<exec_depend>explore_lite</exec_depend>" in package
