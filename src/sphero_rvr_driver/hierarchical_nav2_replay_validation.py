"""Attended, replay-only validation for the Phase 1 Nav2 command chain."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import platform
import re
import shutil
import subprocess
import time


INITIAL_X_M = 0.12
INITIAL_Y_M = -0.26
INITIAL_QZ = 0.62932039
INITIAL_QW = 0.77714596
GOALS = (
    (0.22, 0.20, 0.49242356, 0.87035570),
    (0.40, 0.50, 0.0, 1.0),
    (0.60, 0.50, 0.0, 1.0),
)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Validate Phase 1 in a launch-owned loopback graph."
    )
    parser.add_argument(
        "--mode",
        choices=("audit", "handoff", "veto"),
        default="handoff",
    )
    parser.add_argument("--timeout", type=float, default=90.0)
    parser.add_argument("--source-sha")
    parser.add_argument("--observe-seconds", type=float, default=2.0)
    return parser


SOURCE_SHA_PATTERN = re.compile(r"^[0-9a-f]{40}$")
PROHIBITED_EXECUTABLES = {
    "rvr_node",
    "stationary_perception",
    "rplidar_composition",
    "rplidar_node",
    "sllidar",
    "sllidar_node",
    "camera_node",
    "rvr-camera-node",
    "slam_toolbox",
    "async_slam_toolbox_node",
    "sync_slam_toolbox_node",
}
SERIAL_DEVICE_CANDIDATES = (
    "/dev/ttyAMA0",
    "/dev/ttyUSB0",
    "/dev/rvr",
)


def _validated_source_sha(value: str | None) -> str:
    supplied = str(value or "").strip()
    if not SOURCE_SHA_PATTERN.fullmatch(supplied):
        raise ValueError(
            "audit mode requires --source-sha with exactly 40 lowercase hex characters"
        )
    return supplied


def _prohibited_processes(process_text: str) -> list[str]:
    prohibited = []
    for raw_line in str(process_text).splitlines():
        line = raw_line.strip()
        if not line:
            continue
        fields = line.split()
        executable_tokens = {
            Path(token).name.lower()
            for token in fields[1:]
            if not token.startswith("-")
        }
        if executable_tokens & PROHIBITED_EXECUTABLES:
            prohibited.append(line)
    return sorted(prohibited)


def _serial_device_owners(
    *,
    paths=SERIAL_DEVICE_CANDIDATES,
    runner=subprocess.run,
) -> dict[str, list[int]]:
    if shutil.which("fuser") is None:
        raise RuntimeError("fuser is required for serial-owner verification")
    owners: dict[str, list[int]] = {}
    for raw_path in paths:
        path = Path(raw_path)
        if not path.exists():
            continue
        result = runner(
            ["fuser", str(path)],
            capture_output=True,
            text=True,
            check=False,
        )
        if result.returncode not in (0, 1):
            detail = result.stderr.strip() or "unknown fuser error"
            raise RuntimeError(
                f"serial-owner inspection failed for {path}: {detail}"
            )
        owners[str(path)] = sorted(
            {
                int(token)
                for token in result.stdout.split()
                if token.isdigit()
            }
        )
    return owners


def _host_safety_snapshot() -> dict:
    process_result = subprocess.run(
        ["ps", "-eo", "pid=,comm=,args="],
        capture_output=True,
        text=True,
        check=True,
    )
    serial_owners = _serial_device_owners()
    prohibited = _prohibited_processes(process_result.stdout)
    return {
        "hostname": platform.node(),
        "architecture": platform.machine(),
        "prohibited_processes": prohibited,
        "serial_device_owners": serial_owners,
        "passed": not prohibited
        and all(not values for values in serial_owners.values()),
    }


def main(argv=None) -> int:
    args = _parser().parse_args(argv)

    import rclpy
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import PoseStamped, Twist
    from lifecycle_msgs.srv import GetState
    from nav2_msgs.action import NavigateThroughPoses
    from nav_msgs.msg import Odometry
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from std_srvs.srv import Trigger

    class ReplayValidator(Node):
        def __init__(self) -> None:
            super().__init__("hierarchical_nav2_replay_validator")
            self.action = ActionClient(
                self,
                NavigateThroughPoses,
                "/navigate_through_poses",
            )
            self.stop = self.create_client(Trigger, "/stop")
            self.create_subscription(Odometry, "/odom", self._on_odom, 10)
            self.create_subscription(Twist, "/cmd_vel_motor", self._on_motor, 10)
            self.map_xy = (INITIAL_X_M, INITIAL_Y_M)
            self.min_waypoint_distance = [math.inf, math.inf]
            self.waypoint_samples = [0, 0]
            self.waypoint_zero_samples = [0, 0]
            self.waypoint_zero_run_started_at = [None, None]
            self.waypoint_max_zero_duration_s = [0.0, 0.0]
            self.motor_samples = 0
            self.nonzero_motor_samples = 0
            self.max_linear_mps = 0.0
            self.max_angular_rad_s = 0.0
            self.motion_started_at = None
            self.veto_requested_at = None
            self.first_zero_after_veto_at = None
            self.lifecycle_clients = {
                name: self.create_client(GetState, f"/{name}/get_state")
                for name in (
                    "map_server",
                    "planner_server",
                    "controller_server",
                    "behavior_server",
                    "bt_navigator",
                )
            }

        def _on_odom(self, msg) -> None:
            yaw = 2.0 * math.atan2(INITIAL_QZ, INITIAL_QW)
            odom_x = float(msg.pose.pose.position.x)
            odom_y = float(msg.pose.pose.position.y)
            map_x = INITIAL_X_M + math.cos(yaw) * odom_x - math.sin(yaw) * odom_y
            map_y = INITIAL_Y_M + math.sin(yaw) * odom_x + math.cos(yaw) * odom_y
            self.map_xy = (map_x, map_y)
            for index, (goal_x, goal_y, _, _) in enumerate(GOALS[:2]):
                distance = math.hypot(map_x - goal_x, map_y - goal_y)
                self.min_waypoint_distance[index] = min(
                    self.min_waypoint_distance[index],
                    distance,
                )

        def _on_motor(self, msg) -> None:
            now = time.monotonic()
            linear = float(msg.linear.x)
            angular = float(msg.angular.z)
            nonzero = abs(linear) + abs(angular) > 1e-5
            self.motor_samples += 1
            self.max_linear_mps = max(self.max_linear_mps, abs(linear))
            self.max_angular_rad_s = max(self.max_angular_rad_s, abs(angular))
            if nonzero:
                self.nonzero_motor_samples += 1
                if self.motion_started_at is None:
                    self.motion_started_at = now
            if self.veto_requested_at is not None and not nonzero:
                if self.first_zero_after_veto_at is None:
                    self.first_zero_after_veto_at = now
            if self.motion_started_at is None:
                return
            map_x, map_y = self.map_xy
            for index, (goal_x, goal_y, _, _) in enumerate(GOALS[:2]):
                in_window = (
                    math.hypot(map_x - goal_x, map_y - goal_y) <= 0.15
                )
                if in_window:
                    self.waypoint_samples[index] += 1
                    if not nonzero:
                        self.waypoint_zero_samples[index] += 1
                        if self.waypoint_zero_run_started_at[index] is None:
                            self.waypoint_zero_run_started_at[index] = now
                    elif self.waypoint_zero_run_started_at[index] is not None:
                        self.waypoint_max_zero_duration_s[index] = max(
                            self.waypoint_max_zero_duration_s[index],
                            now - self.waypoint_zero_run_started_at[index],
                        )
                        self.waypoint_zero_run_started_at[index] = None
                elif self.waypoint_zero_run_started_at[index] is not None:
                    self.waypoint_max_zero_duration_s[index] = max(
                        self.waypoint_max_zero_duration_s[index],
                        now - self.waypoint_zero_run_started_at[index],
                    )
                    self.waypoint_zero_run_started_at[index] = None

        def graph(self) -> dict:
            nodes = sorted(
                f"{namespace.rstrip('/')}/{name}"
                for name, namespace in self.get_node_names_and_namespaces()
            )

            def endpoints(topic: str, publishers: bool) -> list[str]:
                getter = (
                    self.get_publishers_info_by_topic
                    if publishers
                    else self.get_subscriptions_info_by_topic
                )
                discovered = sorted(
                    f"{item.node_namespace.rstrip('/')}/{item.node_name}"
                    for item in getter(topic)
                )
                return [
                    value
                    for value in discovered
                    if value != "/hierarchical_nav2_replay_validator"
                ]

            return {
                "nodes": nodes,
                "private_publishers": endpoints(
                    "/nav2_cmd_vel_request",
                    True,
                ),
                "private_subscribers": endpoints(
                    "/nav2_cmd_vel_request",
                    False,
                ),
                "cmd_vel_publishers": endpoints("/cmd_vel", True),
                "cmd_vel_subscribers": endpoints("/cmd_vel", False),
                "motor_publishers": endpoints("/cmd_vel_motor", True),
                "motor_subscribers": endpoints("/cmd_vel_motor", False),
            }

    def spin_until(node, predicate, timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
            if predicate():
                return True
        return False

    rclpy.init()
    node = ReplayValidator()
    goal_handle = None
    summary = {}
    try:
        source_sha = (
            _validated_source_sha(args.source_sha)
            if args.mode == "audit"
            else args.source_sha
        )
        if args.observe_seconds <= 0.0:
            raise ValueError("--observe-seconds must be positive")

        def graph_is_complete() -> bool:
            discovered = node.graph()
            return (
                set(discovered["private_publishers"])
                == {"/behavior_server", "/controller_server"}
                and discovered["private_subscribers"] == ["/live_route_runner"]
                and discovered["cmd_vel_publishers"] == ["/live_route_runner"]
                and discovered["motor_subscribers"] == ["/loopback_simulator"]
            )

        spin_until(node, graph_is_complete, 10.0)
        graph = node.graph()
        node_names = {value.rsplit("/", 1)[-1] for value in graph["nodes"]}
        graph_ok = (
            "rvr_node" not in node_names
            and set(graph["private_publishers"])
            == {"/behavior_server", "/controller_server"}
            and graph["private_subscribers"] == ["/live_route_runner"]
            and graph["cmd_vel_publishers"] == ["/live_route_runner"]
            and graph["cmd_vel_subscribers"]
            == ["/lidar_collision_stop_supervisor"]
            and graph["motor_publishers"]
            == ["/lidar_collision_stop_supervisor"]
            and graph["motor_subscribers"] == ["/loopback_simulator"]
        )
        if not node.action.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("NavigateThroughPoses action server unavailable")

        lifecycle_states = {}
        for name, client in node.lifecycle_clients.items():
            if not client.wait_for_service(timeout_sec=5.0):
                raise RuntimeError(f"{name} lifecycle service unavailable")
            future = client.call_async(GetState.Request())
            if not spin_until(node, future.done, 5.0):
                raise RuntimeError(f"{name} lifecycle query timed out")
            response = future.result()
            lifecycle_states[name] = {
                "id": int(response.current_state.id),
                "label": str(response.current_state.label),
            }

        if args.mode == "audit":
            spin_until(node, lambda: False, args.observe_seconds)
            host_safety = _host_safety_snapshot()
            lifecycle_ok = all(
                state["label"] == "active"
                for state in lifecycle_states.values()
            )
            audit_ok = (
                graph_ok
                and lifecycle_ok
                and host_safety["passed"]
                and node.nonzero_motor_samples == 0
            )
            summary = {
                "schema": "sphero_rvr.m7_phase1_graph_audit.v1",
                "recorded_at_utc": time.strftime(
                    "%Y-%m-%dT%H:%M:%SZ", time.gmtime()
                ),
                "mode": args.mode,
                "source_sha": source_sha,
                "passed": audit_ok,
                "motion_authority": False,
                "physical_execution_enabled": False,
                "live_sensors_started": False,
                "serial_transport_started": False,
                "driver_started": False,
                "graph_ok": graph_ok,
                "graph": graph,
                "navigate_through_poses_available": True,
                "lifecycle_states": lifecycle_states,
                "observation_seconds": args.observe_seconds,
                "motor_samples": node.motor_samples,
                "nonzero_motor_samples": node.nonzero_motor_samples,
                "host_safety": host_safety,
                "hardware_sink_present": False,
                "simulation_sink": "/loopback_simulator",
            }
            print(json.dumps(summary, indent=2, sort_keys=True))
            return 0 if summary["passed"] else 1

        goal = NavigateThroughPoses.Goal()
        for x_m, y_m, qz, qw in GOALS:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = x_m
            pose.pose.position.y = y_m
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            goal.poses.append(pose)

        send_future = node.action.send_goal_async(goal)
        if not spin_until(node, send_future.done, 10.0):
            raise RuntimeError("goal acceptance timed out")
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            raise RuntimeError("NavigateThroughPoses goal rejected")

        result_status = None
        result_error_code = None
        veto_latency_s = None
        if args.mode == "handoff":
            result_future = goal_handle.get_result_async()
            if not spin_until(node, result_future.done, args.timeout):
                raise RuntimeError("NavigateThroughPoses result timed out")
            wrapped_result = result_future.result()
            result_status = int(wrapped_result.status)
            result_error_code = int(wrapped_result.result.error_code)
            spin_until(node, lambda: False, 0.3)
        else:
            if not spin_until(
                node,
                lambda: (
                    node.motion_started_at is not None
                    and time.monotonic() - node.motion_started_at >= 0.5
                ),
                15.0,
            ):
                raise RuntimeError("motion did not start before veto")
            if not node.stop.wait_for_service(timeout_sec=5.0):
                raise RuntimeError("supervisor /stop service unavailable")
            node.veto_requested_at = time.monotonic()
            node.stop.call_async(Trigger.Request())
            if not spin_until(
                node,
                lambda: node.first_zero_after_veto_at is not None,
                1.0,
            ):
                raise RuntimeError("supervisor veto did not produce zero")
            veto_latency_s = (
                node.first_zero_after_veto_at - node.veto_requested_at
            )
            cancel_future = goal_handle.cancel_goal_async()
            spin_until(node, cancel_future.done, 3.0)

        final_distance = math.hypot(
            node.map_xy[0] - GOALS[-1][0],
            node.map_xy[1] - GOALS[-1][1],
        )
        handoff_ok = (
            all(value <= 0.15 for value in node.min_waypoint_distance)
            and all(value > 0 for value in node.waypoint_samples)
            and all(
                value <= 0.15
                for value in node.waypoint_max_zero_duration_s
            )
        )
        bounds_ok = (
            node.max_linear_mps <= 0.100001
            and node.max_angular_rad_s <= 0.400001
        )
        mode_ok = (
            result_status == GoalStatus.STATUS_SUCCEEDED
            and result_error_code == 0
            and final_distance <= 0.10
            and handoff_ok
            if args.mode == "handoff"
            else veto_latency_s is not None and veto_latency_s <= 0.30
        )
        summary = {
            "mode": args.mode,
            "passed": bool(graph_ok and bounds_ok and mode_ok),
            "motion_authority": False,
            "physical_execution_enabled": False,
            "live_sensors_started": False,
            "graph_ok": graph_ok,
            "graph": graph,
            "lifecycle_states": lifecycle_states,
            "result_status": result_status,
            "result_error_code": result_error_code,
            "final_map_xy": [round(value, 6) for value in node.map_xy],
            "final_goal_distance_m": round(final_distance, 6),
            "minimum_waypoint_distance_m": [
                round(value, 6) for value in node.min_waypoint_distance
            ],
            "waypoint_motor_samples": node.waypoint_samples,
            "waypoint_zero_samples": node.waypoint_zero_samples,
            "waypoint_max_zero_duration_s": [
                round(value, 6)
                for value in node.waypoint_max_zero_duration_s
            ],
            "motor_samples": node.motor_samples,
            "nonzero_motor_samples": node.nonzero_motor_samples,
            "max_linear_mps": round(node.max_linear_mps, 6),
            "max_angular_rad_s": round(node.max_angular_rad_s, 6),
            "veto_zero_latency_s": (
                None if veto_latency_s is None else round(veto_latency_s, 6)
            ),
        }
        print(json.dumps(summary, indent=2, sort_keys=True))
        return 0 if summary["passed"] else 1
    except Exception as exc:
        summary = {
            "mode": args.mode,
            "passed": False,
            "error": f"{type(exc).__name__}: {exc}",
            "motion_authority": False,
            "physical_execution_enabled": False,
            "live_sensors_started": False,
        }
        print(json.dumps(summary, indent=2, sort_keys=True))
        return 1
    finally:
        if goal_handle is not None and args.mode == "handoff":
            pass
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
