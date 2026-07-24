"""ROS client for sending one approved prompt-drive route to the live runner.

ROS imports and graph creation are intentionally delayed until ``execute``.
Proposal-only operation therefore cannot publish a route or a velocity command.
This client publishes only the live-route JSON request; the established route
runner remains the sole owner of ``/cmd_vel`` above the collision supervisor.
"""

from __future__ import annotations

import json
import threading
import time
from typing import Any, Mapping, Optional

from .live_route_runner import LiveRouteRequest
from .mission_api import MissionValidationError


class RosLiveRouteExecutor:
    def __init__(
        self,
        *,
        request_topic: str = "/mission_api/v2/live_route/request",
        status_topic: str = "/mission_api/v2/live_route/status",
        cancel_service: str = "live_route/cancel",
        graph_timeout_s: float = 5.0,
        cleanup_timeout_s: float = 3.0,
    ):
        self.request_topic = request_topic
        self.status_topic = status_topic
        self.cancel_service = cancel_service
        self.graph_timeout_s = float(graph_timeout_s)
        self.cleanup_timeout_s = float(cleanup_timeout_s)
        self._cancel_requested = threading.Event()
        self._state_lock = threading.RLock()
        self._active = False

    def cancel(self) -> bool:
        """Request cancellation; execute() confirms it through the ROS service."""

        with self._state_lock:
            if not self._active:
                return False
            self._cancel_requested.set()
            return True

    def execute(self, request: LiveRouteRequest) -> Mapping[str, Any]:
        if not request.approval_id:
            raise MissionValidationError("live prompt-drive route requires a bound operator approval_id")

        with self._state_lock:
            if self._active:
                raise MissionValidationError("prompt-drive route executor already owns an active route")
            self._active = True
            self._cancel_requested.clear()

        try:
            import rclpy
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.node import Node
            from std_msgs.msg import String
            from std_srvs.srv import Trigger
        except BaseException:
            with self._state_lock:
                self._active = False
                self._cancel_requested.clear()
            raise

        initialized_here = not rclpy.ok()
        node = None
        executor = None
        try:
            if initialized_here:
                rclpy.init(args=None)
            node = Node("prompt_drive_client")
            executor = SingleThreadedExecutor(context=node.context)
            executor.add_node(node)
        except BaseException:
            if executor is not None:
                executor.shutdown(timeout_sec=self.cleanup_timeout_s)
            if node is not None:
                node.destroy_node()
            if initialized_here:
                rclpy.try_shutdown()
            with self._state_lock:
                self._active = False
                self._cancel_requested.clear()
            raise
        publisher = node.create_publisher(String, self.request_topic, 10)
        cancel_client = node.create_client(Trigger, self.cancel_service)
        terminal: Optional[Mapping[str, Any]] = None
        route_published = False
        cancellation_attempted = False

        def _on_status(message) -> None:
            nonlocal terminal
            try:
                payload = json.loads(str(message.data))
            except (TypeError, ValueError, json.JSONDecodeError):
                return
            if not isinstance(payload, Mapping) or payload.get("route_id") != request.route_id:
                return
            if str(payload.get("status", "")).lower() != "running":
                terminal = dict(payload)

        subscription = node.create_subscription(String, self.status_topic, _on_status, 10)
        try:
            graph_deadline = time.monotonic() + self.graph_timeout_s
            while (
                publisher.get_subscription_count() < 1 or subscription.get_publisher_count() < 1
            ) and time.monotonic() < graph_deadline:
                executor.spin_once(timeout_sec=0.1)
            if publisher.get_subscription_count() < 1:
                raise MissionValidationError("live route runner is not subscribed; no route was published")
            if subscription.get_publisher_count() < 1:
                raise MissionValidationError("live route status publisher is unavailable; no route was published")

            message = String()
            message.data = json.dumps(request.to_json_dict(), sort_keys=True)
            publisher.publish(message)
            route_published = True
            deadline = time.monotonic() + float(request.max_runtime_s) + self.cleanup_timeout_s
            while terminal is None and time.monotonic() < deadline:
                if self._cancel_requested.is_set() and not cancellation_attempted:
                    cancellation_attempted = True
                    if not self._request_cancel(executor, cancel_client, Trigger):
                        raise MissionValidationError(
                            "live route cancellation could not be confirmed; use physical STOP/ESTOP"
                        )
                executor.spin_once(timeout_sec=0.1)
            if terminal is None:
                cancellation_attempted = True
                cancelled = self._request_cancel(executor, cancel_client, Trigger)
                if cancelled:
                    raise MissionValidationError("live route status timed out; cancellation was acknowledged")
                raise MissionValidationError(
                    "live route status timed out and cancellation could not be confirmed; use physical STOP/ESTOP"
                )
            return terminal
        except BaseException:
            if route_published and terminal is None and not cancellation_attempted:
                self._request_cancel(executor, cancel_client, Trigger)
            raise
        finally:
            executor.remove_node(node)
            executor.shutdown(timeout_sec=self.cleanup_timeout_s)
            node.destroy_node()
            if initialized_here:
                rclpy.try_shutdown()
            with self._state_lock:
                self._active = False
                self._cancel_requested.clear()

    def _request_cancel(self, executor, client, trigger_type) -> bool:
        deadline = time.monotonic() + self.cleanup_timeout_s
        while not client.service_is_ready() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.1)
        if not client.service_is_ready():
            return False
        future = client.call_async(trigger_type.Request())
        while not future.done() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.1)
        if not future.done():
            return False
        try:
            response = future.result()
        except Exception:
            return False
        return bool(getattr(response, "success", False))
