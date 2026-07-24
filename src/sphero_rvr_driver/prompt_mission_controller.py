"""Durable asynchronous prompt-mission orchestration above MissionService.

The controller owns provider and executor threads, while MissionService owns all
durable state.  It has no implicit physical authority: execution requires both a
service constructed with ``live_execution_enabled=True`` and an explicitly
supplied route executor.
"""

from __future__ import annotations

import json
import threading
import time
from typing import Any, Mapping, Optional, Protocol
import uuid

from .live_route_runner import LiveRouteRequest, route_request_from_json
from .mission_api import MissionValidationError
from .mission_service import MissionService
from .prompt_drive import PromptDrivePlanner


class PromptRouteExecutor(Protocol):
    def execute(self, request: LiveRouteRequest) -> Mapping[str, Any]: ...

    def cancel(self) -> bool: ...


class PromptMissionController:
    """Coordinate durable planning, approval, execution, and cancellation."""

    def __init__(
        self,
        service: MissionService,
        planner: PromptDrivePlanner,
        *,
        route_executor: Optional[PromptRouteExecutor] = None,
        execution_enabled: bool = False,
        approval_ttl_s: float = 60.0,
        clock_s: Any = None,
    ) -> None:
        self.service = service
        self.planner = planner
        self.route_executor = route_executor
        self.execution_enabled = bool(execution_enabled)
        self.approval_ttl_s = float(approval_ttl_s)
        self._clock_s = clock_s or time.time
        if self.execution_enabled != self.service.live_execution_enabled:
            raise MissionValidationError(
                "controller and persistent service execution gates must agree"
            )
        if self.execution_enabled and self.route_executor is None:
            raise MissionValidationError("enabled prompt execution requires a route executor")
        if not 1.0 <= self.approval_ttl_s <= 300.0:
            raise MissionValidationError("approval TTL must be between 1 and 300 seconds")
        self._lock = threading.RLock()
        self._threads: dict[str, threading.Thread] = {}
        self._active_execution_id: Optional[str] = None
        self._closed = False

    def submit(
        self,
        prompt: str,
        *,
        session_id: str,
        source: str = "web",
        mission_id: Optional[str] = None,
    ) -> dict[str, Any]:
        with self._lock:
            self._ensure_open()
            identifier = str(mission_id or f"prompt-{uuid.uuid4().hex}")
            snapshot = self.service.begin_prompt_mission(
                mission_id=identifier,
                session_id=session_id,
                prompt=prompt,
                source=source,
            )
            self._start_thread(identifier, self._plan, "planning")
            return snapshot

    def approve(
        self,
        mission_id: str,
        *,
        supplied_approval: str,
        operator: str,
        authentication_source: str = "",
    ) -> dict[str, Any]:
        del authentication_source
        with self._lock:
            self._ensure_open()
            if not self.execution_enabled:
                raise MissionValidationError(
                    "live execution is disabled by reviewed service configuration"
                )
            readiness = getattr(self.route_executor, "assert_ready", None)
            if callable(readiness):
                readiness()
            snapshot = self.service.approve_prompt_mission(
                mission_id,
                supplied_approval=supplied_approval,
                operator=operator,
                expires_at_s=float(self._clock_s()) + self.approval_ttl_s,
            )
            self.service.transition_prompt_mission(mission_id, "queued")
            self._start_thread(mission_id, self._execute, "execution")
            return snapshot

    def status(self, mission_id: str) -> dict[str, Any]:
        return self.service.prompt_status(mission_id)

    def cancel(self, mission_id: str, *, reason: str = "operator cancelled mission") -> dict[str, Any]:
        with self._lock:
            snapshot = self.service.prompt_status(mission_id)
            if snapshot["status"] != "running":
                return self.service.cancel_prompt_mission(mission_id, reason=reason)
            self.service.request_prompt_cancel(mission_id, reason=reason)
            cancel = getattr(self.route_executor, "cancel", None)
            if not callable(cancel) or not bool(cancel()):
                return self.service.transition_prompt_mission(
                    mission_id,
                    "recovery_required",
                    reason="route cancellation could not be confirmed; physical recovery required",
                )
            return self.service.prompt_status(mission_id)

    def service_snapshot(self) -> dict[str, Any]:
        return {
            "api_version": "mission_api.v2",
            "mode": self.service.mode,
            "source_sha": self.service.source_sha,
            "deployed_sha": self.service.deployed_sha,
            "planning_enabled": True,
            "live_execution_enabled": self.execution_enabled,
            "direct_ros_commands_allowed": False,
            "credentials_accepted_over_service": False,
            "capabilities": self.service.capabilities(),
        }

    def close(self, *, timeout_s: float = 130.0) -> None:
        with self._lock:
            self._closed = True
            threads = tuple(self._threads.values())
        deadline = time.monotonic() + float(timeout_s)
        for thread in threads:
            thread.join(timeout=max(0.0, deadline - time.monotonic()))
        alive = [thread.name for thread in threads if thread.is_alive()]
        if alive:
            raise RuntimeError(f"prompt mission worker did not stop: {alive[0]}")

    def _plan(self, mission_id: str) -> None:
        try:
            prompt = self.service.prompt_status(mission_id)["prompt"]
            proposal = self.planner.propose(prompt)
            self.service.record_prompt_proposal(mission_id, proposal.to_json_dict())
        except MissionValidationError as exc:
            self._reject_if_planning(mission_id, str(exc))
        except Exception as exc:  # Provider details must not cross the service boundary.
            self._reject_if_planning(
                mission_id,
                f"prompt provider failure: {exc.__class__.__name__}",
            )
        finally:
            self._finish_thread(mission_id)

    def _execute(self, mission_id: str) -> None:
        with self._lock:
            if self._active_execution_id is not None:
                self.service.transition_prompt_mission(
                    mission_id,
                    "recovery_required",
                    reason="another physical prompt mission already owns execution",
                )
                self._finish_thread(mission_id)
                return
            self._active_execution_id = mission_id
        try:
            snapshot = self.service.prompt_status(mission_id)
            expiry = snapshot["approval_expires_at_s"]
            if expiry is None or float(expiry) <= float(self._clock_s()):
                self.service.transition_prompt_mission(
                    mission_id,
                    "recovery_required",
                    reason="approval expired before route execution started",
                )
                return
            route = route_request_from_json(
                json.dumps(snapshot["route"], sort_keys=True),
                source_sha=snapshot["source_sha"],
            )
            self.service.transition_prompt_mission(mission_id, "running")
            result = dict(self.route_executor.execute(route))  # type: ignore[union-attr]
            status, reason = _terminal_from_route_result(result)
            current = self.service.prompt_status(mission_id)["status"]
            if current not in {"running", "cancel_requested"}:
                return
            self.service.transition_prompt_mission(
                mission_id,
                status,
                reason=reason,
                result=result,
            )
        except Exception as exc:
            current = self.service.prompt_status(mission_id)["status"]
            if current in {"running", "cancel_requested", "queued"}:
                self.service.transition_prompt_mission(
                    mission_id,
                    "recovery_required",
                    reason=f"route executor failure: {exc.__class__.__name__}; physical recovery required",
                )
        finally:
            with self._lock:
                if self._active_execution_id == mission_id:
                    self._active_execution_id = None
            self._finish_thread(mission_id)

    def _start_thread(self, mission_id: str, target: Any, phase: str) -> None:
        thread = threading.Thread(
            target=target,
            args=(mission_id,),
            name=f"prompt-mission-{phase}-{mission_id}",
            daemon=True,
        )
        self._threads[mission_id] = thread
        thread.start()

    def _finish_thread(self, mission_id: str) -> None:
        with self._lock:
            self._threads.pop(mission_id, None)

    def _reject_if_planning(self, mission_id: str, reason: str) -> None:
        try:
            if self.service.prompt_status(mission_id)["status"] in {"received", "planning"}:
                self.service.reject_prompt_planning(mission_id, reason)
        except MissionValidationError:
            return

    def _ensure_open(self) -> None:
        if self._closed:
            raise MissionValidationError("prompt mission controller is closed")


def _terminal_from_route_result(result: Mapping[str, Any]) -> tuple[str, str]:
    raw_status = str(result.get("status", "failed")).lower()
    reason = str(result.get("reason") or result.get("terminal_reason") or "")
    if raw_status in {"complete", "blocked", "cancelled", "stopped", "estopped", "failed"}:
        return raw_status, reason
    return "failed", reason or f"unsupported route terminal status: {raw_status}"
