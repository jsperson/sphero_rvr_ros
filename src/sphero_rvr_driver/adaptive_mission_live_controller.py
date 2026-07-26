"""Durable live Adaptive mission orchestration owned by MissionService."""

from __future__ import annotations

import json
import threading
import time
from typing import Any, Mapping, Optional, Protocol
import uuid

from .mission_api import MissionValidationError
from .mission_service import MissionService
from .adaptive_mission_controller import (
    CodexOAuthAdaptiveMissionIntentProvider,
    AdaptiveMissionApprovalEnvelope,
    AdaptiveMissionController,
    AdaptiveMissionIntent,
    AdaptiveMissionIntentProvider,
    AdaptiveMissionLimits,
    validate_world_snapshot,
)
from .adaptive_mission_physical import PhysicalAdaptiveMissionExecutor


class AdaptiveMissionSessionLifecycle(Protocol):
    """Approval-bound owner of the supervised physical ROS graph."""

    activation_capable: bool

    def activate(
        self,
        *,
        mission_id: str,
        proposal_digest: str,
        operator: str,
    ) -> Mapping[str, Any]: ...

    def deactivate(self, *, reason: str) -> Mapping[str, Any]: ...

    def status(self) -> Mapping[str, Any]: ...


class _AlreadyActiveSessionLifecycle:
    """Compatibility lifecycle for injected executors in ROS-free tests."""

    def __init__(self, activation_capable: bool) -> None:
        self.activation_capable = bool(activation_capable)

    def activate(
        self,
        *,
        mission_id: str,
        proposal_digest: str,
        operator: str,
    ) -> Mapping[str, Any]:
        del proposal_digest, operator
        if not self.activation_capable:
            raise MissionValidationError(
                "approval-time physical activation is disabled"
            )
        return {
            "activation_capable": True,
            "active": True,
            "transitioning": False,
            "mission_id": str(mission_id),
            "detail": "injected physical session active",
        }

    def deactivate(self, *, reason: str) -> Mapping[str, Any]:
        return {
            "activation_capable": self.activation_capable,
            "active": False,
            "transitioning": False,
            "mission_id": "",
            "detail": str(reason),
        }

    def status(self) -> Mapping[str, Any]:
        return {
            "activation_capable": self.activation_capable,
            "active": self.activation_capable,
            "transitioning": False,
            "mission_id": "",
            "detail": (
                "injected physical session ready"
                if self.activation_capable
                else "physical activation disabled"
            ),
        }


class LiveAdaptiveMissionController:
    """Plan and run one repeatedly replanned physical mission at a time."""

    def __init__(
        self,
        service: MissionService,
        provider: AdaptiveMissionIntentProvider,
        executor: PhysicalAdaptiveMissionExecutor,
        *,
        execution_enabled: bool = False,
        limits: Optional[AdaptiveMissionLimits] = None,
        session_lifecycle: Optional[AdaptiveMissionSessionLifecycle] = None,
        activation_timeout_s: float = 30.0,
        clock_s: Any = time.time,
    ) -> None:
        self.service = service
        self.provider = provider
        self.executor = executor
        self.execution_enabled = bool(execution_enabled)
        self.limits = limits or AdaptiveMissionLimits()
        self.session_lifecycle = session_lifecycle or (
            _AlreadyActiveSessionLifecycle(self.execution_enabled)
        )
        self.activation_timeout_s = float(activation_timeout_s)
        self._clock_s = clock_s
        if self.execution_enabled != self.service.live_execution_enabled:
            raise MissionValidationError(
                "Adaptive mission controller and service execution gates must agree"
            )
        if self.execution_enabled != self.executor.execution_enabled:
            raise MissionValidationError(
                "Adaptive mission controller and physical executor gates must agree"
            )
        if self.execution_enabled != bool(
            self.session_lifecycle.activation_capable
        ):
            raise MissionValidationError(
                "Adaptive mission controller and session activation gates must agree"
            )
        if not 1.0 <= self.activation_timeout_s <= 120.0:
            raise MissionValidationError(
                "Adaptive mission activation timeout must be between 1 and 120 seconds"
            )
        self._lock = threading.RLock()
        self._threads: dict[str, threading.Thread] = {}
        self._controllers: dict[str, AdaptiveMissionController] = {}
        self._staged: dict[str, dict[str, Any]] = {}
        self._activation_cancel: dict[str, threading.Event] = {}
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
            if (
                self._active_execution_id is not None
                or self._threads
                or self._staged
            ):
                raise MissionValidationError(
                    "another physical adaptive mission is already active or awaiting approval"
                )
            identifier = str(
                mission_id or f"adaptive-mission-live-{uuid.uuid4().hex}"
            )
            snapshot = self.service.begin_prompt_mission(
                mission_id=identifier,
                session_id=session_id,
                prompt=prompt,
                source=source,
                provider_call_started=False,
            )
            proposal = AdaptiveMissionApprovalEnvelope(
                mission_id=identifier,
                lease_id=f"adaptive-mission-live-lease-{uuid.uuid4().hex}",
                prompt=str(prompt).strip(),
                interpreted_objective=str(prompt).strip(),
                source_sha=self.service.source_sha,
                deployed_sha=self.service.deployed_sha,
                provider_id=self.provider.provider_id,
                model_id=self.provider.model_id,
                reasoning_effort=self.provider.reasoning_effort,
                executor_mode=self.executor.mode,
                starting_snapshot_id="pending-approval-activation",
                first_intent={},
                limits=self.limits,
                physical_execution_enabled=self.execution_enabled,
            ).proposal()
            snapshot = self.service.record_adaptive_mission_proposal(
                identifier, proposal
            )
            self._staged[identifier] = {
                "proposal": json.loads(json.dumps(proposal)),
            }
            return snapshot

    def approve(
        self,
        mission_id: str,
        *,
        supplied_approval: str,
        operator: str,
        authentication_source: str = "",
    ) -> dict[str, Any]:
        with self._lock:
            self._ensure_open()
            if not self.execution_enabled:
                raise MissionValidationError(
                    "physical Adaptive mission is disabled by reviewed service configuration"
                )
            if self._active_execution_id is not None:
                raise MissionValidationError(
                    "another physical adaptive mission owns execution"
                )
            persisted = self.service.prompt_status(mission_id)
            proposal = persisted.get("proposal", {})
            if (
                persisted.get("status") != "proposed"
                or not isinstance(proposal, Mapping)
                or proposal.get("schema")
                != "sphero_rvr.adaptive_mission_proposal.v1"
            ):
                raise MissionValidationError(
                    "a persisted Adaptive mission proposal is required before approval"
                )
            approval_requested_at = float(self._clock_s())
            snapshot = self.service.approve_adaptive_mission(
                mission_id,
                supplied_approval=supplied_approval,
                operator=operator,
                authentication_source=authentication_source,
                expires_at_s=(
                    approval_requested_at + self.limits.mission_lease_s
                ),
            )
            self._active_execution_id = mission_id
            cancellation = threading.Event()
            self._activation_cancel[mission_id] = cancellation
            self._start_thread(
                mission_id,
                self._activate_and_start,
                "approval-activation",
            )
            return snapshot

    def status(self, mission_id: str) -> dict[str, Any]:
        return self.service.prompt_status(mission_id)

    def cancel(
        self,
        mission_id: str,
        *,
        reason: str = "operator cancelled adaptive mission",
    ) -> dict[str, Any]:
        with self._lock:
            snapshot = self.service.prompt_status(mission_id)
            status = str(snapshot["status"])
            if status in {"approved", "queued"}:
                cancellation = self._activation_cancel.get(str(mission_id))
                if cancellation is not None:
                    cancellation.set()
                relock_error = self._deactivate_session(
                    reason="physical session relocked after operator cancellation"
                )
                if relock_error:
                    return self.service.transition_prompt_mission(
                        mission_id,
                        "recovery_required",
                        reason=(
                            "physical session relock failed during cancellation: "
                            + relock_error
                        ),
                    )
                cancelled = self.service.cancel_prompt_mission(
                    mission_id, reason=reason
                )
                return cancelled
            if status == "running":
                self.service.request_prompt_cancel(
                    mission_id, reason=reason
                )
                controller = self._controllers.get(mission_id)
                if controller is None:
                    return self.service.transition_prompt_mission(
                        mission_id,
                        "recovery_required",
                        reason=(
                            "Adaptive mission controller is unavailable; physical "
                            "recovery required"
                        ),
                    )
                controller.cancel()
                return self.service.prompt_status(mission_id)
            cancelled = self.service.cancel_prompt_mission(
                mission_id, reason=reason
            )
            self._staged.pop(str(mission_id), None)
            self._deactivate_session(
                reason="physical session remained locked after cancellation"
            )
            return cancelled

    def service_snapshot(self) -> dict[str, Any]:
        readiness = self.executor.readiness()
        session = dict(self.session_lifecycle.status())
        return {
            "api_version": "mission_api.v2",
            "mode": "live/adaptive-mission",
            "adaptive_mission_enabled": True,
            "source_sha": self.service.source_sha,
            "deployed_sha": self.service.deployed_sha,
            "planning_enabled": True,
            "live_execution_enabled": bool(session.get("active", False)),
            "approval_activation_enabled": self.execution_enabled,
            "adaptive_mission_lease_s": self.limits.mission_lease_s,
            "physical_session": session,
            "motion_authority": False,
            "direct_ros_commands_allowed": False,
            "credentials_accepted_over_service": False,
            "provider_id": self.provider.provider_id,
            "model_id": self.provider.model_id,
            "reasoning_effort": self.provider.reasoning_effort,
            "adaptive_mission_readiness": {
                **readiness,
                "planning_ready": True,
                "planning_reasons": [],
                "activation_capable": self.execution_enabled,
                "session_active": bool(session.get("active", False)),
            },
            "capabilities": self.service.capabilities(),
        }

    def close(self, *, timeout_s: float = 130.0) -> None:
        with self._lock:
            self._closed = True
            controllers = tuple(self._controllers.values())
            threads = tuple(self._threads.values())
            cancellation_events = tuple(self._activation_cancel.values())
        for cancellation in cancellation_events:
            cancellation.set()
        for controller in controllers:
            controller.close()
        deadline = time.monotonic() + float(timeout_s)
        for thread in threads:
            thread.join(
                timeout=max(0.0, deadline - time.monotonic())
            )
        alive = [thread.name for thread in threads if thread.is_alive()]
        if alive:
            raise RuntimeError(
                f"Adaptive mission live worker did not stop: {alive[0]}"
            )
        relock_error = self._deactivate_session(
            reason="physical session relocked because mission service stopped"
        )
        if relock_error:
            self.executor.close()
            raise RuntimeError(
                "physical session did not relock during mission service shutdown: "
                + relock_error
            )
        self.executor.close()

    def _activate_and_start(self, mission_id: str) -> None:
        try:
            mission = self.service.prompt_status(mission_id)
            approval = mission.get("approval", {})
            proposal = mission.get("proposal", {})
            if not isinstance(approval, Mapping) or not isinstance(
                proposal, Mapping
            ):
                raise MissionValidationError(
                    "approved Adaptive mission persistence is incomplete"
                )
            cancellation = self._activation_cancel[mission_id]
            self.session_lifecycle.activate(
                mission_id=mission_id,
                proposal_digest=str(approval.get("proposal_digest", "")),
                operator=str(approval.get("operator", "")),
            )
            deadline = time.monotonic() + self.activation_timeout_s
            while True:
                if cancellation.is_set():
                    raise MissionValidationError(
                        "operator cancelled during physical session activation"
                    )
                readiness = self.executor.readiness()
                if readiness.get("ready") is True:
                    break
                if time.monotonic() >= deadline:
                    raise MissionValidationError(
                        "fresh camera, lidar, localization, and supervised safety "
                        "evidence did not become ready before activation timeout: "
                        + ",".join(
                            str(item)
                            for item in readiness.get("reasons", [])
                        )
                    )
                cancellation.wait(0.05)
            self.executor.reset(mission_id)
            snapshot = dict(self.executor.snapshot(mission_id))
            validate_world_snapshot(
                snapshot,
                mission_id=mission_id,
                require_motion=False,
                require_execution_safety=True,
            )
            raw = dict(self.provider.choose(str(mission["prompt"]), snapshot))
            approved_at = float(approval["approved_at_s"])
            first_intent = AdaptiveMissionIntent.validated(
                raw,
                revision=1,
                snapshot=snapshot,
                issued_at_s=float(self._clock_s()),
                provider_id=self.provider.provider_id,
                model_id=self.provider.model_id,
                limits=self.limits,
            )
            self.executor.bind_approval(
                proposal_digest=str(approval.get("proposal_digest", "")),
                approval_id=str(approval.get("approval_id", "")),
                operator=str(approval.get("operator", "")),
            )
            controller = AdaptiveMissionController(
                mission_id=mission_id,
                prompt=str(proposal["prompt"]),
                proposal_digest=str(proposal["proposal_digest"]),
                operator=str(approval["operator"]),
                authenticated=True,
                authentication_source=str(
                    approval["authentication_source"]
                ),
                approved_at_s=approved_at,
                first_snapshot=snapshot,
                first_intent=first_intent,
                provider=self.provider,
                executor=self.executor,
                limits=self.limits,
                checkpoint=lambda kind, projection: self._checkpoint(
                    mission_id, kind, projection
                ),
                owns_executor=False,
                activation_event_message=(
                    "Authenticated approval activated the supervised physical "
                    "graph; fresh readiness was verified before the provider call."
                ),
                now=self._clock_s,
            )
            with self._lock:
                if cancellation.is_set():
                    raise MissionValidationError(
                        "operator cancelled during physical session activation"
                    )
                self._controllers[mission_id] = controller
            self.service.transition_prompt_mission(mission_id, "queued")
            self.service.transition_prompt_mission(mission_id, "running")
            controller.start()
        except Exception as exc:
            relock_error = self._deactivate_session(
                reason="physical session relocked after activation failure"
            )
            try:
                current = self.service.prompt_status(mission_id)["status"]
                if current in {"approved", "queued"}:
                    self.service.transition_prompt_mission(
                        mission_id,
                        (
                            "recovery_required"
                            if relock_error
                            else "failed"
                        ),
                        reason=(
                            "Adaptive mission approval activation failed: "
                            f"{exc.__class__.__name__}: {exc}"
                            + (
                                "; physical session relock failed: "
                                + relock_error
                                if relock_error
                                else ""
                            )
                        ),
                    )
            except MissionValidationError:
                pass
        finally:
            with self._lock:
                self._threads.pop(mission_id, None)
                self._activation_cancel.pop(mission_id, None)
                if self.service.prompt_status(mission_id)["status"] != "running":
                    self._staged.pop(mission_id, None)
                    if self._active_execution_id == mission_id:
                        self._active_execution_id = None

    def _checkpoint(
        self,
        mission_id: str,
        kind: str,
        projection: Mapping[str, Any],
    ) -> None:
        if kind != "terminal":
            self.service.record_adaptive_mission_checkpoint(
                mission_id,
                kind=kind,
                checkpoint=projection,
            )
            return
        status = str(projection.get("status", "failed"))
        reason = str(projection.get("terminal_reason", ""))
        result = projection.get("result", {})
        if not isinstance(result, Mapping):
            result = {}
        relock_error = self._deactivate_session(
            reason=f"physical session relocked after terminal outcome: {status}"
        )
        if relock_error:
            status = "recovery_required"
            reason = (
                "physical session relock failed after terminal settlement: "
                + relock_error
            )
            result = {
                **dict(result),
                "physical_session_relock": {
                    "verified": False,
                    "error": relock_error,
                },
            }
        else:
            result = {
                **dict(result),
                "physical_session_relock": {
                    "verified": True,
                    "error": "",
                },
            }
        current = self.service.prompt_status(mission_id)["status"]
        if current in {"running", "cancel_requested"}:
            self.service.transition_prompt_mission(
                mission_id,
                status,
                reason=reason,
                result=result,
            )
        with self._lock:
            if self._active_execution_id == mission_id:
                self._active_execution_id = None
            self._staged.pop(mission_id, None)
            self._controllers.pop(mission_id, None)

    def _start_thread(
        self, mission_id: str, target: Any, phase: str
    ) -> None:
        thread = threading.Thread(
            target=target,
            args=(mission_id,),
            name=f"adaptive-mission-live-{phase}-{mission_id}",
            daemon=True,
        )
        self._threads[mission_id] = thread
        thread.start()

    def _deactivate_session(self, *, reason: str) -> str:
        try:
            self.session_lifecycle.deactivate(reason=reason)
            return ""
        except Exception as exc:
            return f"{exc.__class__.__name__}: {exc}"

    def _ensure_open(self) -> None:
        if self._closed:
            raise MissionValidationError(
                "Adaptive mission live mission controller is closed"
            )


def default_adaptive_mission_provider(
    *,
    model: Optional[str],
    reasoning_effort: str,
    limits: Optional[AdaptiveMissionLimits] = None,
) -> CodexOAuthAdaptiveMissionIntentProvider:
    return CodexOAuthAdaptiveMissionIntentProvider(
        model=model,
        reasoning_effort=reasoning_effort,
        limits=limits,
    )
