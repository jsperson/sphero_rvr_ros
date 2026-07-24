"""Durable live Stage D orchestration owned by MissionService."""

from __future__ import annotations

import json
import threading
import time
from typing import Any, Mapping, Optional
import uuid

from .mission_api import MissionValidationError
from .mission_service import MissionService
from .stage_d_controller import (
    CodexOAuthStageDIntentProvider,
    StageDApprovalEnvelope,
    StageDController,
    StageDIntent,
    StageDIntentProvider,
    StageDLimits,
    validate_world_snapshot,
)
from .stage_d_physical import PhysicalStageDExecutor


class StageDLiveMissionController:
    """Plan and run one repeatedly replanned physical mission at a time."""

    def __init__(
        self,
        service: MissionService,
        provider: StageDIntentProvider,
        executor: PhysicalStageDExecutor,
        *,
        execution_enabled: bool = False,
        limits: Optional[StageDLimits] = None,
        clock_s: Any = time.time,
    ) -> None:
        self.service = service
        self.provider = provider
        self.executor = executor
        self.execution_enabled = bool(execution_enabled)
        self.limits = limits or StageDLimits()
        self._clock_s = clock_s
        if self.execution_enabled != self.service.live_execution_enabled:
            raise MissionValidationError(
                "Stage D controller and service execution gates must agree"
            )
        if self.execution_enabled != self.executor.execution_enabled:
            raise MissionValidationError(
                "Stage D controller and physical executor gates must agree"
            )
        self._lock = threading.RLock()
        self._threads: dict[str, threading.Thread] = {}
        self._controllers: dict[str, StageDController] = {}
        self._staged: dict[str, dict[str, Any]] = {}
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
                    "another physical Stage D mission is already active or awaiting approval"
                )
            identifier = str(
                mission_id or f"stage-d-live-{uuid.uuid4().hex}"
            )
            snapshot = self.service.begin_prompt_mission(
                mission_id=identifier,
                session_id=session_id,
                prompt=prompt,
                source=source,
            )
            self._start_thread(identifier, self._plan)
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
                    "physical Stage D is disabled by reviewed service configuration"
                )
            if self._active_execution_id is not None:
                raise MissionValidationError(
                    "another physical Stage D mission owns execution"
                )
            staged = self._staged.get(str(mission_id))
            if staged is None:
                raise MissionValidationError(
                    "Stage D proposal is not staged in this process"
                )
            readiness = self.executor.readiness()
            if readiness.get("ready") is not True:
                raise MissionValidationError(
                    "physical Stage D readiness failed: "
                    + ",".join(str(item) for item in readiness.get("reasons", []))
                )
            approval_requested_at = float(self._clock_s())
            snapshot = self.service.approve_stage_d_mission(
                mission_id,
                supplied_approval=supplied_approval,
                operator=operator,
                authentication_source=authentication_source,
                expires_at_s=(
                    approval_requested_at + self.limits.mission_lease_s
                ),
            )
            approval = snapshot.get("approval", {})
            if not isinstance(approval, Mapping):
                raise MissionValidationError(
                    "Stage D approval persistence failed"
                )
            approved_at = float(approval["approved_at_s"])
            self.executor.bind_approval(
                proposal_digest=str(approval.get("proposal_digest", "")),
                approval_id=str(approval.get("approval_id", "")),
                operator=str(approval.get("operator", "")),
            )
            first_intent = StageDIntent.validated(
                staged["raw_intent"],
                revision=1,
                snapshot=staged["snapshot"],
                issued_at_s=approved_at,
                provider_id=self.provider.provider_id,
                model_id=self.provider.model_id,
                limits=self.limits,
            )
            controller = StageDController(
                mission_id=mission_id,
                prompt=str(staged["proposal"]["prompt"]),
                proposal_digest=str(
                    staged["proposal"]["proposal_digest"]
                ),
                operator=str(approval["operator"]),
                authenticated=True,
                authentication_source=str(
                    approval["authentication_source"]
                ),
                approved_at_s=approved_at,
                first_snapshot=staged["snapshot"],
                first_intent=first_intent,
                provider=self.provider,
                executor=self.executor,
                limits=self.limits,
                checkpoint=lambda kind, projection: self._checkpoint(
                    mission_id, kind, projection
                ),
                owns_executor=False,
                now=self._clock_s,
            )
            self._controllers[mission_id] = controller
            self._active_execution_id = mission_id
            self.service.transition_prompt_mission(mission_id, "queued")
            self.service.transition_prompt_mission(mission_id, "running")
            controller.start()
            return self.service.prompt_status(mission_id)

    def status(self, mission_id: str) -> dict[str, Any]:
        return self.service.prompt_status(mission_id)

    def cancel(
        self,
        mission_id: str,
        *,
        reason: str = "operator cancelled Stage D mission",
    ) -> dict[str, Any]:
        with self._lock:
            snapshot = self.service.prompt_status(mission_id)
            status = str(snapshot["status"])
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
                            "Stage D controller is unavailable; physical "
                            "recovery required"
                        ),
                    )
                controller.cancel()
                return self.service.prompt_status(mission_id)
            cancelled = self.service.cancel_prompt_mission(
                mission_id, reason=reason
            )
            self._staged.pop(str(mission_id), None)
            return cancelled

    def service_snapshot(self) -> dict[str, Any]:
        readiness = self.executor.readiness()
        return {
            "api_version": "mission_api.v2",
            "mode": "live/stage-d",
            "stage_d_enabled": True,
            "source_sha": self.service.source_sha,
            "deployed_sha": self.service.deployed_sha,
            "planning_enabled": True,
            "live_execution_enabled": self.execution_enabled,
            "motion_authority": False,
            "direct_ros_commands_allowed": False,
            "credentials_accepted_over_service": False,
            "provider_id": self.provider.provider_id,
            "model_id": self.provider.model_id,
            "reasoning_effort": self.provider.reasoning_effort,
            "stage_d_readiness": readiness,
            "capabilities": self.service.capabilities(),
        }

    def close(self, *, timeout_s: float = 130.0) -> None:
        with self._lock:
            self._closed = True
            controllers = tuple(self._controllers.values())
            threads = tuple(self._threads.values())
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
                f"Stage D live worker did not stop: {alive[0]}"
            )
        self.executor.close()

    def _plan(self, mission_id: str) -> None:
        try:
            mission = self.service.prompt_status(mission_id)
            self.executor.reset(mission_id)
            snapshot = dict(self.executor.snapshot(mission_id))
            validate_world_snapshot(
                snapshot,
                mission_id=mission_id,
                require_motion=False,
            )
            raw = dict(self.provider.choose(mission["prompt"], snapshot))
            intent = StageDIntent.validated(
                raw,
                revision=1,
                snapshot=snapshot,
                issued_at_s=float(self._clock_s()),
                provider_id=self.provider.provider_id,
                model_id=self.provider.model_id,
                limits=self.limits,
            )
            proposal = StageDApprovalEnvelope(
                mission_id=mission_id,
                lease_id=f"stage-d-live-lease-{uuid.uuid4().hex}",
                prompt=str(mission["prompt"]),
                interpreted_objective=intent.interpreted_objective,
                source_sha=self.service.source_sha,
                deployed_sha=self.service.deployed_sha,
                provider_id=self.provider.provider_id,
                model_id=self.provider.model_id,
                reasoning_effort=self.provider.reasoning_effort,
                executor_mode=self.executor.mode,
                starting_snapshot_id=str(snapshot["snapshot_id"]),
                first_intent=raw,
                limits=self.limits,
                physical_execution_enabled=self.execution_enabled,
            ).proposal()
            self.service.record_stage_d_proposal(mission_id, proposal)
            with self._lock:
                self._staged[mission_id] = {
                    "snapshot": json.loads(json.dumps(snapshot)),
                    "raw_intent": json.loads(json.dumps(raw)),
                    "proposal": json.loads(json.dumps(proposal)),
                }
        except Exception as exc:
            try:
                if self.service.prompt_status(mission_id)["status"] in {
                    "received",
                    "planning",
                }:
                    self.service.reject_prompt_planning(
                        mission_id,
                        f"Stage D planning failed: {exc.__class__.__name__}: {exc}",
                    )
            except MissionValidationError:
                pass
        finally:
            with self._lock:
                self._threads.pop(mission_id, None)

    def _checkpoint(
        self,
        mission_id: str,
        kind: str,
        projection: Mapping[str, Any],
    ) -> None:
        if kind != "terminal":
            self.service.record_stage_d_checkpoint(
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

    def _start_thread(self, mission_id: str, target: Any) -> None:
        thread = threading.Thread(
            target=target,
            args=(mission_id,),
            name=f"stage-d-live-planning-{mission_id}",
            daemon=True,
        )
        self._threads[mission_id] = thread
        thread.start()

    def _ensure_open(self) -> None:
        if self._closed:
            raise MissionValidationError(
                "Stage D live mission controller is closed"
            )


def default_stage_d_provider(
    *, model: Optional[str], reasoning_effort: str
) -> CodexOAuthStageDIntentProvider:
    return CodexOAuthStageDIntentProvider(
        model=model,
        reasoning_effort=reasoning_effort,
    )
