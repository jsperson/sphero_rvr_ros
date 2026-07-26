from __future__ import annotations

import threading
import time
from typing import Any, Mapping, Optional

import pytest

from sphero_rvr_driver.live_mission_service import LiveStateCache
from sphero_rvr_driver.mission_api import MissionValidationError
from sphero_rvr_driver.mission_service import MissionService
from sphero_rvr_driver.adaptive_mission_live_controller import LiveAdaptiveMissionController
from sphero_rvr_driver.adaptive_mission_physical import PhysicalAdaptiveMissionExecutor
from sphero_rvr_driver.adaptive_mission_controller import AdaptiveMissionLimits


SHA = "reviewed-adaptive-mission-live-sha"
PROMPT = "Explore the room, revise after every observation, and stop safely."


def _cache(*, stale: bool = False, stop: bool = False) -> LiveStateCache:
    now = time.time() - (1.0 if stale else 0.0)
    cache = LiveStateCache()
    cache.update(
        "camera",
        {"frame_id": "live-camera-1", "detections": []},
        received_at_s=now,
    )
    cache.update(
        "lidar",
        {"scan_id": "live-scan-1", "sample_count": 720},
        received_at_s=now,
    )
    cache.update(
        "localization",
        {
            "state": "valid",
            "authoritative": True,
            "pose": {
                "frame_id": "map",
                "x_m": 0.0,
                "y_m": 0.0,
                "heading_deg": 0.0,
            },
        },
        received_at_s=now,
    )
    cache.update(
        "odom",
        {
            "frame_id": "odom",
            "x_m": 0.0,
            "y_m": 0.0,
            "heading_deg": 0.0,
        },
        received_at_s=now,
    )
    cache.update(
        "collision",
        {
            "state": "STOP" if stop else "CLEAR",
            "scan_healthy": True,
            "scan_age_s": 0.02,
            "tf_available": True,
            "tf_reason": "identity_same_frame",
            "forward_corridor_clearance_m": 1.2,
            "left_clearance_m": 0.9,
            "right_clearance_m": 0.8,
        },
        received_at_s=now,
    )
    cache.update(
        "control",
        {
            "state": "STOP" if stop else "READY",
            "stop_active": stop,
            "estop_latched": False,
        },
        received_at_s=now,
    )
    return cache


def _raw(
    snapshot: Mapping[str, Any], action: str, value: float = 0.0
) -> dict[str, Any]:
    return {
        "snapshot_id": snapshot["snapshot_id"],
        "action": action,
        "distance_m": value if action == "move_distance" else 0.0,
        "angle_deg": value if action == "turn_angle" else 0.0,
        "observation_focus": "fresh authoritative scan, TF, and odometry",
        "rationale": f"Revise from the typed snapshot and choose {action}.",
        "interpreted_objective": (
            "Explore locally reachable floor space and finish in a stopped state."
        ),
        "objective_status": "complete" if action == "stop" else "in_progress",
        "lease_s": 5.0,
        "timeout_s": 5.0,
    }


class SequenceProvider:
    provider_id = "injected-adaptive-mission-provider"
    model_id = "deterministic-adaptive-mission-model"
    reasoning_effort = "fixture"

    def __init__(self, actions: list[tuple[str, float]]) -> None:
        self.actions = list(actions)
        self.calls = 0
        self.snapshots: list[str] = []
        self.cache: Optional[LiveStateCache] = None

    def choose(
        self, prompt: str, snapshot: Mapping[str, Any]
    ) -> Mapping[str, Any]:
        assert prompt == PROMPT
        self.snapshots.append(str(snapshot["snapshot_id"]))
        action, value = self.actions[self.calls]
        self.calls += 1
        if action == "observe" and self.cache is not None:
            threading.Timer(0.1, self._publish_perception_cycle).start()
        return _raw(snapshot, action, value)

    def _publish_perception_cycle(self) -> None:
        if self.cache is None:
            return
        observed = time.time()
        self.cache.update(
            "camera",
            {"frame_id": f"observe-camera-{self.calls}", "detections": []},
            received_at_s=observed,
        )
        self.cache.update(
            "lidar",
            {"scan_id": f"observe-scan-{self.calls}", "sample_count": 720},
            received_at_s=observed,
        )
        self.cache.update(
            "localization",
            {
                "state": "valid",
                "authoritative": True,
                "pose": {
                    "frame_id": "map",
                    "x_m": 0.0,
                    "y_m": 0.0,
                    "heading_deg": 0.0,
                },
            },
            received_at_s=observed,
        )


class ObjectiveUpdateProvider:
    provider_id = "injected-objective-update-provider"
    model_id = "deterministic-objective-update-model"
    reasoning_effort = "fixture"

    def __init__(self) -> None:
        self.prompts: list[str] = []
        self.second_call_started = threading.Event()
        self.release_second_call = threading.Event()

    def choose(
        self, prompt: str, snapshot: Mapping[str, Any]
    ) -> Mapping[str, Any]:
        self.prompts.append(str(prompt))
        call = len(self.prompts)
        if call == 1:
            return _raw(snapshot, "move_distance", 0.10)
        if call == 2:
            self.second_call_started.set()
            assert self.release_second_call.wait(timeout=2.0)
            return _raw(snapshot, "turn_angle", 30.0)
        return _raw(snapshot, "stop", 0.0)


class ContinuousLeaseProvider(SequenceProvider):
    def __init__(self) -> None:
        super().__init__([("stop", 0.0), ("stop", 0.0)])
        self.prompts: list[str] = []

    def choose(
        self, prompt: str, snapshot: Mapping[str, Any]
    ) -> Mapping[str, Any]:
        self.prompts.append(str(prompt))
        self.snapshots.append(str(snapshot["snapshot_id"]))
        self.calls += 1
        return _raw(snapshot, "stop", 0.0)


class AdvancingClock:
    def __init__(self, now_s: float) -> None:
        self.now_s = float(now_s)

    def __call__(self) -> float:
        return self.now_s

    def advance(self, seconds: float) -> None:
        self.now_s += float(seconds)


class SlowFirstDecisionProvider(SequenceProvider):
    def __init__(self, clock: AdvancingClock) -> None:
        super().__init__([("stop", 0.0)])
        self.clock = clock

    def choose(
        self, prompt: str, snapshot: Mapping[str, Any]
    ) -> Mapping[str, Any]:
        self.clock.advance(6.0)
        return super().choose(prompt, snapshot)


class RecordingSessionLifecycle:
    activation_capable = True

    def __init__(
        self,
        cache: LiveStateCache,
        provider: SequenceProvider,
    ) -> None:
        self.cache = cache
        self.provider = provider
        self.active = False
        self.activations: list[tuple[str, str, str]] = []
        self.deactivations: list[str] = []

    def activate(
        self,
        *,
        mission_id: str,
        proposal_digest: str,
        operator: str,
    ) -> Mapping[str, Any]:
        assert self.provider.calls == 0
        self.activations.append(
            (mission_id, proposal_digest, operator)
        )
        fresh = _cache().snapshot()
        for name in (
            "camera",
            "lidar",
            "localization",
            "odom",
            "collision",
            "control",
        ):
            record = fresh.source(name)
            self.cache.update(
                name,
                record.value,
                received_at_s=time.time(),
            )
        self.active = True
        return self.status()

    def deactivate(self, *, reason: str) -> Mapping[str, Any]:
        self.active = False
        self.deactivations.append(str(reason))
        return self.status()

    def status(self) -> Mapping[str, Any]:
        return {
            "activation_capable": True,
            "active": self.active,
            "transitioning": False,
            "mission_id": (
                self.activations[-1][0]
                if self.active and self.activations
                else ""
            ),
            "detail": (
                "approved session active"
                if self.active
                else "physical session locked"
            ),
        }


class OneShotRelockFailureLifecycle(RecordingSessionLifecycle):
    def __init__(
        self,
        cache: LiveStateCache,
        provider: SequenceProvider,
    ) -> None:
        super().__init__(cache, provider)
        self.failed = False

    def deactivate(self, *, reason: str) -> Mapping[str, Any]:
        if "terminal outcome" in str(reason) and not self.failed:
            self.failed = True
            raise MissionValidationError(
                "systemd graph remained active"
            )
        return super().deactivate(reason=reason)


class FakeRouteTransport:
    def __init__(
        self,
        *,
        terminal_route_id: Optional[str] = None,
        block: bool = False,
    ) -> None:
        self.terminal_route_id = terminal_route_id
        self.block = block
        self.requests = []
        self.entered = threading.Event()
        self.release = threading.Event()
        self.cancelled = False
        self.cache: Optional[LiveStateCache] = None

    def execute(self, request):
        self.requests.append(request)
        self.entered.set()
        if self.block:
            self.release.wait(timeout=2.0)
        segment = request.segments[0]
        cancelled = self.cancelled
        if not cancelled and self.cache is not None:
            observed = time.time()
            self.cache.update(
                "camera",
                {
                    "frame_id": f"live-camera-{len(self.requests) + 1}",
                    "detections": [],
                },
                received_at_s=observed,
            )
            self.cache.update(
                "lidar",
                {
                    "scan_id": f"live-scan-{len(self.requests) + 1}",
                    "sample_count": 720,
                },
                received_at_s=observed,
            )
            self.cache.update(
                "localization",
                {
                    "state": "valid",
                    "authoritative": True,
                    "pose": {
                        "frame_id": "map",
                        "x_m": 0.0,
                        "y_m": 0.0,
                        "heading_deg": 0.0,
                    },
                },
                received_at_s=observed,
            )
        is_move = segment.tool_id == "move_distance"
        requested_linear = (
            float(segment.arguments["speed_mps"]) if is_move else 0.0
        )
        requested_angular = (
            0.0
            if is_move
            else (
                float(segment.arguments["angular_speed_deg_s"])
                * 3.141592653589793
                / 180.0
            )
        )
        return {
            "route_id": self.terminal_route_id or request.route_id,
            "status": "cancelled" if cancelled else "complete",
            "terminal_reason": (
                "operator cancellation settled" if cancelled else "complete"
            ),
            "source_sha": request.source_sha,
            "terminal_settled": True,
            "measured_distance_m": (
                0.0
                if cancelled
                else abs(float(segment.arguments.get("distance_m", 0.0)))
            ),
            "measured_angle_deg": (
                0.0
                if cancelled
                else abs(float(segment.arguments.get("angle_deg", 0.0)))
            ),
            "executed_segments": (
                []
                if cancelled
                else [
                    {
                        "correlation_id": segment.correlation_id,
                        "status": "complete",
                        "terminal_distance_error_m": (
                            0.0 if is_move else None
                        ),
                        "terminal_angle_error_deg": (
                            None if is_move else 0.0
                        ),
                    }
                ]
            ),
            "supervision": {
                "samples": 2,
                "collision_state": "CLEAR",
                "requested": {
                    "linear_mps": requested_linear,
                    "angular_rad_s": requested_angular,
                },
                "supervised": {
                    "linear_mps": 0.0 if cancelled else requested_linear / 2.0,
                    "angular_rad_s": (
                        0.0 if cancelled else requested_angular / 2.0
                    ),
                },
            },
        }

    def cancel(self) -> bool:
        self.cancelled = True
        self.release.set()
        return True


def _build(
    tmp_path,
    provider: SequenceProvider,
    *,
    cache: Optional[LiveStateCache] = None,
    transport: Optional[FakeRouteTransport] = None,
    limits: Optional[AdaptiveMissionLimits] = None,
    keep_session_active_until_lease_end: bool = False,
    clock_s: Any = None,
):
    authority_limits = limits or AdaptiveMissionLimits()
    service = MissionService(
        tmp_path / "adaptive-mission-live.sqlite3",
        source_sha=SHA,
        deployed_sha=SHA,
        mode="live",
        live_execution_enabled=True,
        adaptive_mission_limits=authority_limits.to_json_dict(),
        clock_s=clock_s,
    )
    active_cache = cache or _cache()
    route_transport = transport or FakeRouteTransport()
    route_transport.cache = active_cache
    provider.cache = active_cache
    executor = PhysicalAdaptiveMissionExecutor(
        active_cache,
        source_sha=SHA,
        deployed_sha=SHA,
        reviewed_sha=SHA,
        execution_enabled=True,
        transport=route_transport,
        limits=authority_limits,
    )
    controller = LiveAdaptiveMissionController(
        service,
        provider,
        executor,
        execution_enabled=True,
        limits=authority_limits,
        activation_timeout_s=1.0,
        keep_session_active_until_lease_end=(
            keep_session_active_until_lease_end
        ),
        clock_s=clock_s or time.time,
    )
    return service, controller, route_transport


def _wait_status(
    controller: LiveAdaptiveMissionController,
    mission_id: str,
    expected: set[str],
    *,
    timeout_s: float = 3.0,
) -> dict[str, Any]:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        snapshot = controller.status(mission_id)
        if snapshot["status"] in expected:
            return snapshot
        time.sleep(0.005)
    raise AssertionError(
        f"mission {mission_id} did not reach one of {sorted(expected)}"
    )


def _approve(
    controller: LiveAdaptiveMissionController, proposed: Mapping[str, Any]
) -> dict[str, Any]:
    digest = str(proposed["proposal_digest"])
    return controller.approve(
        str(proposed["mission_id"]),
        supplied_approval=f"APPROVE ADAPTIVE MISSION {digest}",
        operator="scott@example.com",
        authentication_source="tailscale-serve",
    )


def test_authenticated_approval_activates_fresh_evidence_before_first_provider_call(
    tmp_path,
) -> None:
    provider = SequenceProvider([("stop", 0.0)])
    cache = LiveStateCache()
    lifecycle = RecordingSessionLifecycle(cache, provider)
    service = MissionService(
        tmp_path / "approval-activation.sqlite3",
        source_sha=SHA,
        deployed_sha=SHA,
        mode="live",
        live_execution_enabled=True,
    )
    transport = FakeRouteTransport()
    transport.cache = cache
    executor = PhysicalAdaptiveMissionExecutor(
        cache,
        source_sha=SHA,
        deployed_sha=SHA,
        reviewed_sha=SHA,
        execution_enabled=True,
        transport=transport,
    )
    controller = LiveAdaptiveMissionController(
        service,
        provider,
        executor,
        execution_enabled=True,
        session_lifecycle=lifecycle,
        activation_timeout_s=1.0,
        keep_session_active_until_lease_end=False,
    )
    try:
        proposed = controller.submit(
            PROMPT,
            session_id="approval-activation",
            mission_id="approval-activation-mission",
        )
        assert proposed["status"] == "proposed"
        assert provider.calls == 0
        planning_event = next(
            event
            for event in proposed["events"]
            if event["kind"] == "planning"
        )
        assert planning_event["payload"][
            "provider_call_started"
        ] is False
        before = controller.service_snapshot()
        assert before["live_execution_enabled"] is False
        assert before["approval_activation_enabled"] is True
        assert before["adaptive_mission_readiness"]["planning_ready"] is True
        assert before["adaptive_mission_readiness"][
            "evidence_planning_ready"
        ] is False

        approved = _approve(controller, proposed)
        assert approved["status"] == "approved"
        terminal = _wait_status(
            controller, proposed["mission_id"], {"complete"}
        )
    finally:
        controller.close()
        service.close()

    assert provider.calls == 1
    assert len(lifecycle.activations) == 1
    assert lifecycle.activations[0][0] == proposed["mission_id"]
    assert lifecycle.activations[0][1] == proposed["proposal_digest"]
    assert lifecycle.activations[0][2] == "scott@example.com"
    assert lifecycle.active is False
    assert any(
        "terminal outcome: complete" in reason
        for reason in lifecycle.deactivations
    )
    first_snapshot = terminal["result"]["world_snapshots"][0]
    assert first_snapshot["evidence"]["scan_fresh"] is True
    assert first_snapshot["evidence"]["odometry_fresh"] is True


def test_first_intent_lease_starts_after_slow_provider_inference(
    tmp_path,
) -> None:
    clock = AdvancingClock(time.time())
    provider = SlowFirstDecisionProvider(clock)
    service, controller, transport = _build(
        tmp_path,
        provider,
        limits=AdaptiveMissionLimits(mission_lease_s=120.0),
        clock_s=clock,
    )
    try:
        proposed = controller.submit(
            PROMPT,
            session_id="slow-first-decision",
            mission_id="slow-first-decision-mission",
        )
        approved = _approve(controller, proposed)
        approved_at_s = approved["approval"]["approved_at_s"]
        terminal = _wait_status(
            controller,
            proposed["mission_id"],
            {"complete", "timeout"},
        )
    finally:
        controller.close()
        service.close()

    assert terminal["status"] == "complete"
    assert terminal["terminal_reason"] == "planner_stop"
    assert transport.requests == []
    revision = terminal["result"]["intent_revisions"][0]
    assert revision["issued_at_s"] == pytest.approx(approved_at_s + 6.0)
    assert revision["expires_at_s"] - revision["issued_at_s"] == pytest.approx(
        5.0
    )


def test_terminal_is_recovery_required_when_physical_session_cannot_relock(
    tmp_path,
) -> None:
    provider = SequenceProvider([("stop", 0.0)])
    cache = LiveStateCache()
    lifecycle = OneShotRelockFailureLifecycle(cache, provider)
    service = MissionService(
        tmp_path / "relock-failure.sqlite3",
        source_sha=SHA,
        deployed_sha=SHA,
        mode="live",
        live_execution_enabled=True,
    )
    transport = FakeRouteTransport()
    transport.cache = cache
    executor = PhysicalAdaptiveMissionExecutor(
        cache,
        source_sha=SHA,
        deployed_sha=SHA,
        reviewed_sha=SHA,
        execution_enabled=True,
        transport=transport,
    )
    controller = LiveAdaptiveMissionController(
        service,
        provider,
        executor,
        execution_enabled=True,
        session_lifecycle=lifecycle,
        activation_timeout_s=1.0,
        keep_session_active_until_lease_end=False,
    )
    try:
        proposed = controller.submit(
            PROMPT,
            session_id="relock-failure",
            mission_id="relock-failure-mission",
        )
        _approve(controller, proposed)
        terminal = _wait_status(
            controller,
            proposed["mission_id"],
            {"recovery_required"},
        )
    finally:
        controller.close()
        service.close()

    assert "relock failed" in terminal["terminal_reason"]
    assert terminal["result"]["physical_session_relock"] == {
        "verified": False,
        "error": (
            "MissionValidationError: systemd graph remained active"
        ),
    }


def test_configured_lease_is_bound_to_proposal_approval_and_terminal_relock(
    tmp_path,
) -> None:
    limits = AdaptiveMissionLimits(mission_lease_s=120.0)
    provider = SequenceProvider([("stop", 0.0)])
    service, controller, _ = _build(
        tmp_path,
        provider,
        limits=limits,
    )
    try:
        proposed = controller.submit(
            PROMPT,
            session_id="configured-lease",
            mission_id="configured-lease-mission",
        )
        assert proposed["proposal"]["limits"]["mission_lease_s"] == 120.0
        assert "2-minute adaptive mission lease" in proposed["proposal"]["summary"]
        assert controller.service_snapshot()["adaptive_mission_lease_s"] == 120.0
        approved = _approve(controller, proposed)
        approval = approved["approval"]
        assert approval["expires_at_s"] - approval["approved_at_s"] == pytest.approx(
            120.0
        )
        terminal = _wait_status(
            controller,
            proposed["mission_id"],
            {"complete"},
        )
    finally:
        controller.close()
        service.close()

    assert terminal["result"]["limits"]["mission_lease_s"] == 120.0
    assert terminal["result"]["physical_session_relock"]["verified"] is True


def test_browser_selected_shorter_lease_is_digest_bound_and_cannot_exceed_maximum(
    tmp_path,
) -> None:
    provider = SequenceProvider([("stop", 0.0)])
    service, controller, _ = _build(tmp_path, provider)
    try:
        proposed = controller.submit(
            PROMPT,
            session_id="selected-lease",
            mission_id="selected-lease-mission",
            mission_lease_s=120.0,
        )
        assert proposed["proposal"]["limits"]["mission_lease_s"] == 120.0
        assert proposed["proposal"]["contract"][
            "authenticated_objective_updates_within_lease"
        ] is True
        approved = _approve(controller, proposed)
        assert (
            approved["approval"]["expires_at_s"]
            - approved["approval"]["approved_at_s"]
        ) == pytest.approx(120.0)
        terminal = _wait_status(
            controller, proposed["mission_id"], {"complete"}
        )

        with pytest.raises(
            MissionValidationError, match="configured 900-second maximum"
        ):
            controller.submit(
                PROMPT,
                session_id="too-long",
                mission_id="too-long-mission",
                mission_lease_s=901.0,
            )
    finally:
        controller.close()
        service.close()

    assert terminal["result"]["limits"]["mission_lease_s"] == 120.0


def test_planner_stop_keeps_telemetry_lease_idle_for_later_objective(
    tmp_path,
) -> None:
    limits = AdaptiveMissionLimits(mission_lease_s=0.6)
    provider = ContinuousLeaseProvider()
    service, controller, transport = _build(
        tmp_path,
        provider,
        limits=limits,
        keep_session_active_until_lease_end=True,
    )
    updated_prompt = "Inspect the left side while keeping the same lease."
    try:
        proposed = controller.submit(
            PROMPT,
            session_id="continuous-idle-lease",
            mission_id="continuous-idle-lease-mission",
        )
        approved = _approve(controller, proposed)
        expires_at_s = approved["approval"]["expires_at_s"]

        deadline = time.monotonic() + 2.0
        idle = {}
        while time.monotonic() < deadline:
            idle = controller.status(proposed["mission_id"])
            projection = idle.get("result", {})
            if (
                idle["status"] == "running"
                and isinstance(projection, Mapping)
                and projection.get("lease_waiting_for_objective") is True
            ):
                break
            time.sleep(0.005)
        else:
            raise AssertionError("lease did not remain idle after planner stop")

        assert idle["approval"]["expires_at_s"] == expires_at_s
        assert controller.service_snapshot()["live_execution_enabled"] is True
        updated = controller.submit(
            updated_prompt,
            session_id="continuous-idle-lease",
            operator="scott@example.com",
            authentication_source="tailscale-serve",
        )
        assert updated["mission_id"] == proposed["mission_id"]
        assert updated["approval"]["expires_at_s"] == expires_at_s

        terminal = _wait_status(
            controller,
            proposed["mission_id"],
            {"timeout"},
            timeout_s=2.0,
        )
    finally:
        controller.close()
        service.close()

    assert provider.prompts == [PROMPT, updated_prompt]
    assert transport.requests == []
    assert terminal["terminal_reason"] == "mission_lease_expired"
    assert terminal["approval"]["expires_at_s"] == expires_at_s
    assert terminal["result"]["physical_session_relock"]["verified"] is True
    event_types = {
        event["event_type"]
        for event in terminal["result"]["events"]
    }
    assert "objective_complete" in event_types
    assert "objective_updated" in event_types


def test_authenticated_objective_update_keeps_active_lease_and_discards_inflight_plan(
    tmp_path,
) -> None:
    provider = ObjectiveUpdateProvider()
    service, controller, transport = _build(tmp_path, provider)  # type: ignore[arg-type]
    updated_prompt = "Continue mapping, then stop after the next fresh observation."
    try:
        proposed = controller.submit(
            PROMPT,
            session_id="continuous-lease",
            mission_id="continuous-lease-mission",
            mission_lease_s=120.0,
        )
        approved = _approve(controller, proposed)
        expires_at_s = approved["approval"]["expires_at_s"]
        assert provider.second_call_started.wait(timeout=2.0)

        updated = controller.submit(
            updated_prompt,
            session_id="continuous-lease",
            operator="scott@example.com",
            authentication_source="tailscale-serve",
        )
        assert updated["mission_id"] == proposed["mission_id"]
        assert updated["status"] == "running"
        assert updated["approval"]["expires_at_s"] == expires_at_s
        assert updated["prompt"] == updated_prompt

        provider.release_second_call.set()
        terminal = _wait_status(
            controller,
            proposed["mission_id"],
            {
                "complete",
                "failed",
                "blocked",
                "timeout",
                "recovery_required",
            },
        )
    finally:
        provider.release_second_call.set()
        controller.close()
        service.close()

    assert terminal["status"] == "complete", terminal
    assert provider.prompts == [PROMPT, PROMPT, updated_prompt]
    assert terminal["approval"]["expires_at_s"] == expires_at_s
    assert terminal["result"]["limits"]["mission_lease_s"] == 120.0
    assert len(transport.requests) == 1
    event_types = {
        event["event_type"]
        for event in terminal["result"]["events"]
    }
    assert "objective_updated" in event_types
    assert "llm_revision_discarded" in event_types


def test_live_adaptive_mission_replans_through_one_authenticated_lease(tmp_path) -> None:
    provider = SequenceProvider(
        [
            ("move_distance", 0.25),
            ("turn_angle", 45.0),
            ("observe", 0.0),
            ("stop", 0.0),
        ]
    )
    service, controller, transport = _build(tmp_path, provider)
    try:
        submitted = controller.submit(
            PROMPT,
            session_id="adaptive-mission-session",
            mission_id="adaptive-mission-live-replan",
        )
        proposed = _wait_status(
            controller, submitted["mission_id"], {"proposed"}
        )
        assert proposed["proposal"]["segments"] == []
        assert proposed["proposal"]["first_intent"] == {}
        assert proposed["proposal"]["starting_snapshot_id"] == (
            "pending-approval-activation"
        )
        assert proposed["proposal"]["contract"][
            "approval_activates_supervised_graph"
        ] is True
        assert proposed["proposal"]["contract"][
            "first_intent_requires_fresh_post_approval_evidence"
        ] is True
        assert proposed["proposal"]["contract"][
            "telemetry_managed_for_lease"
        ] is True
        assert proposed["proposal"]["contract"][
            "telemetry_stops_when_lease_ends"
        ] is True
        assert provider.calls == 0
        assert proposed["proposal"]["executor_mode"] == (
            "physical-supervised-live-route"
        )
        assert proposed["proposal"]["contract"][
            "physical_execution_enabled"
        ] is True
        assert proposed["proposal"]["limits"]["mission_lease_s"] == 900.0

        with pytest.raises(
            MissionValidationError, match="Tailscale-authenticated"
        ):
            controller.approve(
                submitted["mission_id"],
                supplied_approval=(
                    f"APPROVE ADAPTIVE MISSION {proposed['proposal_digest']}"
                ),
                operator="scott@example.com",
                authentication_source="loopback",
            )

        _approve(controller, proposed)
        terminal = _wait_status(
            controller, submitted["mission_id"], {"complete"}
        )
    finally:
        controller.close()
        service.close()

    result = terminal["result"]
    assert provider.calls == 4
    assert len(set(provider.snapshots)) == 4
    assert [item["action"] for item in result["intent_revisions"]] == [
        "move_distance",
        "turn_angle",
        "observe",
        "stop",
    ]
    assert len(transport.requests) == 2
    assert all(len(request.segments) == 1 for request in transport.requests)
    assert result["approval"]["authentication_source"] == "tailscale-serve"
    assert result["limits"]["max_translation_per_intent_m"] == 0.25
    assert result["limits"]["max_rotation_per_intent_deg"] == 45.0
    assert result["limits"]["linear_speed_mps"] == 0.10
    assert result["limits"]["angular_speed_rad_s"] == 0.4
    assert result["auto_resume"] is False
    assert result["drop_off_detection_available"] is False
    first = result["intent_revisions"][0]["execution"]["movement"]
    assert first["requested"]["linear_mps"] == 0.10
    assert first["supervised"]["linear_mps"] == 0.05
    events = result["events"]
    assert [event["sequence"] for event in events] == list(
        range(1, len(events) + 1)
    )
    kinds = [event["event_type"] for event in events]
    assert kinds[:5] == [
        "approval_bound",
        "physical_session_activated",
        "snapshot",
        "objective_interpreted",
        "llm_revision",
    ]
    assert kinds.count("snapshot") == 5
    assert kinds[-1] == "terminal"
    assert any(
        "requested +0.10 m/s" in event["message"]
        and "supervised +0.05 m/s" in event["message"]
        for event in events
    )
    assert any(
        "camera=fresh@" in event["message"]
        and "lidar=fresh@" in event["message"]
        and "localization=fresh@" in event["message"]
        for event in events
        if event["event_type"] == "snapshot"
    )


def test_locked_live_adaptive_mission_plans_observation_from_fresh_sensors(
    tmp_path,
) -> None:
    now = time.time()
    cache = LiveStateCache()
    cache.update(
        "lidar",
        {"scan_id": "live-scan-1", "sample_count": 720},
        received_at_s=now,
    )
    cache.update(
        "camera",
        {"frame_id": "live-camera-1", "detections": []},
        received_at_s=now,
    )
    cache.update(
        "localization",
        {
            "state": "valid",
            "authoritative": True,
            "pose": {
                "frame_id": "map",
                "x_m": 0.0,
                "y_m": 0.0,
                "heading_deg": 0.0,
            },
        },
        received_at_s=now,
    )
    provider = SequenceProvider([("observe", 0.0)])
    provider.cache = cache
    service = MissionService(
        tmp_path / "adaptive-mission-locked.sqlite3",
        source_sha=SHA,
        deployed_sha=SHA,
        mode="live",
        live_execution_enabled=False,
    )
    executor = PhysicalAdaptiveMissionExecutor(
        cache,
        source_sha=SHA,
        deployed_sha=SHA,
        execution_enabled=False,
    )
    controller = LiveAdaptiveMissionController(
        service,
        provider,
        executor,
        execution_enabled=False,
        keep_session_active_until_lease_end=False,
    )
    try:
        submitted = controller.submit(
            PROMPT,
            session_id="adaptive-mission-locked",
            mission_id="adaptive-mission-locked-observe",
        )
        proposed = _wait_status(
            controller, submitted["mission_id"], {"proposed"}
        )

        assert proposed["proposal"]["first_intent"] == {}
        assert proposed["proposal"]["contract"][
            "physical_execution_enabled"
        ] is False
        assert provider.calls == 0
        with pytest.raises(
            MissionValidationError,
            match="disabled by reviewed service configuration",
        ):
            _approve(controller, proposed)
    finally:
        controller.close()
        service.close()


def test_locked_live_adaptive_mission_rejects_stale_observation_sources(
    tmp_path,
) -> None:
    provider = SequenceProvider([("observe", 0.0)])
    service = MissionService(
        tmp_path / "adaptive-mission-locked-stale.sqlite3",
        source_sha=SHA,
        deployed_sha=SHA,
        mode="live",
        live_execution_enabled=False,
    )
    executor = PhysicalAdaptiveMissionExecutor(
        LiveStateCache(),
        source_sha=SHA,
        deployed_sha=SHA,
        execution_enabled=False,
    )
    controller = LiveAdaptiveMissionController(
        service,
        provider,
        executor,
        execution_enabled=False,
        keep_session_active_until_lease_end=False,
    )
    try:
        proposed = controller.submit(
            PROMPT,
            session_id="adaptive-mission-locked-stale",
            mission_id="adaptive-mission-locked-stale",
        )
        assert proposed["status"] == "proposed"
        assert proposed["proposal"]["first_intent"] == {}
        assert provider.calls == 0
    finally:
        controller.close()
        service.close()


@pytest.mark.parametrize(
    ("stale", "stop", "reason"),
    ((True, False, "scan_fresh"), (False, True, "collision_state")),
)
def test_live_adaptive_mission_readiness_vetoes_stale_or_stop_evidence(
    tmp_path, stale: bool, stop: bool, reason: str
) -> None:
    provider = SequenceProvider([("move_distance", 0.10)])
    cache = _cache()
    service, controller, transport = _build(
        tmp_path, provider, cache=cache
    )
    try:
        submitted = controller.submit(
            PROMPT,
            session_id=f"adaptive-mission-{reason}",
            mission_id=f"adaptive-mission-veto-{reason}",
        )
        proposed = _wait_status(
            controller, submitted["mission_id"], {"proposed"}
        )
        replacement = _cache(stale=stale, stop=stop)
        evidence = replacement.snapshot()
        for name in ("odom", "collision", "control"):
            record = evidence.source(name)
            cache.update(
                name,
                record.value,
                received_at_s=float(record.received_at_s),
            )
        approved = _approve(controller, proposed)
        assert approved["status"] == "approved"
        terminal = _wait_status(
            controller,
            submitted["mission_id"],
            {"failed"},
            timeout_s=2.0,
        )
    finally:
        controller.close()
        service.close()

    assert reason in terminal["terminal_reason"]
    assert transport.requests == []


def test_live_adaptive_mission_cancel_waits_for_correlated_settled_terminal(tmp_path) -> None:
    provider = SequenceProvider([("move_distance", 0.10), ("stop", 0.0)])
    transport = FakeRouteTransport(block=True)
    service, controller, _ = _build(
        tmp_path, provider, transport=transport
    )
    try:
        submitted = controller.submit(
            PROMPT,
            session_id="adaptive-mission-cancel",
            mission_id="adaptive-mission-live-cancel",
        )
        proposed = _wait_status(
            controller, submitted["mission_id"], {"proposed"}
        )
        _approve(controller, proposed)
        assert transport.entered.wait(timeout=1.0)
        cancelling = controller.cancel(submitted["mission_id"])
        assert cancelling["status"] == "cancel_requested"
        terminal = _wait_status(
            controller, submitted["mission_id"], {"cancelled"}
        )
    finally:
        controller.close()
        service.close()

    assert transport.cancelled is True
    assert terminal["terminal_reason"] == "operator cancellation settled"
    movement = terminal["result"]["intent_revisions"][0]["execution"][
        "movement"
    ]
    assert movement["supervised"]["linear_mps"] == 0.0


def test_live_adaptive_mission_uncorrelated_terminal_fails_closed(tmp_path) -> None:
    provider = SequenceProvider([("move_distance", 0.10)])
    service, controller, _ = _build(
        tmp_path,
        provider,
        transport=FakeRouteTransport(terminal_route_id="wrong-route"),
    )
    try:
        submitted = controller.submit(
            PROMPT,
            session_id="adaptive-mission-correlation",
            mission_id="adaptive-mission-live-correlation",
        )
        proposed = _wait_status(
            controller, submitted["mission_id"], {"proposed"}
        )
        _approve(controller, proposed)
        terminal = _wait_status(
            controller, submitted["mission_id"], {"failed"}
        )
    finally:
        controller.close()
        service.close()

    assert terminal["terminal_reason"] == "terminal_correlation_mismatch"
    assert (
        terminal["result"]["intent_revisions"][0]["execution"]["movement"][
            "supervised"
        ]["linear_mps"]
        == 0.0
    )


def test_live_adaptive_mission_restart_never_resumes_approved_mission(tmp_path) -> None:
    database = tmp_path / "adaptive-mission-live.sqlite3"
    provider = SequenceProvider([("observe", 0.0)])
    service, controller, _ = _build(tmp_path, provider)
    mission_id = "adaptive-mission-live-restart"
    try:
        submitted = controller.submit(
            PROMPT,
            session_id="adaptive-mission-restart",
            mission_id=mission_id,
        )
        proposed = _wait_status(
            controller, submitted["mission_id"], {"proposed"}
        )
        service.approve_adaptive_mission(
            mission_id,
            supplied_approval=(
                f"APPROVE ADAPTIVE MISSION {proposed['proposal_digest']}"
            ),
            operator="scott@example.com",
            authentication_source="tailscale-serve",
            expires_at_s=time.time() + 899.0,
        )
    finally:
        controller.close()
        service.close()

    restarted = MissionService(
        database,
        source_sha=SHA,
        deployed_sha=SHA,
        mode="live",
        live_execution_enabled=True,
    )
    try:
        recovered = restarted.prompt_status(mission_id)
    finally:
        restarted.close()

    assert recovered["status"] == "recovery_required"
    assert recovered["recovery_required"] is True
    assert "not resumed" in recovered["terminal_reason"]
