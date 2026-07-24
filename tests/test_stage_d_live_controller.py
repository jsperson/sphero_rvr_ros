from __future__ import annotations

import threading
import time
from typing import Any, Mapping, Optional

import pytest

from sphero_rvr_driver.live_mission_service import LiveStateCache
from sphero_rvr_driver.mission_api import MissionValidationError
from sphero_rvr_driver.mission_service import MissionService
from sphero_rvr_driver.stage_d_live_controller import StageDLiveMissionController
from sphero_rvr_driver.stage_d_physical import PhysicalStageDExecutor


SHA = "reviewed-stage-d-live-sha"
PROMPT = "Explore the room, revise after every observation, and stop safely."


def _cache(*, stale: bool = False, stop: bool = False) -> LiveStateCache:
    now = time.time() - (1.0 if stale else 0.0)
    cache = LiveStateCache()
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
        "lease_s": 5.0,
        "timeout_s": 5.0,
    }


class SequenceProvider:
    provider_id = "injected-stage-d-provider"
    model_id = "deterministic-stage-d-model"
    reasoning_effort = "fixture"

    def __init__(self, actions: list[tuple[str, float]]) -> None:
        self.actions = list(actions)
        self.calls = 0
        self.snapshots: list[str] = []

    def choose(
        self, prompt: str, snapshot: Mapping[str, Any]
    ) -> Mapping[str, Any]:
        assert prompt == PROMPT
        self.snapshots.append(str(snapshot["snapshot_id"]))
        action, value = self.actions[self.calls]
        self.calls += 1
        return _raw(snapshot, action, value)


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

    def execute(self, request):
        self.requests.append(request)
        self.entered.set()
        if self.block:
            self.release.wait(timeout=2.0)
        segment = request.segments[0]
        cancelled = self.cancelled
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
):
    service = MissionService(
        tmp_path / "stage-d-live.sqlite3",
        source_sha=SHA,
        deployed_sha=SHA,
        mode="live",
        live_execution_enabled=True,
    )
    route_transport = transport or FakeRouteTransport()
    executor = PhysicalStageDExecutor(
        cache or _cache(),
        source_sha=SHA,
        deployed_sha=SHA,
        reviewed_sha=SHA,
        execution_enabled=True,
        transport=route_transport,
    )
    controller = StageDLiveMissionController(
        service,
        provider,
        executor,
        execution_enabled=True,
    )
    return service, controller, route_transport


def _wait_status(
    controller: StageDLiveMissionController,
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
    controller: StageDLiveMissionController, proposed: Mapping[str, Any]
) -> dict[str, Any]:
    digest = str(proposed["proposal_digest"])
    return controller.approve(
        str(proposed["mission_id"]),
        supplied_approval=f"APPROVE STAGE D {digest}",
        operator="scott@example.com",
        authentication_source="tailscale-serve",
    )


def test_live_stage_d_replans_through_one_authenticated_lease(tmp_path) -> None:
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
            session_id="stage-d-session",
            mission_id="stage-d-live-replan",
        )
        proposed = _wait_status(
            controller, submitted["mission_id"], {"proposed"}
        )
        assert proposed["proposal"]["segments"] == []
        assert proposed["proposal"]["first_intent"]["action"] == "move_distance"
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
                    f"APPROVE STAGE D {proposed['proposal_digest']}"
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


@pytest.mark.parametrize(
    ("stale", "stop", "reason"),
    ((True, False, "scan_fresh"), (False, True, "collision_state")),
)
def test_live_stage_d_readiness_vetoes_stale_or_stop_evidence(
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
            session_id=f"stage-d-{reason}",
            mission_id=f"stage-d-veto-{reason}",
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
        with pytest.raises(
            MissionValidationError, match="readiness failed"
        ) as error:
            _approve(controller, proposed)
    finally:
        controller.close()
        service.close()

    assert reason in str(error.value)
    assert transport.requests == []


def test_live_stage_d_cancel_waits_for_correlated_settled_terminal(tmp_path) -> None:
    provider = SequenceProvider([("move_distance", 0.10), ("stop", 0.0)])
    transport = FakeRouteTransport(block=True)
    service, controller, _ = _build(
        tmp_path, provider, transport=transport
    )
    try:
        submitted = controller.submit(
            PROMPT,
            session_id="stage-d-cancel",
            mission_id="stage-d-live-cancel",
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


def test_live_stage_d_uncorrelated_terminal_fails_closed(tmp_path) -> None:
    provider = SequenceProvider([("move_distance", 0.10)])
    service, controller, _ = _build(
        tmp_path,
        provider,
        transport=FakeRouteTransport(terminal_route_id="wrong-route"),
    )
    try:
        submitted = controller.submit(
            PROMPT,
            session_id="stage-d-correlation",
            mission_id="stage-d-live-correlation",
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


def test_live_stage_d_restart_never_resumes_approved_mission(tmp_path) -> None:
    database = tmp_path / "stage-d-live.sqlite3"
    provider = SequenceProvider([("observe", 0.0)])
    service, controller, _ = _build(tmp_path, provider)
    mission_id = "stage-d-live-restart"
    try:
        submitted = controller.submit(
            PROMPT,
            session_id="stage-d-restart",
            mission_id=mission_id,
        )
        proposed = _wait_status(
            controller, submitted["mission_id"], {"proposed"}
        )
        service.approve_stage_d_mission(
            mission_id,
            supplied_approval=(
                f"APPROVE STAGE D {proposed['proposal_digest']}"
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
