from __future__ import annotations

from pathlib import Path
import threading
import time

import pytest

from sphero_rvr_driver.mission_api import MissionValidationError
from sphero_rvr_driver.mission_service import MissionService, MissionServiceServer
from sphero_rvr_driver.mission_service_client import MissionServiceClient
from sphero_rvr_driver.prompt_drive import (
    PromptDriveDecision,
    PromptDrivePlanner,
    PromptDriveProviderResponse,
    approval_phrase,
    prompt_drive_proposal_from_json,
)
from sphero_rvr_driver.prompt_mission_controller import PromptMissionController


class DeterministicProvider:
    provider_id = "test-provider"
    model_id = "test-model"
    reasoning_effort = "high"

    def __init__(self, *, blocked: threading.Event | None = None, release: threading.Event | None = None):
        self.blocked = blocked
        self.release = release

    def propose(self, prompt, limits):
        del prompt, limits
        if self.blocked is not None:
            self.blocked.set()
        if self.release is not None:
            assert self.release.wait(timeout=3.0)
        return PromptDriveProviderResponse(
            PromptDriveDecision.PROPOSE,
            "Move forward ten centimeters.",
            ({"tool_name": "move_distance", "value": 0.1},),
        )


class RecordingRouteExecutor:
    def __init__(self, *, block: bool = False, cancel_accepts: bool = True):
        self.requests = []
        self.started = threading.Event()
        self.release = threading.Event()
        self.block = block
        self.cancel_accepts = cancel_accepts

    def execute(self, request):
        self.requests.append(request)
        self.started.set()
        if self.block:
            assert self.release.wait(timeout=3.0)
            return {"status": "cancelled", "reason": "operator_cancelled"}
        return {
            "status": "complete",
            "reason": "target_reached",
            "route_id": request.route_id,
            "measured_distance_m": 0.102,
            "final_heading_deg": 0.4,
            "left_track_count": 123,
            "right_track_count": 121,
        }

    def cancel(self) -> bool:
        if not self.cancel_accepts:
            return False
        self.release.set()
        return True


def _service(path: Path, *, execution_enabled: bool = False) -> MissionService:
    return MissionService(
        path,
        source_sha="reviewed-source-sha",
        deployed_sha="deployed-package-sha",
        mode="live",
        executor_bindings={},
        live_execution_enabled=execution_enabled,
    )


def _controller(service: MissionService, *, executor=None, provider=None) -> PromptMissionController:
    return PromptMissionController(
        service,
        PromptDrivePlanner(provider or DeterministicProvider(), source_sha=service.source_sha),
        route_executor=executor,
        execution_enabled=service.live_execution_enabled,
    )


def _wait_status(controller: PromptMissionController, mission_id: str, expected: set[str]) -> dict:
    deadline = time.monotonic() + 4.0
    while time.monotonic() < deadline:
        snapshot = controller.status(mission_id)
        if snapshot["status"] in expected:
            return snapshot
        time.sleep(0.01)
    raise AssertionError(f"mission did not reach {expected}: {controller.status(mission_id)}")


def test_async_planning_persists_typed_evidence_before_returning(tmp_path: Path) -> None:
    blocked = threading.Event()
    release = threading.Event()
    service = _service(tmp_path / "planning.sqlite3")
    controller = _controller(service, provider=DeterministicProvider(blocked=blocked, release=release))
    try:
        initial = controller.submit(
            "Move forward ten centimeters.",
            session_id="browser-session",
            mission_id="async-planning",
        )
        assert initial["status"] == "planning"
        assert blocked.wait(timeout=1.0)
        assert controller.status("async-planning")["status"] == "planning"
        release.set()
        proposed = _wait_status(controller, "async-planning", {"proposed"})
        assert proposed["proposal"]["provider_id"] == "test-provider"
        assert proposed["proposal"]["model_id"] == "test-model"
        assert proposed["proposal"]["reasoning_effort"] == "high"
        assert proposed["proposal_digest"] == proposed["proposal"]["proposal_digest"]
        assert [event["kind"] for event in proposed["events"]] == [
            "received",
            "planning",
            "proposal",
        ]
        assert "credential" not in str(proposed).lower()
    finally:
        controller.close()
        service.close()


def test_exact_digest_approval_queues_and_executes_asynchronously(tmp_path: Path) -> None:
    executor = RecordingRouteExecutor()
    service = _service(tmp_path / "execute.sqlite3", execution_enabled=True)
    controller = _controller(service, executor=executor)
    try:
        controller.submit("Move 10 cm", session_id="physical-session", mission_id="execute")
        proposed = _wait_status(controller, "execute", {"proposed"})
        proposal = prompt_drive_proposal_from_json(proposed["proposal"])

        controller.approve(
            "execute",
            supplied_approval=approval_phrase(proposal),
            operator="scott",
        )
        terminal = _wait_status(controller, "execute", {"complete"})

        assert terminal["result"]["reason"] == "target_reached"
        assert terminal["result"]["final_heading_deg"] == 0.4
        assert terminal["result"]["left_track_count"] == 123
        assert terminal["approval"]["operator"] == "scott"
        assert terminal["approval"]["proposal_digest"] == proposal.proposal_digest
        assert len(executor.requests) == 1
        assert executor.requests[0].approval_id == f"scott:{proposal.proposal_digest}"
        event_kinds = [event["kind"] for event in terminal["events"]]
        assert event_kinds[-4:] == ["approval", "queued", "running", "terminal"]
    finally:
        controller.close()
        service.close()


def test_digest_mismatch_and_default_disabled_execution_fail_without_route(tmp_path: Path) -> None:
    service = _service(tmp_path / "disabled.sqlite3")
    controller = _controller(service)
    try:
        controller.submit("Move 10 cm", session_id="disabled", mission_id="disabled")
        _wait_status(controller, "disabled", {"proposed"})
        with pytest.raises(MissionValidationError, match="execution is disabled"):
            controller.approve(
                "disabled",
                supplied_approval="APPROVE " + "0" * 64,
                operator="scott",
            )
        assert controller.status("disabled")["status"] == "proposed"
        assert controller.status("disabled")["approval"] == {}
    finally:
        controller.close()
        service.close()

    enabled = _service(tmp_path / "digest.sqlite3", execution_enabled=True)
    executor = RecordingRouteExecutor()
    enabled_controller = _controller(enabled, executor=executor)
    try:
        enabled_controller.submit("Move 10 cm", session_id="digest", mission_id="digest")
        _wait_status(enabled_controller, "digest", {"proposed"})
        with pytest.raises(MissionValidationError, match="does not match"):
            enabled_controller.approve(
                "digest",
                supplied_approval="APPROVE " + "0" * 64,
                operator="scott",
            )
        assert executor.requests == []
        assert enabled_controller.status("digest")["status"] == "proposed"
    finally:
        enabled_controller.close()
        enabled.close()


def test_running_cancel_waits_for_executor_acknowledgement_and_terminal_result(tmp_path: Path) -> None:
    executor = RecordingRouteExecutor(block=True)
    service = _service(tmp_path / "cancel.sqlite3", execution_enabled=True)
    controller = _controller(service, executor=executor)
    try:
        controller.submit("Move 10 cm", session_id="cancel", mission_id="cancel")
        proposed = _wait_status(controller, "cancel", {"proposed"})
        controller.approve(
            "cancel",
            supplied_approval=approval_phrase(prompt_drive_proposal_from_json(proposed["proposal"])),
            operator="scott",
        )
        assert executor.started.wait(timeout=1.0)
        requested = controller.cancel("cancel", reason="browser cancel")
        assert requested["status"] in {"cancel_requested", "cancelled"}
        terminal = _wait_status(controller, "cancel", {"cancelled"})
        assert terminal["result"]["reason"] == "operator_cancelled"
        assert terminal["auto_resume"] is False
    finally:
        controller.close()
        service.close()


def test_unconfirmed_running_cancel_requires_recovery(tmp_path: Path) -> None:
    executor = RecordingRouteExecutor(block=True, cancel_accepts=False)
    service = _service(tmp_path / "cancel-fail.sqlite3", execution_enabled=True)
    controller = _controller(service, executor=executor)
    try:
        controller.submit("Move 10 cm", session_id="cancel-fail", mission_id="cancel-fail")
        proposed = _wait_status(controller, "cancel-fail", {"proposed"})
        controller.approve(
            "cancel-fail",
            supplied_approval=approval_phrase(prompt_drive_proposal_from_json(proposed["proposal"])),
            operator="scott",
        )
        assert executor.started.wait(timeout=1.0)
        recovery = controller.cancel("cancel-fail", reason="browser cancel")
        assert recovery["status"] == "recovery_required"
        assert recovery["recovery_required"] is True
        assert "could not be confirmed" in recovery["terminal_reason"]
        executor.release.set()
    finally:
        controller.close()
        service.close()


def test_restart_preserves_proposal_but_never_resumes_approved_work(tmp_path: Path) -> None:
    database = tmp_path / "restart.sqlite3"
    service = _service(database, execution_enabled=True)
    controller = _controller(service, executor=RecordingRouteExecutor())
    controller.submit("Move 10 cm", session_id="proposal", mission_id="proposal")
    proposed = _wait_status(controller, "proposal", {"proposed"})

    service.begin_prompt_mission(
        mission_id="planning",
        session_id="planning",
        prompt="Move 10 cm",
        source="web",
    )
    service.begin_prompt_mission(
        mission_id="approved",
        session_id="approved",
        prompt="Move 10 cm",
        source="web",
    )
    service.record_prompt_proposal("approved", proposed["proposal"] | {"prompt": "Move 10 cm"})
    approved_proposal = prompt_drive_proposal_from_json(service.prompt_status("approved")["proposal"])
    service.approve_prompt_mission(
        "approved",
        supplied_approval=approval_phrase(approved_proposal),
        operator="scott",
        expires_at_s=time.time() + 30.0,
    )
    controller.close()
    service.close()

    restarted = _service(database, execution_enabled=False)
    try:
        assert restarted.prompt_status("proposal")["status"] == "proposed"
        assert restarted.prompt_status("planning")["status"] == "rejected"
        approved = restarted.prompt_status("approved")
        assert approved["status"] == "recovery_required"
        assert approved["recovery_required"] is True
        assert approved["auto_resume"] is False
    finally:
        restarted.close()


def test_unix_socket_client_runs_proposal_only_flow_and_enforces_user_only_socket(tmp_path: Path) -> None:
    database = tmp_path / "socket.sqlite3"
    socket_path = Path("/tmp") / f"rvr-prompt-{tmp_path.name}.sock"

    def service_factory():
        return _service(database)

    def controller_factory(service):
        return _controller(service)

    server = MissionServiceServer(socket_path, service_factory, controller_factory)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    client = MissionServiceClient(socket_path, timeout_s=2.0)
    try:
        service_snapshot = client.service_snapshot()
        assert service_snapshot["planning_enabled"] is True
        assert service_snapshot["live_execution_enabled"] is False
        assert service_snapshot["direct_ros_commands_allowed"] is False

        initial = client.submit_prompt(
            "Move 10 cm",
            session_id="socket-browser",
            mission_id="socket-proposal",
        )
        assert initial["status"] == "planning"
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            proposed = client.prompt_status("socket-proposal")
            if proposed["status"] == "proposed":
                break
            time.sleep(0.01)
        assert proposed["status"] == "proposed"
        with pytest.raises(MissionValidationError, match="execution is disabled"):
            client.approve_prompt(
                "socket-proposal",
                approval_phrase=approval_phrase(prompt_drive_proposal_from_json(proposed["proposal"])),
                operator="scott",
            )
        cancelled = client.cancel_prompt("socket-proposal", reason="proposal review cancelled")
        assert cancelled["status"] == "cancelled"
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2.0)
    assert not thread.is_alive()
    assert not socket_path.exists()
