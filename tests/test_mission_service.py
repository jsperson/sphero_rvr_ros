from __future__ import annotations

import json
from pathlib import Path
import sqlite3

import pytest

from sphero_rvr_driver.mission_api import (
    FakeCapabilityAdapters,
    MissionBudgets,
    MissionGoal,
    MissionPlan,
    MissionValidationError,
    SuccessCriterion,
    CriterionKind,
    ToolInvocation,
    build_default_registry,
)
from sphero_rvr_driver.mission_service import MissionService, MissionServiceServer



def _status_plan(goal_id: str, correlation_id: str) -> MissionPlan:
    return MissionPlan(
        goal=MissionGoal(
            goal_id=goal_id,
            objective="Read bounded rover status.",
            success_criteria=(
                SuccessCriterion(
                    criterion_id=f"{correlation_id}-complete",
                    description="status telemetry completes",
                    kind=CriterionKind.TOOL_COMPLETE,
                    tool_id="query_status_telemetry",
                ),
            ),
            budgets=MissionBudgets(max_steps=1, max_runtime_s=5.0),
        ),
        invocations=(ToolInvocation(correlation_id, "query_status_telemetry", "1.0", {}),),
    )


def _move_plan(goal_id: str, correlation_id: str) -> MissionPlan:
    return MissionPlan(
        goal=MissionGoal(
            goal_id=goal_id,
            objective="Exercise a fake bounded motion invocation.",
            success_criteria=(
                SuccessCriterion(
                    criterion_id=f"{correlation_id}-complete",
                    description="fake motion completes",
                    kind=CriterionKind.TOOL_COMPLETE,
                    tool_id="move_distance",
                ),
            ),
            budgets=MissionBudgets(
                max_steps=1,
                max_runtime_s=2.0,
                max_travel_m=0.1,
            ),
        ),
        invocations=(
            ToolInvocation(
                correlation_id,
                "move_distance",
                "1.0",
                {"distance_m": 0.1, "speed_mps": 0.1, "timeout_s": 1.0},
            ),
        ),
    )


def test_persistent_service_restart_fails_closed_and_event_log_reconstructs_execution(tmp_path: Path) -> None:
    database = tmp_path / "missions.sqlite3"
    service = MissionService(
        database,
        registry=build_default_registry(detector_classes=("shoe",)),
        adapters=FakeCapabilityAdapters(deployed_sha="deployed-test-sha"),
        mode="replay",
        source_sha="source-test-sha",
        session_budgets=MissionBudgets(max_steps=2, max_runtime_s=20.0),
    )

    first = service.submit_plan(_status_plan("mission-one", "status-one"), session_id="shared", source="mcp")

    assert first["status"] == "complete"
    assert first["ledger"]["steps"] == 1
    assert first["source_sha"] == "source-test-sha"
    assert first["deployed_sha"] == "deployed-test-sha"
    assert [event["kind"] for event in service.events("mission-one")] == [
        "proposal",
        "invocation",
        "observation",
        "terminal",
    ]

    service.close()
    with sqlite3.connect(database) as connection:
        connection.execute("UPDATE missions SET status = 'running' WHERE mission_id = 'mission-one'")
    restarted = MissionService(
        database,
        registry=build_default_registry(detector_classes=("shoe",)),
        adapters=FakeCapabilityAdapters(deployed_sha="deployed-test-sha"),
        mode="replay",
        source_sha="source-test-sha",
        session_budgets=MissionBudgets(max_steps=2, max_runtime_s=20.0),
    )

    recovered = restarted.status("mission-one")
    assert recovered["status"] == "recovery_required"
    assert recovered["cancel_latched"] is True
    assert recovered["auto_resume"] is False
    assert restarted.events("mission-one")[-1]["kind"] == "recovery_required"

    with pytest.raises(MissionValidationError, match="recovery/cancel latch"):
        restarted.submit_plan(_status_plan("mission-two", "status-two"), session_id="shared", source="planner")


def test_service_rejects_mode_namespace_crossing(tmp_path: Path) -> None:
    service = MissionService(
        tmp_path / "replay.sqlite3",
        registry=build_default_registry(detector_classes=("shoe",)),
        adapters=FakeCapabilityAdapters(),
        mode="replay",
    )

    with pytest.raises(MissionValidationError, match="credential namespace"):
        service.submit_plan(
            _status_plan("wrong-namespace", "status"),
            session_id="namespace",
            source="mcp",
            credential_namespace="physical",
        )


def test_executor_process_failure_latches_session_and_requires_recovery(tmp_path: Path) -> None:
    class ExplodingAdapters(FakeCapabilityAdapters):
        def begin_execution(self, *_args, **_kwargs):
            raise RuntimeError("simulated executor process death")

    service = MissionService(tmp_path / "crash.sqlite3", adapters=ExplodingAdapters())

    with pytest.raises(RuntimeError, match="simulated executor process death"):
        service.submit_plan(_status_plan("crashed", "status"), session_id="crash", source="api")

    crashed = service.status("crashed")
    assert crashed["status"] == "recovery_required"
    assert crashed["recovery_required"] is True
    assert crashed["cancel_latched"] is True
    assert crashed["auto_resume"] is False
    assert service.events("crashed")[-1]["kind"] == "recovery_required"
    with pytest.raises(MissionValidationError, match="recovery/cancel latch"):
        service.submit_plan(_status_plan("after-crash", "status-2"), session_id="crash", source="api")


def test_validation_error_after_execution_starts_requires_recovery(tmp_path: Path) -> None:
    class UnprovenQuiescenceAdapters(FakeCapabilityAdapters):
        def begin_execution(self, *_args, **_kwargs):
            raise MissionValidationError("adapter unavailable: cleanup could not prove quiescence")

    service = MissionService(tmp_path / "unsafe.sqlite3", adapters=UnprovenQuiescenceAdapters())

    with pytest.raises(MissionValidationError, match="could not prove quiescence"):
        service.submit_plan(_status_plan("unsafe", "status"), session_id="unsafe", source="api")

    assert service.status("unsafe")["status"] == "recovery_required"
    assert service.session_status("unsafe")["cancel_latched"] is True
    assert service.events("unsafe")[-1]["kind"] == "recovery_required"


def test_running_transition_and_invocation_events_commit_before_execution(tmp_path: Path) -> None:
    database = tmp_path / "durable.sqlite3"

    class InspectingAdapters(FakeCapabilityAdapters):
        def begin_execution(self, *args, **kwargs):
            with sqlite3.connect(database) as observer:
                status = observer.execute(
                    "SELECT status FROM missions WHERE mission_id = 'durable'"
                ).fetchone()
                kinds = [
                    row[0]
                    for row in observer.execute(
                        "SELECT kind FROM events WHERE mission_id = 'durable' ORDER BY event_id"
                    )
                ]
            assert status == ("running",)
            assert kinds == ["proposal", "approval", "invocation"]
            return super().begin_execution(*args, **kwargs)

    service = MissionService(database, adapters=InspectingAdapters())
    result = service.submit_plan(
        _move_plan("durable", "move"), session_id="durable", source="api"
    )

    assert result["status"] == "complete"


def test_event_log_is_append_only_json_and_contains_artifact_and_sha_evidence(tmp_path: Path) -> None:
    database = tmp_path / "events.sqlite3"
    service = MissionService(database, adapters=FakeCapabilityAdapters(), source_sha="source-sha")
    result = service.submit_plan(_status_plan("event-mission", "status"), session_id="events", source="cli")

    records = service.events("event-mission")
    assert all(record["source_sha"] == "source-sha" for record in records)
    assert all(record["deployed_sha"] for record in records)
    assert json.loads(json.dumps(records)) == records
    assert result["route"] == {"measured_distance_m": 0.0, "measured_angle_deg": 0.0, "completed_segments": 0}
    with sqlite3.connect(database) as connection:
        with pytest.raises(sqlite3.IntegrityError, match="append-only"):
            connection.execute("UPDATE events SET kind = 'tampered'")
        with pytest.raises(sqlite3.IntegrityError, match="append-only"):
            connection.execute("DELETE FROM events")


def test_local_socket_service_owns_status_cancel_and_events(tmp_path: Path) -> None:
    socket_path = Path("/tmp") / f"rvr-mission-{tmp_path.name}.sock"
    server = MissionServiceServer(
        socket_path,
        lambda: MissionService(tmp_path / "socket.sqlite3", adapters=FakeCapabilityAdapters()),
    )
    assert socket_path.stat().st_mode & 0o777 == 0o600
    try:
        submitted = server.dispatch(
            {
                "operation": "submit",
                "plan": _status_plan("socket-mission", "status").to_json_dict(),
                "session_id": "socket",
                "source": "api",
            }
        )
        assert submitted["status"] == "complete"
        assert server.dispatch({"operation": "status", "mission_id": "socket-mission"})["ledger"]["steps"] == 1
        assert server.dispatch({"operation": "events", "mission_id": "socket-mission"})[-1]["kind"] == "terminal"
        assert server.dispatch({"operation": "cancel", "session_id": "socket", "reason": "socket cancel"})["cancel_latched"] is True
    finally:
        server.server_close()


def test_local_socket_refuses_second_owner_before_database_recovery(tmp_path: Path) -> None:
    socket_path = Path("/tmp") / f"rvr-owner-{tmp_path.name}.sock"
    competing_socket_path = Path("/tmp") / f"rvr-owner-{tmp_path.name}-other.sock"
    database = tmp_path / "owner.sqlite3"
    first_server = MissionServiceServer(
        socket_path,
        lambda: MissionService(database, adapters=FakeCapabilityAdapters()),
    )
    first_server.service.submit_plan(
        _status_plan("active", "status"), session_id="active", source="api"
    )
    with sqlite3.connect(database) as connection:
        connection.execute("UPDATE missions SET status = 'running' WHERE mission_id = 'active'")
    try:
        with pytest.raises(MissionValidationError, match="database already owned"):
            MissionServiceServer(
                competing_socket_path,
                lambda: MissionService(database, adapters=FakeCapabilityAdapters()),
            )
        assert first_server.service.status("active")["status"] == "running"
        assert [event["kind"] for event in first_server.service.events("active")].count(
            "recovery_required"
        ) == 0
    finally:
        first_server.server_close()
