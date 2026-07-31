from __future__ import annotations

import json
from pathlib import Path
import sqlite3
import subprocess

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
from sphero_rvr_driver.mission_service import (
    ExecutorBinding,
    MissionService,
    MissionServiceServer,
    _hierarchical_no_contact_observation_eligible,
)


def _service(database: Path, **kwargs):
    kwargs.setdefault("source_sha", "source-test-sha")
    kwargs.setdefault("deployed_sha", "deployed-test-sha")
    return MissionService(database, **kwargs)


def _safe_timeout_row() -> dict[str, object]:
    return {
        "mission_id": "canonical-timeout",
        "status": "timeout",
        "proposal_json": json.dumps(
            {"schema": "sphero_rvr.hierarchical_physical_proposal.v1"}
        ),
        "result_json": json.dumps(
            {
                "schema": "sphero_rvr.hierarchical_canonical_result.v1",
                "mission_id": "canonical-timeout",
                "status": "timeout",
                "reason": "canonical M7.6 mission lease expired",
                "source_sha": "source-test-sha",
                "deployed_sha": "deployed-test-sha",
                "cleanup_verified": True,
                "motion_authority": False,
                "restart_resume_allowed": False,
                "run_evidence": {
                    "started_at_s": 10.0,
                    "ended_at_s": 20.0,
                },
            }
        ),
        "terminal_reason": "canonical M7.6 mission lease expired",
        "source_sha": "source-test-sha",
        "deployed_sha": "deployed-test-sha",
    }


def test_safe_lease_timeout_is_eligible_for_attended_observation() -> None:
    assert _hierarchical_no_contact_observation_eligible(
        _safe_timeout_row()
    ) is True


@pytest.mark.parametrize(
    ("result_field", "value"),
    [
        ("cleanup_verified", False),
        ("motion_authority", True),
        ("restart_resume_allowed", True),
        ("reason", "controller_failed"),
        ("source_sha", "wrong-source"),
    ],
)
def test_unsafe_timeout_is_not_eligible_for_attended_observation(
    result_field: str, value: object
) -> None:
    row = _safe_timeout_row()
    result = json.loads(str(row["result_json"]))
    result[result_field] = value
    row["result_json"] = json.dumps(result)

    assert _hierarchical_no_contact_observation_eligible(row) is False

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
    service = _service(
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
        "running",
        "observation",
        "terminal",
    ]

    service.close()
    with sqlite3.connect(database) as connection:
        connection.execute("UPDATE missions SET status = 'running' WHERE mission_id = 'mission-one'")
    restarted = _service(
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
    service = _service(
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

    service = _service(tmp_path / "crash.sqlite3", adapters=ExplodingAdapters())

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

    service = _service(tmp_path / "unsafe.sqlite3", adapters=UnprovenQuiescenceAdapters())

    with pytest.raises(MissionValidationError, match="could not prove quiescence"):
        service.submit_plan(_status_plan("unsafe", "status"), session_id="unsafe", source="api")

    assert service.status("unsafe")["status"] == "recovery_required"
    assert service.session_status("unsafe")["cancel_latched"] is True
    assert service.events("unsafe")[-1]["kind"] == "recovery_required"


def test_running_transition_and_invocation_events_commit_before_execution(tmp_path: Path) -> None:
    database = tmp_path / "durable.sqlite3"
    proposed_plan = _move_plan("durable", "move")

    class InspectingAdapters(FakeCapabilityAdapters):
        def begin_execution(self, *args, **kwargs):
            with sqlite3.connect(database) as observer:
                status = observer.execute(
                    "SELECT status FROM missions WHERE mission_id = 'durable'"
                ).fetchone()
                records = [
                    (row[0], json.loads(row[1]))
                    for row in observer.execute(
                        "SELECT kind, payload_json FROM events "
                        "WHERE mission_id = 'durable' ORDER BY event_id"
                    )
                ]
            assert status == ("running",)
            assert [kind for kind, _payload in records] == [
                "proposal",
                "approval",
                "invocation",
                "running",
            ]
            proposal, approval, invocation, running = [payload for _kind, payload in records]
            assert proposal == {
                "source": "api",
                "mode": "replay",
                "credential_namespace": "replay",
                "plan": proposed_plan.to_json_dict(),
            }
            assert proposal["plan"]["invocations"][0]["approval"] is None
            assert approval["approval"]["approved_by"] == "mission-service-replay-supervisor"
            assert invocation == {
                "invocation": {
                    **proposed_plan.invocations[0].to_json_dict(),
                    "approval": approval["approval"],
                },
                "source": "api",
            }
            assert running == {"status": "running"}
            return super().begin_execution(*args, **kwargs)

    service = _service(database, adapters=InspectingAdapters())
    result = service.submit_plan(
        proposed_plan, session_id="durable", source="api"
    )

    assert result["status"] == "complete"


def test_event_log_is_append_only_json_and_contains_artifact_and_sha_evidence(tmp_path: Path) -> None:
    database = tmp_path / "events.sqlite3"
    service = _service(database, adapters=FakeCapabilityAdapters(), source_sha="source-sha")
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
        lambda: _service(tmp_path / "socket.sqlite3", adapters=FakeCapabilityAdapters()),
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
        assert server.dispatch({"operation": "capabilities"})[
            "query_status_telemetry@1.0"
        ]["healthy"] is True
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
        lambda: _service(database, adapters=FakeCapabilityAdapters()),
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
                lambda: _service(database, adapters=FakeCapabilityAdapters()),
            )
        assert first_server.service.status("active")["status"] == "running"
        assert [event["kind"] for event in first_server.service.events("active")].count(
            "recovery_required"
        ) == 0
    finally:
        first_server.server_close()


def test_database_symlink_alias_cannot_acquire_a_second_owner(tmp_path: Path) -> None:
    database = tmp_path / "owner.sqlite3"
    alias = tmp_path / "owner-alias.sqlite3"
    first = _service(database, adapters=FakeCapabilityAdapters())
    alias.symlink_to(database)
    second = None
    try:
        with pytest.raises(MissionValidationError, match="database already owned"):
            second = _service(alias, adapters=FakeCapabilityAdapters())
    finally:
        if second is not None:
            second.close()
        first.close()


def test_injected_provenance_is_independent_of_unrelated_git_checkout(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    unrelated = tmp_path / "unrelated-checkout"
    unrelated.mkdir()
    subprocess.run(["git", "init", "-q"], cwd=unrelated, check=True)
    (unrelated / "README").write_text("unrelated\n", encoding="utf-8")
    subprocess.run(["git", "add", "README"], cwd=unrelated, check=True)
    subprocess.run(
        [
            "git",
            "-c",
            "user.name=Mission Service Test",
            "-c",
            "user.email=mission-service@example.invalid",
            "commit",
            "-qm",
            "unrelated commit",
        ],
        cwd=unrelated,
        check=True,
    )
    unrelated_sha = subprocess.check_output(
        ["git", "rev-parse", "HEAD"], cwd=unrelated, text=True
    ).strip()
    monkeypatch.chdir(unrelated)

    service = MissionService(
        tmp_path / "provenance.sqlite3",
        adapters=FakeCapabilityAdapters(),
        source_sha="reviewed-source-sha",
        deployed_sha="deployed-build-sha",
    )
    try:
        service.submit_plan(
            _status_plan("provenance", "status"),
            session_id="provenance",
            source="api",
        )
        status = service.status("provenance")
        events = service.events("provenance")
    finally:
        service.close()

    assert unrelated_sha not in {status["source_sha"], status["deployed_sha"]}
    assert status["source_sha"] == "reviewed-source-sha"
    assert status["deployed_sha"] == "deployed-build-sha"
    assert all(event["source_sha"] == "reviewed-source-sha" for event in events)
    assert all(event["deployed_sha"] == "deployed-build-sha" for event in events)


def _executor_binding(
    executor: FakeCapabilityAdapters,
    *,
    heartbeat_at_s: float = 100.0,
    max_age_s: float = 5.0,
    mode: str = "replay",
    credential_namespace: str = "replay",
) -> ExecutorBinding:
    return ExecutorBinding(
        executor=executor,
        mode=mode,
        credential_namespace=credential_namespace,
        heartbeat_at_s=heartbeat_at_s,
        max_age_s=max_age_s,
        evidence={"executor": "fake-status", "probe": "deterministic"},
    )


def test_healthy_executor_binding_reports_fresh_evidence_and_executes(tmp_path: Path) -> None:
    now = [102.0]
    executor = FakeCapabilityAdapters(deployed_sha="executor-sha")
    service = _service(
        tmp_path / "healthy.sqlite3",
        adapters=executor,
        executor_bindings={"query_status_telemetry": _executor_binding(executor)},
        clock_s=lambda: now[0],
    )

    capability = service.capabilities()["query_status_telemetry@1.0"]
    assert capability == {
        "declared": True,
        "bound": True,
        "healthy": True,
        "mode": "replay",
        "credential_namespace": "replay",
        "availability": "available",
        "fresh": True,
        "heartbeat_at_s": 100.0,
        "age_s": 2.0,
        "max_age_s": 5.0,
        "evidence": {"executor": "fake-status", "probe": "deterministic"},
        "health_reason": "healthy",
    }
    assert service.submit_plan(
        _status_plan("bound-status", "status"), session_id="bound", source="api"
    )["status"] == "complete"


def test_missing_executor_binding_fails_closed_before_execution(tmp_path: Path) -> None:
    service = _service(
        tmp_path / "missing.sqlite3",
        executor_bindings={},
        clock_s=lambda: 100.0,
    )

    capability = service.capabilities()["query_status_telemetry@1.0"]
    assert capability["declared"] is True
    assert capability["bound"] is False
    assert capability["healthy"] is False
    assert capability["fresh"] is False
    assert capability["health_reason"] == "executor binding is missing"
    with pytest.raises(MissionValidationError, match="binding is missing"):
        service.submit_plan(
            _status_plan("missing-status", "status"), session_id="missing", source="api"
        )
    assert service.status("missing-status")["status"] == "rejected"


def test_stale_crashed_and_recovered_executor_bindings_fail_closed(tmp_path: Path) -> None:
    now = [106.0]
    executor = FakeCapabilityAdapters(deployed_sha="executor-sha")
    service = _service(
        tmp_path / "recovery.sqlite3",
        adapters=executor,
        executor_bindings={"query_status_telemetry": _executor_binding(executor)},
        clock_s=lambda: now[0],
    )

    stale = service.capabilities()["query_status_telemetry@1.0"]
    assert stale["bound"] is True
    assert stale["healthy"] is False
    assert stale["fresh"] is False
    assert stale["health_reason"] == "executor binding evidence is stale"
    with pytest.raises(MissionValidationError, match="evidence is stale"):
        service.submit_plan(
            _status_plan("stale-status", "status-stale"),
            session_id="stale",
            source="api",
        )

    service.heartbeat_executor(
        "query_status_telemetry",
        evidence={"executor": "fake-status", "probe": "recovered"},
    )
    executor.healthy = False
    crashed = service.capabilities()["query_status_telemetry@1.0"]
    assert crashed["fresh"] is True
    assert crashed["healthy"] is False
    assert crashed["health_reason"] == "executor reported unhealthy"
    with pytest.raises(MissionValidationError, match="reported unhealthy"):
        service.submit_plan(
            _status_plan("crashed-status", "status-crashed"),
            session_id="crashed",
            source="api",
        )

    executor.healthy = True
    now[0] = 107.0
    service.heartbeat_executor(
        "query_status_telemetry",
        evidence={"executor": "fake-status", "probe": "recovered"},
    )
    recovered = service.capabilities()["query_status_telemetry@1.0"]
    assert recovered["healthy"] is True
    assert recovered["evidence"]["probe"] == "recovered"
    assert service.submit_plan(
        _status_plan("recovered-status", "status-recovered"),
        session_id="recovered",
        source="api",
    )["status"] == "complete"


def test_restart_drops_live_bindings_and_never_restores_physical_authority(tmp_path: Path) -> None:
    database = tmp_path / "live.sqlite3"
    executor = FakeCapabilityAdapters(
        execution_mode="physical",
        authority_kind="physical",
        deployed_sha="physical-executor-sha",
    )
    service = _service(
        database,
        adapters=executor,
        mode="live",
        executor_bindings={
            "query_status_telemetry": _executor_binding(
                executor,
                mode="live",
                credential_namespace="physical",
            )
        },
        clock_s=lambda: 101.0,
    )
    assert service.capabilities()["query_status_telemetry@1.0"]["healthy"] is True
    service.close()

    restarted = _service(
        database,
        adapters=executor,
        mode="live",
        clock_s=lambda: 101.0,
    )
    capability = restarted.capabilities()["query_status_telemetry@1.0"]
    assert capability["bound"] is False
    assert capability["healthy"] is False
    assert capability["health_reason"] == "executor binding is missing"
    assert restarted.events() == []


def test_binding_rejects_replay_physical_namespace_or_executor_mode_crossing(tmp_path: Path) -> None:
    replay = FakeCapabilityAdapters(execution_mode="replay", authority_kind="replay")
    service = _service(
        tmp_path / "binding-cross.sqlite3",
        adapters=replay,
        executor_bindings={},
        mode="replay",
        clock_s=lambda: 100.0,
    )

    with pytest.raises(MissionValidationError, match="mode/credential namespace cannot cross"):
        service.bind_executor(
            "query_status_telemetry",
            _executor_binding(replay, mode="live", credential_namespace="physical"),
        )
    physical = FakeCapabilityAdapters(execution_mode="physical", authority_kind="physical")
    with pytest.raises(MissionValidationError, match="executor mode does not match"):
        service.bind_executor(
            "query_status_telemetry",
            _executor_binding(physical),
        )


def test_distinct_status_and_observation_bindings_dispatch_to_their_executors(
    tmp_path: Path,
) -> None:
    class RecordingExecutor(FakeCapabilityAdapters):
        def __init__(self) -> None:
            super().__init__(deployed_sha="recording-executor-sha")
            self.calls: list[str] = []

        def begin_execution(self, invocation, definition, **kwargs):
            self.calls.append(invocation.tool_id)
            return super().begin_execution(invocation, definition, **kwargs)

    status_executor = RecordingExecutor()
    observation_executor = RecordingExecutor()
    plan = MissionPlan(
        goal=MissionGoal(
            goal_id="routed-bindings",
            objective="Use separately bound status and observation executors.",
            success_criteria=(
                SuccessCriterion(
                    "status-complete",
                    "status completes",
                    CriterionKind.TOOL_COMPLETE,
                    tool_id="query_status_telemetry",
                ),
                SuccessCriterion(
                    "observation-complete",
                    "observation completes",
                    CriterionKind.TOOL_COMPLETE,
                    tool_id="capture_observation",
                ),
            ),
            budgets=MissionBudgets(max_steps=2, max_runtime_s=10.0),
        ),
        invocations=(
            ToolInvocation("routed-status", "query_status_telemetry", "1.0", {}),
            ToolInvocation("routed-observation", "capture_observation", "1.0", {}),
        ),
    )
    service = _service(
        tmp_path / "routed.sqlite3",
        executor_bindings={
            "query_status_telemetry": _executor_binding(status_executor),
            "capture_observation": _executor_binding(observation_executor),
        },
        clock_s=lambda: 101.0,
    )

    result = service.submit_plan(plan, session_id="routed", source="api")

    assert result["status"] == "complete"
    assert status_executor.calls == ["query_status_telemetry"]
    assert observation_executor.calls == ["capture_observation"]


def test_binding_metadata_rejects_nonfinite_freshness_values() -> None:
    executor = FakeCapabilityAdapters()

    with pytest.raises(MissionValidationError, match="heartbeat must be finite"):
        _executor_binding(executor, heartbeat_at_s=float("nan"))
    with pytest.raises(MissionValidationError, match="max age must be positive and finite"):
        _executor_binding(executor, max_age_s=float("inf"))


def test_invalid_initial_binding_releases_database_owner_lock(tmp_path: Path) -> None:
    database = tmp_path / "invalid-binding.sqlite3"
    physical = FakeCapabilityAdapters(execution_mode="physical", authority_kind="physical")

    with pytest.raises(MissionValidationError, match="executor mode does not match"):
        _service(
            database,
            adapters=physical,
            executor_bindings={
                "query_status_telemetry": _executor_binding(physical),
            },
        )

    replacement = _service(database, adapters=FakeCapabilityAdapters())
    replacement.close()
