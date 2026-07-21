from __future__ import annotations

import json
from datetime import datetime, timedelta, timezone

import pytest

from sphero_rvr_driver.mission_api import build_canonical_shoe_mapping_plan
from sphero_rvr_driver.mission_controls import (
    MissionControlError,
    MissionControlSession,
    MissionControlState,
    MissionExecutionMode,
    MissionPrincipal,
    PhysicalStartApproval,
    build_static_controls_bundle,
    handle_mission_control_request,
    _issue_physical_start_approval,
)


def _session() -> MissionControlSession:
    return MissionControlSession(build_canonical_shoe_mapping_plan(goal_id="ctrl-001"))


def _operator() -> MissionPrincipal:
    return MissionPrincipal(
        subject="operator:scott",
        permissions=("mission:start", "mission:cancel", "mission:pause"),
    )


def _physical_gate_for(mission_id: str, *, gate_id: str = "rvr-physical-start-gate") -> PhysicalStartApproval:
    approved_at = datetime.now(timezone.utc)
    return _issue_physical_start_approval(
        approved_by="physical-button:operator-present",
        approved_at=approved_at.isoformat(),
        expires_at=(approved_at + timedelta(minutes=5)).isoformat(),
        gate_id=gate_id,
        mission_id=mission_id,
        reason="robot is on blocks; operator present",
    )


def test_start_control_rejects_missing_or_unauthorized_principals_and_audits_denial() -> None:
    session = _session()

    with pytest.raises(MissionControlError, match="authenticated principal required"):
        session.start(None, mode=MissionExecutionMode.REPLAY)

    with pytest.raises(MissionControlError, match="missing permission: mission:start"):
        session.start(MissionPrincipal("viewer", permissions=("mission:read",)), mode=MissionExecutionMode.REPLAY)

    payload = session.to_json_dict()
    assert payload["mission"]["state"] == "IDLE"
    assert [event["decision"] for event in payload["audit_log"]] == ["denied", "denied"]
    assert payload["audit_log"][0]["api_version"] == "mission_api.v2"
    assert payload["audit_log"][1]["action"] == "mission.start"
    assert payload["audit_log"][1]["actor"] == "viewer"


def test_physical_motor_start_requires_explicit_gate_and_approval_is_audited() -> None:
    session = _session()
    operator = _operator()

    with pytest.raises(MissionControlError, match="physical start approval required"):
        session.start(operator, mode=MissionExecutionMode.PHYSICAL)

    with pytest.raises(MissionControlError, match="trusted physical gate"):
        session.start(
            operator,
            mode=MissionExecutionMode.PHYSICAL,
            physical_approval=PhysicalStartApproval(
                approved_by="browser:operator",
                approved_at="2026-07-19T01:20:00Z",
                gate_id="caller-constructed",
                reason="payload supplied by browser",
            ),
        )

    snapshot = session.start(
        operator,
        mode=MissionExecutionMode.PHYSICAL,
        physical_approval=_physical_gate_for("ctrl-001"),
    )

    payload = session.to_json_dict()
    assert snapshot.state is MissionControlState.RUNNING
    assert payload["mission"]["event_log"] == ["start_requested", "validated"]
    assert payload["physical_gate"]["approved"] is True
    assert payload["physical_gate"]["approval"]["gate_id"] == "rvr-physical-start-gate"
    assert payload["audit_log"][-1]["decision"] == "allowed"
    assert payload["audit_log"][-1]["physical_gate"]["approved"] is True
    assert payload["audit_log"][-1]["linked_mission_events"] == ["start_requested", "validated"]

    with pytest.raises(MissionControlError, match="cannot start from state RUNNING"):
        session.start(operator, mode=MissionExecutionMode.PHYSICAL, physical_approval=payload["physical_gate"]["approval"])


def test_control_router_cannot_mint_physical_start_approval_from_caller_payload() -> None:
    session = _session()
    body = json.dumps(
        {
            "api_version": "mission_api.v2",
            "execution_mode": "physical",
            "physical_approval": {
                "approved_by": "physical-button:operator-present",
                "approved_at": "2026-07-19T01:20:00Z",
                "expires_at": "2026-07-19T01:25:00Z",
                "gate_id": "rvr-physical-start-gate",
                "mission_id": "ctrl-001",
                "trusted_gate": True,
            },
        }
    )

    with pytest.raises(MissionControlError, match="trusted physical gate"):
        handle_mission_control_request("POST", "/api/mission/start", body, session, _operator())


def test_replay_start_can_run_without_physical_gate_and_never_claims_motor_authority() -> None:
    session = _session()

    snapshot = session.start(_operator(), mode=MissionExecutionMode.REPLAY)

    payload = session.to_json_dict()
    assert snapshot.state is MissionControlState.RUNNING
    assert payload["execution_mode"] == "replay"
    assert payload["physical_gate"]["approved"] is False
    assert payload["mission"]["telemetry"]["direct_ros_commands_allowed"] is False
    assert "/cmd_vel_motor" not in payload["mission"]["telemetry"]["command_path"]
    assert payload["motor_command_route_exposed"] is False


def test_cancel_and_pause_controls_require_permission_and_link_audit_to_mission_events() -> None:
    session = _session()
    session.start(_operator(), mode=MissionExecutionMode.REPLAY)

    with pytest.raises(MissionControlError, match="missing permission: mission:pause"):
        session.pause(MissionPrincipal("viewer", permissions=("mission:read",)), reason="phone button")

    paused = session.pause(_operator(), reason="operator pause")
    cancelled = session.cancel(_operator(), reason="operator cancel")

    payload = session.to_json_dict()
    assert paused.state is MissionControlState.PAUSED
    assert cancelled.state is MissionControlState.CANCELLED
    assert payload["audit_log"][-2]["action"] == "mission.pause"
    assert payload["audit_log"][-2]["linked_mission_events"][-1] == "pause_requested"
    assert payload["audit_log"][-1]["action"] == "mission.cancel"
    assert payload["audit_log"][-1]["linked_mission_events"][-1] == "cancelled"


def test_robot_side_estop_and_blocked_transitions_remain_independent_and_visible() -> None:
    session = _session()
    session.start(_operator(), mode=MissionExecutionMode.REPLAY)

    estopped = session.robot_safety_event("estop", reason="lidar supervisor estop")

    payload = session.to_json_dict()
    assert estopped.state is MissionControlState.ESTOPPED
    assert payload["robot_side_safety"]["independent_stop_estop_collision_supervisor"] is True
    assert payload["browser_stop_is_sole_safety_mechanism"] is False
    assert payload["audit_log"][-1]["action"] == "robot_safety.estop"
    assert payload["audit_log"][-1]["actor"] == "robot-side-supervisor"
    assert payload["audit_log"][-1]["decision"] == "latched"

    blocked_session = _session()
    blocked_session.start(_operator(), mode=MissionExecutionMode.REPLAY)
    blocked = blocked_session.robot_safety_event("blocked", reason="collision supervisor blocked")
    assert blocked.state is MissionControlState.BLOCKED
    assert blocked_session.to_json_dict()["audit_log"][-1]["linked_mission_events"][-1] == "blocked"


def test_terminal_control_states_are_latched_and_physical_gate_expiry_is_enforced() -> None:
    session = _session()
    session.start(_operator(), mode=MissionExecutionMode.REPLAY)
    session.cancel(_operator())

    with pytest.raises(MissionControlError, match="terminal state CANCELLED"):
        session.start(_operator(), mode=MissionExecutionMode.REPLAY)

    estop_session = _session()
    estop_session.start(_operator(), mode=MissionExecutionMode.REPLAY)
    estop_session.robot_safety_event("estop", reason="hard estop")
    with pytest.raises(MissionControlError, match="cannot pause from state ESTOPPED"):
        estop_session.pause(_operator())
    with pytest.raises(MissionControlError, match="terminal state ESTOPPED"):
        estop_session.start(_operator(), mode=MissionExecutionMode.REPLAY)

    expired = _issue_physical_start_approval(
        approved_by="physical-button:operator-present",
        approved_at="2026-07-19T01:25:00Z",
        expires_at="2026-07-19T01:20:00Z",
        gate_id="rvr-physical-start-gate",
        mission_id="ctrl-001",
        reason="expired gate",
    )
    with pytest.raises(MissionControlError, match="trusted physical gate"):
        _session().start(_operator(), mode=MissionExecutionMode.PHYSICAL, physical_approval=expired)

    ancient = _issue_physical_start_approval(
        approved_by="physical-button:operator-present",
        approved_at="2020-01-01T00:00:00Z",
        expires_at="2020-01-01T00:05:00Z",
        gate_id="rvr-physical-start-gate-old",
        mission_id="ctrl-001",
        reason="ancient gate must not start a physical session today",
    )
    with pytest.raises(MissionControlError, match="trusted physical gate"):
        _session().start(_operator(), mode=MissionExecutionMode.PHYSICAL, physical_approval=ancient)

    replayed = _physical_gate_for("ctrl-001", gate_id="rvr-physical-start-gate-replay")
    assert _session().start(_operator(), mode=MissionExecutionMode.PHYSICAL, physical_approval=replayed).state is MissionControlState.RUNNING
    with pytest.raises(MissionControlError, match="trusted physical gate"):
        _session().start(_operator(), mode=MissionExecutionMode.PHYSICAL, physical_approval=replayed)


def test_control_router_exposes_only_versioned_mission_api_actions_not_motor_or_generic_ros_routes() -> None:
    session = _session()
    principal = _operator()

    status, content_type, body = handle_mission_control_request(
        "POST",
        "/api/mission/start",
        json.dumps({"api_version": "mission_api.v2", "execution_mode": "replay"}),
        session,
        principal,
    )
    assert status == 200
    assert content_type == "application/json"
    assert json.loads(body)["mission"]["state"] == "RUNNING"

    for path in ("/api/motor", "/api/write", "/cmd_vel", "/cmd_vel_motor", "/api/ros"):
        with pytest.raises(MissionControlError, match="not exposed"):
            handle_mission_control_request("POST", path, "{}", session, principal)

    bundle = build_static_controls_bundle()
    assert "Start replay mission" in bundle.index_html
    assert "Request physical start" in bundle.index_html
    assert "Pause" in bundle.index_html
    assert "Cancel / STOP mission" in bundle.index_html
    assert "mission_api.v2" in bundle.index_html
    assert "robot-side STOP/ESTOP/collision supervisor remains independent" in bundle.index_html
    assert "/cmd_vel" not in bundle.index_html
    assert "/cmd_vel_motor" not in bundle.index_html
