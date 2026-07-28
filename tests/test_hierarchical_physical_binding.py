from __future__ import annotations

import json
from pathlib import Path

import pytest

from sphero_rvr_driver.hierarchical_exploration import FrontierCandidate
from sphero_rvr_driver.hierarchical_goal_selection import (
    AsyncSemanticGoalController,
    ScriptedSemanticGoalProvider,
    build_semantic_world_snapshot,
)
from sphero_rvr_driver.hierarchical_physical_binding import (
    ACCEPTED_DIRECTIONAL_ADDENDUM_SHA256,
    ACCEPTED_M7_3_EVIDENCE_SHA256,
    ACCEPTED_M7_4_EVIDENCE_SHA256,
    APPROVAL_SCHEMA,
    AUTHORITY_SCHEMA,
    GOAL_DISPATCH_SCHEMA,
    PHYSICAL_PROPOSAL_SCHEMA,
    HierarchicalBindingJournal,
    HierarchicalPhysicalAuthorityOwner,
    build_goal_dispatch,
    canonical_digest,
    resolve_goal_dispatch,
    validate_authority_heartbeat,
    validate_physical_proposal,
)
from sphero_rvr_driver.mission_api import MissionValidationError
from sphero_rvr_driver.mission_service import MissionService
from sphero_rvr_driver.mission_web import LiveMissionWebAdapter
from sphero_rvr_driver.prompt_drive import (
    PromptDriveDecision,
    PromptDriveLimits,
    PromptDrivePlanner,
    PromptDriveProviderResponse,
)
from sphero_rvr_driver.prompt_mission_controller import (
    PromptMissionController,
)


REPO_ROOT = Path(__file__).resolve().parents[1]
SHA = "a" * 40


class _UnusedPromptProvider:
    provider_id = "hierarchical-binding-test"
    model_id = "test-model"
    reasoning_effort = "low"

    def propose(self, prompt, limits):
        del prompt, limits
        return PromptDriveProviderResponse(
            PromptDriveDecision.REJECT,
            "Not used by the read-only browser projection test.",
            (),
        )


def _approval(*, now_s: float = 100.0) -> dict:
    payload = {
        "schema": APPROVAL_SCHEMA,
        "gate": "m7.6",
        "mission_id": "m7-canonical-001",
        "operator": "attended-operator",
        "source_sha": SHA,
        "deployed_sha": SHA,
        "reviewed_sha": SHA,
        "proposal_digest": "b" * 64,
        "approval_id": "m7.6-test-approval",
        "approved_at_s": now_s,
        "expires_at_s": now_s + 300.0,
        "m7_3_evidence_sha256": ACCEPTED_M7_3_EVIDENCE_SHA256,
        "directional_addendum_sha256": (
            ACCEPTED_DIRECTIONAL_ADDENDUM_SHA256
        ),
        "m7_4_evidence_sha256": ACCEPTED_M7_4_EVIDENCE_SHA256,
        "room": {
            "attended": True,
            "level_bounded": True,
            "stairs_ledges_dropoffs_absent": True,
            "negative_obstacle_sensing_available": False,
        },
        "limits": {
            "max_linear_mps": 0.10,
            "max_angular_rad_s": 0.4,
            "command_lease_s": 0.50,
            "localization_max_age_s": 0.30,
            "mission_lease_max_s": 900.0,
        },
    }
    return {**payload, "approval_digest": canonical_digest(payload)}


def _owner(tmp_path: Path, *, enabled: bool = True):
    journal = HierarchicalBindingJournal(tmp_path / "binding.sqlite3")
    owner = HierarchicalPhysicalAuthorityOwner(
        enabled=enabled,
        source_sha=SHA,
        deployed_sha=SHA,
        reviewed_sha=SHA if enabled else "",
        journal=journal,
        boot_nonce="test-boot",
    )
    return owner, journal


def _snapshot(*, localization_timestamp_s: float = 100.0) -> dict:
    frontier = FrontierCandidate(
        signature="frontier-001",
        map_id="live-room",
        map_revision="map-revision-001",
        cells=((1, 1), (1, 2), (2, 1)),
        approach_cells=((0, 1),),
        approach_cell=(0, 1),
        approach_x_m=1.25,
        approach_y_m=-0.40,
        clearance_m=0.35,
        path_distance_m=1.60,
        information_gain_m=0.80,
    )
    return build_semantic_world_snapshot(
        mission_id="m7-canonical-001",
        objective="Explore and map the attended bounded room.",
        objective_revision=1,
        decision_generation=1,
        event_generation=0,
        map_id="live-room",
        map_revision="map-revision-001",
        robot_x_m=0.0,
        robot_y_m=0.0,
        robot_yaw_rad=0.0,
        localization_timestamp_s=localization_timestamp_s,
        now_s=100.1,
        frontiers=(frontier,),
        tracks=(),
        next_best_views=(),
        origin_x_m=0.0,
        origin_y_m=0.0,
    )


def _decision(snapshot: dict) -> dict:
    return {
        "schema": "sphero_rvr.semantic_goal.v1",
        "mission_id": snapshot["mission_id"],
        "snapshot_id": snapshot["snapshot_id"],
        "decision_generation": snapshot["decision_generation"],
        "event_generation": snapshot["event_generation"],
        "action": "go_to_frontier",
        "arguments": {"frontier_id": "frontier-001"},
        "rationale": "Select frontier-001 from current bounded evidence.",
    }


def _dispatch(snapshot: dict, authority: dict) -> dict:
    payload = {
        "schema": GOAL_DISPATCH_SCHEMA,
        "mission_id": authority["mission_id"],
        "source_sha": authority["source_sha"],
        "approval_digest": authority["approval_digest"],
        "controller_session": 1,
        "reason": "initial_revalidated_goal",
        "goals": [
            {
                "decision": _decision(snapshot),
                "captured_snapshot": snapshot,
                "current_snapshot": snapshot,
            }
        ],
    }
    return {**payload, "dispatch_digest": canonical_digest(payload)}


def _proposal(authority: dict) -> dict:
    payload = {
        "schema": PHYSICAL_PROPOSAL_SCHEMA,
        "mission_id": authority["mission_id"],
        "objective": "Explore and map the attended bounded room.",
        "objective_revision": 1,
        "requested_object_classes": ["shoe", "person"],
        "source_sha": authority["source_sha"],
        "created_at_s": 99.0,
    }
    proposal = {**payload, "proposal_digest": canonical_digest(payload)}
    authority["proposal_digest"] = proposal["proposal_digest"]
    return proposal


def test_authority_is_default_off_and_all_accepted_evidence_is_required(
    tmp_path: Path,
) -> None:
    owner, journal = _owner(tmp_path, enabled=False)
    with pytest.raises(MissionValidationError, match="disabled by default"):
        owner.activate(_approval(), now_s=100.0)
    journal.close()

    owner, journal = _owner(tmp_path / "enabled")
    tampered = _approval()
    tampered["m7_4_evidence_sha256"] = "c" * 64
    unsigned = dict(tampered)
    unsigned.pop("approval_digest")
    tampered["approval_digest"] = canonical_digest(unsigned)
    with pytest.raises(MissionValidationError, match="all accepted"):
        owner.activate(tampered, now_s=100.0)
    journal.close()


def test_exact_digest_bound_authority_expires_and_relocks(
    tmp_path: Path,
) -> None:
    owner, journal = _owner(tmp_path)
    heartbeat = owner.activate(_approval(), now_s=100.0)

    assert heartbeat["schema"] == AUTHORITY_SCHEMA
    assert heartbeat["motion_authority"] is True
    assert heartbeat["direct_twist_publisher"] is False
    assert heartbeat["restart_resume_allowed"] is False
    assert heartbeat["limits"] == {
        "max_linear_mps": 0.10,
        "max_angular_rad_s": 0.4,
        "command_lease_s": 0.50,
        "localization_max_age_s": 0.30,
        "mission_lease_max_s": 900.0,
    }
    valid, reason = validate_authority_heartbeat(
        heartbeat,
        now_s=100.1,
        received_at_s=100.1,
        source_sha=SHA,
        deployed_sha=SHA,
        reviewed_sha=SHA,
    )
    assert (valid, reason) == (True, "active")

    expired = owner.heartbeat(now_s=401.0)
    assert expired["state"] == "locked"
    assert expired["motion_authority"] is False
    assert [event["kind"] for event in journal.events()] == [
        "authority_activated",
        "authority_relocked",
    ]
    journal.close()


def test_restart_after_interrupted_authority_requires_recovery(
    tmp_path: Path,
) -> None:
    owner, journal = _owner(tmp_path)
    owner.activate(_approval(), now_s=100.0)
    journal.close()  # Model a crash: no relock event was appended.

    restarted_journal = HierarchicalBindingJournal(
        tmp_path / "binding.sqlite3"
    )
    restarted = HierarchicalPhysicalAuthorityOwner(
        enabled=True,
        source_sha=SHA,
        deployed_sha=SHA,
        reviewed_sha=SHA,
        journal=restarted_journal,
        boot_nonce="new-process",
    )
    assert restarted.state == "recovery_required"
    with pytest.raises(MissionValidationError, match="never resumes"):
        restarted.activate(_approval(), now_s=101.0)
    restarted_journal.close()


def test_graceful_relock_still_forbids_approval_replay(
    tmp_path: Path,
) -> None:
    owner, journal = _owner(tmp_path)
    owner.activate(_approval(), now_s=100.0)
    owner.relock(reason="complete", now_s=101.0)
    journal.close()

    reopened = HierarchicalBindingJournal(tmp_path / "binding.sqlite3")
    next_owner = HierarchicalPhysicalAuthorityOwner(
        enabled=True,
        source_sha=SHA,
        deployed_sha=SHA,
        reviewed_sha=SHA,
        journal=reopened,
        boot_nonce="new-process",
    )
    with pytest.raises(MissionValidationError, match="replay is forbidden"):
        next_owner.activate(_approval(), now_s=102.0)
    reopened.close()


def test_bridge_heartbeat_fails_closed_on_staleness_or_sha_mismatch(
    tmp_path: Path,
) -> None:
    owner, journal = _owner(tmp_path)
    heartbeat = owner.activate(_approval(), now_s=100.0)

    assert validate_authority_heartbeat(
        heartbeat,
        now_s=100.31,
        received_at_s=100.0,
        source_sha=SHA,
        deployed_sha=SHA,
        reviewed_sha=SHA,
    ) == (False, "authority_heartbeat_stale")
    assert validate_authority_heartbeat(
        heartbeat,
        now_s=100.1,
        received_at_s=100.1,
        source_sha=SHA,
        deployed_sha=SHA,
        reviewed_sha="d" * 40,
    ) == (False, "authority_sha_mismatch")
    journal.close()


def test_dispatch_uses_server_geometry_and_rejects_model_geometry(
    tmp_path: Path,
) -> None:
    owner, journal = _owner(tmp_path)
    authority = owner.activate(_approval(), now_s=100.0)
    snapshot = _snapshot()
    batch = resolve_goal_dispatch(
        _dispatch(snapshot, authority),
        authority=authority,
        now_s=100.1,
    )

    assert len(batch.poses) == 1
    assert batch.poses[0].x_m == pytest.approx(1.25)
    assert batch.poses[0].y_m == pytest.approx(-0.40)
    assert batch.to_json_dict()["twist_publisher"] is False

    injected = _dispatch(snapshot, authority)
    injected["goals"][0]["decision"]["arguments"]["x_m"] = 99.0
    unsigned = dict(injected)
    unsigned.pop("dispatch_digest")
    injected["dispatch_digest"] = canonical_digest(unsigned)
    with pytest.raises(MissionValidationError, match="arguments are invalid"):
        resolve_goal_dispatch(
            injected, authority=authority, now_s=100.1
        )
    journal.close()


def test_physical_proposal_is_semantic_only_and_authority_bound(
    tmp_path: Path,
) -> None:
    owner, journal = _owner(tmp_path)
    authority = owner.activate(_approval(), now_s=100.0)
    proposal = _proposal(authority)
    assert validate_physical_proposal(
        proposal, authority=authority, source_sha=SHA
    )["requested_object_classes"] == ["shoe", "person"]

    injected = dict(proposal)
    injected["x_m"] = 2.0
    with pytest.raises(MissionValidationError, match="unreviewed fields"):
        validate_physical_proposal(
            injected, authority=authority, source_sha=SHA
        )
    journal.close()


def test_replay_proven_async_engine_exports_only_bound_semantic_dispatch(
    tmp_path: Path,
) -> None:
    owner, journal = _owner(tmp_path)
    authority = owner.activate(_approval(), now_s=100.0)
    snapshot = _snapshot()
    snapshot["observed_at_s"] = 100.1
    controller = AsyncSemanticGoalController(
        ScriptedSemanticGoalProvider([])
    )
    try:
        controller.start(_decision(snapshot), snapshot, now_s=100.1)
        dispatch = build_goal_dispatch(
            controller,
            authority=authority,
            current_snapshot=snapshot,
            controller_session=1,
            reason="initial_revalidated_goal",
        )
        assert dispatch["goals"][0]["decision"]["arguments"] == {
            "frontier_id": "frontier-001"
        }
        assert "x_m" not in dispatch["goals"][0]["decision"]
        assert dispatch["source_sha"] == SHA
        assert resolve_goal_dispatch(
            dispatch, authority=authority, now_s=100.1
        ).poses[0].x_m == pytest.approx(1.25)
    finally:
        controller.close()
        journal.close()


def test_dispatch_revalidates_current_freshness_and_frontier_identity(
    tmp_path: Path,
) -> None:
    owner, journal = _owner(tmp_path)
    authority = owner.activate(_approval(), now_s=100.0)
    captured = _snapshot()
    stale = json.loads(json.dumps(captured))
    stale["safety"]["motion_evidence_fresh"] = False
    dispatch = _dispatch(captured, authority)
    dispatch["goals"][0]["current_snapshot"] = stale
    unsigned = dict(dispatch)
    unsigned.pop("dispatch_digest")
    dispatch["dispatch_digest"] = canonical_digest(unsigned)

    with pytest.raises(MissionValidationError, match="stale or invalid"):
        resolve_goal_dispatch(dispatch, authority=authority, now_s=100.1)
    journal.close()


def test_localization_0297610_passes_but_0301_fails_closed(
    tmp_path: Path,
) -> None:
    owner, journal = _owner(tmp_path)
    authority = owner.activate(_approval(), now_s=100.0)
    snapshot = _snapshot()
    dispatch = _dispatch(snapshot, authority)
    dispatch["goals"][0]["current_snapshot"]["localization"]["age_s"] = (
        0.297610
    )
    unsigned = dict(dispatch)
    unsigned.pop("dispatch_digest")
    dispatch["dispatch_digest"] = canonical_digest(unsigned)
    assert resolve_goal_dispatch(
        dispatch, authority=authority, now_s=100.1
    ).poses

    dispatch["goals"][0]["current_snapshot"]["localization"]["age_s"] = 0.301
    unsigned = dict(dispatch)
    unsigned.pop("dispatch_digest")
    dispatch["dispatch_digest"] = canonical_digest(unsigned)
    with pytest.raises(MissionValidationError, match="0.300 s"):
        resolve_goal_dispatch(
            dispatch, authority=authority, now_s=100.1
        )
    journal.close()


def test_physical_launch_and_unit_are_default_off_and_non_bootable() -> None:
    launch = (
        REPO_ROOT
        / "launch"
        / "hierarchical_exploration_physical.launch.py"
    ).read_text()
    unit = (
        REPO_ROOT
        / "systemd"
        / "user"
        / "rvr-hierarchical-mission.service"
    ).read_text()
    route = (
        REPO_ROOT
        / "src"
        / "sphero_rvr_driver"
        / "live_route_runner_node.py"
    ).read_text()

    assert launch.count('default_value="false"') >= 5
    assert '"use_sim_time": False' in launch
    assert '"camera_info_url": camera_info_url' in launch
    assert "rvr_pi_imx708_calibrated_800x600.yaml" in launch
    assert '("cmd_vel", "/nav2_cmd_vel_request")' in launch
    assert '"nav2_cmd_lease_s": 0.50' in launch
    assert "WantedBy=" not in unit
    assert "Restart=no" in unit
    assert "RVR_HIERARCHICAL_M7_6_APPROVED" in unit
    assert "RVR_HIERARCHICAL_PROPOSAL_FILE" in unit
    assert 'executable="hierarchical_mission_controller"' in launch
    assert 'executable="stationary_perception"' in launch
    assert "mission_lease_valid=authority_valid" in route
    assert "hierarchical_physical_binding_enabled" in route


def test_nav2_adapter_has_no_twist_or_direct_motor_surface() -> None:
    source = (
        REPO_ROOT
        / "src"
        / "sphero_rvr_driver"
        / "hierarchical_nav2_adapter_node.py"
    ).read_text()
    authority = (
        REPO_ROOT
        / "src"
        / "sphero_rvr_driver"
        / "hierarchical_authority_node.py"
    ).read_text()
    controller = (
        REPO_ROOT
        / "src"
        / "sphero_rvr_driver"
        / "hierarchical_mission_node.py"
    ).read_text()

    for candidate in (source, authority, controller):
        assert "Twist" not in candidate
        assert "/cmd_vel" not in candidate
        assert "serial" not in candidate.lower()
    assert "GOAL_DISPATCH_TOPIC" in controller
    assert "AsyncSemanticGoalController" in controller
    assert "detect_frontiers" in controller
    assert "HierarchicalBindingJournal" in controller


def test_controller_keeps_authority_heartbeat_live_during_map_processing() -> None:
    controller = (
        REPO_ROOT
        / "src"
        / "sphero_rvr_driver"
        / "hierarchical_mission_node.py"
    ).read_text()
    authority = (
        REPO_ROOT
        / "src"
        / "sphero_rvr_driver"
        / "hierarchical_authority_node.py"
    ).read_text()

    assert "MutuallyExclusiveCallbackGroup" in controller
    assert "callback_group=self._authority_callbacks" in controller
    assert "MultiThreadedExecutor(num_threads=2)" in controller
    assert "AUTHORITY_HEARTBEAT_MAX_AGE_S = 0.30" in (
        REPO_ROOT
        / "src"
        / "sphero_rvr_driver"
        / "hierarchical_physical_binding.py"
    ).read_text()
    assert "if rclpy.ok(context=self.context):" in authority


def test_browser_projection_exposes_installed_but_locked_binding(
    tmp_path: Path,
) -> None:
    service = MissionService(
        tmp_path / "missions.sqlite3",
        source_sha=SHA,
        deployed_sha=SHA,
        mode="live",
        live_execution_enabled=False,
    )
    service.hierarchical_physical_binding = {
        "installed": True,
        "state": "locked",
        "m7_6_execution_approved": False,
        "canonical_mission_approved": False,
        "motion_authority": False,
        "physical_execution_enabled": False,
        "restart_resume_allowed": False,
    }
    controller = PromptMissionController(
        service,
        PromptDrivePlanner(
            _UnusedPromptProvider(),
            limits=PromptDriveLimits(),
            source_sha=SHA,
        ),
        execution_enabled=False,
    )

    snapshot = controller.service_snapshot()
    binding = snapshot["hierarchical_physical_binding"]
    assert binding["installed"] is True
    assert binding["state"] == "locked"
    assert binding["motion_authority"] is False
    assert binding["m7_6_execution_approved"] is False

    class Client:
        def service_snapshot(self):
            return snapshot

        def latest_prompt_status(self, session_id):
            del session_id
            return None

    web = LiveMissionWebAdapter(
        Client(), session_id="m7-browser", operator="attended-operator"
    ).snapshot()
    projected = web["adapter"]["hierarchical_physical_binding"]
    assert projected["installed"] is True
    assert projected["state"] == "locked"
    assert web["approval"]["enabled"] is False
    controller.close()
    service.close()
