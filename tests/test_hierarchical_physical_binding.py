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
from sphero_rvr_driver.hierarchical_mission_node import (
    adapter_recovery_reason,
    adapter_remaining_distance,
    goal_dispatch_queue_key,
    live_semantic_track_signature,
    semantic_target_invalidation_reason,
    updated_semantic_rejection_count,
)
from sphero_rvr_driver.hierarchical_nav2_adapter_node import (
    controller_status_cancel_mode,
    nav2_result_state,
    stronger_cancel_reason,
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


def test_controller_ignores_placeholder_zero_until_nav2_reports_success() -> None:
    fallback = 1.25
    assert adapter_remaining_distance(
        {
            "state": "dispatching",
            "goal_active": False,
            "distance_remaining_m": 0.0,
        },
        fallback,
    ) == pytest.approx(fallback)
    assert adapter_remaining_distance(
        {
            "state": "navigating",
            "goal_active": True,
            "distance_remaining_m": 0.72,
        },
        fallback,
    ) == pytest.approx(0.72)
    assert adapter_remaining_distance(
        {
            "state": "wait_planning",
            "reason": "nav2_result_status_4",
            "goal_active": False,
            "distance_remaining_m": 0.0,
        },
        fallback,
    ) == pytest.approx(0.0)


def test_controller_fails_closed_on_nav2_recovery_status() -> None:
    assert (
        adapter_recovery_reason(
            {
                "state": "recovery_required",
                "reason": "nav2_result_status_6",
            }
        )
        == "nav2_result_status_6"
    )
    assert (
        adapter_recovery_reason(
            {"state": "wait_planning", "reason": "nav2_result_status_4"}
        )
        == ""
    )


def test_dispatch_queue_key_ignores_refresh_but_tracks_semantic_change() -> None:
    authority = _approval()
    snapshot = _snapshot()
    first = _dispatch(snapshot, authority)
    refreshed = json.loads(json.dumps(first))
    refreshed["reason"] = "navigating"
    refreshed["goals"][0]["current_snapshot"]["map_revision"] = (
        "map-revision-002"
    )
    assert goal_dispatch_queue_key(refreshed) == goal_dispatch_queue_key(first)

    changed = json.loads(json.dumps(first))
    changed["goals"][0]["decision"]["decision_generation"] = 2
    assert goal_dispatch_queue_key(changed) != goal_dispatch_queue_key(first)


def test_live_track_signature_ignores_rolling_frame_evidence() -> None:
    raw = {
        "track_id": "object-0003",
        "kind": "object",
        "label": "possible_shoe",
        "x_m": 0.8,
        "y_m": -0.2,
        "last_seen_s": 100.0,
        "observation_count": 4,
        "evidence_ids": ["frame-1", "frame-2"],
        "recognized_from_enrollment": False,
        "enrollment_evidence_ids": [],
    }
    rolling = json.loads(json.dumps(raw))
    rolling["x_m"] = 0.83
    rolling["last_seen_s"] = 101.0
    rolling["observation_count"] = 5
    rolling["evidence_ids"].append("frame-3")
    assert live_semantic_track_signature(rolling) == (
        live_semantic_track_signature(raw)
    )

    relabeled = json.loads(json.dumps(raw))
    relabeled["label"] = "backpack"
    assert live_semantic_track_signature(relabeled) != (
        live_semantic_track_signature(raw)
    )


def test_active_semantic_target_invalidation_is_not_frontier_only() -> None:
    snapshot = _snapshot()
    controller = AsyncSemanticGoalController(
        ScriptedSemanticGoalProvider([])
    )
    try:
        controller.start(_decision(snapshot), snapshot, now_s=100.1)
        goal, captured = controller.resolved_motion_goals()[0]
        invalidated = json.loads(json.dumps(snapshot))
        invalidated["frontiers"] = []
        assert semantic_target_invalidation_reason(
            goal, captured, invalidated
        ) == "frontier_signature_invalidated"
    finally:
        controller.close()


def test_semantic_revalidation_churn_is_bounded_and_success_resets() -> None:
    count = 0
    for _ in range(3):
        count = updated_semantic_rejection_count(
            count,
            ({"kind": "prefetch_discarded"},),
        )
    assert count == 3
    assert count >= 3
    assert (
        updated_semantic_rejection_count(
            count,
            ({"kind": "prefetch_revalidated"},),
        )
        == 0
    )


def test_physical_nav2_clears_only_its_declared_footprint() -> None:
    nav2 = (
        REPO_ROOT / "config" / "hierarchical_nav2_physical.yaml"
    ).read_text()
    collision = (REPO_ROOT / "config" / "collision_stop.yaml").read_text()

    assert nav2.count("footprint_clearing_enabled: true") == 3
    assert "restore_cleared_footprint: true" in nav2
    assert nav2.count("robot_radius: 0.22") == 2
    assert nav2.count("topic: /scan") == 2
    assert "min_range_m: 0.08" in collision
    assert "footprint_front_m: 0.22" in collision
    assert "footprint_rear_m: 0.16" in collision


def test_nav2_adapter_binds_feedback_and_results_to_active_batch() -> None:
    source = (
        REPO_ROOT
        / "src"
        / "sphero_rvr_driver"
        / "hierarchical_nav2_adapter_node.py"
    ).read_text()

    assert "digest != self._active_batch_digest" in source
    assert source.count("recovery_required") >= 5
    assert '"nav2_action_unavailable"' in source
    assert '"nav2_goal_rejected"' in source
    controller = (
        REPO_ROOT
        / "src"
        / "sphero_rvr_driver"
        / "hierarchical_mission_node.py"
    ).read_text()
    assert "MAX_CONSECUTIVE_SEMANTIC_REJECTIONS = 3" in controller
    assert "semantic_revalidation_exhausted" in controller
    assert "_initial_pool.shutdown(wait=True" in controller


def test_controller_replan_status_can_only_cancel_nav2() -> None:
    base = {
        "schema": "sphero_rvr.hierarchical_controller_status.v1",
        "source_sha": SHA,
        "mission_id": "m7-canonical-001",
        "state": "wait_planning",
        "reason": "event_triggered_replan",
    }
    assert controller_status_cancel_mode(
        base,
        source_sha=SHA,
        mission_id="m7-canonical-001",
    ) == "replan"
    stale_localization = {
        **base,
        "reason": "live localization exceeds the fixed 0.300 s gate",
    }
    assert controller_status_cancel_mode(
        stale_localization,
        source_sha=SHA,
        mission_id="m7-canonical-001",
    ) == "replan"

    navigating = {**base, "state": "dispatching", "reason": "navigating"}
    assert controller_status_cancel_mode(
        navigating,
        source_sha=SHA,
        mission_id="m7-canonical-001",
    ) == ""

    mismatched = {**base, "source_sha": "b" * 40}
    assert controller_status_cancel_mode(
        mismatched,
        source_sha=SHA,
        mission_id="m7-canonical-001",
    ) == "veto"

    recovery = {
        **base,
        "state": "recovery_required",
        "reason": "semantic_revalidation_exhausted",
    }
    assert controller_status_cancel_mode(
        recovery,
        source_sha=SHA,
        mission_id="m7-canonical-001",
    ) == "veto"
    complete = {
        **base,
        "state": "complete",
        "reason": "finish",
    }
    assert controller_status_cancel_mode(
        complete,
        source_sha=SHA,
        mission_id="m7-canonical-001",
    ) == "complete"

    source = (
        REPO_ROOT
        / "src"
        / "sphero_rvr_driver"
        / "hierarchical_nav2_adapter_node.py"
    ).read_text()
    assert "_cancel_for_replan" in source
    assert "_cancel_for_completion" in source
    assert '"controller_replan_cancelled"' in source
    assert "CONTROLLER_STATUS_TOPIC" in source
    assert nav2_result_state(4, "") == (
        "wait_planning",
        "nav2_result_status_4",
    )
    assert nav2_result_state(5, "controller_replan") == (
        "wait_planning",
        "controller_replan_cancelled",
    )
    assert nav2_result_state(5, "controller_complete") == (
        "complete",
        "controller_complete_cancelled",
    )
    assert nav2_result_state(6, "") == (
        "recovery_required",
        "nav2_result_status_6",
    )
    assert stronger_cancel_reason("", "controller_replan") == (
        "controller_replan"
    )
    assert stronger_cancel_reason(
        "controller_replan", "controller_complete"
    ) == "controller_complete"
    assert stronger_cancel_reason("controller_complete", "veto") == "veto"
    assert stronger_cancel_reason("veto", "controller_replan") == "veto"
    assert "_pending_batch_digest" in source
    assert "veto_pending_acceptance" in source


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
