from __future__ import annotations

import subprocess
import threading
import time

import pytest

from sphero_rvr_driver.hierarchical_physical_binding import (
    ACCEPTED_DIRECTIONAL_ADDENDUM_SHA256,
    ACCEPTED_M7_3_EVIDENCE_SHA256,
    ACCEPTED_M7_4_EVIDENCE_SHA256,
    APPROVAL_SCHEMA,
    PHYSICAL_PROPOSAL_SCHEMA,
    HierarchicalBindingJournal,
)
from sphero_rvr_driver.hierarchical_m7_canonical_validation import (
    capture_cleanup_evidence,
    evaluate_canonical_mission,
)
from sphero_rvr_driver.hierarchical_physical_live_controller import (
    CANONICAL_M7_OBJECTIVE,
    HierarchicalPhysicalMissionController,
)
from sphero_rvr_driver.hierarchical_physical_session import (
    HIERARCHICAL_MISSION_UNIT,
    SystemdHierarchicalMissionSession,
)
from sphero_rvr_driver.live_mission_service import LiveStateCache
from sphero_rvr_driver.mission_api import MissionValidationError
from sphero_rvr_driver.mission_service import MissionService
from sphero_rvr_driver.mission_web import LiveMissionWebAdapter


SHA = "a" * 40
ROOM = {
    "attended": True,
    "level_bounded": True,
    "stairs_ledges_dropoffs_absent": True,
    "negative_obstacle_sensing_available": False,
}


def _seed_sensor_preflight(
    cache: LiveStateCache, *, received_at_s: float | None = None
) -> None:
    now_s = time.time() if received_at_s is None else received_at_s
    cache.update(
        "lidar",
        {
            "schema": "sphero_rvr.live_lidar.v1",
            "scan_id": "live-scan-preflight",
            "sample_count": 720,
            "motion_authority": False,
            "physical_execution_enabled": False,
        },
        received_at_s=now_s,
        source_timestamp_s=now_s,
    )
    cache.update(
        "camera",
        {
            "schema": "sphero_rvr.live_camera_perception.v1",
            "frame_id": "live-camera-preflight",
            "width": 800,
            "height": 600,
            "calibrated": True,
            "motion_authority": False,
            "physical_execution_enabled": False,
        },
        received_at_s=now_s,
        source_timestamp_s=now_s,
    )
    cache.update(
        "localization",
        {
            "state": "valid",
            "source": "slam_toolbox:map->base_link",
            "map_id": "live-map-preflight",
            "stationary_session": True,
            "pose": {
                "x_m": 0.0,
                "y_m": 0.0,
                "yaw_rad": 0.0,
            },
            "motion_authority": False,
            "physical_execution_enabled": False,
        },
        received_at_s=now_s,
        source_timestamp_s=now_s,
    )
    cache.update(
        "semantic_map",
        {
            "schema": "sphero_rvr.live_semantic_map.v1",
            "revision": 1,
            "occupancy": {"map_id": "live-map-preflight"},
            "map": {
                "stationary": True,
                "occupancy_available": True,
            },
            "motion_authority": False,
            "physical_execution_enabled": False,
        },
        received_at_s=now_s,
        source_timestamp_s=now_s,
    )


class FakeSession:
    activation_capable = True

    def __init__(self) -> None:
        self.active = False
        self.activations = []
        self.deactivations = []

    def activate(
        self,
        *,
        proposal,
        approval,
        now_s,
        cancel_event=None,
    ):
        if cancel_event is not None and cancel_event.is_set():
            raise MissionValidationError("activation cancelled")
        self.active = True
        self.activations.append((proposal, approval, now_s))
        return self.status()

    def deactivate(self, *, reason):
        self.active = False
        self.deactivations.append(reason)
        return self.status()

    def status(self):
        return {
            "activation_capable": True,
            "active": self.active,
            "transitioning": False,
            "mission_id": "",
            "detail": "active" if self.active else "locked",
            "unit": HIERARCHICAL_MISSION_UNIT,
            "restart_resume_allowed": False,
        }


def _controller(tmp_path):
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
        "reviewed_sha": SHA,
        "m7_3_evidence_sha256": ACCEPTED_M7_3_EVIDENCE_SHA256,
        "directional_addendum_sha256": (
            ACCEPTED_DIRECTIONAL_ADDENDUM_SHA256
        ),
        "m7_4_evidence_sha256": ACCEPTED_M7_4_EVIDENCE_SHA256,
        "motion_authority": False,
    }
    cache = LiveStateCache()
    _seed_sensor_preflight(cache)
    session = FakeSession()
    controller = HierarchicalPhysicalMissionController(
        service,
        cache,
        session,
        execution_enabled=True,
        monitor_period_s=0.05,
    )
    return service, cache, session, controller


def test_canonical_proposal_is_semantic_only_and_durable(tmp_path) -> None:
    service, cache, session, controller = _controller(tmp_path)
    del cache, session
    try:
        proposed = controller.submit(
            CANONICAL_M7_OBJECTIVE,
            session_id="m7-browser",
            source="web",
            mission_id="m7-canonical-test",
            mission_lease_s=900.0,
        )
        proposal = proposed["proposal"]
        assert proposed["status"] == "proposed"
        assert proposal["schema"] == PHYSICAL_PROPOSAL_SCHEMA
        assert proposal["requested_object_classes"] == ["shoe", "person"]
        assert set(proposal) == {
            "schema",
            "mission_id",
            "objective",
            "objective_revision",
            "requested_object_classes",
            "source_sha",
            "created_at_s",
            "proposal_digest",
        }
        assert not any(
            name in proposal
            for name in ("x_m", "y_m", "pose", "route", "cmd_vel")
        )
        assert [
            event["kind"] for event in proposed["events"]
        ] == [
            "received",
            "planning",
            "hierarchical_physical_proposal",
        ]
    finally:
        controller.close()
        service.close()


def test_canonical_approval_requires_auth_room_and_exact_proposal(
    tmp_path,
) -> None:
    service, cache, session, controller = _controller(tmp_path)
    del cache, session
    try:
        proposed = controller.submit(
            CANONICAL_M7_OBJECTIVE,
            session_id="m7-browser",
            mission_id="m7-canonical-test",
        )
        phrase = (
            "APPROVE M7.6 CANONICAL MISSION "
            + proposed["proposal_digest"]
        )
        with pytest.raises(
            MissionValidationError, match="authenticated Tailscale"
        ):
            controller.approve(
                proposed["mission_id"],
                supplied_approval=phrase,
                operator="scott",
                physical_room_confirmation=ROOM,
            )
        with pytest.raises(
            MissionValidationError, match="room confirmation"
        ):
            controller.approve(
                proposed["mission_id"],
                supplied_approval=phrase,
                operator="scott",
                authentication_source="tailscale-serve",
                physical_room_confirmation={
                    **ROOM,
                    "stairs_ledges_dropoffs_absent": False,
                },
            )
        with pytest.raises(
            MissionValidationError, match="current canonical proposal"
        ):
            controller.approve(
                proposed["mission_id"],
                supplied_approval="APPROVE M7.6 CANONICAL MISSION " + "b" * 64,
                operator="scott",
                authentication_source="tailscale-serve",
                physical_room_confirmation=ROOM,
            )
    finally:
        controller.close()
        service.close()


def test_canonical_approval_requires_fresh_no_motion_sensor_preflight(
    tmp_path,
) -> None:
    service, cache, session, controller = _controller(tmp_path)
    del session
    try:
        proposed = controller.submit(
            CANONICAL_M7_OBJECTIVE,
            session_id="m7-browser",
            mission_id="m7-canonical-preflight",
        )
        phrase = (
            "APPROVE M7.6 CANONICAL MISSION "
            + proposed["proposal_digest"]
        )
        cache.mark_invalid(
            "camera",
            "camera unavailable",
            received_at_s=time.time(),
        )
        with pytest.raises(
            MissionValidationError, match="valid camera evidence"
        ):
            controller.approve(
                proposed["mission_id"],
                supplied_approval=phrase,
                operator="scott",
                authentication_source="tailscale-serve",
                physical_room_confirmation=ROOM,
            )
        _seed_sensor_preflight(
            cache, received_at_s=time.time() - 1.01
        )
        with pytest.raises(
            MissionValidationError, match="lidar evidence is stale"
        ):
            controller.approve(
                proposed["mission_id"],
                supplied_approval=phrase,
                operator="scott",
                authentication_source="tailscale-serve",
                physical_room_confirmation=ROOM,
            )
    finally:
        controller.close()
        service.close()


def test_cancel_during_activation_cannot_start_or_resume_graph(
    tmp_path,
) -> None:
    service = MissionService(
        tmp_path / "missions.sqlite3",
        source_sha=SHA,
        deployed_sha=SHA,
        mode="live",
        live_execution_enabled=False,
    )
    cache = LiveStateCache()
    _seed_sensor_preflight(cache)

    class BlockingSession(FakeSession):
        def __init__(self):
            super().__init__()
            self.entered = threading.Event()

        def activate(
            self,
            *,
            proposal,
            approval,
            now_s,
            cancel_event=None,
        ):
            del proposal, approval, now_s
            self.entered.set()
            assert cancel_event is not None
            cancel_event.wait(1.0)
            if cancel_event.is_set():
                raise MissionValidationError("activation cancelled")
            self.active = True
            return self.status()

    session = BlockingSession()
    controller = HierarchicalPhysicalMissionController(
        service,
        cache,
        session,
        execution_enabled=True,
        monitor_period_s=0.05,
    )
    try:
        proposed = controller.submit(
            CANONICAL_M7_OBJECTIVE,
            session_id="m7-browser",
            mission_id="m7-canonical-cancel",
        )
        controller.approve(
            proposed["mission_id"],
            supplied_approval=(
                "APPROVE M7.6 CANONICAL MISSION "
                + proposed["proposal_digest"]
            ),
            operator="scott",
            authentication_source="tailscale-serve",
            physical_room_confirmation=ROOM,
        )
        assert session.entered.wait(1.0)
        cancelled = controller.cancel(proposed["mission_id"])
        assert cancelled["status"] == "cancelled"
        assert session.active is False
        time.sleep(0.05)
        assert controller.status(proposed["mission_id"])["status"] == (
            "cancelled"
        )
    finally:
        controller.close()
        service.close()


def test_approval_binds_all_evidence_limits_and_terminal_cleanup(
    tmp_path,
) -> None:
    service, cache, session, controller = _controller(tmp_path)
    try:
        proposed = controller.submit(
            CANONICAL_M7_OBJECTIVE,
            session_id="m7-browser",
            mission_id="m7-canonical-test",
        )
        approved = controller.approve(
            proposed["mission_id"],
            supplied_approval=(
                "APPROVE M7.6 CANONICAL MISSION "
                + proposed["proposal_digest"]
            ),
            operator="scott@example.com",
            authentication_source="tailscale-serve",
            physical_room_confirmation=ROOM,
        )
        approval = approved["approval"]
        assert approval["schema"] == APPROVAL_SCHEMA
        assert approval["m7_3_evidence_sha256"] == (
            ACCEPTED_M7_3_EVIDENCE_SHA256
        )
        assert approval["directional_addendum_sha256"] == (
            ACCEPTED_DIRECTIONAL_ADDENDUM_SHA256
        )
        assert approval["m7_4_evidence_sha256"] == (
            ACCEPTED_M7_4_EVIDENCE_SHA256
        )
        assert approval["room"] == ROOM
        assert approval["limits"] == {
            "max_linear_mps": 0.10,
            "max_angular_rad_s": 0.4,
            "command_lease_s": 0.50,
            "localization_max_age_s": 0.30,
            "mission_lease_max_s": 900.0,
        }
        deadline = time.monotonic() + 2.0
        while not session.active and time.monotonic() < deadline:
            time.sleep(0.01)
        cache.update(
            "hierarchical_controller",
            {
                "schema": "sphero_rvr.hierarchical_controller_status.v1",
                "mission_id": proposed["mission_id"],
                "state": "complete",
                "reason": "return_to_origin",
                "source_sha": SHA,
                "direct_twist_publisher": False,
            },
            received_at_s=time.time(),
        )
        terminal = approved
        while (
            terminal["status"] not in {"complete", "recovery_required"}
            and time.monotonic() < deadline
        ):
            time.sleep(0.02)
            terminal = controller.status(proposed["mission_id"])
        assert terminal["status"] == "complete"
        assert terminal["result"]["cleanup_verified"] is True
        assert session.active is False
        assert session.deactivations
        assert any(
            event["kind"] == "hierarchical_checkpoint"
            for event in terminal["events"]
        )
    finally:
        controller.close()
        service.close()


def test_browser_creates_and_approves_canonical_mission_without_hash_entry(
    tmp_path,
) -> None:
    service, cache, session, controller = _controller(tmp_path)

    class Client:
        def service_snapshot(self):
            return controller.service_snapshot()

        def latest_prompt_status(self, session_id):
            return service.latest_prompt_status(session_id)

        def prompt_status(self, mission_id):
            return controller.status(mission_id)

        def submit_prompt(self, prompt, **kwargs):
            return controller.submit(prompt, **kwargs)

        def approve_prompt(self, mission_id, **kwargs):
            return controller.approve(
                mission_id,
                supplied_approval=kwargs["approval_phrase"],
                operator=kwargs["operator"],
                authentication_source=kwargs["authentication_source"],
                physical_room_confirmation=kwargs[
                    "physical_room_confirmation"
                ],
            )

    try:
        browser = LiveMissionWebAdapter(
            Client(),
            session_id="m7-browser",
            operator="fallback",
        )
        browser.set_request_identity(
            "scott@example.com", authenticated=True
        )
        proposed = browser.propose(
            CANONICAL_M7_OBJECTIVE, "live", mission_lease_s=900.0
        )
        assert proposed["adapter"]["hierarchical_canonical"] is True
        assert proposed["approval"]["enabled"] is True
        approved = browser.approve(
            "",
            confirm_current_proposal=True,
            physical_room_confirmation=ROOM,
        )
        assert approved["mission"]["state"] in {"APPROVED", "QUEUED"}
        assert approved["approval"]["required_phrase"] == ""
        assert session.activations or approved["mission"]["state"] == "APPROVED"
        reopened = browser.reopen(proposed["mission"]["mission_id"])
        assert reopened["mission"]["mission_id"] == (
            proposed["mission"]["mission_id"]
        )
    finally:
        controller.cancel(
            service.latest_prompt_status("m7-browser")["mission_id"]
        )
        controller.close()
        service.close()


def test_systemd_session_files_are_private_and_consumed_on_relock(
    tmp_path,
) -> None:
    state = {"active": False}

    def runner(command, **kwargs):
        del kwargs
        if command[:3] == ["systemctl", "--user", "start"]:
            state["active"] = True
        elif command[:3] == ["systemctl", "--user", "stop"]:
            if command[-1] == HIERARCHICAL_MISSION_UNIT:
                state["active"] = False
        if command[:3] == ["systemctl", "--user", "show"]:
            active = "active" if state["active"] else "inactive"
            sub = "running" if state["active"] else "dead"
            return subprocess.CompletedProcess(
                command,
                0,
                stdout=(
                    "LoadState=loaded\n"
                    f"ActiveState={active}\n"
                    f"SubState={sub}\n"
                    "Result=success\n"
                ),
                stderr="",
            )
        return subprocess.CompletedProcess(
            command, 0, stdout="", stderr=""
        )

    session = SystemdHierarchicalMissionSession(
        activation_capable=True,
        source_sha=SHA,
        deployed_sha=SHA,
        reviewed_sha=SHA,
        state_directory=tmp_path / "session",
        runner=runner,
    )
    service, cache, fake, controller = _controller(tmp_path / "controller")
    del cache, fake
    try:
        proposed = controller.submit(
            CANONICAL_M7_OBJECTIVE,
            session_id="m7-browser",
            mission_id="m7-canonical-test",
        )
        controller.approve(
            proposed["mission_id"],
            supplied_approval=(
                "APPROVE M7.6 CANONICAL MISSION "
                + proposed["proposal_digest"]
            ),
            operator="scott",
            authentication_source="tailscale-serve",
            physical_room_confirmation=ROOM,
        )
        approval = service.prompt_status(
            proposed["mission_id"]
        )["approval"]
        session.activate(
            proposal=proposed["proposal"],
            approval=approval,
            now_s=time.time(),
            cancel_event=None,
        )
        assert session.status()["active"] is True
        assert oct(session.environment_path.stat().st_mode & 0o777) == "0o600"
        assert "RVR_HIERARCHICAL_M7_6_APPROVED=\"true\"" in (
            session.environment_path.read_text()
        )
        session.deactivate(reason="test complete")
        assert session.status()["active"] is False
        assert session.environment_path.exists() is False
        assert session.approval_path.exists() is False
        assert session.proposal_path.exists() is False
    finally:
        controller.cancel(proposed["mission_id"])
        controller.close()
        service.close()


def test_canonical_evaluator_recomputes_motion_goals_authority_and_cleanup(
    tmp_path,
) -> None:
    service, cache, session, controller = _controller(tmp_path)
    mission_id = "m7-canonical-evaluator"
    try:
        proposed = controller.submit(
            CANONICAL_M7_OBJECTIVE,
            session_id="m7-browser",
            mission_id=mission_id,
        )
        approved = controller.approve(
            mission_id,
            supplied_approval=(
                "APPROVE M7.6 CANONICAL MISSION "
                + proposed["proposal_digest"]
            ),
            operator="scott@example.com",
            authentication_source="tailscale-serve",
            physical_room_confirmation=ROOM,
        )
        deadline = time.monotonic() + 3.0
        while not session.active and time.monotonic() < deadline:
            time.sleep(0.01)
        now = time.time()
        cache.update(
            "odom",
            {
                "x_m": 0.0,
                "y_m": 0.0,
                "linear_mps": 0.0,
                "angular_rad_s": 0.0,
            },
            received_at_s=now,
        )
        cache.update(
            "localization",
            {"pose": {"x_m": 0.0, "y_m": 0.0}},
            received_at_s=now,
        )
        time.sleep(0.07)
        now = time.time()
        cache.update(
            "odom",
            {
                "x_m": 0.04,
                "y_m": 0.0,
                "linear_mps": 0.05,
                "angular_rad_s": 0.0,
            },
            received_at_s=now,
        )
        cache.update(
            "localization",
            {"pose": {"x_m": 0.04, "y_m": 0.0}},
            received_at_s=now,
        )
        time.sleep(0.07)
        cache.update(
            "hierarchical_controller",
            {
                "schema": "sphero_rvr.hierarchical_controller_status.v1",
                "mission_id": mission_id,
                "state": "complete",
                "reason": "return_to_origin",
                "source_sha": SHA,
                "direct_twist_publisher": False,
            },
            received_at_s=time.time(),
        )
        terminal = approved
        while (
            terminal["status"] not in {"complete", "recovery_required"}
            and time.monotonic() < deadline
        ):
            time.sleep(0.02)
            terminal = controller.status(mission_id)
        assert terminal["status"] == "complete"

        journal_path = tmp_path / "binding.sqlite3"
        journal = HierarchicalBindingJournal(journal_path)
        approval = terminal["approval"]
        journal.append(
            mission_id,
            "authority_activated",
            {
                "approval_digest": approval["approval_digest"],
                "motion_authority": True,
            },
            recorded_at_s=approval["approved_at_s"],
        )
        journal.append(
            mission_id,
            "provider_call_completed",
            {
                "provider_elapsed_s": 11.02,
                "real_provider": True,
            },
            recorded_at_s=approval["approved_at_s"] + 0.1,
        )
        for index, (action, arguments) in enumerate(
            (
                ("go_to_frontier", {"frontier_id": "frontier-001"}),
                ("return_to_origin", {}),
            ),
            start=1,
        ):
            journal.append(
                mission_id,
                "goal_dispatch",
                {
                    "goals": [
                        {
                            "decision": {
                                "action": action,
                                "arguments": arguments,
                                "rationale": f"reviewed semantic goal {index}",
                            },
                            "current_snapshot": {"tracks": []},
                        }
                    ]
                },
                recorded_at_s=approval["approved_at_s"] + index,
            )
        journal.append(
            mission_id,
            "controller_event",
            {"kind": "atomic_handoff", "at_s": 2.0},
            recorded_at_s=approval["approved_at_s"] + 2.0,
        )
        journal.append(
            mission_id,
            "authority_relocked",
            {"reason": "complete", "motion_authority": False},
            recorded_at_s=approval["approved_at_s"] + 3.0,
        )
        journal.close()

        def cleanup_runner(command, **kwargs):
            del kwargs
            if command[:3] == ["systemctl", "--user", "show"]:
                return subprocess.CompletedProcess(
                    command,
                    0,
                    stdout="ActiveState=inactive\nSubState=dead\n",
                    stderr="",
                )
            if "topic" in command:
                return subprocess.CompletedProcess(
                    command, 1, stdout="", stderr="not found"
                )
            return subprocess.CompletedProcess(
                command, 0, stdout="", stderr=""
            )

        cleanup = capture_cleanup_evidence(
            runner=cleanup_runner,
            session_directory=tmp_path / "empty-session",
        )
        report = evaluate_canonical_mission(
            mission_database=service.database,
            binding_journal=journal_path,
            mission_id=mission_id,
            cleanup_capture=cleanup,
        )
        assert report["passed"] is True
        assert all(report["checks"].values())
        assert len(report["evidence"]["semantic_decisions"]) == 2
    finally:
        controller.close()
        service.close()
