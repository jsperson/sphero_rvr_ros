from __future__ import annotations

import json
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
    canonical_digest,
)
from sphero_rvr_driver.hierarchical_m7_canonical_validation import (
    _cleanup_checks,
    _wait_planning_intervals,
    active_graph_checks,
    capture_active_graph_evidence,
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
from sphero_rvr_driver.mission_web import (
    LiveMissionWebAdapter,
    _is_adaptive_preapproval,
)


SHA = "a" * 40
ROOM = {
    "attended": True,
    "level_bounded": True,
    "stairs_ledges_dropoffs_absent": True,
    "negative_obstacle_sensing_available": False,
}


def _active_graph_capture():
    def runner(command, **kwargs):
        del kwargs
        stdout = ""
        returncode = 0
        if command[0] == "git" and command[-2:] == [
            "rev-parse",
            "HEAD",
        ]:
            stdout = SHA + "\n"
        elif command[0] == "git":
            stdout = ""
        elif command[-4:-1] == ["list", "--spin-time", "3.0"]:
            stdout = "\n".join(
                (
                    "/sphero_rvr_driver",
                    "/lidar_collision_stop_supervisor",
                    "/live_route_runner",
                    "/controller_server",
                    "/planner_server",
                    "/hierarchical_physical_authority",
                    "/hierarchical_mission_controller",
                    "/hierarchical_nav2_adapter",
                )
            )
        elif command[-1] == "/cmd_vel":
            stdout = (
                "Publisher count: 1\n"
                "Node name: /live_route_runner\n"
                "Subscription count: 1\n"
                "Node name: /lidar_collision_stop_supervisor\n"
            )
        elif command[-1] == "/cmd_vel_motor":
            stdout = (
                "Publisher count: 1\n"
                "Node name: /lidar_collision_stop_supervisor\n"
                "Subscription count: 1\n"
                "Node name: /sphero_rvr_driver\n"
            )
        elif command[-1] == "/nav2_cmd_vel_request":
            stdout = (
                "Publisher count: 2\n"
                "Node name: /controller_server\n"
                "Node name: /behavior_server\n"
                "Subscription count: 1\n"
                "Node name: /live_route_runner\n"
            )
        elif command[0] == "fuser":
            stdout = "1234"
        return subprocess.CompletedProcess(
            command, returncode, stdout=stdout, stderr=""
        )

    return capture_active_graph_evidence(
        source_sha=SHA,
        source_repository="/reviewed/source",
        runner=runner,
        captured_at_s=100.0,
    )


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
            "active_graph_capture": (
                _active_graph_capture() if self.active else {}
            ),
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
        now = time.time()
        _seed_sensor_preflight(cache, received_at_s=now)
        camera = cache.snapshot(now_s=now).source("camera")
        cache.update(
            "camera",
            dict(camera.value),
            received_at_s=now,
            source_timestamp_s=now - 1.001,
        )
        with pytest.raises(
            MissionValidationError, match="camera evidence is stale"
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
        deadline = time.monotonic() + 1.0
        while controller._threads and time.monotonic() < deadline:
            time.sleep(0.01)
        assert controller._threads == {}
        replacement = controller.submit(
            CANONICAL_M7_OBJECTIVE,
            session_id="m7-browser",
            mission_id="m7-canonical-after-cancel",
        )
        assert replacement["status"] == "proposed"
    finally:
        controller.close()
        service.close()


def test_active_motion_fails_closed_on_stale_localization(
    tmp_path,
) -> None:
    service, cache, session, controller = _controller(tmp_path)
    try:
        proposed = controller.submit(
            CANONICAL_M7_OBJECTIVE,
            session_id="m7-browser",
            mission_id="m7-canonical-stale-localization",
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
        deadline = time.monotonic() + 1.0
        while not session.active and time.monotonic() < deadline:
            time.sleep(0.01)
        assert session.active is True
        now = time.time()
        cache.update(
            "localization",
            {"pose": {"x_m": 0.0, "y_m": 0.0}},
            received_at_s=now - 0.301,
            source_timestamp_s=now - 0.301,
        )
        cache.update(
            "hierarchical_adapter",
            {"goal_active": True},
            received_at_s=now,
        )
        terminal = controller.status(proposed["mission_id"])
        while (
            terminal["status"] != "recovery_required"
            and time.monotonic() < deadline
        ):
            time.sleep(0.01)
            terminal = controller.status(proposed["mission_id"])
        assert terminal["status"] == "recovery_required"
        assert terminal["result"]["cleanup_verified"] is True
        assert terminal["result"]["run_evidence"][
            "localization_freshness_violations"
        ] == 1
        assert terminal["result"]["run_evidence"][
            "max_localization_age_s"
        ] > 0.300
        assert session.active is False
    finally:
        controller.close()
        service.close()


@pytest.mark.parametrize(
    ("source_name", "stale_age_s"),
    (
        ("lidar", 0.501),
        ("camera", 1.001),
        ("semantic_map", 1.001),
    ),
)
def test_active_motion_fails_closed_on_any_stale_required_sensor(
    tmp_path,
    source_name,
    stale_age_s,
) -> None:
    service, cache, session, controller = _controller(tmp_path)
    try:
        proposed = controller.submit(
            CANONICAL_M7_OBJECTIVE,
            session_id="m7-browser",
            mission_id=f"m7-canonical-stale-{source_name}",
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
        deadline = time.monotonic() + 1.0
        while not session.active and time.monotonic() < deadline:
            time.sleep(0.01)
        assert session.active is True
        now = time.time()
        prior = cache.snapshot(now_s=now).source(source_name)
        cache.update(
            source_name,
            dict(prior.value),
            received_at_s=now - stale_age_s,
            source_timestamp_s=now,
        )
        cache.update(
            "hierarchical_adapter",
            {"goal_active": True},
            received_at_s=now,
        )
        terminal = controller.status(proposed["mission_id"])
        while (
            terminal["status"] != "recovery_required"
            and time.monotonic() < deadline
        ):
            time.sleep(0.01)
            terminal = controller.status(proposed["mission_id"])
        assert terminal["status"] == "recovery_required"
        evidence = terminal["result"]["run_evidence"]
        assert evidence["required_sensor_freshness_violations"] == 1
        assert evidence["localization_freshness_violations"] == 0
        assert (
            evidence["max_required_sensor_age_s"][source_name]
            > stale_age_s - 0.001
        )
        assert session.active is False
    finally:
        controller.close()
        service.close()


def test_controller_close_relocks_active_session_and_never_resumes(
    tmp_path,
) -> None:
    service, cache, session, controller = _controller(tmp_path)
    del cache
    proposed = controller.submit(
        CANONICAL_M7_OBJECTIVE,
        session_id="m7-browser",
        mission_id="m7-canonical-service-shutdown",
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
    deadline = time.monotonic() + 1.0
    while not session.active and time.monotonic() < deadline:
        time.sleep(0.01)
    assert session.active is True
    controller.close(timeout_s=1.0)
    assert session.active is False
    terminal = service.prompt_status(proposed["mission_id"])
    assert terminal["status"] == "recovery_required"
    assert terminal["result"]["cleanup_verified"] is True
    assert terminal["result"]["restart_resume_allowed"] is False
    controller.close(timeout_s=1.0)
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
    old_adaptive_status = {
        "mission_id": "adaptive-mission-old",
        "session_id": "m7-browser",
        "status": "recovery_required",
        "proposal": {
            "schema": "sphero_rvr.adaptive_mission_proposal.v1",
            "prompt": "Old adaptive objective",
        },
        "approval": {},
        "result": {
            "schema": "sphero_rvr.adaptive_mission_result.v1",
        },
        "terminal_reason": "service_restart",
        "events": [],
    }

    class Client:
        stale_status = old_adaptive_status

        def service_snapshot(self):
            return controller.service_snapshot()

        def latest_prompt_status(self, session_id):
            if self.stale_status is not None:
                return self.stale_status
            return service.latest_prompt_status(session_id)

        def prompt_status(self, mission_id):
            if (
                self.stale_status is not None
                and mission_id
                == self.stale_status["mission_id"]
            ):
                return self.stale_status
            return controller.status(mission_id)

        def submit_prompt(self, prompt, **kwargs):
            self.stale_status = None
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

        def confirm_prompt_no_contact(self, mission_id, **kwargs):
            return controller.confirm_no_contact(
                mission_id,
                operator=kwargs["operator"],
                authentication_source=kwargs[
                    "authentication_source"
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
        initial = browser.snapshot()
        assert initial["adapter"]["hierarchical_canonical"] is True
        assert initial["adapter"].get("adaptive_mission", False) is False
        assert initial["approval"]["enabled"] is False
        assert _is_adaptive_preapproval(initial) is False
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
        cache.update(
            "hierarchical_controller",
            {
                "schema": (
                    "sphero_rvr.hierarchical_controller_status.v1"
                ),
                "mission_id": proposed["mission"]["mission_id"],
                "state": "complete",
                "reason": "return_to_origin",
                "source_sha": SHA,
            },
            received_at_s=time.time(),
        )
        deadline = time.monotonic() + 1.0
        terminal = browser.snapshot()
        while (
            terminal["mission"]["state"] != "COMPLETE"
            and time.monotonic() < deadline
        ):
            time.sleep(0.01)
            terminal = browser.snapshot()
        assert terminal["mission"]["state"] == "COMPLETE"
        assert terminal["mission"]["no_contact_confirmed"] is False
        observed = browser.confirm_no_contact()
        assert observed["mission"]["no_contact_confirmed"] is True
        assert observed["mission"]["no_contact_operator"] == (
            "scott@example.com"
        )
    finally:
        latest = service.latest_prompt_status("m7-browser")
        if latest is not None:
            controller.cancel(latest["mission_id"])
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
        active_graph_capture=(
            lambda **kwargs: _active_graph_capture()
        ),
    )
    service, cache, fake, controller = _controller(tmp_path / "controller")
    del cache, fake
    try:
        session.state_directory.mkdir(parents=True)
        session.graph_audit_path.write_text(
            '{"stale_previous_capture":true}\n'
        )
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
        persisted_graph = json.loads(
            session.graph_audit_path.read_text()
        )
        assert persisted_graph["schema"].endswith(
            "active_graph_capture.v1"
        )
        assert "stale_previous_capture" not in persisted_graph
        assert oct(session.environment_path.stat().st_mode & 0o777) == "0o600"
        assert "RVR_HIERARCHICAL_M7_6_APPROVED=\"true\"" in (
            session.environment_path.read_text()
        )
        session.deactivate(reason="test complete")
        assert session.status()["active"] is False
        assert session.environment_path.exists() is False
        assert session.graph_audit_path.exists() is False
        assert session.approval_path.exists() is False
        assert session.proposal_path.exists() is False
    finally:
        controller.cancel(proposed["mission_id"])
        controller.close()
        service.close()


def test_wait_planning_intervals_are_reconstructed_without_hand_entered_durations() -> None:
    events = [
        {
            "event_id": 1,
            "kind": "hierarchical_checkpoint",
            "payload": {
                "source": "hierarchical_controller",
                "received_at_s": 10.0,
                "value": {
                    "state": "wait_planning",
                    "reason": "initial_provider_in_flight",
                },
            },
        },
        {
            "event_id": 2,
            "kind": "hierarchical_checkpoint",
            "payload": {
                "source": "hierarchical_controller",
                "received_at_s": 11.5,
                "value": {
                    "state": "wait_planning",
                    "reason": "motion_evidence_stale",
                },
            },
        },
        {
            "event_id": 3,
            "kind": "hierarchical_checkpoint",
            "payload": {
                "source": "hierarchical_controller",
                "received_at_s": 12.25,
                "value": {
                    "state": "dispatching",
                    "reason": "initial_goal",
                },
            },
        },
    ]
    intervals, valid = _wait_planning_intervals(
        events, terminal_at_s=13.0
    )
    assert valid is True
    assert intervals == [
        {
            "started_at_s": 10.0,
            "started_event_id": 1,
            "reasons": [
                "initial_provider_in_flight",
                "motion_evidence_stale",
            ],
            "ended_at_s": 12.25,
            "ended_event_id": 3,
            "duration_s": 2.25,
            "terminal_close": False,
        }
    ]


def test_active_graph_capture_recomputes_exact_command_ownership() -> None:
    capture = _active_graph_capture()
    assert all(active_graph_checks(capture, source_sha=SHA).values())
    altered = json.loads(json.dumps(capture))
    altered["observations"]["cmd_vel"]["stdout"] = altered[
        "observations"
    ]["cmd_vel"]["stdout"].replace(
        "Publisher count: 1", "Publisher count: 2"
    )
    unsigned = dict(altered)
    unsigned.pop("capture_digest")
    altered["capture_digest"] = canonical_digest(unsigned)
    assert active_graph_checks(altered, source_sha=SHA)[
        "exclusive_cmd_vel_owner"
    ] is False


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
            {
                "pose": {"x_m": 0.0, "y_m": 0.0},
                "motion_authority": False,
                "physical_execution_enabled": False,
            },
            received_at_s=now,
            source_timestamp_s=now,
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
            {
                "pose": {"x_m": 0.04, "y_m": 0.0},
                "motion_authority": False,
                "physical_execution_enabled": False,
            },
            received_at_s=now,
            source_timestamp_s=now,
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
        with pytest.raises(
            MissionValidationError, match="authenticated Tailscale"
        ):
            controller.confirm_no_contact(
                mission_id,
                operator="scott@example.com",
            )
        observed = controller.confirm_no_contact(
            mission_id,
            operator="scott@example.com",
            authentication_source="tailscale-serve",
        )
        assert [
            event["kind"] for event in observed["events"]
        ].count("hierarchical_no_contact_observation") == 1
        repeated = controller.confirm_no_contact(
            mission_id,
            operator="scott@example.com",
            authentication_source="tailscale-serve",
        )
        assert [
            event["kind"] for event in repeated["events"]
        ].count("hierarchical_no_contact_observation") == 1

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
                ("return_to_start", {}),
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
                            "current_snapshot": {
                                "tracks": [],
                                "map": {
                                    "coverage_fraction": (
                                        round(
                                            0.35 + index * 0.05,
                                            2,
                                        )
                                    )
                                },
                            },
                        }
                    ]
                },
                recorded_at_s=approval["approved_at_s"] + index,
            )
        journal.append(
            mission_id,
            "controller_event",
            {
                "kind": "semantic_non_motion_goal_ready",
                "at_s": 2.5,
                "decision": {
                    "action": "finish",
                    "arguments": {
                        "outcome": "complete",
                        "evidence_ids": ["camera-frame-01"],
                    },
                    "rationale": (
                        "Finish complete with camera-frame-01."
                    ),
                },
            },
            recorded_at_s=approval["approved_at_s"] + 2.5,
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
                    command,
                    1,
                    stdout="",
                    stderr=f"Unknown topic '{command[6]}'",
                )
            if command[0] == "fuser":
                return subprocess.CompletedProcess(
                    command, 1, stdout="", stderr=""
                )
            return subprocess.CompletedProcess(
                command, 0, stdout="", stderr=""
            )

        cleanup = capture_cleanup_evidence(
            runner=cleanup_runner,
            session_directory=tmp_path / "empty-session",
            evidence_directory=tmp_path / "empty-evidence",
        )
        report = evaluate_canonical_mission(
            mission_database=service.database,
            binding_journal=journal_path,
            mission_id=mission_id,
            cleanup_capture=cleanup,
        )
        assert report["passed"] is True
        assert all(report["checks"].values())
        assert len(report["evidence"]["semantic_decisions"]) == 3
        assert report["evidence"]["coverage_samples"] == [
            0.4,
            0.45,
        ]
        assert report["evidence"][
            "operator_no_contact_observation"
        ]["no_contact"] is True
        assert report["evidence"]["cleanup_capture"] == cleanup
        assert report["evidence"]["binding_events"]
        assert report["evidence"]["service_events"]
        assert report["evidence"]["terminal_result"]["status"] == "complete"

        failed_graph = {
            **cleanup,
            "observations": {
                **cleanup["observations"],
                "nodes": {
                    **cleanup["observations"]["nodes"],
                    "returncode": 1,
                    "stderr": "daemon unavailable",
                },
            },
        }
        failed_graph_unsigned = dict(failed_graph)
        failed_graph_unsigned.pop("capture_digest")
        failed_graph["capture_digest"] = canonical_digest(
            failed_graph_unsigned
        )
        assert _cleanup_checks(failed_graph)[
            "motion_nodes_absent"
        ] is False

        failed_topic = {
            **cleanup,
            "observations": {
                **cleanup["observations"],
                "cmd_vel": {
                    **cleanup["observations"]["cmd_vel"],
                    "returncode": 1,
                    "stderr": "permission denied",
                },
            },
        }
        failed_topic_unsigned = dict(failed_topic)
        failed_topic_unsigned.pop("capture_digest")
        failed_topic["capture_digest"] = canonical_digest(
            failed_topic_unsigned
        )
        assert _cleanup_checks(failed_topic)[
            "cmd_vel_publishers_absent"
        ] is False

        failed_serial = {
            **cleanup,
            "observations": {
                **cleanup["observations"],
                "serial_owner": {
                    **cleanup["observations"]["serial_owner"],
                    "returncode": 2,
                    "stderr": "inspection failed",
                },
            },
        }
        failed_serial_unsigned = dict(failed_serial)
        failed_serial_unsigned.pop("capture_digest")
        failed_serial["capture_digest"] = canonical_digest(
            failed_serial_unsigned
        )
        assert _cleanup_checks(failed_serial)[
            "serial_owner_absent"
        ] is False
    finally:
        controller.close()
        service.close()
