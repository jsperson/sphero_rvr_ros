from __future__ import annotations

from subprocess import CompletedProcess

import pytest

from sphero_rvr_driver.adaptive_mission_session import (
    ADAPTIVE_MISSION_UNIT,
    TELEMETRY_UNIT,
    SystemdAdaptiveMissionSession,
)
from sphero_rvr_driver.mission_api import MissionValidationError


DIGEST = "a" * 64


def test_systemd_session_activates_only_fixed_graph_after_complete_binding(
    monkeypatch,
) -> None:
    active = False
    commands: list[tuple[str, ...]] = []

    def run(command, **kwargs):
        nonlocal active
        del kwargs
        command = tuple(command)
        commands.append(command)
        if command[:4] == (
            "systemctl",
            "--user",
            "start",
            ADAPTIVE_MISSION_UNIT,
        ):
            active = True
        elif command[:4] == (
            "systemctl",
            "--user",
            "stop",
            ADAPTIVE_MISSION_UNIT,
        ):
            active = False
        if "show" in command:
            state = "active" if active else "inactive"
            substate = "running" if active else "dead"
            return CompletedProcess(
                command,
                0,
                stdout=(
                    "LoadState=loaded\n"
                    f"ActiveState={state}\n"
                    f"SubState={substate}\n"
                    "Result=success\n"
                ),
                stderr="",
            )
        return CompletedProcess(command, 0, stdout="", stderr="")

    monkeypatch.setattr(
        "sphero_rvr_driver.adaptive_mission_session.subprocess.run",
        run,
    )
    lifecycle = SystemdAdaptiveMissionSession(
        activation_capable=True
    )

    activated = lifecycle.activate(
        mission_id="mission-1",
        proposal_digest=DIGEST,
        operator="scott@example.com",
    )
    assert activated["active"] is True
    assert activated["mission_id"] == "mission-1"
    assert commands[0] == (
        "systemctl",
        "--user",
        "stop",
        TELEMETRY_UNIT,
    )
    assert commands[1] == (
        "systemctl",
        "--user",
        "start",
        ADAPTIVE_MISSION_UNIT,
    )

    relocked = lifecycle.deactivate(reason="terminal")
    assert relocked["active"] is False
    assert relocked["mission_id"] == ""
    assert (
        "systemctl",
        "--user",
        "stop",
        ADAPTIVE_MISSION_UNIT,
    ) in commands
    assert (
        "systemctl",
        "--user",
        "reset-failed",
        ADAPTIVE_MISSION_UNIT,
    ) in commands


def test_systemd_session_fails_closed_for_disabled_or_incomplete_activation(
    monkeypatch,
) -> None:
    calls = []

    def run(command, **kwargs):
        del kwargs
        calls.append(tuple(command))
        return CompletedProcess(command, 0, stdout="", stderr="")

    monkeypatch.setattr(
        "sphero_rvr_driver.adaptive_mission_session.subprocess.run",
        run,
    )
    disabled = SystemdAdaptiveMissionSession(
        activation_capable=False
    )
    with pytest.raises(
        MissionValidationError, match="disabled by reviewed configuration"
    ):
        disabled.activate(
            mission_id="mission-1",
            proposal_digest=DIGEST,
            operator="scott@example.com",
        )
    assert calls == []

    enabled = SystemdAdaptiveMissionSession(
        activation_capable=True
    )
    with pytest.raises(
        MissionValidationError, match="binding is incomplete"
    ):
        enabled.activate(
            mission_id="",
            proposal_digest="short",
            operator="",
        )
    assert calls == []

    with pytest.raises(
        MissionValidationError, match="only the fixed"
    ):
        SystemdAdaptiveMissionSession(
            activation_capable=True,
            unit="some-other.service",
        )


def test_systemd_session_stops_graph_when_start_fails(monkeypatch) -> None:
    commands: list[tuple[str, ...]] = []

    def run(command, **kwargs):
        del kwargs
        command = tuple(command)
        commands.append(command)
        if command[:3] == ("systemctl", "--user", "start"):
            return CompletedProcess(
                command, 1, stdout="", stderr="start rejected"
            )
        return CompletedProcess(command, 0, stdout="", stderr="")

    monkeypatch.setattr(
        "sphero_rvr_driver.adaptive_mission_session.subprocess.run",
        run,
    )
    lifecycle = SystemdAdaptiveMissionSession(
        activation_capable=True
    )

    with pytest.raises(
        MissionValidationError, match="start rejected"
    ):
        lifecycle.activate(
            mission_id="mission-1",
            proposal_digest=DIGEST,
            operator="scott@example.com",
        )
    assert commands[-1] == (
        "systemctl",
        "--user",
        "stop",
        ADAPTIVE_MISSION_UNIT,
    )


def test_systemd_session_requires_failed_state_to_clear_before_relock(
    monkeypatch,
) -> None:
    def run(command, **kwargs):
        del kwargs
        command = tuple(command)
        if command[:3] == ("systemctl", "--user", "reset-failed"):
            return CompletedProcess(
                command, 1, stdout="", stderr="reset rejected"
            )
        return CompletedProcess(command, 0, stdout="", stderr="")

    monkeypatch.setattr(
        "sphero_rvr_driver.adaptive_mission_session.subprocess.run",
        run,
    )
    lifecycle = SystemdAdaptiveMissionSession(
        activation_capable=True
    )

    with pytest.raises(
        MissionValidationError, match="reset rejected"
    ):
        lifecycle.deactivate(reason="terminal")
