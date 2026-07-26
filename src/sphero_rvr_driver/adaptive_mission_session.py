"""Approval-scoped lifecycle for the motor-capable Adaptive mission graph."""

from __future__ import annotations

import subprocess
import threading
from typing import Any, Mapping

from .mission_api import MissionValidationError


ADAPTIVE_MISSION_UNIT = "rvr-adaptive-mission.service"
TELEMETRY_UNIT = "rvr-telemetry.service"


class SystemdAdaptiveMissionSession:
    """Start the fixed supervised graph only after authenticated approval."""

    def __init__(
        self,
        *,
        activation_capable: bool,
        unit: str = ADAPTIVE_MISSION_UNIT,
    ) -> None:
        if str(unit).strip() != ADAPTIVE_MISSION_UNIT:
            raise MissionValidationError(
                "only the fixed Adaptive mission systemd unit may be activated"
            )
        self.activation_capable = bool(activation_capable)
        self.unit = ADAPTIVE_MISSION_UNIT
        self._lock = threading.RLock()
        self._mission_id = ""
        self._active = False
        self._detail = "physical session locked"

    def activate(
        self,
        *,
        mission_id: str,
        proposal_digest: str,
        operator: str,
    ) -> Mapping[str, Any]:
        """Replace no-motion telemetry with the reviewed supervised graph."""

        mission = str(mission_id).strip()
        digest = str(proposal_digest).strip().lower()
        principal = str(operator).strip()
        if not self.activation_capable:
            raise MissionValidationError(
                "approval-time physical activation is disabled by reviewed configuration"
            )
        if (
            not mission
            or len(digest) != 64
            or any(character not in "0123456789abcdef" for character in digest)
            or not principal
        ):
            raise MissionValidationError(
                "approval-time physical activation binding is incomplete"
            )
        with self._lock:
            if self._active:
                raise MissionValidationError(
                    "another approved physical session is already active"
                )
            self._run(
                ["systemctl", "--user", "stop", TELEMETRY_UNIT],
                action="stop no-motion telemetry",
                timeout_s=30.0,
            )
            try:
                self._run(
                    ["systemctl", "--user", "start", self.unit],
                    action="start supervised Adaptive mission graph",
                    timeout_s=30.0,
                )
                status = self._unit_status()
                if not status["active"]:
                    raise MissionValidationError(
                        "supervised Adaptive mission graph did not become active: "
                        + str(status["detail"])
                    )
            except Exception:
                self._best_effort_stop()
                raise
            self._mission_id = mission
            self._active = True
            self._detail = (
                "supervised graph active for authenticated approval "
                f"{digest[:12]}"
            )
            return self.status()

    def deactivate(self, *, reason: str) -> Mapping[str, Any]:
        """Stop the supervised graph and verify systemd reports it inactive."""

        with self._lock:
            self._run(
                ["systemctl", "--user", "stop", self.unit],
                action="stop supervised Adaptive mission graph",
                timeout_s=35.0,
            )
            self._run(
                ["systemctl", "--user", "reset-failed", self.unit],
                action="clear supervised Adaptive mission graph failure state",
                timeout_s=5.0,
            )
            status = self._unit_status()
            if (
                status["active"]
                or status["transitioning"]
                or status["state"] != "inactive"
            ):
                raise MissionValidationError(
                    "supervised Adaptive mission graph did not relock: "
                    + str(status["detail"])
                )
            self._active = False
            self._mission_id = ""
            self._detail = str(reason).strip() or "physical session locked"
            return self.status()

    def status(self) -> Mapping[str, Any]:
        with self._lock:
            unit = self._unit_status()
            active = bool(unit["active"])
            if not active:
                self._active = False
                self._mission_id = ""
            return {
                "activation_capable": self.activation_capable,
                "active": active,
                "transitioning": bool(unit["transitioning"]),
                "mission_id": self._mission_id if active else "",
                "detail": self._detail if active else str(unit["detail"]),
                "unit": self.unit,
            }

    def ensure_locked(self) -> Mapping[str, Any]:
        """Fail startup unless an orphaned graph can be stopped."""

        return self.deactivate(reason="physical session locked after service start")

    def _unit_status(self) -> dict[str, Any]:
        completed = self._run(
            [
                "systemctl",
                "--user",
                "show",
                "--property=LoadState",
                "--property=ActiveState",
                "--property=SubState",
                "--property=Result",
                self.unit,
            ],
            action="query supervised Adaptive mission graph",
            timeout_s=5.0,
            allow_failure=True,
        )
        properties: dict[str, str] = {}
        for line in str(completed.stdout).splitlines():
            key, separator, value = line.partition("=")
            if separator:
                properties[key.strip()] = value.strip()
        loaded = (
            completed.returncode == 0
            and properties.get("LoadState", "not-found").lower() == "loaded"
        )
        active_state = properties.get("ActiveState", "unknown").lower()
        sub_state = properties.get("SubState", "unknown").lower()
        detail = (
            f"{active_state} / {sub_state}"
            if loaded
            else str(completed.stderr).strip()
            or "Adaptive mission systemd unit is unavailable"
        )
        return {
            "available": loaded,
            "active": loaded and active_state == "active",
            "transitioning": active_state
            in {"activating", "deactivating", "reloading"},
            "state": active_state,
            "detail": detail,
        }

    @staticmethod
    def _run(
        command: list[str],
        *,
        action: str,
        timeout_s: float,
        allow_failure: bool = False,
    ) -> subprocess.CompletedProcess[str]:
        try:
            completed = subprocess.run(
                command,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                timeout=timeout_s,
                check=False,
            )
        except (OSError, subprocess.TimeoutExpired) as exc:
            raise MissionValidationError(f"unable to {action}: {exc}") from exc
        if completed.returncode != 0 and not allow_failure:
            detail = str(completed.stderr).strip() or str(
                completed.stdout
            ).strip()
            raise MissionValidationError(
                f"unable to {action}: {detail or 'systemd request failed'}"
            )
        return completed

    def _best_effort_stop(self) -> None:
        try:
            subprocess.run(
                ["systemctl", "--user", "stop", self.unit],
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                timeout=35.0,
                check=False,
            )
        except (OSError, subprocess.TimeoutExpired):
            pass
