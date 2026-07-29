"""Approval-scoped lifecycle for the canonical M7.6 physical mission."""

from __future__ import annotations

import json
import os
from pathlib import Path
import subprocess
import threading
import time
from typing import Any, Callable, Mapping, Optional

from .hierarchical_physical_binding import (
    HierarchicalPhysicalApproval,
    validate_physical_proposal,
)
from .hierarchical_m7_canonical_validation import (
    capture_active_graph_evidence,
    validate_active_graph_evidence,
)
from .mission_api import MissionValidationError


HIERARCHICAL_MISSION_UNIT = "rvr-hierarchical-mission.service"
TELEMETRY_UNIT = "rvr-telemetry.service"
ACTIVE_GRAPH_READY_TIMEOUT_S = 45.0
ACTIVE_GRAPH_RETRY_INTERVAL_S = 1.0


class SystemdHierarchicalMissionSession:
    """Materialize one approval, start the fixed graph, then consume the files."""

    def __init__(
        self,
        *,
        activation_capable: bool,
        source_sha: str,
        deployed_sha: str,
        reviewed_sha: str,
        ros_workspace: str = "/home/jsperson/ros2_ws",
        state_directory: str | Path = (
            "~/.local/state/sphero_rvr/hierarchical-session"
        ),
        source_repository: str | Path = (
            "/home/jsperson/ros2_ws/src/sphero_rvr_ros"
        ),
        unit: str = HIERARCHICAL_MISSION_UNIT,
        runner: Optional[Callable[..., subprocess.CompletedProcess[str]]] = None,
        active_graph_capture: Optional[
            Callable[..., Mapping[str, Any]]
        ] = None,
    ) -> None:
        if str(unit).strip() != HIERARCHICAL_MISSION_UNIT:
            raise MissionValidationError(
                "only the fixed hierarchical mission systemd unit may be activated"
            )
        self.activation_capable = bool(activation_capable)
        self.source_sha = str(source_sha).strip()
        self.deployed_sha = str(deployed_sha).strip()
        self.reviewed_sha = str(reviewed_sha).strip()
        if self.activation_capable and not (
            len(self.source_sha) == 40
            and self.source_sha == self.deployed_sha == self.reviewed_sha
        ):
            raise MissionValidationError(
                "hierarchical session requires matching exact source, deployed, and reviewed SHAs"
            )
        self.ros_workspace = str(Path(ros_workspace).expanduser())
        self.state_directory = Path(state_directory).expanduser()
        self.source_repository = str(
            Path(source_repository).expanduser()
        )
        self.unit = HIERARCHICAL_MISSION_UNIT
        self._runner = runner or subprocess.run
        self._active_graph_capture = (
            active_graph_capture or capture_active_graph_evidence
        )
        self._lock = threading.RLock()
        self._mission_id = ""
        self._active = False
        self._active_graph: dict[str, Any] = {}
        self._detail = "canonical physical session locked"

    @property
    def proposal_path(self) -> Path:
        return self.state_directory / "proposal.json"

    @property
    def approval_path(self) -> Path:
        return self.state_directory / "approval.json"

    @property
    def environment_path(self) -> Path:
        return self.state_directory / "session.env"

    @property
    def graph_audit_path(self) -> Path:
        return self.state_directory / "active-graph.json"

    def activate(
        self,
        *,
        proposal: Mapping[str, Any],
        approval: Mapping[str, Any],
        now_s: float,
        cancel_event: Optional[threading.Event] = None,
    ) -> Mapping[str, Any]:
        if not self.activation_capable:
            raise MissionValidationError(
                "canonical physical activation is disabled by reviewed configuration"
            )
        validated_approval = HierarchicalPhysicalApproval.validated(
            approval,
            now_s=now_s,
            source_sha=self.source_sha,
            deployed_sha=self.deployed_sha,
            reviewed_sha=self.reviewed_sha,
        )
        authority_binding = {
            "mission_id": validated_approval.mission_id,
            "proposal_digest": validated_approval.proposal_digest,
            "mission_lease_s": validated_approval.mission_lease_s,
        }
        validated_proposal = validate_physical_proposal(
            proposal,
            authority=authority_binding,
            source_sha=self.source_sha,
        )
        with self._lock:
            if cancel_event is not None and cancel_event.is_set():
                raise MissionValidationError(
                    "canonical physical activation was cancelled before start"
                )
            if self._active:
                raise MissionValidationError(
                    "another canonical physical session is already active"
                )
            self.state_directory.mkdir(parents=True, exist_ok=True)
            os.chmod(self.state_directory, 0o700)
            # A crash may have left a prior capture behind. Remove it before
            # graph startup so the controller can only observe this
            # activation's newly generated audit.
            self.graph_audit_path.unlink(missing_ok=True)
            self._write_json(self.proposal_path, validated_proposal)
            self._write_json(self.approval_path, dict(approval))
            self._write_environment()
            self._run(
                ["systemctl", "--user", "stop", TELEMETRY_UNIT],
                action="stop no-motion telemetry",
                timeout_s=30.0,
            )
            self._run(
                ["systemctl", "--user", "reset-failed", TELEMETRY_UNIT],
                action="clear no-motion telemetry stop result",
                timeout_s=5.0,
            )
            if cancel_event is not None and cancel_event.is_set():
                self._remove_activation_files()
                raise MissionValidationError(
                    "canonical physical activation was cancelled before graph start"
                )
            try:
                self._run(
                    ["systemctl", "--user", "daemon-reload"],
                    action="reload the hierarchical mission unit",
                    timeout_s=10.0,
                )
                self._run(
                    ["systemctl", "--user", "start", self.unit],
                    action="start the canonical hierarchical mission graph",
                    timeout_s=45.0,
                )
                status = self._unit_status()
                if not status["active"]:
                    raise MissionValidationError(
                        "canonical hierarchical mission graph did not become active: "
                        + str(status["detail"])
                    )
                if cancel_event is not None and cancel_event.is_set():
                    self._best_effort_stop()
                    raise MissionValidationError(
                        "canonical physical activation was cancelled during graph start"
                    )
                graph_capture = self._capture_ready_active_graph(
                    cancel_event=cancel_event
                )
                if cancel_event is not None and cancel_event.is_set():
                    raise MissionValidationError(
                        "canonical physical activation was cancelled during graph audit"
                    )
                self._write_json(
                    self.graph_audit_path, graph_capture
                )
            except Exception:
                self._best_effort_stop()
                self._remove_activation_files()
                raise
            self._mission_id = validated_approval.mission_id
            self._active = True
            self._active_graph = graph_capture
            self._detail = (
                "canonical hierarchical graph active for approval "
                f"{validated_approval.approval_digest[:12]}"
            )
            return self.status()

    def _capture_ready_active_graph(
        self,
        *,
        cancel_event: Optional[threading.Event],
    ) -> dict[str, Any]:
        """Wait for one complete, self-consistent ROS graph before authority."""

        deadline = time.monotonic() + ACTIVE_GRAPH_READY_TIMEOUT_S
        last_error = "active graph audit did not run"
        while True:
            if cancel_event is not None and cancel_event.is_set():
                raise MissionValidationError(
                    "canonical physical activation was cancelled during graph audit"
                )
            status = self._unit_status()
            if not status["active"]:
                raise MissionValidationError(
                    "canonical hierarchical mission graph exited before its "
                    f"active graph was ready: {status['detail']}"
                )
            graph_capture = dict(
                self._active_graph_capture(
                    source_sha=self.source_sha,
                    source_repository=self.source_repository,
                )
            )
            try:
                validate_active_graph_evidence(
                    graph_capture, source_sha=self.source_sha
                )
                return graph_capture
            except MissionValidationError as exc:
                last_error = str(exc)
            remaining_s = deadline - time.monotonic()
            if remaining_s <= 0.0:
                raise MissionValidationError(
                    "canonical active graph did not become ready within "
                    f"{ACTIVE_GRAPH_READY_TIMEOUT_S:.0f} seconds: {last_error}"
                )
            time.sleep(
                min(ACTIVE_GRAPH_RETRY_INTERVAL_S, remaining_s)
            )

    def deactivate(self, *, reason: str) -> Mapping[str, Any]:
        with self._lock:
            self._run(
                ["systemctl", "--user", "stop", self.unit],
                action="stop the canonical hierarchical mission graph",
                timeout_s=40.0,
            )
            status = self._unit_status()
            if status["state"] == "failed":
                self._run(
                    ["systemctl", "--user", "reset-failed", self.unit],
                    action="clear the canonical mission unit failure state",
                    timeout_s=5.0,
                )
                status = self._unit_status()
            if (
                status["active"]
                or status["transitioning"]
                or status["state"] != "inactive"
            ):
                raise MissionValidationError(
                    "canonical hierarchical graph did not relock: "
                    + str(status["detail"])
                )
            self._remove_activation_files()
            self._active = False
            self._mission_id = ""
            self._active_graph = {}
            self._detail = str(reason).strip() or "canonical physical session locked"
            return self.status()

    def ensure_locked(self) -> Mapping[str, Any]:
        return self.deactivate(
            reason="canonical physical session locked after service start"
        )

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
                "approval_files_present": bool(
                    self.proposal_path.is_file()
                    and self.approval_path.is_file()
                    and self.environment_path.is_file()
                    and self.graph_audit_path.is_file()
                ),
                "active_graph_capture": (
                    dict(self._active_graph) if active else {}
                ),
                "restart_resume_allowed": False,
            }

    def _write_json(self, path: Path, payload: Mapping[str, Any]) -> None:
        temporary = path.with_suffix(path.suffix + ".tmp")
        rendered = json.dumps(
            dict(payload),
            sort_keys=True,
            separators=(",", ":"),
            allow_nan=False,
        )
        with temporary.open("w", encoding="utf-8") as stream:
            os.chmod(temporary, 0o600)
            stream.write(rendered)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)

    def _write_environment(self) -> None:
        values = {
            "RVR_HIERARCHICAL_BINDING_INSTALLED": "true",
            "RVR_HIERARCHICAL_M7_6_APPROVED": "true",
            "RVR_SOURCE_SHA": self.source_sha,
            "RVR_DEPLOYED_SHA": self.deployed_sha,
            "RVR_HIERARCHICAL_REVIEWED_SHA": self.reviewed_sha,
            "RVR_HIERARCHICAL_APPROVAL_FILE": str(self.approval_path),
            "RVR_HIERARCHICAL_PROPOSAL_FILE": str(self.proposal_path),
            "RVR_HIERARCHICAL_GRAPH_AUDIT_FILE": str(
                self.graph_audit_path
            ),
            "RVR_ROS_WORKSPACE": self.ros_workspace,
        }
        for value in values.values():
            if not value or any(character in value for character in "\r\n\x00"):
                raise MissionValidationError(
                    "hierarchical session environment contains an unsafe value"
                )
        temporary = self.environment_path.with_suffix(".env.tmp")
        with temporary.open("w", encoding="utf-8") as stream:
            os.chmod(temporary, 0o600)
            for name, value in values.items():
                escaped = value.replace("\\", "\\\\").replace('"', '\\"')
                stream.write(f'{name}="{escaped}"\n')
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, self.environment_path)

    def _remove_activation_files(self) -> None:
        for path in (
            self.environment_path,
            self.graph_audit_path,
            self.approval_path,
            self.proposal_path,
        ):
            try:
                path.unlink()
            except FileNotFoundError:
                pass

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
            action="query the canonical hierarchical mission graph",
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
            or "canonical hierarchical mission unit is unavailable"
        )
        return {
            "available": loaded,
            "active": loaded and active_state == "active",
            "transitioning": active_state
            in {"activating", "deactivating", "reloading"},
            "state": active_state,
            "detail": detail,
        }

    def _run(
        self,
        command: list[str],
        *,
        action: str,
        timeout_s: float,
        allow_failure: bool = False,
    ) -> subprocess.CompletedProcess[str]:
        try:
            completed = self._runner(
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
            self._runner(
                ["systemctl", "--user", "stop", self.unit],
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                timeout=40.0,
                check=False,
            )
        except (OSError, subprocess.TimeoutExpired):
            pass
