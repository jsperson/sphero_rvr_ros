"""Typed local Unix-socket client for the authoritative MissionService owner."""

from __future__ import annotations

import json
from pathlib import Path
import socket
import stat
from typing import Any, Mapping, Optional

from .mission_api import MissionValidationError


class MissionServiceClient:
    def __init__(
        self,
        socket_path: str | Path,
        *,
        timeout_s: float = 130.0,
        max_response_bytes: int = 4_000_000,
    ) -> None:
        self.socket_path = Path(socket_path).expanduser()
        self.timeout_s = float(timeout_s)
        self.max_response_bytes = int(max_response_bytes)
        if self.timeout_s <= 0.0:
            raise ValueError("mission service timeout must be positive")
        if self.max_response_bytes < 1024:
            raise ValueError("mission service response limit is too small")

    def call(self, operation: str, **payload: Any) -> Any:
        request = {"operation": str(operation), **payload}
        encoded = (json.dumps(request, sort_keys=True, separators=(",", ":")) + "\n").encode("utf-8")
        if len(encoded) > 4_000_000:
            raise MissionValidationError("mission service request exceeds the bounded size")
        self._validate_socket()
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
            client.settimeout(self.timeout_s)
            client.connect(str(self.socket_path))
            client.sendall(encoded)
            response = bytearray()
            while not response.endswith(b"\n"):
                chunk = client.recv(min(65_536, self.max_response_bytes + 1 - len(response)))
                if not chunk:
                    break
                response.extend(chunk)
                if len(response) > self.max_response_bytes:
                    raise MissionValidationError("mission service response exceeds the bounded size")
        try:
            envelope = json.loads(response.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise MissionValidationError("mission service returned malformed JSON") from exc
        if not isinstance(envelope, Mapping):
            raise MissionValidationError("mission service response must be an object")
        if not envelope.get("ok"):
            error = envelope.get("error", {})
            message = error.get("message", "mission service request failed") if isinstance(error, Mapping) else error
            raise MissionValidationError(str(message))
        return envelope.get("result")

    def service_snapshot(self) -> Mapping[str, Any]:
        return self.call("service_snapshot")

    def submit_prompt(
        self,
        prompt: str,
        *,
        session_id: str,
        source: str = "web",
        mission_id: Optional[str] = None,
        mission_lease_s: Optional[float] = None,
        operator: str = "",
        authentication_source: str = "",
    ) -> Mapping[str, Any]:
        payload: dict[str, Any] = {
            "prompt": prompt,
            "session_id": session_id,
            "source": source,
        }
        if mission_id is not None:
            payload["mission_id"] = mission_id
        if mission_lease_s is not None:
            payload["mission_lease_s"] = mission_lease_s
        if operator:
            payload["operator"] = operator
        if authentication_source:
            payload["authentication_source"] = authentication_source
        return self.call("prompt_submit", **payload)

    def prompt_status(self, mission_id: str) -> Mapping[str, Any]:
        return self.call("prompt_status", mission_id=mission_id)

    def latest_prompt_status(self, session_id: str) -> Optional[Mapping[str, Any]]:
        result = self.call("prompt_latest", session_id=session_id)
        return None if result is None else result

    def approve_prompt(
        self,
        mission_id: str,
        *,
        approval_phrase: str,
        operator: str,
        authentication_source: str = "",
        physical_room_confirmation: Optional[Mapping[str, Any]] = None,
    ) -> Mapping[str, Any]:
        payload: dict[str, Any] = {
            "mission_id": mission_id,
            "approval_phrase": approval_phrase,
            "operator": operator,
            "authentication_source": authentication_source,
        }
        if physical_room_confirmation is not None:
            payload["physical_room_confirmation"] = dict(
                physical_room_confirmation
            )
        return self.call("prompt_approve", **payload)

    def cancel_prompt(self, mission_id: str, *, reason: str) -> Mapping[str, Any]:
        return self.call("prompt_cancel", mission_id=mission_id, reason=reason)

    def confirm_prompt_no_contact(
        self,
        mission_id: str,
        *,
        operator: str,
        authentication_source: str,
    ) -> Mapping[str, Any]:
        return self.call(
            "prompt_confirm_no_contact",
            mission_id=mission_id,
            operator=operator,
            authentication_source=authentication_source,
        )

    def _validate_socket(self) -> None:
        try:
            metadata = self.socket_path.stat()
        except FileNotFoundError as exc:
            raise MissionValidationError(f"mission service socket is unavailable: {self.socket_path}") from exc
        if not stat.S_ISSOCK(metadata.st_mode):
            raise MissionValidationError("mission service path is not a Unix socket")
        if stat.S_IMODE(metadata.st_mode) & 0o077:
            raise MissionValidationError("mission service socket permissions are not user-only")
