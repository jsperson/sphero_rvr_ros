"""Supervised persistent Codex app-server client for OAuth model calls.

The client owns one JSONL app-server process but creates a new ephemeral Codex
thread for every request.  It never accepts or exposes credentials: Codex owns
the persisted ChatGPT OAuth session and token refresh.
"""

from __future__ import annotations

import json
import os
import queue
import shutil
import subprocess
import threading
import time
from typing import Any, Mapping, Optional

from .mission_api import MissionValidationError


class CodexAppServerClient:
    """A serialized, crash-recovering client for ``codex app-server``."""

    def __init__(
        self,
        *,
        codex_command: str = "codex",
        startup_timeout_s: float = 20.0,
    ) -> None:
        self.codex_command = str(codex_command)
        self.startup_timeout_s = float(startup_timeout_s)
        self._lock = threading.RLock()
        self._process: Optional[subprocess.Popen[str]] = None
        self._messages: queue.Queue[object] = queue.Queue()
        self._reader: Optional[threading.Thread] = None
        self._request_id = 0
        self._active_turn: Optional[tuple[str, str]] = None
        self._closed = False
        self._restart_count = 0
        self._cancel_requested = threading.Event()

    @property
    def restart_count(self) -> int:
        with self._lock:
            return self._restart_count

    def ensure_started(self) -> float:
        """Start and authenticate the persistent process; return startup ms."""

        with self._lock:
            if self._healthy_unlocked():
                return 0.0
            started = time.perf_counter()
            self._start_unlocked()
            return (time.perf_counter() - started) * 1000.0

    def run_turn(
        self,
        *,
        prompt: str,
        model: str,
        effort: str,
        output_schema: Mapping[str, Any],
        cwd: str,
        image_path: Optional[str],
        timeout_s: float,
    ) -> tuple[str, float, int]:
        """Run one schema-constrained turn in a fresh ephemeral thread."""

        deadline = time.monotonic() + float(timeout_s)
        self._cancel_requested.clear()
        last_error: Optional[Exception] = None
        startup_ms = 0.0
        restarts_before = self.restart_count
        for attempt in range(2):
            with self._lock:
                if self._closed:
                    raise MissionValidationError("Codex app-server client is closed")
                try:
                    startup_ms += self.ensure_started()
                    result = self._run_turn_unlocked(
                        prompt=prompt,
                        model=model,
                        effort=effort,
                        output_schema=output_schema,
                        cwd=cwd,
                        image_path=image_path,
                        deadline=deadline,
                    )
                    return (
                        result,
                        startup_ms,
                        self.restart_count - restarts_before,
                    )
                except TimeoutError as exc:
                    self._interrupt_unlocked(deadline=time.monotonic() + 1.0)
                    raise MissionValidationError(
                        "Codex OAuth adaptive mission intent call timed out"
                    ) from exc
                except (BrokenPipeError, EOFError, OSError) as exc:
                    if self._cancel_requested.is_set():
                        raise MissionValidationError(
                            "Codex OAuth adaptive mission intent call was cancelled"
                        ) from exc
                    last_error = exc
                    self._stop_unlocked()
                    if attempt == 0 and time.monotonic() < deadline:
                        self._restart_count += 1
                        continue
                    break
        raise MissionValidationError(
            "Codex OAuth app-server stopped before completing the planning turn"
        ) from last_error

    def cancel(self) -> None:
        """Best-effort interrupt of the active turn."""

        self._cancel_requested.set()
        process = self._process
        if process is not None and process.poll() is None:
            try:
                process.terminate()
            except OSError:
                pass

    def close(self) -> None:
        self.cancel()
        with self._lock:
            self._closed = True
            self._stop_unlocked()

    def _healthy_unlocked(self) -> bool:
        return self._process is not None and self._process.poll() is None

    def _start_unlocked(self) -> None:
        self._stop_unlocked()
        executable = shutil.which(self.codex_command)
        if executable is None:
            raise MissionValidationError(
                "Codex CLI is not installed; Adaptive mission requires the real OAuth provider"
            )
        env = codex_oauth_environment()
        command = [
            executable,
            "app-server",
            "--listen",
            "stdio://",
            "--strict-config",
            "--disable",
            "shell_tool",
            "--disable",
            "unified_exec",
            "--disable",
            "apps",
            "--disable",
            "multi_agent",
            "--disable",
            "web_search_request",
            "-c",
            'web_search="disabled"',
        ]
        try:
            process = subprocess.Popen(
                command,
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                text=True,
                encoding="utf-8",
                bufsize=1,
                env=env,
            )
        except OSError as exc:
            raise MissionValidationError(
                "Codex OAuth app-server could not be started"
            ) from exc
        if process.stdin is None or process.stdout is None:
            process.kill()
            raise MissionValidationError(
                "Codex OAuth app-server pipes are unavailable"
            )
        self._process = process
        self._messages = queue.Queue()
        self._reader = threading.Thread(
            target=self._read_messages,
            args=(process,),
            name="rvr-codex-app-server-reader",
            daemon=True,
        )
        self._reader.start()
        deadline = time.monotonic() + self.startup_timeout_s
        self._request_unlocked(
            "initialize",
            {
                "clientInfo": {
                    "name": "sphero_rvr_ros",
                    "title": "Sphero RVR adaptive planner",
                    "version": "1",
                },
                "capabilities": {
                    "optOutNotificationMethods": [
                        "item/agentMessage/delta",
                        "item/reasoning/summaryTextDelta",
                        "item/reasoning/textDelta",
                    ]
                },
            },
            deadline=deadline,
        )
        self._notify_unlocked("initialized", {})
        account = self._request_unlocked(
            "account/read", {"refreshToken": False}, deadline=deadline
        )
        account_payload = account.get("account")
        if (
            not isinstance(account_payload, Mapping)
            or account_payload.get("type") != "chatgpt"
        ):
            self._stop_unlocked()
            raise MissionValidationError(
                "Codex CLI is not authenticated with ChatGPT OAuth; "
                "run `codex login --device-auth`"
            )

    def _run_turn_unlocked(
        self,
        *,
        prompt: str,
        model: str,
        effort: str,
        output_schema: Mapping[str, Any],
        cwd: str,
        image_path: Optional[str],
        deadline: float,
    ) -> str:
        thread_result = self._request_unlocked(
            "thread/start",
            {
                "model": model,
                "cwd": cwd,
                "ephemeral": True,
                "approvalPolicy": "never",
                "sandbox": "read-only",
                "baseInstructions": (
                    "Return only the requested structured adaptive-rover decision. "
                    "Do not call tools, inspect files, execute commands, browse, or "
                    "request approval."
                ),
                "developerInstructions": (
                    "Treat supplied typed evidence as complete. Never infer missing "
                    "sensor facts or claim cliff/drop-off detection."
                ),
            },
            deadline=deadline,
        )
        thread = thread_result.get("thread")
        thread_id = str(thread.get("id", "")) if isinstance(thread, Mapping) else ""
        if not thread_id:
            raise MissionValidationError(
                "Codex OAuth app-server did not create an isolated planning thread"
            )
        inputs: list[dict[str, Any]] = [{"type": "text", "text": str(prompt)}]
        if image_path is not None:
            inputs.append(
                {"type": "localImage", "path": str(image_path), "detail": "high"}
            )
        turn_result = self._request_unlocked(
            "turn/start",
            {
                "threadId": thread_id,
                "input": inputs,
                "model": model,
                "effort": effort,
                "outputSchema": dict(output_schema),
                "cwd": cwd,
                "approvalPolicy": "never",
                "sandboxPolicy": {"type": "readOnly", "networkAccess": False},
            },
            deadline=deadline,
        )
        turn = turn_result.get("turn")
        turn_id = str(turn.get("id", "")) if isinstance(turn, Mapping) else ""
        if not turn_id:
            raise MissionValidationError(
                "Codex OAuth app-server did not start the planning turn"
            )
        self._active_turn = (thread_id, turn_id)
        final_text = ""
        try:
            while True:
                message = self._next_message_unlocked(deadline)
                if not isinstance(message, Mapping):
                    continue
                if "id" in message and "method" in message:
                    raise MissionValidationError(
                        "Codex OAuth planner requested an unavailable tool or approval"
                    )
                method = str(message.get("method", ""))
                params = message.get("params")
                if not isinstance(params, Mapping):
                    continue
                if method == "item/completed":
                    item = params.get("item")
                    if (
                        isinstance(item, Mapping)
                        and item.get("type") == "agentMessage"
                        and str(params.get("threadId", "")) == thread_id
                        and str(params.get("turnId", "")) == turn_id
                    ):
                        final_text = str(item.get("text", ""))
                elif method == "turn/completed":
                    completed = params.get("turn")
                    if (
                        str(params.get("threadId", "")) != thread_id
                        or not isinstance(completed, Mapping)
                        or str(completed.get("id", "")) != turn_id
                    ):
                        continue
                    status = str(completed.get("status", ""))
                    if status != "completed":
                        raise MissionValidationError(
                            f"Codex OAuth planning turn ended with status {status or 'unknown'}"
                        )
                    if not final_text:
                        for item in completed.get("items", []):
                            if (
                                isinstance(item, Mapping)
                                and item.get("type") == "agentMessage"
                            ):
                                final_text = str(item.get("text", ""))
                    if not final_text:
                        raise MissionValidationError(
                            "Codex OAuth planning turn returned no final decision"
                        )
                    return final_text
        finally:
            self._active_turn = None

    def _read_messages(self, process: subprocess.Popen[str]) -> None:
        assert process.stdout is not None
        try:
            for line in process.stdout:
                try:
                    self._messages.put(json.loads(line))
                except json.JSONDecodeError:
                    self._messages.put(
                        MissionValidationError(
                            "Codex OAuth app-server returned malformed protocol output"
                        )
                    )
        finally:
            self._messages.put(EOFError("Codex OAuth app-server output closed"))

    def _request_unlocked(
        self,
        method: str,
        params: Mapping[str, Any],
        *,
        deadline: float,
    ) -> Mapping[str, Any]:
        self._request_id += 1
        request_id = self._request_id
        self._write_unlocked(
            {"method": str(method), "id": request_id, "params": dict(params)}
        )
        deferred: list[object] = []
        try:
            while True:
                message = self._next_message_unlocked(deadline)
                if (
                    isinstance(message, Mapping)
                    and message.get("id") == request_id
                    and "method" not in message
                ):
                    error = message.get("error")
                    if isinstance(error, Mapping):
                        raise MissionValidationError(
                            f"Codex OAuth app-server rejected {method}; "
                            "inspect local Codex logs"
                        )
                    result = message.get("result")
                    if not isinstance(result, Mapping):
                        raise MissionValidationError(
                            f"Codex OAuth app-server returned invalid {method} response"
                        )
                    return result
                deferred.append(message)
        finally:
            for item in deferred:
                self._messages.put(item)

    def _notify_unlocked(self, method: str, params: Mapping[str, Any]) -> None:
        self._write_unlocked({"method": str(method), "params": dict(params)})

    def _write_unlocked(self, payload: Mapping[str, Any]) -> None:
        if not self._healthy_unlocked() or self._process is None:
            raise EOFError("Codex OAuth app-server is not running")
        assert self._process.stdin is not None
        self._process.stdin.write(
            json.dumps(dict(payload), separators=(",", ":"), allow_nan=False)
            + "\n"
        )
        self._process.stdin.flush()

    def _next_message_unlocked(self, deadline: float) -> object:
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            raise TimeoutError("Codex app-server request timed out")
        try:
            item = self._messages.get(timeout=remaining)
        except queue.Empty as exc:
            raise TimeoutError("Codex app-server request timed out") from exc
        if isinstance(item, BaseException):
            raise item
        return item

    def _interrupt_unlocked(self, *, deadline: float) -> None:
        if self._active_turn is None or not self._healthy_unlocked():
            return
        thread_id, turn_id = self._active_turn
        try:
            self._request_unlocked(
                "turn/interrupt",
                {"threadId": thread_id, "turnId": turn_id},
                deadline=deadline,
            )
        except Exception:
            self._stop_unlocked()
        finally:
            self._active_turn = None

    def _stop_unlocked(self) -> None:
        process = self._process
        self._process = None
        self._active_turn = None
        if process is None:
            return
        if process.stdin is not None:
            try:
                process.stdin.close()
            except OSError:
                pass
        if process.poll() is None:
            try:
                process.terminate()
                process.wait(timeout=2.0)
            except (OSError, subprocess.TimeoutExpired):
                try:
                    process.kill()
                    process.wait(timeout=2.0)
                except (OSError, subprocess.TimeoutExpired):
                    pass


def codex_oauth_environment() -> dict[str, str]:
    """Return only non-secret process settings needed by persisted OAuth."""

    allowed = {
        "CODEX_HOME",
        "CODEX_SQLITE_HOME",
        "HOME",
        "LANG",
        "LC_ALL",
        "LOGNAME",
        "NO_PROXY",
        "PATH",
        "SSL_CERT_DIR",
        "SSL_CERT_FILE",
        "TMPDIR",
        "USER",
        "XDG_CACHE_HOME",
        "XDG_CONFIG_HOME",
        "XDG_DATA_HOME",
        "XDG_STATE_HOME",
        "http_proxy",
        "https_proxy",
        "no_proxy",
    }
    return {
        name: value
        for name, value in os.environ.items()
        if name in allowed
    }
