"""Bounded physical-drive trace capture and deterministic summary metrics."""

from __future__ import annotations

from dataclasses import dataclass, field
import json
import math
import os
from pathlib import Path
import re
import threading
from typing import Any, Mapping, Optional


TRACE_SAMPLE_SCHEMA = "sphero_rvr.physical_drive_trace_sample.v1"
TRACE_SUMMARY_SCHEMA = "sphero_rvr.physical_drive_trace_summary.v1"
_MISSION_ID_RE = re.compile(r"[A-Za-z0-9][A-Za-z0-9._-]{0,127}")


def _finite(value: Any, name: str) -> float:
    try:
        parsed = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be finite") from exc
    if not math.isfinite(parsed):
        raise ValueError(f"{name} must be finite")
    return parsed


def _sign(value: float, *, threshold: float = 0.01) -> int:
    if value > threshold:
        return 1
    if value < -threshold:
        return -1
    return 0


@dataclass
class _CommandMetrics:
    samples: int = 0
    nonzero_samples: int = 0
    angular_sign_reversals: int = 0
    max_abs_linear_mps: float = 0.0
    max_abs_angular_rad_s: float = 0.0
    last_nonzero_angular_sign: int = 0

    def record(self, linear_x: float, angular_z: float) -> None:
        self.samples += 1
        if abs(linear_x) > 0.001 or abs(angular_z) > 0.01:
            self.nonzero_samples += 1
        self.max_abs_linear_mps = max(
            self.max_abs_linear_mps, abs(linear_x)
        )
        self.max_abs_angular_rad_s = max(
            self.max_abs_angular_rad_s, abs(angular_z)
        )
        sign = _sign(angular_z)
        if sign:
            if (
                self.last_nonzero_angular_sign
                and sign != self.last_nonzero_angular_sign
            ):
                self.angular_sign_reversals += 1
            self.last_nonzero_angular_sign = sign

    def to_json_dict(self) -> dict[str, Any]:
        return {
            "samples": self.samples,
            "nonzero_samples": self.nonzero_samples,
            "angular_sign_reversals": self.angular_sign_reversals,
            "max_abs_linear_mps": self.max_abs_linear_mps,
            "max_abs_angular_rad_s": self.max_abs_angular_rad_s,
        }


@dataclass
class DriveTraceMetrics:
    """Compute command oscillation and physical progress without ROS."""

    command_streams: dict[str, _CommandMetrics] = field(default_factory=dict)
    odom_samples: int = 0
    odom_origin: Optional[tuple[float, float]] = None
    max_displacement_m: float = 0.0
    first_yaw_rad: Optional[float] = None
    last_yaw_rad: Optional[float] = None
    collision_samples: int = 0
    controller_state_samples: int = 0

    def record_command(
        self, stream: str, linear_x: Any, angular_z: Any
    ) -> dict[str, Any]:
        name = str(stream).strip()
        if name not in {
            "nav2_request",
            "supervisor_request",
            "motor_output",
        }:
            raise ValueError("drive trace command stream is invalid")
        linear = _finite(linear_x, "linear command")
        angular = _finite(angular_z, "angular command")
        self.command_streams.setdefault(name, _CommandMetrics()).record(
            linear, angular
        )
        return {
            "kind": "twist",
            "stream": name,
            "linear_x": linear,
            "angular_z": angular,
        }

    def record_odom(
        self, x_m: Any, y_m: Any, yaw_rad: Any
    ) -> dict[str, Any]:
        x_value = _finite(x_m, "odom x")
        y_value = _finite(y_m, "odom y")
        yaw_value = _finite(yaw_rad, "odom yaw")
        if self.odom_origin is None:
            self.odom_origin = (x_value, y_value)
            self.first_yaw_rad = yaw_value
        self.odom_samples += 1
        self.last_yaw_rad = yaw_value
        self.max_displacement_m = max(
            self.max_displacement_m,
            math.hypot(
                x_value - self.odom_origin[0],
                y_value - self.odom_origin[1],
            ),
        )
        return {
            "kind": "odom",
            "x_m": x_value,
            "y_m": y_value,
            "yaw_rad": yaw_value,
        }

    def record_state(self, stream: str, value: Any) -> dict[str, Any]:
        name = str(stream).strip()
        if name == "collision":
            self.collision_samples += 1
        elif name in {"hierarchical_controller", "hierarchical_adapter"}:
            self.controller_state_samples += 1
        else:
            raise ValueError("drive trace state stream is invalid")
        payload: Any = value
        if isinstance(value, Mapping):
            payload = dict(value)
        return {"kind": "state", "stream": name, "value": payload}

    def summary(self) -> dict[str, Any]:
        return {
            "schema": TRACE_SUMMARY_SCHEMA,
            "command_streams": {
                name: metrics.to_json_dict()
                for name, metrics in sorted(self.command_streams.items())
            },
            "odom_samples": self.odom_samples,
            "max_displacement_m": self.max_displacement_m,
            "first_yaw_rad": self.first_yaw_rad,
            "last_yaw_rad": self.last_yaw_rad,
            "collision_samples": self.collision_samples,
            "controller_state_samples": self.controller_state_samples,
        }


class BoundedDriveTrace:
    """Append compact samples while bounding each mission to two segments."""

    def __init__(
        self,
        directory: Path,
        *,
        mission_id: str,
        source_sha: str,
        max_segment_bytes: int = 8_000_000,
        retained_files: int = 16,
    ) -> None:
        identifier = str(mission_id).strip()
        if not _MISSION_ID_RE.fullmatch(identifier):
            raise ValueError("drive trace mission ID is invalid")
        provenance = str(source_sha).strip()
        if len(provenance) != 40 or any(
            character not in "0123456789abcdef" for character in provenance
        ):
            raise ValueError("drive trace source SHA is invalid")
        if max_segment_bytes < 4096:
            raise ValueError("drive trace segment bound is too small")
        if retained_files < 2:
            raise ValueError("drive trace retention must keep at least two files")
        self.directory = Path(directory).expanduser().resolve()
        self.directory.mkdir(parents=True, exist_ok=True)
        os.chmod(self.directory, 0o700)
        self.mission_id = identifier
        self.source_sha = provenance
        self.max_segment_bytes = int(max_segment_bytes)
        self.retained_files = int(retained_files)
        self.path = self.directory / f"drive-trace-{identifier}.jsonl"
        self.previous_path = (
            self.directory / f"drive-trace-{identifier}.previous.jsonl"
        )
        self.metrics = DriveTraceMetrics()
        self._lock = threading.RLock()
        self._closed = False
        self._stream = self.path.open("a", encoding="utf-8")
        os.chmod(self.path, 0o600)
        self._write(
            {
                "schema": TRACE_SAMPLE_SCHEMA,
                "kind": "trace_started",
                "mission_id": self.mission_id,
                "source_sha": self.source_sha,
            }
        )
        self._prune()

    def record(
        self,
        sample: Mapping[str, Any],
        *,
        recorded_at_s: Any,
        monotonic_s: Any,
    ) -> None:
        event = {
            "schema": TRACE_SAMPLE_SCHEMA,
            "mission_id": self.mission_id,
            "source_sha": self.source_sha,
            "recorded_at_s": _finite(recorded_at_s, "trace receipt time"),
            "monotonic_s": _finite(monotonic_s, "trace monotonic time"),
            **dict(sample),
        }
        with self._lock:
            if not self._closed:
                self._write(event)

    def flush(self) -> None:
        with self._lock:
            if not self._closed:
                self._stream.flush()

    def close(self) -> None:
        with self._lock:
            if self._closed:
                return
            self._write(
                {
                    "schema": TRACE_SAMPLE_SCHEMA,
                    "kind": "trace_summary",
                    "mission_id": self.mission_id,
                    "source_sha": self.source_sha,
                    "summary": self.metrics.summary(),
                }
            )
            self._stream.flush()
            os.fsync(self._stream.fileno())
            self._stream.close()
            self._closed = True

    def _write(self, event: Mapping[str, Any]) -> None:
        encoded = json.dumps(
            dict(event), sort_keys=True, separators=(",", ":")
        ) + "\n"
        if (
            self._stream.tell() > 0
            and self._stream.tell() + len(encoded.encode("utf-8"))
            > self.max_segment_bytes
        ):
            self._stream.flush()
            os.fsync(self._stream.fileno())
            self._stream.close()
            self.previous_path.unlink(missing_ok=True)
            os.replace(self.path, self.previous_path)
            self._stream = self.path.open("w", encoding="utf-8")
            os.chmod(self.path, 0o600)
        self._stream.write(encoded)

    def _prune(self) -> None:
        retained = sorted(
            self.directory.glob("drive-trace-*.jsonl"),
            key=lambda path: path.stat().st_mtime_ns,
            reverse=True,
        )
        for expired in retained[self.retained_files :]:
            expired.unlink(missing_ok=True)
