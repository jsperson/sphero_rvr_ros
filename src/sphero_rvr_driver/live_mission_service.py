"""ROS-free live-state boundary for the persistent Mission Service.

ROS callbacks feed a truthful receipt-time cache.  Read-only bindings are always
available; physical route bindings remain unhealthy unless a separately reviewed
configuration explicitly installs authority.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import threading
import time
from typing import Any, Mapping, Optional

from .mission_api import (
    CompletedExecutionHandle,
    MissionValidationError,
    ToolDefinition,
    ToolInvocation,
    ToolResult,
    ToolResultStatus,
)
from .mission_service import ExecutorBinding


LIVE_SOURCE_NAMES = (
    "odom",
    "collision",
    "control",
    "route_progress",
    "camera",
    "lidar",
    "localization",
    "semantic_map",
)


@dataclass(frozen=True)
class LiveSourceRecord:
    value: Mapping[str, Any]
    received_at_s: Optional[float]
    source_timestamp_s: Optional[float]
    valid: bool
    error: str


@dataclass(frozen=True)
class LiveStateSnapshot:
    observed_at_s: float
    sources: Mapping[str, LiveSourceRecord]

    def source(self, name: str) -> LiveSourceRecord:
        return self.sources.get(str(name), _missing_record())


class LiveStateCache:
    """Thread-safe evidence cache populated only by live ROS callbacks."""

    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._records = {name: _missing_record() for name in LIVE_SOURCE_NAMES}

    def update(
        self,
        name: str,
        value: Mapping[str, Any],
        *,
        received_at_s: float,
        source_timestamp_s: Optional[float] = None,
    ) -> None:
        source_name = _source_name(name)
        received = _finite_time(received_at_s, "live ROS receipt time")
        source_timestamp = (
            None
            if source_timestamp_s is None
            else _finite_time(source_timestamp_s, "live source timestamp")
        )
        payload = _safe_mapping(value)
        with self._lock:
            self._records[source_name] = LiveSourceRecord(
                value=payload,
                received_at_s=received,
                source_timestamp_s=source_timestamp,
                valid=True,
                error="",
            )

    def mark_invalid(
        self,
        name: str,
        error: str,
        *,
        received_at_s: float,
        source_timestamp_s: Optional[float] = None,
    ) -> None:
        source_name = _source_name(name)
        received = _finite_time(received_at_s, "live ROS receipt time")
        source_timestamp = (
            None
            if source_timestamp_s is None
            else _finite_time(source_timestamp_s, "live source timestamp")
        )
        reason = str(error).strip() or "malformed live source payload"
        with self._lock:
            self._records[source_name] = LiveSourceRecord(
                value={},
                received_at_s=received,
                source_timestamp_s=source_timestamp,
                valid=False,
                error=reason,
            )

    def update_odom(self, value: Mapping[str, Any], *, received_at_s: float) -> None:
        self.update("odom", value, received_at_s=received_at_s)

    def update_collision(self, value: Mapping[str, Any], *, received_at_s: float) -> None:
        self.update("collision", value, received_at_s=received_at_s)

    def update_route(self, value: Mapping[str, Any], *, received_at_s: float) -> None:
        self.update("route_progress", value, received_at_s=received_at_s)

    def snapshot(self, *, now_s: Optional[float] = None) -> LiveStateSnapshot:
        observed = _finite_time(time.time() if now_s is None else now_s, "live snapshot time")
        with self._lock:
            records = {
                name: LiveSourceRecord(
                    value=dict(record.value),
                    received_at_s=record.received_at_s,
                    source_timestamp_s=record.source_timestamp_s,
                    valid=record.valid,
                    error=record.error,
                )
                for name, record in self._records.items()
            }
        return LiveStateSnapshot(observed_at_s=observed, sources=records)


def snapshot_evidence(snapshot: LiveStateSnapshot, *, max_age_s: float) -> dict[str, Any]:
    """Summarize every source without manufacturing unavailable health."""

    age_limit = float(max_age_s)
    if not math.isfinite(age_limit) or age_limit <= 0.0:
        raise ValueError("max source age must be positive and finite")

    def evidence(record: LiveSourceRecord) -> dict[str, Any]:
        present = record.received_at_s is not None
        age = None if not present else max(0.0, snapshot.observed_at_s - float(record.received_at_s))
        fresh = bool(present and record.valid and age is not None and age <= age_limit)
        return {
            "present": present,
            "valid": bool(record.valid),
            "fresh": fresh,
            "age_s": age,
            "received_at_s": record.received_at_s,
            "source_timestamp_s": record.source_timestamp_s,
            "error": record.error,
            "value": dict(record.value),
        }

    payload = {
        name: evidence(snapshot.source(name))
        for name in LIVE_SOURCE_NAMES
    }
    payload.update(
        {
            "observed_at_s": snapshot.observed_at_s,
            "max_source_age_s": age_limit,
        }
    )
    return payload


class LiveStatusExecutor:
    """Read-only physical-mode executor backed by the node's live cache."""

    cooperative_execution = True
    execution_mode = "physical"
    authority_kind = "physical"
    healthy = True
    health_reason = "live no-motion status reader is available"
    evidence_level = "live_ros_receipt"
    supported_tool_ids = ("query_status_telemetry",)
    satisfied_preconditions: tuple[str, ...] = ()

    def __init__(
        self,
        cache: LiveStateCache,
        *,
        source_sha: str,
        deployed_sha: str,
        max_source_age_s: float = 1.0,
    ) -> None:
        self.cache = cache
        self.source_sha = _required_provenance(source_sha, "source_sha")
        self.deployed_sha = _required_provenance(deployed_sha, "deployed_sha")
        self.max_source_age_s = float(max_source_age_s)

    def evidence(self, *, now_s: Optional[float] = None) -> dict[str, Any]:
        snapshot = self.cache.snapshot(now_s=now_s)
        evidence = snapshot_evidence(snapshot, max_age_s=self.max_source_age_s)
        evidence.update(
            {
                "executor": self.__class__.__name__,
                "source_sha": self.source_sha,
                "deployed_sha": self.deployed_sha,
                "motion_authority": False,
                "route_submission_enabled": False,
                "status_state": self._state(snapshot),
                "safety": self._safety(snapshot),
            }
        )
        return evidence

    def begin_execution(
        self,
        invocation: ToolInvocation,
        definition: ToolDefinition,
        *,
        started_at_s: float,
        index: int,
    ) -> CompletedExecutionHandle:
        del definition, index
        if invocation.tool_id != "query_status_telemetry":
            return CompletedExecutionHandle(
                _blocked_result(invocation, started_at_s, "live status executor is read-only")
            )
        snapshot = self.cache.snapshot()
        return CompletedExecutionHandle(
            ToolResult(
                invocation=invocation,
                status=ToolResultStatus.COMPLETE,
                started_at_s=started_at_s,
                completed_at_s=started_at_s,
                observation={
                    "state": self._state(snapshot),
                    "evidence": snapshot_evidence(snapshot, max_age_s=self.max_source_age_s),
                },
                provenance={
                    "adapter": "live_mission_service/status",
                    "source_sha": self.source_sha,
                    "deployed_sha": self.deployed_sha,
                    "motion_authority": False,
                },
            )
        )

    @staticmethod
    def _state(snapshot: LiveStateSnapshot) -> str:
        collision = snapshot.source("collision").value
        control = snapshot.source("control").value
        route = snapshot.source("route_progress").value
        collision_state = str(collision.get("state", "")).upper()
        control_state = str(control.get("state", "")).upper()
        route_status = str(route.get("status", "")).lower()
        if (
            collision_state in {"ESTOP", "ESTOPPED", "LATCHED"}
            or control_state in {"ESTOP", "ESTOPPED", "LATCHED"}
            or bool(control.get("estop_latched", False))
            or route_status == "estopped"
        ):
            return "ESTOPPED"
        if (
            collision_state in {"STOP", "STOPPED"}
            or control_state in {"STOP", "STOPPED"}
            or bool(control.get("stop_active", False))
            or route_status == "stopped"
        ):
            return "STOPPED"
        if control_state in {"CANCEL", "CANCELLED"} or route_status == "cancelled":
            return "CANCELLED"
        if not any(record.received_at_s is not None for record in snapshot.sources.values()):
            return "NO_MOTION_OFFLINE"
        return "NO_MOTION_MONITORING"

    def _safety(self, snapshot: LiveStateSnapshot) -> dict[str, Any]:
        collision = snapshot.source("collision")
        control = snapshot.source("control")
        route = snapshot.source("route_progress")
        collision_state = str(collision.value.get("state", "UNKNOWN")).upper()
        control_state = str(control.value.get("state", "UNKNOWN")).upper()
        route_status = str(route.value.get("status", "unknown")).lower()
        stop_active = (
            collision_state in {"STOP", "STOPPED"}
            or control_state in {"STOP", "STOPPED"}
            or bool(control.value.get("stop_active", False))
            or route_status == "stopped"
        )
        estop_latched = (
            collision_state in {"ESTOP", "ESTOPPED", "LATCHED"}
            or control_state in {"ESTOP", "ESTOPPED", "LATCHED"}
            or bool(control.value.get("estop_latched", False))
            or route_status == "estopped"
        )
        control_known = bool(
            control.received_at_s is not None
            and control.valid
            and snapshot.observed_at_s - float(control.received_at_s) <= self.max_source_age_s
        )
        collision_known = bool(
            collision.received_at_s is not None
            and collision.valid
            and snapshot.observed_at_s - float(collision.received_at_s) <= self.max_source_age_s
        )
        safety_known = control_known or collision_known
        return {
            "collision_state": collision_state,
            "front_clearance_m": collision.value.get("front_clearance_m"),
            "forward_corridor_clearance_m": collision.value.get(
                "forward_corridor_clearance_m"
            ),
            "forward_corridor_min_angle_deg": collision.value.get(
                "forward_corridor_min_angle_deg"
            ),
            "forward_corridor_max_angle_deg": collision.value.get(
                "forward_corridor_max_angle_deg"
            ),
            "collision_stop_distance_m": collision.value.get(
                "collision_stop_distance_m"
            ),
            "collision_slow_distance_m": collision.value.get(
                "collision_slow_distance_m"
            ),
            "control_state": control_state,
            "control_present": control.received_at_s is not None,
            "control_valid": control.valid,
            "stop_active": stop_active,
            "estop_latched": estop_latched,
            "stop_state": "ACTIVE" if stop_active else ("READY" if safety_known else "UNKNOWN"),
            "estop_state": "LATCHED" if estop_latched else ("CLEAR" if safety_known else "UNKNOWN"),
            "route_status": route_status,
            "independent_robot_safety": True,
            "browser_is_safety_authority": False,
        }


class LiveRouteProgressExecutor:
    """Live route evidence binding with physical execution intentionally disabled."""

    cooperative_execution = True
    execution_mode = "physical"
    authority_kind = "physical"
    healthy = False
    health_reason = "physical route authority is disabled pending measured-route gate"
    evidence_level = "live_ros_receipt"
    supported_tool_ids = ("move_distance", "turn_angle")
    satisfied_preconditions: tuple[str, ...] = ()

    def __init__(self, status_executor: LiveStatusExecutor) -> None:
        self.status_executor = status_executor

    def evidence(self, *, now_s: Optional[float] = None) -> dict[str, Any]:
        evidence = self.status_executor.evidence(now_s=now_s)
        evidence.update(
            {
                "executor": self.__class__.__name__,
                "motion_authority": False,
                "route_submission_enabled": False,
                "health_reason": self.health_reason,
            }
        )
        return evidence

    def begin_execution(
        self,
        invocation: ToolInvocation,
        definition: ToolDefinition,
        *,
        started_at_s: float,
        index: int,
    ) -> CompletedExecutionHandle:
        del definition, index
        return CompletedExecutionHandle(_blocked_result(invocation, started_at_s, self.health_reason))


class SafetyGatedPromptRouteExecutor:
    """Delegate an approved route only while authoritative safety evidence is ready."""

    def __init__(self, status_executor: LiveStatusExecutor, route_executor: Any) -> None:
        self.status_executor = status_executor
        self.route_executor = route_executor

    def assert_ready(self) -> Mapping[str, Any]:
        evidence = self.status_executor.evidence()
        for source in ("odom", "collision"):
            source_evidence = evidence.get(source, {})
            if not isinstance(source_evidence, Mapping) or not bool(source_evidence.get("fresh", False)):
                raise MissionValidationError(
                    f"live route approval requires fresh authoritative {source} evidence"
                )
        safety = evidence.get("safety", {})
        if not isinstance(safety, Mapping):
            raise MissionValidationError("live route approval requires authoritative safety evidence")
        if str(safety.get("collision_state", "UNKNOWN")).upper() != "CLEAR":
            raise MissionValidationError("live route approval requires collision state CLEAR")
        if safety.get("stop_state") != "READY":
            raise MissionValidationError("live route approval requires STOP state READY")
        if safety.get("estop_state") != "CLEAR":
            raise MissionValidationError("live route approval requires ESTOP state CLEAR")
        return evidence

    def execute(self, request: Any) -> Mapping[str, Any]:
        self.assert_ready()
        return self.route_executor.execute(request)

    def cancel(self) -> bool:
        return bool(self.route_executor.cancel())


def live_executor_bindings(
    status_executor: LiveStatusExecutor,
    route_executor: LiveRouteProgressExecutor,
    *,
    heartbeat_at_s: float,
    max_binding_age_s: float = 2.0,
) -> dict[str, ExecutorBinding]:
    """Bind only the real executors installed by the production ROS node."""

    common = {
        "mode": "live",
        "credential_namespace": "physical",
        "heartbeat_at_s": float(heartbeat_at_s),
        "max_age_s": float(max_binding_age_s),
    }
    return {
        "query_status_telemetry": ExecutorBinding(
            executor=status_executor,
            evidence=status_executor.evidence(now_s=heartbeat_at_s),
            **common,
        ),
        "move_distance": ExecutorBinding(
            executor=route_executor,
            evidence=route_executor.evidence(now_s=heartbeat_at_s),
            **common,
        ),
        "turn_angle": ExecutorBinding(
            executor=route_executor,
            evidence=route_executor.evidence(now_s=heartbeat_at_s),
            **common,
        ),
    }


def _missing_record() -> LiveSourceRecord:
    return LiveSourceRecord({}, None, None, False, "source has not been observed")


def _source_name(name: str) -> str:
    normalized = str(name).strip()
    if normalized not in LIVE_SOURCE_NAMES:
        raise ValueError(f"unsupported live source: {normalized}")
    return normalized


def _finite_time(value: Any, label: str) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{label} must be finite")
    return result


def _safe_mapping(value: Mapping[str, Any]) -> dict[str, Any]:
    if not isinstance(value, Mapping):
        raise ValueError("live source payload must be an object")
    return {str(key): _safe_value(item) for key, item in value.items()}


def _safe_value(value: Any) -> Any:
    if value is None or isinstance(value, (str, bool, int)):
        return value
    if isinstance(value, float):
        if not math.isfinite(value):
            raise ValueError("live source payload contains a non-finite number")
        return value
    if isinstance(value, Mapping):
        return _safe_mapping(value)
    if isinstance(value, (list, tuple)):
        return [_safe_value(item) for item in value]
    raise ValueError(f"live source payload contains unsupported value: {type(value).__name__}")


def _required_provenance(value: str, name: str) -> str:
    result = str(value).strip()
    if not result or result.lower() == "unknown":
        raise ValueError(f"{name} is required")
    return result


def _blocked_result(invocation: ToolInvocation, started_at_s: float, reason: str) -> ToolResult:
    return ToolResult(
        invocation=invocation,
        status=ToolResultStatus.BLOCKED,
        started_at_s=started_at_s,
        completed_at_s=started_at_s,
        error={"message": str(reason)},
        provenance={"adapter": "live_mission_service/no_motion", "motion_authority": False},
    )
