"""Default-off live M6 semantic controller for the M7 physical binding.

This node owns no motion API. It converts live map/perception evidence into the
existing M6 semantic snapshot, runs the toolless semantic-goal provider, and
publishes only a digest-bound semantic dispatch for the deterministic Nav2
adapter.
"""

from __future__ import annotations

from concurrent.futures import Future, ThreadPoolExecutor
import hashlib
import json
import math
import os
from pathlib import Path
import time
from typing import Any, Mapping, Optional

from .hierarchical_exploration import OccupancyGrid, detect_frontiers
from .hierarchical_goal_selection import (
    AsyncSemanticGoalController,
    CodexOAuthSemanticGoalProvider,
    SemanticEventKind,
    SemanticReplanEvent,
    SemanticTrack,
    build_semantic_world_snapshot,
    generate_next_best_views,
    revalidate_resolved_goal,
    semantic_goal_prompt,
)
from .hierarchical_physical_binding import (
    AUTHORITY_HEARTBEAT_MAX_AGE_S,
    AUTHORITY_TOPIC,
    CONTROLLER_STATUS_TOPIC,
    GOAL_DISPATCH_TOPIC,
    LOCALIZATION_MAX_AGE_S,
    HierarchicalBindingJournal,
    build_goal_dispatch,
    resolve_goal_dispatch,
    transient_authority_hold,
    validate_authority_heartbeat,
    validate_physical_proposal,
)
from .hierarchical_m7_canonical_validation import (
    validate_active_graph_evidence,
)
from .mission_api import MissionValidationError


MAX_CONSECUTIVE_SEMANTIC_REJECTIONS = 3
COLLISION_EVIDENCE_MAX_AGE_S = 0.300
CAMERA_EVIDENCE_MAX_AGE_S = 3.0
MAP_EVIDENCE_MAX_AGE_S = 3.0
TARGET_INVALIDATION_REASONS = {
    "event_generation_changed",
    "map_identity_changed",
    "frontier_signature_invalidated",
    "track_signature_changed",
    "track_position_changed",
    "viewpoint_invalidated",
}


def _json_object(value: Any, name: str) -> dict[str, Any]:
    try:
        parsed = json.loads(str(value))
    except (TypeError, ValueError, json.JSONDecodeError) as exc:
        raise MissionValidationError(f"{name} must be a JSON object") from exc
    if not isinstance(parsed, Mapping):
        raise MissionValidationError(f"{name} must be a JSON object")
    return dict(parsed)


def _sha1(value: str, name: str) -> str:
    parsed = str(value).strip()
    if len(parsed) != 40 or any(character not in "0123456789abcdef" for character in parsed):
        raise MissionValidationError(f"{name} must be an exact lowercase Git SHA")
    return parsed


def _stamp_s(stamp: Any) -> float:
    value = float(stamp.sec) + float(stamp.nanosec) / 1_000_000_000.0
    return value if value > 0.0 else time.time()


def _yaw(orientation: Any) -> float:
    return math.atan2(
        2.0
        * (
            float(orientation.w) * float(orientation.z)
            + float(orientation.x) * float(orientation.y)
        ),
        1.0
        - 2.0
        * (
            float(orientation.y) ** 2
            + float(orientation.z) ** 2
        ),
    )


def live_semantic_track_signature(raw: Mapping[str, Any]) -> str:
    """Bind stable semantic identity, not rolling frame evidence."""

    identity = {
        "track_id": str(raw.get("track_id", "")).strip(),
        "kind": str(raw.get("kind", "object")).strip(),
        "label": str(raw.get("label", raw.get("kind", "object"))).strip(),
        "recognized_from_enrollment": bool(
            raw.get("recognized_from_enrollment", False)
        ),
        "enrollment_evidence_ids": sorted(
            str(item)
            for item in raw.get("enrollment_evidence_ids", ())
        ),
    }
    if not identity["track_id"] or not identity["label"]:
        raise MissionValidationError("live semantic track identity is invalid")
    return hashlib.sha256(
        json.dumps(
            identity,
            sort_keys=True,
            separators=(",", ":"),
            allow_nan=False,
        ).encode("utf-8")
    ).hexdigest()


def semantic_target_invalidation_reason(
    goal: Any,
    captured_snapshot: Mapping[str, Any],
    current_snapshot: Mapping[str, Any],
) -> str:
    result = revalidate_resolved_goal(
        goal,
        captured_snapshot=captured_snapshot,
        current_snapshot=current_snapshot,
    )
    invalidations = [
        reason
        for reason in result.reasons
        if reason in TARGET_INVALIDATION_REASONS
    ]
    return ",".join(invalidations)


def updated_semantic_rejection_count(
    previous: int, events: Any
) -> int:
    count = max(0, int(previous))
    normalized = [
        event for event in events if isinstance(event, Mapping)
    ]
    count += sum(
        str(event.get("kind", "")) == "prefetch_discarded"
        for event in normalized
    )
    if any(
        str(event.get("kind", ""))
        in {
            "prefetch_revalidated",
            "semantic_non_motion_goal_ready",
        }
        for event in normalized
    ):
        return 0
    return count


def adapter_remaining_distance(
    status: Mapping[str, Any], fallback_remaining_m: float
) -> float:
    """Use Nav2 distance only when it represents feedback or success."""

    fallback = max(0.0, float(fallback_remaining_m))
    state = str(status.get("state", "")).strip()
    reason = str(status.get("reason", "")).strip()
    trustworthy = (
        state == "navigating" and status.get("goal_active") is True
    ) or (
        state == "wait_planning"
        and reason in {
            "nav2_result_status_4",
            "nav2_result_status_6",
        }
        and status.get("goal_active") is False
    )
    if not trustworthy:
        return fallback
    try:
        remaining = float(status.get("distance_remaining_m"))
    except (TypeError, ValueError):
        return fallback
    if not math.isfinite(remaining) or remaining < 0.0:
        return fallback
    return remaining


def adapter_recovery_reason(status: Mapping[str, Any]) -> str:
    """Return a fail-closed Nav2 execution failure, if present."""

    if str(status.get("state", "")).strip() != "recovery_required":
        return ""
    reason = str(status.get("reason", "")).strip()
    return reason or "nav2_recovery_required"


def rolling_frontier_invalidation_preserves_route(
    invalidation_reason: str,
    adapter_status: Mapping[str, Any],
) -> bool:
    """Let Nav2 finish a safe accepted route through rolling-map churn."""

    return (
        str(invalidation_reason).strip()
        == "frontier_signature_invalidated"
        and str(adapter_status.get("state", "")).strip() == "navigating"
        and adapter_status.get("goal_active") is True
    )


def planning_hold_controller_state(status: Mapping[str, Any]) -> str:
    """Keep an accepted Nav2 route alive while planning evidence catches up."""

    if (
        str(status.get("state", "")).strip()
        not in {"locked", "recovery_required", "rejected"}
        and status.get("goal_active") is True
    ):
        return "navigating"
    return "wait_planning"


def semantic_non_motion_status(action: str) -> tuple[str, str, bool]:
    """Map a validated non-motion semantic action to controller state."""

    normalized = str(action).strip()
    if normalized == "finish":
        return "complete", "finish", True
    if normalized == "wait":
        return "wait_planning", "semantic_wait", False
    raise MissionValidationError(
        "unsupported non-motion semantic action"
    )


def parse_collision_evidence(value: Any) -> tuple[str, bool]:
    """Extract the supervisor state and scan-health bit without trusting defaults."""

    raw = str(value).strip()
    state = "BLOCKED"
    scan_healthy = False
    try:
        parsed = json.loads(raw)
    except (TypeError, ValueError, json.JSONDecodeError):
        parsed = None
    if isinstance(parsed, Mapping):
        state = str(parsed.get("state", "BLOCKED")).upper()
        scan_healthy = parsed.get("scan_healthy") is True
    elif raw:
        tokens = raw.split()
        state = tokens[0].upper()
        fields = {
            key: field
            for token in tokens[1:]
            for key, separator, field in (token.partition("="),)
            if separator
        }
        scan_healthy = fields.get("scan_healthy", "").lower() == "true"
    if state not in {"CLEAR", "SLOW"}:
        state = "BLOCKED"
    return state, scan_healthy


def live_motion_evidence_is_fresh(
    *,
    now_s: float,
    collision_received_at_s: Optional[float],
    scan_healthy: bool,
    max_age_s: float = COLLISION_EVIDENCE_MAX_AGE_S,
) -> bool:
    """Require a recent healthy collision-supervisor receipt for motion."""

    if collision_received_at_s is None or scan_healthy is not True:
        return False
    try:
        now = float(now_s)
        received = float(collision_received_at_s)
        maximum = float(max_age_s)
    except (TypeError, ValueError):
        return False
    age_s = now - received
    return (
        math.isfinite(now)
        and math.isfinite(received)
        and math.isfinite(maximum)
        and maximum == COLLISION_EVIDENCE_MAX_AGE_S
        and 0.0 <= age_s <= maximum
    )


def live_source_is_fresh(
    *,
    now_s: float,
    received_at_s: Optional[float],
    source_timestamp_s: Optional[float],
    max_age_s: float,
) -> bool:
    """Require both source and local receipt ages to remain in bounds."""

    if received_at_s is None or source_timestamp_s is None:
        return False
    try:
        now = float(now_s)
        received = float(received_at_s)
        source = float(source_timestamp_s)
        maximum = float(max_age_s)
    except (TypeError, ValueError):
        return False
    return (
        math.isfinite(now)
        and math.isfinite(received)
        and math.isfinite(source)
        and math.isfinite(maximum)
        and maximum > 0.0
        and 0.0 <= now - received <= maximum
        and 0.0 <= now - source <= maximum
    )


def bounded_camera_evidence(value: Mapping[str, Any]) -> dict[str, Any]:
    """Retain bounded perception evidence without thumbnails or raw pixels."""

    raw_attachment = value.get("image_attachment", {})
    image_attachment = (
        {
            key: raw_attachment[key]
            for key in {
                "schema",
                "frame_id",
                "path",
                "mime_type",
                "sha256",
                "byte_count",
            }
            if key in raw_attachment
        }
        if isinstance(raw_attachment, Mapping)
        else {}
    )
    allowed_detection_fields = {
        "kind",
        "label",
        "confidence",
        "status",
        "track_id",
        "bbox",
        "position_method",
        "calibration_id",
        "map_revision",
        "localization_evidence_ids",
        "localization_reason",
        "source_timestamps_ns",
        "bearing",
    }
    detections = []
    raw_detections = value.get("detections", ())
    if isinstance(raw_detections, list):
        for raw in raw_detections[:32]:
            if isinstance(raw, Mapping):
                detections.append(
                    {
                        key: raw[key]
                        for key in allowed_detection_fields
                        if key in raw
                    }
                )
    candidate = {
        "schema": str(value.get("schema", "")),
        "frame_id": str(value.get("frame_id", "")),
        "stamp_s": value.get("stamp_s"),
        "width": value.get("width"),
        "height": value.get("height"),
        "calibrated": value.get("calibrated") is True,
        "uncertain_track_id": str(
            value.get("uncertain_track_id", "")
        ),
        "detections": detections,
        "image_attachment": image_attachment,
    }
    try:
        return json.loads(
            json.dumps(
                candidate,
                sort_keys=True,
                separators=(",", ":"),
                allow_nan=False,
            )
        )
    except (TypeError, ValueError):
        return {}


def nav2_path_evidence(
    message: Any,
    *,
    source_sha: str,
    mission_id: str,
    dispatch_digest: str,
    goal_batch_digest: str,
    recorded_at_s: Optional[float] = None,
    maximum_poses: int = 512,
) -> dict[str, Any]:
    """Capture a bounded server-planned map path for durable evidence."""

    if maximum_poses < 2:
        raise MissionValidationError(
            "Nav2 path evidence requires at least two pose slots"
        )
    header = getattr(message, "header", None)
    frame_id = str(getattr(header, "frame_id", "")).strip()
    raw_poses = tuple(getattr(message, "poses", ()))
    if frame_id != "map" or not raw_poses:
        raise MissionValidationError(
            "Nav2 path evidence requires a nonempty map-frame path"
        )
    if len(raw_poses) <= maximum_poses:
        indices = tuple(range(len(raw_poses)))
    else:
        indices = tuple(
            sorted(
                {
                    round(
                        index
                        * (len(raw_poses) - 1)
                        / (maximum_poses - 1)
                    )
                    for index in range(maximum_poses)
                }
            )
        )
    poses = []
    for index in indices:
        stamped = raw_poses[index]
        pose_frame = str(
            getattr(getattr(stamped, "header", None), "frame_id", "")
            or frame_id
        ).strip()
        pose = getattr(stamped, "pose", None)
        position = getattr(pose, "position", None)
        orientation = getattr(pose, "orientation", None)
        try:
            x_m = float(position.x)
            y_m = float(position.y)
            yaw_rad = float(_yaw(orientation))
        except (AttributeError, TypeError, ValueError):
            raise MissionValidationError(
                "Nav2 path evidence contains an invalid pose"
            ) from None
        if (
            pose_frame != "map"
            or not all(
                math.isfinite(item)
                for item in (x_m, y_m, yaw_rad)
            )
        ):
            raise MissionValidationError(
                "Nav2 path evidence contains non-map or nonfinite geometry"
            )
        poses.append(
            {
                "source_index": index,
                "x_m": x_m,
                "y_m": y_m,
                "yaw_rad": yaw_rad,
            }
        )
    stamp = getattr(header, "stamp", None)
    try:
        source_stamp_s = (
            float(stamp.sec)
            + float(stamp.nanosec) / 1_000_000_000.0
        )
    except (AttributeError, TypeError, ValueError):
        raise MissionValidationError(
            "Nav2 path evidence requires a valid source stamp"
        ) from None
    if not math.isfinite(source_stamp_s) or source_stamp_s <= 0.0:
        raise MissionValidationError(
            "Nav2 path evidence requires a valid source stamp"
        )
    content = {
        "schema": "sphero_rvr.hierarchical_nav2_path_evidence.v1",
        "source_sha": str(source_sha).strip(),
        "mission_id": str(mission_id).strip(),
        "dispatch_digest": str(dispatch_digest).strip(),
        "goal_batch_digest": str(goal_batch_digest).strip(),
        "frame_id": frame_id,
        "source_stamp_s": source_stamp_s,
        "original_pose_count": len(raw_poses),
        "sampled_pose_count": len(poses),
        "poses": poses,
    }
    if (
        len(content["source_sha"]) != 40
        or len(content["dispatch_digest"]) != 64
        or len(content["goal_batch_digest"]) != 64
        or not content["mission_id"]
    ):
        raise MissionValidationError(
            "Nav2 path evidence binding is invalid"
        )
    content_digest = hashlib.sha256(
        json.dumps(
            content,
            sort_keys=True,
            separators=(",", ":"),
            allow_nan=False,
        ).encode("utf-8")
    ).hexdigest()
    payload = {
        **content,
        "path_content_digest": content_digest,
        "recorded_at_s": float(
            time.time() if recorded_at_s is None else recorded_at_s
        ),
    }
    payload["path_digest"] = hashlib.sha256(
        json.dumps(
            payload,
            sort_keys=True,
            separators=(",", ":"),
            allow_nan=False,
        ).encode("utf-8")
    ).hexdigest()
    return payload


def goal_dispatch_queue_key(dispatch: Mapping[str, Any]) -> str:
    """Identify semantic queue changes without map-refresh churn."""

    raw_goals = dispatch.get("goals")
    if not isinstance(raw_goals, list) or not raw_goals:
        raise MissionValidationError("goal dispatch queue is unavailable")
    goals = []
    for raw in raw_goals:
        if not isinstance(raw, Mapping):
            raise MissionValidationError("goal dispatch queue is invalid")
        decision = raw.get("decision")
        captured = raw.get("captured_snapshot")
        if not isinstance(decision, Mapping) or not isinstance(
            captured, Mapping
        ):
            raise MissionValidationError("goal dispatch queue is invalid")
        goals.append(
            {
                "decision": dict(decision),
                "captured_snapshot_id": str(
                    captured.get("snapshot_id", "")
                ).strip(),
            }
        )
    stable = {
        "mission_id": str(dispatch.get("mission_id", "")).strip(),
        "source_sha": str(dispatch.get("source_sha", "")).strip(),
        "approval_digest": str(
            dispatch.get("approval_digest", "")
        ).strip(),
        "controller_session": int(dispatch.get("controller_session", 0)),
        "goals": goals,
    }
    return hashlib.sha256(
        json.dumps(
            stable,
            sort_keys=True,
            separators=(",", ":"),
            allow_nan=False,
        ).encode("utf-8")
    ).hexdigest()


def main(args=None):
    import rclpy
    from nav_msgs.msg import OccupancyGrid as RosOccupancyGrid
    from nav_msgs.msg import Path as RosPath
    from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
    from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import String

    class HierarchicalMissionNode(Node):
        def __init__(self):
            super().__init__("hierarchical_mission_controller")
            for name, default in {
                "enabled": False,
                "source_sha": "",
                "deployed_sha": "",
                "reviewed_sha": "",
                "proposal_file": "",
                "graph_audit_file": "",
                "journal_path": (
                    "~/.local/state/sphero_rvr/"
                    "hierarchical-physical-evidence.sqlite3"
                ),
                "map_topic": "/map",
                "localization_topic": "/mission_api/v2/localization/status",
                "semantic_map_topic": "/mission_api/v2/map/status",
                "camera_topic": "/mission_api/v2/camera/status",
                "nav2_path_topic": "/plan",
                "collision_topic": "/collision_stop/state",
                "authority_topic": AUTHORITY_TOPIC,
                "adapter_status_topic": "/mission_api/v2/hierarchical/status",
                "controller_status_topic": CONTROLLER_STATUS_TOPIC,
                "goal_dispatch_topic": GOAL_DISPATCH_TOPIC,
                "planning_model": "gpt-5.6-luna",
                "planning_reasoning_effort": "low",
                "provider_timeout_s": 20.0,
                "provider_p95_s": 14.34809786885,
                "prefetch_margin_s": 1.0,
            }.items():
                self.declare_parameter(name, default)
            if not bool(self.get_parameter("enabled").value):
                raise ValueError(
                    "hierarchical mission controller is default-off; enabled must be explicit"
                )
            self._source_sha = self._required_sha(
                "source_sha", "RVR_SOURCE_SHA"
            )
            self._deployed_sha = self._required_sha(
                "deployed_sha", "RVR_DEPLOYED_SHA"
            )
            self._reviewed_sha = self._required_sha(
                "reviewed_sha", "RVR_HIERARCHICAL_REVIEWED_SHA"
            )
            if not (
                self._source_sha
                == self._deployed_sha
                == self._reviewed_sha
            ):
                raise ValueError(
                    "hierarchical mission controller requires matching exact SHAs"
                )
            if (
                str(self.get_parameter("authority_topic").value)
                != AUTHORITY_TOPIC
                or str(self.get_parameter("goal_dispatch_topic").value)
                != GOAL_DISPATCH_TOPIC
                or str(
                    self.get_parameter("controller_status_topic").value
                )
                != CONTROLLER_STATUS_TOPIC
            ):
                raise ValueError(
                    "hierarchical authority, dispatch, and status topics are fixed"
                )
            proposal_file = Path(
                str(self.get_parameter("proposal_file").value)
            ).expanduser()
            if not proposal_file.is_file():
                raise ValueError(
                    "hierarchical mission controller requires a browser proposal file"
                )
            self._raw_proposal = _json_object(
                proposal_file.read_text(encoding="utf-8"),
                "hierarchical proposal",
            )
            graph_audit_file = str(
                self.get_parameter("graph_audit_file").value
            ).strip()
            if not graph_audit_file:
                raise ValueError(
                    "hierarchical mission controller requires an active graph audit path"
                )
            self._graph_audit_file = Path(
                graph_audit_file
            ).expanduser()
            self._active_graph_evidence: Optional[dict[str, Any]] = None
            self._journal = HierarchicalBindingJournal(
                str(self.get_parameter("journal_path").value)
            )
            self._provider = CodexOAuthSemanticGoalProvider(
                model=str(self.get_parameter("planning_model").value),
                reasoning_effort=str(
                    self.get_parameter("planning_reasoning_effort").value
                ),
                timeout_s=float(
                    self.get_parameter("provider_timeout_s").value
                ),
            )
            self._controller: Optional[AsyncSemanticGoalController] = None
            self._initial_pool = ThreadPoolExecutor(
                max_workers=1, thread_name_prefix="hierarchical-initial-goal"
            )
            self._initial_future: Optional[Future[Mapping[str, Any]]] = None
            self._initial_snapshot: Optional[dict[str, Any]] = None
            self._initial_provider_started_at_s: Optional[float] = None
            self._authority: Optional[dict[str, Any]] = None
            self._authority_received_at_s: Optional[float] = None
            self._proposal: Optional[dict[str, Any]] = None
            self._grid: Optional[OccupancyGrid] = None
            self._map_received_at_s: Optional[float] = None
            self._map_source_timestamp_s: Optional[float] = None
            self._localization: Optional[dict[str, Any]] = None
            self._semantic_map: dict[str, Any] = {}
            self._camera_received_at_s: Optional[float] = None
            self._camera_source_timestamp_s: Optional[float] = None
            self._camera_valid = False
            self._camera_evidence: dict[str, Any] = {}
            self._origin: Optional[tuple[float, float]] = None
            self._collision_state = "BLOCKED"
            self._collision_scan_healthy = False
            self._collision_received_at_s: Optional[float] = None
            self._adapter_status: dict[str, Any] = {}
            self._event_generation = 0
            self._known_stable_tracks: set[str] = set()
            self._replan_pending = False
            self._consecutive_semantic_rejections = 0
            self._controller_session = 1
            self._last_dispatch_digest = ""
            self._last_dispatch_queue_key = ""
            self._last_resolved_batch_digest = ""
            self._last_nav2_path_content_digest = ""
            self._terminal = False
            # Map hashing and frontier extraction are intentionally
            # server-owned and can occupy the main callback group long enough
            # to delay authority receipt on a loaded Pi. Keep only
            # heartbeat receipt in an independent callback group; the command
            # bridge still enforces the bounded authority age independently.
            self._authority_callbacks = MutuallyExclusiveCallbackGroup()
            self._status_pub = self.create_publisher(
                String,
                CONTROLLER_STATUS_TOPIC,
                10,
            )
            self._dispatch_pub = self.create_publisher(
                String, GOAL_DISPATCH_TOPIC, 10
            )
            self.create_subscription(
                String,
                AUTHORITY_TOPIC,
                self._on_authority,
                10,
                callback_group=self._authority_callbacks,
            )
            self.create_subscription(
                RosOccupancyGrid,
                str(self.get_parameter("map_topic").value),
                self._on_map,
                qos_profile_sensor_data,
            )
            self.create_subscription(
                String,
                str(self.get_parameter("localization_topic").value),
                self._on_localization,
                10,
            )
            self.create_subscription(
                String,
                str(self.get_parameter("semantic_map_topic").value),
                self._on_semantic_map,
                10,
            )
            self.create_subscription(
                String,
                str(self.get_parameter("camera_topic").value),
                self._on_camera,
                10,
            )
            self.create_subscription(
                RosPath,
                str(self.get_parameter("nav2_path_topic").value),
                self._on_nav2_path,
                10,
            )
            self.create_subscription(
                String,
                str(self.get_parameter("collision_topic").value),
                self._on_collision,
                10,
            )
            self.create_subscription(
                String,
                str(self.get_parameter("adapter_status_topic").value),
                self._on_adapter_status,
                10,
            )
            self.create_timer(0.10, self._tick)
            self._publish_status("locked", "awaiting_fresh_authority")

        def _required_sha(self, parameter: str, environment: str) -> str:
            value = (
                str(self.get_parameter(parameter).value).strip()
                or os.environ.get(environment, "").strip()
            )
            return _sha1(value, parameter)

        def _on_authority(self, message: Any) -> None:
            try:
                authority = _json_object(
                    getattr(message, "data", ""), "authority"
                )
            except MissionValidationError:
                self._authority = None
                self._authority_received_at_s = None
                return
            self._authority = authority
            self._authority_received_at_s = time.time()

        def _on_map(self, message: Any) -> None:
            width = int(message.info.width)
            height = int(message.info.height)
            resolution = float(message.info.resolution)
            if width <= 0 or height <= 0 or resolution <= 0.0:
                self._grid = None
                self._map_received_at_s = None
                self._map_source_timestamp_s = None
                return
            cells = tuple(
                0 if int(value) == 0 else 100 if int(value) >= 50 else -1
                for value in message.data
            )
            identity = {
                "width": width,
                "height": height,
                "resolution_m": resolution,
                "origin_x_m": float(message.info.origin.position.x),
                "origin_y_m": float(message.info.origin.position.y),
                "cells": cells,
            }
            revision = hashlib.sha256(
                json.dumps(
                    identity, sort_keys=True, separators=(",", ":")
                ).encode("utf-8")
            ).hexdigest()
            self._grid = OccupancyGrid(
                width=width,
                height=height,
                resolution_m=resolution,
                origin_x_m=float(message.info.origin.position.x),
                origin_y_m=float(message.info.origin.position.y),
                frame_id=str(message.header.frame_id or "map"),
                map_id="live-slam-map",
                revision=revision,
                cells=cells,
                source="slam_toolbox:/map",
            )
            self._map_received_at_s = time.time()
            self._map_source_timestamp_s = _stamp_s(
                message.header.stamp
            )

        def _on_localization(self, message: Any) -> None:
            try:
                raw = _json_object(
                    getattr(message, "data", ""), "localization"
                )
                localization = raw.get("localization", raw)
                if not isinstance(localization, Mapping):
                    raise MissionValidationError(
                        "localization body must be an object"
                    )
                pose = localization.get("pose")
                if not isinstance(pose, Mapping):
                    raise MissionValidationError(
                        "localization pose is unavailable"
                    )
                self._localization = {
                    "x_m": float(pose["x_m"]),
                    "y_m": float(pose["y_m"]),
                    "yaw_rad": float(pose["yaw_rad"]),
                    "timestamp_s": float(
                        pose.get(
                            "stamp_s",
                            localization.get(
                                "stamp_s", raw.get("stamp_s", 0.0)
                            ),
                        )
                    ),
                }
            except (KeyError, TypeError, ValueError, MissionValidationError):
                self._localization = None

        def _on_semantic_map(self, message: Any) -> None:
            try:
                self._semantic_map = _json_object(
                    getattr(message, "data", ""), "semantic map"
                )
            except MissionValidationError:
                self._semantic_map = {}

        def _on_camera(self, message: Any) -> None:
            try:
                raw = _json_object(
                    getattr(message, "data", ""), "camera"
                )
                camera = raw.get("camera", raw)
                if (
                    not isinstance(camera, Mapping)
                    or camera.get("schema")
                    != "sphero_rvr.live_camera_perception.v1"
                    or camera.get("calibrated") is not True
                    or not str(camera.get("frame_id", "")).strip()
                ):
                    raise MissionValidationError(
                        "camera evidence is invalid"
                    )
                source_timestamp_s = float(camera["stamp_s"])
                if (
                    not math.isfinite(source_timestamp_s)
                    or source_timestamp_s <= 0.0
                ):
                    raise MissionValidationError(
                        "camera source timestamp is invalid"
                    )
            except (
                KeyError,
                TypeError,
                ValueError,
                MissionValidationError,
            ):
                self._camera_valid = False
                self._camera_evidence = {}
                self._camera_received_at_s = None
                self._camera_source_timestamp_s = None
                return
            self._camera_valid = True
            self._camera_evidence = bounded_camera_evidence(camera)
            self._camera_received_at_s = time.time()
            self._camera_source_timestamp_s = source_timestamp_s

        def _on_nav2_path(self, message: Any) -> None:
            if (
                self._authority is None
                or self._active_graph_evidence is None
                or not self._last_dispatch_digest
                or not self._last_resolved_batch_digest
                or self._terminal
            ):
                return
            try:
                evidence = nav2_path_evidence(
                    message,
                    source_sha=self._source_sha,
                    mission_id=str(
                        self._authority["mission_id"]
                    ),
                    dispatch_digest=self._last_dispatch_digest,
                    goal_batch_digest=self._last_resolved_batch_digest,
                )
            except MissionValidationError:
                return
            content_digest = str(evidence["path_content_digest"])
            if content_digest == self._last_nav2_path_content_digest:
                return
            self._last_nav2_path_content_digest = content_digest
            self._journal.append(
                str(self._authority["mission_id"]),
                "nav2_path",
                evidence,
                recorded_at_s=float(evidence["recorded_at_s"]),
            )

        def _on_collision(self, message: Any) -> None:
            raw = str(getattr(message, "data", "")).strip()
            (
                self._collision_state,
                self._collision_scan_healthy,
            ) = parse_collision_evidence(raw)
            self._collision_received_at_s = time.time()

        def _motion_evidence_fresh(self, now_s: float) -> bool:
            return live_motion_evidence_is_fresh(
                now_s=now_s,
                collision_received_at_s=self._collision_received_at_s,
                scan_healthy=self._collision_scan_healthy,
            )

        def _on_adapter_status(self, message: Any) -> None:
            try:
                self._adapter_status = _json_object(
                    getattr(message, "data", ""), "adapter status"
                )
            except MissionValidationError:
                self._adapter_status = {}

        def _authority_valid(self, now_s: float) -> tuple[bool, str]:
            if (
                self._authority is None
                or self._authority_received_at_s is None
            ):
                return False, "authority_missing"
            return validate_authority_heartbeat(
                self._authority,
                now_s=now_s,
                received_at_s=self._authority_received_at_s,
                source_sha=self._source_sha,
                deployed_sha=self._deployed_sha,
                reviewed_sha=self._reviewed_sha,
                max_age_s=AUTHORITY_HEARTBEAT_MAX_AGE_S,
            )

        def _active_graph_ready(self) -> tuple[bool, str]:
            if self._active_graph_evidence is not None:
                return True, "active_graph_verified"
            if not self._graph_audit_file.is_file():
                return False, "awaiting_active_graph_audit"
            try:
                raw = _json_object(
                    self._graph_audit_file.read_text(encoding="utf-8"),
                    "active graph audit",
                )
                self._active_graph_evidence = (
                    validate_active_graph_evidence(
                        raw, source_sha=self._source_sha
                    )
                )
            except (OSError, MissionValidationError):
                return False, "active_graph_audit_invalid"
            return True, "active_graph_verified"

        def _tracks(self, grid: OccupancyGrid) -> tuple[SemanticTrack, ...]:
            result = []
            for raw in self._semantic_map.get("tracks", ()):
                if not isinstance(raw, Mapping):
                    continue
                try:
                    track = SemanticTrack(
                        track_id=str(raw["track_id"]),
                        signature=live_semantic_track_signature(raw),
                        class_name=str(raw.get("label", raw.get("kind", "object"))),
                        x_m=float(raw["x_m"]),
                        y_m=float(raw["y_m"]),
                        position_method=str(
                            raw["position_method"]
                        ),
                        position_sigma_m=float(
                            raw["uncertainty_m"]
                        ),
                        last_seen_s=float(raw["last_seen_s"]),
                        evidence_ids=tuple(
                            dict.fromkeys(
                                str(item)
                                for item in (
                                    *raw.get("evidence_ids", ()),
                                    *raw.get(
                                        "localization_evidence_ids",
                                        (),
                                    ),
                                )
                            )
                        ),
                        stable_observations=int(
                            raw.get("observation_count", 1)
                        ),
                    )
                    grid.world_to_cell(track.x_m, track.y_m)
                    result.append(track)
                except (KeyError, TypeError, ValueError, IndexError, MissionValidationError):
                    continue
            return tuple(result[:16])

        def _snapshot(self, now_s: float) -> dict[str, Any]:
            if self._proposal is None or self._grid is None or self._localization is None:
                raise MissionValidationError(
                    "live map, localization, and proposal are required"
                )
            if not live_source_is_fresh(
                now_s=now_s,
                received_at_s=self._map_received_at_s,
                source_timestamp_s=self._map_source_timestamp_s,
                max_age_s=MAP_EVIDENCE_MAX_AGE_S,
            ):
                raise MissionValidationError(
                    "live map exceeds the fixed 3.000 s gate"
                )
            if not self._camera_valid or not live_source_is_fresh(
                now_s=now_s,
                received_at_s=self._camera_received_at_s,
                source_timestamp_s=self._camera_source_timestamp_s,
                max_age_s=CAMERA_EVIDENCE_MAX_AGE_S,
            ):
                raise MissionValidationError(
                    "live camera exceeds the fixed 3.000 s gate"
                )
            age = now_s - float(self._localization["timestamp_s"])
            if age < 0.0 or age > LOCALIZATION_MAX_AGE_S:
                raise MissionValidationError(
                    "live localization exceeds the fixed 0.500 s gate"
                )
            frontiers = detect_frontiers(
                self._grid,
                robot_x_m=float(self._localization["x_m"]),
                robot_y_m=float(self._localization["y_m"]),
            )
            tracks = self._tracks(self._grid)
            if self._origin is None:
                self._origin = (
                    float(self._localization["x_m"]),
                    float(self._localization["y_m"]),
                )
            next_best_views = []
            for track in tracks:
                try:
                    next_best_views.append(
                        generate_next_best_views(
                            self._grid,
                            track,
                            robot_x_m=float(self._localization["x_m"]),
                            robot_y_m=float(self._localization["y_m"]),
                        )
                    )
                except MissionValidationError:
                    continue
            motion_evidence_fresh = self._motion_evidence_fresh(now_s)
            return build_semantic_world_snapshot(
                mission_id=str(self._proposal["mission_id"]),
                objective=str(self._proposal["objective"]),
                objective_revision=int(
                    self._proposal["objective_revision"]
                ),
                decision_generation=1,
                event_generation=self._event_generation,
                requested_object_classes=tuple(
                    self._proposal["requested_object_classes"]
                ),
                map_id=self._grid.map_id,
                map_revision=self._grid.revision,
                robot_x_m=float(self._localization["x_m"]),
                robot_y_m=float(self._localization["y_m"]),
                robot_yaw_rad=float(self._localization["yaw_rad"]),
                localization_timestamp_s=float(
                    self._localization["timestamp_s"]
                ),
                now_s=now_s,
                frontiers=frontiers,
                tracks=tracks,
                next_best_views=tuple(next_best_views),
                origin_x_m=self._origin[0],
                origin_y_m=self._origin[1],
                coverage_fraction=(
                    sum(
                        int(value) >= 0
                        for value in self._grid.cells
                    )
                    / max(1, len(self._grid.cells))
                ),
                collision_state=(
                    self._collision_state
                    if motion_evidence_fresh
                    else "BLOCKED"
                ),
                mission_lease_valid=True,
                motion_evidence_fresh=motion_evidence_fresh,
            )

        def _tick(self) -> None:
            if self._terminal:
                return
            now_s = time.time()
            authority_valid, reason = self._authority_valid(now_s)
            if not authority_valid:
                if (
                    transient_authority_hold(reason)
                    and (
                        self._controller is not None
                        or self._initial_future is not None
                    )
                ):
                    self._publish_status(
                        planning_hold_controller_state(
                            self._adapter_status
                        ),
                        "authority_heartbeat_hold",
                    )
                elif (
                    self._controller is not None
                    or self._initial_future is not None
                ):
                    self._terminal = True
                    self._provider.cancel()
                    self._publish_status("recovery_required", reason)
                else:
                    self._publish_status("locked", reason)
                return
            assert self._authority is not None
            graph_ready, graph_reason = self._active_graph_ready()
            if not graph_ready:
                if graph_reason == "active_graph_audit_invalid":
                    self._terminal = True
                    self._publish_status(
                        "recovery_required", graph_reason
                    )
                else:
                    self._publish_status("locked", graph_reason)
                return
            if self._proposal is None:
                try:
                    self._proposal = validate_physical_proposal(
                        self._raw_proposal,
                        authority=self._authority,
                        source_sha=self._source_sha,
                    )
                except MissionValidationError:
                    self._terminal = True
                    self._publish_status(
                        "recovery_required", "proposal_binding_invalid"
                    )
                    return
            try:
                snapshot = self._snapshot(now_s)
            except MissionValidationError as exc:
                hold_state = planning_hold_controller_state(
                    self._adapter_status
                )
                self._publish_status(
                    hold_state,
                    (
                        f"planning_evidence_hold: {exc}"
                        if hold_state == "navigating"
                        else str(exc)
                    ),
                )
                return
            motion_evidence_fresh = bool(
                snapshot.get("safety", {}).get(
                    "motion_evidence_fresh", False
                )
            )
            if self._controller is None:
                if not motion_evidence_fresh:
                    self._publish_status(
                        "wait_planning", "motion_evidence_stale"
                    )
                    return
                self._known_stable_tracks.update(
                    self._stable_track_ids()
                )
                self._tick_initial(snapshot, now_s)
                return
            recovery_reason = adapter_recovery_reason(
                self._adapter_status
            )
            if recovery_reason:
                self._terminal = True
                self._provider.cancel()
                self._publish_status(
                    "recovery_required", recovery_reason
                )
                return
            active_goals = self._controller.resolved_motion_goals()
            fallback_remaining = (
                0.0
                if not active_goals
                else active_goals[0][0].route_length_m
            )
            remaining = adapter_remaining_distance(
                self._adapter_status, fallback_remaining
            )
            if not motion_evidence_fresh:
                if (
                    planning_hold_controller_state(
                        self._adapter_status
                    )
                    == "navigating"
                ):
                    # The independent collision supervisor already forces
                    # motor zero while scan evidence is unhealthy. Keep the
                    # accepted Nav2 action alive so it can continue when the
                    # supervisor reports fresh evidence again.
                    self._publish_status(
                        "navigating", "motion_evidence_hold"
                    )
                    return
                step = self._controller.tick(
                    snapshot,
                    now_s=now_s,
                    remaining_distance_m=remaining,
                    eta_s=remaining / 0.10,
                    collision_state="BLOCKED",
                    motion_evidence_fresh=False,
                )
                self._record_controller_events(
                    step.events, now_s=now_s
                )
                self._publish_status(
                    "wait_planning", "motion_evidence_stale"
                )
                return
            event_step = self._event_replan(snapshot, now_s)
            if event_step is not None:
                step, _event_snapshot = event_step
                self._record_controller_events(
                    step.events, now_s=now_s
                )
                for event in step.events:
                    if str(event.get("kind", "")) == "prefetch_started":
                        provider_snapshot = (
                            self._controller.provider_snapshot_in_flight()
                        )
                        if (
                            provider_snapshot is None
                            or provider_snapshot.get("snapshot_id")
                            != event.get("snapshot_id")
                        ):
                            self._terminal = True
                            self._provider.cancel()
                            self._publish_status(
                                "recovery_required",
                                "provider_snapshot_evidence_unavailable",
                            )
                            return
                        self._record_world_snapshot(
                            provider_snapshot,
                            now_s=now_s,
                            reason="event_replan_provider_call",
                            provider_snapshot_id=str(
                                event.get("snapshot_id", "")
                            ),
                        )
                cancel_active_goal = (
                    step.handoff.state == "wait_planning"
                )
                self._publish_status(
                    (
                        "wait_planning"
                        if cancel_active_goal
                        else planning_hold_controller_state(
                            self._adapter_status
                        )
                    ),
                    "event_replan_provider_in_flight",
                    cancel_active_goal=cancel_active_goal,
                )
                return
            step = self._controller.tick(
                snapshot,
                now_s=now_s,
                remaining_distance_m=remaining,
                eta_s=remaining / 0.10,
                collision_state=self._collision_state,
                motion_evidence_fresh=motion_evidence_fresh,
            )
            self._record_controller_events(step.events, now_s=now_s)
            for event in step.events:
                if str(event.get("kind", "")) == "prefetch_started":
                    provider_snapshot = (
                        self._controller.provider_snapshot_in_flight()
                    )
                    if (
                        provider_snapshot is None
                        or provider_snapshot.get("snapshot_id")
                        != event.get("snapshot_id")
                    ):
                        self._terminal = True
                        self._provider.cancel()
                        self._publish_status(
                            "recovery_required",
                            "provider_snapshot_evidence_unavailable",
                        )
                        return
                    self._record_world_snapshot(
                        provider_snapshot,
                        now_s=now_s,
                        reason="prefetch_provider_call",
                        provider_snapshot_id=str(
                            event.get("snapshot_id", "")
                        ),
                    )
            self._consecutive_semantic_rejections = (
                updated_semantic_rejection_count(
                    self._consecutive_semantic_rejections,
                    step.events,
                )
            )
            if (
                self._consecutive_semantic_rejections
                >= MAX_CONSECUTIVE_SEMANTIC_REJECTIONS
            ):
                self._terminal = True
                self._provider.cancel()
                self._publish_status(
                    "recovery_required",
                    "semantic_revalidation_exhausted",
                )
                return
            if any(
                str(event.get("kind", ""))
                in {"prefetch_revalidated", "prefetch_redundant"}
                for event in step.events
            ):
                self._replan_pending = False
            if self._replan_pending and step.provider_in_flight:
                self._publish_status(
                    planning_hold_controller_state(
                        self._adapter_status
                    ),
                    "event_replan_provider_in_flight",
                )
                return
            ready_non_motion = self._controller.ready_non_motion_goal()
            if ready_non_motion is not None:
                state, reason, terminal = semantic_non_motion_status(
                    ready_non_motion.decision.action
                )
                if terminal:
                    self._terminal = True
                self._publish_status(
                    state,
                    reason,
                    cancel_active_goal=(state == "wait_planning"),
                )
                return
            self._publish_dispatch(snapshot, now_s, step.handoff.state)

        def _stable_track_ids(self) -> set[str]:
            return {
                str(item.get("track_id", "")).strip()
                for item in self._semantic_map.get("tracks", ())
                if isinstance(item, Mapping)
                and int(item.get("observation_count", 0)) >= 2
                and str(item.get("track_id", "")).strip()
            }

        def _event_replan(
            self, snapshot: Mapping[str, Any], now_s: float
        ) -> Optional[tuple[Any, Mapping[str, Any]]]:
            assert self._controller is not None
            stable_tracks = self._stable_track_ids()
            new_tracks = sorted(stable_tracks - self._known_stable_tracks)
            self._known_stable_tracks.update(stable_tracks)
            if self._replan_pending:
                return None
            kind: Optional[SemanticEventKind] = None
            target_id = ""
            if new_tracks:
                kind = SemanticEventKind.NEW_DETECTION
                target_id = new_tracks[0]
            else:
                active = self._controller.resolved_motion_goals()
                if active:
                    active_goal, captured = active[0]
                    invalidation = semantic_target_invalidation_reason(
                        active_goal, captured, snapshot
                    )
                    if rolling_frontier_invalidation_preserves_route(
                        invalidation, self._adapter_status
                    ):
                        # Frontier signatures describe the rolling exploration
                        # boundary, not a physical hazard.  Once Nav2 accepted
                        # server-owned geometry, let Nav2 and the collision
                        # supervisor continue to own path and obstacle safety.
                        # A changed frontier will naturally be discarded on
                        # completion/abort or during successor revalidation.
                        return None
                    if invalidation:
                        kind = SemanticEventKind.INVALID_TARGET
                        target_id = active_goal.target_id
            if kind is None:
                return None
            self._event_generation += 1
            self._replan_pending = True
            # The public builder owns snapshot hashing; rebuild to keep the
            # provider binding exact after an event-generation transition.
            refreshed = self._snapshot(now_s)
            event = SemanticReplanEvent(
                event_id=(
                    f"{kind.value}-{self._event_generation}-{target_id}"
                ),
                kind=kind,
                observed_at_s=now_s,
                target_id=target_id,
                confidence=1.0,
                stable_observations=2,
            )
            return (
                self._controller.handle_event(
                    event, refreshed, now_s=now_s
                ),
                refreshed,
            )

        def _tick_initial(
            self, snapshot: Mapping[str, Any], now_s: float
        ) -> None:
            if self._initial_future is None:
                captured = json.loads(json.dumps(dict(snapshot)))
                self._initial_snapshot = captured
                self._record_world_snapshot(
                    captured,
                    now_s=now_s,
                    reason="initial_provider_call",
                    provider_snapshot_id=str(
                        captured.get("snapshot_id", "")
                    ),
                )
                prompt = semantic_goal_prompt(
                    str(captured["objective"]), captured
                )
                self._initial_future = self._initial_pool.submit(
                    self._provider.choose, prompt, captured
                )
                self._initial_provider_started_at_s = now_s
                self._publish_status(
                    "wait_planning", "initial_provider_in_flight"
                )
                return
            if not self._initial_future.done():
                self._publish_status(
                    "wait_planning", "initial_provider_in_flight"
                )
                return
            try:
                raw = self._initial_future.result()
                provider_elapsed_s = (
                    None
                    if self._initial_provider_started_at_s is None
                    else max(
                        0.0,
                        now_s - self._initial_provider_started_at_s,
                    )
                )
                captured = self._initial_snapshot
                assert captured is not None
                self._journal.append(
                    str(self._authority["mission_id"]),
                    "provider_call_completed",
                    {
                        "decision_generation": 1,
                        "snapshot_id": str(
                            captured.get("snapshot_id", "")
                        ),
                        "provider_elapsed_s": provider_elapsed_s,
                        "provider_timeout_s": float(
                            self.get_parameter(
                                "provider_timeout_s"
                            ).value
                        ),
                        "real_provider": True,
                    },
                    recorded_at_s=now_s,
                )
                controller = AsyncSemanticGoalController(
                    self._provider,
                    provider_p95_s=float(
                        self.get_parameter("provider_p95_s").value
                    ),
                    prefetch_margin_s=float(
                        self.get_parameter("prefetch_margin_s").value
                    ),
                )
                controller.start(raw, captured, now_s=now_s)
                self._controller = controller
                self._initial_future = None
                self._initial_snapshot = None
                self._initial_provider_started_at_s = None
                self._publish_dispatch(snapshot, now_s, "initial_goal")
            except Exception as exc:
                self._terminal = True
                self._publish_status(
                    "recovery_required",
                    f"initial_goal_rejected:{exc.__class__.__name__}",
                )

        def _publish_dispatch(
            self,
            snapshot: Mapping[str, Any],
            now_s: float,
            reason: str,
        ) -> None:
            assert self._controller is not None
            assert self._authority is not None
            try:
                dispatch = build_goal_dispatch(
                    self._controller,
                    authority=self._authority,
                    current_snapshot=snapshot,
                    controller_session=self._controller_session,
                    reason=reason,
                )
            except MissionValidationError as exc:
                self._publish_status("wait_planning", str(exc))
                return
            resolved_batch = resolve_goal_dispatch(
                dispatch,
                authority=self._authority,
                now_s=now_s,
            ).to_json_dict()
            digest = str(dispatch["dispatch_digest"])
            queue_key = goal_dispatch_queue_key(dispatch)
            if queue_key == self._last_dispatch_queue_key:
                return
            message = String()
            message.data = json.dumps(
                dispatch,
                sort_keys=True,
                separators=(",", ":"),
                allow_nan=False,
            )
            self._last_dispatch_digest = digest
            self._last_dispatch_queue_key = queue_key
            self._last_resolved_batch_digest = str(
                resolved_batch["batch_digest"]
            )
            self._journal.append(
                str(self._authority["mission_id"]),
                "goal_dispatch",
                dispatch,
                recorded_at_s=now_s,
            )
            self._journal.append(
                str(self._authority["mission_id"]),
                "resolved_goal_batch",
                resolved_batch,
                recorded_at_s=now_s,
            )
            self._dispatch_pub.publish(message)
            self._publish_status("dispatching", reason)

        def _record_world_snapshot(
            self,
            snapshot: Mapping[str, Any],
            *,
            now_s: float,
            reason: str,
            provider_snapshot_id: str,
        ) -> None:
            assert self._authority is not None
            payload = {
                "schema": (
                    "sphero_rvr.hierarchical_world_evidence.v1"
                ),
                "source_sha": self._source_sha,
                "mission_id": str(
                    self._authority["mission_id"]
                ),
                "recorded_at_s": float(now_s),
                "reason": str(reason),
                "provider_snapshot_id": str(provider_snapshot_id),
                "snapshot": json.loads(
                    json.dumps(
                        dict(snapshot),
                        sort_keys=True,
                        separators=(",", ":"),
                        allow_nan=False,
                    )
                ),
                "camera_evidence": json.loads(
                    json.dumps(
                        self._camera_evidence,
                        sort_keys=True,
                        separators=(",", ":"),
                        allow_nan=False,
                    )
                ),
            }
            self._journal.append(
                str(self._authority["mission_id"]),
                "world_snapshot",
                payload,
                recorded_at_s=now_s,
            )

        def _record_controller_events(
            self, events: Any, *, now_s: float
        ) -> None:
            assert self._authority is not None
            for event in events:
                payload = dict(event)
                if payload.get("provider_elapsed_s") is not None:
                    payload["real_provider"] = True
                self._journal.append(
                    str(self._authority["mission_id"]),
                    "controller_event",
                    payload,
                    recorded_at_s=now_s,
                )

        def _publish_status(
            self,
            state: str,
            reason: str,
            *,
            cancel_active_goal: bool = False,
        ) -> None:
            payload = {
                "schema": "sphero_rvr.hierarchical_controller_status.v1",
                "state": str(state),
                "reason": str(reason),
                "cancel_active_goal": bool(cancel_active_goal),
                "source_sha": self._source_sha,
                "mission_id": (
                    ""
                    if self._authority is None
                    else str(self._authority.get("mission_id", ""))
                ),
                "last_dispatch_digest": self._last_dispatch_digest,
                "provider_in_flight": (
                    self._initial_future is not None
                    and not self._initial_future.done()
                ),
                "provider_timeout_s": float(
                    self.get_parameter("provider_timeout_s").value
                ),
                "provider_latency_p95_s": float(
                    self.get_parameter("provider_p95_s").value
                ),
                "prefetch_margin_s": float(
                    self.get_parameter("prefetch_margin_s").value
                ),
                "provider_latency_history": list(
                    self._provider.latency_history()[-8:]
                ),
                "direct_twist_publisher": False,
                "restart_resume_allowed": False,
            }
            message = String()
            message.data = json.dumps(payload, sort_keys=True)
            self._status_pub.publish(message)

        def close(self) -> None:
            self._terminal = True
            try:
                if self._controller is not None:
                    self._controller.close()
                else:
                    self._provider.close()
            finally:
                self._initial_pool.shutdown(wait=True, cancel_futures=True)
                self._journal.close()

    rclpy.init(args=args)
    node = HierarchicalMissionNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
