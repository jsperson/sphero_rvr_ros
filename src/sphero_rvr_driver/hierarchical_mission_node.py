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
    HierarchicalBindingJournal,
    build_goal_dispatch,
    validate_authority_heartbeat,
    validate_physical_proposal,
)
from .mission_api import MissionValidationError


MAX_CONSECUTIVE_SEMANTIC_REJECTIONS = 3
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
        and reason == "nav2_result_status_4"
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
                "journal_path": (
                    "~/.local/state/sphero_rvr/"
                    "hierarchical-physical-evidence.sqlite3"
                ),
                "map_topic": "/map",
                "localization_topic": "/mission_api/v2/localization/status",
                "semantic_map_topic": "/mission_api/v2/map/status",
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
            self._localization: Optional[dict[str, Any]] = None
            self._semantic_map: dict[str, Any] = {}
            self._origin: Optional[tuple[float, float]] = None
            self._collision_state = "BLOCKED"
            self._adapter_status: dict[str, Any] = {}
            self._event_generation = 0
            self._known_stable_tracks: set[str] = set()
            self._replan_pending = False
            self._consecutive_semantic_rejections = 0
            self._controller_session = 1
            self._last_dispatch_digest = ""
            self._last_dispatch_queue_key = ""
            self._terminal = False
            # Map hashing and frontier extraction are intentionally
            # server-owned and can occupy the main callback group long enough
            # to exceed the 0.300 s authority lease on a loaded Pi. Keep only
            # heartbeat receipt in an independent callback group; the command
            # bridge still enforces the same unrelaxed 0.300 s lease.
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

        def _on_collision(self, message: Any) -> None:
            raw = str(getattr(message, "data", "")).strip()
            try:
                payload = _json_object(raw, "collision")
                state = str(payload.get("state", "BLOCKED")).upper()
            except MissionValidationError:
                state = raw.split(maxsplit=1)[0].upper() if raw else "BLOCKED"
            self._collision_state = (
                state if state in {"CLEAR", "SLOW"} else "BLOCKED"
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
                        position_method="floor_projection",
                        position_sigma_m=float(raw.get("uncertainty_m", 0.05)),
                        last_seen_s=float(raw["last_seen_s"]),
                        evidence_ids=tuple(
                            str(item) for item in raw.get("evidence_ids", ())
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
            age = now_s - float(self._localization["timestamp_s"])
            if age < 0.0 or age > 0.300:
                raise MissionValidationError(
                    "live localization exceeds the fixed 0.300 s gate"
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
                collision_state=self._collision_state,
                mission_lease_valid=True,
                motion_evidence_fresh=True,
            )

        def _tick(self) -> None:
            if self._terminal:
                return
            now_s = time.time()
            authority_valid, reason = self._authority_valid(now_s)
            if not authority_valid:
                if self._controller is not None or self._initial_future is not None:
                    self._terminal = True
                    self._provider.cancel()
                    self._publish_status("recovery_required", reason)
                else:
                    self._publish_status("locked", reason)
                return
            assert self._authority is not None
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
                self._publish_status("wait_planning", str(exc))
                return
            if self._controller is None:
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
            event_step = self._event_replan(snapshot, now_s)
            if event_step is not None:
                step, event_snapshot = event_step
                for event in step.events:
                    self._journal.append(
                        str(self._authority["mission_id"]),
                        "controller_event",
                        dict(event),
                        recorded_at_s=now_s,
                    )
                del event_snapshot
                self._publish_status(
                    "wait_planning", "event_triggered_replan"
                )
                return
            active_goals = self._controller.resolved_motion_goals()
            fallback_remaining = (
                0.0 if not active_goals else active_goals[0][0].route_length_m
            )
            remaining = adapter_remaining_distance(
                self._adapter_status, fallback_remaining
            )
            step = self._controller.tick(
                snapshot,
                now_s=now_s,
                remaining_distance_m=remaining,
                eta_s=remaining / 0.10,
                collision_state=self._collision_state,
                motion_evidence_fresh=True,
            )
            for event in step.events:
                self._journal.append(
                    str(self._authority["mission_id"]),
                    "controller_event",
                    dict(event),
                    recorded_at_s=now_s,
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
                str(event.get("kind", "")) == "prefetch_revalidated"
                for event in step.events
            ):
                self._replan_pending = False
            if self._replan_pending and step.provider_in_flight:
                self._publish_status(
                    "wait_planning", "event_replan_provider_in_flight"
                )
                return
            if self._controller.ready_non_motion_goal() is not None:
                self._terminal = True
                self._publish_status(
                    "complete",
                    self._controller.ready_non_motion_goal().decision.action,
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
                self._journal.append(
                    str(self._authority["mission_id"]),
                    "provider_call_completed",
                    {
                        "decision_generation": 1,
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
                captured = self._initial_snapshot
                assert captured is not None
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
            self._dispatch_pub.publish(message)
            self._last_dispatch_digest = digest
            self._last_dispatch_queue_key = queue_key
            self._journal.append(
                str(self._authority["mission_id"]),
                "goal_dispatch",
                dispatch,
                recorded_at_s=now_s,
            )
            self._publish_status("dispatching", reason)

        def _publish_status(self, state: str, reason: str) -> None:
            payload = {
                "schema": "sphero_rvr.hierarchical_controller_status.v1",
                "state": str(state),
                "reason": str(reason),
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
