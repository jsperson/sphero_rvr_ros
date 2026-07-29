"""Replay-only semantic-goal selection for hierarchical exploration.

Phase 3 promotes the model from motor-shaped intents to stable semantic IDs.
The model never supplies geometry or motion parameters.  Deterministic code
resolves and revalidates its choice, while the Phase 1 follower remains the
only modeled Nav2 handoff seam.  This module is deliberately ROS-free and
reports no physical or motion authority.
"""

from __future__ import annotations

from collections import deque
from concurrent.futures import Future, ThreadPoolExecutor
from dataclasses import asdict, dataclass, field
from enum import Enum
import hashlib
import json
import math
import os
from pathlib import Path
import threading
import tempfile
import time
from typing import Any, Callable, Mapping, Optional, Protocol, Sequence

from .codex_app_server import CodexAppServerClient
from .hierarchical_exploration import (
    ContinuousGoalFollowerReplay,
    FrontierCandidate,
    FrontierGoal,
    HandoffStep,
    OccupancyGrid,
)
from .mission_api import MissionValidationError
from .prompt_drive import ALLOWED_REASONING_EFFORTS, DEFAULT_CODEX_MODEL_ID


SEMANTIC_GOAL_SCHEMA = "sphero_rvr.semantic_goal.v1"
SEMANTIC_SNAPSHOT_SCHEMA = "sphero_rvr.semantic_world_snapshot.v1"
PHASE3_REPLAY_SCHEMA = "sphero_rvr.phase3_replay_evidence.v1"
ALLOWED_ACTIONS = {
    "go_to_frontier",
    "inspect",
    "search_region",
    "return_to_start",
    "wait",
    "finish",
}
MOTION_ACTIONS = {
    "go_to_frontier",
    "inspect",
    "search_region",
    "return_to_start",
}
FORBIDDEN_MODEL_KEYS = {
    "x",
    "y",
    "x_m",
    "y_m",
    "yaw",
    "yaw_rad",
    "pose",
    "route",
    "path",
    "speed",
    "velocity",
    "acceleration",
    "clearance",
    "lease",
    "timeout",
    "ros",
    "code",
}
MAX_TRACK_REVALIDATION_DRIFT_M = 0.10
MAX_VIEWPOINT_REVALIDATION_DRIFT_M = 0.10


def _finite(value: Any, name: str) -> float:
    try:
        parsed = float(value)
    except (TypeError, ValueError) as exc:
        raise MissionValidationError(f"{name} must be finite") from exc
    if not math.isfinite(parsed):
        raise MissionValidationError(f"{name} must be finite")
    return parsed


def _digest(value: Mapping[str, Any]) -> str:
    rendered = json.dumps(
        value, sort_keys=True, separators=(",", ":"), allow_nan=False
    ).encode("utf-8")
    return hashlib.sha256(rendered).hexdigest()


def _angle_delta(left: float, right: float) -> float:
    return math.atan2(math.sin(left - right), math.cos(left - right))


@dataclass(frozen=True)
class SemanticTrack:
    track_id: str
    signature: str
    class_name: str
    x_m: float
    y_m: float
    position_method: str
    position_sigma_m: float
    last_seen_s: float
    evidence_ids: tuple[str, ...]
    last_view_bearing_rad: float = 0.0
    stable_observations: int = 2

    def __post_init__(self) -> None:
        if not self.track_id or not self.signature or not self.class_name:
            raise MissionValidationError("semantic track identity is required")
        if self.position_method not in {
            "lidar_range",
            "floor_projection",
            "bearing_only",
        }:
            raise MissionValidationError("semantic track position method is invalid")
        for name in (
            "x_m",
            "y_m",
            "position_sigma_m",
            "last_seen_s",
            "last_view_bearing_rad",
        ):
            _finite(getattr(self, name), f"semantic track {name}")
        if self.position_method == "bearing_only":
            raise MissionValidationError(
                "bearing-only tracks cannot generate point viewpoints"
            )
        if self.position_sigma_m < 0.0 or self.stable_observations < 1:
            raise MissionValidationError("semantic track uncertainty/stability is invalid")
        if not self.evidence_ids:
            raise MissionValidationError("semantic track evidence IDs are required")

    def to_json_dict(self) -> dict[str, Any]:
        return {
            "track_id": self.track_id,
            "signature": self.signature,
            "class_name": self.class_name,
            "position": {"frame_id": "map", "x_m": self.x_m, "y_m": self.y_m},
            "position_method": self.position_method,
            "position_sigma_m": self.position_sigma_m,
            "last_seen_s": self.last_seen_s,
            "evidence_ids": list(self.evidence_ids),
            "stable_observations": self.stable_observations,
        }


@dataclass(frozen=True)
class ViewpointCandidate:
    viewpoint_id: str
    track_id: str
    track_signature: str
    map_id: str
    map_revision: str
    x_m: float
    y_m: float
    yaw_rad: float
    route_length_m: float
    clearance_m: float
    expected_uncertainty_reduction: float
    view_diversity_rad: float
    score: float

    def to_json_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class RejectedViewpoint:
    sample_index: int
    x_m: float
    y_m: float
    reason: str


@dataclass(frozen=True)
class NextBestViewPlan:
    track_id: str
    candidates: tuple[ViewpointCandidate, ...]
    rejected: tuple[RejectedViewpoint, ...]

    def to_json_dict(self) -> dict[str, Any]:
        return {
            "track_id": self.track_id,
            "candidates": [item.to_json_dict() for item in self.candidates],
            "rejected": [asdict(item) for item in self.rejected],
        }


def _neighbors4(
    grid: OccupancyGrid, cell: tuple[int, int]
) -> tuple[tuple[int, int], ...]:
    return tuple(
        (x, y)
        for x, y in (
            (cell[0] - 1, cell[1]),
            (cell[0] + 1, cell[1]),
            (cell[0], cell[1] - 1),
            (cell[0], cell[1] + 1),
        )
        if 0 <= x < grid.width and 0 <= y < grid.height
    )


def _reachable_cells(
    grid: OccupancyGrid, start: tuple[int, int]
) -> dict[tuple[int, int], int]:
    if grid.value(start) != 0:
        raise MissionValidationError("Next-Best-View robot pose is not in free space")
    distances = {start: 0}
    queue = deque([start])
    while queue:
        cell = queue.popleft()
        for neighbor in _neighbors4(grid, cell):
            if neighbor in distances or grid.value(neighbor) != 0:
                continue
            distances[neighbor] = distances[cell] + 1
            queue.append(neighbor)
    return distances


def _cell_clearance(
    grid: OccupancyGrid, cell: tuple[int, int]
) -> float:
    blocked = (
        (x, y)
        for y in range(grid.height)
        for x in range(grid.width)
        if grid.value((x, y)) != 0
    )
    distances = [
        math.hypot(cell[0] - x, cell[1] - y) * grid.resolution_m
        for x, y in blocked
    ]
    return min(distances) if distances else math.inf


def _line_cells(
    start: tuple[int, int], end: tuple[int, int]
) -> tuple[tuple[int, int], ...]:
    x0, y0 = start
    x1, y1 = end
    dx = abs(x1 - x0)
    dy = -abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    error = dx + dy
    cells: list[tuple[int, int]] = []
    while True:
        cells.append((x0, y0))
        if (x0, y0) == (x1, y1):
            break
        doubled = 2 * error
        if doubled >= dy:
            error += dy
            x0 += sx
        if doubled <= dx:
            error += dx
            y0 += sy
    return tuple(cells)


def generate_next_best_views(
    grid: OccupancyGrid,
    track: SemanticTrack,
    *,
    robot_x_m: float,
    robot_y_m: float,
    minimum_clearance_m: float = 0.15,
    sample_count: int = 12,
    maximum_candidates: int = 6,
) -> NextBestViewPlan:
    """Generate finite, stable, reachable camera viewpoints around one track."""

    if sample_count < 4 or maximum_candidates < 1:
        raise ValueError("Next-Best-View sampling bounds are invalid")
    robot_cell = grid.world_to_cell(robot_x_m, robot_y_m)
    reachable = _reachable_cells(grid, robot_cell)
    try:
        track_cell = grid.world_to_cell(track.x_m, track.y_m)
    except IndexError as exc:
        raise MissionValidationError("semantic track lies outside the map") from exc
    radius = min(0.90, max(0.45, 0.35 + 2.0 * track.position_sigma_m))
    accepted: list[ViewpointCandidate] = []
    rejected: list[RejectedViewpoint] = []
    for index in range(sample_count):
        angle = 2.0 * math.pi * index / sample_count
        x_m = track.x_m + radius * math.cos(angle)
        y_m = track.y_m + radius * math.sin(angle)
        try:
            cell = grid.world_to_cell(x_m, y_m)
        except IndexError:
            rejected.append(RejectedViewpoint(index, x_m, y_m, "out_of_map"))
            continue
        if grid.value(cell) != 0:
            rejected.append(
                RejectedViewpoint(index, x_m, y_m, "occupied_or_unknown")
            )
            continue
        if cell not in reachable:
            rejected.append(RejectedViewpoint(index, x_m, y_m, "unreachable"))
            continue
        clearance = _cell_clearance(grid, cell)
        if clearance < minimum_clearance_m:
            rejected.append(
                RejectedViewpoint(index, x_m, y_m, "insufficient_clearance")
            )
            continue
        line = _line_cells(cell, track_cell)
        if any(grid.value(item) != 0 for item in line):
            rejected.append(RejectedViewpoint(index, x_m, y_m, "occluded"))
            continue
        yaw = math.atan2(track.y_m - y_m, track.x_m - x_m)
        observation_bearing = math.atan2(y_m - track.y_m, x_m - track.x_m)
        diversity = abs(
            _angle_delta(observation_bearing, track.last_view_bearing_rad)
        )
        route_length = reachable[cell] * grid.resolution_m
        uncertainty_reduction = min(
            1.0, 0.25 + track.position_sigma_m / max(radius, 1e-9)
        )
        score = (
            2.0 * uncertainty_reduction
            + 0.35 * diversity
            + 0.20 * min(clearance, 1.0)
            - 0.20 * route_length
        )
        identity = {
            "track_signature": track.signature,
            "map_id": grid.map_id,
            "cell": cell,
        }
        accepted.append(
            ViewpointCandidate(
                viewpoint_id=f"nbv-{_digest(identity)[:16]}",
                track_id=track.track_id,
                track_signature=track.signature,
                map_id=grid.map_id,
                map_revision=grid.revision,
                x_m=x_m,
                y_m=y_m,
                yaw_rad=yaw,
                route_length_m=route_length,
                clearance_m=clearance,
                expected_uncertainty_reduction=uncertainty_reduction,
                view_diversity_rad=diversity,
                score=score,
            )
        )
    ordered = tuple(
        sorted(
            accepted,
            key=lambda item: (-item.score, item.route_length_m, item.viewpoint_id),
        )[:maximum_candidates]
    )
    return NextBestViewPlan(
        track_id=track.track_id,
        candidates=ordered,
        rejected=tuple(rejected),
    )


def build_semantic_world_snapshot(
    *,
    mission_id: str,
    objective: str,
    objective_revision: int,
    decision_generation: int = 1,
    event_generation: int,
    requested_object_classes: Sequence[str] = (),
    map_id: str,
    map_revision: str,
    robot_x_m: float,
    robot_y_m: float,
    robot_yaw_rad: float,
    localization_timestamp_s: float,
    now_s: float,
    frontiers: Sequence[FrontierCandidate],
    tracks: Sequence[SemanticTrack],
    next_best_views: Sequence[NextBestViewPlan],
    origin_x_m: float,
    origin_y_m: float,
    active_goal: Optional[Mapping[str, Any]] = None,
    coverage_fraction: float = 0.0,
    remaining_route_budget_m: float = 20.0,
    mission_lease_valid: bool = True,
    motion_evidence_fresh: bool = True,
    stop: bool = False,
    estop: bool = False,
    collision_state: str = "CLEAR",
    cancelled: bool = False,
) -> dict[str, Any]:
    """Build the bounded projection supplied to a semantic-goal provider."""

    if not mission_id or not objective:
        raise MissionValidationError("semantic snapshot mission and objective are required")
    if len(tracks) > 16:
        raise MissionValidationError(
            "semantic snapshot track candidate list exceeds bounds"
        )
    # WFD operates on the complete server-owned map and may legitimately find
    # more regions than the compact model contract admits. It already returns
    # candidates in deterministic path-distance/information-gain/signature
    # order, so retain the first bounded window before constructing the
    # provider-visible snapshot.
    bounded_frontiers = tuple(frontiers)[:16]
    if decision_generation < 1 or event_generation < 0:
        raise MissionValidationError("semantic snapshot generations are invalid")
    if any(not isinstance(item, str) for item in requested_object_classes):
        raise MissionValidationError(
            "semantic snapshot requested object classes are invalid"
        )
    requested_classes = tuple(item.strip() for item in requested_object_classes)
    if (
        len(requested_classes) > 8
        or any(not item for item in requested_classes)
        or len(set(requested_classes)) != len(requested_classes)
    ):
        raise MissionValidationError(
            "semantic snapshot requested object classes are invalid"
        )
    localization_age_s = _finite(now_s, "snapshot now") - _finite(
        localization_timestamp_s, "localization timestamp"
    )
    if localization_age_s < 0.0:
        raise MissionValidationError("localization timestamp is in the future")
    nbv_by_track = {
        plan.track_id: [candidate.to_json_dict() for candidate in plan.candidates]
        for plan in next_best_views
    }
    evidence_ids = sorted(
        {
            evidence_id
            for track in tracks
            for evidence_id in track.evidence_ids
        }
    )
    payload: dict[str, Any] = {
        "schema": SEMANTIC_SNAPSHOT_SCHEMA,
        "mission_id": mission_id,
        "objective": objective,
        "objective_revision": int(objective_revision),
        "decision_generation": int(decision_generation),
        "event_generation": int(event_generation),
        "requested_object_classes": list(requested_classes),
        "map": {
            "map_id": map_id,
            "map_revision": map_revision,
            "frame_id": "map",
            "coverage_fraction": _finite(
                coverage_fraction, "coverage fraction"
            ),
        },
        "localization": {
            "frame_id": "map",
            "x_m": _finite(robot_x_m, "robot x"),
            "y_m": _finite(robot_y_m, "robot y"),
            "yaw_rad": _finite(robot_yaw_rad, "robot yaw"),
            "timestamp_s": localization_timestamp_s,
            "age_s": localization_age_s,
            "fresh": localization_age_s <= 0.50,
            "quality": 0.98,
        },
        "origin": {
            "frame_id": "map",
            "x_m": _finite(origin_x_m, "origin x"),
            "y_m": _finite(origin_y_m, "origin y"),
        },
        "frontiers": [
            {
                **candidate.to_json_dict(),
                "region_id": "recorded_room",
                "reachable": True,
                "last_validated_s": now_s,
            }
            for candidate in bounded_frontiers
        ],
        "tracks": [track.to_json_dict() for track in tuple(tracks)[:16]],
        "next_best_views": nbv_by_track,
        "active_goal": dict(active_goal or {}),
        "safety": {
            "stop": bool(stop),
            "estop": bool(estop),
            "collision_state": str(collision_state).upper(),
            "cancelled": bool(cancelled),
            "mission_lease_valid": bool(mission_lease_valid),
            "motion_evidence_fresh": (
                bool(motion_evidence_fresh) and localization_age_s <= 0.50
            ),
        },
        "budgets": {
            "remaining_route_m": _finite(
                remaining_route_budget_m, "remaining route budget"
            ),
        },
        "evidence_ids": evidence_ids,
        "authority": {
            "motion_authority": False,
            "physical_execution_enabled": False,
            "live_sensors": False,
            "serial_access": False,
        },
    }
    payload["snapshot_id"] = _digest(payload)
    return payload


def semantic_goal_output_schema() -> dict[str, Any]:
    """Strict structured-output schema; no geometry or control fields exist."""

    common = {
        "schema": {"type": "string", "enum": [SEMANTIC_GOAL_SCHEMA]},
        "mission_id": {"type": "string", "minLength": 1},
        "snapshot_id": {"type": "string", "minLength": 16},
        "decision_generation": {"type": "integer", "minimum": 1},
        "event_generation": {"type": "integer", "minimum": 0},
        "rationale": {"type": "string", "minLength": 1, "maxLength": 300},
    }

    def arguments_branch(
        properties: Mapping[str, Any], required: Sequence[str]
    ) -> dict[str, Any]:
        return {
            "type": "object",
            "additionalProperties": False,
            "properties": dict(properties),
            "required": list(required),
        }

    identifier = {"type": "string", "minLength": 1, "maxLength": 128}
    return {
        "$schema": "http://json-schema.org/draft-07/schema#",
        "title": "Sphero RVR hierarchical semantic goal",
        "type": "object",
        "additionalProperties": False,
        "required": [
            "schema",
            "mission_id",
            "snapshot_id",
            "decision_generation",
            "event_generation",
            "action",
            "arguments",
            "rationale",
        ],
        "properties": {
            **common,
            "action": {"type": "string", "enum": sorted(ALLOWED_ACTIONS)},
            "arguments": {
                "anyOf": [
                    arguments_branch(
                        {"frontier_id": identifier}, ["frontier_id"]
                    ),
                    arguments_branch({"track_id": identifier}, ["track_id"]),
                    arguments_branch(
                        {
                            "region_id": identifier,
                            "target_classes": {
                                "type": "array",
                                "minItems": 1,
                                "maxItems": 8,
                                "items": identifier,
                            },
                        },
                        ["region_id", "target_classes"],
                    ),
                    arguments_branch({}, []),
                    arguments_branch(
                        {
                            "outcome": {
                                "enum": ["complete", "partial", "blocked"],
                            },
                            "evidence_ids": {
                                "type": "array",
                                "minItems": 1,
                                "maxItems": 32,
                                "items": identifier,
                            },
                        },
                        ["outcome", "evidence_ids"],
                    ),
                ]
            },
        },
    }


def semantic_goal_provider_output_schema(
    snapshot: Mapping[str, Any],
) -> dict[str, Any]:
    """App-server-compatible semantic schema bound to one planning snapshot."""

    mission_id = str(snapshot.get("mission_id", "")).strip()
    snapshot_id = str(snapshot.get("snapshot_id", "")).strip()
    decision_generation = int(snapshot.get("decision_generation", 0))
    event_generation = int(snapshot.get("event_generation", -1))
    if (
        not mission_id
        or not snapshot_id
        or decision_generation < 1
        or event_generation < 0
    ):
        raise MissionValidationError(
            "semantic goal provider schema requires exact snapshot bindings"
        )
    schema = semantic_goal_output_schema()
    properties = schema["properties"]
    properties["mission_id"] = {"type": "string", "enum": [mission_id]}
    properties["snapshot_id"] = {"type": "string", "enum": [snapshot_id]}
    properties["decision_generation"] = {
        "type": "integer",
        "minimum": decision_generation,
        "maximum": decision_generation,
    }
    properties["event_generation"] = {
        "type": "integer",
        "minimum": event_generation,
        "maximum": event_generation,
    }
    supplied_actions = snapshot.get("provider_action_allowlist")
    if supplied_actions is not None:
        if (
            not isinstance(supplied_actions, list)
            or not supplied_actions
            or any(
                not isinstance(action, str) or action not in ALLOWED_ACTIONS
                for action in supplied_actions
            )
        ):
            raise MissionValidationError(
                "semantic provider action allowlist is invalid"
            )
        properties["action"] = {
            "type": "string",
            "enum": sorted(set(supplied_actions)),
        }
    return schema


@dataclass(frozen=True)
class SemanticGoalDecision:
    mission_id: str
    snapshot_id: str
    decision_generation: int
    event_generation: int
    action: str
    arguments: Mapping[str, Any]
    rationale: str
    provider_id: str
    model_id: str

    @classmethod
    def validated(
        cls,
        value: Mapping[str, Any],
        *,
        snapshot: Mapping[str, Any],
        expected_generation: int,
        provider_id: str,
        model_id: str,
    ) -> "SemanticGoalDecision":
        required = {
            "schema",
            "mission_id",
            "snapshot_id",
            "decision_generation",
            "event_generation",
            "action",
            "arguments",
            "rationale",
        }
        if set(value) != required:
            raise MissionValidationError(
                "semantic goal response fields do not match the strict schema"
            )
        if value.get("schema") != SEMANTIC_GOAL_SCHEMA:
            raise MissionValidationError("semantic goal schema is unsupported")
        if str(value.get("mission_id")) != str(snapshot.get("mission_id")):
            raise MissionValidationError("semantic goal mission binding mismatch")
        if str(value.get("snapshot_id")) != str(snapshot.get("snapshot_id")):
            raise MissionValidationError("semantic goal snapshot binding mismatch")
        generation = int(value.get("decision_generation", 0))
        if generation != expected_generation:
            raise MissionValidationError("semantic goal generation mismatch")
        if generation != int(snapshot.get("decision_generation", 0)):
            raise MissionValidationError(
                "semantic goal snapshot generation binding mismatch"
            )
        event_generation = int(value.get("event_generation", -1))
        if event_generation != int(snapshot.get("event_generation", -2)):
            raise MissionValidationError("semantic goal event binding mismatch")
        action = str(value.get("action", ""))
        if action not in ALLOWED_ACTIONS:
            raise MissionValidationError("semantic goal action is not allowlisted")
        arguments = value.get("arguments")
        if not isinstance(arguments, Mapping):
            raise MissionValidationError("semantic goal arguments must be an object")
        expected_keys: dict[str, set[str]] = {
            "go_to_frontier": {"frontier_id"},
            "inspect": {"track_id"},
            "search_region": {"region_id", "target_classes"},
            "return_to_start": set(),
            "wait": set(),
            "finish": {"outcome", "evidence_ids"},
        }
        if set(arguments) != expected_keys[action]:
            raise MissionValidationError(
                f"semantic goal arguments are invalid for {action}"
            )
        if any(key in FORBIDDEN_MODEL_KEYS for key in arguments):
            raise MissionValidationError(
                "semantic goal response attempted to provide deterministic geometry"
            )
        rationale = str(value.get("rationale", "")).strip()
        if not rationale or len(rationale) > 300:
            raise MissionValidationError("semantic goal rationale is invalid")
        cls._validate_arguments(action, arguments, snapshot, rationale)
        return cls(
            mission_id=str(value["mission_id"]),
            snapshot_id=str(value["snapshot_id"]),
            decision_generation=generation,
            event_generation=event_generation,
            action=action,
            arguments=json.loads(json.dumps(dict(arguments))),
            rationale=rationale,
            provider_id=str(provider_id),
            model_id=str(model_id),
        )

    @staticmethod
    def _validate_arguments(
        action: str,
        arguments: Mapping[str, Any],
        snapshot: Mapping[str, Any],
        rationale: str,
    ) -> None:
        referenced_id = ""
        if action == "go_to_frontier":
            referenced_id = str(arguments["frontier_id"])
            available = {
                str(item["signature"]) for item in snapshot.get("frontiers", ())
            }
            if referenced_id not in available:
                raise MissionValidationError("selected frontier is not in the snapshot")
        elif action == "inspect":
            referenced_id = str(arguments["track_id"])
            available = {
                str(item["track_id"]) for item in snapshot.get("tracks", ())
            }
            if referenced_id not in available:
                raise MissionValidationError("selected track is not in the snapshot")
        elif action == "search_region":
            referenced_id = str(arguments["region_id"])
            classes = arguments["target_classes"]
            if (
                not isinstance(classes, list)
                or not classes
                or len(classes) > 8
                or any(
                    not isinstance(item, str) or not item.strip()
                    for item in classes
                )
                or len(set(classes)) != len(classes)
            ):
                raise MissionValidationError("search target classes are invalid")
            requested = set(
                str(item) for item in snapshot.get("requested_object_classes", ())
            )
            if not set(classes) <= requested:
                raise MissionValidationError(
                    "search target classes are outside the approved objective"
                )
            regions = {
                str(item["region_id"]) for item in snapshot.get("frontiers", ())
            }
            if referenced_id not in regions:
                raise MissionValidationError("selected search region is unavailable")
        elif action == "finish":
            outcome = str(arguments["outcome"])
            evidence_ids = arguments["evidence_ids"]
            if outcome not in {"complete", "partial", "blocked"}:
                raise MissionValidationError("finish outcome is invalid")
            if (
                not isinstance(evidence_ids, list)
                or not evidence_ids
                or len(evidence_ids) > 32
                or any(
                    not isinstance(item, str) or not item
                    for item in evidence_ids
                )
                or len(set(evidence_ids)) != len(evidence_ids)
            ):
                raise MissionValidationError("finish requires evidence IDs")
            available = set(str(item) for item in snapshot.get("evidence_ids", ()))
            if not set(str(item) for item in evidence_ids) <= available:
                raise MissionValidationError(
                    "finish cites evidence outside the planning snapshot"
                )
            if not any(str(item) in rationale for item in evidence_ids):
                raise MissionValidationError(
                    "finish rationale must cite supplied evidence IDs"
                )
        if referenced_id and referenced_id not in rationale:
            raise MissionValidationError(
                "semantic goal rationale must cite its selected candidate ID"
            )

    def to_json_dict(self) -> dict[str, Any]:
        return {
            "schema": SEMANTIC_GOAL_SCHEMA,
            "mission_id": self.mission_id,
            "snapshot_id": self.snapshot_id,
            "decision_generation": self.decision_generation,
            "event_generation": self.event_generation,
            "action": self.action,
            "arguments": json.loads(json.dumps(dict(self.arguments))),
            "rationale": self.rationale,
            "provider_id": self.provider_id,
            "model_id": self.model_id,
        }


def _semantic_goal_model_projection(
    snapshot: Mapping[str, Any],
) -> dict[str, Any]:
    """Remove server-owned resolved poses while retaining semantic summaries."""

    projected = json.loads(json.dumps(dict(snapshot)))
    for frontier in projected.get("frontiers", ()):
        frontier.pop("approach_pose", None)
    for candidates in projected.get("next_best_views", {}).values():
        for candidate in candidates:
            for field in ("x_m", "y_m", "yaw_rad"):
                candidate.pop(field, None)
    active = projected.get("active_goal")
    if isinstance(active, Mapping):
        projected["active_goal"] = {
            key: value
            for key, value in active.items()
            if key not in FORBIDDEN_MODEL_KEYS
        }
    return projected


def semantic_goal_prompt(
    objective: str, snapshot: Mapping[str, Any]
) -> str:
    request = {
        "role": "Select exactly one bounded semantic exploration goal.",
        "objective": str(objective),
        "world_snapshot": _semantic_goal_model_projection(snapshot),
        "rules": [
            "Return exactly one response matching the supplied structured schema.",
            "Bind the exact mission_id, snapshot_id, decision_generation, and event_generation.",
            "Select only stable frontier, track, region, origin, wait, or evidence IDs supplied by the snapshot.",
            "Never emit poses, routes, paths, speeds, acceleration, clearance, leases, ROS names, files, credentials, motor commands, or code.",
            "Cite the selected candidate or evidence IDs in the concise rationale.",
        ],
    }
    evaluation = snapshot.get("evaluation")
    if isinstance(evaluation, Mapping):
        request["evaluation"] = json.loads(json.dumps(dict(evaluation)))
    return json.dumps(request, sort_keys=True, separators=(",", ":"), allow_nan=False)


@dataclass(frozen=True)
class ResolvedSemanticGoal:
    decision: SemanticGoalDecision
    kind: str
    target_id: str
    target_signature: str
    map_id: str
    map_revision: str
    x_m: Optional[float]
    y_m: Optional[float]
    yaw_rad: Optional[float]
    route_length_m: float
    minimum_clearance_m: float
    ready_at_s: float
    evidence_ids: tuple[str, ...] = ()

    def as_frontier_goal(self) -> FrontierGoal:
        if self.kind != "motion" or self.x_m is None or self.y_m is None:
            raise MissionValidationError("non-motion semantic goal has no Nav2 pose")
        return FrontierGoal(
            generation=self.decision.decision_generation,
            frontier_signature=self.target_signature,
            map_id=self.map_id,
            map_revision=self.map_revision,
            x_m=self.x_m,
            y_m=self.y_m,
            route_length_m=self.route_length_m,
            ready_at_s=self.ready_at_s,
        )


class DeterministicGoalResolver:
    def resolve(
        self,
        decision: SemanticGoalDecision,
        snapshot: Mapping[str, Any],
        *,
        ready_at_s: float,
    ) -> ResolvedSemanticGoal:
        map_value = snapshot["map"]
        map_id = str(map_value["map_id"])
        map_revision = str(map_value["map_revision"])
        pose = snapshot["localization"]
        if decision.action == "go_to_frontier":
            frontier_id = str(decision.arguments["frontier_id"])
            frontier = next(
                item
                for item in snapshot["frontiers"]
                if str(item["signature"]) == frontier_id
            )
            approach = frontier["approach_pose"]
            return self._motion(
                decision,
                frontier_id,
                frontier_id,
                map_id,
                map_revision,
                approach,
                float(frontier["path_distance_m"]),
                float(frontier["clearance_m"]),
                ready_at_s,
            )
        if decision.action == "inspect":
            track_id = str(decision.arguments["track_id"])
            candidates = snapshot.get("next_best_views", {}).get(track_id, ())
            if not candidates:
                raise MissionValidationError(
                    "selected track has no safe reachable Next-Best-View"
                )
            viewpoint = candidates[0]
            return self._motion(
                decision,
                track_id,
                str(viewpoint["viewpoint_id"]),
                map_id,
                map_revision,
                viewpoint,
                float(viewpoint["route_length_m"]),
                float(viewpoint["clearance_m"]),
                ready_at_s,
                yaw_rad=float(viewpoint["yaw_rad"]),
            )
        if decision.action == "search_region":
            region_id = str(decision.arguments["region_id"])
            options = [
                item
                for item in snapshot["frontiers"]
                if str(item["region_id"]) == region_id
            ]
            if not options:
                raise MissionValidationError("search region has no reachable frontier")
            frontier = min(
                options,
                key=lambda item: (
                    -float(item["information_gain_m"]),
                    float(item["path_distance_m"]),
                    str(item["signature"]),
                ),
            )
            return self._motion(
                decision,
                region_id,
                str(frontier["signature"]),
                map_id,
                map_revision,
                frontier["approach_pose"],
                float(frontier["path_distance_m"]),
                float(frontier["clearance_m"]),
                ready_at_s,
            )
        if decision.action == "return_to_start":
            origin = snapshot["origin"]
            route_length = math.hypot(
                float(origin["x_m"]) - float(pose["x_m"]),
                float(origin["y_m"]) - float(pose["y_m"]),
            )
            return self._motion(
                decision,
                "mission_origin",
                "mission_origin",
                map_id,
                map_revision,
                origin,
                route_length,
                0.15,
                ready_at_s,
            )
        if decision.action == "wait":
            return ResolvedSemanticGoal(
                decision,
                "wait",
                "wait",
                "wait",
                map_id,
                map_revision,
                None,
                None,
                None,
                0.0,
                0.0,
                ready_at_s,
            )
        evidence = tuple(
            str(item) for item in decision.arguments["evidence_ids"]
        )
        return ResolvedSemanticGoal(
            decision,
            "finish",
            str(decision.arguments["outcome"]),
            "finish",
            map_id,
            map_revision,
            None,
            None,
            None,
            0.0,
            0.0,
            ready_at_s,
            evidence,
        )

    @staticmethod
    def _motion(
        decision: SemanticGoalDecision,
        target_id: str,
        signature: str,
        map_id: str,
        map_revision: str,
        pose: Mapping[str, Any],
        route_length_m: float,
        clearance_m: float,
        ready_at_s: float,
        *,
        yaw_rad: Optional[float] = None,
    ) -> ResolvedSemanticGoal:
        return ResolvedSemanticGoal(
            decision=decision,
            kind="motion",
            target_id=target_id,
            target_signature=signature,
            map_id=map_id,
            map_revision=map_revision,
            x_m=_finite(pose["x_m"], "resolved goal x"),
            y_m=_finite(pose["y_m"], "resolved goal y"),
            yaw_rad=(
                None if yaw_rad is None else _finite(yaw_rad, "resolved goal yaw")
            ),
            route_length_m=max(
                0.0, _finite(route_length_m, "resolved route length")
            ),
            minimum_clearance_m=max(
                0.0, _finite(clearance_m, "resolved clearance")
            ),
            ready_at_s=max(0.0, _finite(ready_at_s, "resolved ready time")),
        )


@dataclass(frozen=True)
class RevalidationResult:
    accepted: bool
    reasons: tuple[str, ...]


def revalidate_resolved_goal(
    goal: ResolvedSemanticGoal,
    *,
    captured_snapshot: Mapping[str, Any],
    current_snapshot: Mapping[str, Any],
    minimum_clearance_m: float = 0.10,
) -> RevalidationResult:
    reasons: list[str] = []
    decision = goal.decision
    if decision.mission_id != str(current_snapshot.get("mission_id")):
        reasons.append("mission_changed")
    if int(captured_snapshot.get("objective_revision", -1)) != int(
        current_snapshot.get("objective_revision", -2)
    ):
        reasons.append("objective_changed")
    if decision.event_generation != int(
        current_snapshot.get("event_generation", -1)
    ):
        reasons.append("event_generation_changed")
    if str(goal.map_id) != str(current_snapshot.get("map", {}).get("map_id")):
        reasons.append("map_identity_changed")
    safety = current_snapshot.get("safety", {})
    if bool(safety.get("stop")):
        reasons.append("stop_active")
    if bool(safety.get("estop")):
        reasons.append("estop_active")
    if bool(safety.get("cancelled")):
        reasons.append("cancelled")
    if str(safety.get("collision_state", "")).upper() not in {"CLEAR", "SLOW"}:
        reasons.append("collision_veto")
    if not bool(safety.get("mission_lease_valid")):
        reasons.append("mission_lease_expired")
    if not bool(safety.get("motion_evidence_fresh")):
        reasons.append("motion_evidence_stale")
    if not bool(current_snapshot.get("localization", {}).get("fresh")):
        reasons.append("localization_stale")
    if goal.kind == "motion":
        if goal.minimum_clearance_m < minimum_clearance_m:
            reasons.append("clearance_below_minimum")
        if goal.route_length_m > float(
            current_snapshot.get("budgets", {}).get("remaining_route_m", -1.0)
        ):
            reasons.append("route_budget_exceeded")
    if decision.action in {"go_to_frontier", "search_region"}:
        signatures = {
            str(item["signature"]) for item in current_snapshot.get("frontiers", ())
        }
        if goal.target_signature not in signatures:
            reasons.append("frontier_signature_invalidated")
    elif decision.action == "inspect":
        tracks = {
            str(item["track_id"]): item
            for item in current_snapshot.get("tracks", ())
        }
        captured_tracks = {
            str(item["track_id"]): item
            for item in captured_snapshot.get("tracks", ())
        }
        track_id = decision.arguments["track_id"]
        current_track = tracks.get(track_id)
        captured_track = captured_tracks.get(track_id)
        if (
            not isinstance(current_track, Mapping)
            or not isinstance(captured_track, Mapping)
            or str(current_track.get("signature", ""))
            != str(captured_track.get("signature", ""))
        ):
            reasons.append("track_signature_changed")
        else:
            try:
                current_position = current_track["position"]
                captured_position = captured_track["position"]
                track_drift = math.hypot(
                    float(current_position["x_m"])
                    - float(captured_position["x_m"]),
                    float(current_position["y_m"])
                    - float(captured_position["y_m"]),
                )
            except (KeyError, TypeError, ValueError):
                track_drift = math.inf
            if (
                not math.isfinite(track_drift)
                or track_drift > MAX_TRACK_REVALIDATION_DRIFT_M
            ):
                reasons.append("track_position_changed")
        current_viewpoints = (
            current_snapshot.get("next_best_views", {}).get(track_id, ())
        )
        viewpoint_valid = False
        for item in current_viewpoints:
            if not isinstance(item, Mapping):
                continue
            try:
                viewpoint_drift = math.hypot(
                    float(item["x_m"]) - float(goal.x_m),
                    float(item["y_m"]) - float(goal.y_m),
                )
                clearance = float(item["clearance_m"])
            except (KeyError, TypeError, ValueError):
                continue
            if (
                math.isfinite(viewpoint_drift)
                and viewpoint_drift
                <= MAX_VIEWPOINT_REVALIDATION_DRIFT_M
                and math.isfinite(clearance)
                and clearance >= minimum_clearance_m
            ):
                viewpoint_valid = True
                break
        if not viewpoint_valid:
            reasons.append("viewpoint_invalidated")
    elif decision.action == "finish":
        available = set(
            str(item) for item in current_snapshot.get("evidence_ids", ())
        )
        if not set(goal.evidence_ids) <= available:
            reasons.append("finish_evidence_invalidated")
    return RevalidationResult(not reasons, tuple(reasons))


class SemanticGoalProvider(Protocol):
    provider_id: str
    model_id: str

    def choose(
        self, prompt: str, snapshot: Mapping[str, Any]
    ) -> Mapping[str, Any]: ...


class CodexOAuthSemanticGoalProvider:
    """Toolless ChatGPT-OAuth provider constrained to the semantic schema."""

    provider_id = "openai-codex-oauth-semantic-goal"

    def __init__(
        self,
        *,
        model: Optional[str] = None,
        reasoning_effort: str = "low",
        codex_command: str = "codex",
        timeout_s: float = 120.0,
        client: Optional[CodexAppServerClient] = None,
    ) -> None:
        if reasoning_effort not in ALLOWED_REASONING_EFFORTS:
            raise MissionValidationError(
                f"unsupported reasoning effort: {reasoning_effort}"
            )
        if not math.isfinite(float(timeout_s)) or float(timeout_s) <= 0.0:
            raise MissionValidationError(
                "semantic goal provider timeout must be positive and finite"
            )
        self.model_id = model or os.environ.get(
            "OPENAI_MODEL", DEFAULT_CODEX_MODEL_ID
        )
        self.reasoning_effort = reasoning_effort
        self.timeout_s = float(timeout_s)
        self._client = client or CodexAppServerClient(
            codex_command=codex_command
        )
        self._latency_history: list[dict[str, Any]] = []

    def choose(
        self, prompt: str, snapshot: Mapping[str, Any]
    ) -> Mapping[str, Any]:
        started = time.perf_counter()
        metric: dict[str, Any] = {
            "schema": "sphero_rvr.semantic_goal_latency.v1",
            "snapshot_id": str(snapshot.get("snapshot_id", "")),
            "decision_generation": int(snapshot.get("decision_generation", 0)),
            "model_id": self.model_id,
            "reasoning_effort": self.reasoning_effort,
            "success": False,
            "total_ms": 0.0,
            "oauth_client_startup_ms": 0.0,
            "server_restart_count": 0,
        }
        try:
            with tempfile.TemporaryDirectory(
                prefix="rvr-semantic-goal-"
            ) as directory:
                output, startup_ms, restart_count = self._client.run_turn(
                    prompt=str(prompt),
                    model=self.model_id,
                    effort=self.reasoning_effort,
                    output_schema=semantic_goal_provider_output_schema(snapshot),
                    cwd=str(Path(directory)),
                    image_path=None,
                    timeout_s=self.timeout_s,
                )
            try:
                parsed = json.loads(output)
            except json.JSONDecodeError as exc:
                raise MissionValidationError(
                    "Codex semantic-goal provider returned malformed JSON"
                ) from exc
            if not isinstance(parsed, Mapping):
                raise MissionValidationError(
                    "Codex semantic-goal provider output must be an object"
                )
            metric["oauth_client_startup_ms"] = startup_ms
            metric["server_restart_count"] = restart_count
            metric["success"] = True
            return dict(parsed)
        finally:
            metric["total_ms"] = (time.perf_counter() - started) * 1000.0
            self._latency_history.append(metric)
            if len(self._latency_history) > 100:
                self._latency_history = self._latency_history[-100:]

    def latency_history(self) -> tuple[Mapping[str, Any], ...]:
        return tuple(json.loads(json.dumps(self._latency_history)))

    def cancel(self) -> None:
        self._client.cancel()

    def close(self) -> None:
        self._client.close()


class ScriptedSemanticGoalProvider:
    """Deterministic provider used only for bounded replay evidence."""

    provider_id = "scripted-semantic-goal-replay"
    model_id = "recorded-latency-fixture"

    def __init__(
        self,
        decisions: Sequence[
            Mapping[str, Any] | Callable[[Mapping[str, Any]], Mapping[str, Any]]
        ],
        *,
        release_event: Optional[threading.Event] = None,
    ) -> None:
        self._decisions = deque(decisions)
        self._release_event = release_event
        self._cancelled = threading.Event()
        self.calls = 0
        self.completed_calls = 0
        self.captured_snapshots: list[dict[str, Any]] = []

    def choose(
        self, prompt: str, snapshot: Mapping[str, Any]
    ) -> Mapping[str, Any]:
        del prompt
        self.calls += 1
        self.captured_snapshots.append(
            json.loads(json.dumps(dict(snapshot)))
        )
        while (
            self._release_event is not None
            and not self._release_event.is_set()
            and not self._cancelled.wait(0.005)
        ):
            pass
        if self._cancelled.is_set():
            raise MissionValidationError("scripted semantic provider cancelled")
        if not self._decisions:
            raise MissionValidationError("scripted semantic provider exhausted")
        decision = self._decisions.popleft()
        result = (
            decision(snapshot)
            if callable(decision)
            else json.loads(json.dumps(dict(decision)))
        )
        self.completed_calls += 1
        return result

    def cancel(self) -> None:
        self._cancelled.set()


class SemanticEventKind(str, Enum):
    SAFETY = "safety"
    OPERATOR_REDIRECT = "operator_redirect"
    INVALID_TARGET = "invalid_target"
    LOST_PROGRESS = "lost_progress"
    NEW_DETECTION = "new_detection"
    UNCERTAINTY_CHANGED = "uncertainty_changed"


@dataclass(frozen=True)
class SemanticReplanEvent:
    event_id: str
    kind: SemanticEventKind
    observed_at_s: float
    target_id: str = ""
    confidence: float = 1.0
    stable_observations: int = 1


@dataclass
class SemanticEventGate:
    minimum_confidence: float = 0.70
    minimum_stable_observations: int = 2
    hysteresis_s: float = 2.0
    _last_accepted: dict[tuple[str, str], float] = field(default_factory=dict)

    def accept(self, event: SemanticReplanEvent) -> tuple[bool, str]:
        if event.kind is SemanticEventKind.SAFETY:
            return True, "safety_immediate"
        if event.kind in {
            SemanticEventKind.NEW_DETECTION,
            SemanticEventKind.UNCERTAINTY_CHANGED,
        }:
            if event.confidence < self.minimum_confidence:
                return False, "confidence_below_gate"
            if event.stable_observations < self.minimum_stable_observations:
                return False, "insufficient_stability"
        key = (event.kind.value, event.target_id)
        previous = self._last_accepted.get(key)
        if (
            previous is not None
            and event.observed_at_s - previous < self.hysteresis_s
        ):
            return False, "hysteresis_coalesced"
        self._last_accepted[key] = event.observed_at_s
        return True, "accepted"


@dataclass(frozen=True)
class SemanticControllerStep:
    handoff: HandoffStep
    events: tuple[Mapping[str, Any], ...]
    provider_in_flight: bool
    prefetched_generation: Optional[int]


class AsyncSemanticGoalController:
    """One-at-a-time async provider plus deterministic goal revalidation."""

    def __init__(
        self,
        provider: SemanticGoalProvider,
        *,
        provider_p95_s: float = 12.691,
        prefetch_margin_s: float = 1.0,
        modeled_provider_latency_s: Optional[float] = None,
        queue_depth: int = 3,
    ) -> None:
        self.provider = provider
        self.provider_p95_s = _finite(provider_p95_s, "provider p95")
        self.prefetch_margin_s = _finite(prefetch_margin_s, "prefetch margin")
        if self.provider_p95_s <= 0.0 or self.prefetch_margin_s < 0.0:
            raise ValueError("provider latency profile is invalid")
        self.modeled_provider_latency_s = (
            None
            if modeled_provider_latency_s is None
            else _finite(modeled_provider_latency_s, "modeled provider latency")
        )
        if (
            self.modeled_provider_latency_s is not None
            and self.modeled_provider_latency_s < 0.0
        ):
            raise ValueError("modeled provider latency cannot be negative")
        self.follower = ContinuousGoalFollowerReplay(queue_depth=queue_depth)
        self.resolver = DeterministicGoalResolver()
        self.event_gate = SemanticEventGate()
        self._pool = ThreadPoolExecutor(
            max_workers=1, thread_name_prefix="semantic-goal-provider"
        )
        self._future: Optional[Future[Mapping[str, Any]]] = None
        self._future_snapshot: Optional[dict[str, Any]] = None
        self._future_generation = 0
        self._future_started_at_s = 0.0
        self._future_ready_at_s = 0.0
        self._future_invalidated_reason = ""
        self._prefetched: Optional[ResolvedSemanticGoal] = None
        self._prefetched_snapshot: Optional[dict[str, Any]] = None
        self._ready_non_motion: Optional[ResolvedSemanticGoal] = None
        self._active: Optional[ResolvedSemanticGoal] = None
        self._active_snapshot: Optional[dict[str, Any]] = None
        self._generation = 0
        self._event_generation = 0
        self._events: list[dict[str, Any]] = []
        self._provider_calls_started = 0
        self._provider_calls_completed = 0
        self._provider_rejections = 0
        self._motion_goal_decisions = 0
        self._distance_m = 0.0
        self._last_remaining_m: Optional[float] = None
        self._closed = False

    @property
    def prefetch_threshold_s(self) -> float:
        return self.provider_p95_s + self.prefetch_margin_s

    def start(
        self,
        raw_decision: Mapping[str, Any],
        snapshot: Mapping[str, Any],
        *,
        now_s: float = 0.0,
    ) -> SemanticControllerStep:
        self._generation = 1
        self._event_generation = int(snapshot.get("event_generation", 0))
        decision = SemanticGoalDecision.validated(
            raw_decision,
            snapshot=snapshot,
            expected_generation=1,
            provider_id=self.provider.provider_id,
            model_id=self.provider.model_id,
        )
        resolved = self.resolver.resolve(decision, snapshot, ready_at_s=now_s)
        if resolved.kind != "motion":
            raise MissionValidationError("Phase 3 replay must start with a motion goal")
        validation = revalidate_resolved_goal(
            resolved,
            captured_snapshot=snapshot,
            current_snapshot=snapshot,
        )
        if not validation.accepted:
            raise MissionValidationError(
                f"initial semantic goal failed revalidation: {validation.reasons}"
            )
        self._active = resolved
        self._active_snapshot = json.loads(json.dumps(dict(snapshot)))
        self._motion_goal_decisions = 1
        self._last_remaining_m = resolved.route_length_m
        handoff = self.follower.start([resolved.as_frontier_goal()], now_s=now_s)
        event = self._record(
            "semantic_goal_started",
            now_s,
            generation=1,
            action=decision.action,
            target_id=resolved.target_id,
        )
        return SemanticControllerStep(handoff, (event,), False, None)

    def tick(
        self,
        snapshot: Mapping[str, Any],
        *,
        now_s: float,
        remaining_distance_m: float,
        eta_s: float,
        collision_state: str = "CLEAR",
        stop: bool = False,
        estop: bool = False,
        cancelled: bool = False,
        motion_evidence_fresh: bool = True,
    ) -> SemanticControllerStep:
        events: list[Mapping[str, Any]] = []
        if self._last_remaining_m is not None:
            progress = max(0.0, self._last_remaining_m - remaining_distance_m)
            self._distance_m += progress
        self._last_remaining_m = remaining_distance_m
        if (
            stop
            or estop
            or cancelled
            or not motion_evidence_fresh
            or str(collision_state).upper() not in {"CLEAR", "SLOW"}
        ):
            in_flight = self._future is not None
            handoff = self.follower.advance(
                now_s=now_s,
                remaining_distance_m=remaining_distance_m,
                collision_state=collision_state,
                stop=stop,
                estop=estop,
                cancelled=cancelled,
                motion_evidence_fresh=motion_evidence_fresh,
            )
            if in_flight:
                events.append(
                    self._record(
                        "safety_veto_during_provider",
                        now_s,
                        reason=handoff.command.reason,
                    )
                )
            return SemanticControllerStep(
                handoff, tuple(events), in_flight, self._prefetched_generation()
            )

        events.extend(self._collect_provider(snapshot, now_s=now_s))
        events.extend(self._revalidate_prefetch(snapshot, now_s=now_s))
        handoff = self.follower.advance(
            now_s=now_s,
            remaining_distance_m=remaining_distance_m,
            collision_state=collision_state,
            motion_evidence_fresh=motion_evidence_fresh,
        )
        if any(
            event.get("kind") in {"atomic_handoff", "planning_resume"}
            for event in handoff.events
        ):
            if self._prefetched is not None:
                self._active = self._prefetched
                self._active_snapshot = self._prefetched_snapshot
                self._prefetched = None
                self._prefetched_snapshot = None
                self._last_remaining_m = self._active.route_length_m
        if (
            self._future is None
            and self._prefetched is None
            and self._ready_non_motion is None
            and handoff.state in {"navigating", "wait_planning"}
            and eta_s <= self.prefetch_threshold_s
        ):
            events.append(
                self._dispatch(
                    snapshot,
                    now_s=now_s,
                    remaining_distance_m=remaining_distance_m,
                    eta_s=eta_s,
                )
            )
        return SemanticControllerStep(
            handoff,
            tuple(events),
            self._future is not None,
            self._prefetched_generation(),
        )

    def handle_event(
        self,
        event: SemanticReplanEvent,
        snapshot: Mapping[str, Any],
        *,
        now_s: float,
    ) -> SemanticControllerStep:
        accepted, reason = self.event_gate.accept(event)
        if not accepted:
            handoff = self.follower.advance(
                now_s=now_s,
                remaining_distance_m=max(0.0, self._last_remaining_m or 0.0),
            )
            record = self._record(
                "semantic_event_coalesced",
                now_s,
                event_id=event.event_id,
                reason=reason,
            )
            return SemanticControllerStep(
                handoff,
                (record,),
                self._future is not None,
                self._prefetched_generation(),
            )
        if event.kind is SemanticEventKind.SAFETY:
            return self.tick(
                snapshot,
                now_s=now_s,
                remaining_distance_m=max(0.0, self._last_remaining_m or 0.0),
                eta_s=0.0,
                collision_state="BLOCKED",
            )
        next_event_generation = int(snapshot.get("event_generation", -1))
        if next_event_generation != self._event_generation + 1:
            raise MissionValidationError(
                "semantic replan snapshot must advance event generation exactly once"
            )
        self._event_generation = next_event_generation
        if self._future is not None:
            self._future_invalidated_reason = event.kind.value
        if self._prefetched is not None:
            self.follower.discard_prefetch(
                self._prefetched.decision.decision_generation,
                now_s=now_s,
                reason=event.kind.value,
            )
            self._prefetched = None
            self._prefetched_snapshot = None
        if (
            self._ready_non_motion is not None
            and self._ready_non_motion.decision.action == "wait"
        ):
            self._ready_non_motion = None
        handoff = self.follower.preempt_for_replan(
            now_s=now_s, reason=event.kind.value
        )
        records = [
            self._record(
            "event_triggered_replan",
            now_s,
            event_id=event.event_id,
            event_kind=event.kind.value,
            target_id=event.target_id,
            )
        ]
        if self._future is None:
            records.append(
                self._dispatch(
                    snapshot,
                    now_s=now_s,
                    remaining_distance_m=max(
                        0.0, self._last_remaining_m or 0.0
                    ),
                    eta_s=0.0,
                )
            )
        return SemanticControllerStep(
            handoff,
            tuple(records),
            self._future is not None,
            None,
        )

    def _dispatch(
        self,
        snapshot: Mapping[str, Any],
        *,
        now_s: float,
        remaining_distance_m: float,
        eta_s: float,
    ) -> Mapping[str, Any]:
        if self._future is not None:
            raise MissionValidationError("only one semantic prefetch may be active")
        captured = json.loads(json.dumps(dict(snapshot)))
        self._generation += 1
        captured["decision_generation"] = self._generation
        captured["active_goal"] = (
            {}
            if self._active is None
            else {
                "generation": self._active.decision.decision_generation,
                "action": self._active.decision.action,
                "target_id": self._active.target_id,
                "remaining_distance_m": max(
                    0.0,
                    _finite(
                        remaining_distance_m,
                        "active goal remaining distance",
                    ),
                ),
                "eta_s": max(0.0, _finite(eta_s, "active goal ETA")),
                "follower_state": self.follower.state,
                "prefetch_state": "requested",
            }
        )
        captured.pop("snapshot_id", None)
        captured["snapshot_id"] = _digest(captured)
        self._future_generation = self._generation
        self._future_snapshot = captured
        self._future_started_at_s = now_s
        self._future_ready_at_s = now_s + (
            self.modeled_provider_latency_s or 0.0
        )
        self._provider_calls_started += 1
        prompt = semantic_goal_prompt(str(snapshot["objective"]), captured)
        self._future = self._pool.submit(self.provider.choose, prompt, captured)
        return self._record(
            "prefetch_started",
            now_s,
            generation=self._future_generation,
            snapshot_id=captured["snapshot_id"],
            modeled_ready_at_s=self._future_ready_at_s,
            threshold_s=self.prefetch_threshold_s,
        )

    def _collect_provider(
        self, snapshot: Mapping[str, Any], *, now_s: float
    ) -> list[Mapping[str, Any]]:
        if (
            self._future is None
            or not self._future.done()
            or now_s < self._future_ready_at_s
        ):
            return []
        future = self._future
        captured = self._future_snapshot
        generation = self._future_generation
        invalidated_reason = self._future_invalidated_reason
        started_at_s = self._future_started_at_s
        self._future = None
        self._future_snapshot = None
        self._future_started_at_s = None
        self._future_invalidated_reason = ""
        self._provider_calls_completed += 1
        provider_elapsed_s = (
            None
            if started_at_s is None
            else max(0.0, now_s - float(started_at_s))
        )
        assert captured is not None
        if invalidated_reason:
            self._provider_rejections += 1
            return [
                self._record(
                    "prefetch_discarded",
                    now_s,
                    generation=generation,
                    reason=f"event_invalidated:{invalidated_reason}",
                    provider_elapsed_s=provider_elapsed_s,
                    snapshot_id=str(captured["snapshot_id"]),
                )
            ]
        try:
            raw = future.result()
            decision = SemanticGoalDecision.validated(
                raw,
                snapshot=captured,
                expected_generation=generation,
                provider_id=self.provider.provider_id,
                model_id=self.provider.model_id,
            )
            resolved = self.resolver.resolve(
                decision, captured, ready_at_s=now_s
            )
            validation = revalidate_resolved_goal(
                resolved,
                captured_snapshot=captured,
                current_snapshot=snapshot,
            )
            if not validation.accepted:
                raise MissionValidationError(
                    f"revalidation failed: {','.join(validation.reasons)}"
                )
        except Exception as exc:
            self._provider_rejections += 1
            return [
                self._record(
                    "prefetch_discarded",
                    now_s,
                    generation=generation,
                    reason=str(exc),
                    provider_elapsed_s=provider_elapsed_s,
                    snapshot_id=str(captured["snapshot_id"]),
                )
            ]
        if resolved.kind == "motion":
            self.follower.submit_prefetch(resolved.as_frontier_goal())
            self._prefetched = resolved
            self._prefetched_snapshot = captured
            self._motion_goal_decisions += 1
            return [
                self._record(
                    "prefetch_revalidated",
                    now_s,
                    generation=generation,
                    action=decision.action,
                    target_id=resolved.target_id,
                    provider_elapsed_s=provider_elapsed_s,
                    snapshot_id=str(captured["snapshot_id"]),
                )
            ]
        self._ready_non_motion = resolved
        return [
            self._record(
                "semantic_non_motion_goal_ready",
                now_s,
                generation=generation,
                action=decision.action,
                target_id=resolved.target_id,
                provider_elapsed_s=provider_elapsed_s,
                snapshot_id=str(captured["snapshot_id"]),
                decision=decision.to_json_dict(),
            )
        ]

    def _revalidate_prefetch(
        self, snapshot: Mapping[str, Any], *, now_s: float
    ) -> list[Mapping[str, Any]]:
        if self._prefetched is None or self._prefetched_snapshot is None:
            return []
        result = revalidate_resolved_goal(
            self._prefetched,
            captured_snapshot=self._prefetched_snapshot,
            current_snapshot=snapshot,
        )
        if result.accepted:
            return []
        generation = self._prefetched.decision.decision_generation
        self.follower.discard_prefetch(
            generation,
            now_s=now_s,
            reason=",".join(result.reasons),
        )
        self._prefetched = None
        self._prefetched_snapshot = None
        self._provider_rejections += 1
        return [
            self._record(
                "prefetch_discarded",
                now_s,
                generation=generation,
                reason=",".join(result.reasons),
            )
        ]

    def _prefetched_generation(self) -> Optional[int]:
        return (
            None
            if self._prefetched is None
            else self._prefetched.decision.decision_generation
        )

    def provider_snapshot_in_flight(
        self,
    ) -> Optional[dict[str, Any]]:
        """Return an evidence-only copy of the exact snapshot sent upstream."""

        if self._future_snapshot is None:
            return None
        return json.loads(json.dumps(self._future_snapshot))

    def resolved_motion_goals(
        self,
    ) -> tuple[
        tuple[ResolvedSemanticGoal, Mapping[str, Any]],
        ...,
    ]:
        """Return the active/validated successor geometry owned by the server.

        This is the only live-binding export from the replay-proven controller;
        model responses remain semantic IDs and consumers must still revalidate
        against their current snapshot before sending a Nav2 goal.
        """

        result: list[
            tuple[ResolvedSemanticGoal, Mapping[str, Any]]
        ] = []
        if self._active is not None and self._active_snapshot is not None:
            result.append((self._active, self._active_snapshot))
        if (
            self._prefetched is not None
            and self._prefetched_snapshot is not None
        ):
            result.append((self._prefetched, self._prefetched_snapshot))
        return tuple(result)

    def ready_non_motion_goal(self) -> Optional[ResolvedSemanticGoal]:
        return self._ready_non_motion

    def _record(
        self, kind: str, at_s: float, **details: Any
    ) -> dict[str, Any]:
        event = {"kind": kind, "at_s": float(at_s), **details}
        self._events.append(event)
        return event

    def evidence(self) -> dict[str, Any]:
        decisions_per_m = (
            self._motion_goal_decisions / self._distance_m
            if self._distance_m > 0.0
            else math.inf
        )
        baseline_decisions_per_m = 4.0
        return {
            "schema": PHASE3_REPLAY_SCHEMA,
            "semantic_goal_schema": SEMANTIC_GOAL_SCHEMA,
            "provider_latency_p95_s": self.provider_p95_s,
            "prefetch_margin_s": self.prefetch_margin_s,
            "prefetch_threshold_s": self.prefetch_threshold_s,
            "provider_calls_started": self._provider_calls_started,
            "provider_calls_completed": self._provider_calls_completed,
            "provider_rejections": self._provider_rejections,
            "motion_goal_decisions": self._motion_goal_decisions,
            "distance_m": self._distance_m,
            "decisions_per_m": decisions_per_m,
            "legacy_025m_decisions_per_m": baseline_decisions_per_m,
            "decision_reduction_ratio": (
                baseline_decisions_per_m / decisions_per_m
                if decisions_per_m > 0.0 and math.isfinite(decisions_per_m)
                else 0.0
            ),
            "provider_in_flight": self._future is not None,
            "ready_non_motion_goal": (
                None
                if self._ready_non_motion is None
                else self._ready_non_motion.decision.to_json_dict()
            ),
            "events": list(self._events),
            "handoff": self.follower.evidence(),
            "authority": {
                "motion_authority": False,
                "physical_execution_enabled": False,
                "live_sensors": False,
                "serial_access": False,
            },
            "carryovers": {
                "phase2_accuracy_is_physical_certification": False,
                "pi_no_motion_wfd_before_physical": True,
                "pi_command_ownership_before_physical": True,
                "dropoff_sensing_available": False,
            },
        }

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        cancel = getattr(self.provider, "cancel", None)
        if callable(cancel):
            cancel()
        if self._future is not None:
            self._future.cancel()
        self._pool.shutdown(wait=True, cancel_futures=True)
        close_provider = getattr(self.provider, "close", None)
        if callable(close_provider):
            close_provider()
