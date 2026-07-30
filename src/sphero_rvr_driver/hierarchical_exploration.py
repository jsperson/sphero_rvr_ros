"""Deterministic frontier and continuous-handoff contracts for replay.

The module is intentionally ROS-free.  It consumes the same trinary PGM/YAML
maps emitted by ``slam_toolbox`` and models the Nav2 ``NavigateThroughPoses``
handoff boundary without publishing a Twist or owning physical authority.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from enum import Enum
import hashlib
import json
import math
from pathlib import Path
from typing import Any, Iterable, Mapping, Optional, Sequence

from .adaptive_mission_controller import ReplayAdaptiveMissionExecutor
from .mission_api import MissionValidationError


FRONTIER_SCHEMA = "sphero_rvr.frontier_snapshot.v1"
HANDOFF_SCHEMA = "sphero_rvr.nav2_handoff_replay.v1"
UPSTREAM_FRONTIER_REVISION = "ec530d2a813739cd25dd0c438d2365c510b9fad8"
PRIVATE_NAV2_CMD_TOPIC = "/nav2_cmd_vel_request"
SUPERVISOR_REQUEST_TOPIC = "/cmd_vel"
MOTOR_TOPIC = "/cmd_vel_motor"


@dataclass(frozen=True)
class OccupancyGrid:
    width: int
    height: int
    resolution_m: float
    origin_x_m: float
    origin_y_m: float
    frame_id: str
    map_id: str
    revision: str
    cells: tuple[int, ...]
    source: str

    def __post_init__(self) -> None:
        if self.width <= 0 or self.height <= 0:
            raise MissionValidationError("occupancy grid dimensions must be positive")
        if (
            not math.isfinite(self.resolution_m)
            or self.resolution_m <= 0.0
        ):
            raise MissionValidationError("occupancy grid resolution must be positive")
        if len(self.cells) != self.width * self.height:
            raise MissionValidationError("occupancy grid cell count is inconsistent")
        if not self.map_id or not self.revision:
            raise MissionValidationError("occupancy grid identity and revision are required")
        if any(value not in {-1, 0, 100} for value in self.cells):
            raise MissionValidationError("occupancy grid must be trinary")

    def index(self, cell: tuple[int, int]) -> int:
        x, y = cell
        if x < 0 or y < 0 or x >= self.width or y >= self.height:
            raise IndexError(cell)
        return y * self.width + x

    def value(self, cell: tuple[int, int]) -> int:
        return self.cells[self.index(cell)]

    def world_to_cell(self, x_m: float, y_m: float) -> tuple[int, int]:
        x = math.floor((float(x_m) - self.origin_x_m) / self.resolution_m)
        y = math.floor((float(y_m) - self.origin_y_m) / self.resolution_m)
        cell = (int(x), int(y))
        self.index(cell)
        return cell

    def cell_to_world(self, cell: tuple[int, int]) -> tuple[float, float]:
        self.index(cell)
        return (
            self.origin_x_m + (cell[0] + 0.5) * self.resolution_m,
            self.origin_y_m + (cell[1] + 0.5) * self.resolution_m,
        )


def _yaml_scalars(path: Path) -> dict[str, str]:
    values: dict[str, str] = {}
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.split("#", 1)[0].strip()
        if not line or ":" not in line:
            continue
        key, value = line.split(":", 1)
        values[key.strip()] = value.strip()
    return values


def _pgm_tokens(data: bytes) -> tuple[bytes, int, int, int, bytes]:
    tokens: list[bytes] = []
    offset = 0
    while len(tokens) < 4:
        while offset < len(data) and data[offset : offset + 1].isspace():
            offset += 1
        if offset >= len(data):
            raise MissionValidationError("PGM header is truncated")
        if data[offset : offset + 1] == b"#":
            newline = data.find(b"\n", offset)
            if newline < 0:
                raise MissionValidationError("PGM comment is unterminated")
            offset = newline + 1
            continue
        end = offset
        while end < len(data) and not data[end : end + 1].isspace():
            end += 1
        tokens.append(data[offset:end])
        offset = end
    while offset < len(data) and data[offset : offset + 1].isspace():
        offset += 1
    try:
        magic, width, height, maximum = (
            tokens[0],
            int(tokens[1]),
            int(tokens[2]),
            int(tokens[3]),
        )
    except (TypeError, ValueError) as exc:
        raise MissionValidationError("PGM header is malformed") from exc
    return magic, width, height, maximum, data[offset:]


def _pgm_pixels(path: Path) -> tuple[int, int, tuple[int, ...]]:
    magic, width, height, maximum, body = _pgm_tokens(path.read_bytes())
    if width <= 0 or height <= 0 or maximum != 255:
        raise MissionValidationError("PGM must have positive dimensions and max value 255")
    count = width * height
    if magic == b"P5":
        if len(body) < count:
            raise MissionValidationError("binary PGM pixels are truncated")
        pixels = tuple(body[:count])
    elif magic == b"P2":
        try:
            pixels = tuple(int(token) for token in body.split())
        except ValueError as exc:
            raise MissionValidationError("ASCII PGM pixels are malformed") from exc
        if len(pixels) != count:
            raise MissionValidationError("ASCII PGM pixel count is inconsistent")
    else:
        raise MissionValidationError("only P2 and P5 PGM maps are supported")
    if any(pixel < 0 or pixel > 255 for pixel in pixels):
        raise MissionValidationError("PGM pixel lies outside [0, 255]")
    return width, height, pixels


def load_slam_toolbox_map(
    yaml_path: str | Path,
    *,
    map_id: Optional[str] = None,
    frame_id: str = "map",
) -> OccupancyGrid:
    """Load one saved ``slam_toolbox`` trinary map with a content revision."""

    yaml_file = Path(yaml_path)
    values = _yaml_scalars(yaml_file)
    required = {
        "image",
        "mode",
        "resolution",
        "origin",
        "negate",
        "occupied_thresh",
        "free_thresh",
    }
    missing = sorted(required - values.keys())
    if missing:
        raise MissionValidationError(
            f"slam_toolbox map YAML lacks required fields: {', '.join(missing)}"
        )
    if values["mode"].strip("\"'").lower() != "trinary":
        raise MissionValidationError("only trinary slam_toolbox maps are supported")
    image_file = (yaml_file.parent / values["image"].strip("\"'")).resolve()
    width, height, pixels = _pgm_pixels(image_file)
    try:
        resolution = float(values["resolution"])
        origin_values = json.loads(values["origin"])
        negate = int(values["negate"])
        occupied_threshold = float(values["occupied_thresh"])
        free_threshold = float(values["free_thresh"])
    except (TypeError, ValueError, json.JSONDecodeError) as exc:
        raise MissionValidationError("slam_toolbox map metadata is malformed") from exc
    if (
        not isinstance(origin_values, list)
        or len(origin_values) != 3
        or negate not in {0, 1}
        or not 0.0 <= free_threshold < occupied_threshold <= 1.0
    ):
        raise MissionValidationError("slam_toolbox map metadata is invalid")

    rows = [
        pixels[row * width : (row + 1) * width]
        for row in range(height)
    ]
    occupancy: list[int] = []
    for row in reversed(rows):
        for pixel in row:
            probability = (pixel if negate else 255 - pixel) / 255.0
            if probability > occupied_threshold:
                occupancy.append(100)
            elif probability < free_threshold:
                occupancy.append(0)
            else:
                occupancy.append(-1)
    revision_payload = {
        "yaml": values,
        "image_sha256": hashlib.sha256(image_file.read_bytes()).hexdigest(),
        "cells": occupancy,
    }
    revision = hashlib.sha256(
        json.dumps(
            revision_payload, sort_keys=True, separators=(",", ":")
        ).encode("utf-8")
    ).hexdigest()
    return OccupancyGrid(
        width=width,
        height=height,
        resolution_m=resolution,
        origin_x_m=float(origin_values[0]),
        origin_y_m=float(origin_values[1]),
        frame_id=str(frame_id),
        map_id=str(map_id or yaml_file.stem),
        revision=revision,
        cells=tuple(occupancy),
        source=str(yaml_file),
    )


@dataclass(frozen=True)
class FrontierDetectionConfig:
    minimum_frontier_cells: int = 3
    minimum_clearance_m: float = 0.10
    connectivity: int = 8

    def __post_init__(self) -> None:
        if self.minimum_frontier_cells <= 0:
            raise ValueError("minimum_frontier_cells must be positive")
        if (
            not math.isfinite(self.minimum_clearance_m)
            or self.minimum_clearance_m < 0.0
        ):
            raise ValueError("minimum_clearance_m must be finite and non-negative")
        if self.connectivity not in {4, 8}:
            raise ValueError("connectivity must be 4 or 8")


@dataclass(frozen=True)
class FrontierCandidate:
    signature: str
    map_id: str
    map_revision: str
    cells: tuple[tuple[int, int], ...]
    approach_cells: tuple[tuple[int, int], ...]
    approach_cell: tuple[int, int]
    approach_x_m: float
    approach_y_m: float
    clearance_m: float
    path_distance_m: float
    information_gain_m: float

    def to_json_dict(self) -> dict[str, Any]:
        return {
            "signature": self.signature,
            "map_id": self.map_id,
            "map_revision": self.map_revision,
            "cell_count": len(self.cells),
            "approach_region": [list(cell) for cell in self.approach_cells],
            "approach_pose": {
                "frame_id": "map",
                "x_m": self.approach_x_m,
                "y_m": self.approach_y_m,
            },
            "clearance_m": self.clearance_m,
            "path_distance_m": self.path_distance_m,
            "information_gain_m": self.information_gain_m,
        }


def _neighbors(
    grid: OccupancyGrid,
    cell: tuple[int, int],
    connectivity: int,
) -> Iterable[tuple[int, int]]:
    offsets = (
        ((-1, 0), (1, 0), (0, -1), (0, 1))
        if connectivity == 4
        else (
            (-1, 0),
            (1, 0),
            (0, -1),
            (0, 1),
            (-1, -1),
            (-1, 1),
            (1, -1),
            (1, 1),
        )
    )
    for dx, dy in offsets:
        candidate = (cell[0] + dx, cell[1] + dy)
        if 0 <= candidate[0] < grid.width and 0 <= candidate[1] < grid.height:
            yield candidate


def _reachable_free(
    grid: OccupancyGrid, start: tuple[int, int]
) -> dict[tuple[int, int], int]:
    if grid.value(start) != 0:
        raise MissionValidationError("frontier search pose must lie in known free space")
    distances = {start: 0}
    queue = deque([start])
    while queue:
        current = queue.popleft()
        for neighbor in _neighbors(grid, current, 4):
            if neighbor in distances or grid.value(neighbor) != 0:
                continue
            distances[neighbor] = distances[current] + 1
            queue.append(neighbor)
    return distances


def _clearance_m(
    grid: OccupancyGrid,
    cell: tuple[int, int],
    occupied: Sequence[tuple[int, int]],
) -> float:
    if not occupied:
        return math.inf
    return min(
        math.hypot(cell[0] - point[0], cell[1] - point[1])
        * grid.resolution_m
        for point in occupied
    )


def detect_frontiers(
    grid: OccupancyGrid,
    *,
    robot_x_m: float,
    robot_y_m: float,
    config: Optional[FrontierDetectionConfig] = None,
) -> tuple[FrontierCandidate, ...]:
    """Run deterministic two-wavefront frontier detection on reachable free space."""

    settings = config or FrontierDetectionConfig()
    robot_cell = grid.world_to_cell(robot_x_m, robot_y_m)
    reachable = _reachable_free(grid, robot_cell)
    frontier_cells = {
        neighbor
        for free_cell in reachable
        for neighbor in _neighbors(grid, free_cell, 4)
        if grid.value(neighbor) == -1
    }
    occupied = [
        (x, y)
        for y in range(grid.height)
        for x in range(grid.width)
        if grid.value((x, y)) == 100
    ]
    unseen = set(frontier_cells)
    candidates: list[FrontierCandidate] = []
    while unseen:
        seed = min(unseen, key=lambda cell: (cell[1], cell[0]))
        unseen.remove(seed)
        cluster = {seed}
        queue = deque([seed])
        while queue:
            current = queue.popleft()
            for neighbor in _neighbors(grid, current, settings.connectivity):
                if neighbor in unseen:
                    unseen.remove(neighbor)
                    cluster.add(neighbor)
                    queue.append(neighbor)
        if len(cluster) < settings.minimum_frontier_cells:
            continue
        approach_cells = {
            neighbor
            for frontier_cell in cluster
            for neighbor in _neighbors(grid, frontier_cell, 4)
            if neighbor in reachable
        }
        approach_with_clearance = [
            (cell, _clearance_m(grid, cell, occupied))
            for cell in approach_cells
        ]
        approach_with_clearance = [
            item
            for item in approach_with_clearance
            if item[1] >= settings.minimum_clearance_m
        ]
        if not approach_with_clearance:
            continue
        approach, clearance = min(
            approach_with_clearance,
            key=lambda item: (
                reachable[item[0]],
                -item[1],
                item[0][1],
                item[0][0],
            ),
        )
        cells = tuple(sorted(cluster, key=lambda cell: (cell[1], cell[0])))
        region = tuple(
            sorted(
                (item[0] for item in approach_with_clearance),
                key=lambda cell: (cell[1], cell[0]),
            )
        )
        signature_payload = {
            "map_id": grid.map_id,
            "frontier_cells": cells,
        }
        signature = hashlib.sha256(
            json.dumps(
                signature_payload, sort_keys=True, separators=(",", ":")
            ).encode("utf-8")
        ).hexdigest()
        approach_x, approach_y = grid.cell_to_world(approach)
        candidates.append(
            FrontierCandidate(
                signature=signature,
                map_id=grid.map_id,
                map_revision=grid.revision,
                cells=cells,
                approach_cells=region,
                approach_cell=approach,
                approach_x_m=approach_x,
                approach_y_m=approach_y,
                clearance_m=clearance,
                path_distance_m=reachable[approach] * grid.resolution_m,
                information_gain_m=len(cells) * grid.resolution_m,
            )
        )
    return tuple(
        sorted(
            candidates,
            key=lambda item: (
                item.path_distance_m,
                -item.information_gain_m,
                item.signature,
            ),
        )
    )


class FrontierState(str, Enum):
    ACTIVE = "active"
    VISITED = "visited"
    SUPPRESSED = "suppressed"
    FAILED = "failed"
    INVALIDATED = "invalidated"


@dataclass
class FrontierRegistry:
    states: dict[str, FrontierState] = field(default_factory=dict)
    candidates: dict[str, FrontierCandidate] = field(default_factory=dict)

    def update(
        self, candidates: Sequence[FrontierCandidate]
    ) -> tuple[FrontierCandidate, ...]:
        current = {candidate.signature for candidate in candidates}
        for signature, state in tuple(self.states.items()):
            if state is FrontierState.ACTIVE and signature not in current:
                self.states[signature] = FrontierState.INVALIDATED
        accepted: list[FrontierCandidate] = []
        for candidate in candidates:
            self.candidates[candidate.signature] = candidate
            state = self.states.get(candidate.signature)
            if state is None or state is FrontierState.INVALIDATED:
                self.states[candidate.signature] = FrontierState.ACTIVE
                accepted.append(candidate)
            elif state is FrontierState.ACTIVE:
                accepted.append(candidate)
        return tuple(accepted)

    def mark(self, signature: str, state: FrontierState) -> None:
        if signature not in self.candidates:
            raise MissionValidationError("unknown frontier signature")
        if state is FrontierState.ACTIVE:
            raise MissionValidationError("frontier state transitions cannot reactivate directly")
        self.states[signature] = state

    @property
    def exhausted(self) -> bool:
        return not any(state is FrontierState.ACTIVE for state in self.states.values())


@dataclass(frozen=True)
class FrontierGoal:
    generation: int
    frontier_signature: str
    map_id: str
    map_revision: str
    x_m: float
    y_m: float
    route_length_m: float
    ready_at_s: float
    planning_snapshot_valid: bool = True

    @classmethod
    def from_candidate(
        cls,
        candidate: FrontierCandidate,
        *,
        generation: int,
        ready_at_s: float,
        route_length_m: Optional[float] = None,
    ) -> "FrontierGoal":
        return cls(
            generation=generation,
            frontier_signature=candidate.signature,
            map_id=candidate.map_id,
            map_revision=candidate.map_revision,
            x_m=candidate.approach_x_m,
            y_m=candidate.approach_y_m,
            route_length_m=(
                candidate.path_distance_m
                if route_length_m is None
                else float(route_length_m)
            ),
            ready_at_s=float(ready_at_s),
        )

    def __post_init__(self) -> None:
        if self.generation <= 0 or not self.frontier_signature:
            raise MissionValidationError("frontier goal generation and signature are required")
        for name in ("x_m", "y_m", "route_length_m", "ready_at_s"):
            if not math.isfinite(float(getattr(self, name))):
                raise MissionValidationError(f"frontier goal {name} must be finite")
        if self.route_length_m < 0.0 or self.ready_at_s < 0.0:
            raise MissionValidationError("frontier goal route and ready time cannot be negative")


@dataclass(frozen=True)
class ReplayCommand:
    requested_linear_mps: float
    requested_angular_rad_s: float
    bridged_linear_mps: float
    bridged_angular_rad_s: float
    reason: str
    zero_required: bool


@dataclass(frozen=True)
class HierarchicalBridgeConfig:
    enabled: bool = False
    command_lease_s: float = 0.25
    max_linear_mps: float = 0.10
    max_angular_rad_s: float = 0.4
    clear_breakaway_linear_mps: float = 0.0
    clear_breakaway_angular_rad_s: float = 0.0
    reverse_escape_linear_mps: float = 0.0

    def __post_init__(self) -> None:
        if (
            not math.isfinite(self.command_lease_s)
            or not 0.0 < self.command_lease_s <= 0.50
        ):
            raise ValueError("hierarchical command lease must be in (0, 0.50] seconds")
        if not 0.0 < self.max_linear_mps <= 0.10:
            raise ValueError("hierarchical linear ceiling exceeds 0.10 m/s")
        if not 0.0 < self.max_angular_rad_s <= 0.4:
            raise ValueError("hierarchical angular ceiling exceeds 0.4 rad/s")
        if (
            not math.isfinite(self.clear_breakaway_linear_mps)
            or not 0.0
            <= self.clear_breakaway_linear_mps
            <= self.max_linear_mps
        ):
            raise ValueError(
                "hierarchical CLEAR breakaway speed must be between zero "
                "and the linear ceiling"
            )
        if (
            not math.isfinite(self.clear_breakaway_angular_rad_s)
            or not 0.0
            <= self.clear_breakaway_angular_rad_s
            <= self.max_angular_rad_s
        ):
            raise ValueError(
                "hierarchical CLEAR angular breakaway speed must be between "
                "zero and the angular ceiling"
            )
        if (
            not math.isfinite(self.reverse_escape_linear_mps)
            or not 0.0
            <= self.reverse_escape_linear_mps
            <= self.max_linear_mps
        ):
            raise ValueError(
                "hierarchical reverse escape speed must be between zero "
                "and the linear ceiling"
            )


class HierarchicalCommandBridge:
    """Receipt-time lease and bounds gate from private Nav2 commands to /cmd_vel."""

    def __init__(
        self, config: Optional[HierarchicalBridgeConfig] = None
    ) -> None:
        self.config = config or HierarchicalBridgeConfig()
        self._linear_mps = 0.0
        self._angular_rad_s = 0.0
        self._received_at_s: Optional[float] = None

    def accept(
        self, linear_mps: float, angular_rad_s: float, *, received_at_s: float
    ) -> None:
        values = (float(linear_mps), float(angular_rad_s), float(received_at_s))
        if not all(math.isfinite(value) for value in values):
            self._received_at_s = None
            return
        self._linear_mps, self._angular_rad_s, self._received_at_s = values

    def evaluate(
        self,
        *,
        now_s: float,
        goal_active: bool,
        mission_lease_valid: bool,
        motion_evidence_fresh: bool,
        collision_state: str,
        collision_reason: str = "",
        stop: bool = False,
        estop: bool = False,
        cancelled: bool = False,
    ) -> ReplayCommand:
        if not self.config.enabled:
            return ContinuousGoalFollowerReplay._zero_command(
                "hierarchical_mode_disabled"
            )
        if estop:
            return ContinuousGoalFollowerReplay._zero_command("estop")
        if stop:
            return ContinuousGoalFollowerReplay._zero_command("operator_stop")
        if cancelled:
            return ContinuousGoalFollowerReplay._zero_command("cancelled")
        if not goal_active or not mission_lease_valid:
            return ContinuousGoalFollowerReplay._zero_command("goal_lease_inactive")
        if not motion_evidence_fresh:
            return ContinuousGoalFollowerReplay._zero_command(
                "motion_evidence_stale"
            )
        normalized_collision_state = str(collision_state).upper()
        normalized_collision_reason = str(collision_reason).lower().strip()
        if normalized_collision_state not in {"CLEAR", "SLOW", "STOPPED"}:
            return ContinuousGoalFollowerReplay._zero_command("collision_veto")
        if (
            self._received_at_s is None
            or now_s < self._received_at_s
            or now_s - self._received_at_s > self.config.command_lease_s
        ):
            return ContinuousGoalFollowerReplay._zero_command(
                "nav2_command_stale"
            )
        linear = max(
            -self.config.max_linear_mps,
            min(self.config.max_linear_mps, self._linear_mps),
        )
        angular = max(
            -self.config.max_angular_rad_s,
            min(self.config.max_angular_rad_s, self._angular_rad_s),
        )
        reason = "clear"
        if normalized_collision_state == "STOPPED":
            if (
                self.config.reverse_escape_linear_mps <= 0.0
                or (linear == 0.0 and angular == 0.0)
            ):
                return ContinuousGoalFollowerReplay._zero_command(
                    "collision_veto"
                )
            # The downstream supervisor permits this only when its private
            # latch reason is front_stop and its live rear sector plus swept
            # reverse trajectory are clear. Every other STOPPED latch still
            # receives motor zero.
            return ReplayCommand(
                requested_linear_mps=self._linear_mps,
                requested_angular_rad_s=self._angular_rad_s,
                bridged_linear_mps=-self.config.reverse_escape_linear_mps,
                bridged_angular_rad_s=0.0,
                reason="stopped_reverse_escape_request",
                zero_required=False,
            )
        near_pure_turn = (
            abs(self._linear_mps) <= 0.01
            and abs(self._angular_rad_s) > 0.0
        )
        if (
            normalized_collision_state == "CLEAR"
            and near_pure_turn
            and abs(angular) < self.config.clear_breakaway_angular_rad_s
        ):
            linear = 0.0
            angular = math.copysign(
                self.config.clear_breakaway_angular_rad_s,
                angular,
            )
            reason = "clear_angular_breakaway_floor"
        elif (
            normalized_collision_state == "CLEAR"
            and not near_pure_turn
            and 0.0 < abs(linear) < self.config.clear_breakaway_linear_mps
        ):
            linear = math.copysign(
                self.config.clear_breakaway_linear_mps,
                linear,
            )
            reason = "clear_breakaway_floor"
        elif normalized_collision_state == "SLOW" and linear > 0.0:
            if (
                abs(angular) > 0.0
                and self.config.clear_breakaway_angular_rad_s > 0.0
            ):
                # Do not crawl a mixed arc toward a close obstacle at a
                # drivetrain-stalling speed. Remove its approach component
                # and pivot in Nav2's requested direction. The independent
                # downstream supervisor still evaluates the swept footprint
                # and can scale or veto this request.
                linear = 0.0
                angular = math.copysign(
                    max(
                        abs(angular),
                        self.config.clear_breakaway_angular_rad_s,
                    ),
                    angular,
                )
                reason = "slow_pivot_breakaway"
            elif self.config.reverse_escape_linear_mps > 0.0:
                # A straight approach cannot steer around the obstacle and a
                # twice-scaled crawl cannot break this drivetrain free.
                # Request the user's explicit "front obstacle -> back away"
                # behavior. The downstream supervisor evaluates the actual
                # reverse trajectory and keeps unconditional rear veto power.
                linear = -self.config.reverse_escape_linear_mps
                angular = 0.0
                reason = "slow_reverse_escape"
            else:
                reason = "supervisor_slow_passthrough"
        elif (
            normalized_collision_state == "SLOW"
            and self.config.reverse_escape_linear_mps > 0.0
            and normalized_collision_reason
            in {
                "forward_trajectory_blocked",
                "left_trajectory_blocked",
                "right_trajectory_blocked",
                "front_stop_reverse_escape",
            }
        ):
            # A blocked turn cannot clear its own swept footprint. Request
            # the same rear-checked escape as a blocked forward trajectory;
            # the downstream supervisor remains the final arbiter.
            linear = -self.config.reverse_escape_linear_mps
            angular = 0.0
            reason = "blocked_trajectory_reverse_escape"
        return ReplayCommand(
            requested_linear_mps=self._linear_mps,
            requested_angular_rad_s=self._angular_rad_s,
            bridged_linear_mps=linear,
            bridged_angular_rad_s=angular,
            reason=reason,
            zero_required=False,
        )


@dataclass(frozen=True)
class HandoffStep:
    state: str
    active_generation: Optional[int]
    queued_generations: tuple[int, ...]
    controller_session: int
    controller_active: bool
    command: ReplayCommand
    events: tuple[Mapping[str, Any], ...]


class ContinuousGoalFollowerReplay:
    """Deterministic replay of one Nav2 NavigateThroughPoses controller session."""

    def __init__(
        self,
        *,
        queue_depth: int = 3,
        max_linear_mps: float = 0.10,
        max_angular_rad_s: float = 0.4,
    ) -> None:
        if queue_depth not in {2, 3}:
            raise ValueError("hierarchical lookahead queue depth must be 2 or 3")
        if not 0.0 < max_linear_mps <= 0.10:
            raise ValueError("hierarchical linear ceiling exceeds 0.10 m/s")
        if not 0.0 < max_angular_rad_s <= 0.4:
            raise ValueError("hierarchical angular ceiling exceeds 0.4 rad/s")
        self.queue_depth = queue_depth
        self.max_linear_mps = max_linear_mps
        self.max_angular_rad_s = max_angular_rad_s
        self.state = "idle"
        self.active: Optional[FrontierGoal] = None
        self._pending: deque[FrontierGoal] = deque()
        self._queued: deque[FrontierGoal] = deque()
        self.controller_session = 0
        self.controller_active = False
        self.trace: list[dict[str, Any]] = []

    @property
    def motion_authority(self) -> bool:
        return False

    @property
    def physical_execution_enabled(self) -> bool:
        return False

    def start(
        self,
        goals: Sequence[FrontierGoal],
        *,
        now_s: float = 0.0,
    ) -> HandoffStep:
        if self.state != "idle":
            raise MissionValidationError("hierarchical replay is already started")
        if not goals:
            raise MissionValidationError("hierarchical replay requires at least one goal")
        ordered = list(goals[: self.queue_depth])
        if [goal.generation for goal in ordered] != sorted(
            goal.generation for goal in ordered
        ) or len({goal.generation for goal in ordered}) != len(ordered):
            raise MissionValidationError("frontier goal generations must be unique and ordered")
        first = ordered.pop(0)
        if first.ready_at_s > now_s or not first.planning_snapshot_valid:
            raise MissionValidationError("first frontier goal must be valid and ready")
        self.active = first
        self._pending.extend(ordered)
        self.state = "navigating"
        self.controller_session = 1
        self.controller_active = True
        event = self._event("navigate_through_poses_started", first, now_s)
        return self._step(self._moving_command(), (event,))

    def submit_prefetch(self, goal: FrontierGoal) -> None:
        """Add one model-selected goal without restarting the controller."""

        if self.state in {"idle", "terminal_safety"}:
            raise MissionValidationError(
                "prefetch requires an active hierarchical replay"
            )
        existing = [
            item
            for item in (self.active, *self._pending, *self._queued)
            if item is not None
        ]
        if len(existing) >= self.queue_depth:
            raise MissionValidationError("hierarchical lookahead queue is full")
        generations = [item.generation for item in existing]
        if generations and goal.generation <= max(generations):
            raise MissionValidationError(
                "prefetched goal generation must increase monotonically"
            )
        self._pending.append(goal)

    def discard_prefetch(
        self, generation: int, *, now_s: float, reason: str
    ) -> HandoffStep:
        """Discard one queued/pending result after deterministic revalidation."""

        removed: Optional[FrontierGoal] = None
        for queue in (self._pending, self._queued):
            retained: deque[FrontierGoal] = deque()
            while queue:
                candidate = queue.popleft()
                if removed is None and candidate.generation == generation:
                    removed = candidate
                else:
                    retained.append(candidate)
            queue.extend(retained)
        events: tuple[Mapping[str, Any], ...] = ()
        if removed is not None:
            event = self._event("prefetch_discarded", removed, now_s)
            event["reason"] = str(reason)
            events = (event,)
        command = (
            self._moving_command()
            if self.state == "navigating"
            else self._zero_command(self.state)
        )
        return self._step(command, events)

    def preempt_for_replan(
        self, *, now_s: float, reason: str
    ) -> HandoffStep:
        """Safely leave the current controller session for a semantic replan."""

        if self.state not in {"navigating", "wait_planning"}:
            raise MissionValidationError(
                "semantic replanning requires an active or waiting replay"
            )
        self._pending.clear()
        self._queued.clear()
        self.state = "wait_planning"
        self.controller_active = False
        event = self._event("semantic_replan_preempted", self.active, now_s)
        event["reason"] = str(reason)
        return self._step(self._zero_command("semantic_replan"), (event,))

    def advance(
        self,
        *,
        now_s: float,
        remaining_distance_m: float,
        requested_linear_mps: float = 0.10,
        requested_angular_rad_s: float = 0.0,
        collision_state: str = "CLEAR",
        stop: bool = False,
        estop: bool = False,
        cancelled: bool = False,
        motion_evidence_fresh: bool = True,
        planning_snapshot_valid: bool = True,
    ) -> HandoffStep:
        if self.state == "idle":
            raise MissionValidationError("hierarchical replay has not started")
        if not math.isfinite(remaining_distance_m) or remaining_distance_m < 0.0:
            raise MissionValidationError("remaining distance must be finite and non-negative")
        safety_reason = ""
        if estop:
            safety_reason = "estop"
        elif stop:
            safety_reason = "operator_stop"
        elif cancelled:
            safety_reason = "cancelled"
        if safety_reason:
            self.state = "terminal_safety"
            self.controller_active = False
            event = self._event(safety_reason, self.active, now_s)
            return self._step(self._zero_command(safety_reason), (event,))
        if not motion_evidence_fresh:
            # Fresh localization is required for motion, but a transient
            # freshness miss is a recoverable planning hold rather than an
            # operator/supervisor terminal.  The physical adapter cancels the
            # active Nav2 route on this zero-motion result.  A revalidated
            # prefetched goal may then start a new controller session once
            # motion evidence is fresh again.
            self.state = "wait_planning"
            self.controller_active = False
            event = self._event("motion_evidence_stale", self.active, now_s)
            return self._step(
                self._zero_command("motion_evidence_stale"), (event,)
            )
        if str(collision_state).upper() not in {"CLEAR", "SLOW"}:
            self.state = "terminal_safety"
            self.controller_active = False
            event = self._event("collision_veto", self.active, now_s)
            return self._step(
                self._zero_command("collision_veto"), (event,)
            )

        events: list[Mapping[str, Any]] = []
        while self._pending and self._pending[0].ready_at_s <= now_s:
            goal = self._pending.popleft()
            if not planning_snapshot_valid or not goal.planning_snapshot_valid:
                events.append(self._event("prefetch_discarded", goal, now_s))
                continue
            self._queued.append(goal)
            events.append(self._event("path_extended", goal, now_s))

        if self.state == "wait_planning":
            if not self._queued:
                return self._step(
                    self._zero_command("wait_planning"), tuple(events)
                )
            self.active = self._queued.popleft()
            self.state = "navigating"
            self.controller_session += 1
            self.controller_active = True
            events.append(self._event("planning_resume", self.active, now_s))
            return self._step(
                self._bounded_command(
                    requested_linear_mps, requested_angular_rad_s, collision_state
                ),
                tuple(events),
            )

        if self.state != "navigating":
            return self._step(
                self._zero_command(self.state), tuple(events)
            )

        if remaining_distance_m == 0.0:
            if self._queued:
                previous = self.active
                self.active = self._queued.popleft()
                events.append(
                    {
                        **self._event("atomic_handoff", self.active, now_s),
                        "from_generation": (
                            previous.generation if previous is not None else None
                        ),
                        "controller_exit": False,
                        "deliberate_zero": False,
                    }
                )
                return self._step(
                    self._bounded_command(
                        requested_linear_mps,
                        requested_angular_rad_s,
                        collision_state,
                    ),
                    tuple(events),
                )
            self.state = "wait_planning"
            self.controller_active = False
            events.append(self._event("wait_planning", self.active, now_s))
            return self._step(
                self._zero_command("wait_planning"), tuple(events)
            )

        return self._step(
            self._bounded_command(
                requested_linear_mps, requested_angular_rad_s, collision_state
            ),
            tuple(events),
        )

    def _bounded_command(
        self, linear: float, angular: float, collision_state: str
    ) -> ReplayCommand:
        if not math.isfinite(linear) or not math.isfinite(angular):
            return self._zero_command("malformed_nav2_command")
        bounded_linear = max(-self.max_linear_mps, min(self.max_linear_mps, linear))
        bounded_angular = max(
            -self.max_angular_rad_s, min(self.max_angular_rad_s, angular)
        )
        if str(collision_state).upper() == "SLOW" and bounded_linear > 0.0:
            bounded_linear *= 0.5
        return ReplayCommand(
            requested_linear_mps=float(linear),
            requested_angular_rad_s=float(angular),
            bridged_linear_mps=bounded_linear,
            bridged_angular_rad_s=bounded_angular,
            reason=(
                "supervisor_slowed"
                if str(collision_state).upper() == "SLOW"
                else "clear"
            ),
            zero_required=False,
        )

    def _moving_command(self) -> ReplayCommand:
        return self._bounded_command(self.max_linear_mps, 0.0, "CLEAR")

    @staticmethod
    def _zero_command(reason: str) -> ReplayCommand:
        return ReplayCommand(0.0, 0.0, 0.0, 0.0, reason, True)

    def _event(
        self, kind: str, goal: Optional[FrontierGoal], now_s: float
    ) -> dict[str, Any]:
        event = {
            "kind": kind,
            "at_s": float(now_s),
            "generation": goal.generation if goal is not None else None,
            "frontier_signature": (
                goal.frontier_signature if goal is not None else ""
            ),
        }
        self.trace.append(event)
        return event

    def _step(
        self, command: ReplayCommand, events: tuple[Mapping[str, Any], ...]
    ) -> HandoffStep:
        return HandoffStep(
            state=self.state,
            active_generation=(
                self.active.generation if self.active is not None else None
            ),
            queued_generations=tuple(goal.generation for goal in self._queued),
            controller_session=self.controller_session,
            controller_active=self.controller_active,
            command=command,
            events=events,
        )

    def evidence(self) -> dict[str, Any]:
        return {
            "schema": HANDOFF_SCHEMA,
            "navigator": "NavigateThroughPoses",
            "planner": "nav2_smac_planner/SmacPlanner2D",
            "controller": "dwb_core::DWBLocalPlanner",
            "state": self.state,
            "queue_depth": self.queue_depth,
            "controller_session": self.controller_session,
            "controller_active": self.controller_active,
            "active_generation": (
                self.active.generation if self.active is not None else None
            ),
            "queued_generations": [
                goal.generation for goal in self._queued
            ],
            "pending_generations": [
                goal.generation for goal in self._pending
            ],
            "private_command_topic": PRIVATE_NAV2_CMD_TOPIC,
            "bridge_output_topic": SUPERVISOR_REQUEST_TOPIC,
            "motor_topic": MOTOR_TOPIC,
            "cmd_vel_publishers": ["live_route_runner"],
            "cmd_vel_motor_publishers": ["lidar_collision_stop_supervisor"],
            "motion_authority": False,
            "physical_execution_enabled": False,
            "trace": list(self.trace),
        }


class HierarchicalReplayAdaptiveMissionExecutor(ReplayAdaptiveMissionExecutor):
    """Phase 1 frontier replay through the existing adaptive executor seam."""

    mode = "hierarchical-replay-simulation"

    def __init__(self, *, queue_depth: int = 3) -> None:
        super().__init__(motion_permitted=False)
        self.frontiers = FrontierRegistry()
        self.goal_follower = ContinuousGoalFollowerReplay(queue_depth=queue_depth)
        self._grid: Optional[OccupancyGrid] = None
        self._active_frontiers: tuple[FrontierCandidate, ...] = ()

    def update_map(
        self,
        grid: OccupancyGrid,
        *,
        robot_x_m: float,
        robot_y_m: float,
        config: Optional[FrontierDetectionConfig] = None,
    ) -> tuple[FrontierCandidate, ...]:
        detected = detect_frontiers(
            grid,
            robot_x_m=robot_x_m,
            robot_y_m=robot_y_m,
            config=config,
        )
        self._grid = grid
        self._active_frontiers = self.frontiers.update(detected)
        return self._active_frontiers

    def frontier_snapshot(self) -> dict[str, Any]:
        if self._grid is None:
            raise MissionValidationError("hierarchical replay has no occupancy map")
        return {
            "schema": FRONTIER_SCHEMA,
            "map_id": self._grid.map_id,
            "map_revision": self._grid.revision,
            "source": self._grid.source,
            "upstream_frontier_revision": UPSTREAM_FRONTIER_REVISION,
            "frontiers": [
                candidate.to_json_dict()
                for candidate in self._active_frontiers
            ],
            "exhausted": self.frontiers.exhausted,
            "motion_authority": False,
            "physical_execution_enabled": False,
        }

    def map_projection(self) -> dict[str, Any]:
        projection = super().map_projection()
        projection.update(
            {
                "fixture_only": False,
                "source": (
                    self._grid.source
                    if self._grid is not None
                    else "hierarchical-replay-no-map"
                ),
                "frontiers": [
                    candidate.to_json_dict()
                    for candidate in self._active_frontiers
                ],
                "nav2_handoff": self.goal_follower.evidence(),
            }
        )
        return projection
