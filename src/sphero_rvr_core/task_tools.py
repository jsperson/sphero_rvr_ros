"""Pure logic for the NL task surface: envelope clamping, semantic query, results.

Track 2 v1 exposes three tools -- `goto`, `observe`, `query_semantic_map` -- over
capabilities that are already hardware-validated. This module holds everything about
them that can be decided without ROS: what a caller is allowed to ask for, how a
semantic question is answered from a map snapshot, and what a result looks like on
the wire. The ROS shell (`sphero_rvr_driver/task_node.py`) does the talking.

**The envelope is the point.** The caller -- eventually a language model -- proposes;
this code clamps. That split is lifted from the culled `PromptDriveLimits`
(`c5e87d2~1:src/sphero_rvr_driver/prompt_drive.py:43`), which got the shape right:
limits are a frozen dataclass validated at construction against ceilings the caller
cannot raise, so an out-of-range request is a rejection at the boundary rather than a
surprise at the motors. What changed is the units. The old envelope bounded
dead-reckoned segments (`move_distance` / `turn_angle`) against a mission API that no
longer exists; this one bounds a MAP-FRAME GOAL POSE handed to Nav2, because that is
the live binding today (`NavigateToPose`, verified at HEAD). Distance is measured
from the robot's current pose, so "how far may it be sent" is a question about
displacement, not about a route.

**What this module deliberately cannot do:** it holds no velocity, no duty, no
timing. Motion is Nav2's, and beneath Nav2 the collision/STOP/ESTOP supervisor
remains the sole `/cmd_vel_motor` publisher. A tool surface that could set a speed
would be a second controller.
"""

from dataclasses import dataclass
import json
import math
from typing import Any, Optional


# Ceilings the caller cannot raise. A GoalEnvelope may be constructed narrower than
# these for a given deployment, never wider -- the same "intentionally widenable, but
# not beyond runtime ceilings" rule the old envelope tests pinned.
MAX_GOAL_DISTANCE_CEILING_M = 10.0
MAX_QUERY_RADIUS_CEILING_M = 20.0


@dataclass(frozen=True)
class GoalEnvelope:
    """Trusted bounds on a `goto` request. The caller proposes; this clamps.

    `max_goal_distance_m` is straight-line displacement from the robot's CURRENT pose
    to the requested goal -- not path length, which only the planner knows. It exists
    so a mis-typed or hallucinated coordinate cannot send the rover across the
    building; it is not a safety device (the supervisor is) but a sanity boundary.
    """

    max_goal_distance_m: float = 5.0
    max_query_radius_m: float = 10.0

    def __post_init__(self) -> None:
        for name in ("max_goal_distance_m", "max_query_radius_m"):
            value = float(getattr(self, name))
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be positive and finite")
        if self.max_goal_distance_m > MAX_GOAL_DISTANCE_CEILING_M:
            raise ValueError(
                f"max_goal_distance_m exceeds the {MAX_GOAL_DISTANCE_CEILING_M} m ceiling"
            )
        if self.max_query_radius_m > MAX_QUERY_RADIUS_CEILING_M:
            raise ValueError(
                f"max_query_radius_m exceeds the {MAX_QUERY_RADIUS_CEILING_M} m ceiling"
            )

    def to_json_dict(self) -> dict[str, Any]:
        return {
            "max_goal_distance_m": self.max_goal_distance_m,
            "max_query_radius_m": self.max_query_radius_m,
        }


class EnvelopeError(ValueError):
    """A request the envelope refuses. Carries a reason meant to be read by whoever
    (or whatever) made the request, so a rejected tool call can be corrected."""


def validate_goal(x, y, robot_xy, envelope: GoalEnvelope) -> tuple[float, float]:
    """Return the goal as floats, or raise EnvelopeError with a usable reason.

    Fails CLOSED on every uncertainty: non-numeric input, NaN/inf, an unknown robot
    pose (we cannot bound displacement we cannot measure), or a goal beyond the
    envelope. Refusing an unlocatable robot is deliberate -- the alternative is
    accepting an unbounded goal precisely when localization is broken.
    """
    try:
        gx, gy = float(x), float(y)
    except (TypeError, ValueError):
        raise EnvelopeError(f"goal coordinates must be numbers, got ({x!r}, {y!r})")
    if not (math.isfinite(gx) and math.isfinite(gy)):
        raise EnvelopeError(f"goal coordinates must be finite, got ({gx}, {gy})")
    if robot_xy is None:
        raise EnvelopeError(
            "robot pose unknown (no map->base_link transform) — cannot bound the goal"
        )
    distance = math.hypot(gx - float(robot_xy[0]), gy - float(robot_xy[1]))
    if distance > envelope.max_goal_distance_m:
        raise EnvelopeError(
            f"goal is {distance:.2f} m away, beyond the "
            f"{envelope.max_goal_distance_m:.2f} m envelope"
        )
    return gx, gy


def validate_query(label, near, radius_m, envelope: GoalEnvelope):
    """Normalize a semantic query, or raise EnvelopeError.

    Unlike `goto`, an unbounded query is harmless -- it moves nothing -- so `near`
    and `radius_m` are optional and only checked when supplied. An empty label means
    "everything", which is how `SemanticMap.query` already reads `label=None`.
    """
    text = None if label is None else str(label).strip()
    if text == "":
        text = None
    if near is None and radius_m is None:
        return text, None, None
    if (near is None) != (radius_m is None):
        raise EnvelopeError("near and radius_m must be given together")
    try:
        nx, ny, r = float(near[0]), float(near[1]), float(radius_m)
    except (TypeError, ValueError, IndexError):
        raise EnvelopeError("near must be (x, y) numbers and radius_m a number")
    if not (math.isfinite(nx) and math.isfinite(ny) and math.isfinite(r)):
        raise EnvelopeError("near and radius_m must be finite")
    if r <= 0.0:
        raise EnvelopeError("radius_m must be positive")
    if r > envelope.max_query_radius_m:
        raise EnvelopeError(
            f"radius_m {r:.2f} exceeds the {envelope.max_query_radius_m:.2f} m envelope"
        )
    return text, (nx, ny), r


def query_semantic_objects(map_json, label=None, near=None, radius_m=None,
                           min_confidence=None, envelope: Optional[GoalEnvelope] = None):
    """Answer a semantic question from a `/semantic_map/objects` JSON snapshot.

    Thin by design: `SemanticMap` already does fuzzy label matching, proximity
    filtering and nearest-first ordering, and it is tested. This adds the envelope
    check, tolerates a missing/garbage snapshot (answering "nothing known" rather
    than raising -- a query before the first observation is a normal state, not an
    error), and returns plain dicts ready to serialize.
    """
    from sphero_rvr_core.semantic_map import SemanticMap

    env = envelope or GoalEnvelope()
    text, near_xy, radius = validate_query(label, near, radius_m, env)
    if not map_json:
        return []
    try:
        smap = SemanticMap.from_json(map_json)
    except Exception:
        return []
    found = smap.query(
        label=text, near=near_xy, radius_m=radius, min_confidence=min_confidence
    )
    return [o.to_dict() for o in found]


def tool_result(ok: bool, tool: str, message: str = "", **fields) -> str:
    """The one result shape every tool returns: a JSON string.

    `ok` plus a typed `outcome` rather than prose, for the same reason the mission
    report carries an outcome FIELD -- a caller (or a model) should never have to
    parse a sentence to learn whether the robot did the thing. Services return this
    in `Trigger.Response.message`, which is a string, so JSON-in-a-string is the
    honest encoding rather than a bespoke message type.
    """
    payload = {"ok": bool(ok), "tool": tool}
    if message:
        payload["message"] = message
    payload.update(fields)
    return json.dumps(payload, sort_keys=True)
