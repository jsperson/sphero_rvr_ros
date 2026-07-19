"""Constrained plain-English translator for Mission API requests.

This module is intentionally deterministic and ROS-free.  It recognizes only the
canonical shoe-mapping vertical slice and emits either a validated Mission API
schema or a structured rejection.  It never publishes ROS topics, calls
``/cmd_vel``, talks to a browser, or exposes a generic ROS bridge.
"""

from __future__ import annotations

import re
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Mapping, Optional

from .mission_api import (
    CapabilitySet,
    MissionRequest,
    MissionValidationError,
    build_canonical_shoe_mapping_request,
    validate_mission_request,
)


class MissionLanguageRejectionReason(str, Enum):
    UNSUPPORTED_OBJECT = "unsupported_object"
    UNSUPPORTED_ACTION = "unsupported_action"
    DIRECT_MOVEMENT_REQUEST = "direct_movement_request"
    DIRECT_ROS_REQUEST = "direct_ros_request"
    PROMPT_INJECTION_OR_POLICY_BYPASS = "prompt_injection_or_policy_bypass"
    UNSUPPORTED_REQUEST = "unsupported_request"
    SCHEMA_VALIDATION_FAILED = "schema_validation_failed"


@dataclass(frozen=True)
class MissionLanguageRejection:
    reason: MissionLanguageRejectionReason
    message: str
    requires_confirmation: bool = True
    unsafe_surfaces_exposed: tuple[str, ...] = field(default_factory=tuple)
    detected_unsafe_surfaces: tuple[str, ...] = field(default_factory=tuple)

    def to_json_dict(self) -> dict[str, Any]:
        return {
            "reason": self.reason.value,
            "message": self.message,
            "requires_confirmation": self.requires_confirmation,
            "unsafe_surfaces_exposed": list(self.unsafe_surfaces_exposed),
            "detected_unsafe_surfaces": list(self.detected_unsafe_surfaces),
        }


@dataclass(frozen=True)
class MissionLanguageResult:
    request: Optional[MissionRequest] = None
    rejection: Optional[MissionLanguageRejection] = None

    @property
    def accepted(self) -> bool:
        return self.request is not None

    def __post_init__(self) -> None:
        if (self.request is None) == (self.rejection is None):
            raise MissionValidationError("translator must emit exactly one schema or one rejection")

    def to_json_dict(self) -> dict[str, Any]:
        return {
            "accepted": self.accepted,
            "schema": None if self.request is None else self.request.to_json_dict(),
            "rejection": None if self.rejection is None else self.rejection.to_json_dict(),
        }


_DIRECT_ROS_PATTERNS = (
    "/cmd_vel",
    "/cmd_vel_motor",
    "cmd_vel",
    "cmd_vel_motor",
    "ros topic",
    "publish",
    "generic ros bridge",
    "ros bridge",
    "raw motor",
    "teleop",
)
_PROMPT_BYPASS_PATTERNS = (
    "ignore previous",
    "ignore safety",
    "bypass safety",
    "override safety",
    "forget the rules",
    "system prompt",
    "developer message",
    "expose a ros bridge",
    "generic ros bridge",
)
_DIRECT_MOVEMENT_VERBS = (
    "drive",
    "move",
    "go",
    "roll",
    "turn",
    "spin",
    "reverse",
    "back up",
)
_DIRECT_MOVEMENT_QUALIFIERS = (
    "forward",
    "backward",
    "left",
    "right",
    "meter",
    "metre",
    "cm",
    "inch",
    "degree",
    "speed",
    "velocity",
)
_MAPPING_TERMS = ("map", "mapping", "mapped", "build a map", "create a map")
_ROOM_TERMS = ("room", "space", "area")
_SHOE_TERMS = ("shoe", "shoes", "footwear", "sneaker", "sneakers", "boot", "boots")
_DETECTION_TERMS = ("identify", "find", "detect", "mark", "label", "locate", "every", "all")
_UNSUPPORTED_OBJECT_TERMS = (
    "backpack",
    "backpacks",
    "bag",
    "bags",
    "person",
    "people",
    "pet",
    "pets",
    "cat",
    "dog",
    "keys",
    "cup",
    "cups",
    "chair",
    "chairs",
)
_UNSUPPORTED_ACTION_TERMS = (
    "pick up",
    "pickup",
    "bring",
    "carry",
    "push",
    "move the",
    "grab",
    "follow",
    "chase",
    "navigate to",
)


def translate_plain_english_mission(
    text: str,
    *,
    mission_id: str = "shoe-room-map",
    capabilities: Optional[CapabilitySet] = None,
) -> MissionLanguageResult:
    """Translate constrained plain English into Mission API schema or rejection.

    The only accepted intent is semantic shoe mapping for a room/space.  All
    unsupported, unsafe, or validation-failing requests return a structured
    rejection instead of a partial schema.
    """

    normalized = _normalize_text(text)
    detected_unsafe_surfaces = _detected_unsafe_surfaces(normalized)

    if _contains_any(normalized, _PROMPT_BYPASS_PATTERNS):
        return _reject(
            MissionLanguageRejectionReason.PROMPT_INJECTION_OR_POLICY_BYPASS,
            "Request attempts to bypass the constrained Mission API translator.",
            detected_unsafe_surfaces=detected_unsafe_surfaces,
        )
    if detected_unsafe_surfaces:
        return _reject(
            MissionLanguageRejectionReason.DIRECT_ROS_REQUEST,
            "Direct ROS topics, raw motor commands, teleop, and ROS bridges are not supported by this translator.",
            detected_unsafe_surfaces=detected_unsafe_surfaces,
        )
    if _is_direct_movement_request(normalized):
        return _reject(
            MissionLanguageRejectionReason.DIRECT_MOVEMENT_REQUEST,
            "Direct movement requests require a supervised Mission API command, not plain velocity control.",
        )
    if _contains_any(normalized, _UNSUPPORTED_ACTION_TERMS):
        return _reject(
            MissionLanguageRejectionReason.UNSUPPORTED_ACTION,
            "The translator only supports mapping the room and identifying shoes; manipulation/navigation actions are unsupported.",
        )
    if _mentions_unsupported_object(normalized):
        return _reject(
            MissionLanguageRejectionReason.UNSUPPORTED_OBJECT,
            "The translator only supports the shoe semantic label for this vertical slice.",
        )
    if not _is_supported_shoe_mapping_request(normalized):
        return _reject(
            MissionLanguageRejectionReason.UNSUPPORTED_REQUEST,
            "Request is outside the constrained shoe-mapping language.",
        )

    request = build_canonical_shoe_mapping_request(mission_id=mission_id)
    try:
        validate_mission_request(request, capabilities or CapabilitySet.all_enabled())
    except MissionValidationError as exc:
        return _reject(
            MissionLanguageRejectionReason.SCHEMA_VALIDATION_FAILED,
            f"Mission schema validation failed: {exc}",
        )
    return MissionLanguageResult(request=request)


def _normalize_text(text: str) -> str:
    if not isinstance(text, str):
        raise MissionValidationError("plain-English request must be text")
    normalized = re.sub(r"\s+", " ", text.strip().lower())
    if not normalized:
        raise MissionValidationError("plain-English request is required")
    return normalized


def _contains_any(text: str, needles: tuple[str, ...]) -> bool:
    return any(needle in text for needle in needles)


def _detected_unsafe_surfaces(text: str) -> tuple[str, ...]:
    return tuple(surface for surface in _DIRECT_ROS_PATTERNS if surface in text)


def _is_direct_movement_request(text: str) -> bool:
    return _contains_any(text, _DIRECT_MOVEMENT_VERBS) and _contains_any(text, _DIRECT_MOVEMENT_QUALIFIERS)


def _mentions_unsupported_object(text: str) -> bool:
    return _contains_any(text, _UNSUPPORTED_OBJECT_TERMS) and not _contains_any(text, _SHOE_TERMS)


def _is_supported_shoe_mapping_request(text: str) -> bool:
    return (
        _contains_any(text, _MAPPING_TERMS)
        and _contains_any(text, _SHOE_TERMS)
        and (_contains_any(text, _ROOM_TERMS) or _contains_any(text, _DETECTION_TERMS))
    )


def _reject(
    reason: MissionLanguageRejectionReason,
    message: str,
    *,
    detected_unsafe_surfaces: tuple[str, ...] = (),
) -> MissionLanguageResult:
    return MissionLanguageResult(
        rejection=MissionLanguageRejection(
            reason=reason,
            message=message,
            requires_confirmation=True,
            unsafe_surfaces_exposed=(),
            detected_unsafe_surfaces=detected_unsafe_surfaces,
        )
    )
