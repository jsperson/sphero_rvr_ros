"""Slash-command parsing for the Sphero RVR terminal UI."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Union

NUDGE_MIN_DISTANCE_M = 0.02
NUDGE_MAX_DISTANCE_M = 0.1524
NUDGE_INCHES_TO_METERS = 0.0254


@dataclass(frozen=True)
class NudgeCommand:
    direction: str
    distance_m: float
    confirmed: bool = False


CommandValue = Union[float, str, NudgeCommand]


class CommandParseError(ValueError):
    """Raised when a TUI slash command is unknown or malformed."""


@dataclass(frozen=True)
class TUICommand:
    name: str
    value: Optional[CommandValue] = None


_NO_ARG_COMMANDS = {
    "battery",
    "status",
    "stop",
    "estop",
    "clear-estop",
    "disarm",
    "help",
    "quit",
}


ALIASES = {
    "clear_estop": "clear-estop",
    "clear": "clear-estop",
    "exit": "quit",
    "q": "quit",
}


def parse_command(raw: str) -> TUICommand:
    text = raw.strip()
    if not text.startswith("/"):
        raise CommandParseError("commands must start with '/'")

    parts = text[1:].split()
    if not parts:
        raise CommandParseError("empty command")

    name = ALIASES.get(parts[0].lower(), parts[0].lower())
    args = parts[1:]

    if name == "clear-estop":
        if not args or args == ["confirm"]:
            return TUICommand(name="clear-estop")
        raise CommandParseError("use /clear-estop or /clear-estop confirm")

    if name in _NO_ARG_COMMANDS:
        if args:
            raise CommandParseError(f"/{name} does not take arguments")
        return TUICommand(name=name)

    if name == "arm":
        if not args:
            return TUICommand(name="arm")
        if args == ["confirm"]:
            return TUICommand(name="arm", value="confirm")
        raise CommandParseError("use /arm or /arm confirm")

    if name == "lidar":
        if args == ["start"]:
            return TUICommand(name="lidar", value="start")
        if args == ["stop"]:
            return TUICommand(name="lidar", value="stop")
        raise CommandParseError("use /lidar start or /lidar stop")

    if name == "mapping":
        if args == ["start"]:
            return TUICommand(name="mapping", value="start")
        if args == ["stop"]:
            return TUICommand(name="mapping", value="stop")
        if args == ["status"]:
            return TUICommand(name="mapping", value="status")
        if args == ["full"]:
            return TUICommand(name="mapping", value="full")
        if args == ["full", "confirm"]:
            return TUICommand(name="mapping", value="full-confirm")
        raise CommandParseError("use /mapping start, /mapping stop, /mapping status, /mapping full, or /mapping full confirm")

    if name == "map":
        if len(args) >= 2 and args[0] == "save":
            return TUICommand(name="map", value=" ".join(args[1:]))
        raise CommandParseError("use /map save <name>")

    if name == "nudge":
        return TUICommand(name="nudge", value=_parse_nudge_args(args))

    if name in {"speed", "turn"}:
        if len(args) != 1:
            raise CommandParseError(f"/{name} requires one numeric argument")
        try:
            value = float(args[0])
        except ValueError as exc:
            raise CommandParseError(f"/{name} requires a numeric argument") from exc
        if value < 0:
            raise CommandParseError(f"/{name} must be non-negative")
        return TUICommand(name=name, value=value)

    raise CommandParseError(f"unknown command: /{name}")


def _parse_nudge_args(args: list[str]) -> NudgeCommand:
    if len(args) not in {2, 3, 4}:
        raise CommandParseError("use /nudge forward <distance> confirm or /nudge back <distance> confirm")
    direction = args[0].lower()
    if direction not in {"forward", "back"}:
        raise CommandParseError("/nudge direction must be forward or back")

    confirmed = False
    distance_parts = args[1:]
    if distance_parts[-1].lower() == "confirm":
        confirmed = True
        distance_parts = distance_parts[:-1]
    if not distance_parts:
        raise CommandParseError("/nudge requires a distance")

    distance_m = _parse_distance_m(distance_parts)
    if distance_m < NUDGE_MIN_DISTANCE_M:
        raise CommandParseError("/nudge distance must be at least 0.02 m")
    if distance_m > NUDGE_MAX_DISTANCE_M:
        raise CommandParseError("/nudge distance is capped at 6 inches")
    return NudgeCommand(direction=direction, distance_m=distance_m, confirmed=confirmed)


def _parse_distance_m(parts: list[str]) -> float:
    if len(parts) == 1:
        token = parts[0].lower()
        for suffix in ("inches", "inch", "in"):
            if token.endswith(suffix):
                value_text = token[: -len(suffix)]
                return _parse_float(value_text, "distance") * NUDGE_INCHES_TO_METERS
        for suffix in ("meters", "meter", "m"):
            if token.endswith(suffix):
                value_text = token[: -len(suffix)]
                return _parse_float(value_text, "distance")
        return _parse_float(token, "distance")

    if len(parts) == 2:
        value = _parse_float(parts[0], "distance")
        unit = parts[1].lower()
        if unit in {"in", "inch", "inches"}:
            return value * NUDGE_INCHES_TO_METERS
        if unit in {"m", "meter", "meters"}:
            return value
    raise CommandParseError("/nudge distance must be meters or inches")


def _parse_float(value_text: str, label: str) -> float:
    try:
        return float(value_text)
    except ValueError as exc:
        raise CommandParseError(f"/nudge requires a numeric {label}") from exc
