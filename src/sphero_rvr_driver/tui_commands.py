"""Slash-command parsing for the Sphero RVR terminal UI."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Union

CommandValue = Union[float, str]


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
