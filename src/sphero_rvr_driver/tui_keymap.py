"""Keyboard mapping for the Sphero RVR terminal UI."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class KeyAction:
    kind: str
    linear_mps: float = 0.0
    angular_rad_s: float = 0.0
    command_name: Optional[str] = None

    @classmethod
    def motion(cls, linear_mps: float, angular_rad_s: float) -> "KeyAction":
        return cls(kind="motion", linear_mps=linear_mps, angular_rad_s=angular_rad_s)

    @classmethod
    def command(cls, command: str) -> "KeyAction":
        return cls(kind="command", command_name=command)


_FORWARD = {"KEY_UP", "w", "W"}
_REVERSE = {"KEY_DOWN", "s", "S"}
_LEFT = {"KEY_LEFT", "a", "A"}
_RIGHT = {"KEY_RIGHT", "d", "D"}
_STOP = {" ", "KEY_BACKSPACE"}
_ESTOP = {"e", "E"}
_QUIT = {"q", "Q"}


def map_key(key: str, *, speed: float, turn: float) -> Optional[KeyAction]:
    if key in _FORWARD:
        return KeyAction.motion(speed, 0.0)
    if key in _REVERSE:
        return KeyAction.motion(-speed, 0.0)
    if key in _LEFT:
        return KeyAction.motion(0.0, turn)
    if key in _RIGHT:
        return KeyAction.motion(0.0, -turn)
    if key in _STOP:
        return KeyAction.command("stop")
    if key in _ESTOP:
        return KeyAction.command("estop")
    if key in _QUIT:
        return KeyAction.command("quit")
    return None
