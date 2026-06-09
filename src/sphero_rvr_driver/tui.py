"""Curses terminal UI for controlling the Sphero RVR through ROS 2."""

from __future__ import annotations

import curses
import time
from dataclasses import dataclass, field
from typing import List, Optional

from .tui_commands import CommandParseError, TUICommand, parse_command
from .tui_keymap import KeyAction, map_key
from .tui_ros import RVRROSClient

DEFAULT_SPEED = 0.10
DEFAULT_TURN = 0.40
KEY_STOP_SECONDS = 0.75
KEY_REPEAT_SECONDS = 0.10


@dataclass
class TUIState:
    armed: bool = False
    speed: float = DEFAULT_SPEED
    turn: float = DEFAULT_TURN
    last_message: str = "Type /help. Use /arm before driving."
    history: List[str] = field(default_factory=list)

    def log(self, message: str) -> None:
        self.last_message = message
        self.history.append(message)
        self.history[:] = self.history[-6:]


HELP_TEXT = [
    "Controls: arrows/WASD drive when armed, space=/stop, e=/estop, q=/quit",
    "Commands: /arm, /disarm, /battery, /status, /speed <mps>, /turn <rad_s>",
    "          /stop, /estop, /clear-estop, /help, /quit",
]


class RVRTUI:
    def __init__(self, client: RVRROSClient):
        self.client = client
        self.state = TUIState()
        self._last_motion_at: Optional[float] = None
        self._last_motion_publish_at: Optional[float] = None
        self._active_linear_mps = 0.0
        self._active_angular_rad_s = 0.0
        self._motion_active = False
        self._quit = False

    def run(self) -> None:
        curses.wrapper(self._main)

    def _main(self, screen) -> None:
        curses.curs_set(0)
        screen.nodelay(True)
        screen.keypad(True)
        screen.timeout(50)
        self.client.start()
        try:
            while not self._quit:
                self._draw(screen)
                key_code = screen.getch()
                if key_code != -1:
                    self._handle_key(screen, key_code)
                self._maintain_motion()
        finally:
            self._safe_stop()
            self.client.close()

    def _handle_key(self, screen, key_code: int) -> None:
        key = self._key_name(screen, key_code)
        if key == "/":
            self._read_and_run_command(screen)
            return

        action = map_key(key, speed=self.state.speed, turn=self.state.turn)
        if action is None:
            return
        self._apply_key_action(action)

    def _apply_key_action(self, action: KeyAction) -> None:
        if action.kind == "motion":
            if not self.state.armed:
                self.state.log("Ignored drive key while disarmed. Use /arm.")
                return
            self._publish_motion(action.linear_mps, action.angular_rad_s)
            self._motion_active = True
            self._last_motion_at = time.monotonic()
            self.state.log(f"cmd_vel linear={action.linear_mps:.2f} angular={action.angular_rad_s:.2f}")
            return

        if action.kind == "command":
            self._run_command(TUICommand(name=action.command_name or ""))

    def _run_command(self, command: TUICommand) -> None:
        name = command.name
        try:
            if name == "help":
                for line in HELP_TEXT:
                    self.state.log(line)
            elif name == "arm":
                self.state.armed = True
                if command.value == "confirm":
                    self.state.log("Armed. Keyboard drive commands can start the RVR motors.")
                else:
                    self.state.log("Armed. Keyboard drive commands can start the RVR motors. Use /disarm to disable.")
            elif name == "disarm":
                self.state.armed = False
                self._publish_zero_velocity()
                self.state.log("Disarmed and published zero velocity.")
            elif name == "speed":
                if command.value is None:
                    raise ValueError("/speed requires a value")
                self.state.speed = float(command.value)
                self.state.log(f"Speed set to {self.state.speed:.2f} m/s")
            elif name == "turn":
                if command.value is None:
                    raise ValueError("/turn requires a value")
                self.state.turn = float(command.value)
                self.state.log(f"Turn speed set to {self.state.turn:.2f} rad/s")
            elif name == "battery":
                self.state.log(self._battery_text())
            elif name == "status":
                self.state.log(self._status_text())
            elif name == "stop":
                self.state.log(self.client.stop())
                self._motion_active = False
            elif name == "estop":
                self.state.armed = False
                self.state.log(self.client.estop())
                self._motion_active = False
            elif name == "clear-estop":
                self.state.log(self.client.clear_estop())
            elif name == "quit":
                self._quit = True
            else:
                self.state.log(f"Unknown command: /{name}")
        except Exception as exc:
            self.state.log(f"ERROR: {exc}")

    def _read_and_run_command(self, screen) -> None:
        curses.curs_set(1)
        command = "/"
        try:
            while True:
                self._draw(screen, command_prompt=command)
                code = screen.getch()
                if code in {10, 13}:
                    break
                if code in {27}:  # Escape
                    self.state.log("Command cancelled.")
                    return
                if code in {curses.KEY_BACKSPACE, 127, 8}:
                    command = command[:-1] or "/"
                    continue
                if 0 <= code < 256:
                    command += chr(code)
            try:
                parsed = parse_command(command)
            except CommandParseError as exc:
                self.state.log(f"ERROR: {exc}")
                return
            self._run_command(parsed)
        finally:
            curses.curs_set(0)

    def _maintain_motion(self) -> None:
        if not self._motion_active or self._last_motion_at is None:
            return
        now = time.monotonic()
        if now - self._last_motion_at > KEY_STOP_SECONDS:
            self.client.publish_velocity(0.0, 0.0)
            self._motion_active = False
            self._last_motion_publish_at = None
            self.state.log("Stopped after key timeout.")
            return
        if self._last_motion_publish_at is None or now - self._last_motion_publish_at >= KEY_REPEAT_SECONDS:
            self._publish_motion(self._active_linear_mps, self._active_angular_rad_s)

    def _safe_stop(self) -> None:
        try:
            self._publish_zero_velocity()
        except Exception:
            pass

    def _publish_zero_velocity(self) -> None:
        try:
            self.client.publish_velocity(0.0, 0.0)
        finally:
            self._motion_active = False
            self._last_motion_publish_at = None

    def _publish_motion(self, linear_mps: float, angular_rad_s: float) -> None:
        self._active_linear_mps = linear_mps
        self._active_angular_rad_s = angular_rad_s
        self.client.publish_velocity(linear_mps, angular_rad_s)
        self._last_motion_publish_at = time.monotonic()

    def _draw(self, screen, command_prompt: str = "> ") -> None:
        screen.erase()
        status = self.client.status
        battery = self._battery_text()
        rows = [
            "Sphero RVR Console",
            "────────────────────────────────────────",
            f"Driver: {status.diagnostic_message}    Connected: {status.connected}",
            f"Battery: {battery}",
            f"Armed: {self.state.armed}    Estop: {status.emergency_stopped}",
            f"Speed: {self.state.speed:.2f} m/s    Turn: {self.state.turn:.2f} rad/s",
            "",
            "↑/w forward  ↓/s reverse  ←/a left  →/d right  space stop  e estop  q quit",
            "Type /help for slash commands.",
            "",
        ]
        rows.extend(self.state.history[-6:])
        rows.extend(["", command_prompt])
        height, width = screen.getmaxyx()
        for idx, row in enumerate(rows[: height - 1]):
            screen.addnstr(idx, 0, row, max(0, width - 1))
        screen.refresh()

    def _battery_text(self) -> str:
        status = self.client.status
        parts = []
        if status.battery_percentage is not None:
            parts.append(f"{status.battery_percentage * 100:.0f}%")
        if status.battery_voltage is not None:
            parts.append(f"{status.battery_voltage:.2f} V")
        return " / ".join(parts) if parts else "waiting"

    def _status_text(self) -> str:
        status = self.client.status
        return (
            f"connected={status.connected} estop={status.emergency_stopped} "
            f"battery={self._battery_text()} diag='{status.diagnostic_message}'"
        )

    @staticmethod
    def _key_name(screen, key_code: int) -> str:
        if key_code >= 0:
            name = curses.keyname(key_code).decode(errors="ignore")
            if name.startswith("^J"):
                return "\n"
            if len(name) == 1:
                return name
            return name
        return ""


def main() -> None:
    client = RVRROSClient()
    RVRTUI(client).run()


if __name__ == "__main__":
    main()
