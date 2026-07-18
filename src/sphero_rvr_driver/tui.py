"""Curses terminal UI for controlling the Sphero RVR through ROS 2."""

from __future__ import annotations

import argparse
import curses
import math
import textwrap
import time
from dataclasses import dataclass, field
from typing import List, Optional

from .tui_commands import CommandParseError, NudgeCommand, TUICommand, parse_command
from .tui_keymap import KeyAction, map_key
from .tui_launch import LaunchManager, MappingMode, MapSaver
from .tui_ros import DryRunRVRClient, RVRROSClient, collision_stop_allows_motion, format_status_lines, freshness_text

DEFAULT_SPEED = 0.10
DEFAULT_TURN = 0.40
NUDGE_LINEAR_MPS = 0.10
# Measured with the final lightweight rack on 2026-07-16: five accepted
# forward runs traveled 0.3048 m in 10.0 s at the reliable duty threshold.
NUDGE_CALIBRATED_METERS_PER_SECOND = 0.03048
NUDGE_ODOM_COUNTS_PER_METER = 4337.768
KEY_STOP_SECONDS = 0.30
TURN_TAP_STOP_SECONDS = 0.14
TURN_HOLD_STOP_SECONDS = 0.45
TURN_HOLD_WINDOW_SECONDS = 0.75
TURN_TAP_RAD_S = DEFAULT_TURN
KEY_REPEAT_SECONDS = 0.08


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
    "          /lidar start|stop, /mapping start|stop|status, /mapping full confirm",
    "          /map save <name> saves to ~/maps/<safe-name>.yaml/.pgm",
    "          /nudge forward|back <distance> confirm (distance max 6in; accepts m/in)",
    "          /stop, /estop, /clear-estop, /help, /quit",
    "MOTOR-CAPABLE: /mapping full confirm; /nudge ... confirm. WARNING: this can start the RVR motors",
]


class RVRTUI:
    def __init__(self, client, launch_manager: LaunchManager | None = None, map_saver: MapSaver | None = None):
        self.client = client
        dry_run = getattr(getattr(client, "status", None), "mode", "live") == "dry-run"
        self.launch_manager = launch_manager if launch_manager is not None else LaunchManager(
            dry_run=dry_run
        )
        self.map_saver = map_saver if map_saver is not None else MapSaver(dry_run=dry_run)
        self.state = TUIState()
        self._last_motion_at: Optional[float] = None
        self._last_motion_publish_at: Optional[float] = None
        self._active_linear_mps = 0.0
        self._active_angular_rad_s = 0.0
        self._active_stop_seconds = KEY_STOP_SECONDS
        self._active_repeat_enabled = True
        self._disarm_after_motion = False
        self._last_turn_signature: Optional[float] = None
        self._last_turn_key_at: Optional[float] = None
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
            self.launch_manager.stop()
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
            if self._motion_active and self._disarm_after_motion:
                self.state.log("Ignored drive key during active nudge. Use STOP, ESTOP, or /disarm.")
                return
            if not self.state.armed:
                self.state.log("Ignored drive key while disarmed. Use /arm.")
                return
            now = time.monotonic()
            linear_mps, angular_rad_s = self._motion_for_key_action(action, now)
            try:
                self._publish_motion(linear_mps, angular_rad_s)
            except Exception as exc:
                self._motion_active = False
                self._last_motion_publish_at = None
                self.state.armed = False
                self.state.log(f"Motion rejected and disarmed: {exc}")
                return
            self._motion_active = True
            self._last_motion_at = now
            self._configure_motion_mode(linear_mps, angular_rad_s, now)
            self.state.log(self._motion_log_text(linear_mps, angular_rad_s))
            return

        if action.kind == "command":
            self._run_command(TUICommand(name=action.command_name or ""))

    def _run_command(self, command: TUICommand) -> None:
        name = command.name
        if self._motion_active and self._disarm_after_motion and name not in {
            "stop",
            "estop",
            "disarm",
            "quit",
            "help",
            "status",
            "battery",
        }:
            self.state.log(
                f"Command /{name} rejected during active nudge. "
                "Use STOP, ESTOP, /disarm, or /quit."
            )
            return
        try:
            if name == "help":
                for line in HELP_TEXT:
                    self.state.log(line)
            elif name == "arm":
                if not collision_stop_allows_motion(self.client.status):
                    raise RuntimeError("collision stop is not CLEAR/SLOW and fresh; refusing to arm")
                self._enable_motion_publisher_if_supported()
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
            elif name == "lidar":
                self._run_lidar_command(command)
            elif name == "mapping":
                self._run_mapping_command(command)
            elif name == "map":
                self._run_map_command(command)
            elif name == "nudge":
                self._run_nudge_command(command)
            elif name == "stop":
                self.state.armed = False
                self._motion_active = False
                self._disarm_after_motion = False
                try:
                    message = self.client.stop()
                finally:
                    self._safe_stop()
                self.state.log(message)
            elif name == "estop":
                self.state.armed = False
                self._motion_active = False
                self._disarm_after_motion = False
                try:
                    message = self.client.estop()
                finally:
                    self._safe_stop()
                self.state.log(message)
            elif name == "clear-estop":
                self.state.log(self.client.clear_estop())
            elif name == "quit":
                self.state.armed = False
                self._safe_stop()
                self._quit = True
            else:
                self.state.log(f"Unknown command: /{name}")
        except Exception as exc:
            self.state.log(f"ERROR: {exc}")

    def _run_lidar_command(self, command: TUICommand) -> None:
        if command.value == "start":
            state = self.launch_manager.start_lidar()
        elif command.value == "stop":
            state = self.launch_manager.stop()
        else:
            raise ValueError("use /lidar start or /lidar stop")
        self.state.log(state.message)

    def _run_mapping_command(self, command: TUICommand) -> None:
        if command.value == "start":
            state = self.launch_manager.start_mapping(start_rvr=False)
            self.state.log(state.message)
            return
        if command.value == "stop":
            self.state.armed = False
            self._publish_zero_velocity()
            state = self.launch_manager.stop()
            self._disable_motion_publisher_if_supported()
            self.state.log(state.message)
            return
        if command.value == "status":
            self.state.log(self._launch_status_text())
            return
        if command.value == "full":
            self.state.armed = False
            self.state.log("WARNING: this can start the RVR motors. Run /mapping full confirm to continue.")
            return
        if command.value == "full-confirm":
            self.state.armed = False
            state = self.launch_manager.start_mapping(start_rvr=True)
            if not self.launch_manager.dry_run and state.mode is MappingMode.MOTOR_CAPABLE:
                self._enable_motion_publisher_if_supported()
            self.state.log(f"WARNING: this can start the RVR motors. {state.message}")
            return
        raise ValueError("use /mapping start, /mapping stop, /mapping status, or /mapping full confirm")

    def _run_map_command(self, command: TUICommand) -> None:
        if not isinstance(command.value, str):
            raise ValueError("use /map save <name>")
        self.state.log(self.map_saver.save(command.value).message)

    def _run_nudge_command(self, command: TUICommand) -> None:
        if not isinstance(command.value, NudgeCommand):
            raise ValueError("use /nudge forward <distance> confirm or /nudge back <distance> confirm")
        nudge = command.value
        if not math.isfinite(nudge.distance_m):
            raise ValueError("/nudge distance must be finite")
        if not nudge.confirmed:
            self.state.log(
                "WARNING: this can start the RVR motors. "
                f"Run /nudge {nudge.direction} {nudge.distance_m:.2f} confirm to continue."
            )
            return
        if not self.state.armed:
            self.state.log("Nudge rejected while disarmed. Use /arm confirm first.")
            return
        if not self._velocity_sink_available():
            self.state.log("Nudge rejected: velocity publisher not enabled.")
            return

        velocity = NUDGE_LINEAR_MPS if nudge.direction == "forward" else -NUDGE_LINEAR_MPS
        duration = nudge.distance_m / NUDGE_CALIBRATED_METERS_PER_SECOND
        expected_counts = nudge.distance_m * NUDGE_ODOM_COUNTS_PER_METER
        mode_prefix = "DRY-RUN nudge" if getattr(self.client.status, "mode", "live") == "dry-run" else "nudge"

        try:
            self._publish_motion(velocity, 0.0)
        except Exception:
            self._safe_stop()
            self._motion_active = False
            self._last_motion_publish_at = None
            self._disarm_after_motion = False
            self.state.armed = False
            raise

        self._motion_active = True
        self._last_motion_at = time.monotonic()
        self._active_stop_seconds = duration
        self._active_repeat_enabled = True
        self._disarm_after_motion = True

        self.state.log(
            f"{mode_prefix} {nudge.direction} distance={nudge.distance_m:.2f} m "
            f"velocity={abs(velocity):.2f} m/s duration={duration:.2f}s "
            f"expected_encoder_counts={expected_counts:.1f}; STOP/ESTOP remain available"
        )

    def _read_and_run_command(self, screen) -> None:
        curses.curs_set(1)
        command = "/"
        try:
            while True:
                self._draw(screen, command_prompt=command)
                code = screen.getch()
                self._maintain_motion()
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


    def _motion_for_key_action(self, action: KeyAction, now: float) -> tuple[float, float]:
        linear_mps = action.linear_mps
        angular_rad_s = action.angular_rad_s
        is_turn_only = abs(linear_mps) < 1e-9 and abs(angular_rad_s) > 1e-9
        if not is_turn_only:
            return linear_mps, angular_rad_s

        signature = 1.0 if angular_rad_s > 0 else -1.0
        held = (
            self._last_turn_signature == signature
            and self._last_turn_key_at is not None
            and now - self._last_turn_key_at <= TURN_HOLD_WINDOW_SECONDS
        )
        # Turning under the top rack needs full breakaway torque immediately.
        # Tap distance is controlled by the short timeout, not by lowering power
        # into the static-friction deadband.
        return 0.0, signature * min(abs(angular_rad_s), TURN_TAP_RAD_S)

    def _maintain_motion(self) -> None:
        if not self._motion_active or self._last_motion_at is None:
            return
        now = time.monotonic()
        if now - self._last_motion_at > self._active_stop_seconds:
            disarm_after_motion = self._disarm_after_motion
            self._publish_zero_velocity()
            self._motion_active = False
            self._last_motion_publish_at = None
            if disarm_after_motion:
                self.state.armed = False
                self.state.log("Nudge complete; stopped and disarmed.")
            else:
                self.state.log("Stopped after key timeout.")
            return
        if not self._active_repeat_enabled:
            return
        if self._last_motion_publish_at is None or now - self._last_motion_publish_at >= KEY_REPEAT_SECONDS:
            try:
                self._publish_motion(self._active_linear_mps, self._active_angular_rad_s)
            except Exception as exc:
                self._safe_stop()
                self.state.armed = False
                self.state.log(f"Motion repeat failed and disarmed: {exc}")

    def _safe_stop(self) -> None:
        try:
            self._publish_zero_velocity()
        except Exception:
            pass

    def _publish_zero_velocity(self) -> None:
        try:
            if self._velocity_sink_available():
                self.client.publish_velocity(0.0, 0.0)
        finally:
            self._motion_active = False
            self._last_motion_publish_at = None
            self._disarm_after_motion = False

    def _publish_motion(self, linear_mps: float, angular_rad_s: float) -> None:
        self._active_linear_mps = linear_mps
        self._active_angular_rad_s = angular_rad_s
        self.client.publish_velocity(linear_mps, angular_rad_s)
        self._last_motion_publish_at = time.monotonic()

    def _velocity_sink_available(self) -> bool:
        enabled = getattr(self.client, "velocity_publisher_enabled", None)
        if enabled is None:
            return True
        return bool(enabled)

    def _enable_motion_publisher_if_supported(self) -> None:
        enable = getattr(self.client, "enable_velocity_publisher", None)
        if callable(enable):
            enable()

    def _disable_motion_publisher_if_supported(self) -> None:
        disable = getattr(self.client, "disable_velocity_publisher", None)
        if callable(disable):
            disable()

    def _motion_log_text(self, linear_mps: float, angular_rad_s: float) -> str:
        prefix = "DRY-RUN cmd_vel" if getattr(self.client.status, "mode", "live") == "dry-run" else "cmd_vel"
        return f"{prefix} linear={linear_mps:.2f} angular={angular_rad_s:.2f}"

    def _configure_motion_mode(self, linear_mps: float, angular_rad_s: float, now: float) -> None:
        self._disarm_after_motion = False
        is_turn_only = abs(linear_mps) < 1e-9 and abs(angular_rad_s) > 1e-9
        if not is_turn_only:
            self._active_stop_seconds = KEY_STOP_SECONDS
            self._active_repeat_enabled = True
            self._last_turn_signature = None
            self._last_turn_key_at = None
            return

        signature = 1.0 if angular_rad_s > 0 else -1.0
        held = (
            self._last_turn_signature == signature
            and self._last_turn_key_at is not None
            and now - self._last_turn_key_at <= TURN_HOLD_WINDOW_SECONDS
        )
        self._active_stop_seconds = TURN_HOLD_STOP_SECONDS if held else TURN_TAP_STOP_SECONDS
        self._active_repeat_enabled = held
        self._last_turn_signature = signature
        self._last_turn_key_at = now

    def _draw(self, screen, command_prompt: str = "> ") -> None:
        screen.erase()
        height, width = screen.getmaxyx()
        line_width = max(1, width - 1)
        content_limit = max(0, height - 2)
        status = self.client.status
        static_rows = format_status_lines(status, armed=self.state.armed, speed=self.state.speed, turn=self.state.turn)
        static_rows.extend([
            "",
            "↑/w forward  ↓/s reverse  ←/a arc left  →/d arc right  space stop  e estop  q quit",
            "Type /help for slash commands.",
            "",
        ])
        static_wrapped = self._wrap_display_rows(static_rows, line_width)
        history_wrapped = self._wrap_display_rows(self.state.history[-6:], line_width)
        if history_wrapped and content_limit > 0:
            history_limit = min(len(history_wrapped), max(1, content_limit // 3))
            static_limit = max(0, content_limit - history_limit)
            rows = static_wrapped[:static_limit] + history_wrapped[-history_limit:]
        else:
            rows = static_wrapped[:content_limit]
        prompt_row = min(len(rows), max(0, height - 1))
        for idx in range(max(0, height - 1)):
            if hasattr(screen, "move") and hasattr(screen, "clrtoeol"):
                screen.move(idx, 0)
                screen.clrtoeol()
            if idx < len(rows):
                screen.addnstr(idx, 0, rows[idx], line_width)
        if hasattr(screen, "move") and hasattr(screen, "clrtoeol"):
            screen.move(prompt_row, 0)
            screen.clrtoeol()
        screen.addnstr(prompt_row, 0, command_prompt[:line_width], line_width)
        screen.refresh()

    @staticmethod
    def _wrap_display_rows(rows: list[str], width: int) -> list[str]:
        wrapped: list[str] = []
        for row in rows:
            if not row:
                wrapped.append("")
                continue
            wrapped.extend(
                textwrap.wrap(
                    row,
                    width=width,
                    break_long_words=True,
                    break_on_hyphens=False,
                    replace_whitespace=False,
                )
                or [""]
            )
        return wrapped

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
        now = time.monotonic()
        odom_state = freshness_text(getattr(status, "odom_received_at", None), now)
        scan_state = freshness_text(getattr(status, "scan_received_at", None), now)
        return (
            f"mode={getattr(status, 'mode', 'live')} connected={status.connected} estop={status.emergency_stopped} "
            f"cmd_vel={getattr(status, 'cmd_vel_available', False)} "
            f"battery={self._battery_text()} odom={odom_state} "
            f"scan={scan_state} {self._launch_status_text()} diag='{status.diagnostic_message}'"
        )

    def _launch_status_text(self) -> str:
        launch_state = self.launch_manager.state
        pid = launch_state.pid if launch_state.pid is not None else "none"
        return f"launch={launch_state.profile.value} mapping={launch_state.mode.value} pid={pid}"

    @staticmethod
    def _key_name(screen, key_code: int) -> str:
        if key_code >= 0:
            if key_code < 256:
                return chr(key_code)
            try:
                name = curses.keyname(key_code).decode(errors="ignore")
            except curses.error:
                return screen.getkey()
            if name.startswith("^J"):
                return "\n"
            return name
        return ""


def main(argv: Optional[list[str]] = None) -> None:
    parser = argparse.ArgumentParser(description="Sphero RVR curses console")
    parser.add_argument("--dry-run", action="store_true", help="run with fake ROS surfaces and no hardware access")
    args = parser.parse_args(argv)
    client = DryRunRVRClient() if args.dry_run else RVRROSClient()
    RVRTUI(client).run()


if __name__ == "__main__":
    main()
