import sphero_rvr_driver.tui as tui_module
from sphero_rvr_driver.tui import RVRTUI
from sphero_rvr_driver.tui_commands import TUICommand
from sphero_rvr_driver.tui_keymap import KeyAction


class FakeClient:
    def __init__(self):
        self.published = []
        self.stopped = False
        self.status = type(
            "Status",
            (),
            {
                "connected": True,
                "emergency_stopped": False,
                "diagnostic_message": "fake",
                "battery_percentage": 0.5,
                "battery_voltage": 7.4,
            },
        )()

    def publish_velocity(self, linear_mps, angular_rad_s):
        self.published.append((linear_mps, angular_rad_s))

    def stop(self, timeout_sec=2.0):
        self.stopped = True
        return "stopped"

    def estop(self, timeout_sec=2.0):
        return "estopped"

    def clear_estop(self, timeout_sec=2.0):
        return "cleared"


def test_slash_arm_enables_keyboard_motion():
    client = FakeClient()
    tui = RVRTUI(client)

    tui._run_command(TUICommand("arm"))
    tui._apply_key_action(KeyAction.motion(0.1, 0.0))

    assert tui.state.armed is True
    assert client.published == [(0.1, 0.0)]


def test_arm_confirm_remains_supported_alias():
    client = FakeClient()
    tui = RVRTUI(client)

    tui._run_command(TUICommand("arm", "confirm"))

    assert tui.state.armed is True


def test_disarm_publishes_zero_velocity_without_calling_stop_service():
    client = FakeClient()
    tui = RVRTUI(client)

    tui._run_command(TUICommand("arm"))
    tui._run_command(TUICommand("disarm"))

    assert tui.state.armed is False
    assert client.published == [(0.0, 0.0)]
    assert client.stopped is False


def test_safe_stop_on_exit_publishes_zero_velocity_without_calling_stop_service():
    client = FakeClient()
    tui = RVRTUI(client)

    tui._safe_stop()

    assert client.published == [(0.0, 0.0)]
    assert client.stopped is False


def test_disarmed_keyboard_motion_is_ignored():
    client = FakeClient()
    tui = RVRTUI(client)

    tui._apply_key_action(KeyAction.motion(0.1, 0.0))

    assert client.published == []


def test_active_motion_is_republished_until_key_timeout(monkeypatch):
    now = 100.0
    monkeypatch.setattr(tui_module.time, "monotonic", lambda: now)
    client = FakeClient()
    tui = RVRTUI(client)
    tui._run_command(TUICommand("arm"))

    tui._apply_key_action(KeyAction.motion(0.1, 0.0))
    now = 100.11
    tui._maintain_motion()
    now = 100.26
    tui._maintain_motion()
    now = 100.31
    tui._maintain_motion()

    assert client.published == [(0.1, 0.0), (0.1, 0.0), (0.1, 0.0), (0.0, 0.0)]


def test_turn_tap_stops_quickly_without_internal_republish(monkeypatch):
    now = 100.0
    monkeypatch.setattr(tui_module.time, "monotonic", lambda: now)
    client = FakeClient()
    tui = RVRTUI(client)
    tui._run_command(TUICommand("arm"))

    tui._apply_key_action(KeyAction.motion(0.0, 0.35))
    now = 100.05
    tui._maintain_motion()
    now = 100.10
    tui._maintain_motion()

    assert client.published == [(0.0, 0.35), (0.0, 0.0)]


def test_repeated_turn_keypresses_continue_turning(monkeypatch):
    now = 100.0
    monkeypatch.setattr(tui_module.time, "monotonic", lambda: now)
    client = FakeClient()
    tui = RVRTUI(client)
    tui._run_command(TUICommand("arm"))

    tui._apply_key_action(KeyAction.motion(0.0, 0.35))
    now = 100.08
    tui._apply_key_action(KeyAction.motion(0.0, 0.35))
    now = 100.16
    tui._apply_key_action(KeyAction.motion(0.0, 0.35))
    now = 100.26
    tui._maintain_motion()
    now = 100.47
    tui._maintain_motion()

    assert client.published == [
        (0.0, 0.35),
        (0.0, 0.35),
        (0.0, 0.35),
        (0.0, 0.35),
        (0.0, 0.0),
    ]
