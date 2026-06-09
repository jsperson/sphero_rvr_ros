from sphero_rvr_driver.tui import MOTION_ENABLE_ENV, RVRTUI
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


def test_slash_arm_refuses_motion_when_safety_env_is_absent(monkeypatch):
    monkeypatch.delenv(MOTION_ENABLE_ENV, raising=False)
    client = FakeClient()
    tui = RVRTUI(client)

    tui._run_command(TUICommand("arm"))
    tui._apply_key_action(KeyAction.motion(0.1, 0.0))

    assert tui.state.armed is False
    assert client.published == []
    assert any("Motion disabled" in line for line in tui.state.history)


def test_slash_arm_enables_keyboard_motion_when_safety_env_is_set(monkeypatch):
    monkeypatch.setenv(MOTION_ENABLE_ENV, "1")
    client = FakeClient()
    tui = RVRTUI(client)

    tui._run_command(TUICommand("arm"))
    tui._apply_key_action(KeyAction.motion(0.1, 0.0))

    assert tui.state.armed is True
    assert client.published == [(0.1, 0.0)]


def test_arm_confirm_remains_supported_alias(monkeypatch):
    monkeypatch.setenv(MOTION_ENABLE_ENV, "1")
    client = FakeClient()
    tui = RVRTUI(client)

    tui._run_command(TUICommand("arm", "confirm"))

    assert tui.state.armed is True


def test_disarmed_keyboard_motion_is_ignored():
    client = FakeClient()
    tui = RVRTUI(client)

    tui._apply_key_action(KeyAction.motion(0.1, 0.0))

    assert client.published == []
