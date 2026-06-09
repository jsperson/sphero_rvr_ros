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


def test_slash_arm_immediately_enables_keyboard_motion():
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


def test_disarmed_keyboard_motion_is_ignored():
    client = FakeClient()
    tui = RVRTUI(client)

    tui._apply_key_action(KeyAction.motion(0.1, 0.0))

    assert client.published == []
