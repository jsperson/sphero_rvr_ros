from sphero_rvr_driver.tui_keymap import KeyAction, map_key


def test_direction_keys_map_to_motion_intents():
    assert map_key("KEY_UP", speed=0.1, turn=0.4) == KeyAction.motion(0.1, 0.0)
    assert map_key("w", speed=0.1, turn=0.4) == KeyAction.motion(0.1, 0.0)
    assert map_key("KEY_DOWN", speed=0.1, turn=0.4) == KeyAction.motion(-0.1, 0.0)
    assert map_key("s", speed=0.1, turn=0.4) == KeyAction.motion(-0.1, 0.0)
    assert map_key("KEY_LEFT", speed=0.1, turn=0.4) == KeyAction.motion(0.05, 0.4)
    assert map_key("a", speed=0.1, turn=0.4) == KeyAction.motion(0.05, 0.4)
    assert map_key("KEY_RIGHT", speed=0.1, turn=0.4) == KeyAction.motion(0.05, -0.4)
    assert map_key("d", speed=0.1, turn=0.4) == KeyAction.motion(0.05, -0.4)


def test_stop_estop_quit_keys():
    assert map_key(" ", speed=0.1, turn=0.4) == KeyAction.command("stop")
    assert map_key("e", speed=0.1, turn=0.4) == KeyAction.command("estop")
    assert map_key("q", speed=0.1, turn=0.4) == KeyAction.command("quit")


def test_unknown_key_returns_none():
    assert map_key("x", speed=0.1, turn=0.4) is None


def test_key_name_has_single_printable_and_special_key_path():
    from sphero_rvr_driver.tui import RVRTUI

    class Screen:
        @staticmethod
        def getkey():
            return "KEY_UP"

    assert RVRTUI._key_name(Screen(), ord("w")) == "w"
    assert RVRTUI._key_name(Screen(), 259) == "KEY_UP"
