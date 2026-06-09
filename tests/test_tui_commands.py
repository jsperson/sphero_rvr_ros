import pytest

from sphero_rvr_driver.tui_commands import CommandParseError, parse_command


@pytest.mark.parametrize(
    "raw,name,value",
    [
        ("/battery", "battery", None),
        ("/status", "status", None),
        ("/stop", "stop", None),
        ("/estop", "estop", None),
        ("/clear-estop", "clear-estop", None),
        ("/arm", "arm", None),
        ("/arm confirm", "arm", "confirm"),
        ("/disarm", "disarm", None),
        ("/help", "help", None),
        ("/quit", "quit", None),
        ("/speed 0.15", "speed", 0.15),
        ("/turn 0.4", "turn", 0.4),
    ],
)
def test_parse_known_commands(raw, name, value):
    command = parse_command(raw)

    assert command.name == name
    assert command.value == value


@pytest.mark.parametrize("raw", ["", "battery", "/bogus", "/speed", "/speed nope", "/speed -1", "/turn", "/turn nope"])
def test_parse_rejects_invalid_commands(raw):
    with pytest.raises(CommandParseError):
        parse_command(raw)
