import pytest

from sphero_rvr_driver.tui_commands import CommandParseError, NudgeCommand, parse_command


@pytest.mark.parametrize(
    "raw,name,value",
    [
        ("/battery", "battery", None),
        ("/status", "status", None),
        ("/stop", "stop", None),
        ("/estop", "estop", None),
        ("/clear-estop", "clear-estop", None),
        ("/clear-estop confirm", "clear-estop", None),
        ("/arm", "arm", None),
        ("/arm confirm", "arm", "confirm"),
        ("/disarm", "disarm", None),
        ("/help", "help", None),
        ("/quit", "quit", None),
        ("/lidar start", "lidar", "start"),
        ("/lidar stop", "lidar", "stop"),
        ("/mapping start", "mapping", "start"),
        ("/mapping stop", "mapping", "stop"),
        ("/mapping status", "mapping", "status"),
        ("/mapping full", "mapping", "full"),
        ("/mapping full confirm", "mapping", "full-confirm"),
        ("/speed 0.15", "speed", 0.15),
        ("/turn 0.4", "turn", 0.4),
    ],
)
def test_parse_known_commands(raw, name, value):
    command = parse_command(raw)

    assert command.name == name
    assert command.value == value


@pytest.mark.parametrize(
    "raw,direction,distance_m,confirmed",
    [
        ("/nudge forward 0.02", "forward", 0.02, False),
        ("/nudge forward 0.02 confirm", "forward", 0.02, True),
        ("/nudge back 6in confirm", "back", 0.1524, True),
        ("/nudge back 3 inches confirm", "back", 0.0762, True),
    ],
)
def test_parse_nudge_commands(raw, direction, distance_m, confirmed):
    command = parse_command(raw)

    assert command.name == "nudge"
    assert command.value == NudgeCommand(direction=direction, distance_m=pytest.approx(distance_m), confirmed=confirmed)


@pytest.mark.parametrize(
    "raw",
    [
        "",
        "battery",
        "/bogus",
        "/speed",
        "/speed nope",
        "/speed -1",
        "/turn",
        "/turn nope",
        "/lidar",
        "/lidar full confirm",
        "/lidar start confirm",
        "/mapping",
        "/mapping begin",
        "/mapping full yes",
        "/mapping start confirm",
        "/nudge forward nope confirm",
        "/nudge forward 0 confirm",
        "/nudge forward 0.01 confirm",
        "/nudge forward -0.01 confirm",
        "/nudge forward 7in confirm",
        "/nudge back 0.20 confirm",
    ],
)
def test_parse_rejects_invalid_commands(raw):
    with pytest.raises(CommandParseError):
        parse_command(raw)
