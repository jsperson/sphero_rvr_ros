import pytest

from sphero_rvr_driver.tui import DryRunRVRClient, RVRTUI
from sphero_rvr_driver.tui_commands import CommandParseError, TUICommand, parse_command
from sphero_rvr_driver.tui_launch import MapSaver, sanitize_map_name


class RecordingMapSaveRunner:
    def __init__(self, returncode=0, stdout="", stderr=""):
        self.commands = []
        self.returncode = returncode
        self.stdout = stdout
        self.stderr = stderr

    def save(self, command):
        self.commands.append(list(command))
        return type(
            "Completed",
            (),
            {"returncode": self.returncode, "stdout": self.stdout, "stderr": self.stderr},
        )()


@pytest.mark.parametrize(
    "raw,expected",
    [
        ("room_1", "room_1"),
        ("Room One", "Room_One"),
        ("../Bad Room!!!", "Bad_Room"),
        ("...rvr-map...", "rvr-map"),
    ],
)
def test_sanitize_map_name_keeps_safe_filename_stem(raw, expected):
    assert sanitize_map_name(raw) == expected


@pytest.mark.parametrize("raw", ["", "../", "!!!", ".", ".."])
def test_sanitize_map_name_rejects_empty_or_unsafe_names(raw):
    with pytest.raises(ValueError):
        sanitize_map_name(raw)


def test_parse_map_save_command():
    command = parse_command("/map save Room One")

    assert command.name == "map"
    assert command.value == "Room One"


@pytest.mark.parametrize("raw", ["/map", "/map save", "/map load room"])
def test_parse_map_save_rejects_invalid_shapes(raw):
    with pytest.raises(CommandParseError):
        parse_command(raw)


def test_map_saver_dry_run_reports_path_without_running_ros(tmp_path):
    runner = RecordingMapSaveRunner()
    saver = MapSaver(runner=runner, dry_run=True, output_dir=tmp_path)

    result = saver.save("Room One")

    assert result.success is True
    assert result.path == tmp_path / "Room_One"
    assert runner.commands == []
    assert "DRY-RUN map save" in result.message
    assert str(tmp_path / "Room_One") in result.message


def test_map_saver_runs_nav2_map_saver_cli_with_sanitized_path(tmp_path):
    runner = RecordingMapSaveRunner(stdout="saved")
    saver = MapSaver(runner=runner, output_dir=tmp_path)

    result = saver.save("../room one")

    assert result.success is True
    assert runner.commands == [
        ["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", str(tmp_path / "room_one")]
    ]
    assert result.message == f"Saved map to {tmp_path / 'room_one'}.yaml and {tmp_path / 'room_one'}.pgm"


def test_map_saver_reports_command_failure(tmp_path):
    runner = RecordingMapSaveRunner(returncode=2, stderr="no map")
    saver = MapSaver(runner=runner, output_dir=tmp_path)

    result = saver.save("room")

    assert result.success is False
    assert result.message == "Map save failed rc=2: no map"


def test_dry_run_tui_map_save_logs_intended_path_without_running_ros(tmp_path):
    runner = RecordingMapSaveRunner()
    saver = MapSaver(runner=runner, dry_run=True, output_dir=tmp_path)
    tui = RVRTUI(DryRunRVRClient(), map_saver=saver)

    tui._run_command(TUICommand("map", "Room One"))

    assert runner.commands == []
    assert tui.state.last_message.startswith("DRY-RUN map save:")
    assert str(tmp_path / "Room_One") in tui.state.last_message
