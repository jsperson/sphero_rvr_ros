import pytest

import sphero_rvr_driver.tui as tui_module
from sphero_rvr_driver.tui import DryRunRVRClient, RVRTUI
from sphero_rvr_driver.tui_commands import NudgeCommand, TUICommand
from sphero_rvr_driver.tui_keymap import KeyAction
from sphero_rvr_driver.tui_launch import LaunchManager, LaunchProfile, MappingMode


class FakeClient:
    def __init__(self):
        self.published = []
        self.stopped = False
        self.cmd_vel_enable_count = 0
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

    def enable_velocity_publisher(self):
        self.cmd_vel_enable_count += 1

    def stop(self, timeout_sec=2.0):
        self.stopped = True
        return "stopped"

    def estop(self, timeout_sec=2.0):
        return "estopped"

    def clear_estop(self, timeout_sec=2.0):
        return "cleared"


class VelocityLifecycleClient(FakeClient):
    def __init__(self):
        super().__init__()
        self.velocity_publisher_enabled = False
        self.enable_velocity_publisher_calls = 0

    def enable_velocity_publisher(self):
        self.enable_velocity_publisher_calls += 1
        self.velocity_publisher_enabled = True


class RecordingRunner:
    def __init__(self):
        self.started = []
        self.stopped = []
        self.next_pid = 42

    def start(self, command):
        self.started.append(list(command))
        self.next_pid += 1
        return self.next_pid

    def stop(self, pid, timeout_sec=5.0):
        self.stopped.append((pid, timeout_sec))

    def run(self, command, timeout_sec=5.0):
        return type("Completed", (), {"returncode": 0, "stdout": "Transition successful", "stderr": ""})()


class FailingRunner(RecordingRunner):
    def start(self, command):
        self.started.append(list(command))
        raise RuntimeError("boom")


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


def test_turn_taps_outside_hold_window_remain_discrete(monkeypatch):
    now = 100.0
    monkeypatch.setattr(tui_module.time, "monotonic", lambda: now)
    client = FakeClient()
    tui = RVRTUI(client)
    tui._run_command(TUICommand("arm"))

    tui._apply_key_action(KeyAction.motion(0.0, 0.35))
    now = 100.10
    tui._maintain_motion()
    now = 100.16
    tui._apply_key_action(KeyAction.motion(0.0, 0.35))
    now = 100.26
    tui._maintain_motion()

    assert client.published == [
        (0.0, 0.35),
        (0.0, 0.0),
        (0.0, 0.35),
        (0.0, 0.0),
    ]


def test_dry_run_client_simulates_status_without_ros_publisher():
    client = DryRunRVRClient()

    client.start()
    client.publish_velocity(0.1, -0.2)

    assert client.status.connected is True
    assert client.status.diagnostic_message == "DRY RUN: fake ROS surfaces active"
    assert client.status.battery_percentage == 0.87
    assert client.status.battery_voltage == 7.8
    assert client.status.odom_received_at is not None
    assert client.status.scan_received_at is not None
    assert client.published_commands == [(0.1, -0.2)]
    assert not hasattr(client, "_cmd_pub")


def test_dry_run_tui_logs_intended_motion_instead_of_real_publish():
    client = DryRunRVRClient()
    tui = RVRTUI(client)

    tui._run_command(TUICommand("arm"))
    tui._apply_key_action(KeyAction.motion(0.1, 0.0))

    assert client.published_commands == [(0.1, 0.0)]
    assert tui.state.history[-1] == "DRY-RUN cmd_vel linear=0.10 angular=0.00"


def test_nudge_requires_confirm_before_publishing_motion():
    client = FakeClient()
    tui = RVRTUI(client)
    tui._run_command(TUICommand("arm", "confirm"))

    tui._run_command(TUICommand("nudge", NudgeCommand(direction="forward", distance_m=0.02, confirmed=False)))

    assert client.published == []
    assert "WARNING: this can start the RVR motors" in tui.state.last_message
    assert "/nudge forward 0.02 confirm" in tui.state.last_message


def test_nudge_requires_armed_state():
    client = FakeClient()
    tui = RVRTUI(client)

    tui._run_command(TUICommand("nudge", NudgeCommand(direction="forward", distance_m=0.02, confirmed=True)))

    assert client.published == []
    assert "Use /arm confirm" in tui.state.last_message


def test_confirmed_nudge_requires_enabled_velocity_sink():
    client = VelocityLifecycleClient()
    tui = RVRTUI(client)
    tui._run_command(TUICommand("arm", "confirm"))

    tui._run_command(TUICommand("nudge", NudgeCommand(direction="forward", distance_m=0.02, confirmed=True)))

    assert client.published == []
    assert tui.state.armed is True
    assert tui.state.last_message == "Nudge rejected: velocity publisher not enabled."


def test_confirmed_forward_nudge_publishes_motion_then_zero_and_disarms(monkeypatch):
    sleeps = []
    monkeypatch.setattr(tui_module.time, "sleep", sleeps.append)
    client = FakeClient()
    tui = RVRTUI(client)
    tui._run_command(TUICommand("arm", "confirm"))

    tui._run_command(TUICommand("nudge", NudgeCommand(direction="forward", distance_m=0.02, confirmed=True)))

    assert client.published == [(0.05, 0.0), (0.0, 0.0)]
    assert sleeps == [pytest.approx(0.08748906386701663)]
    assert tui.state.armed is False
    assert "nudge forward distance=0.02 m" in tui.state.last_message


def test_confirmed_back_nudge_uses_reverse_velocity_and_zero_on_sleep_failure(monkeypatch):
    def fail_sleep(duration):
        raise RuntimeError("interrupted")

    monkeypatch.setattr(tui_module.time, "sleep", fail_sleep)
    client = FakeClient()
    tui = RVRTUI(client)
    tui._run_command(TUICommand("arm", "confirm"))

    tui._run_command(TUICommand("nudge", NudgeCommand(direction="back", distance_m=0.02, confirmed=True)))

    assert client.published == [(-0.05, 0.0), (0.0, 0.0)]
    assert tui.state.armed is False
    assert tui.state.last_message == "ERROR: interrupted"


def test_dry_run_nudge_logs_fake_motion_and_expected_encoder_counts(monkeypatch):
    monkeypatch.setattr(tui_module.time, "sleep", lambda _duration: None)
    client = DryRunRVRClient()
    tui = RVRTUI(client)
    tui._run_command(TUICommand("arm", "confirm"))

    tui._run_command(TUICommand("nudge", NudgeCommand(direction="forward", distance_m=0.02, confirmed=True)))

    assert client.published_commands == [(0.05, 0.0), (0.0, 0.0)]
    assert "DRY-RUN nudge forward distance=0.02 m" in tui.state.last_message
    assert "expected_encoder_counts=86.8" in tui.state.last_message


def test_dry_run_status_is_visually_obvious():
    tui = RVRTUI(DryRunRVRClient())

    assert "mode=dry-run" in tui._status_text()
    assert "odom=fresh" in tui._status_text()
    assert "scan=fresh" in tui._status_text()


def test_main_uses_dry_run_client_when_requested(monkeypatch):
    used_clients = []

    class CapturingTUI:
        def __init__(self, client):
            used_clients.append(client)

        def run(self):
            return None

    monkeypatch.setattr(tui_module, "RVRTUI", CapturingTUI)

    tui_module.main(["--dry-run"])

    assert isinstance(used_clients[0], DryRunRVRClient)

def test_lidar_start_and_stop_commands_manage_launch_process():
    runner = RecordingRunner()
    launcher = LaunchManager(runner=runner)
    tui = RVRTUI(FakeClient(), launch_manager=launcher)

    tui._run_command(TUICommand("lidar", "start"))

    assert runner.started == [["ros2", "launch", "sphero_rvr_driver", "lidar.launch.py"]]
    assert tui.client.cmd_vel_enable_count == 0
    assert launcher.state.profile is LaunchProfile.LIDAR
    assert launcher.state.mode is MappingMode.LIDAR_ONLY

    tui._run_command(TUICommand("lidar", "stop"))

    assert runner.stopped == [(43, 5.0)]
    assert launcher.state.mode is MappingMode.IDLE


def test_mapping_start_uses_safe_launch_without_rvr_driver():
    runner = RecordingRunner()
    launcher = LaunchManager(runner=runner)
    tui = RVRTUI(FakeClient(), launch_manager=launcher)

    tui._run_command(TUICommand("mapping", "start"))

    assert runner.started == [
        ["ros2", "launch", "sphero_rvr_driver", "mapping.launch.py", "start_rvr:=false"]
    ]
    assert tui.client.cmd_vel_enable_count == 0
    assert launcher.state.profile is LaunchProfile.MAPPING_LIDAR
    assert launcher.state.mode is MappingMode.LIDAR_ONLY


def test_mapping_full_requires_confirm_and_leaves_tui_disarmed():
    runner = RecordingRunner()
    launcher = LaunchManager(runner=runner)
    tui = RVRTUI(FakeClient(), launch_manager=launcher)

    tui._run_command(TUICommand("mapping", "full"))

    assert runner.started == []
    assert tui.client.cmd_vel_enable_count == 0
    assert tui.state.armed is False
    assert "WARNING: this can start the RVR motors" in tui.state.last_message

    tui._run_command(TUICommand("mapping", "full-confirm"))

    assert runner.started == [
        ["ros2", "launch", "sphero_rvr_driver", "mapping.launch.py", "start_rvr:=true"]
    ]
    assert launcher.state.profile is LaunchProfile.MAPPING_MOTOR
    assert launcher.state.mode is MappingMode.MOTOR_CAPABLE
    assert tui.client.cmd_vel_enable_count == 1
    assert tui.state.armed is False


def test_mapping_full_confirm_does_not_enable_cmd_vel_when_launch_fails():
    runner = FailingRunner()
    launcher = LaunchManager(runner=runner)
    tui = RVRTUI(FakeClient(), launch_manager=launcher)

    tui._run_command(TUICommand("mapping", "full-confirm"))

    assert runner.started == [
        ["ros2", "launch", "sphero_rvr_driver", "mapping.launch.py", "start_rvr:=true"]
    ]
    assert launcher.state.mode is MappingMode.FAILED_LAUNCH
    assert tui.client.cmd_vel_enable_count == 0


def test_mapping_stop_publishes_zero_velocity_and_stops_launch():
    runner = RecordingRunner()
    launcher = LaunchManager(runner=runner)
    tui = RVRTUI(FakeClient(), launch_manager=launcher)
    tui._run_command(TUICommand("mapping", "full-confirm"))
    tui._run_command(TUICommand("arm", "confirm"))

    tui._run_command(TUICommand("mapping", "stop"))

    assert tui.state.armed is False
    assert tui.client.published == [(0.0, 0.0)]
    assert runner.stopped == [(43, 5.0)]


def test_dry_run_mapping_full_exercises_state_without_ros_process():
    runner = RecordingRunner()
    launcher = LaunchManager(runner=runner, dry_run=True)
    client = DryRunRVRClient()
    tui = RVRTUI(client, launch_manager=launcher)

    tui._run_command(TUICommand("mapping", "full-confirm"))
    tui._run_command(TUICommand("mapping", "stop"))
    assert runner.started == []
    assert runner.stopped == []
    assert launcher.state.mode is MappingMode.IDLE
    assert client.published_commands == [(0.0, 0.0)]
    assert not hasattr(client, "_cmd_pub")
