from types import SimpleNamespace

from sphero_rvr_driver.tui_launch import LaunchManager, LaunchProfile, MappingMode


class RecordingRunner:
    def __init__(self):
        self.started = []
        self.stopped = []
        self.ran = []
        self.next_pid = 1000

    def start(self, command):
        self.started.append(list(command))
        self.next_pid += 1
        return self.next_pid

    def stop(self, pid, timeout_sec=5.0):
        self.stopped.append((pid, timeout_sec))

    def run(self, command, timeout_sec=5.0):
        self.ran.append((list(command), timeout_sec))
        return SimpleNamespace(returncode=0, stdout="Transition successful", stderr="")


def test_lidar_start_uses_lidar_launch_and_stop_cleans_owned_process():
    runner = RecordingRunner()
    manager = LaunchManager(runner=runner)

    result = manager.start_lidar()

    assert result.profile is LaunchProfile.LIDAR
    assert result.mode is MappingMode.LIDAR_ONLY
    assert result.pid == 1001
    assert runner.started == [["ros2", "launch", "sphero_rvr_driver", "lidar.launch.py"]]

    stop_result = manager.stop()

    assert stop_result.profile is LaunchProfile.NONE
    assert stop_result.mode is MappingMode.IDLE
    assert runner.stopped == [(1001, 5.0)]


def test_mapping_start_uses_safe_slam_without_rvr_driver():
    runner = RecordingRunner()
    manager = LaunchManager(runner=runner)

    result = manager.start_mapping(start_rvr=False)

    assert result.profile is LaunchProfile.MAPPING_LIDAR
    assert result.mode is MappingMode.LIDAR_ONLY
    assert runner.started == [
        ["ros2", "launch", "sphero_rvr_driver", "mapping.launch.py", "start_rvr:=false"]
    ]
    assert runner.ran == [
        (["ros2", "lifecycle", "set", "/slam_toolbox", "configure"], 5.0),
        (["ros2", "lifecycle", "set", "/slam_toolbox", "activate"], 5.0),
    ]
    assert "slam_toolbox lifecycle active" in result.message


def test_mapping_full_confirm_replaces_safe_launch_with_motor_capable_launch():
    runner = RecordingRunner()
    manager = LaunchManager(runner=runner)

    safe = manager.start_mapping(start_rvr=False)

    result = manager.start_mapping(start_rvr=True)

    assert result.profile is LaunchProfile.MAPPING_MOTOR
    assert result.mode is MappingMode.MOTOR_CAPABLE
    assert runner.stopped == [(safe.pid, 5.0)]
    assert runner.started[-1] == [
        "ros2",
        "launch",
        "sphero_rvr_driver",
        "mapping.launch.py",
        "start_rvr:=true",
    ]


def test_dry_run_records_state_without_starting_ros_processes():
    runner = RecordingRunner()
    manager = LaunchManager(runner=runner, dry_run=True)

    result = manager.start_mapping(start_rvr=True)

    assert result.profile is LaunchProfile.MAPPING_MOTOR
    assert result.mode is MappingMode.MOTOR_CAPABLE
    assert result.pid is None
    assert runner.started == []


def test_failed_pre_stop_blocks_replacement_launch():
    class StopFailingRunner(RecordingRunner):
        def stop(self, pid, timeout_sec=5.0):
            super().stop(pid, timeout_sec=timeout_sec)
            raise TimeoutError("still running")

    runner = StopFailingRunner()
    manager = LaunchManager(runner=runner)
    manager.start_lidar()

    result = manager.start_mapping(start_rvr=True)

    assert result.mode is MappingMode.FAILED_LAUNCH
    assert result.profile is LaunchProfile.NONE
    assert runner.stopped == [(1001, 5.0)]
    assert runner.started == [["ros2", "launch", "sphero_rvr_driver", "lidar.launch.py"]]


def test_failed_launch_records_failed_state_and_reason():
    class FailingRunner(RecordingRunner):
        def start(self, command):
            raise FileNotFoundError("ros2")

    manager = LaunchManager(runner=FailingRunner())

    result = manager.start_lidar()

    assert result.mode is MappingMode.FAILED_LAUNCH
    assert result.profile is LaunchProfile.NONE
    assert "ros2" in result.message


def test_mapping_start_fails_and_stops_when_slam_lifecycle_activation_fails():
    class LifecycleFailingRunner(RecordingRunner):
        def run(self, command, timeout_sec=5.0):
            self.ran.append((list(command), timeout_sec))
            return SimpleNamespace(returncode=1, stdout="", stderr="transition failed")

    runner = LifecycleFailingRunner()
    manager = LaunchManager(runner=runner)

    result = manager.start_mapping(start_rvr=True)

    assert result.mode is MappingMode.FAILED_LAUNCH
    assert result.profile is LaunchProfile.NONE
    assert "Failed to configure slam_toolbox" in result.message
    assert runner.stopped == [(1001, 5.0)]
