"""Launch process management for the Sphero RVR terminal UI."""

from __future__ import annotations

import subprocess
from dataclasses import dataclass
from enum import Enum
from typing import Sequence


class MappingMode(Enum):
    IDLE = "idle"
    LIDAR_ONLY = "lidar-only"
    MOTOR_CAPABLE = "motor-capable"
    STOPPING = "stopping"
    FAILED_LAUNCH = "failed-launch"


class LaunchProfile(Enum):
    NONE = "none"
    LIDAR = "lidar"
    MAPPING_LIDAR = "mapping-lidar"
    MAPPING_FULL = "mapping-full"


@dataclass(frozen=True)
class LaunchState:
    mode: MappingMode = MappingMode.IDLE
    profile: LaunchProfile = LaunchProfile.NONE
    pid: int | None = None
    message: str = "No managed launch."


class SubprocessLaunchRunner:
    """Starts and stops ROS launch processes owned by the TUI."""

    def __init__(self):
        self._processes: dict[int, subprocess.Popen] = {}

    def start(self, command: Sequence[str]) -> int:
        process = subprocess.Popen(list(command))
        self._processes[process.pid] = process
        return process.pid

    def stop(self, pid: int, timeout_sec: float = 5.0) -> None:
        process = self._processes.pop(pid, None)
        if process is None:
            return
        if process.poll() is not None:
            return
        process.terminate()
        try:
            process.wait(timeout=timeout_sec)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=timeout_sec)


class LaunchManager:
    """Tracks one managed lidar/mapping launch process and cleans it up."""

    def __init__(self, runner=None, dry_run: bool = False):
        self._runner = runner if runner is not None else SubprocessLaunchRunner()
        self._dry_run = dry_run
        self.state = LaunchState(message="Dry-run launch manager ready." if dry_run else "No managed launch.")

    @property
    def dry_run(self) -> bool:
        return self._dry_run

    def start_lidar(self) -> LaunchState:
        command = ["ros2", "launch", "sphero_rvr_driver", "lidar.launch.py"]
        return self._start(command, LaunchProfile.LIDAR, MappingMode.LIDAR_ONLY)

    def start_mapping(self, *, start_rvr: bool) -> LaunchState:
        command = [
            "ros2",
            "launch",
            "sphero_rvr_driver",
            "mapping.launch.py",
            f"start_rvr:={'true' if start_rvr else 'false'}",
        ]
        profile = LaunchProfile.MAPPING_FULL if start_rvr else LaunchProfile.MAPPING_LIDAR
        mode = MappingMode.MOTOR_CAPABLE if start_rvr else MappingMode.LIDAR_ONLY
        return self._start(command, profile, mode)

    def stop(self) -> LaunchState:
        pid = self.state.pid
        self.state = LaunchState(
            mode=MappingMode.STOPPING,
            profile=self.state.profile,
            pid=pid,
            message="Stopping managed launch.",
        )
        try:
            if pid is not None and not self._dry_run:
                self._runner.stop(pid, timeout_sec=5.0)
        except Exception as exc:  # pragma: no cover - defensive cleanup path
            self.state = LaunchState(
                mode=MappingMode.FAILED_LAUNCH,
                profile=LaunchProfile.NONE,
                message=f"Failed to stop managed launch: {exc}",
            )
            return self.state
        self.state = LaunchState(message="Managed launch stopped.")
        return self.state

    def _start(self, command: Sequence[str], profile: LaunchProfile, mode: MappingMode) -> LaunchState:
        if self.state.pid is not None or self.state.profile is not LaunchProfile.NONE:
            self.stop()
        if self._dry_run:
            self.state = LaunchState(
                mode=mode,
                profile=profile,
                pid=None,
                message=f"DRY-RUN launch: {' '.join(command)}",
            )
            return self.state
        try:
            pid = self._runner.start(command)
        except Exception as exc:
            self.state = LaunchState(
                mode=MappingMode.FAILED_LAUNCH,
                profile=LaunchProfile.NONE,
                message=f"Launch failed: {exc}",
            )
            return self.state
        self.state = LaunchState(
            mode=mode,
            profile=profile,
            pid=pid,
            message=f"Started {profile.value} launch pid={pid}.",
        )
        return self.state
