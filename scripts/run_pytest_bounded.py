#!/usr/bin/env python3
"""Run pytest under a repository lock and hard wall-clock deadline."""

from __future__ import annotations

import argparse
import fcntl
import os
import signal
import subprocess
import sys
import tempfile
from collections.abc import Sequence
from pathlib import Path
from typing import TextIO

TIMEOUT_EXIT_CODE = 124
INTERRUPTED_EXIT_CODE = 130
CONCURRENT_RUN_EXIT_CODE = 75


def _acquire_lock(path: Path) -> TextIO | None:
    """Acquire the repository-wide test lock or return None without waiting."""
    path.parent.mkdir(parents=True, exist_ok=True)
    lock = path.open("a+")
    try:
        fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        lock.close()
        return None
    lock.seek(0)
    lock.truncate()
    lock.write(f"pid={os.getpid()} cwd={Path.cwd()}\n")
    lock.flush()
    return lock


def _terminate_process_group(process: subprocess.Popen[bytes], grace_seconds: float) -> None:
    """Terminate and reap pytest plus every descendant in its process group."""
    if process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGTERM)
    except ProcessLookupError:
        pass
    try:
        process.wait(timeout=grace_seconds)
        return
    except subprocess.TimeoutExpired:
        pass
    try:
        os.killpg(process.pid, signal.SIGKILL)
    except ProcessLookupError:
        pass
    process.wait()


def run_pytest(
    pytest_args: Sequence[str],
    *,
    timeout_seconds: float,
    grace_seconds: float,
    lock_path: Path,
) -> int:
    """Run pytest once with mutual exclusion and bounded process cleanup."""
    lock = _acquire_lock(lock_path)
    if lock is None:
        print(
            f"BOUNDED_PYTEST_CONCURRENT lock={lock_path}; another test run owns the lock; "
            "result is NO VERDICT",
            file=sys.stderr,
            flush=True,
        )
        return CONCURRENT_RUN_EXIT_CODE
    try:
        command = [sys.executable, "-m", "pytest", *pytest_args]
        process = subprocess.Popen(command, start_new_session=True)
        print(
            f"BOUNDED_PYTEST_START pid={process.pid} timeout_s={timeout_seconds:g} "
            f"lock={lock_path} command={' '.join(command)}",
            flush=True,
        )
        try:
            return process.wait(timeout=timeout_seconds)
        except subprocess.TimeoutExpired:
            print(
                f"BOUNDED_PYTEST_TIMEOUT pid={process.pid} timeout_s={timeout_seconds:g}; "
                "terminating process group; result is NO VERDICT",
                file=sys.stderr,
                flush=True,
            )
            _terminate_process_group(process, grace_seconds)
            return TIMEOUT_EXIT_CODE
        except KeyboardInterrupt:
            print(
                f"BOUNDED_PYTEST_INTERRUPTED pid={process.pid}; terminating process group; "
                "result is NO VERDICT",
                file=sys.stderr,
                flush=True,
            )
            _terminate_process_group(process, grace_seconds)
            return INTERRUPTED_EXIT_CODE
    finally:
        lock.close()


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run pytest with mutual exclusion and a hard process-group timeout.",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=90.0,
        help="hard wall-clock deadline in seconds (default: 90)",
    )
    parser.add_argument(
        "--term-grace",
        type=float,
        default=5.0,
        help="seconds between SIGTERM and SIGKILL (default: 5)",
    )
    parser.add_argument(
        "--lock-file",
        type=Path,
        default=Path(tempfile.gettempdir()) / "sphero-rvr-ros-pytest.lock",
        help="repository-wide nonblocking lock file",
    )
    parser.add_argument("pytest_args", nargs=argparse.REMAINDER)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = _parser()
    args = parser.parse_args(argv)
    if args.timeout <= 0:
        parser.error("--timeout must be positive")
    if args.term_grace <= 0:
        parser.error("--term-grace must be positive")
    pytest_args = list(args.pytest_args)
    if pytest_args[:1] == ["--"]:
        pytest_args = pytest_args[1:]
    if not pytest_args:
        pytest_args = ["-q"]
    return run_pytest(
        pytest_args,
        timeout_seconds=args.timeout,
        grace_seconds=args.term_grace,
        lock_path=args.lock_file,
    )


if __name__ == "__main__":
    raise SystemExit(main())
