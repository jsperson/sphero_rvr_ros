#!/usr/bin/env python3
"""Run pytest under a repository lock and hard wall-clock deadline."""

from __future__ import annotations

import argparse
import fcntl
import os
import re
import shlex
import signal
import subprocess
import sys
import tempfile
import time
from collections.abc import Sequence
from pathlib import Path
from types import FrameType
from typing import Any, TextIO

TIMEOUT_EXIT_CODE = 124
CLEANUP_FAILURE_EXIT_CODE = 125
INTERRUPTED_EXIT_CODE = 130
CONCURRENT_RUN_EXIT_CODE = 75


class _ExternalTermination(Exception):
    def __init__(self, signum: int) -> None:
        super().__init__(signum)
        self.signum = signum


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


def _process_group_exists(process_group: int) -> bool:
    try:
        os.killpg(process_group, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def _wait_for_group_exit(process_group: int, timeout_seconds: float) -> bool:
    deadline = time.monotonic() + timeout_seconds
    while _process_group_exists(process_group):
        if time.monotonic() >= deadline:
            return False
        time.sleep(0.05)
    return True


def _terminate_process_group(process: subprocess.Popen[bytes], grace_seconds: float) -> bool:
    """Terminate pytest's entire process group, including resistant descendants."""
    process_group = process.pid
    if _process_group_exists(process_group):
        try:
            os.killpg(process_group, signal.SIGTERM)
        except (ProcessLookupError, PermissionError):
            pass

    # Reap the group leader while allowing the whole group its TERM grace.
    # Otherwise the leader's zombie alone keeps killpg(..., 0) reporting that
    # the group exists and obscures whether resistant descendants survived.
    deadline = time.monotonic() + grace_seconds
    while _process_group_exists(process_group) and time.monotonic() < deadline:
        if process.poll() is None:
            try:
                process.wait(timeout=min(0.05, max(0.001, deadline - time.monotonic())))
            except subprocess.TimeoutExpired:
                pass
        else:
            time.sleep(0.05)

    if _process_group_exists(process_group):
        try:
            os.killpg(process_group, signal.SIGKILL)
        except (ProcessLookupError, PermissionError):
            pass
    if process.poll() is None:
        try:
            process.wait(timeout=max(5.0, grace_seconds))
        except subprocess.TimeoutExpired:
            return False

    # Keep the repository lock until the OS has reaped post-SIGKILL orphans.
    return _wait_for_group_exit(process_group, max(5.0, grace_seconds))


def _signal_handler(signum: int, _frame: FrameType | None) -> None:
    raise _ExternalTermination(signum)


def _install_signal_handlers() -> dict[int, Any]:
    previous: dict[int, Any] = {}
    for signum in (signal.SIGTERM, signal.SIGHUP):
        previous[signum] = signal.getsignal(signum)
        signal.signal(signum, _signal_handler)
    return previous


def _restore_signal_handlers(previous: dict[int, Any]) -> None:
    for signum, handler in previous.items():
        signal.signal(signum, handler)


#: How long a finished session is allowed to exit on its own before we reap it. Long
#: enough for an ordinary interpreter shutdown, far short of the hard deadline -- the
#: whole point is not to pay the deadline for a session that has already answered.
SESSION_EXIT_GRACE_S = 5.0


def _session_finished(junit_path: Path | None) -> bool:
    """Has pytest written a COMPLETE junit report? The closing tag is the test: a
    partially-flushed file means the hook is still running, and treating that as
    'finished' would reap a session mid-write."""
    if junit_path is None or not junit_path.exists():
        return False
    try:
        tail = junit_path.read_bytes()[-64:]
    except OSError:
        return False
    return b"</testsuites>" in tail or b"</testsuite>" in tail


def _verdict_from_junit(junit_path: Path) -> int:
    """Exit code derived from the report when the process had to be reaped.

    ONLY used when pytest finished testing and then refused to exit. The number a
    landing gates on then comes from the ARTIFACT rather than from the process, which
    is a real change of source and is announced loudly at the call site.
    """
    try:
        text = junit_path.read_text(encoding="utf-8", errors="replace")
    except OSError:
        return CLEANUP_FAILURE_EXIT_CODE
    failures = re.search(r'failures="(\d+)"', text)
    errors = re.search(r'errors="(\d+)"', text)
    bad = int(failures.group(1) if failures else 0) + int(errors.group(1) if errors else 0)
    return 0 if bad == 0 else 1


def _wait_for_session(
    process: "subprocess.Popen[bytes]",
    junit_path: Path | None,
    timeout_seconds: float,
    grace_seconds: float,
) -> int:
    """Wait for pytest to FINISH TESTING, not merely to die."""
    deadline = time.monotonic() + timeout_seconds
    finished_at: float | None = None
    while True:
        code = process.poll()
        if code is not None:
            return code                      # the ordinary path, unchanged
        now = time.monotonic()
        if finished_at is None and _session_finished(junit_path):
            finished_at = now
            print(
                f"BOUNDED_PYTEST_SESSION_DONE pid={process.pid}; report written, "
                f"allowing {SESSION_EXIT_GRACE_S:g}s for a normal exit",
                flush=True,
            )
        if finished_at is not None and now - finished_at >= SESSION_EXIT_GRACE_S:
            print(
                f"BOUNDED_PYTEST_REAPED pid={process.pid}; the session finished and the "
                f"process did not exit within {SESSION_EXIT_GRACE_S:g}s (the known "
                f"teardown hang). Reaping the group; THE EXIT CODE IS DERIVED FROM THE "
                f"REPORT, not from pytest.",
                file=sys.stderr,
                flush=True,
            )
            cleaned = _terminate_process_group(process, grace_seconds)
            if not cleaned:
                return CLEANUP_FAILURE_EXIT_CODE
            assert junit_path is not None
            return _verdict_from_junit(junit_path)
        if now >= deadline:
            raise subprocess.TimeoutExpired(process.args, timeout_seconds)
        time.sleep(0.05)


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

    process: subprocess.Popen[bytes] | None = None
    previous_handlers = _install_signal_handlers()
    try:
        # GATE ON THE ARTIFACT, THEN REAP THE CORPSE. Measured 2026-08-31: TEN of ten
        # full suites on the Pi exited rc=124 -- the hard deadline -- because pytest
        # prints its summary and then SITS (an asyncio teardown hang that has been on
        # the books since 2026-08-18). Every bounded run paid the full timeout in
        # wall-clock: 300 s of deadline for 150 s of testing. The corpse trap is not
        # intermittent here, it is the normal path.
        #
        # So the wait watches for the session's own ARTIFACT rather than for the
        # process to die. `--junitxml` is written in pytest's session-finish hook,
        # BEFORE the interpreter hangs, so its appearance means "the testing is done"
        # even when the process will never exit. This is the same two-step an operator
        # does by hand -- read the summary, then kill by pid -- encoded.
        #
        # It does NOT touch the stuck path: a pytest that hangs without finishing
        # writes no artifact and still hits the deadline exactly as before.
        junit_path: Path | None = None
        args = list(pytest_args)
        if not any(str(a).startswith("--junitxml") for a in args):
            junit_path = Path(tempfile.mkdtemp(prefix="bounded-pytest-")) / "session.xml"
            args.append(f"--junitxml={junit_path}")
        command = [sys.executable, "-m", "pytest", *args]
        process = subprocess.Popen(command, start_new_session=True)
        print(
            f"BOUNDED_PYTEST_START pid={process.pid} timeout_s={timeout_seconds:g} "
            f"lock={lock_path} command={shlex.join(command)}",
            flush=True,
        )
        try:
            return_code = _wait_for_session(
                process, junit_path, timeout_seconds, grace_seconds)
        except subprocess.TimeoutExpired:
            print(
                f"BOUNDED_PYTEST_TIMEOUT pid={process.pid} timeout_s={timeout_seconds:g}; "
                "terminating process group; result is NO VERDICT",
                file=sys.stderr,
                flush=True,
            )
            cleaned = _terminate_process_group(process, grace_seconds)
            return TIMEOUT_EXIT_CODE if cleaned else CLEANUP_FAILURE_EXIT_CODE
        except KeyboardInterrupt:
            print(
                f"BOUNDED_PYTEST_INTERRUPTED pid={process.pid}; terminating process group; "
                "result is NO VERDICT",
                file=sys.stderr,
                flush=True,
            )
            cleaned = _terminate_process_group(process, grace_seconds)
            return INTERRUPTED_EXIT_CODE if cleaned else CLEANUP_FAILURE_EXIT_CODE
        except _ExternalTermination as exc:
            print(
                f"BOUNDED_PYTEST_SIGNAL pid={process.pid} signal={exc.signum}; "
                "terminating process group; result is NO VERDICT",
                file=sys.stderr,
                flush=True,
            )
            cleaned = _terminate_process_group(process, grace_seconds)
            return 128 + exc.signum if cleaned else CLEANUP_FAILURE_EXIT_CODE

        if _process_group_exists(process.pid):
            print(
                f"BOUNDED_PYTEST_LEAK pid={process.pid}; pytest exited but descendants remain; "
                "terminating process group; result is NO VERDICT",
                file=sys.stderr,
                flush=True,
            )
            _terminate_process_group(process, grace_seconds)
            return CLEANUP_FAILURE_EXIT_CODE
        return return_code
    except _ExternalTermination as exc:
        if process is not None:
            _terminate_process_group(process, grace_seconds)
        return 128 + exc.signum
    finally:
        _restore_signal_handlers(previous_handlers)
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


def _enforce_verbose(pytest_args: list[str], parser: argparse.ArgumentParser) -> list[str]:
    quiet_option = re.compile(r"^-[A-Za-z]*q[A-Za-z]*$")
    if any(arg == "--quiet" or quiet_option.fullmatch(arg) for arg in pytest_args):
        parser.error("quiet pytest output is prohibited; use -vv so a timeout identifies the last test")
    verbosity = 0
    for arg in pytest_args:
        if arg == "--verbose":
            verbosity += 1
        elif re.fullmatch(r"-v+", arg):
            verbosity += len(arg) - 1
    if verbosity < 2:
        pytest_args.insert(0, "-" + "v" * (2 - verbosity))
    return pytest_args


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
    pytest_args = _enforce_verbose(pytest_args, parser)
    return run_pytest(
        pytest_args,
        timeout_seconds=args.timeout,
        grace_seconds=args.term_grace,
        lock_path=args.lock_file,
    )


if __name__ == "__main__":
    raise SystemExit(main())
