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
FOREIGN_SESSION_EXIT_CODE = 76


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


# --------------------------------------------------------------------------
# machine state -- norm 22: a measurement carries the conditions it was taken under
# --------------------------------------------------------------------------

def _own_ancestry() -> set[int]:
    """This process and every ancestor.

    A scan for `-m pytest` in `/proc/*/cmdline` MATCHES THE SCANNER, because that
    string is in the scanning process's own command line the moment it is written
    down. That trap has now cost this project three sightings in one day, the last of
    which nearly filed a fabricated defect against this very file. Excluded by
    construction rather than by remembering.
    """
    pids: set[int] = set()
    pid = os.getpid()
    while pid > 1:
        pids.add(pid)
        try:
            stat = Path(f"/proc/{pid}/stat").read_text()
            pid = int(stat.rsplit(")", 1)[1].split()[1])
        except (OSError, IndexError, ValueError):
            break
    return pids


def foreign_pytest_sessions(proc_root: Path = Path("/proc")) -> list[tuple[int, str]] | None:
    """Other pytest sessions on this machine, or None if we cannot tell.

    WHY THIS IS NOT THE LOCK'S JOB. The repository lock catches a COOPERATING runner:
    something that took the lock and is holding it. It cannot see a bare
    `python3 -m pytest` somebody started by hand and walked away from, and it cannot see
    a child orphaned when its parent was killed. On 2026-08-31 five such sessions
    accumulated -- the oldest running 5h08m -- and every rate measured that afternoon,
    INCLUDING THE BASELINE every later number was compared against, was taken beside
    them at load 30 on four cores.

    None means "cannot tell", which on a machine without `/proc` is the honest answer
    and is REPORTED rather than silently treated as clean.
    """
    if not proc_root.is_dir():
        return None
    mine = _own_ancestry()
    found: list[tuple[int, str]] = []
    for entry in proc_root.iterdir():
        if not entry.name.isdigit():
            continue
        pid = int(entry.name)
        if pid in mine:
            continue
        try:
            cmdline = (entry / "cmdline").read_bytes().replace(b"\0", b" ").decode(
                "utf-8", "replace")
        except OSError:
            continue
        if "-m pytest" in cmdline or "run_pytest_bounded" in cmdline:
            found.append((pid, cmdline.strip()[:120]))
    return found


def machine_state_line() -> str:
    """The conditions, printed into the artifact before the run rather than believed.

    Load comes from `os.getloadavg`, which is portable; the session scan needs `/proc`
    and says so when it is absent. Printed unconditionally, including when everything is
    fine -- a clean board is only strong evidence if the conditions it was taken under
    were recorded at the time. The zero-red verification run on 2026-08-31 turned out to
    have been taken under two concurrent suites, which made it a BETTER result than the
    one reported, and nobody could say so because nobody had written the number down.
    """
    try:
        load = "%.2f %.2f %.2f" % os.getloadavg()
    except (OSError, AttributeError):
        load = "unavailable"
    sessions = foreign_pytest_sessions()
    if sessions is None:
        state = "foreign_pytest=UNKNOWN (no /proc)"
    else:
        state = f"foreign_pytest={len(sessions)}"
    return f"BOUNDED_PYTEST_MACHINE_STATE load={load} {state}"


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
    parser.add_argument(
        "--require-quiet", action="store_true",
        help=("refuse to run when another pytest session is already on this machine. "
              "A measurement wants this; an ordinary local run does not, which is why "
              "it is opt-in rather than the default."),
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
    print(machine_state_line(), flush=True)
    if args.require_quiet:
        sessions = foreign_pytest_sessions()
        if sessions is None:
            print("BOUNDED_PYTEST_FOREIGN_UNKNOWN --require-quiet was asked for and this "
                  "machine has no /proc, so the condition CANNOT BE CHECKED; refusing "
                  "rather than measuring under conditions nobody can state.", flush=True)
            return FOREIGN_SESSION_EXIT_CODE
        if sessions:
            for pid, cmdline in sessions:
                print(f"BOUNDED_PYTEST_FOREIGN pid={pid} :: {cmdline}", flush=True)
            print(f"BOUNDED_PYTEST_FOREIGN_REFUSED {len(sessions)} other pytest "
                  "session(s) are running; a rate measured beside them is not a rate. "
                  "Kill them by walking /proc and matching CMDLINE -- `pkill -x pytest` "
                  "matches comm and has never matched one of these.", flush=True)
            return FOREIGN_SESSION_EXIT_CODE
    return run_pytest(
        pytest_args,
        timeout_seconds=args.timeout,
        grace_seconds=args.term_grace,
        lock_path=args.lock_file,
    )


if __name__ == "__main__":
    raise SystemExit(main())
