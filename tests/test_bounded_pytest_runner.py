from __future__ import annotations

import fcntl
import importlib.util
import os
import re
import shlex
import signal
import subprocess
import sys
import time
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
RUNNER = ROOT / "scripts" / "run_pytest_bounded.py"


@pytest.fixture(scope="module")
def runner_module():
    """The runner imported as a module, for the parts that are pure predicates.

    Everything else in this file drives the runner as a SUBPROCESS, which is right for
    behaviour that only exists across a process boundary (signals, groups, exit codes).
    The machine-state scan is a pure function over a directory tree, so it is tested
    directly -- and can therefore be tested against a FAKE `/proc`, which is the only
    way to plant a foreign session without starting one.
    """
    spec = importlib.util.spec_from_file_location("run_pytest_bounded", RUNNER)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _run(*args: str, timeout: float = 10.0) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [sys.executable, str(RUNNER), *args],
        cwd=ROOT,
        text=True,
        capture_output=True,
        timeout=timeout,
        check=False,
    )


def _wait_group_gone(process_group: int) -> None:
    for _ in range(40):
        try:
            os.killpg(process_group, 0)
        except ProcessLookupError:
            return
        time.sleep(0.05)
    raise AssertionError(f"pytest process group {process_group} still exists")


def _write_hang_test(path: Path, *, resistant_child: bool = False) -> None:
    child = "import time; time.sleep(60)"
    if resistant_child:
        child = "import signal,time; signal.signal(signal.SIGTERM, signal.SIG_IGN); time.sleep(60)"
    path.write_text(
        "import subprocess\n"
        "import sys\n"
        "import time\n\n"
        "def test_hang():\n"
        f"    subprocess.Popen([sys.executable, '-c', {child!r}])\n"
        "    time.sleep(60)\n",
        encoding="utf-8",
    )


def test_bounded_runner_preserves_passing_pytest_exit_and_exact_command(tmp_path: Path) -> None:
    test_file = tmp_path / "test pass.py"
    test_file.write_text("def test_pass():\n    assert True\n", encoding="utf-8")
    lock_path = tmp_path / "pytest.lock"

    result = _run("--timeout", "5", "--lock-file", str(lock_path), "--", str(test_file))

    assert result.returncode == 0, result.stdout + result.stderr
    assert "1 passed" in result.stdout
    expected = shlex.join([sys.executable, "-m", "pytest", "-vv", str(test_file)])
    assert f"command={expected}" in result.stdout
    assert "BOUNDED_PYTEST_TIMEOUT" not in result.stderr


def test_bounded_runner_preserves_failing_pytest_exit(tmp_path: Path) -> None:
    test_file = tmp_path / "test_fail.py"
    test_file.write_text("def test_fail():\n    assert False\n", encoding="utf-8")

    result = _run("--timeout", "5", "--lock-file", str(tmp_path / "pytest.lock"), "--", str(test_file))

    assert result.returncode == 1
    assert "1 failed" in result.stdout


def test_bounded_runner_times_out_and_kills_sigterm_resistant_descendant(tmp_path: Path) -> None:
    test_file = tmp_path / "test_hang.py"
    _write_hang_test(test_file, resistant_child=True)

    started = time.monotonic()
    result = _run(
        "--timeout",
        "1",
        "--term-grace",
        "0.2",
        "--lock-file",
        str(tmp_path / "pytest.lock"),
        "--",
        str(test_file),
    )
    elapsed = time.monotonic() - started

    assert result.returncode == 124, result.stdout + result.stderr
    assert elapsed < 10
    assert "NO VERDICT" in result.stderr
    match = re.search(r"BOUNDED_PYTEST_START pid=(\d+)", result.stdout)
    assert match is not None
    _wait_group_gone(int(match.group(1)))


def test_bounded_runner_cleans_group_when_wrapper_receives_sigterm(tmp_path: Path) -> None:
    test_file = tmp_path / "test_hang.py"
    _write_hang_test(test_file)
    lock_path = tmp_path / "pytest.lock"
    wrapper = subprocess.Popen(
        [
            sys.executable,
            str(RUNNER),
            "--timeout",
            "30",
            "--term-grace",
            "0.2",
            "--lock-file",
            str(lock_path),
            "--",
            str(test_file),
        ],
        cwd=ROOT,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    assert wrapper.stdout is not None
    # READ UNTIL THE MARKER, NOT THE FIRST LINE. The runner prints its machine-state
    # header (norm 22: a measurement carries its conditions) before starting the group,
    # so the START marker is no longer line 1. Position was never the contract.
    start_line = ""
    match = None
    for _ in range(10):
        line = wrapper.stdout.readline()
        if not line:
            break
        start_line += line
        match = re.search(r"BOUNDED_PYTEST_START pid=(\d+)", line)
        if match:
            break
    assert match is not None, start_line
    process_group = int(match.group(1))

    wrapper.send_signal(signal.SIGTERM)
    stdout, stderr = wrapper.communicate(timeout=10)

    assert wrapper.returncode == 128 + signal.SIGTERM, start_line + stdout + stderr
    assert "BOUNDED_PYTEST_SIGNAL" in stderr
    _wait_group_gone(process_group)
    follow_up = _run("--timeout", "5", "--lock-file", str(lock_path), "--", __file__ + "::test_bounded_runner_rejects_nonpositive_deadline")
    assert follow_up.returncode == 0, follow_up.stdout + follow_up.stderr


def test_bounded_runner_rejects_nominal_exit_with_leaked_descendant(tmp_path: Path) -> None:
    test_file = tmp_path / "test_leak.py"
    test_file.write_text(
        "import signal\n"
        "import subprocess\n"
        "import sys\n\n"
        "def test_leak():\n"
        "    subprocess.Popen([sys.executable, '-c', "
        "'import signal,time; signal.signal(signal.SIGTERM, signal.SIG_IGN); time.sleep(60)'])\n",
        encoding="utf-8",
    )

    result = _run(
        "--timeout",
        "5",
        "--term-grace",
        "0.2",
        "--lock-file",
        str(tmp_path / "pytest.lock"),
        "--",
        str(test_file),
    )

    assert result.returncode == 125, result.stdout + result.stderr
    assert "BOUNDED_PYTEST_LEAK" in result.stderr
    match = re.search(r"BOUNDED_PYTEST_START pid=(\d+)", result.stdout)
    assert match is not None
    _wait_group_gone(int(match.group(1)))


def test_bounded_runner_rejects_nonpositive_deadline() -> None:
    result = _run("--timeout", "0")

    assert result.returncode == 2
    assert "--timeout must be positive" in result.stderr


def test_bounded_runner_rejects_quiet_mode() -> None:
    result = _run("--", "-q")

    assert result.returncode == 2
    assert "quiet pytest output is prohibited" in result.stderr


def test_bounded_runner_rejects_concurrent_test_owner(tmp_path: Path) -> None:
    lock_path = tmp_path / "pytest.lock"
    with lock_path.open("a+") as lock:
        fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        result = _run("--lock-file", str(lock_path), "--", __file__ + "::test_bounded_runner_rejects_nonpositive_deadline")

    assert result.returncode == 75
    assert "BOUNDED_PYTEST_CONCURRENT" in result.stderr
    assert "NO VERDICT" in result.stderr


# THESE THREE PASS THEIR OWN --lock-file, AND THAT IS NOT COSMETIC. The runner takes a
# REPOSITORY lock, so a test that invokes it from inside a suite which is itself running
# under the runner contends with the outer lock and gets NO VERDICT (exit 75). On the
# Mac, where the suite is usually run as bare `pytest`, there is no outer lock and the
# omission is invisible: 1,447 green there, 4-of-4 red on the Pi. Every pre-existing
# test in this file already passed `--lock-file`; I did not copy the idiom, and the
# machine that runs the suite the way it is actually run found it.

def test_bounded_runner_exits_promptly_when_the_session_finished_but_pytest_hangs(
    tmp_path: Path,
) -> None:
    """THE CORPSE TRAP, ENCODED. Measured 2026-08-31: ten of ten full Pi suites exited
    on the hard deadline because pytest prints its summary and then SITS. Every run
    paid the full timeout in wall-clock for testing that had already finished.

    The runner now watches for the session's own junit report -- written in the
    session-finish hook, BEFORE the interpreter hangs -- and reaps the group once it
    appears. This test hangs the interpreter the way the real thing does: a non-daemon
    thread that outlives the session.
    """
    test_file = tmp_path / "test_finished_then_hangs.py"
    test_file.write_text(
        "import threading, time\n"
        "def test_pass():\n"
        "    threading.Thread(target=lambda: time.sleep(120)).start()\n"
        "    assert True\n",
        encoding="utf-8",
    )
    started = time.monotonic()
    result = _run("--timeout", "60", "--lock-file", str(tmp_path / "pytest.lock"),
                  "--", str(test_file), "-vv", timeout=70.0)
    elapsed = time.monotonic() - started
    assert elapsed < 30.0, (
        f"the runner took {elapsed:.1f}s for a session that finished in under a second "
        f"-- it is still waiting for the process instead of the report")
    assert result.returncode == 0, (
        f"a passing session was reaped and reported {result.returncode}")
    assert "BOUNDED_PYTEST_SESSION_DONE" in result.stdout
    assert "DERIVED FROM THE REPORT" in result.stderr, (
        "the exit code changed source and the runner did not say so")


def test_bounded_runner_derives_a_failing_verdict_from_the_report(tmp_path: Path) -> None:
    """The reaped path must not launder a red into a green. Same hang, failing test."""
    test_file = tmp_path / "test_fails_then_hangs.py"
    test_file.write_text(
        "import threading, time\n"
        "def test_fail():\n"
        "    threading.Thread(target=lambda: time.sleep(120)).start()\n"
        "    assert False\n",
        encoding="utf-8",
    )
    result = _run("--timeout", "60", "--lock-file", str(tmp_path / "pytest.lock"),
                  "--", str(test_file), "-vv", timeout=70.0)
    assert result.returncode == 1, (
        f"a FAILING session that hung reported {result.returncode} -- a red must not "
        f"become a green because the process would not exit")


def test_a_genuinely_hung_pytest_still_hits_the_bound(tmp_path: Path) -> None:
    """DO NOT FIX THE FAST PATH BY BREAKING THE STUCK ONE. A pytest that hangs WITHOUT
    finishing writes no report, so there is nothing to gate on and the hard deadline is
    still the only thing that ends it."""
    test_file = tmp_path / "test_hangs_forever.py"
    _write_hang_test(test_file)
    started = time.monotonic()
    result = _run("--timeout", "3", "--lock-file", str(tmp_path / "pytest.lock"),
                  "--", str(test_file), "-vv", timeout=40.0)
    elapsed = time.monotonic() - started
    assert result.returncode != 0, "a hung session must never report success"
    assert "BOUNDED_PYTEST_TIMEOUT" in result.stderr, (
        f"the deadline path did not run: rc={result.returncode} stderr={result.stderr[:300]}")
    assert elapsed >= 3.0, "it did not wait for the deadline at all"


# --- norm 22: the runner states the conditions, and can refuse to measure under bad ones


def test_the_scan_does_not_count_itself(runner_module):
    """THE TRAP THAT NEARLY FILED A DEFECT AGAINST THIS FILE.

    A scan for `-m pytest` in `/proc/*/cmdline` matches the SCANNER, because the string
    is in the scanning process's own command line. On 2026-08-31 the naive form reported
    "3 sessions before, 3 after" and came one step from convicting `run_pytest_bounded`
    of leaking its children; the runner was innocent on both exit paths.

    This test runs inside pytest, so the scanner's own ancestry IS a pytest session --
    which makes it the perfect fixture: if ancestry exclusion is dropped, this test
    finds itself.
    """
    found = runner_module.foreign_pytest_sessions()
    if found is None:
        pytest.skip("no /proc on this machine; the scan cannot run here")
    mine = runner_module._own_ancestry()
    assert all(pid not in mine for pid, _ in found), (
        f"the scan returned a process from its own ancestry: {found}")


def test_a_planted_foreign_session_is_found_and_a_stranger_is_not(runner_module, tmp_path):
    """Plant the condition rather than describe it, using a FAKE /proc.

    Two things must both hold or the gate is useless: a foreign pytest session is FOUND
    (or the gate never fires), and an unrelated process is NOT (or the gate fires on
    everything and gets turned off, which is the same as never firing).
    """
    proc = tmp_path / "proc"
    for pid, cmdline in (
        ("4242", "/usr/bin/python3 -m pytest tests/ -vv"),
        ("4243", "/usr/bin/python3 scripts/run_pytest_bounded.py --timeout 600 tests/"),
        ("4244", "/usr/lib/systemd/systemd --user"),
        ("4245", "sshd: jsperson@notty"),
    ):
        d = proc / pid
        d.mkdir(parents=True)
        (d / "cmdline").write_bytes(cmdline.replace(" ", "\0").encode())
    (proc / "not-a-pid").mkdir()

    found = runner_module.foreign_pytest_sessions(proc_root=proc)
    pids = sorted(pid for pid, _ in found)
    assert pids == [4242, 4243], (
        f"expected exactly the two pytest sessions, got {found} -- a gate that also "
        f"fires on sshd is a gate that gets disabled")


def test_no_proc_reports_UNKNOWN_rather_than_clean(runner_module, tmp_path):
    """"Could not tell" must never render as "nothing found".

    The whole failure this gate exists for is a condition that was true and unstated.
    A scan that cannot run returning an empty list would state the opposite of what it
    knows, which is worse than not scanning.
    """
    assert runner_module.foreign_pytest_sessions(proc_root=tmp_path / "absent") is None


def test_require_quiet_refuses_when_it_cannot_check(runner_module, tmp_path, monkeypatch, capsys):
    """--require-quiet on a machine with no /proc must REFUSE, not proceed.

    Opposite ruling from the same fact as the test above, and deliberately so: reporting
    an unknown is honest, but MEASURING under an unknown after being asked for quiet is
    the exact thing that produced today's void numbers.
    """
    monkeypatch.setattr(runner_module, "foreign_pytest_sessions", lambda *a, **k: None)
    rc = runner_module.main(["--require-quiet", "--lock-file", str(tmp_path / "l"),
                             "--", "tests/", "-vv"])
    assert rc == runner_module.FOREIGN_SESSION_EXIT_CODE
    assert "CANNOT BE CHECKED" in capsys.readouterr().out


def test_require_quiet_refuses_and_names_every_offender(runner_module, tmp_path, monkeypatch, capsys):
    """The refusal must NAME them, or the operator cannot act on it -- and the remedy
    must say why the obvious kill does not work, because `pkill -x pytest` matches comm
    and never matched one of these all day."""
    monkeypatch.setattr(runner_module, "foreign_pytest_sessions",
                        lambda *a, **k: [(4242, "python3 -m pytest tests/ -vv")])
    rc = runner_module.main(["--require-quiet", "--lock-file", str(tmp_path / "l"),
                             "--", "tests/", "-vv"])
    out = capsys.readouterr().out
    assert rc == runner_module.FOREIGN_SESSION_EXIT_CODE
    assert "pid=4242" in out
    assert "pkill -x pytest" in out and "comm" in out


def test_the_conditions_are_printed_even_when_nothing_is_wrong(runner_module, monkeypatch, capsys):
    """A clean board is only strong evidence if its conditions were recorded AT THE TIME.

    The zero-red verification run on 2026-08-31 turned out to have been taken beside two
    concurrent suites, which made it a better result than the one reported -- and nobody
    could say so, because the number had never been written down. So the header prints
    unconditionally, including on a quiet machine.
    """
    monkeypatch.setattr(runner_module, "foreign_pytest_sessions", lambda *a, **k: [])
    line = runner_module.machine_state_line()
    assert line.startswith("BOUNDED_PYTEST_MACHINE_STATE")
    assert "load=" in line and "foreign_pytest=0" in line
