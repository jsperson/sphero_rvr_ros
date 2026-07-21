from __future__ import annotations

import fcntl
import os
import re
import shlex
import signal
import subprocess
import sys
import time
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
RUNNER = ROOT / "scripts" / "run_pytest_bounded.py"


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
    start_line = wrapper.stdout.readline()
    match = re.search(r"BOUNDED_PYTEST_START pid=(\d+)", start_line)
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
