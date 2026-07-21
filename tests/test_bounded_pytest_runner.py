from __future__ import annotations

import fcntl
import os
import re
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


def test_bounded_runner_preserves_passing_pytest_exit(tmp_path: Path) -> None:
    test_file = tmp_path / "test_pass.py"
    test_file.write_text("def test_pass():\n    assert True\n", encoding="utf-8")

    result = _run(
        "--timeout",
        "5",
        "--lock-file",
        str(tmp_path / "pytest.lock"),
        "--",
        "-q",
        str(test_file),
    )

    assert result.returncode == 0, result.stdout + result.stderr
    assert "1 passed" in result.stdout
    assert "BOUNDED_PYTEST_START" in result.stdout
    assert "BOUNDED_PYTEST_TIMEOUT" not in result.stderr


def test_bounded_runner_times_out_and_reaps_entire_process_group(tmp_path: Path) -> None:
    test_file = tmp_path / "test_hang.py"
    test_file.write_text(
        "import subprocess\n"
        "import sys\n"
        "import time\n\n"
        "def test_hang():\n"
        "    subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(60)'])\n"
        "    time.sleep(60)\n",
        encoding="utf-8",
    )

    started = time.monotonic()
    result = _run(
        "--timeout",
        "0.5",
        "--term-grace",
        "0.2",
        "--lock-file",
        str(tmp_path / "pytest.lock"),
        "--",
        "-q",
        str(test_file),
    )
    elapsed = time.monotonic() - started

    assert result.returncode == 124, result.stdout + result.stderr
    assert elapsed < 5
    assert "NO VERDICT" in result.stderr
    match = re.search(r"BOUNDED_PYTEST_START pid=(\d+)", result.stdout)
    assert match is not None
    process_group = int(match.group(1))
    for _ in range(20):
        try:
            os.killpg(process_group, 0)
        except ProcessLookupError:
            break
        time.sleep(0.05)
    else:
        raise AssertionError(f"pytest process group {process_group} still exists")


def test_bounded_runner_rejects_nonpositive_deadline() -> None:
    result = _run("--timeout", "0")

    assert result.returncode == 2
    assert "--timeout must be positive" in result.stderr


def test_bounded_runner_rejects_concurrent_test_owner(tmp_path: Path) -> None:
    lock_path = tmp_path / "pytest.lock"
    with lock_path.open("a+") as lock:
        fcntl.flock(lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        result = _run("--lock-file", str(lock_path), "--", "-q")

    assert result.returncode == 75
    assert "BOUNDED_PYTEST_CONCURRENT" in result.stderr
    assert "NO VERDICT" in result.stderr
