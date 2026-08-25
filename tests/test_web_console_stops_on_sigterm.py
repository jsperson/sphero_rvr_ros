"""D74: the console must die when it is TOLD to, not when it is killed.

2026-08-25 teardown: `web_console` survived a process-group SIGINT, a direct
SIGINT and a SIGTERM, was still answering HTTP 200 half a minute later, and died
only on SIGKILL. A SIGTERM-deaf process is what systemd WAITS OUT for a full
TimeoutStopUSec -- Scott's "several minutes to shut down".

THE HARNESS HAD TO BE BUILT TO FAIL FIRST. Plain `fake_web_console` exits on
SIGTERM in 0.57 s, because Python's DEFAULT SIGTERM disposition kills the
process -- so it cannot reproduce the defect and would certify nothing. The
defect needs the thing the Pi has and the Mac does not: `rclpy.init()` installs
SIGTERM/SIGINT handlers that ask the ROS context to shut down and know nothing
about a blocking `serve_forever()`. `_RCLPY_SHAPED_HANDLER` below stands in for
exactly that, and with it the unfixed code was measured surviving SIGTERM for
6 s while still serving HTTP 200 (falsifier-before-certifier).

Bounded at every step -- a test that CAN hang WILL hang (norm 13).
"""

from __future__ import annotations

import ast
import os
import signal
import socket
import subprocess
import sys
import time

import pytest

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
EXIT_BOUND_S = 5.0
STARTUP_BOUND_S = 20.0

# Installs a handler of rclpy's SHAPE (runs, returns, does not touch the server)
# and then runs the real console harness through its real entry point.
_RCLPY_SHAPED_HANDLER = """
import runpy, signal, sys
signal.signal(signal.SIGTERM, lambda signum, frame: None)
sys.argv = ["fake_web_console.py", "--port", sys.argv[1]]
runpy.run_path("scripts/fake_web_console.py", run_name="__main__")
"""


def _free_port():
    with socket.socket() as probe:
        probe.bind(("127.0.0.1", 0))
        return probe.getsockname()[1]


def _listening(port):
    with socket.socket() as probe:
        probe.settimeout(0.5)
        return probe.connect_ex(("127.0.0.1", port)) == 0


def test_sigterm_stops_the_console_and_frees_the_port():
    port = _free_port()
    proc = subprocess.Popen(
        [sys.executable, "-c", _RCLPY_SHAPED_HANDLER, str(port)],
        cwd=REPO, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        deadline = time.monotonic() + STARTUP_BOUND_S
        while time.monotonic() < deadline and not _listening(port):
            if proc.poll() is not None:
                pytest.fail(f"console exited before serving (rc={proc.returncode})")
            time.sleep(0.2)
        assert _listening(port), f"console never came up on {port}"

        started = time.monotonic()
        proc.send_signal(signal.SIGTERM)
        try:
            proc.wait(timeout=EXIT_BOUND_S)
        except subprocess.TimeoutExpired:
            pytest.fail(
                f"console SURVIVED SIGTERM for {EXIT_BOUND_S:.0f}s and is still "
                f"serving={_listening(port)} -- this is D74, and systemd will "
                f"wait it out at shutdown")
        took = time.monotonic() - started
        assert took <= EXIT_BOUND_S, f"exited, but took {took:.2f}s"

        # The port is the observation that matters: a process can exit while a
        # child or a stray thread keeps the listener, which is what held 8088
        # against its own replacement.
        freed = time.monotonic() + 2.0
        while time.monotonic() < freed and _listening(port):
            time.sleep(0.1)
        assert not _listening(port), (
            f"process exited but port {port} is still being listened on")
    finally:
        if proc.poll() is None:
            proc.kill()
            proc.wait(timeout=10)


def test_the_stop_handler_is_installed_on_the_shipped_path_too():
    """The harness proves the mechanism; this pins that the SHIPPED node uses it,
    and installs it BEFORE serve_forever takes the main thread.

    Read with `ast`, not by grepping the text: the first version of this assertion
    matched the word "serve_forever" inside the very COMMENT explaining the fix and
    failed on it (Appendix A5, in my own guard, within a minute of writing it).
    """
    source = open(os.path.join(REPO, "src", "sphero_rvr_driver",
                               "web_console_node.py")).read()
    tree = ast.parse(source)
    installs, serves = [], []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        if getattr(node.func, "id", None) == "install_stop_handlers":
            installs.append(node.lineno)
        if getattr(node.func, "attr", None) == "serve_forever":
            serves.append(node.lineno)
    assert installs, "the shipped web_console node no longer installs stop handlers"
    assert serves, "the shipped node no longer calls serve_forever -- re-read this test"
    assert min(installs) < min(serves), (
        f"install_stop_handlers (line {min(installs)}) must run BEFORE "
        f"serve_forever (line {min(serves)}) takes the main thread")
