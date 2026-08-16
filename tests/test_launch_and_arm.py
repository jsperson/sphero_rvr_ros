"""The launch-and-arm script must make arming FASTER, never EASIER.

A one-command bringup is a lever on the robot's wheels. The danger is not that it fails
-- it is that it succeeds while a gate did not, because every convenience script drifts
toward "get it moving". These tests hold its SHAPE: no arming path that skips a check,
no check that shrugs, and no camera in default bringup.

Source-level, by AST, because the script's own behaviour needs a robot and its shape
does not.
"""

import ast
import inspect
from pathlib import Path

import pytest

SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "launch_and_arm.py"
SRC = SCRIPT.read_text()
TREE = ast.parse(SRC)


def function(name):
    for node in ast.walk(TREE):
        if isinstance(node, ast.FunctionDef) and node.name == name:
            return node
    raise AssertionError(f"{name}() is missing from launch_and_arm.py")


def calls_in(node):
    return {getattr(c.func, "id", None) or getattr(c.func, "attr", None)
            for c in ast.walk(node) if isinstance(c, ast.Call)}


# --- the gates must all be on the arming path ---------------------------------------

REQUIRED_GATES = [
    "gate_preflight",     # chassis, clock, graph, installed tree
    "gate_params",        # the safety constants, read from the ROBOT
    "gate_tof",           # the brake's only producer, and its rate band
    "gate_brake_state",   # the D39 hold must not already be engaged
    "gate_disarmed",      # D29: bringup is disarmed, and we must be the one to arm
    "gate_recording",     # growth, not existence
]


@pytest.mark.parametrize("gate", REQUIRED_GATES)
def test_main_calls_every_gate(gate):
    """Each gate exists because a run was lost without it. Dropping one from main() is
    the exact way this script becomes a way to skip checks."""
    assert gate in calls_in(function("main")), f"main() no longer calls {gate}()"


def test_every_gate_can_stop_the_run():
    """A check that cannot fail is not a check. Each gate must have a path to die()."""
    for gate in REQUIRED_GATES:
        assert "die" in calls_in(function(gate)), (
            f"{gate}() has no failure path -- it can only pass, which makes it "
            f"decoration on the arming path")


def test_arming_happens_after_the_gates_in_main():
    """Ordering, not just presence. A gate called after mission/start is a gate that
    reads a robot which is already driving."""
    main = function("main")
    lines_of = {}
    for node in ast.walk(main):
        if isinstance(node, ast.Call):
            name = getattr(node.func, "id", None) or getattr(node.func, "attr", None)
            if name in REQUIRED_GATES:
                lines_of.setdefault(name, node.lineno)
    # The ACTUAL arming call, not any string mentioning it. The first version of this
    # took the minimum line of every constant containing "mission/start" and picked
    # argparse's --no-arm HELP TEXT, which sits above the gates. Fourth time in one
    # night that an assertion matched prose instead of code (standards A5).
    arm_line = min(
        call.lineno for call in ast.walk(main)
        if isinstance(call, ast.Call)
        and (getattr(call.func, "id", None) == "sh")
        and any(isinstance(a, ast.Constant) and isinstance(a.value, str)
                and "mission/start" in a.value for a in call.args))
    for gate, line in lines_of.items():
        assert line < arm_line, f"{gate}() runs after mission/start is called"


# --- the charter, and the failure modes that cost runs ------------------------------

def test_bringup_starts_neither_the_camera_nor_the_monocular_detector():
    """Scott's charter: the camera is an intelligence sensor, on demand, never in
    default bringup and never in the safety stack or direct navigation."""
    main_src = ast.get_source_segment(SRC, function("main"))
    assert "camera.launch" not in main_src
    assert "start_low_obstacle" not in main_src


def test_the_tof_rate_gate_uses_the_derived_band():
    """The staleness bound low_obstacle_max_age_s (0.30 s) is derived as ~two frames at
    6.5-7.6 Hz. Below 6.5 Hz two frames exceed the bound, so a single dropped frame ages
    the cloud out and the brake stops looking -- which is what shedding the camera fixed
    on gauntlet mission 1 (5.4 -> 6.9 Hz)."""
    assert "TOF_RATE_MIN_HZ = 6.5" in SRC
    assert "TOF_RATE_MIN_HZ" in ast.get_source_segment(SRC, function("gate_tof"))


def test_the_recording_gate_measures_growth_not_existence():
    """A recorder that opened a file and died leaves a header behind, and 'the file is
    there' has been read as 'it is recording'."""
    src = ast.get_source_segment(SRC, function("gate_recording"))
    assert "sleep" in src and "size" in src
    assert "cam_hold_active" in src, (
        "the header check must confirm the D39 hold's columns exist -- an episode "
        "without them cannot be reconstructed")


def test_teardown_stops_the_lidar_by_service_before_killing_anything():
    """Killing the node leaves the disc spinning ownerless. The service call must come
    first, and it must not be a kill."""
    src = ast.get_source_segment(SRC, function("teardown"))
    assert src.index("/stop_motor") < src.index("killpg"), (
        "the lidar motor must be stopped by service BEFORE processes are signalled")


def test_teardown_never_uses_pkill_f():
    """`pkill -f` matches this script's own command line and has killed an operator's
    ssh session four times. Explicit PIDs only."""
    # Checked against CODE, not the file text: this function's own docstring explains
    # why pkill is forbidden, and a raw-text guard goes red on that explanation and
    # pressures the next person into deleting it (standards A5).
    code_strings = []
    for node in ast.walk(TREE):
        if isinstance(node, ast.Constant) and isinstance(node.value, str):
            code_strings.append(node.value)
    docstrings = set()
    for node in ast.walk(TREE):
        if isinstance(node, (ast.FunctionDef, ast.ClassDef, ast.Module)):
            d = ast.get_docstring(node, clean=False)
            if d:
                docstrings.add(d)
    executable = [s for s in code_strings if s not in docstrings]
    assert not any("pkill" in s for s in executable), (
        "teardown uses pkill; it matches this script's own command line")
    assert "killpg" in SRC


def test_no_arm_mode_exists_and_skips_only_the_arming():
    """The rehearsal path. It must still run every gate -- a dry run that skips checks
    teaches nothing about whether the real run would have passed."""
    assert "--no-arm" in SRC
    main_src = ast.get_source_segment(SRC, function("main"))
    no_arm_at = main_src.index("no_arm")
    for gate in REQUIRED_GATES:
        assert main_src.index(gate + "(") < no_arm_at, (
            f"{gate}() is skipped in --no-arm mode; the rehearsal must gate identically")
