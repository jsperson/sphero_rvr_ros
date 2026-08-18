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


# --- the recording must capture the seam it is used to reason across ------------------

# 2026-08-16: mission 1's bag held 41 commands at 0.4 rad/s on /cmd_vel_motor and an
# /odom that never moved, and could not say whether the driver ever wrote a motor packet.
# rvr_node publishes precisely that on /diagnostics and always did. The bag simply did
# not record it, so the autopsy inferred across the driver's seam and blamed a pivot
# ceiling that a floor measurement later exonerated. A recording that omits the owner's
# own statement of fact turns every question at that seam into an inference.
REQUIRED_BAG_TOPICS = [
    "/cmd_vel",              # what Nav2 asked for
    "/cmd_vel_motor",        # what the supervisor passed to the driver's door
    "/diagnostics",          # WHETHER THE DRIVER ACTED ON IT -- the seam
    "/collision_stop/state",
    "/odom",
    "/scan",
]


def _module():
    """Import the script as a module (it has no side effects at import)."""
    import sys

    sys.path.insert(0, str(SCRIPT.parent))
    try:
        import launch_and_arm
        return launch_and_arm
    finally:
        sys.path.pop(0)


def test_the_bag_records_both_sides_of_the_driver_seam():
    """Formerly a source grep around the record command; the topics now live in the
    pure bag_topics(), so the assertion moved to the actual list -- strictly
    stronger, and it holds for BOTH stacks."""
    mod = _module()
    for stack in ("stock", "bespoke"):
        topics = mod.bag_topics(stack)
        for topic in REQUIRED_BAG_TOPICS:
            assert topic in topics, (
                f"{topic} is not in the {stack} bag list. A mission recording that "
                "cannot answer a question about a seam is the reason an autopsy "
                "convicts the wrong component."
            )


def test_the_stock_bag_observes_the_layers_and_the_touch_port():
    """Run 3d's horizontal-leg analysis had to INFER tof_layer paint/clear because
    the costmaps were not in the bag, and the touch port's receipt is /contact_marks
    going 0 -> nonzero. The stock list must carry all of it."""
    topics = _module().bag_topics("stock")
    for topic in (
        "/contact_marks", "/plan",
        "/local_costmap/costmap_raw", "/local_costmap/costmap_raw_updates",
        "/global_costmap/costmap_raw", "/global_costmap/costmap_raw_updates",
    ):
        assert topic in topics, f"{topic} missing from the stock bag list"


# --- the two stacks' launch commands are pinned --------------------------------------

def test_the_stock_command_is_the_flown_3c_shape():
    """The stock middle's proven invocation: no explorer, no decisive controller,
    lean_nav2_stock resolved from the DEPLOYED share (ros2 pkg prefix, not a source
    path), fusion per the protocol's standing default."""
    cmd = _module().launch_command("stock")
    for token in ("start_explore:=false", "use_decisive_controller:=false",
                  "use_coverage_explorer:=false", "enable_imu_fusion:=true",
                  "lean_nav2_stock.yaml", "ros2 pkg prefix"):
        assert token in cmd, f"stock command lost {token}"
    assert "mission_autostart" not in cmd
    assert _module().launch_command("stock", imu_fusion=False).count(
        "enable_imu_fusion:=false") == 1


def test_the_bespoke_command_is_unchanged_plus_an_explicit_marker_off():
    """bespoke stays byte-compatible with what this script always launched, and it
    turns the touch-port marker OFF explicitly (the launch default is true for the
    stock middle; an idle marker costs ~14% of a Pi core)."""
    cmd = _module().launch_command("bespoke")
    for token in ("start_explore:=true", "use_coverage_explorer:=true",
                  "use_decisive_controller:=true", "start_contact_marker:=false"):
        assert token in cmd, f"bespoke command lost {token}"


def test_an_unknown_stack_refuses_rather_than_guessing():
    with pytest.raises(ValueError):
        _module().launch_command("both")


# --- the stock gate roster and the never-arms rule -----------------------------------

STOCK_GATES = ["gate_lifecycles_active", "gate_marks_publisher", "gate_battery"]


@pytest.mark.parametrize("gate", STOCK_GATES)
def test_main_calls_every_stock_gate(gate):
    assert gate in calls_in(function("main")), f"main() no longer calls {gate}()"


@pytest.mark.parametrize("gate", STOCK_GATES)
def test_every_stock_gate_can_stop_the_run(gate):
    assert "die" in calls_in(function(gate)), (
        f"{gate}() has no failure path -- decoration on the bringup path")


def test_stock_mode_never_reaches_the_arm_call():
    """For the stock middle, arming IS the goal and the goal tool owns it. The
    stock branch must return before the mission/start call in source order --
    a stock bringup that can arm the bespoke explorer is a category error."""
    assert SRC.index("STOCK STACK UP") < SRC.index('say("ARM"'), (
        "the stock early-return no longer precedes the arm call")
    assert "fly_stock_goal.py" in SRC, (
        "stock mode no longer points the operator at the goal tool")


# --- every spawned command must survive `exec` ---------------------------------------

# spawn() runs `bash -lc "SETUP && exec CMD"`. `exec` replaces the shell with an
# EXECUTABLE, so a CMD beginning with a shell builtin -- `cd`, `export`, `source` -- dies
# with "exec: cd: not found" and the process never starts. On 2026-08-16 the recorder was
# spawned as `cd {REPO}/diagnostics && python3 run_recorder.py ...`; it was the only spawn
# that used `cd`, and it had therefore NEVER run. The first real mission use brought the
# entire stack up with no recorder at all.
SHELL_BUILTINS = ("cd", "export", "source", "alias", "set", "unset", "eval")


def spawn_command_literals():
    """The first f-string literal of every spawn() call -- where the command starts."""
    commands = []
    for node in ast.walk(TREE):
        if not isinstance(node, ast.Call):
            continue
        if (getattr(node.func, "id", None) or getattr(node.func, "attr", None)) != "spawn":
            continue
        arg = node.args[0] if node.args else None
        if isinstance(arg, ast.JoinedStr):
            for value in arg.values:
                if isinstance(value, ast.Constant) and value.value.strip():
                    commands.append(value.value.strip())
                    break
        elif isinstance(arg, ast.Constant):
            commands.append(arg.value.strip())
    return commands


def test_no_spawned_command_starts_with_a_shell_builtin():
    commands = spawn_command_literals()
    assert commands, "no spawn() commands found -- the AST walk is broken, not the script"
    for command in commands:
        head = command.split()[0]
        assert head not in SHELL_BUILTINS, (
            f"spawn({command!r}...) starts with the shell builtin {head!r}. "
            "spawn() execs its command, so this process will never start -- silently, "
            "while the mission proceeds believing it did."
        )


def test_no_spawned_command_chains_with_a_shell_operator():
    # `exec` takes one command. `&&`, `;` and `|` in a spawn are a sign the command is
    # being written as a shell script that exec cannot honour.
    for command in spawn_command_literals():
        for operator in ("&&", ";", "|"):
            assert operator not in command, (
                f"spawn({command!r}...) uses {operator!r}; exec runs a single executable."
            )
