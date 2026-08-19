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

# The 2026-08-18 spin-up batch moved every post-launch gate into ONE resident
# rclpy probe (bringup_gates.py) -- same gates, no per-gate CLI spawn, no fixed
# settle sleep. The gates did not get weaker; they got a faster vehicle. These
# tests hold BOTH files' shapes: the caller must refuse when the probe dies, and
# the probe must still carry every gate concept the old roster carried.
PROBE = Path(__file__).resolve().parents[1] / "scripts" / "bringup_gates.py"
PROBE_SRC = PROBE.read_text()
PROBE_TREE = ast.parse(PROBE_SRC)

# The flight launch itself, because the watcher default is now a RATIFIED value
# (Scott, 2026-08-19): a silent revert must fail a test, not a flight.
LAUNCH = Path(__file__).resolve().parents[1] / "launch" / "explore.launch.py"
LAUNCH_SRC = " ".join(LAUNCH.read_text().split())


def function(name, tree=None):
    for node in ast.walk(tree or TREE):
        if isinstance(node, ast.FunctionDef) and node.name == name:
            return node
    raise AssertionError(f"{name}() is missing")


def calls_in(node):
    return {getattr(c.func, "id", None) or getattr(c.func, "attr", None)
            for c in ast.walk(node) if isinstance(c, ast.Call)}


# --- the gates must all be on the arming path ---------------------------------------

REQUIRED_GATES = [
    "gate_verify",        # HEAD==origin, tree clean, installed tree byte-matches
    "gate_preflight",     # chassis, clock, graph, installed tree
    "run_gate_probe",     # EVERY post-launch gate, in one resident process
]

#: Every gate concept the pre-probe roster carried, by the probe's own fail() tag.
#: A concept missing here is a check that silently stopped existing -- the exact
#: drift this file exists to catch.
PROBE_GATE_CONCEPTS = [
    "lifecycles",   # slam + the nav2 four say ACTIVE themselves (stock)
    "params",       # the safety constants, read from the ROBOT
    "tof",          # the brake's only producer, and its rate band
    "brake",        # the D39 hold must not already be engaged
    "disarmed",     # D29: bringup is disarmed (bespoke)
    "marks",        # the touch port's producer is on the wire (stock)
    "battery",      # read aloud at connect, 25% floor (stock)
    "recording",    # growth, not existence
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


def test_every_probe_gate_concept_survived_the_move():
    """The probe replaced the CLI gate roster; each old gate concept must appear as
    a fail() tag in the probe -- a gate with no failure path did not move, it died."""
    fail_tags = set()
    for node in ast.walk(PROBE_TREE):
        if (isinstance(node, ast.Call)
                and getattr(node.func, "id", None) == "fail"
                and node.args and isinstance(node.args[0], ast.Constant)):
            fail_tags.add(node.args[0].value)
    for concept in PROBE_GATE_CONCEPTS:
        assert concept in fail_tags, (
            f"the probe has no failure path tagged {concept!r} -- that gate "
            f"concept silently stopped existing in the move off the CLI roster")


def test_the_probe_dying_refuses_the_bringup():
    """Fail-closed by exit code: run_gate_probe must die() on a nonzero exit OR a
    missing READY line, and the probe itself must exit through main()'s status."""
    src = ast.get_source_segment(SRC, function("run_gate_probe"))
    assert "rc != 0" in src and "ready is None" in src, (
        "run_gate_probe no longer checks both the exit code and the READY line")
    assert "sys.exit(main())" in PROBE_SRC, (
        "bringup_gates.py no longer exits through main()'s status -- a failed "
        "gate would exit 0 and the caller would arm on it")


def test_the_ready_line_is_machine_parseable_with_the_sha():
    """The READY line is the contract with whatever mission layer sits above this
    verb: one line, json payload, carrying the sha the verify phase pinned."""
    assert 'print("READY " + json.dumps' in PROBE_SRC
    main_src = ast.get_source_segment(SRC, function("main"))
    assert 'print("READY " + json.dumps' in main_src
    assert '"sha"' in main_src, "the caller's READY payload lost the sha"


def test_the_verify_phase_checks_all_three_authorities():
    """SHA against ORIGIN (not the clone's idea of itself), a clean tree, and a
    byte-compared installed tree -- each catches a different way of flying
    unreviewed code."""
    src = ast.get_source_segment(SRC, function("gate_verify"))
    assert "fetch origin" in src, "gate_verify no longer fetches -- it would " \
        "compare HEAD against a stale ref and call it verified"
    assert "rev-parse HEAD origin/" in src
    assert "status --porcelain" in src
    assert "filecmp" in src, "the installed-tree byte comparison is gone; a " \
        "clean repo above a stale colcon build runs last week's code"


def test_bringup_never_sleeps():
    """The whole point of the batch: main() polls (via the probe) and never
    sleeps. A time.sleep reappearing in main() is the dead time creeping back."""
    assert "sleep" not in calls_in(function("main")), (
        "main() sleeps again -- the 133 s bringup was 38 s of fixed sleeps that "
        "polling replaced; put waits in the probe as polls with ceilings")


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
    on gauntlet mission 1 (5.4 -> 6.9 Hz). The band moved to the probe with the gate."""
    assert "TOF_RATE_MIN_HZ = 6.5" in PROBE_SRC
    assert "TOF_RATE_MIN_HZ" in ast.get_source_segment(PROBE_SRC,
                                                       function("main", PROBE_TREE))


def test_the_battery_floor_moved_with_its_gate():
    """25% is chassis-run protocol; the probe reads it aloud in its receipt."""
    assert "BATTERY_FLOOR = 0.25" in PROBE_SRC
    assert "BATTERY_FLOOR" in ast.get_source_segment(PROBE_SRC,
                                                     function("main", PROBE_TREE))


def test_the_recording_gate_measures_growth_not_existence():
    """A recorder that opened a file and died leaves a header behind, and 'the file is
    there' has been read as 'it is recording'. Now a poll-with-timeout in the probe:
    same growth semantics, no fixed sleep."""
    src = ast.get_source_segment(PROBE_SRC, function("main", PROBE_TREE))
    assert "size(args.csv) > a_csv" in src and "size(args.bag) > a_bag" in src
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


def test_the_bespoke_command_is_unchanged_plus_the_explicit_stock_ports_off():
    """bespoke stays byte-compatible with what this script always launched, and it
    turns the stock middle's ports OFF explicitly: the touch-port marker, and --
    since the 2026-08-19 ratification flipped the launch default to true -- the
    refusal watcher (ratified for stock only, and bespoke has no contact_marker
    to grant its requests anyway)."""
    cmd = _module().launch_command("bespoke")
    for token in ("start_explore:=true", "use_coverage_explorer:=true",
                  "use_decisive_controller:=true", "start_contact_marker:=false",
                  "start_refusal_watcher:=false"):
        assert token in cmd, f"bespoke command lost {token}"


def test_an_unknown_stack_refuses_rather_than_guessing():
    with pytest.raises(ValueError):
        _module().launch_command("both")


# --- stock-explore: the third mode (explore-on-stock, consensus 2026-08-18) ----------

def test_stock_explore_is_the_stock_middle_plus_the_explorer():
    """The ratified v1: coverage_explorer UNCHANGED, riding the exact stock middle.
    One mode string, one exact command -- never a flag cross-product on stock."""
    mod = _module()
    cmd = mod.launch_command("stock-explore")
    for token in ("start_explore:=true", "use_coverage_explorer:=true",
                  "use_decisive_controller:=false", "enable_imu_fusion:=true",
                  "lean_nav2_stock.yaml", "ros2 pkg prefix"):
        assert token in cmd, f"stock-explore command lost {token}"
    assert "mission_autostart" not in cmd, (
        "the explorer must come up DISARMED (D29); arming is this script's "
        "explicit last act, never a launch default")
    # and plain stock did not silently grow the explorer
    assert "use_coverage_explorer:=false" in mod.launch_command("stock")
    # the OFF override composes with the mode like it does with stock; the
    # baseline says nothing and rides the launch's ratified default (true)
    assert "start_refusal_watcher:=false" in mod.launch_command(
        "stock-explore", no_watcher=True)
    assert "start_refusal_watcher" not in mod.launch_command("stock-explore")


def test_the_stock_explore_bag_is_the_stock_bag_plus_the_mission_lanes():
    """Everything the stock bag records still applies (same middle, same seams),
    plus the mission's own narration -- D44's lesson: a report the recording
    cannot corroborate turns the autopsy back into inference."""
    mod = _module()
    stock, explore = mod.bag_topics("stock"), mod.bag_topics("stock-explore")
    for topic in stock:
        assert topic in explore, f"stock-explore bag lost the stock topic {topic}"
    for topic in ("/coverage_explorer/status", "/coverage_explorer/report"):
        assert topic in explore, f"stock-explore bag lacks {topic}"
        assert topic not in stock, f"plain stock has no explorer; drop {topic}"


def test_the_probe_gates_stock_explore_as_stock_plus_disarmed():
    """The mode's gate roster is the UNION: the stock middle's goal-path
    lifecycles AND the explorer's D29 disarmed gate."""
    assert '"stock-explore": ["slam_toolbox", "planner_server", "controller_server"' \
        in PROBE_SRC.replace("\n                      ", " "), (
            "the probe's stock-explore lifecycle roster is not the stock five")
    assert '("bespoke", "stock-explore")' in PROBE_SRC, (
        "the probe no longer runs the disarmed gate for stock-explore")
    assert '("stock", "stock-explore")' in PROBE_SRC, (
        "the probe no longer runs marks/battery for stock-explore")


def test_only_plain_stock_takes_the_never_arms_early_return():
    """stock NEVER arms (liftoff belongs to the goal tool); stock-explore MUST
    fall through to the mission/start arm. The guard has to be exact equality --
    a membership test that catches 'stock-explore' would silently make the new
    mode unarmable, which reads as a hang at the end of a clean bringup."""
    main_src = ast.get_source_segment(SRC, function("main"))
    assert 'args.stack == "stock":' in main_src, (
        "the stock early-return guard is no longer exact equality on 'stock'")


# --- the stock gate roster and the never-arms rule -----------------------------------

def test_the_stock_lifecycle_roster_is_the_goal_path():
    """slam_toolbox is the card's P2 MUST (launched without autostart it sits
    unconfigured looking exactly like a healthy node); the nav2 four are the goal
    path. The probe must poll ALL of them to ACTIVE -- this roster IS the settle."""
    for node in ("slam_toolbox", "planner_server", "controller_server",
                 "bt_navigator", "behavior_server"):
        assert f'"{node}"' in PROBE_SRC, (
            f"the probe's stock lifecycle roster lost {node} -- a goal sent into "
            f"an inactive node is refused by nothing")


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


def test_the_watcher_default_is_the_ratified_on_with_an_explicit_off_override():
    """Scott ratified the flip on 2026-08-19 (docs/watcher_default_decision_
    2026-08-19.md): the D watcher flies default-ON for the stock middle. This
    test held the OLD default (the D-era ride-along pin) and flips WITH the
    ratification. Three pins: the launch default itself is true (a silent
    revert must fail here, not in a flight); stock rides that default (the
    command says nothing, like contact_marker); and OFF is one explicit,
    deviation-logged flag away -- the memo's own rollback promise."""
    mod = _module()
    assert '"start_refusal_watcher", default_value="true"' in LAUNCH_SRC, (
        "the launch default reverted from the ratified true -- if that was the "
        "watch-item rollback (one false promotion), flip this pin with it and "
        "cite the flight; if not, it is a silent revert")
    assert "start_refusal_watcher" not in mod.launch_command("stock")
    assert "start_refusal_watcher:=false" in mod.launch_command(
        "stock", no_watcher=True)
    assert "start_refusal_watcher:=false" in mod.launch_command("bespoke")


def test_the_stock_bag_records_the_promotion_request_lane():
    """Every watcher firing -- including ones the marker rejects -- must be
    reconstructable from the bag alone."""
    assert "/contact_marks/promote" in _module().bag_topics("stock")
