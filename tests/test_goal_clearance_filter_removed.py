"""The goal-clearance filter is GONE, and must not come back.

It was removed rather than retuned because its defect is structural, not numeric.
A frontier borders unknown space **by definition**, and unknown cells are precisely
what a clearance probe cannot judge — so frontier goals sit at the bottom of the
clearance distribution as a matter of geometry, not of clutter. Measured on the live
costmap during gauntlet run 1 (2026-08-11):

  * frontier population median clearance **0.05 m**, the probe floor;
  * **0 of 125** frontiers passed the deployed 0.35 m;
  * even at 0.10 m only 9% passed.

A filter that rejects frontier goals makes frontier exploration impossible in any
room, on any robot, at any threshold above the grid resolution. There is no value to
tune it to. It was neutralised to `0.0` in config mid-gauntlet as the smallest safe
diff; this is the removal that was owed afterwards.

WHAT THIS FILE IS GUARDING AGAINST, precisely: not a typo, but a plausible-sounding
future change. "Never send the rover somewhere it cannot leave" is a *good sentence*.
It reads as prudence, it has a real incident behind it (run 185048: a pose with 0.22 m
on two sides, unplannable and unescapable), and acting on it here is what produced a
filter that killed 100% of frontiers. The incident is real; the remedy belongs in the
stall ladder, which recovers from a pose the rover is ACTUALLY in, rather than in a
predicate over poses it might go to.

WHY THESE ARE SOURCE-LEVEL ASSERTIONS. The behavioural proof — a real explorer node
offering a frontier goal whose clearance is at the floor — needs rclpy and lives in
`test_coverage_explorer_mission.py`, which SKIPS on the Mac. These run everywhere, on
every commit, including on the machine where the change would actually be typed. Same
pattern and same reasoning as `test_camera_authority_removed.py`.
"""

import re
from pathlib import Path

import pytest

import sphero_rvr_core.coverage_exploration as coverage_core

ROOT = Path(__file__).resolve().parents[1]
EXPLORER = ROOT / "src" / "sphero_rvr_driver" / "coverage_explorer_node.py"
EXPLORER_CFG = ROOT / "config" / "coverage_explorer.yaml"


def _explorer_source():
    return EXPLORER.read_text()


def _code_lines(text):
    """Source with comments and docstring prose stripped, so that DESCRIBING the
    removed filter (which this file's own subject matter requires the module docstring
    to do at length) is never mistaken for reinstating it."""
    out = []
    in_doc = False
    for line in text.splitlines():
        stripped = line.strip()
        if stripped.startswith('"""') or stripped.endswith('"""'):
            # crude but sufficient: the explorer's only triple-quoted blocks are
            # docstrings, and a toggle across them is what we want.
            in_doc = not in_doc if stripped.count('"""') % 2 else in_doc
            continue
        if in_doc or stripped.startswith("#"):
            continue
        out.append(line.split("#", 1)[0])
    return "\n".join(out)


# ------------------------------------------------------------------ the parameter


def test_the_parameter_is_not_declared():
    """A declared-but-unread parameter is worse than none: it makes a config key look
    like it controls something."""
    code = _code_lines(_explorer_source())
    assert "min_goal_clearance" not in code, (
        "min_goal_clearance is back in the explorer's code"
    )


def test_the_deployed_config_does_not_set_it():
    """And the key must not linger in the YAML either. Setting an UNDECLARED parameter
    is accepted silently by ROS and read by nobody — a config that looks live and is
    inert, which is the failure mode this project keeps meeting under other names."""
    yaml = pytest.importorskip("yaml")
    raw = yaml.safe_load(EXPLORER_CFG.read_text())

    found = []

    def walk(node, path=""):
        if isinstance(node, dict):
            for k, v in node.items():
                if k == "min_goal_clearance_m":
                    found.append(f"{path}/{k}")
                walk(v, f"{path}/{k}")

    walk(raw)
    assert not found, f"min_goal_clearance_m is back in the deployed config at {found}"


# ------------------------------------------------------------------ the mechanism


def test_no_clearance_predicate_stands_between_a_candidate_and_the_planner():
    """THE PROPERTY, not the parameter name.

    Renaming the threshold would defeat a pure `min_goal_clearance` grep, so this
    asserts the shape instead: nothing in the selection loop may compute a clearance
    and `continue` on it. The planner is the only gate a candidate faces.
    """
    code = _code_lines(_explorer_source())
    assert "pose_clearance" not in code, (
        "a pose-clearance probe is back in the explorer"
    )
    assert not re.search(r"clearance\s*[<>]", code), (
        "a clearance comparison is back in the explorer's selection path"
    )


def test_the_approach_loop_has_EXACTLY_ONE_gate_and_it_is_the_planner():
    """THE DEFEAT-RESISTANT ONE. Read this before trusting the two above it.

    Every assertion in this file that greps for a NAME is defeatable by choosing a
    different name, and that is not hypothetical: while building this guard I
    reintroduced the filter as `room = self._pose_room(...); if room < 0.10: continue`
    and all six name-based tests passed. A guard that its own author can walk around
    in thirty seconds is the "defeatable check" failure mode, and finding it is the
    entire reason mutation testing runs against verification artifacts too.

    So this asserts the SHAPE of the approach-point loop with the parser rather than
    the spelling of its identifiers. Any filter whatsoever — under any name, reading
    any source — has to do one of two things to take effect:

      * `continue` (or `break`) past the planner call, or
      * make reaching the planner conditional on something else.

    Both are visible in the AST regardless of naming. The loop must contain exactly
    one `if`, that `if` must test a bare call to `_planner_can_reach`, and the loop
    body must contain no `continue` at all.
    """
    import ast

    tree = ast.parse(_explorer_source())

    loops = [
        node for node in ast.walk(tree)
        if isinstance(node, ast.For)
        and isinstance(node.iter, ast.Call)
        and isinstance(node.iter.func, ast.Attribute)
        and node.iter.func.attr == "_approach_points"
    ]
    assert len(loops) == 1, f"expected one approach-point loop, found {len(loops)}"
    loop = loops[0]

    continues = [n for n in ast.walk(loop) if isinstance(n, ast.Continue)]
    assert not continues, (
        "the approach-point loop skips candidates before reaching the planner -- "
        "something is filtering goals again, whatever it is called"
    )

    ifs = [n for n in loop.body if isinstance(n, ast.If)]
    assert len(ifs) == 1, (
        f"the approach-point loop has {len(ifs)} branches; exactly one gate is "
        "allowed and it must be the planner"
    )

    test = ifs[0].test
    assert isinstance(test, ast.Call) and isinstance(test.func, ast.Attribute), (
        "the loop's only gate is no longer a bare call -- a condition has been "
        "added alongside the planner query"
    )
    assert test.func.attr == "_planner_can_reach", (
        f"the loop's gate is {test.func.attr!r}, not the planner"
    )
    # RESIDUAL GAP, measured and left open on purpose. A filter buried INSIDE
    # `_planner_can_reach` itself -- an early `return False` on a clearance probe --
    # defeats every assertion in this file, and was verified to do so. It is not
    # closed here because that method legitimately returns False for action-server
    # timeouts and unavailable servers, so a shape assertion over its body would fire
    # on correct changes and be deleted within the month, leaving nothing. The guard
    # this file DOES provide is that the filter cannot come back where it was, under
    # any name; hiding it inside the planner query is a different and more deliberate
    # act, and the honest statement is that it would pass.


def test_the_core_no_longer_ships_a_pose_clearance_probe():
    """The pure helper is deleted too, and that deletion is part of the guard rather
    than tidiness. Left in place it was a loaded gun: an unused, fully-tested function
    whose docstring told the reader it "lets a caller decline to send the rover
    somewhere it will not be able to leave". The next person to want that sentence
    would have found a working implementation and wired it back up in one line.
    """
    assert not hasattr(coverage_core, "pose_clearance_m")


def test_the_start_pose_guard_SURVIVES():
    """The removal must not take the start-pose check with it, and the distinction is
    the whole point: `robot_start_blocked` asks whether the robot's OWN current cell
    is wedged. It is not a filter over candidates, so it has no anti-frontier
    property — the rover's own pose is not a frontier. Deleting it because it is
    costmap-shaped would trade one structural bug for a different one (2026-08-07:
    0.26 m rear clearance, every goal unplannable, a four-minute run burned).
    """
    assert hasattr(coverage_core, "robot_start_blocked")
    assert hasattr(coverage_core, "INSCRIBED_COST")

    code = _code_lines(_explorer_source())
    assert "robot_start_blocked" in code
    assert "blocked_start_check" in code
    # And the costmap subscription it reads must still exist.
    assert "_on_costmap" in code and "costmap_topic" in code


def test_the_reasoning_survives_where_the_next_author_will_look():
    """A deleted mechanism leaves no trace at its own call site, so the measurement
    that killed it has to live somewhere a future author reads BEFORE re-adding it.
    Pinned here so a tidy-up that strips the explanation fails: the 0/125 result is
    the entire argument, and without it "add a clearance check" looks reasonable.
    """
    doc = _explorer_source()
    assert "125" in doc, "the 0-of-125-frontiers measurement is no longer recorded"
    assert "anti-frontier" in doc
    assert "blocked_start_check" in doc, (
        "the docstring no longer distinguishes the surviving start-pose guard from "
        "the removed candidate filter -- the distinction a re-adder needs most"
    )
