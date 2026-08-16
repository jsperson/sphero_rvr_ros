"""D41: the mission clock starts when the mission ARMS, not when the node is ready.

D29 made the stack come up DISARMED and moved liftoff to a service call. Nothing
re-anchored `duration_s`, which kept measuring from node-ready and over-reported
2.94x on 2026-08-14b (638.2 s against 217.5 s armed), 4.35x on 08-15a, 3.25x on
08-15b. Every derived RATE went with it, which invalidated cross-run coverage-rate
comparison in both directions -- the project's main better-or-worse metric.

The class lesson is standards rule 2: a re-anchoring change obliges re-checking every
quantity measured from the old anchor. These tests are asserted STRUCTURALLY, against
the node's source, because importing the node needs rclpy and this machine has none --
the same approach as the AST import guard, and for the same reason.
"""

import re
from pathlib import Path

import pytest

NODE = (Path(__file__).resolve().parents[1] / "src" / "sphero_rvr_driver"
        / "coverage_explorer_node.py")


@pytest.fixture(scope="module")
def source():
    return NODE.read_text()


def test_the_mission_clock_is_not_stamped_at_node_construction(source):
    """REVERT-PROOF. Fails against the code it indicts.

    `self._mission_start = time.monotonic()` in `__init__` IS the defect -- that line
    is node-ready, and D29 moved the mission's start away from it.
    """
    init = source[source.index("def __init__"):source.index("def _arm(")]
    assert not re.search(r"_mission_start\s*=\s*time\.monotonic\(\)", init), (
        "the mission clock is being stamped during construction again; that is "
        "node-ready, not mission-start, and it is exactly the D41 defect"
    )
    assert re.search(r"self\._mission_start\s*=\s*None", init), (
        "before arming there is no mission clock, and None is what says so -- a "
        "pre-arm timestamp is what made D41 possible"
    )


def test_arming_is_the_only_place_the_clock_starts(source):
    """ONE AUTHOR. D41 happened because a quantity was stamped somewhere other than
    where the mission began; two arming paths stamping it separately would rebuild
    the defect with an extra place to forget."""
    stamps = re.findall(r"self\._mission_start\s*=\s*time\.monotonic\(\)", source)
    assert len(stamps) == 1, f"the mission clock is set in {len(stamps)} places"

    arm = source[source.index("def _arm("):source.index("def _mission_elapsed_s(")]
    assert "self._mission_start = time.monotonic()" in arm
    assert "self._armed = True" in arm


def test_both_routes_in_go_through_arm(source):
    """The service and autostart. A second path setting `_armed` directly would arm a
    mission whose clock never started."""
    assert source.count('self._arm("autostart")') == 1
    assert source.count('self._arm("mission/start service")') == 1
    # `_armed = True` may appear ONLY inside _arm.
    arm = source[source.index("def _arm("):source.index("def _mission_elapsed_s(")]
    assert source.count("self._armed = True") == arm.count("self._armed = True") == 1, (
        "something arms the mission without going through _arm, so it would run with "
        "no mission clock"
    )


def test_every_duration_reported_comes_from_the_elapsed_helper(source):
    """The enumeration, enforced. `_mission_start` feeds exactly two `duration_s=`
    call sites -- the START_BLOCKED report and the terminal report -- and both must
    read through the helper so neither can drift back to a raw subtraction."""
    durations = re.findall(r"duration_s=([^,\n]+)", source)
    assert durations, "no duration_s call sites found; this test has gone stale"
    for expr in durations:
        assert expr.strip() == "self._mission_elapsed_s()", (
            f"duration_s={expr.strip()} bypasses the helper; every reported duration "
            f"must measure from ARM"
        )
    assert len(durations) == 2, (
        f"expected the 2 enumerated call sites, found {len(durations)} -- a new one "
        f"appeared and needs checking against the anchor"
    )


def test_an_unarmed_mission_reports_zero_not_node_uptime(source):
    """A report from a node that never armed describes a mission that did not run.
    The old code would have reported however long the node had been up."""
    helper = source[source.index("def _mission_elapsed_s("):source.index("def _publish_generation(")]
    assert "if self._mission_start is None:" in helper
    assert "return 0.0" in helper


def test_arming_is_LOGGED(source):
    """Disarm has logged itself since D29; arming never did -- so the single instant
    every duration is measured from was the one absent from the launch log, and each
    after-the-fact alignment had to infer it from goal traffic."""
    arm = source[source.index("def _arm("):source.index("def _mission_elapsed_s(")]
    assert re.search(r"get_logger\(\)\.(warn|info)\(", arm), "arming emits no log line"
    assert "MISSION ARMED" in arm


def test_the_costmap_dump_fires_on_the_block_and_cannot_kill_the_node(source):
    """D43's rider. A diagnostic attached to a failure path that can itself raise
    turns 'the planner is blocked' into 'the explorer crashed' -- the rule the
    stuck-survey already follows."""
    assert "self._dump_costmap_window(wx, wy)" in source
    dump = source[source.index("def _dump_costmap_window("):source.index("def _robot_world(")]
    assert "except Exception" in dump, "the dump can raise out of the blocked path"
    assert "map_save_dir" in dump, (
        "the dump must land in the deployed mission directory, which is what the "
        "artifact-collection step actually sweeps up"
    )
