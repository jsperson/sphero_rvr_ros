"""SCENARIO 8 of Scott's nine, 2026-08-31.

  REQUIREMENT, his words: "Accept general instructions like 'explore and map this
  room', locate a specific object, and 'drive around looking busy for 10 min', as
  well as specific ones: 'move forward 2 metres', turn right/left 90 deg, pivot
  180 deg and drive forward 1 m."

  BAR, PRE-REGISTERED: all SIX named classes dispatch to a tool call.

  PREDICTION FILED BEFORE EXECUTION: FAIL, 3 of 6.

WHAT THIS MEASURES, AND WHAT IT DELIBERATELY DOES NOT. This is a CAPABILITY audit
of `TOOL_SCHEMAS`, not a language test: it asks whether a tool exists that can
express each instruction, never whether the model picks it. Running the model would
measure the model on the day it ran; the tool surface is the thing that either can
or cannot represent what Scott asked for, and it is the thing we control.

That distinction matters for the score. A class marked missing here is missing from
the ROBOT, not merely misunderstood by a prompt -- no amount of prompt work adds a
verb that does not exist.
"""

from __future__ import annotations

import pytest

from sphero_rvr_core.task_agent import ARG_BOUNDS, TOOL_SCHEMAS

#: Scott's six classes, each mapped to the tool that would have to carry it.
CLASSES = {
    "explore and map this room": "explore",
    "locate a specific object": "look_and_recognize",
    "drive around looking busy for 10 min": None,      # no such verb
    "move forward 2 metres": None,                     # no relative motion
    "turn right/left 90 degrees": "turn",
    "pivot 180 degrees and drive forward 1 m": None,   # needs relative motion
}


def test_scenario_8_the_three_classes_that_are_expressible_today():
    """The positive half, asserted so the score is not just a list of absences."""
    for phrase, tool in CLASSES.items():
        if tool is None:
            continue
        assert tool in TOOL_SCHEMAS, f"{phrase!r} needs {tool!r}, which is gone"


@pytest.mark.xfail(strict=True, reason=(
    "SCENARIO 8: 3 of Scott's 6 instruction classes have no tool that can express "
    "them. Strict xfail so the bar stays at SIX rather than at three, and so the day "
    "a relative-motion verb lands this test fails loudly and forces a re-score."))
def test_scenario_8_all_six_instruction_classes_are_expressible():
    missing = [phrase for phrase, tool in CLASSES.items() if tool is None]
    assert not missing, (
        "no tool can express: " + "; ".join(missing) +
        f" -- available tools: {sorted(TOOL_SCHEMAS)}")


def test_scenario_8_the_relative_motion_gap_is_closed_in_the_TOOL_SURFACE():
    """THE GAP, AND ITS CLOSING, pinned separately from the score.

    UNTIL 2026-08-31 this test asserted the ABSENCE: `goto` takes absolute map-frame
    x/y, so "forward 2 m" was not a command the robot had -- it was trigonometry the
    agent would do for itself from `where_am_i`, unguarded and untested, and an
    unverified composition is not a verb. It was the smallest of the three gaps and
    the one Scott named twice.

    `move_relative` closed it in the tool surface. THE ROW HAS NOT BEEN RE-SCORED
    YET, DELIBERATELY: the schema existing is not the verb working, and the bar for
    5-of-6 is the wiring proven ON THE PI -- self-call completes, busy semantics
    inherited, inner refusal carried, stale pose refused. Until that passes, the
    six-class xfail below stays exactly where it is. A tool surface is a promise; the
    bench is where it becomes a capability.
    """
    assert "goto" in TOOL_SCHEMAS
    assert set(TOOL_SCHEMAS["goto"]) == {"x", "y"}, (
        "goto's arguments changed -- re-read what relative motion now means")
    assert "move_relative" in TOOL_SCHEMAS, "the relative-motion verb went away again"
    assert set(TOOL_SCHEMAS["move_relative"]) == {"distance_m", "heading_deg"}
    assert ARG_BOUNDS["move_relative"]["distance_m"][0] == 0.15, (
        "the 0.15 m floor is a fact about the deployed stack -- xy_goal_tolerance is "
        "0.12 m, so a shorter move reports SUCCESS with the rover motionless")
