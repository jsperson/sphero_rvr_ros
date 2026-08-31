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

from sphero_rvr_core.task_agent import TOOL_SCHEMAS

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


def test_scenario_8_there_is_no_relative_motion_primitive():
    """THE GAP AS A FINDING, pinned separately from the score.

    `goto` takes absolute map-frame x/y. "Forward 2 m" is therefore not a command the
    robot has -- it is a computation the agent would have to do for itself from
    `where_am_i`, with the yaw trigonometry unguarded and untested. This test states
    the absence rather than the workaround, because a composed workaround that nobody
    has verified is not the same as a verb.

    It is also the SMALLEST of the three gaps and the one Scott named twice ("move
    forward 2 metres", "drive forward 1 m").
    """
    assert "goto" in TOOL_SCHEMAS
    assert set(TOOL_SCHEMAS["goto"]) == {"x", "y"}, (
        "goto's arguments changed -- re-read whether relative motion arrived")
    relative = [name for name in TOOL_SCHEMAS
                if any(k in ("distance", "distance_m", "forward", "forward_m", "metres")
                       for k in TOOL_SCHEMAS[name])]
    assert not relative, (
        f"a relative-motion verb exists after all: {relative} -- scenario 8 must be "
        f"re-scored")
