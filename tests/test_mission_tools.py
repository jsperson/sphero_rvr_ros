"""Track 2 v2 — `explore`, `stop`, `status`. Design: docs/track2_mission_tools.md.

Everything here runs without ROS: the schemas and the prompt are pure data, the
summariser is pure logic, and the node's behaviour is asserted against its source the
same way its safety boundary is. The end-to-end round trip needs a running stack and
lives on the Pi.

The three properties these proofs exist for, each of which is a way the tools could be
green and still lie to a user:

  * `explore` returns when the mission STARTS. A caller that reads ok=true as "the room
    has been explored" is wrong within one second of a ten-minute drive.
  * `stop` is NOT an emergency stop. Someone who says "stop" in an emergency must not be
    left believing they hit a brake.
  * `status` reports STALENESS as the answer. A status tool that serves the last good
    value when the publisher has gone quiet turns the most informative symptom a stuck
    rover has -- silence -- into a reassuring report.
"""
import json
import os
import re
import sys
from pathlib import Path

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from sphero_rvr_core.task_agent import (  # noqa: E402
    SYSTEM_PROMPT, TOOL_SCHEMAS, parse_reply, validate_args,
)
from sphero_rvr_core.task_tools import describe_mission  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
NODE = ROOT / "src" / "sphero_rvr_driver" / "task_node.py"
CLIENT = ROOT / "src" / "sphero_rvr_driver" / "task_client.py"


def _body(text):
    """Executable source: docstrings and comments carry no behaviour, and a promise in
    a comment has satisfied more than one guard in this repo."""
    stripped = text.split('"""', 2)[-1]
    return "\n".join(l for l in stripped.splitlines() if not l.strip().startswith("#"))


# --- the model can reach them, and cannot reach anything else -----------------

def test_the_three_mission_tools_are_callable_by_the_model():
    for tool in ("explore", "stop", "status"):
        assert tool in TOOL_SCHEMAS, f"{tool} is not a tool the agent will accept"
        assert TOOL_SCHEMAS[tool] == {}, (
            f"{tool} grew arguments; the design says none of them need any, and an "
            "argument the node ignores is a lie in the schema")
        assert validate_args(tool, {}) == {}


def test_an_argument_the_node_would_ignore_is_REFUSED_not_dropped():
    """A model that hallucinates `explore(area="kitchen")` must be told, not silently
    obeyed with the argument discarded -- which would look to the user like the robot
    understood a request it never received."""
    with pytest.raises(Exception):
        validate_args("explore", {"area": "kitchen"})


def test_the_prompt_warns_that_explore_returns_IMMEDIATELY():
    """The likeliest way this batch produces a confidently wrong demo: a model reports
    the room explored one second into the drive. It is a prompt property because the
    model is the thing that would get it wrong."""
    # WHITESPACE-NORMALISED. The prompt is hand-wrapped, so a phrase that matters can
    # straddle a line break -- and a test that fails on the wrap rather than on the
    # content teaches its next reader to reword the prompt to satisfy the test.
    prompt = re.sub(r"\s+", " ", SYSTEM_PROMPT.lower())
    assert "explore()" in prompt
    assert "returns as soon as the mission starts" in prompt
    assert "status()" in prompt, (
        "the prompt tells the model explore() returns early without telling it how to "
        "find out when the mission actually ends")


def test_the_prompt_and_the_node_BOTH_say_stop_is_not_an_emergency_stop():
    """Said twice on purpose, in the two places a reader or a model can encounter it:
    the instruction the model is given, and the result string the user is shown."""
    prompt = re.sub(r"\s+", " ", SYSTEM_PROMPT.lower())
    assert "not an emergency" in prompt
    node = re.sub(r"\s+", " ", NODE.read_text().lower())
    assert "not an emergency" in node
    assert "coasts to a halt" in node, (
        "the node does not say what stopping actually does; 'not an emergency stop' "
        "alone tells a user what it ISN'T without telling them what happens")


def test_the_tool_count_in_the_prompt_matches_the_schema():
    """A prompt that says 'three tools' while six exist teaches the model to distrust
    its own instructions -- and this one said three until the day it said six."""
    n = len(TOOL_SCHEMAS)
    words = {3: "three", 4: "four", 5: "five", 6: "six", 7: "seven",
             8: "eight", 9: "nine", 10: "ten"}
    assert f"exactly {words[n]} tools" in re.sub(r"\s+", " ", SYSTEM_PROMPT), (
        f"{n} tools exist but the prompt does not say 'exactly {words[n]} tools'")


# --- the status summary, which is what a user actually hears ------------------

def test_the_summary_distinguishes_the_states_a_user_would_confuse():
    running = describe_mission({"running": True, "goal_in_flight": True})
    choosing = describe_mission({"running": True, "goal_in_flight": False})
    idle = describe_mission({})
    done = describe_mission({"done": True})

    assert "exploring" in running and "driving" in running
    # A rover BETWEEN goals is still exploring. Reporting it idle would make every
    # selection cycle look like a stall, which is the report a user panics at.
    assert "exploring" in choosing and "idle" not in choosing.lower()
    assert "idle" in idle.lower()
    assert "finished" in done.lower()
    assert len({running, choosing, idle, done}) == 4, "two states read identically"


def test_an_UNKNOWN_candidate_count_is_never_rendered_as_zero():
    """D24 in a new field. `0 places still wanted` is the most reassuring line this
    tool can produce -- it means the room is covered -- and it must never appear on
    the strength of a number nobody measured."""
    unknown = describe_mission({"running": True, "remaining_candidates": None})
    zero = describe_mission({"running": True, "remaining_candidates": 0})
    assert "unknown place(s)" in unknown
    assert "0 place(s)" in zero
    assert unknown != zero


# --- staleness: the property the whole tool exists to preserve ----------------

def test_status_treats_stale_and_absent_as_FAILURES_not_as_values():
    """Both branches must set success False and neither may present the cached payload
    as current. Asserted against the source because the behaviour lives in a ROS
    callback that cannot be constructed on this host."""
    body = _body(NODE.read_text())
    status = body[body.index("def _on_status"):body.index("def _on_observe")]

    assert status.count("response.success = False") >= 2, (
        "one of the two bad-status branches (never received / stale) reports success")
    assert "stale" in status.lower() and "age" in status.lower(), (
        "the stale branch does not say it is stale, or does not say how old")
    assert "last_known" in status, (
        "the stale branch drops the cached payload entirely. It should be RETURNED and "
        "LABELLED -- a reader who knows it is 40 s old can still use it; a reader given "
        "nothing has to guess")
    # ...and the fresh branch must be the only one that presents values as current.
    fresh = status[status.index("response.success = True"):]
    assert "describe_mission" in fresh


def test_a_malformed_status_message_is_not_a_status():
    """A JSON parse failure must leave the previous timestamp alone rather than
    refreshing it with garbage -- otherwise a node emitting nonsense at 1 Hz reads as
    perfectly healthy and staleness never fires."""
    body = _body(NODE.read_text())
    handler = body[body.index("def _on_mission_status"):body.index("def _forward_trigger")]

    assert "except" in handler, "the status handler does not guard the parse at all"
    # The parse must come FIRST and the store must come after it, so unparseable
    # traffic cannot refresh the timestamp.
    assert handler.index("json.loads") < handler.index("self._mission_status = ("), (
        "the timestamp is refreshed before the payload is parsed, so a node emitting "
        "nonsense at 1 Hz reads as perfectly healthy and staleness never fires")
    # And the failure path must LEAVE, not fall through to the store.
    after_except = handler[handler.index("except"):handler.index("self._mission_status = (")]
    assert "return" in after_except, (
        "a failed parse falls through to the store instead of returning")


# --- the forwarding contract --------------------------------------------------

def test_explore_and_stop_report_the_OTHER_node_s_verdict():
    """This node does not do the work, so it must not overrule the node that did. A
    refusal from the explorer -- 'mission already finished', say -- has to reach the
    user as a refusal."""
    body = _body(NODE.read_text())
    fwd = body[body.index("def _forward_trigger"):body.index("def _on_explore")]
    assert "if not result.success" in fwd
    assert "result.message" in fwd, (
        "the forwarded refusal's own message is discarded, so the user is told the "
        "call failed but not what the explorer said about it")


def test_the_client_reaches_the_mission_tools_and_holds_no_motion_primitive():
    body = _body(CLIENT.read_text())
    for tool in ("explore", "stop", "status"):
        assert tool in body, f"the client cannot call {tool}"
    # The v1 boundary, restated for the new surface: a model cannot ask for a velocity
    # if nothing in the path can express one.
    assert "Twist" not in body and "cmd_vel" not in body
