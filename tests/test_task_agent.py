"""Tests for the pure agent contract: what the client does with model output.

These are the tests that matter most in this commit. Everything a language model
might do wrong — hallucinate a tool, invent an argument, emit prose, emit two
actions, loop forever — is a canned string here rather than something discovered
live with a robot attached. No network, no ROS.
"""

import json

import pytest

from sphero_rvr_core.task_agent import (
    TOOL_SCHEMAS,
    Budget,
    ContractError,
    build_user_turn,
    describe_call,
    parse_reply,
    validate_args,
)


# --- the happy contract ------------------------------------------------------

def test_tool_call_parses():
    d = parse_reply('{"tool": "goto", "args": {"x": 1.0, "y": 0.5}}')
    assert d.tool == "goto" and d.args == {"x": 1.0, "y": 0.5}
    assert not d.is_final


def test_say_finishes_the_instruction():
    d = parse_reply('{"say": "There is a shoe ahead."}')
    assert d.is_final and d.say == "There is a shoe ahead."


def test_json_wrapped_in_prose_and_fences_is_still_read():
    """Models narrate. The balanced-brace scan is what makes that survivable."""
    reply = 'Sure! Let me look.\n```json\n{"tool": "observe", "args": {}}\n```\nOK?'
    assert parse_reply(reply).tool == "observe"


def test_nested_json_is_not_mis_read_as_an_inner_object():
    """The regression that made semantic_map report 0 objects against a reply
    listing three: a non-greedy regex returns the INNER object on nested input."""
    d = parse_reply('{"tool": "query_semantic_map", "args": {"label": "shoe"}}')
    assert d.tool == "query_semantic_map" and d.args == {"label": "shoe"}


def test_observe_needs_no_args_and_missing_args_is_fine():
    assert parse_reply('{"tool": "observe"}').args == {}


# --- fail-closed: the whole point -------------------------------------------

def test_hallucinated_tool_is_refused_and_the_error_names_the_real_tools():
    with pytest.raises(ContractError) as exc:
        parse_reply('{"tool": "launch_missiles", "args": {}}')
    msg = str(exc.value)
    assert "launch_missiles" in msg
    # The message is fed back to the model, so it must teach the right answer.
    for real in TOOL_SCHEMAS:
        assert real in msg


def test_unknown_argument_is_refused_rather_than_silently_dropped():
    """Dropping it would let the model believe it constrained something it did not
    — e.g. think it capped a speed that was never read."""
    with pytest.raises(ContractError, match="speed"):
        parse_reply('{"tool": "goto", "args": {"x": 1, "y": 0, "speed": 99}}')


def test_missing_required_argument_is_refused():
    with pytest.raises(ContractError, match="'y'"):
        parse_reply('{"tool": "goto", "args": {"x": 1.0}}')


@pytest.mark.parametrize("bad", ('"over there"', "true", "null", "[1, 2]"))
def test_wrong_typed_coordinate_is_refused(bad):
    with pytest.raises(ContractError, match="must be a number"):
        parse_reply('{"tool": "goto", "args": {"x": %s, "y": 0.0}}' % bad)


def test_boolean_is_not_accepted_as_a_number():
    """bool is an int subclass in Python; True in a pose is a mistake, not a 1."""
    with pytest.raises(ContractError):
        validate_args("goto", {"x": True, "y": 0.0})


def test_prose_with_no_json_is_refused_with_an_instructive_message():
    with pytest.raises(ContractError, match="no JSON object"):
        parse_reply("I'll drive forward about half a metre now.")


def test_reply_with_neither_tool_nor_say_is_refused():
    with pytest.raises(ContractError, match="neither"):
        parse_reply('{"thinking": "hmm"}')


def test_reply_with_both_tool_and_say_is_refused():
    """One action per reply, or the client would have to guess the order."""
    with pytest.raises(ContractError, match="not both"):
        parse_reply('{"tool": "observe", "args": {}, "say": "looking"}')


def test_empty_say_is_refused():
    with pytest.raises(ContractError):
        parse_reply('{"say": "   "}')


def test_args_must_be_an_object():
    with pytest.raises(ContractError, match='"args" must be'):
        parse_reply('{"tool": "goto", "args": [1, 2]}')


def test_partial_proximity_query_is_refused_with_the_missing_names():
    with pytest.raises(ContractError, match="together"):
        parse_reply('{"tool": "query_semantic_map", "args": {"near_x": 1.0}}')


def test_full_proximity_query_is_accepted():
    d = parse_reply('{"tool": "query_semantic_map", "args": '
                    '{"near_x": 1.0, "near_y": 0.0, "radius_m": 2.0}}')
    assert d.args["radius_m"] == 2.0


# --- budget ------------------------------------------------------------------

def test_budget_counts_and_exhausts():
    b = Budget(max_tool_calls=2)
    b.spend(); b.spend()
    assert b.exhausted and b.remaining == 0
    with pytest.raises(ContractError):
        b.spend()


def test_budget_reports_remaining():
    b = Budget(max_tool_calls=8)
    b.spend()
    assert b.remaining == 7 and not b.exhausted


# --- prompt assembly ---------------------------------------------------------

def test_user_turn_carries_the_instruction_and_prior_results():
    text = build_user_turn("find the shoe", [("observe()", '{"ok": true}')])
    assert "find the shoe" in text and "observe()" in text and '{"ok": true}' in text


def test_describe_call_is_stable_and_sorted():
    assert describe_call("goto", {"y": 2, "x": 1}) == "goto(x=1, y=2)"
    assert describe_call("observe", {}) == "observe()"
