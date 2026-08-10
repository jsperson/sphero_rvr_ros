"""Tests for the pure task-tool logic: envelope, semantic query, result shape.

The envelope-ceiling shapes are lifted from the culled prompt-drive suite
(`c5e87d2~1:tests/test_prompt_drive.py:127-166`), which pinned the rule that matters:
a deployment may narrow the trusted envelope, but nothing may widen it past the
runtime ceiling. Only the units changed -- that envelope bounded dead-reckoned
segments against a mission API that no longer exists; this one bounds a map-frame
goal handed to Nav2.
"""

import json
import math

import pytest

from sphero_rvr_core.task_tools import (
    MAX_GOAL_DISTANCE_CEILING_M,
    MAX_QUERY_RADIUS_CEILING_M,
    EnvelopeError,
    GoalEnvelope,
    query_semantic_objects,
    tool_result,
    validate_goal,
    validate_query,
)


# --- envelope ceilings (lifted shape) ---------------------------------------

def test_envelope_can_be_widened_but_not_beyond_the_runtime_ceiling():
    wide = GoalEnvelope(max_goal_distance_m=MAX_GOAL_DISTANCE_CEILING_M,
                        max_query_radius_m=MAX_QUERY_RADIUS_CEILING_M)
    assert wide.max_goal_distance_m == pytest.approx(MAX_GOAL_DISTANCE_CEILING_M)
    with pytest.raises(ValueError, match="max_goal_distance_m"):
        GoalEnvelope(max_goal_distance_m=MAX_GOAL_DISTANCE_CEILING_M + 0.01)
    with pytest.raises(ValueError, match="max_query_radius_m"):
        GoalEnvelope(max_query_radius_m=MAX_QUERY_RADIUS_CEILING_M + 0.01)


@pytest.mark.parametrize("bad", (0.0, -1.0, float("nan"), float("inf")))
def test_envelope_rejects_nonpositive_and_nonfinite(bad):
    with pytest.raises(ValueError):
        GoalEnvelope(max_goal_distance_m=bad)


def test_envelope_serializes_for_audit():
    assert GoalEnvelope(max_goal_distance_m=2.0).to_json_dict()["max_goal_distance_m"] == 2.0


# --- goal validation --------------------------------------------------------

def test_goal_inside_the_envelope_is_returned_as_floats():
    assert validate_goal(1, 2, (0.0, 0.0), GoalEnvelope()) == (1.0, 2.0)


def test_goal_beyond_the_envelope_is_refused_with_the_distance_in_the_reason():
    env = GoalEnvelope(max_goal_distance_m=1.0)
    with pytest.raises(EnvelopeError, match="beyond the"):
        validate_goal(5.0, 0.0, (0.0, 0.0), env)


def test_distance_is_measured_from_the_robot_not_the_origin():
    # A goal 5 m from the origin but 0.5 m from the robot is INSIDE a 1 m envelope:
    # the envelope bounds displacement, not absolute coordinates.
    env = GoalEnvelope(max_goal_distance_m=1.0)
    assert validate_goal(5.0, 0.0, (4.5, 0.0), env) == (5.0, 0.0)


@pytest.mark.parametrize("bad", (float("nan"), float("inf"), float("-inf")))
def test_nonfinite_goal_is_refused(bad):
    with pytest.raises(EnvelopeError, match="finite"):
        validate_goal(bad, 0.0, (0.0, 0.0), GoalEnvelope())


@pytest.mark.parametrize("bad", ("over there", None, object()))
def test_non_numeric_goal_is_refused(bad):
    with pytest.raises(EnvelopeError, match="numbers"):
        validate_goal(bad, 0.0, (0.0, 0.0), GoalEnvelope())


def test_unknown_robot_pose_refuses_rather_than_assuming_the_origin():
    # Fails CLOSED on purpose: assuming (0,0) would accept an unbounded goal at
    # exactly the moment localization is broken.
    with pytest.raises(EnvelopeError, match="robot pose unknown"):
        validate_goal(1.0, 1.0, None, GoalEnvelope())


# --- query validation -------------------------------------------------------

def test_query_without_proximity_is_unbounded_and_allowed():
    assert validate_query("shoe", None, None, GoalEnvelope()) == ("shoe", None, None)


def test_empty_label_means_everything():
    assert validate_query("   ", None, None, GoalEnvelope())[0] is None


def test_near_and_radius_must_come_together():
    with pytest.raises(EnvelopeError, match="together"):
        validate_query("shoe", (1.0, 2.0), None, GoalEnvelope())


def test_query_radius_beyond_the_envelope_is_refused():
    env = GoalEnvelope(max_query_radius_m=1.0)
    with pytest.raises(EnvelopeError, match="exceeds"):
        validate_query("shoe", (0.0, 0.0), 5.0, env)


# --- semantic query over canned data ---------------------------------------

CANNED = json.dumps({
    "objects": [
        {"label": "pink shoe", "x": 1.0, "y": 0.0, "confidence": 0.9, "count": 3,
         "first_seen": 0.0, "last_seen": 1.0},
        {"label": "backpack", "x": 4.0, "y": 0.0, "confidence": 0.8, "count": 2,
         "first_seen": 0.0, "last_seen": 1.0},
    ]
})


def test_query_finds_by_fuzzy_label():
    found = query_semantic_objects(CANNED, label="shoe")
    assert [o["label"] for o in found] == ["pink shoe"]


def test_query_filters_by_proximity():
    near_only = query_semantic_objects(CANNED, near=(0.0, 0.0), radius_m=2.0)
    assert [o["label"] for o in near_only] == ["pink shoe"]


def test_query_with_no_map_yet_answers_nothing_known_rather_than_raising():
    # A query before the first observation is a normal state, not an error.
    assert query_semantic_objects("") == []
    assert query_semantic_objects("{not json") == []


def test_query_envelope_still_applies_to_the_radius():
    with pytest.raises(EnvelopeError):
        query_semantic_objects(CANNED, near=(0.0, 0.0), radius_m=99.0,
                               envelope=GoalEnvelope(max_query_radius_m=1.0))


# --- result shape -----------------------------------------------------------

def test_tool_result_is_json_with_a_typed_ok_flag():
    payload = json.loads(tool_result(True, "goto", "arrived", x=1.0, y=2.0))
    assert payload["ok"] is True and payload["tool"] == "goto"
    assert payload["x"] == 1.0 and payload["message"] == "arrived"


def test_tool_result_failure_carries_the_reason():
    payload = json.loads(tool_result(False, "goto", "too far"))
    assert payload["ok"] is False and payload["message"] == "too far"
