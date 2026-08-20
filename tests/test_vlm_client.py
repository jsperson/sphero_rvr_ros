"""Tests for the VLM reply parsing + exploration geometry (pure parts)."""

import math

import pytest

from sphero_rvr_core.vlm_client import direction_to_goal, extract_json


def test_extract_plain_json():
    assert extract_json('{"turn_deg": 30, "go": true}') == {"turn_deg": 30, "go": True}


def test_extract_json_embedded_in_prose():
    text = 'Sure! I think you should go here: {"turn_deg": -20, "go": true, "reason": "doorway left"}. Good luck.'
    assert extract_json(text)["turn_deg"] == -20


def test_extract_json_in_code_fence():
    text = "```json\n{\"turn_deg\": 0, \"go\": false}\n```"
    assert extract_json(text) == {"turn_deg": 0, "go": False}


def test_extract_json_none_raises():
    with pytest.raises(ValueError):
        extract_json("I cannot see any openings, sorry.")


def test_direction_to_goal_straight():
    gx, gy, gyaw = direction_to_goal(0.0, 0.0, 0.0, turn_deg=0, distance_m=1.5)
    assert gx == pytest.approx(1.5) and gy == pytest.approx(0.0)


def test_direction_to_goal_right_is_negative_y():
    gx, gy, _ = direction_to_goal(0.0, 0.0, 0.0, turn_deg=90, distance_m=1.0)
    assert gx == pytest.approx(0.0, abs=1e-9) and gy == pytest.approx(-1.0)


def test_direction_to_goal_left_is_positive_y():
    gx, gy, _ = direction_to_goal(0.0, 0.0, 0.0, turn_deg=-90, distance_m=1.0)
    assert gy == pytest.approx(1.0)


def test_direction_to_goal_respects_robot_heading():
    # Robot facing +y (yaw 90deg), go straight -> point at (0, d).
    gx, gy, _ = direction_to_goal(0.0, 0.0, math.pi / 2, turn_deg=0, distance_m=2.0)
    assert gx == pytest.approx(0.0, abs=1e-9) and gy == pytest.approx(2.0)


# --- nested-JSON regression (semantic map reported "0 objects" 2026-08-07) ---

def test_extract_nested_object_returns_the_OUTER_object():
    """The bug: a non-greedy regex returned an inner element, so .get('objects')
    silently yielded nothing against a reply that listed three."""
    text = ('{"objects": [{"label": "window", "u": 0.17, "distance_m": 1.6, "confidence": 0.9}, '
            '{"label": "pink bin", "u": 0.91, "distance_m": 0.8, "confidence": 0.75}]}')
    got = extract_json(text)
    assert "objects" in got
    assert [o["label"] for o in got["objects"]] == ["window", "pink bin"]


def test_extract_nested_object_wrapped_in_prose():
    text = 'Sure, here you go:\n{"objects": [{"label": "door", "u": 0.5}]}\nHope that helps!'
    assert extract_json(text)["objects"][0]["label"] == "door"


def test_extract_deeply_nested_object():
    text = '{"a": {"b": {"c": [1, 2, {"d": "e"}]}}, "f": 1}'
    assert extract_json(text)["f"] == 1


def test_extract_ignores_braces_inside_strings():
    text = '{"reason": "use {} braces } carefully", "go": true}'
    got = extract_json(text)
    assert got["go"] is True and got["reason"] == "use {} braces } carefully"


def test_extract_skips_a_malformed_leading_object():
    text = '{not json at all} then {"turn_deg": 5, "go": false}'
    got = extract_json(text)
    assert got["turn_deg"] == 5


def test_extract_still_handles_the_flat_explorer_schema():
    text = '```json\n{"turn_deg": -20, "go": true, "reason": "doorway left"}\n```'
    got = extract_json(text)
    assert got["turn_deg"] == -20 and got["go"] is True


# --- malformed 200 bodies refuse, never raise bare (the R6a family, widened) ---

class _FakeResponse:
    def __init__(self, body):
        self._body = body

    def raise_for_status(self):
        pass

    def json(self):
        if isinstance(self._body, Exception):
            raise self._body
        return self._body


@pytest.mark.parametrize("body", [
    {},                                        # no choices
    {"choices": []},                           # empty choices
    {"choices": [{}]},                         # no message
    {"choices": [{"message": {}}]},            # no content
    {"choices": [{"message": {"content": None}}]},  # content not a string
    {"choices": "surprise"},                   # wrong container type
    ValueError("body is not JSON"),            # .json() itself refuses
])
def test_a_malformed_200_body_is_a_RuntimeError_not_a_bare_KeyError(monkeypatch, body):
    """A 200 with an unexpected body shape is a misbehaving endpoint, not a
    transport failure — the same refuse-don't-die doctrine as the node's R6a
    fix (consensus 2026-08-20): the caller gets RuntimeError, which the node's
    call site already turns into a loud refusal."""
    import sphero_rvr_core.vlm_client as vc
    monkeypatch.setattr(vc.requests, "post", lambda *a, **k: _FakeResponse(body))
    with pytest.raises(RuntimeError, match="unexpected body shape"):
        vc.query_vlm("http://x", "k", "m", "prompt", b"jpeg")
    with pytest.raises(RuntimeError, match="unexpected body shape"):
        vc.query_text("http://x", "k", "m", "prompt")


def test_a_wellformed_body_still_returns_its_text(monkeypatch):
    import sphero_rvr_core.vlm_client as vc
    monkeypatch.setattr(
        vc.requests, "post",
        lambda *a, **k: _FakeResponse(
            {"choices": [{"message": {"content": '  {"match": true}  '}}]}))
    assert vc.query_vlm("http://x", "k", "m", "p", b"j") == '{"match": true}'
    assert vc.query_text("http://x", "k", "m", "p") == '{"match": true}'
