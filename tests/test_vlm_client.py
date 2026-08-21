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


# --- the reasoning-burn ladder (flights 2+3, replay E as the must-flip) --------

def _body(content):
    return {"choices": [{"message": {"content": content}}]}


def test_the_escalating_ladder_is_replay_E_as_a_test(monkeypatch):
    """THE MUST-FLIP, from the offline replay of flight 3's failing prompt:
    empty at the base cap, a correct goto at the escalated cap. The ladder must
    make exactly two calls — base, then base x3 — and return the second's
    content. A same-cap retry (the old behavior) would re-run the identical
    guillotine and this test would fail on the caps it saw."""
    import sphero_rvr_core.vlm_client as vc
    caps = []

    def fake_post(url, headers=None, json=None, timeout=None):
        caps.append(json["max_tokens"])
        if json["max_tokens"] <= 1500:
            return _FakeResponse(_body(""))          # reasoning ate the cap
        return _FakeResponse(_body('{"tool": "goto", "args": {"x": 0.1, "y": 0.3}}'))

    monkeypatch.setattr(vc.requests, "post", fake_post)
    reply = vc.query_text("http://x", "k", "m", "p", max_tokens=1500, json_mode=True)
    assert reply == '{"tool": "goto", "args": {"x": 0.1, "y": 0.3}}'
    assert caps == [1500, 4500], \
        f"expected one base attempt then one x3 escalation, saw caps {caps}"


def test_all_attempts_empty_still_raises_after_the_full_ladder(monkeypatch):
    import sphero_rvr_core.vlm_client as vc
    caps = []

    def fake_post(url, headers=None, json=None, timeout=None):
        caps.append(json["max_tokens"])
        return _FakeResponse(_body(""))

    monkeypatch.setattr(vc.requests, "post", fake_post)
    with pytest.raises(RuntimeError, match="no usable text"):
        vc.query_text("http://x", "k", "m", "p", max_tokens=1500)
    assert caps == [1500, 4500, 4500], \
        "later attempts stay at the escalated cap (transient-garbage retries live there)"


def test_transient_garbage_at_the_escalated_cap_still_gets_its_retry(monkeypatch):
    import sphero_rvr_core.vlm_client as vc
    replies = iter(["", "<|garbage|>", "final answer"])

    def fake_post(url, headers=None, json=None, timeout=None):
        return _FakeResponse(_body(next(replies)))

    monkeypatch.setattr(vc.requests, "post", fake_post)
    assert vc.query_text("http://x", "k", "m", "p", max_tokens=1500) == "final answer"


def test_sticky_escalation_pays_the_ladder_once_per_session(monkeypatch):
    """§4 (flight 4 paid the ladder FIVE times): with a shared sticky dict, the
    first hard decision escalates 1500->4500 and every later call STARTS at
    4500 — one base attempt wasted per session, not per decision."""
    import sphero_rvr_core.vlm_client as vc
    caps = []

    def fake_post(url, headers=None, json=None, timeout=None):
        caps.append(json["max_tokens"])
        if json["max_tokens"] <= 1500:
            return _FakeResponse(_body(""))
        return _FakeResponse(_body("decision"))

    monkeypatch.setattr(vc.requests, "post", fake_post)
    sticky = {}
    assert vc.query_text("http://x", "k", "m", "p1", max_tokens=1500,
                         sticky=sticky) == "decision"
    assert vc.query_text("http://x", "k", "m", "p2", max_tokens=1500,
                         sticky=sticky) == "decision"
    assert caps == [1500, 4500, 4500], \
        f"second call must START escalated, saw {caps}"
    assert sticky["cap"] == 4500


def test_the_timeout_scales_with_the_escalated_cap(monkeypatch):
    """Flight 5's ending: an escalated 4500-token generation (~112 s at the
    worst observed reasoning speed) raced a flat 60 s read timeout and lost —
    the guard saved the client, but the timeout and the ladder were tuned in
    different rounds and had never met. Escalated attempts now get
    timeout × escalation; base attempts stay snappy."""
    import sphero_rvr_core.vlm_client as vc
    seen = []

    def fake_post(url, headers=None, json=None, timeout=None):
        seen.append((json["max_tokens"], timeout))
        if json["max_tokens"] <= 1500:
            return _FakeResponse(_body(""))
        return _FakeResponse(_body("decision"))

    monkeypatch.setattr(vc.requests, "post", fake_post)
    assert vc.query_text("http://x", "k", "m", "p", max_tokens=1500,
                         timeout=60.0) == "decision"
    assert seen == [(1500, 60.0), (4500, 180.0)]
    # sticky start at an escalated cap gets the scaled ceiling from attempt 0
    seen.clear()
    vc.query_text("http://x", "k", "m", "p", max_tokens=1500, timeout=60.0,
                  sticky={"cap": 4500})
    assert seen == [(4500, 180.0)]


def test_without_sticky_each_call_pays_its_own_ladder(monkeypatch):
    """The default stays stateless — sticky is opt-in, caller-owned."""
    import sphero_rvr_core.vlm_client as vc
    caps = []

    def fake_post(url, headers=None, json=None, timeout=None):
        caps.append(json["max_tokens"])
        if json["max_tokens"] <= 1500:
            return _FakeResponse(_body(""))
        return _FakeResponse(_body("decision"))

    monkeypatch.setattr(vc.requests, "post", fake_post)
    vc.query_text("http://x", "k", "m", "p1", max_tokens=1500)
    vc.query_text("http://x", "k", "m", "p2", max_tokens=1500)
    assert caps == [1500, 4500, 1500, 4500]
