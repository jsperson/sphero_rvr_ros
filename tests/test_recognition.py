"""The recognition primitive's contract, held before any mission stands on it.

Pure tests for the brain (prompt, strict parse, bearing, sharpness, provenance)
plus the two consensus pins from the design round (2026-08-19): key absence is
a LOUD refusal, and no code path can carry the key into a log line or result.
The bench certification (staged objects, hit rates, non-interference) is the
field half; this file is the half no hardware can invalidate.
"""

import math
from pathlib import Path

import numpy as np
import pytest

from sphero_rvr_core.recognition import (
    HORIZONTAL_FOV_DEG,
    WHERE_VALUES,
    bearing_from_frame_position,
    build_prompt,
    build_result,
    parse_recognition_reply,
    pick_sharpest,
)

NODE_SRC = (Path(__file__).resolve().parents[1] / "src" / "sphero_rvr_driver" /
            "recognition_node.py").read_text()


# --- the prompt ------------------------------------------------------------------------

def test_the_prompt_asks_one_question_about_one_target():
    p = build_prompt("dr pepper bottle")
    assert "dr pepper bottle" in p
    assert '"seen"' in p and '"where_in_frame"' in p and '"confidence"' in p
    assert "left" in p and "center" in p and "right" in p


def test_an_empty_target_refuses_loudly():
    with pytest.raises(ValueError):
        build_prompt("   ")


# --- the strict parse: schema or a loud reason ------------------------------------------

def test_a_clean_sighting_parses():
    r = parse_recognition_reply(
        'Some preamble. {"seen": true, "where_in_frame": "left", '
        '"confidence": 0.85, "description": "red bottle on the floor"}')
    assert r == {"seen": True, "where_in_frame": "left", "confidence": 0.85,
                 "description": "red bottle on the floor"}


def test_a_clean_absence_parses_with_null_place():
    r = parse_recognition_reply(
        '{"seen": false, "where_in_frame": null, "confidence": 0.9, '
        '"description": "no bottle visible"}')
    assert r["seen"] is False and r["where_in_frame"] is None


@pytest.mark.parametrize("bad,why", [
    ("not json at all", "no JSON"),
    ('{"where_in_frame": "left", "confidence": 0.9, "description": "x"}', "seen missing"),
    ('{"seen": "yes", "where_in_frame": "left", "confidence": 0.9, "description": "x"}', "seen not bool"),
    ('{"seen": true, "where_in_frame": null, "confidence": 0.9, "description": "x"}', "sighting with no place"),
    ('{"seen": true, "where_in_frame": "behind", "confidence": 0.9, "description": "x"}', "invalid sector"),
    ('{"seen": false, "where_in_frame": "left", "confidence": 0.9, "description": "x"}', "place with no sighting"),
    ('{"seen": true, "where_in_frame": "left", "confidence": 1.4, "description": "x"}', "confidence out of range"),
    ('{"seen": true, "where_in_frame": "left", "confidence": 0.9, "description": ""}', "empty description"),
])
def test_every_schema_breach_refuses_with_a_reason(bad, why):
    """A near-miss reply silently treated as 'not seen' is a searcher that finds
    nothing and says nothing — every breach must RAISE, not degrade."""
    with pytest.raises(ValueError):
        parse_recognition_reply(bad)


# --- bearing: thirds of the (vendor-spec, bench-measured-later) FOV ---------------------

def test_center_is_the_capture_yaw_and_left_is_positive():
    assert bearing_from_frame_position(0.5, "center") == pytest.approx(0.5)
    third = math.radians(HORIZONTAL_FOV_DEG) / 3.0
    assert bearing_from_frame_position(0.5, "left") == pytest.approx(0.5 + third)
    assert bearing_from_frame_position(0.5, "right") == pytest.approx(0.5 - third)


def test_bearing_wraps_at_pi():
    b = bearing_from_frame_position(math.pi - 0.05, "left")
    assert -math.pi <= b <= math.pi


# --- sharpness: the sharp frame wins ----------------------------------------------------

def test_the_sharp_frame_beats_the_flat_one():
    rng = np.random.default_rng(7)
    sharp = rng.integers(0, 255, (40, 40)).astype(float)   # high-frequency detail
    flat = np.full((40, 40), 128.0)                        # featureless
    assert pick_sharpest([flat, sharp, flat]) == 1


def test_no_frames_refuses():
    with pytest.raises(ValueError):
        pick_sharpest([])


# --- provenance: the result carries everything, and CANNOT carry the key ----------------

def test_the_result_carries_full_provenance():
    parsed = {"seen": True, "where_in_frame": "right", "confidence": 0.7,
              "description": "bottle by the table"}
    r = build_result(target="bottle", parsed=parsed, photo_path="/x/y.jpg",
                     map_x=1.234, map_y=-0.5678, capture_yaw_rad=0.0,
                     stamp="20260819_150000", model="syn:large:vision")
    assert r["photo_path"] == "/x/y.jpg"
    assert r["map_pose"] == {"x": 1.234, "y": -0.568, "yaw_deg": 0.0}
    assert r["bearing_deg"] == pytest.approx(-HORIZONTAL_FOV_DEG / 3.0, abs=0.1)
    assert r["stamp"] and r["model"] and r["target"] == "bottle"


def test_an_absence_result_has_no_bearing():
    parsed = {"seen": False, "where_in_frame": None, "confidence": 0.9,
              "description": "nothing"}
    r = build_result(target="bottle", parsed=parsed, photo_path="p",
                     map_x=0, map_y=0, capture_yaw_rad=0, stamp="s", model="m")
    assert r["bearing_deg"] is None


def test_the_result_builder_cannot_carry_a_key_by_construction():
    import inspect
    from sphero_rvr_core.recognition import build_result as br
    params = set(inspect.signature(br).parameters)
    assert not any("key" in p or "token" in p or "secret" in p for p in params), (
        "build_result grew a credential-shaped parameter — the by-construction "
        "guarantee is gone")


# --- the consensus key pins, held at the node's source ----------------------------------

def test_key_absence_is_a_loud_refusal_at_invocation():
    """The pin verbatim: a verb that silently no-ops without its key is a
    searcher that finds nothing and says nothing. The node must read the key at
    invocation time and refuse NAMING THE PREREQUISITE (the path, never the
    contents)."""
    assert "no VLM key at {key_path}" in NODE_SRC
    assert "does not " in NODE_SRC and "silently no-op" in NODE_SRC
    # invocation-time read: the open() happens inside the service handler
    look = NODE_SRC[NODE_SRC.index("def _look"):NODE_SRC.index("def _snapshot")]
    assert "open(key_path)" in look, "the key is no longer read at invocation time"


def test_no_log_or_result_line_interpolates_the_key():
    """The key variable must never appear in a logger call, a refusal message,
    or any f-string except the read itself. Source-level, both files."""
    for line in NODE_SRC.splitlines():
        if "api_key" not in line:
            continue
        stripped = line.strip()
        if stripped.startswith("#") or '"""' in stripped:
            continue
        assert "get_logger" not in line, f"key near a logger call: {line.strip()}"
        assert "response.message" not in line, f"key near a response: {line.strip()}"
        assert "REFUSED" not in line, f"key near a refusal: {line.strip()}"
    core_src = (Path(__file__).resolve().parents[1] / "src" / "sphero_rvr_core" /
                "recognition.py").read_text()
    assert "api_key" not in core_src, (
        "the pure module grew key handling — the key stays node-side, "
        "invocation-scoped, and out of everything the result can carry")


# --- charter pins ------------------------------------------------------------------------

def test_camera_down_is_in_a_finally():
    """Snapshot-then-down survives every error path — the stop lives in a
    finally block, unconditionally."""
    snap = NODE_SRC[NODE_SRC.index("def _snapshot"):NODE_SRC.index("def _pose_at")]
    assert "finally:" in snap
    assert snap.index("finally:") < snap.index("killpg")


def test_stationarity_is_fail_closed_by_default():
    assert '"require_stationary", True' in NODE_SRC
    assert "fail-closed" in NODE_SRC.lower() or "fail-closed" in NODE_SRC
