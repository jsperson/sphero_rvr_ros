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

def test_the_prompt_asks_both_verdicts_about_one_target():
    p = build_prompt("dr pepper bottle")
    assert "dr pepper bottle" in p
    assert '"match"' in p and '"identity"' in p
    assert '"where_in_frame"' in p and '"confidence"' in p
    assert "confirmed" in p and "unverified" in p and "mismatch" in p
    assert "left" in p and "center" in p and "right" in p


def test_the_prompt_demands_named_evidence_for_both_hard_verdicts():
    """PIN 1 (consensus 2026-08-20): match AND mismatch both require their
    evidence named in description — a bare contradiction claim is exactly the
    misread-label failure the range rule exists for."""
    p = build_prompt("dr pepper bottle")
    assert "supporting evidence" in p
    assert "contradicting evidence" in p


def test_an_empty_target_refuses_loudly():
    with pytest.raises(ValueError):
        build_prompt("   ")


# --- the strict parse: schema or a loud reason ------------------------------------------

def test_a_confirmed_match_parses():
    r = parse_recognition_reply(
        'Some preamble. {"match": true, "identity": "confirmed", '
        '"where_in_frame": "left", '
        '"confidence": 0.85, "description": "red bottle on the floor"}')
    assert r == {"match": True, "identity": "confirmed", "where_in_frame": "left",
                 "confidence": 0.85, "description": "red bottle on the floor"}


def test_the_honest_middle_parses():
    """The redesign's reason for existing: right kind of object, identity not
    resolvable — the answer the old single boolean could not carry."""
    r = parse_recognition_reply(
        '{"match": true, "identity": "unverified", "where_in_frame": "center", '
        '"confidence": 0.8, "description": "a bottle, label unreadable"}')
    assert r["match"] is True and r["identity"] == "unverified"


def test_a_clean_absence_parses_with_null_identity_and_place():
    r = parse_recognition_reply(
        '{"match": false, "identity": null, "where_in_frame": null, '
        '"confidence": 0.9, "description": "no bottle visible"}')
    assert r["match"] is False
    assert r["identity"] is None and r["where_in_frame"] is None


@pytest.mark.parametrize("bad,why", [
    ("not json at all", "no JSON"),
    ('{"identity": null, "where_in_frame": "left", "confidence": 0.9, "description": "x"}', "match missing"),
    ('{"match": "yes", "identity": "confirmed", "where_in_frame": "left", "confidence": 0.9, "description": "x"}', "match not bool"),
    ('{"match": true, "identity": null, "where_in_frame": "left", "confidence": 0.9, "description": "x"}', "match with no identity verdict"),
    ('{"match": true, "identity": "probably", "where_in_frame": "left", "confidence": 0.9, "description": "x"}', "invented identity value"),
    ('{"match": true, "identity": "confirmed", "where_in_frame": null, "confidence": 0.9, "description": "x"}', "sighting with no place"),
    ('{"match": true, "identity": "confirmed", "where_in_frame": "behind", "confidence": 0.9, "description": "x"}', "invalid sector"),
    ('{"match": false, "identity": "mismatch", "where_in_frame": null, "confidence": 0.9, "description": "x"}', "identity verdict with no object"),
    ('{"match": false, "identity": null, "where_in_frame": "left", "confidence": 0.9, "description": "x"}', "place with no sighting"),
    ('{"match": true, "identity": "confirmed", "where_in_frame": "left", "confidence": 1.4, "description": "x"}', "confidence out of range"),
    ('{"match": true, "identity": "confirmed", "where_in_frame": "left", "confidence": 0.9, "description": ""}', "empty description"),
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

def test_the_result_carries_full_provenance_and_both_verdicts():
    parsed = {"match": True, "identity": "unverified", "where_in_frame": "right",
              "confidence": 0.7, "description": "bottle by the table"}
    r = build_result(target="bottle", parsed=parsed, photo_path="/x/y.jpg",
                     map_x=1.234, map_y=-0.5678, capture_yaw_rad=0.0,
                     stamp="20260819_150000", model="syn:large:vision")
    assert r["photo_path"] == "/x/y.jpg"
    assert r["map_pose"] == {"x": 1.234, "y": -0.568, "yaw_deg": 0.0}
    assert r["bearing_deg"] == pytest.approx(-HORIZONTAL_FOV_DEG / 3.0, abs=0.1)
    assert r["stamp"] and r["model"] and r["target"] == "bottle"
    assert r["match"] is True and r["identity"] == "unverified"


def test_an_absence_result_has_no_bearing():
    parsed = {"match": False, "identity": None, "where_in_frame": None,
              "confidence": 0.9, "description": "nothing"}
    r = build_result(target="bottle", parsed=parsed, photo_path="p",
                     map_x=0, map_y=0, capture_yaw_rad=0, stamp="s", model="m")
    assert r["bearing_deg"] is None
    assert r["identity"] is None


def test_an_unverified_candidate_still_gets_a_bearing():
    """Two-stage search consumes match=true candidates by bearing; identity
    doubt must not cost the approach its direction."""
    parsed = {"match": True, "identity": "unverified", "where_in_frame": "left",
              "confidence": 0.6, "description": "a bottle, brand unclear"}
    r = build_result(target="dr pepper bottle", parsed=parsed, photo_path="p",
                     map_x=0, map_y=0, capture_yaw_rad=0, stamp="s", model="m")
    assert r["bearing_deg"] == pytest.approx(HORIZONTAL_FOV_DEG / 3.0, abs=0.1)


def test_the_result_builder_cannot_carry_a_key_by_construction():
    import inspect
    from sphero_rvr_core.recognition import build_result as br
    params = set(inspect.signature(br).parameters)
    assert not any("key" in p or "token" in p or "secret" in p for p in params), (
        "build_result grew a credential-shaped parameter — the by-construction "
        "guarantee is gone")


# --- range at the look (search round 2 §1): tof band geometry ---------------------------

from sphero_rvr_core.recognition import (  # noqa: E402
    CAMERA_MOUNT_OFFSET_DEG,
    range_from_tof_points,
)


def test_the_mount_offset_constant_is_the_measured_value():
    assert CAMERA_MOUNT_OFFSET_DEG == 14.0


def test_a_point_dead_ahead_of_the_camera_is_center():
    """The offset's sign, pinned: the camera points 14° LEFT, so a point at
    body azimuth +14° sits on the CAMERA axis — sector center."""
    p = (math.cos(math.radians(14.0)), math.sin(math.radians(14.0)))
    assert range_from_tof_points([p], "center") == (pytest.approx(1.0), False)


def test_a_point_dead_ahead_of_the_BODY_is_sector_right():
    """The live confirmation from the sitting: a body-centered object reads
    RIGHT (camera azimuth −14°, inside the right band)."""
    r, amb = range_from_tof_points([(1.2, 0.0)], "right")
    assert r == pytest.approx(1.2) and amb is False
    assert range_from_tof_points([(1.2, 0.0)], "center") is None


def test_a_single_standing_cluster_is_unambiguous():
    ahead = math.radians(14.0)
    pts = [(r * math.cos(ahead), r * math.sin(ahead)) for r in (0.9, 1.0, 1.1)]
    out_band = [(0.0, 1.0)]   # body +90° — no sector holds it
    assert range_from_tof_points(pts + out_band, "center") == \
        (pytest.approx(1.0), False)


def test_two_standing_clusters_return_the_nearest_flagged_ambiguous():
    ahead = math.radians(14.0)
    pts = [(r * math.cos(ahead), r * math.sin(ahead))
           for r in (1.0, 1.05, 2.0, 2.1)]
    r, amb = range_from_tof_points(pts, "center")
    assert r == pytest.approx(1.05) and amb is True


def test_floor_clutter_alone_is_none_never_a_guess():
    """Below the derived 0.8 m cutoff lives the tof's own floor — those
    returns must never masquerade as a sighting range."""
    ahead = math.radians(14.0)
    floor = [(r * math.cos(ahead), r * math.sin(ahead)) for r in (0.3, 0.4, 0.5)]
    assert range_from_tof_points(floor, "center") is None
    assert range_from_tof_points([], "center") is None
    with pytest.raises(ValueError):
        range_from_tof_points([(1.0, 0.0)], "behind")


#: THE MUST-FLIP FIXTURE (consensus pin 3, free from placement 1's own six
#: clouds): the 114 in-band ranges verbatim. Old median-of-everything said
#: 0.502 (the FAIL); the ruling-C aggregation must say nearest-standing ~1.14
#: with ambiguous=True (two other clusters share the sector).
PLACEMENT1_RANGES = [
    0.26, 0.27, 0.27, 0.27, 0.27, 0.27, 0.27, 0.27, 0.27, 0.28, 0.28, 0.28,
    0.28, 0.28, 0.28, 0.28, 0.28, 0.28, 0.28, 0.28, 0.29, 0.29, 0.29, 0.29,
    0.31, 0.31, 0.31, 0.32, 0.32, 0.32, 0.32, 0.32, 0.33, 0.33, 0.33, 0.33,
    0.34, 0.34, 0.37, 0.38, 0.38, 0.39, 0.39, 0.39, 0.39, 0.39, 0.39, 0.40,
    0.40, 0.40, 0.40, 0.41, 0.41, 0.41, 0.41, 0.50, 0.51, 0.52, 0.53, 0.54,
    1.09, 1.11, 1.11, 1.11, 1.12, 1.12, 1.13, 1.14, 1.15, 1.15, 1.16, 1.16,
    1.16, 1.17, 1.19, 1.19, 1.20, 1.58, 1.58, 1.58, 1.59, 1.59, 1.59, 1.59,
    1.60, 1.60, 1.60, 1.60, 1.61, 1.61, 1.61, 1.61, 1.61, 1.61, 1.61, 1.61,
    1.61, 1.62, 1.62, 1.62, 1.62, 1.62, 1.62, 1.62, 1.62, 1.63, 1.63, 1.63,
    1.63, 1.64, 1.64, 1.64, 1.64, 1.71,
]


def test_placement_1_is_the_must_flip():
    """The committed field fixture: the exact distribution that failed the
    ±0.15 bar (median 0.502 vs truth 1.754) must now yield the nearest
    STANDING cluster (~1.14, the mystery object) flagged ambiguous — the safe
    minimum, exactly what ruling C promises the searcher."""
    ahead = math.radians(14.0)
    pts = [(r * math.cos(ahead), r * math.sin(ahead)) for r in PLACEMENT1_RANGES]
    r, amb = range_from_tof_points(pts, "center")
    assert 1.09 <= r <= 1.20, f"nearest standing cluster expected, got {r}"
    assert amb is True
    # and the old failure is provably dead: nothing near the floor median
    assert abs(r - 0.502) > 0.4


def test_the_result_carries_range_only_on_a_match():
    parsed_hit = {"match": True, "identity": "unverified",
                  "where_in_frame": "right", "confidence": 0.7, "description": "d"}
    r = build_result(target="t", parsed=parsed_hit, photo_path="p", map_x=0,
                     map_y=0, capture_yaw_rad=0, stamp="s", model="m",
                     range_m=1.234, range_source="tof", range_ambiguous=True)
    assert r["range_m"] == 1.234 and r["range_source"] == "tof"
    assert r["range_ambiguous"] is True
    parsed_miss = {"match": False, "identity": None, "where_in_frame": None,
                   "confidence": 0.9, "description": "d"}
    r = build_result(target="t", parsed=parsed_miss, photo_path="p", map_x=0,
                     map_y=0, capture_yaw_rad=0, stamp="s", model="m",
                     range_m=1.234, range_source="tof", range_ambiguous=True)
    assert r["range_m"] is None and r["range_source"] is None
    assert r["range_ambiguous"] is None
    r = build_result(target="t", parsed=parsed_hit, photo_path="p", map_x=0,
                     map_y=0, capture_yaw_rad=0, stamp="s", model="m")
    assert r["range_m"] is None and r["range_source"] is None


def test_the_node_gathers_tof_in_the_snapshot_window_only():
    """Source pins: the tof subscription exists, collection is gated on the
    same flag as frames (freshness by construction), and the buffer resets at
    snapshot start."""
    assert '"tof_points_topic", "/tof/points"' in NODE_SRC
    on_tof = NODE_SRC[NODE_SRC.index("def _on_tof_points"):
                      NODE_SRC.index("def _sighting_range")]
    assert "if not self._collect" in on_tof
    snap = NODE_SRC[NODE_SRC.index("def _snapshot"):NODE_SRC.index("def _pose_at")]
    assert "self._tof_clouds = []" in snap


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


# --- the R6a fix: transport failure is a refusal, never node death ----------------------

def test_transport_exceptions_are_caught_at_the_vlm_call_site():
    """Source pin (consensus 2026-08-20, §7): requests.RequestException is
    caught AT THE CALL SITE — on the bench card a ConnectionError escaped,
    propagated out of executor.spin(), and killed the node (R6a's receipt).
    No blanket except is allowed to absorb this job."""
    look = NODE_SRC[NODE_SRC.index("def _look"):NODE_SRC.index("def _snapshot")]
    assert "except requests.RequestException" in look
    assert look.index("query_vlm") < look.index("except requests.RequestException")
    assert "except Exception" not in look and "except:" not in look


def test_a_transport_failure_is_a_refusal_and_the_node_survives(tmp_path, monkeypatch):
    """The pin as a live behavior, where rclpy exists (the Pi): a VLM call that
    raises a transport error yields REFUSED naming the failure class, and the
    node answers AGAIN afterwards — alive, no manual cleanup."""
    rclpy = pytest.importorskip("rclpy")
    import requests as _requests
    from sensor_msgs.msg import Image
    from std_srvs.srv import Trigger
    import sphero_rvr_driver.recognition_node as rn

    key = tmp_path / "key"
    key.write_text("test-key-not-real")
    rclpy.init()
    node = None
    try:
        node = rn.RecognitionNode()
        node.set_parameters([
            rclpy.parameter.Parameter("target", value="bottle"),
            rclpy.parameter.Parameter("api_key_file", value=str(key)),
            rclpy.parameter.Parameter("require_stationary", value=False),
        ])
        frame = np.full((8, 8, 3), 128, dtype=np.uint8)
        monkeypatch.setattr(node, "_snapshot", lambda: [(Image(), frame)])
        monkeypatch.setattr(node, "_pose_at", lambda stamp: (0.0, 0.0, 0.0))
        monkeypatch.setattr(node, "_encode", lambda img: b"jpeg-bytes")

        def _raise(*a, **k):
            raise _requests.ConnectionError("connection refused (test)")
        monkeypatch.setattr(rn, "query_vlm", _raise)

        resp = node._look(Trigger.Request(), Trigger.Response())
        assert resp.success is False
        assert "REFUSED" in resp.message and "ConnectionError" in resp.message
        # the message names the class, never the endpoint (URL stays log-side)
        assert "http" not in resp.message.lower()
        # ALIVE: the same node answers a second invocation
        resp2 = node._look(Trigger.Request(), Trigger.Response())
        assert resp2.success is False and "REFUSED" in resp2.message
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


# --- the warm-up fix: frame quality gate (bench card R2's convicted mechanism) --------

from sphero_rvr_core.recognition import (  # noqa: E402
    CAST_RATIO_MAX,
    LUMA_FLOOR,
    frame_quality_ok,
)

#: MEASURED on the 2026-08-20 bench card's own frames (per-channel means):
#: the dark class (092438) and the bright class (092454) — the fixtures below
#: are those exact JPEGs, and the synthetic twins here carry their measured
#: means so the boundary logic tests everywhere while the real-frame must-flip
#: runs where cv2 exists.
DARK_MEANS = (9.0, 9.5, 9.3)      # B, G, R -> luma 9.4
BRIGHT_MEANS = (91.2, 98.4, 98.5)  # -> luma 97.6


def _flat(bgr_means):
    frame = np.zeros((24, 24, 3))
    for i, v in enumerate(bgr_means):
        frame[:, :, i] = v
    return frame


def test_the_measured_dark_class_is_rejected_and_bright_accepted():
    ok, reason = frame_quality_ok(_flat(DARK_MEANS))
    assert not ok and "underexposed" in reason
    ok, reason = frame_quality_ok(_flat(BRIGHT_MEANS))
    assert ok


def test_the_floor_sits_between_the_measured_classes_with_margin():
    """Derivation pin: 40 is 4.3x above the measured dark luma (9.4) and 2.4x
    below the measured bright floor (97.2). If either class drifts toward the
    floor, this fails before the field does."""
    dark_luma = 0.114 * 9.0 + 0.587 * 9.5 + 0.299 * 9.3
    bright_luma = 0.114 * 91.2 + 0.587 * 98.4 + 0.299 * 98.5
    assert dark_luma * 3 < LUMA_FLOOR < bright_luma / 2


def test_a_gross_color_cast_is_rejected_even_when_bright():
    ok, reason = frame_quality_ok(_flat((150.0, 60.0, 60.0)))
    assert not ok and "cast" in reason
    assert CAST_RATIO_MAX == 1.5


def test_the_real_card_frames_flip_the_gate():
    """THE MUST-FLIP ON REAL FRAMES (the ruling's literal pin): the card's own
    dark frame is REJECTED and its bright frame ACCEPTED, decoded from the
    committed JPEGs. Runs where cv2 exists (the Pi; skipped elsewhere — the
    synthetic twins above cover the logic everywhere)."""
    cv2 = pytest.importorskip("cv2")
    fx = Path(__file__).resolve().parent / "fixtures"
    dark = cv2.imread(str(fx / "recognition_20260820_092438.jpg"))
    bright = cv2.imread(str(fx / "recognition_20260820_092454.jpg"))
    assert dark is not None and bright is not None
    ok, reason = frame_quality_ok(dark)
    assert not ok, "the card's dark frame passed the gate — R2's mechanism is back"
    ok, _ = frame_quality_ok(bright)
    assert ok, "the card's bright frame failed the gate — the fix over-rejects"


def test_the_node_gates_frames_inside_the_collect_loop():
    """Source pin: quality filtering happens AS FRAMES ARRIVE in _snapshot (so
    collection continues past warm-up frames), not after the fact."""
    snap = NODE_SRC[NODE_SRC.index("def _snapshot"):NODE_SRC.index("def _pose_at")]
    assert "frame_quality_ok" in snap
    # the CODE's finally (rindex — the docstring mentions the word too)
    assert snap.index("frame_quality_ok") < snap.rindex("finally:")
