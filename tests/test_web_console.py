"""The web console's pure core, under test — including the marker-drift pin.

The pin test drives the REAL `task_agent.run_instruction` with scripted models and
a fake runner, then asserts every line the loop emits classifies to a non-note
event type. The classifier reads the loop's own markers, which is an inference at
a seam — this test is what turns marker drift into a red build instead of a chat
pane silently rendering everything as unstyled notes.
"""

import json
import math
import os
import struct
import zlib

import pytest

from sphero_rvr_core.recognition import build_result
from sphero_rvr_core.task_agent import Budget, run_instruction
from sphero_rvr_core.task_tools import tool_result
from sphero_rvr_core.web_console import (
    EventBroker, SSE_HEARTBEAT, build_state, classify_line, extract_look,
    format_sse, grid_to_png, project_scan, safe_photo_path,
)


# ---------------------------------------------------------------------------
# the marker-drift pin: the real loop's lines all classify
# ---------------------------------------------------------------------------

class ScriptedModel:
    """Replays canned replies; an Exception entry raises (the model-failure path)."""

    def __init__(self, replies):
        self._replies = list(replies)

    def __call__(self, system, prompt):
        reply = self._replies.pop(0)
        if isinstance(reply, Exception):
            raise reply
        return reply


class FakeRunner:
    def run(self, tool, args):
        return tool_result(True, tool, "fine")


def _drive(replies, max_tool_calls=8):
    lines = []
    run_instruction("test instruction", ScriptedModel(replies), FakeRunner(),
                    Budget(max_tool_calls), out=lines.append)
    return lines


def _types(lines):
    return [classify_line(line)["type"] for line in lines]


def test_pin_happy_path_markers_classify():
    lines = _drive([json.dumps({"tool": "observe", "args": {}}),
                    json.dumps({"say": "all done"})])
    assert _types(lines) == ["tool_call", "tool_result", "say"]
    call = classify_line(lines[0])
    assert (call["n"], call["max"], call["call"]) == (1, 8, "observe()")
    result = classify_line(lines[1])
    assert result["data"]["ok"] is True and result["data"]["tool"] == "observe"
    assert classify_line(lines[2])["text"] == "all done"


def test_pin_model_failure_markers_classify():
    lines = _drive([RuntimeError("the api fell over")])
    assert _types(lines) == ["model_failure", "model_failure"]
    assert "the api fell over" in classify_line(lines[0])["text"]


def test_pin_budget_wordless_ending_markers_classify():
    # Budget(1): one tool call spends it; the final-say demand is answered with
    # ANOTHER tool call, so the loop ends in its own words — the [budget] marker.
    tool = json.dumps({"tool": "observe", "args": {}})
    lines = _drive([tool, tool], max_tool_calls=1)
    assert _types(lines) == ["tool_call", "tool_result", "budget"]


def test_pin_reprompt_and_refusal_markers_classify():
    lines = _drive(["no json here at all", "still no json"])
    assert _types(lines) == ["reprompt", "refused", "refused"]


def test_unknown_line_is_a_note_not_dropped():
    event = classify_line("something the classifier has never met")
    assert event["type"] == "note"
    assert event["text"] == "something the classifier has never met"


def test_result_with_unparseable_body_keeps_text():
    event = classify_line("[result] REFUSED: camera warm-up gate")
    assert event["type"] == "tool_result"
    assert "data" not in event
    assert event["text"] == "REFUSED: camera warm-up gate"


# ---------------------------------------------------------------------------
# the look card, pinned to recognition.build_result's real contract
# ---------------------------------------------------------------------------

def _real_look_result(match=True):
    parsed = {"match": match,
              "identity": "unverified" if match else None,
              "where_in_frame": "center" if match else None,
              "confidence": 0.4,
              "description": "a clear bottle with a pink label" if match else ""}
    return build_result(
        target="dr pepper bottle", parsed=parsed,
        photo_path="/home/rvr/recognitions/recognition_20260820_1.jpg",
        map_x=1.0, map_y=2.0, capture_yaw_rad=0.5,
        stamp="2026-08-20T22:00:00", model="syn:large:vision",
        range_m=1.055, range_source="tof", range_ambiguous=False)


def test_look_extracted_from_real_build_result():
    look = extract_look(_real_look_result())
    assert look["photo"] == "recognition_20260820_1.jpg"
    assert "photo_path" not in look        # the browser never sees a filesystem path
    assert look["match"] is True
    assert look["identity"] == "unverified"
    assert look["range_m"] == 1.055 and look["range_source"] == "tof"
    assert look["range_ambiguous"] is False
    assert look["bearing_deg"] is not None
    assert look["bearing_relative_deg"] is not None
    assert look["model"] == "syn:large:vision"


def test_honest_no_match_still_gets_a_card():
    look = extract_look(_real_look_result(match=False))
    assert look is not None
    assert look["match"] is False
    assert look["range_m"] is None         # no range without a match, by contract


def test_look_rides_the_result_event():
    line = "[result] " + json.dumps(_real_look_result())
    event = classify_line(line)
    assert event["type"] == "tool_result"
    assert event["look"]["photo"] == "recognition_20260820_1.jpg"


def test_non_look_results_carry_no_card():
    assert "look" not in classify_line("[result] " + tool_result(True, "turn", "ok"))
    assert extract_look({"match": True}) is None          # no photo, no card
    assert extract_look({"photo_path": "x.jpg"}) is None  # no verdict, no card
    assert extract_look("not a dict") is None


# ---------------------------------------------------------------------------
# the state tick's age honesty
# ---------------------------------------------------------------------------

def test_state_reports_never_received():
    state = build_state(None, None, now=100.0, max_age_s=3.0,
                        chat={"state": "idle"}, map_meta=None)
    assert state["mission"]["available"] is False
    assert state["mission"]["stale"] is False
    assert state["pose"] is None


def test_state_reports_fresh_status():
    state = build_state({"x": 1.0, "y": 2.0, "yaw_deg": 90.0},
                        (99.0, {"running": True}), now=100.0, max_age_s=3.0,
                        chat={"state": "idle"}, map_meta={"stamp": 7})
    assert state["mission"]["available"] is True
    assert state["mission"]["data"] == {"running": True}
    assert state["mission"]["age_s"] == 1.0


def test_state_reports_staleness_with_age_never_smoothed():
    state = build_state(None, (90.0, {"running": True}), now=100.0, max_age_s=3.0,
                        chat={"state": "idle"}, map_meta=None)
    mission = state["mission"]
    assert mission["available"] is False and mission["stale"] is True
    assert mission["age_s"] == 10.0
    assert mission["last_known"] == {"running": True}
    assert "data" not in mission           # a stale value never wears a live key


# ---------------------------------------------------------------------------
# the broker: replay, drop-oldest, unregister
# ---------------------------------------------------------------------------

def test_broker_ids_and_replay():
    broker = EventBroker()
    for i in range(3):
        broker.publish({"type": "note", "text": f"n{i}"})
    cid, feed = broker.register(last_seen_id=1)
    got = [feed.get(timeout=0), feed.get(timeout=0)]
    assert [e["id"] for e in got] == [2, 3]
    assert feed.get(timeout=0) is None
    broker.publish({"type": "note", "text": "live"})
    assert feed.get(timeout=0)["id"] == 4
    broker.unregister(cid)
    assert broker.client_count == 0


def test_slow_client_drops_oldest_and_stalls_no_one():
    broker = EventBroker(feed_size=3)
    cid, feed = broker.register()
    for i in range(10):
        broker.publish({"type": "note", "text": f"n{i}"})   # never blocks
    got = []
    while True:
        event = feed.get(timeout=0)
        if event is None:
            break
        got.append(event["id"])
    assert got == [8, 9, 10]               # newest kept, drop was oldest-first
    broker.unregister(cid)


def test_unregistered_feed_gets_nothing_more():
    broker = EventBroker()
    cid, feed = broker.register()
    broker.unregister(cid)
    broker.publish({"type": "note", "text": "after"})
    assert feed.get(timeout=0) is None


# ---------------------------------------------------------------------------
# SSE framing
# ---------------------------------------------------------------------------

def test_sse_framing_carries_id_and_json():
    wire = format_sse({"id": 12, "type": "say", "text": "hello"}).decode()
    assert wire.startswith("id: 12\n")
    assert wire.endswith("\n\n")
    payload = json.loads(wire.split("data: ", 1)[1].strip())
    assert payload == {"id": 12, "type": "say", "text": "hello"}


def test_sse_heartbeat_is_a_comment():
    assert SSE_HEARTBEAT.startswith(b":")
    assert SSE_HEARTBEAT.endswith(b"\n\n")


# ---------------------------------------------------------------------------
# the map PNG
# ---------------------------------------------------------------------------

def _png_chunks(png):
    assert png[:8] == b"\x89PNG\r\n\x1a\n"
    chunks, at = {}, 8
    while at < len(png):
        length = struct.unpack(">I", png[at:at + 4])[0]
        tag = png[at + 4:at + 8]
        chunks[tag] = png[at + 8:at + 8 + length]
        at += 12 + length
    return chunks


def test_grid_png_shape_palette_and_yflip():
    # Row 0 is the map's BOTTOM row: [0 free, 100 occupied]; top row: [-1, 50].
    png = grid_to_png(2, 2, [0, 100, -1, 50])
    chunks = _png_chunks(png)
    width, height, depth, colour = struct.unpack(">IIBB", chunks[b"IHDR"][:10])
    assert (width, height, depth, colour) == (2, 2, 8, 3)
    assert len(chunks[b"PLTE"]) == 768
    # unknown (-1 -> index 255) wears the unknown tone, and it differs from free
    assert chunks[b"PLTE"][255 * 3:] != chunks[b"PLTE"][0:3]
    scanlines = zlib.decompress(chunks[b"IDAT"])
    # top-first scanlines, filter byte 0: the map's TOP row [-1, 50] comes first
    assert scanlines == bytes([0, 255, 50, 0, 0, 100])


def test_grid_png_refuses_shape_mismatch():
    with pytest.raises(ValueError):
        grid_to_png(2, 2, [0, 0, 0])


# ---------------------------------------------------------------------------
# photo confinement
# ---------------------------------------------------------------------------

def test_photo_confinement(tmp_path):
    photo = tmp_path / "recognition_1.jpg"
    photo.write_bytes(b"jpegish")
    outside = tmp_path.parent / "outside.jpg"
    outside.write_bytes(b"nope")

    assert safe_photo_path(str(tmp_path), "recognition_1.jpg") == str(photo)
    for bad in ("../outside.jpg", "/etc/passwd", "a/b.jpg", "..\\x.jpg",
                ".hidden.jpg", "recognition_1.png", "", None, "missing.jpg"):
        assert safe_photo_path(str(tmp_path), bad) is None


def test_photo_confinement_symlink_escape(tmp_path):
    root = tmp_path / "photos"
    root.mkdir()
    secret = tmp_path / "secret.jpg"
    secret.write_bytes(b"secret")
    (root / "link.jpg").symlink_to(secret)
    assert safe_photo_path(str(root), "link.jpg") is None


# ---------------------------------------------------------------------------
# the live lidar overlay (Scott's ask 2026-08-21) — geometry, honesty, budget
# ---------------------------------------------------------------------------

def test_projection_uses_the_laser_pose_not_the_robot_origin():
    """The mount matters: this rover's laser sits rotated ~179 deg from the body,
    so a projection that assumed the robot's own yaw would draw the room BEHIND
    the rover. The laser pose is passed in whole, from the same TF the costmaps
    read."""
    pts = project_scan([2.0], 0.0, 0.1, 0.1, 10.0, (1.0, 0.0, math.pi))
    assert pts[0][0] == pytest.approx(-1.0, abs=1e-3)   # 2 m along +x rotated pi
    assert pts[0][1] == pytest.approx(0.0, abs=1e-3)


def test_invalid_returns_are_DROPPED_not_drawn():
    """A dot on the floor plan is a claim that something is there. inf/nan and
    out-of-band ranges are absences, not obstacles at a guessed distance."""
    pts = project_scan([float("inf"), float("nan"), 0.05, 99.0, 1.0],
                       0.0, 0.1, 0.1, 10.0, (0.0, 0.0, 0.0))
    # only index 4 survives, and it keeps ITS OWN bearing (4 * 0.1 rad) — a
    # projection that collapsed survivors onto the first angle would draw the
    # room rotated
    assert len(pts) == 1
    assert pts[0][0] == pytest.approx(math.cos(0.4), abs=1e-3)
    assert pts[0][1] == pytest.approx(math.sin(0.4), abs=1e-3)


def test_decimation_spans_the_whole_field_of_view():
    """Keeping every Nth beam, not the first N: a decimated scan must still see
    behind the robot, or the overlay would silently crop the room."""
    ranges = [1.0] * 720
    pts = project_scan(ranges, -math.pi, 2 * math.pi / 720, 0.1, 10.0,
                       (0.0, 0.0, 0.0), max_points=60)
    assert len(pts) <= 60
    xs = [p[0] for p in pts]
    assert min(xs) < -0.9 and max(xs) > 0.9, "decimation cropped the field of view"


def test_empty_and_zero_length_scans_are_empty_not_crashes():
    assert project_scan([], 0.0, 0.1, 0.1, 10.0, (0.0, 0.0, 0.0)) == []


def test_state_carries_the_scan_and_None_is_honest():
    """None means the robot is NOT SEEING (no scan, stale, or no TF) — the
    viewer must be able to tell that from an empty room."""
    with_scan = build_state(None, None, 10.0, 3.0, {"state": "idle"}, None,
                            scan=[[1.0, 0.0]])
    assert with_scan["scan"] == [[1.0, 0.0]]
    without = build_state(None, None, 10.0, 3.0, {"state": "idle"}, None)
    assert without["scan"] is None


def test_the_tick_stays_small_enough_to_send_at_1hz():
    """Budget guard: the console's costless property is certified, and a scan
    overlay is the first thing that could quietly spend it. A full 720-beam scan
    must serialize to a few KB, not tens."""
    pts = project_scan([2.0] * 720, -math.pi, 2 * math.pi / 720, 0.1, 10.0,
                       (0.0, 0.0, 0.0))
    payload = json.dumps(build_state({"x": 0.0, "y": 0.0, "yaw_deg": 0.0}, None,
                                     10.0, 3.0, {"state": "idle"},
                                     {"stamp": "1", "width": 10, "height": 10,
                                      "resolution_m": 0.05,
                                      "origin": {"x": 0.0, "y": 0.0},
                                      "known_pct": 50.0},
                                     scan=pts))
    assert len(payload) < 8000, f"state tick grew to {len(payload)} bytes"
