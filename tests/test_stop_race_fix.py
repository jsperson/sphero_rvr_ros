"""The stop-race fix's shape and its pure pieces (2026-08-18 night, D34 family).

Field receipt: a STOP command waited 1.218 s in the /cmd_vel subscription queue --
behind the shared mutually-exclusive slot congested by scans, ticks and blocking
TF lookups -- while the tick path replayed the last pivot. The abort did not stop
the actuator, in the safety node. Rig baseline (pre-fix, N=20, pre-registered):
zero->wire p95 0.625 s, 14/20 trials with tail replays, ~23 stale-flaps per 4 s
pivot. These tests hold the fix's shape; the distributional claims live in
scripts/stop_race_test.py runs recorded in the commit.
"""

import ast
import re
from pathlib import Path

import pytest

from sphero_rvr_driver.collision_stop import StaticTransformCache, Transform2D

NODE_SRC = (Path(__file__).resolve().parents[1] / "src" / "sphero_rvr_driver"
            / "collision_stop_node.py").read_text()


# --- the pure cache -------------------------------------------------------------------

def test_first_success_is_captured_and_announced_exactly_once():
    cache = StaticTransformCache()
    calls = []

    def lookup():
        calls.append(1)
        return Transform2D(0.0045, -0.011, 3.124), None

    t1, err1, first1 = cache.get("laser", lookup)
    t2, err2, first2 = cache.get("laser", lookup)
    assert first1 is True and first2 is False
    assert err1 is None and err2 is None
    assert t1 is t2
    assert len(calls) == 1, "the whole point is ONE lookup per frame, forever"


def test_a_failed_lookup_is_never_cached_and_the_retry_stays_live():
    """TF may not be up yet at the node's first scans; failure must not poison the
    cache -- the first SUCCESS is what gets captured."""
    cache = StaticTransformCache()
    answers = [(None, "missing_tf"), (None, "missing_tf"),
               (Transform2D(1.0, 2.0, 0.5), None)]

    def lookup():
        return answers.pop(0)

    t, err, first = cache.get("laser", lookup)
    assert t is None and err == "missing_tf" and not first
    t, err, first = cache.get("laser", lookup)
    assert t is None and not first
    t, err, first = cache.get("laser", lookup)
    assert t is not None and first
    t2, _, first2 = cache.get("laser", lookup)   # answers exhausted: must not call
    assert t2 is t and not first2


def test_frames_are_cached_independently():
    cache = StaticTransformCache()
    cache.get("laser", lambda: (Transform2D(1, 0, 0), None))
    t, _, first = cache.get("other", lambda: (Transform2D(2, 0, 0), None))
    assert first and t.x == 2


# --- the node's shape, source-level (the class lives inside main(); rclpy-free file
# --- import is the repo's own pattern for these guards) --------------------------------

def test_cmd_vel_has_its_own_callback_group():
    """The fix itself: /cmd_vel must not share the default mutually-exclusive slot
    with /scan and the tick timer. The D22 comment in this very file names that
    disease; this guard keeps the cure from silently regressing in an edit."""
    m = re.search(
        r"create_subscription\(\s*Twist,\s*requested_cmd_topic[^)]*callback_group\s*=\s*self\._cmd_group",
        NODE_SRC, re.S)
    assert m, ("/cmd_vel subscription lost its dedicated callback group -- the "
               "stop race (zero waiting 1.2 s behind a congested slot) returns")
    assert "self._cmd_group = ReentrantCallbackGroup()" in NODE_SRC


def test_the_scan_transform_is_cached_not_relooked_up():
    """base<-laser is a bolted static mount; a blocking 50 ms lookup per scan
    inside the state lock was standing slot pressure. The cached path must be the
    one _on_scan actually reaches."""
    assert "_scan_tf_cache" in NODE_SRC
    assert "StaticTransformCache()" in NODE_SRC
    body = NODE_SRC[NODE_SRC.index("def _lookup_scan_transform("):]
    body = body[:body.index("def _lookup_scan_transform_uncached")]
    assert "_scan_tf_cache.get" in body, (
        "_lookup_scan_transform no longer consults the cache")


def test_slot_health_is_counted_not_narrated():
    """counters-not-levels: the congestion class must be readable from any future
    bag. Both counters must appear in the state line."""
    for token in ("slot_tick_overruns=", "slot_scan_gaps="):
        assert token in NODE_SRC, f"state line lost {token}"
    assert "self._tick_overruns += 1" in NODE_SRC
    assert "self._scan_gaps += 1" in NODE_SRC
