"""The freeze-mark set: TTL, merge, honest serialization.

These cases moved from test_freeze_sensor.py when the bespoke controller (and its
ProgressGuard freeze detector, that suite's other subject) was deleted in the
2026-08-21 project review — the mark set itself lives on in freeze_marks.py,
serving the touch port and the mission report.
"""

import pytest

from sphero_rvr_core.freeze_marks import FreezeMarkSet


# --- the mark set -----------------------------------------------------------

def test_a_mark_expires_after_its_ttl():
    """REVERT-PROOF SCENARIO 4. A moved chair must not haunt the map forever."""
    marks = FreezeMarkSet(ttl_s=300.0)
    marks.add(1.0, 2.0, now=0.0)
    assert len(marks.live(now=299.0)) == 1
    assert len(marks.live(now=301.0)) == 0


def test_refreezing_the_same_spot_refreshes_rather_than_stacks():
    """Three retries at one obstacle is one fact, not three."""
    marks = FreezeMarkSet(ttl_s=100.0, merge_radius_m=0.15)
    marks.add(1.0, 1.0, now=0.0)
    marks.add(1.05, 1.02, now=50.0)
    live = marks.live(now=60.0)
    assert len(live) == 1
    assert live[0].expires_at == pytest.approx(150.0), "the retry must extend the TTL"


def test_distinct_obstacles_are_kept_separately():
    marks = FreezeMarkSet(ttl_s=100.0, merge_radius_m=0.15)
    marks.add(0.0, 0.0, now=0.0)
    marks.add(2.0, 0.0, now=1.0)
    assert len(marks.live(now=2.0)) == 2


def test_report_list_is_plain_serializable_data():
    marks = FreezeMarkSet()
    marks.add(1.23456, -0.5, now=10.0)
    import json
    payload = marks.as_report_list(now=11.0)
    assert json.loads(json.dumps(payload)) == payload
    assert payload[0]["x"] == 1.235
