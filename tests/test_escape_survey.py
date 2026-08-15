"""The stuck-state survey, against THREE REAL RECORDED WEDGES.

The fixtures are real scans lifted from the flight bags at the exact stamps the
autopsies were written from, complete with their recorded angle/range metadata and
the DEPLOYED base_link<-laser rotation. That last part is load-bearing: a fixture
without the transform would let these tests pass under a mirrored bearing
convention, which is the 2026-08-14 defect that mislabelled an entire night's
analysis. Baking the transform in makes the fixture describe a GEOMETRY rather than
a convention.

Every expected value below is a published fact from the archived autopsy READMEs, not
a number produced by this code and then blessed. If the survey and the archive ever
disagree, one of them is wrong and that is the point of the test.
"""

import json
import os

import pytest

from sphero_rvr_core.escape_survey import (
    VOUCH_LIDAR,
    VOUCH_UNVOUCHED,
    Survey,
    SurveyConfig,
    format_survey,
    survey_from_scan,
)

FIXTURES = os.path.join(os.path.dirname(__file__), "fixtures")


def load(name):
    with open(os.path.join(FIXTURES, name + ".json")) as fh:
        return json.load(fh)


def survey_of(name, **kwargs):
    fx = load(name)
    return survey_from_scan(
        ranges=fx["ranges"],
        angle_min=fx["angle_min"],
        angle_increment=fx["angle_increment"],
        range_min=fx["range_min"],
        range_max=fx["range_max"],
        laser_yaw_deg=fx["laser_yaw_deg"],
        **kwargs,
    )


# name, min range, the clock it sits at, how many of 12 read open
# -- all three straight out of the archived autopsies.
SPECIMENS = (
    ("wedge_mission2_2026-08-14", 0.166, 9, 8),
    ("wedge_20260814b", 0.150, 5, 7),
    ("wedge_20260815_postspin", 0.232, 8, 9),
)


@pytest.mark.parametrize("name,min_m,min_clock,open_count", SPECIMENS)
def test_survey_reproduces_the_archived_wedge_measurements(name, min_m, min_clock, open_count):
    survey = survey_of(name)
    assert survey.min_range_m == pytest.approx(min_m, abs=0.002), (
        f"{name}: survey disagrees with the archived autopsy on the minimum range"
    )
    assert survey.min_clock == min_clock, (
        f"{name}: minimum should sit at {min_clock} o'clock — a mismatch here is a "
        f"bearing-convention error, which is exactly how an entire night's analysis "
        f"once came out mirrored"
    )
    assert len(survey.open_clocks) == open_count


def test_the_three_specimens_do_not_all_open_the_same_way():
    """The mirror-pair property, asserted so a side-biased rule cannot hide.

    Mission 2 was blocked on the LEFT with the right side open; 08-14b was blocked on
    the RIGHT with the left side open. A ranking rule that satisfies only one of them
    is side-biased, and this test exists so that failure is visible in the fixtures
    rather than in a room.
    """
    m2 = survey_of("wedge_mission2_2026-08-14")
    b = survey_of("wedge_20260814b")
    # 7..11 o'clock is the LEFT half; 1..5 is the RIGHT half.
    left = set(range(7, 12))
    right = set(range(1, 6))
    m2_open = set(m2.open_clocks)
    b_open = set(b.open_clocks)
    assert len(m2_open & right) > len(m2_open & left), "mission 2 should open to the RIGHT"
    assert len(b_open & left) > len(b_open & right), "08-14b should open to the LEFT"


def test_specimen_3_has_open_floor_dead_ahead():
    """Scott, from the floor: "It 100% can drive forward clear across the room."

    The survey must agree, because this is the pose where the escape reversed instead.
    """
    survey = survey_of("wedge_20260815_postspin")
    ahead = survey.directions[12]
    assert ahead.range_m is not None
    assert ahead.range_m > 1.5, (
        "12 o'clock must read as wide open here — the whole point of specimen 3 is "
        "that forward was available and never chosen"
    )
    assert 12 in survey.open_clocks


def test_footprint_overlap_is_counted_at_the_pose_where_it_explains_the_refusals():
    """08-14b had returns INSIDE the declared footprint, which is why the trajectory
    projection refused everything correctly. A survey that did not count them would
    report that pose as ordinary."""
    overlapped = survey_of("wedge_20260814b")
    assert overlapped.footprint_overlapped
    assert overlapped.footprint_overlap_count >= 20, (
        "the archived autopsy counted 28 returns inside the declared footprint"
    )


def test_a_direction_nothing_can_speak_for_is_UNVOUCHED_not_open():
    """Absence of evidence is a first-class fact.

    2026-08-15's pin lived in unvouched space -- every freeze was "an obstacle no
    sensor on this robot can see" while the ToF reported zero zones. A survey that
    presented such a direction as merely open would repeat the mistake that produced
    the freeze.
    """
    survey = survey_from_scan(
        ranges=[None] * 720,
        angle_min=-3.14159,
        angle_increment=0.008726,
        range_min=0.15,
        range_max=12.0,
        laser_yaw_deg=178.99,
    )
    assert len(survey.unvouched_clocks) == 12
    assert survey.open_clocks == ()
    assert survey.min_range_m is None
    for reading in survey.directions.values():
        assert reading.vouched_by == VOUCH_UNVOUCHED
        assert reading.range_m is None


def test_lidar_returns_are_vouched_by_the_lidar_and_say_so():
    survey = survey_of("wedge_20260815_postspin")
    seen = [d for d in survey.directions.values() if d.range_m is not None]
    assert seen, "the fixture has returns"
    assert all(d.vouched_by == VOUCH_LIDAR for d in seen), (
        "stage one vouches from the lidar only; ToF and trail vouching arrive with "
        "the plan stage and must not be silently claimed here"
    )


def test_the_survey_line_carries_what_an_autopsy_needs():
    """The survey is logged as ONE unit because it is both the plan's input and the
    register's evidence. Three wedge autopsies each cost hours of reconstruction that
    this line would have supplied."""
    survey = survey_of("wedge_20260815_postspin", cause="freeze", tof_zones=0,
                       tof_available=True, trail_available=True, trail_length_m=1.72)
    line = format_survey(survey)
    for token in ("SURVEY", "cause=freeze", "open=9/12", "min=", "oclock",
                  "footprint_overlap=", "tof=zones=0", "trail=1.72m"):
        assert token in line, f"survey line is missing {token!r}: {line}"
    assert "\n" not in line, "the survey must be ONE greppable line"


def test_an_absent_tof_is_reported_as_UNAVAILABLE_not_as_zero():
    """Zero zones and no sensor are different facts. Reporting a dead ToF as
    "zones=0" would read, forever after, as "the ToF looked and saw nothing"."""
    survey = survey_of("wedge_20260814b", tof_available=False, tof_zones=None)
    assert "tof=UNAVAILABLE" in format_survey(survey)


def test_survey_is_pure_and_needs_no_ros():
    """Asserted structurally: this module must stay importable on a machine with no
    rclpy, or the proofs cannot bind to it (commit 1's lesson, applied up front)."""
    import sphero_rvr_core.escape_survey as mod

    assert "rclpy" not in getattr(mod, "__dict__", {})
    assert isinstance(survey_of("wedge_20260814b"), Survey)


def test_open_threshold_comes_from_config_not_a_literal():
    tight = survey_of("wedge_20260815_postspin", config=SurveyConfig(open_m=1.0))
    loose = survey_of("wedge_20260815_postspin", config=SurveyConfig(open_m=0.20))
    assert len(tight.open_clocks) < len(loose.open_clocks)
