"""The clock convention, pinned against PHYSICALLY-KNOWN directions.

Regression test for the 2026-08-14 mirrored-label defect: an analysis script bucketed
by counter-clockwise bearing and called it "o'clock", mirroring left for right in
every label except 12 and 6. The written record described the open side of a wedge as
the wrong side.

These assertions are deliberately phrased as physical facts ("a point to the RIGHT is
3 o'clock"), not as restatements of the formula. A test that re-derives the formula
would have passed happily under the mirror.
"""

import math

import pytest

from sphero_rvr_core.bearings import (
    bearing_deg_to_clock,
    bearing_of_point,
    clock_to_bearing_deg,
)


# (description, point in body frame with +y LEFT, the clock position a human reads)
PHYSICAL_CASES = (
    ("straight ahead", (1.0, 0.0), 12),
    ("directly to the RIGHT", (0.0, -1.0), 3),
    ("directly BEHIND", (-1.0, 0.0), 6),
    ("directly to the LEFT", (0.0, 1.0), 9),
    # Mid-sector points, NOT the 45 deg diagonals: +/-45 lands exactly on a sector
    # boundary (1|2 and 10|11), where either label is defensible and the assertion
    # would be about rounding rather than about left vs right.
    ("ahead and to the right", (1.0, -math.sqrt(3.0)), 2),    # -60 deg
    ("ahead and to the left", (1.0, math.sqrt(3.0)), 10),     # +60 deg
    ("behind and to the right", (-1.0, -math.sqrt(3.0)), 4),  # -120 deg
    ("behind and to the left", (-1.0, math.sqrt(3.0)), 8),    # +120 deg
)


@pytest.mark.parametrize("what,point,expected", PHYSICAL_CASES)
def test_clock_labels_match_physical_directions(what, point, expected):
    x, y = point
    bearing = bearing_of_point(x, y)
    got = bearing_deg_to_clock(bearing)
    assert got == expected, (
        f"a point {what} (x={x}, y={y}, bearing {bearing:+.1f} deg) must read "
        f"{expected} o'clock, got {got} -- left and right are mirrored"
    )


def test_right_and_left_are_not_interchangeable():
    """The single assertion the mirrored version could not survive."""
    assert bearing_deg_to_clock(-90.0) == 3, "-90 deg is the RIGHT side: 3 o'clock"
    assert bearing_deg_to_clock(+90.0) == 9, "+90 deg is the LEFT side: 9 o'clock"
    assert bearing_deg_to_clock(-90.0) != bearing_deg_to_clock(+90.0)


def test_the_recorded_wedge_blocker_is_behind_RIGHT():
    """The 2026-08-14b wedge, as the corrected convention reads it.

    The 0.150 m blocker sits at CCW +201.2 deg. Under the mirrored labelling it was
    reported as 7 o'clock (behind-LEFT). It is behind-RIGHT.
    """
    assert bearing_deg_to_clock(201.2) == 5
    assert bearing_deg_to_clock(120.0) == 8      # the 2.18 m opening, behind-LEFT


@pytest.mark.parametrize("clock", range(1, 13))
def test_clock_and_bearing_round_trip(clock):
    assert bearing_deg_to_clock(clock_to_bearing_deg(clock)) == clock


def test_sector_boundaries_land_in_the_expected_position():
    """+/-15 deg either side of a centre stays in that clock position."""
    for clock in range(1, 13):
        centre = clock_to_bearing_deg(clock)
        assert bearing_deg_to_clock(centre + 14.0) == clock
        assert bearing_deg_to_clock(centre - 14.0) == clock


def test_wrapping_is_stable_across_the_seam():
    for offset in (-720.0, -360.0, 0.0, 360.0, 720.0):
        assert bearing_deg_to_clock(-90.0 + offset) == 3
        assert bearing_deg_to_clock(math.degrees(math.pi) + offset) == 6
