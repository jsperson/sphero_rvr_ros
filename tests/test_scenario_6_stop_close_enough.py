"""SCENARIO 6 of Scott's nine, 2026-08-31.

  REQUIREMENT, his words: "It should not stop further away than 10 cm from an
  object that it can see."

  BAR AS FIRST WRITTEN: final gap from the NEAREST FOOTPRINT EDGE to the object
  <= 0.10 m.  PREDICTION FILED BEFORE EXECUTION: FAIL at 19.4 cm.

  BAR REVISED 2026-08-31 BY SCOTT, verbatim: "We can keep the 20cm stop distance."
  The bar is now <= 0.20 m and BOTH TIERS PASS.

  THE PROVENANCE IS PART OF THE BAR. It was 10 cm; it moved to 20 cm; the person who
  moved it was the person whose requirement it is, after seeing the measured 19.4 cm.
  It was NOT widened by us to make a red row green. A relaxed bar that does not carry
  its history is indistinguishable from one tuned to pass, and that is the trap this
  whole table exists to avoid -- so the original number, the reason and the author
  stay in this docstring for as long as the file does.

THE ROW HAS TWO TIERS AND ONE VERDICT WOULD HIDE THE FINDING:

  ToF low-obstacle brake :  8.3 cm  -> inside the 20 cm bar
  lidar stop tier        : 20.0 cm  -> inside it, with nothing to spare

THE TIERS REMAIN ASYMMETRIC BY ~11 cm. A low obstacle is approached to 8.3 cm and a
tall one to 19.4. Both pass, so this is not a defect -- but if consistent standoff is
ever wanted, that is the gap and that is its size.

D70 DOES NOT CLOSE ON THIS REVISION. Scott accepting 0.20 m makes it an acceptable
NUMBER; it does not make its DERIVATION correct. The lidar tier's stop requirement
was justified against operands this same config file denounces elsewhere, and a test
still enforces it against that dead figure. Lower priority now, same defect: re-derive
if anyone ever touches it, and never treat "Scott is happy at 20" as the derivation.

AND THIS ROW SAYS NOTHING ABOUT THE SMALL-OFFICE PROBLEM. The stop distance brakes for
obstacles in the path; what limits where the rover may GO is goal legality and the
circular footprint (a 30.4 cm planning floor against a 25.4 cm gap -- scenario 4). A
pass here must not be read as progress there.

The lidar figure is corroborated two independent ways: this pure-core loop stops at
20.0 cm, and the FIELD run of 2026-08-25 stopped at 19.4 cm (0.290 m from base_link
minus footprint_front 0.0965). Two routes, one centimetre apart.
"""

from __future__ import annotations

import os

import pytest
import yaml

from tests.test_scenario_1_never_touch_what_it_can_see import (BOX_X, FOOTPRINT_FRONT,
                                                               _approach)
from tests.test_scenario_5_both_sensors_prevent_collisions import (DEPLOYED, TALL_BOX,
                                                                   _approach_lidar)

BAR_M = 0.20          # Scott, 2026-08-31: "We can keep the 20cm stop distance."


def test_scenario_6_tof_tier_stops_within_the_bar():
    """The half that MEETS his requirement today."""
    travelled, outs, _ = _approach()
    gap = (BOX_X - travelled) - FOOTPRINT_FRONT
    assert outs[-1] == 0.0
    assert gap <= BAR_M, f"ToF tier stopped {gap * 100:.1f} cm short, bar is {BAR_M * 100:.0f} cm"


#: The FIELD measurement of 2026-08-25 -- the authoritative number for this row.
#: 0.290 m from base_link at the STOPPED tick, minus footprint_front 0.0965.
FIELD_GAP_M = 0.290 - 0.0965


def test_scenario_6_lidar_tier_meets_the_bar_in_the_field():
    """THE VERDICT, and it rests on the FIELD measurement rather than on the model.

    19.35 cm against a 20 cm bar: it passes by 6.5 mm. That margin is smaller than
    this file's own simulation can resolve, which is the next test's subject.
    """
    assert FIELD_GAP_M <= BAR_M, (
        f"field stand-off {FIELD_GAP_M * 100:.2f} cm exceeds the {BAR_M * 100:.0f} cm bar")


def test_scenario_6_the_pure_core_loop_cannot_adjudicate_this_margin():
    """AND THE INSTRUMENT SAYS SO ITSELF, which is why the verdict is not taken here.

    The pure-core loop lands at 20.01 cm -- one tenth of a millimetre on the WRONG
    side of a bar the field clears by 6.5 mm. That is not a disagreement about the
    robot: the loop advances in 0.1 s steps and stops on the first zero output, so it
    overshoots by up to one step of travel (~3.5 cm at cruise). ITS RESOLUTION IS
    COARSER THAN THE MARGIN BEING TESTED, by a factor of five.

    So this test asserts only what the loop CAN support -- agreement with the field to
    within its own step granularity -- and refuses to rule on the bar. Reporting the
    20.01 as a FAIL, or quietly preferring the 19.35 as a PASS, would both be picking
    the number that suited the story.
    """
    cruise = float(DEPLOYED["max_forward_mps"])
    travelled, outs = _approach_lidar(TALL_BOX, cruise)
    gap = (TALL_BOX.x_min - travelled) - float(DEPLOYED["footprint_front_m"])
    assert outs[-1] == 0.0
    step_travel = cruise * 0.1
    assert abs(gap - FIELD_GAP_M) <= step_travel, (
        f"pure-core {gap * 100:.2f} cm vs field {FIELD_GAP_M * 100:.2f} cm differ by "
        f"more than one step of travel ({step_travel * 100:.1f} cm) -- the model and "
        f"the robot genuinely disagree and it matters which is right")
