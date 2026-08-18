"""Turning a stall COUNT into a place on the map. Pure logic; no ROS.

This is the consumer half of D48. The producer (`driver.py`) counts motor-stall
transitions and diagnostics publishes the running total; this module decides what that
total means and where the resulting mark goes.

THE SEAM'S SHAPE IS THE HARD PART, and it is worth stating plainly because the obvious
reading is wrong. The driver does not publish a stall EVENT with a timestamp. It
publishes a monotonic COUNT, at 1 Hz, inside `/diagnostics`. So a delta tells us
"N contacts happened since the last message" and nothing whatever about WHEN inside
that second. That is a deliberate design (see `counters-not-levels-at-seams`: a 1 Hz
boolean misses sub-second events, a counter never does) -- but it means the pose we
attach is the pose at the DIAGNOSTIC, not the pose at the contact.

WHY THAT IS STILL SOUND, with the measurement rather than the hope: during a contact
the robot is stalled, which is what "stall" means. The 2026-08-18 retest measured four
stall spans of **exactly 2.0 s each** -- twice the diagnostic period -- so a diagnostic
message lands *during* the stall, while the robot is pinned, on every contact observed
so far. The pose error is therefore bounded by robot motion while stalled, which is the
creep measured in mission 1 at 2.9 mm/cycle, not by the 0.3 m/s a moving robot covers
in a second.

THE CASE THAT BREAKS IT, named rather than discovered later: a stall shorter than the
diagnostic period. Then the delta arrives after the robot has resumed and the mark
lands wherever the robot got to. Nothing here can fix that from a counter alone -- the
fix is a stamped event topic from the driver, and it is written down in
`docs/blind_contact_bt_node_TODO.md` rather than pretended away.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from sphero_rvr_core.decisive_control import freeze_mark_pose

#: THE MEASURED BODY -- Scott's tape 2026-08-15, referenced to base_link, asserted in
#: `tests/test_footprint_derivation.py`. NOT the padded numbers: `collision_stop.yaml`
#: carries `footprint_rear_m: 0.1145`, which is this 0.0995 plus 0.015 m of margin, and
#: the bespoke controller carried 0.11/0.16, padded ~2x for braking. Padding is right
#: for a brake and wrong for saying where the body IS.
#:
#: (0.1145 shipped here for one commit, labelled "measured". Caught 2026-08-18 by
#: reading the derivation test instead of trusting the config field's name.)
FOOTPRINT_FRONT_M = 0.0965
FOOTPRINT_REAR_M = 0.0995   # cable compressed when taped: a MINIMUM

#: LATERAL half-extents, and they are ASYMMETRIC because base_link is not the body's
#: centre. Recorded here because the v2 strip geometry needs them and the temptation is
#: to reach for a longitudinal number instead -- a half-LENGTH standing in for a
#: half-WIDTH is the same category error as measuring the right thing about the wrong
#: population. Use max(left, right) for a symmetric strip.
HALF_WIDTH_LEFT_M = 0.098
HALF_WIDTH_RIGHT_M = 0.106

#: Circumscribed radius of that footprint (`lean_nav2_stock.yaml: robot_radius`).
ROBOT_RADIUS_M = 0.145


class ContactPoseUnavailable(RuntimeError):
    """No trustworthy pose for a contact. The contact is real; its LOCATION is not.

    Raised so the caller reports the contact and plants nothing. A mark at an invented
    pose is worse than no mark: it is permanent (nothing clears this layer), it is
    lethal-cost, and it is wrong -- three properties that combine into a robot that
    refuses to plan through clear floor for the rest of the mission.
    """


@dataclass(frozen=True)
class ContactBatch:
    """What one diagnostics message says happened since the last one."""

    contacts: int
    #: True when more than one contact arrived in a single message and they therefore
    #: collapse to a single mark. Not an error -- an honesty flag, so the report can
    #: say "4 contacts, 3 marks" instead of quietly implying they are the same number.
    collapsed: bool


class StallEventTracker:
    """Counter deltas, with the two ways a monotonic counter stops being monotonic.

    1. **First observation.** The count starts wherever the driver already is. Treating
       the first message as `N` contacts would plant a pile of marks at the robot's
       start pose the instant this node comes up -- the D43 shape (buried at the start
       pose) reached through a different door.
    2. **The driver restarted.** The count goes BACKWARDS. That is not negative
       contacts; it is a new counter. Re-baseline and mark nothing, because the
       contacts it is now counting from zero already happened somewhere we cannot
       place.
    """

    def __init__(self) -> None:
        self._last: int | None = None

    def observe(self, count: int) -> ContactBatch | None:
        count = int(count)
        previous, self._last = self._last, count
        if previous is None or count < previous:
            return None
        delta = count - previous
        if delta == 0:
            return None
        return ContactBatch(contacts=delta, collapsed=delta > 1)


def default_margin_m(mark_radius_m: float = ROBOT_RADIUS_M) -> float:
    """The gap between the footprint edge and the centre of the mark disc.

    DERIVED, not chosen. A mark is a disc of radius ~`robot_radius` (we know the robot
    could not pass here; we do not know the obstacle's extent, so we mark the footprint
    we proved was blocked). Put that disc's centre at the footprint edge and it swallows
    the robot's own centre cell -- and a robot whose own cell is at inscribed cost
    cannot plan, cannot recover, and cannot be rescued by any behaviour
    (`exploration-start-clearance`, and D43 in the field).

    So the requirement is `front + margin > mark_radius`, and the strongest form of it
    -- `margin = mark_radius` -- puts the disc's NEAR edge exactly on the footprint's
    front edge. Nothing is ever planted inside the body.

    THE PRICE, stated: the disc's far edge then reaches `front + 2 x radius` ~ 0.39 m
    ahead, so a contact over-marks roughly 0.29 m of floor beyond the thing it hit.
    That is the conservative direction for an obstacle no sensor can see, and it is the
    cheap error -- the expensive one was measured twice.
    """
    return float(mark_radius_m)


def contact_mark_centre(
    x: float,
    y: float,
    yaw: float,
    *,
    reversing: bool,
    front_m: float = FOOTPRINT_FRONT_M,
    rear_m: float = FOOTPRINT_REAR_M,
    margin_m: float | None = None,
) -> tuple[float, float]:
    """Where the disc goes: the leading edge that was driving in, plus the margin.

    The leading-edge geometry is `decisive_control.freeze_mark_pose` -- D37's fix,
    ported rather than reinvented. Its rule, and the reason it is not symmetric: a
    contact while REVERSING means the obstacle is behind, and marking the front edge
    then plants a lethal disc on clear floor ahead while leaving the real obstacle
    unmarked.

    `reversing` must come from the COMMAND, never from anything inferred -- that was
    the D37 lesson and it is also the `assert-dont-infer-seams` lesson. The caller
    reads it from the commanded velocity it published, not from odometry.
    """
    if margin_m is None:
        margin_m = default_margin_m()
    return freeze_mark_pose(
        x, y, yaw, front_m + margin_m, rear_m + margin_m, reversing=reversing
    )


def disc_points(
    cx: float, cy: float, radius_m: float, ring_points: int
) -> list[tuple[float, float]]:
    """A filled-enough disc: centre, plus two rings at half and full radius.

    Two rings rather than one because the costmap marks CELLS: at 0.05 m resolution a
    single ring of radius 0.145 leaves the disc's interior cells untouched between the
    ring points, and an obstacle layer that marks an annulus has left a hole in the
    middle of the thing it is describing.
    """
    points = [(float(cx), float(cy))]
    for k in range(int(ring_points)):
        angle = 2.0 * math.pi * k / float(ring_points)
        for fraction in (0.5, 1.0):
            r = float(radius_m) * fraction
            points.append((cx + r * math.cos(angle), cy + r * math.sin(angle)))
    return points
