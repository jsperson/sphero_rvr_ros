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
`docs/blind_contact_bt_node_TODO.md` rather than pretended away (the
blind_contact MODULE retired with bucket zero 2026-08-21 — the unbuilt BT
node's helper; the stamped-event fix idea and this named case remain).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from sphero_rvr_core.freeze_marks import freeze_mark_pose
from sphero_rvr_core.pivot_curve import PIVOT_LINEAR_EPSILON_MPS

#: D57's stall classes. The 2026-08-19 flight proved the touch sense has a
#: false-positive class: a ROTATION stall against floor grip produces the same
#: counter delta as a real contact with nothing there -- two false marks planted
#: in the door gap (Scott's eyewitness ground truth: chassis touching nothing).
#: Translation stalls remain strong contact evidence (the d45bd24 boot class);
#: rotation stalls are weak evidence and must corroborate before painting.
STALL_TRANSLATION = "translation"
STALL_ROTATION = "rotation"
STALL_IDLE = "idle"

#: plan_pivot's own zero test (its `zero_epsilon` default): a commanded rotation
#: at or below this produces duty 0 -- treads never move, so no stall can be a
#: rotation stall below it. CHANGE BOTH OR NEITHER; the drift test reads both.
PIVOT_ZERO_EPSILON_RAD_S = 1e-9


def classify_stall(vx: float, wz: float) -> str:
    """Which kind of commanded motion was stalling when the counter ticked.

    Both boundaries are the DRIVER'S OWN, imported not copied, so the marker and
    the drivetrain agree about what a command was (derived-with-receipt, no new
    tunables):

    - `PIVOT_LINEAR_EPSILON_MPS` (pivot_curve, 0.005): the one shared definition
      of "in place" -- the driver's control loop routes a command to the pivot
      path below it and the translation path at or above it.
    - `PIVOT_ZERO_EPSILON_RAD_S`: plan_pivot's zero test -- below it no duty is
      emitted at all, and any nonzero request above it is raised to at least
      `pivot_min_duty` (treads move, a stall is possible).

    TRANSLATION: the rover was commanded through space -- a stall is strong
    contact evidence; mark as always. ROTATION: commanded rotation only -- the
    flight's floor-grip signature; paint requires corroboration. IDLE: a stall
    with no commanded motion is a phantom; never mark.
    """
    if abs(float(vx)) >= PIVOT_LINEAR_EPSILON_MPS:
        return STALL_TRANSLATION
    if abs(float(wz)) > PIVOT_ZERO_EPSILON_RAD_S:
        return STALL_ROTATION
    return STALL_IDLE

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

#: Circumscribed radius of that footprint AS CONFIGURED (`lean_nav2_stock.yaml:
#: robot_radius`). This is what we ASK for. It is not what the costmap uses -- see below,
#: and do not derive mark geometry from it.
ROBOT_RADIUS_M = 0.145

#: WHAT THE COSTMAP ACTUALLY USES, measured 2026-08-18 on the deployed binary (M1/M2).
#:
#: nav2 pads the footprint by `footprint_padding` (default 0.01, now DECLARED in the YAML
#: so it stops being invisible) and then derives BOTH the inscribed radius the inflation
#: layer uses AND the polygon that footprint clearing erases from, out of the PADDED
#: shape. So `robot_radius: 0.145` never reaches the geometry.
#:
#: M1, off `/local_costmap/published_footprint`: 16 vertices, vertex radii 0.1550-0.1591,
#: apothem 0.1519. M2, independently, off a radial cost profile: a cell at diagonal
#: distance hypot(0.15,0.05)=0.1581 from a lethal cell reads 245, and
#: `computeCost` (read from the deployed `inflation_layer.hpp`) gives
#: 252*exp(-4*(0.1581-0.1519)) = 245.8. Two methods, three decimals, agreeing.
#:
#: THE LESSON, because it cost this project both of its mark-geometry derivations: a
#: config field's value is not the deployed value once a framework default transforms it.
#: `robot_radius` is a request; the published footprint is the answer.
COSTMAP_INSCRIBED_RADIUS_M = 0.1519
COSTMAP_CIRCUMSCRIBED_RADIUS_M = 0.1591


class ContactPoseUnavailable(RuntimeError):
    """No trustworthy pose for a contact. The contact is real; its LOCATION is not.

    Raised so the caller reports the contact and plants nothing. A mark at an invented
    pose is worse than no mark: it is permanent (nothing clears this layer), it is
    lethal-cost, and it is wrong -- three properties that combine into a robot that
    refuses to plan through clear floor for the rest of the mission.
    """


class PoseDataLagsStamp(RuntimeError):
    """The exact-stamp lookup failed ONLY because the transform feed has not caught up.

    The caller's adapter raises this for tf2's ExtrapolationException and for nothing
    else. Lookup failures that mean the transform is genuinely absent (frame unknown,
    tree disconnected) must NOT be mapped here -- they keep refusing, because a
    fallback over a dead transform is an invented pose with extra steps.
    """


#: How stale a latest-available pose may be and still place a mark, in seconds.
#:
#: DERIVED, not chosen, from the 2026-08-18 run-3d bag (the three field contacts this
#: policy exists for; full anatomy in the 3d vault README): SLAM's map->odom ran
#: 69/73/87 ms behind the contact stamps, and across the whole flight its inter-stamp
#: gap was p99 = 103 ms with a worst single gap of 396 ms. The bound must cover the
#: worst gap with margin, or the fallback re-fails exactly when SLAM hiccups --
#: 0.396 x ~1.25 rounds to 0.5 s.
#:
#: WHY HALF A SECOND OF POSE AGE IS SAFE FOR A *CONTACT*: the detector's own definition
#: (stall + packets written + no motion) means the robot is stationary while the mark
#: is being placed, so a slightly-old pose differs by stall creep (2.9 mm/cycle,
#: mission 1), not by driving speed. Even the pathological moving case bounds at
#: ~0.2 m/s x 0.5 s = 0.10 m -- inside the forgiveness of the SHIPPED disc geometry
#: (radius `ROBOT_RADIUS_M` 0.145 placed `default_margin_m` beyond the footprint
#: edge). IF THE MARK GEOMETRY CHANGES -- the strip debate is dormant, not dead --
#: this bound re-derives against the new shape; the guard test names that coupling.
STALENESS_BOUND_S = 0.5

#: The measurements the bound derives from, kept next to it so a future SLAM-cadence
#: change breaks a test instead of a mission (`tests/test_contact_placement_tf_lag.py`
#: asserts the bound clears both with margin).
MEASURED_MAP_TF_GAP_P99_S = 0.103
MEASURED_MAP_TF_GAP_MAX_S = 0.396


@dataclass(frozen=True)
class ResolvedContactPose:
    """A pose the policy is willing to put a permanent lethal disc at.

    `path` says how it was obtained -- "exact" (transform at the contact's own stamp)
    or "fallback" (latest available, within `STALENESS_BOUND_S`) -- and `staleness_s`
    is the signed stamp gap (contact stamp minus transform stamp; 0.0 on the exact
    path). Both are carried into the mark log AND the mission report, so an autopsy
    can always ask "was this mark exactly-placed or bounded-fallback" without
    reconstructing it from timestamps.
    """

    x: float
    y: float
    yaw: float
    staleness_s: float
    path: str


def resolve_contact_pose(exact, latest, stamp_s, bound_s=STALENESS_BOUND_S):
    """The placement policy that run 3d showed v1 needed. Pure; adapters do the TF.

    v1 looked up the transform AT the contact's stamp, full stop -- and in the field
    that lookup lost 3 of 3 real contacts, because SLAM's map->odom runs tens of
    milliseconds behind wall clock and tf2 refuses to extrapolate into the future. The
    closed-loop rig never saw it: the rig pinned map->odom with a STATIC transform,
    which tf2 treats as timeless, so the exact-stamp lookup was unfalsifiable there.

    The policy, in order:

    1. `exact()` -- the transform at the contact's stamp. When the feed is caught up
       this is the strictly-correct pose and it is kept, staleness 0, path "exact".
    2. Only when `exact()` raises `PoseDataLagsStamp` (the adapter's translation of
       tf2's ExtrapolationException, and of nothing else): `latest()` returns the
       newest available pose and its stamp. Accept iff |stamp gap| <= `bound_s`,
       path "fallback". A gap beyond the bound is a genuine transform outage and
       raises `ContactPoseUnavailable` -- the honest refusal is preserved, it just no
       longer fires on the ordinary lag every moving contact has.

    Any other exception from `exact()` (frame missing, tree disconnected) propagates:
    those mean the pose is not merely late but untrustworthy, and no fallback answers
    that.

    `exact`: () -> (x, y, yaw). `latest`: () -> ((x, y, yaw), transform_stamp_s).
    """
    try:
        x, y, yaw = exact()
        return ResolvedContactPose(x, y, yaw, staleness_s=0.0, path="exact")
    except PoseDataLagsStamp:
        pass
    (x, y, yaw), tf_stamp_s = latest()
    staleness_s = float(stamp_s) - float(tf_stamp_s)
    if abs(staleness_s) > float(bound_s):
        raise ContactPoseUnavailable(
            f"latest transform is {staleness_s * 1000.0:+.0f} ms from the contact "
            f"stamp, beyond the {bound_s * 1000.0:.0f} ms bound -- that is an outage, "
            f"not lag"
        )
    return ResolvedContactPose(x, y, yaw, staleness_s=staleness_s, path="fallback")


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

    SUPERSEDED IN ITS ARITHMETIC, 2026-08-18, and left standing only because the geometry
    that replaces it has not been agreed yet. Two things above are now known to be false.
    (1) The derivation uses `ROBOT_RADIUS_M` = 0.145, which the costmap does not use --
    the real inscribed radius is `COSTMAP_INSCRIBED_RADIUS_M` = 0.1519 (M1/M2). (2) The
    claim that the disc's near edge "sits exactly on the footprint's front edge" describes
    a cell that footprint clearing deletes: the near cap of every disc is erased, and what
    actually survives is a crescent starting ~0.169 m out that nobody designed. The disc
    is not dangerous -- the closed-loop A/B shows RPP refuses the approach well before any
    of this matters -- it is simply not the shape this docstring claims.
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
