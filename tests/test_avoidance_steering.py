"""Revert-proofs for the gentle turn-away (docs/turning_batch_design.md item 1).

The mission this exists to save is gauntlet run 114626: 12 aborts, every one of them
the same loop -- drive forward, camera low-obstacle brake zeroes the output at
0.50 m, 2 s of suppression trips the stall ladder, rung 1 reverses (always granted,
because nothing brakes reverse), re-approach on the same heading, repeat, escape
budget spent, abort. Five aborts in a row ended the mission at 10.128 m2.

Nothing in the stack steers around anything today, so most of these fail against
HEAD by construction. The two that carry real weight are the ones that would pass
against a plausible WRONG implementation:

  * `test_clear_ray_endpoints_never_steer` -- the camera cloud carries clear-ray
    endpoints at 1.8 m mixed in with obstacle marks, and a consumer that forgets the
    range filter steers away from proof that the floor is clear.
  * `test_blockers_beside_the_corridor_are_ignored` -- a law without the corridor
    test swerves around everything it passes.
"""

import math

import pytest

from sphero_rvr_core.decisive_control import (
    AvoidanceConfig,
    DecisiveControlConfig,
    avoidance_heading_offset,
    camera_points_to_polar,
    compute_drive_command,
    corridor_blocker,
)

CFG = AvoidanceConfig()


def _settle(nearest, cycles=10, open_bearing=0.0, ladder_active=False, cfg=CFG):
    """Run the rate limiter to steady state, as the control loop would."""
    offset = 0.0
    for _ in range(cycles):
        offset = avoidance_heading_offset(nearest, open_bearing, offset,
                                          ladder_active, cfg)
    return offset


# --------------------------------------------------------------------------- #
# 1. The headline: a curve begins while the rover is still allowed to move.
# --------------------------------------------------------------------------- #

def test_steering_curves_before_the_brake_fires():
    """A blocker 8 deg off axis at 0.60 m must produce a lean AWAY from it.

    0.60 m is inside the engagement radius (0.90) and still outside the camera
    brake's hard-zero (0.50), which is the entire point: the rover is still moving,
    still permitted, and now already turning.
    """
    bearing = math.radians(8.0)
    nearest = corridor_blocker([(0.60, bearing)], CFG)
    assert nearest is not None, "a blocker dead in the corridor must be seen"

    offset = _settle(nearest)
    assert offset < 0.0, (
        "blocker to the LEFT must lean the target heading RIGHT; "
        f"got {offset:+.3f} rad")
    urgency = (CFG.engage_m - 0.60) / (CFG.engage_m - CFG.stop_ref_m)
    assert offset == pytest.approx(-urgency * CFG.max_offset_rad, abs=1e-6)


def test_the_lean_grows_as_the_gate_approaches():
    """Urgency must ramp: barely anything at the engagement radius, everything at
    the brake distance. A step function would swerve; a flat gain would dawdle."""
    far = _settle(corridor_blocker([(0.88, 0.0)], CFG), open_bearing=1.0)
    mid = _settle(corridor_blocker([(0.70, 0.0)], CFG), open_bearing=1.0)
    near = _settle(corridor_blocker([(0.50, 0.0)], CFG), open_bearing=1.0)
    assert 0.0 < far < mid < near
    assert near == pytest.approx(CFG.max_offset_rad, abs=1e-6)


def test_nothing_steers_beyond_the_engagement_radius():
    assert corridor_blocker([(0.95, 0.0)], CFG) is None
    assert _settle(None) == 0.0


# --------------------------------------------------------------------------- #
# 2. The trap: this topic is not a pure obstacle cloud.
# --------------------------------------------------------------------------- #

def test_clear_ray_endpoints_never_steer():
    """`/camera/low_obstacles` mixes CLEAR-RAY endpoints (published at
    `clear_range_m` 1.8 m to raytrace stale marks out of the costmap) in with real
    obstacle marks, at the same z, distinguishable only by range. The deployed
    camera brake is immune purely because `camera_max_range_m` is 1.20.

    A steering law that consumes the cloud without that filter reads every clear ray
    as an obstacle -- i.e. it swerves away from the evidence that the floor ahead is
    empty. It would look like working code in every bench test that only ever places
    real objects. This test fails against exactly that implementation.
    """
    clear_rays = [(1.8 * math.cos(a), 1.8 * math.sin(a))
                  for a in (math.radians(d) for d in range(-30, 31, 5))]
    assert camera_points_to_polar(clear_rays, CFG) == []
    assert corridor_blocker(camera_points_to_polar(clear_rays, CFG), CFG) is None
    assert _settle(corridor_blocker(camera_points_to_polar(clear_rays, CFG), CFG)) == 0.0


def test_a_real_mark_among_clear_rays_still_steers():
    """The filter must not be so blunt that it discards marks too. Same cloud as
    above plus one genuine low obstacle at 0.62 m, 10 deg right."""
    cloud = [(1.8 * math.cos(a), 1.8 * math.sin(a))
             for a in (math.radians(d) for d in range(-30, 31, 5))]
    cloud.append((0.62 * math.cos(math.radians(-10.0)),
                  0.62 * math.sin(math.radians(-10.0))))
    nearest = corridor_blocker(camera_points_to_polar(cloud, CFG), CFG)
    assert nearest is not None and nearest[0] == pytest.approx(0.62, abs=1e-3)
    assert _settle(nearest) > 0.0, "a blocker on the right must lean left"


def test_points_inside_the_cameras_near_limit_are_not_marks():
    """Below `camera_min_range_m` the monocular detector is unreliable, and the
    deployed brake ignores that band. A steering law must not be more credulous
    about the same sensor than the safety layer is."""
    assert camera_points_to_polar([(0.30, 0.0)], CFG) == []


# --------------------------------------------------------------------------- #
# 3. Corridor, goal, and the supervisor's cap: the clauses that bound the law.
# --------------------------------------------------------------------------- #

def test_blockers_beside_the_corridor_are_ignored():
    """Something we will drive PAST is not something to steer around. At 0.60 m and
    40 deg the blocker sits 0.386 m off the centreline -- twice the corridor half
    width -- and reacting to it would make the rover swerve down a hallway.

    Fails against a law that reacts to bearing alone.
    """
    beside = (0.60, math.radians(40.0))
    assert abs(beside[0] * math.sin(beside[1])) > CFG.corridor_half_width_m
    assert corridor_blocker([beside], CFG) is None


def test_the_corridor_matches_the_gate_not_the_body():
    """The corridor half-width is 0.18 m, not the robot's own 0.10: what stops the
    rover is the camera brake, which tests its swept path at half-width 0.16. A law
    that only clears the BODY leaves the obstacle inside the GATE's corridor and
    gets braked anyway -- a fix that stops just short of working."""
    assert CFG.corridor_half_width_m > 0.16
    just_inside = (0.60, math.asin(0.17 / 0.60))
    assert corridor_blocker([just_inside], CFG) is not None


def test_blockers_behind_the_goal_are_ignored():
    """We stop at the goal, so a wall 0.2 m past it never threatens us. Steering
    away from it would be steering away from the destination -- and near the end of
    a leg that is indistinguishable from refusing to arrive."""
    assert corridor_blocker([(0.70, 0.0)], CFG, max_relevant_m=0.50) is None
    assert corridor_blocker([(0.70, 0.0)], CFG, max_relevant_m=1.00) is not None


def test_the_lean_cannot_outrun_the_supervisors_angular_cap():
    """The supervisor clamps angular to 0.40 rad/s. A heading offset big enough to
    ask for more would be silently clipped, and the controller would then be lying
    to its own ladder about what it requested -- the same commanded-vs-achieved
    substitution that made the D32 guard reject every real pivot.
    """
    drive = DecisiveControlConfig()
    worst = compute_drive_command(CFG.max_offset_rad, 5.0, drive)
    assert abs(worst.angular_rad_s) <= 0.40 + 1e-9, (
        f"a full-offset arc asks for {worst.angular_rad_s:+.3f} rad/s, above the "
        "supervisor's 0.40 cap")


# --------------------------------------------------------------------------- #
# 4. Rate limiting, tie-breaking, and the ladder's right of way.
# --------------------------------------------------------------------------- #

def test_the_offset_is_rate_limited_in_both_directions():
    """One noisy camera frame must not snap the heading, and releasing must be as
    smooth as engaging. The camera runs ~5 Hz into a 10 Hz loop, so half of all
    camera-driven blockers are already a cycle stale."""
    nearest = corridor_blocker([(0.50, math.radians(5.0))], CFG)
    first = avoidance_heading_offset(nearest, 0.0, 0.0, False, CFG)
    assert abs(first) == pytest.approx(CFG.max_offset_step_rad, abs=1e-9)
    settled = _settle(nearest)
    back = avoidance_heading_offset(None, 0.0, settled, False, CFG)
    assert abs(back - settled) == pytest.approx(CFG.max_offset_step_rad, abs=1e-9)


def test_a_blocker_dead_ahead_turns_toward_the_open_side():
    """Straight ahead gives no hint about which way to escape, so the widest gap
    decides -- the same base-frame bearing the ladder already uses for rungs 3 and
    4. Without this the law would have to pick blindly at the exact geometry where
    picking wrong drives into the closed side."""
    nearest = corridor_blocker([(0.60, 0.0)], CFG)
    assert _settle(nearest, open_bearing=+1.2) > 0.0
    assert _settle(nearest, open_bearing=-1.2) < 0.0


def test_steering_is_bypassed_while_a_rung_is_running():
    """A rung is what happens after normal control has already failed. Letting the
    steering law bias a heading during an escape would have two authors for one
    motion, which is the failure mode the ladder was built to end.

    Note this releases through the rate limiter rather than snapping to zero: a
    discontinuity is a discontinuity whichever direction it points.
    """
    nearest = corridor_blocker([(0.55, math.radians(5.0))], CFG)
    engaged = _settle(nearest)
    assert engaged != 0.0
    assert _settle(nearest, cycles=20, ladder_active=True) == 0.0


# --------------------------------------------------------------------------- #
# 5. Replay: the recorded approach, with its limits stated.
# --------------------------------------------------------------------------- #

# Recorded rows from run_20260811_114626.csv, t=322.01..322.61, the cleanest
# forward approach in the mission: (front range m, /cmd_vel_motor linear x).
# The supervisor scaled forward down to 0.175 m/s and then the CAMERA brake cut it
# to zero at front 0.507 m -- the mission's whole failure mode in seven rows.
#
# HONEST LIMITATION, and it must not be described as equivalent to the chair-pin
# replay: the raw scan and the camera cloud were NOT recorded, so the blocker here
# is RECONSTRUCTED from the state line's front-sector range rather than replayed
# from sensor data. What this proves is that the law engages, in time, on real
# recorded ranges. What it cannot prove is clearance in the real geometry.
REPLAY_APPROACH = [
    (0.854, 0.000), (0.805, 0.200), (0.653, 0.200), (0.544, 0.186),
    (0.518, 0.175), (0.507, 0.000), (0.496, 0.000),
]


def test_replay_of_the_recorded_approach_starts_turning_before_the_cut():
    offset = 0.0
    offsets = []
    for front, _out in REPLAY_APPROACH:
        nearest = corridor_blocker([(front, math.radians(4.0))], CFG)
        offset = avoidance_heading_offset(nearest, 0.0, offset, False, CFG)
        offsets.append(offset)

    cut_index = next(i for i, (_f, out) in enumerate(REPLAY_APPROACH)
                     if out == 0.0 and i > 0)
    assert offsets[0] != 0.0, "the very first recorded range is already inside 0.90 m"
    assert abs(offsets[cut_index - 1]) > 0.15, (
        "by the row before the camera cut the rover should be leaning hard; got "
        f"{offsets[cut_index - 1]:+.3f} rad")
    assert all(o <= 0.0 for o in offsets), "the lean must not change sign mid-approach"


def test_the_replay_is_a_marginal_case_and_says_so():
    """Reported rather than hidden: in this recorded episode the blocker entered the
    front sector at 0.854 m and the output was cut 0.6 s later, so the law gets ~6
    cycles. The geometry in the design note wants ~0.38 m of forward travel to swing
    the corridor clear, and 0.2 m/s x 0.6 s is 0.12 m. The fix gives this approach a
    turn it currently never starts; it does NOT guarantee it clears in time when an
    obstacle appears that late. Cases where the blocker is visible from 0.90 m have
    the full 2.0 s.
    """
    travelled = 0.2 * (len(REPLAY_APPROACH) - 1) * 0.1
    assert travelled < 0.38
