"""The radius prune must be invisible to blocked/clear -- the equivalence property.

Item 2's revised fix (2026-08-18 night): points beyond corner_radius + |v|*horizon
are pruned before the stepped sweep, exact by the footprint-reach bound (and for
pivots by the rotation-preserves-radius theorem). These tests drive the PUBLIC
path (prune active) and the extracted stepped path (no prune) on identical inputs
and require identical blocked/clear verdicts -- any disagreement is a loud failure.
Seeded RNG throughout; no wall-clock randomness (resume discipline).
"""

import math
import random

import pytest

from sphero_rvr_driver.collision_stop import (
    CollisionStopConfig,
    ScanInput,
    Transform2D,
    TwistCommand,
    _projected_trajectory_stepped,
    _valid_base_points,
    evaluate_projected_trajectory,
)

CFG = CollisionStopConfig()
HORIZON = CFG.requested_cmd_timeout_s + CFG.measured_stop_time_s
MARGIN = CFG.trajectory_clearance_margin_m
FRONT, REAR = CFG.footprint_front_m + MARGIN, CFG.footprint_rear_m + MARGIN
LEFT, RIGHT = CFG.footprint_left_m + MARGIN, CFG.footprint_right_m + MARGIN
CORNER = math.hypot(max(FRONT, REAR), max(LEFT, RIGHT))


def scan_of(points):
    """A 360-ray scan whose finite returns are exactly `points` (base frame)."""
    ranges = [float("inf")] * 360
    for x, y in points:
        bearing = math.atan2(y, x)
        idx = int((bearing + math.pi) / (2 * math.pi / 360)) % 360
        ranges[idx] = math.hypot(x, y)
    return ScanInput(
        ranges=tuple(ranges), angle_min=-math.pi,
        angle_increment=2 * math.pi / 360, range_min=0.05, range_max=8.0,
        stamp=0.0, received_at=0.0, frame_id="base_link",
        transform_to_base=Transform2D(), transform_error=None,
    )


def stepped_reference(scan, command):
    """The stepped sweep over ALL of the scan's points, extracted EXACTLY the way
    the public path extracts them (same binning, same range gates) -- comparing
    raw coordinates against ray-quantized ones flips millimetre boundary cases
    and tests the harness, not the prune."""
    points = _valid_base_points(scan, CFG, Transform2D())
    linear_x, angular_z = float(command.linear_x), float(command.angular_z)
    linear_steps = math.ceil(abs(linear_x) * HORIZON / 0.01)
    angular_steps = math.ceil(abs(angular_z) * HORIZON / math.radians(2.0))
    step_count = max(1, linear_steps, angular_steps)
    return _projected_trajectory_stepped(
        list(points), command, linear_x, angular_z, HORIZON, step_count,
        front=FRONT, rear=REAR, left=LEFT, right=RIGHT,
    )


COMMANDS = [
    TwistCommand(0.0, 3.55),     # the curve floor -- the flap's own regime
    TwistCommand(0.0, 5.83),     # the ceiling
    TwistCommand(0.0, -3.55),    # signed
    TwistCommand(0.0, 0.4),      # sub-floor pivot (the old arc-cap value)
    TwistCommand(0.08, 0.0),     # straight
    TwistCommand(-0.08, 0.0),    # reverse
    TwistCommand(0.1, 0.8),      # arc
    TwistCommand(0.2, -1.2),     # arc, signed
]


def test_the_equivalence_property_holds_across_seeded_random_scenes():
    """400 scene/command pairs: the pruned public path and the unpruned stepped
    reference must agree on blocked/clear EVERY time. One disagreement is a
    safety-geometry bug, not a flake."""
    rng = random.Random(20260818)
    disagreements = []
    for trial in range(50):
        n = rng.randint(1, 8)
        points = []
        for _ in range(n):
            radius = rng.choice([
                rng.uniform(0.09, CORNER),            # inside reach
                rng.uniform(CORNER, CORNER + 0.05),   # boundary band
                rng.uniform(0.4, 3.0),                # room scale
            ])
            bearing = rng.uniform(-math.pi, math.pi)
            points.append((radius * math.cos(bearing), radius * math.sin(bearing)))
        for command in COMMANDS:
            scan = scan_of(points)
            public = evaluate_projected_trajectory(scan, CFG, command)
            reference = stepped_reference(scan, command)
            if public.blocked != reference.blocked:
                disagreements.append((trial, command, points,
                                      public.blocked, reference.blocked))
    assert not disagreements, f"prune changed a verdict: {disagreements[:3]}"


def test_a_wall_scene_under_a_floor_rate_pivot_prunes_to_nothing_and_stays_clear():
    """The flap's exact scenario: room walls, fast pivot. Everything prunes, the
    verdict is clear, and the clearance reported is the honest lower bound."""
    points = [(1.5 * math.cos(a), 1.5 * math.sin(a))
              for a in (0.1, 1.0, 2.0, -2.0)]
    result = evaluate_projected_trajectory(scan_of(points), CFG,
                                           TwistCommand(0.0, 3.55))
    assert result.blocked is False
    assert result.minimum_clearance_m == pytest.approx(1.5 - CORNER, abs=0.02)


def test_boundary_points_are_kept_not_pruned():
    """A point AT max_reach survives into the stepped sweep (<= keeps it): the
    prune only removes points STRICTLY beyond any possible contact."""
    command = TwistCommand(0.0, 3.55)
    at_reach = (CORNER * math.cos(0.3), CORNER * math.sin(0.3))
    just_beyond = ((CORNER + 0.02) * math.cos(0.3), (CORNER + 0.02) * math.sin(0.3))
    blocked_at = evaluate_projected_trajectory(scan_of([at_reach]), CFG, command)
    reference = stepped_reference(scan_of([at_reach]), command)
    assert blocked_at.blocked == reference.blocked
    clear_beyond = evaluate_projected_trajectory(scan_of([just_beyond]), CFG, command)
    assert clear_beyond.blocked is False


def test_pivot_travel_term_is_zero_and_arc_travel_widens_reach():
    """For an arc, a point beyond the pivot corner but inside corner + travel must
    NOT be pruned -- the origin moves. Equivalence covers correctness; this pins
    the reach formula's |v| term specifically."""
    command = TwistCommand(0.2, 0.0)          # 0.2 m/s straight, travel 0.1 m
    x = CORNER + 0.05                          # beyond pivot reach, inside travel
    public = evaluate_projected_trajectory(scan_of([(x, 0.0)]), CFG, command)
    reference = stepped_reference(scan_of([(x, 0.0)]), command)
    assert public.blocked == reference.blocked == True  # noqa: E712


def test_pruned_clear_points_veto_the_all_receding_fail_closed_branch():
    """The subtlety the safety suite caught on first contact: an overlapped point
    receding under reverse plus a FAR wall used to leave the fail-closed
    all-receding branch unfired (the wall was in the loop); pruning the wall must
    not change that."""
    # inside the rear-left rectangle AND above min_range_m (0.08) -- a nearer
    # point is invisible to the scan pipeline entirely, which is its own filter,
    # not this test's subject
    overlapped = (-0.09, 0.05)
    wall = (2.0, 0.0)
    command = TwistCommand(0.08, 0.0)        # forward: rear point recedes
    public = evaluate_projected_trajectory(scan_of([overlapped, wall]), CFG, command)
    assert public.blocked is False
    assert public.moving_away_point_count == 1
