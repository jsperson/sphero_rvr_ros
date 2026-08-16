"""Camera low-obstacle contribution to the collision brake (pure/testable).

The lidar collision-stop supervisor is the safety authority. This adds a *purely
additive* forward-motion limit from a sub-lidar obstacle cloud (points in
base_link) so the rover also stops for things below the 2D-lidar plane (chair legs,
a shoe, a floor cable). It can only reduce
forward speed, never increase it, and never touches reverse or rotation -- so the
worst case of a camera false positive is an over-cautious stop, and a stale/absent
cloud simply yields no limit (the lidar behaviour is unchanged).

WHICH PRODUCER FEEDS IT IS A DEPLOYED FACT, NOT A FACT ABOUT THIS FILE. The
consumer reads `low_obstacle_topic` from `config/collision_stop.yaml`, which is
`/tof/obstacles` today -- the ToF, not the camera. This docstring named
`/camera/low_obstacles` until 2026-08-15 and was simply wrong about the flying
system; the same stale claim still sits in `explore.launch.py`'s `start_low_obstacle`
help text. Naming a topic here at all is a config-is-a-claim hazard, so the rule is:
the module says what SHAPE it consumes (base_link points), and the deployed YAML
says where they come from.

Geometry note, and it is producer-specific: the CAMERA detector publishes points
relative to the camera origin but labelled base_link, and the camera sits ~0.06 m
forward of base_link, so those ranges slightly UNDER-estimate the true base_link
range -> the brake triggers a touch early, which is the safe direction. ToF points
are already base_link, so no such offset applies to them.
"""

import math

#: Below this the command is not TRANSLATING, and a non-translating command has no
#: forward corridor to ask about. One constant, used by both the turning test and the
#: refusal below, so the two can never disagree about where translation begins --
#: which is exactly the gap a second threshold would open.
_MIN_TRANSLATION_MPS = 1e-3


class NoSweptPath(ValueError):
    """Raised when a command has no swept path to answer about.

    RAISED, NOT RETURNED, and the distinction is the whole point. `None` already
    means "the swept path is clear" in this module, and `forward_speed_scale(None,
    ...)` returns 1.0 -- full speed. So returning None for an unanswerable query
    would not be a refusal at all; it would be the most permissive answer the API
    can give, handed out for the one input the function cannot model. Fail-open,
    wearing a refusal's clothes.

    A raise cannot destabilise the flying system, because production never reaches
    it: `_apply_low_obstacle_brake` returns early on `linear_x <= 0.0` before this
    module is called. It can only fire for a NEW caller, at development time, which
    is precisely who this guard exists for.
    """


def nearest_forward_obstacle(points_xy, half_angle_rad, min_range_m=0.0, max_range_m=float("inf")):
    """Nearest forward range among base-frame (x_forward, y_left) points inside the
    forward cone (|bearing| <= half_angle_rad, x > 0), within [min_range, max_range].
    Returns the range in metres, or None if the cone is clear."""
    best = None
    for x, y in points_xy:
        if x <= 0.0:
            continue
        rng = math.hypot(x, y)
        if rng < min_range_m or rng > max_range_m:
            continue
        if abs(math.atan2(y, x)) <= half_angle_rad:
            if best is None or rng < best:
                best = rng
    return best


def swept_path_obstacle(points_xy, linear_mps, angular_rad_s, half_width_m,
                        min_range_m=0.0, max_range_m=float("inf")):
    """Nearest obstacle on the arc the rover is ACTUALLY about to drive, or None.

    A fixed forward cone is wrong whenever the rover is turning: a differential
    drive pivots about a centre off to one side, so the *flank* leads the turn and
    sweeps ground the nose never covers. That is how a chair leg gets hit during a
    left turn while a straight-ahead cone reports "clear".

    Travelling straight (|w| tiny) the swept region is the corridor
    |y| <= half_width, x > 0. Turning, the rover follows a circle of radius
    R = v / w centred at (0, R) in the base frame; a point threatens the robot when
    its distance from that centre falls within [|R| - half_width, |R| + half_width].
    Returns the straight-line range to the nearest such point (comparable to the
    cone version, so the existing stop/slow distances still apply).

    Raises `NoSweptPath` for a non-translating command -- see `_swept_path_ranges`.
    `None` here means CLEAR, so an unanswerable query cannot be reported as None
    without it reading as full speed ahead.
    """
    best = None
    for rng in _swept_path_ranges(points_xy, linear_mps, angular_rad_s, half_width_m,
                                  min_range_m, max_range_m):
        if best is None or rng < best:
            best = rng
    return best


def points_in_swept_path(points_xy, linear_mps, angular_rad_s, half_width_m,
                         min_range_m=0.0, max_range_m=float("inf")):
    """How many points the brake ACTUALLY CONSIDERED -- after the range window and
    the swept-path filter, not before.

    THE DEFECT THIS EXISTS TO KILL is a reporting one, and it cost two sessions.
    `/tof/state`'s `obstacle_zones` counts rule-B zones over the SENSOR'S WHOLE
    REACH; this brake acts only within [min_range, max_range] and only on the arc
    actually commanded. Nothing published the second number, so the first one stood
    in for it, and "zones cycled 0 -> 10 while cam_scale never left 1.00" read as a
    lost-detection bug in the detection-to-brake path.

    It was not. On 2026-08-15 those points sat at 0.542-1.563 m against a deployed
    `low_obstacle_max_range_m` of 0.60: out of reach, correctly ignored, and the
    brake engaged on the first frame the object was genuinely reachable. One extra
    number on the state line would have said so in five minutes.

    THE FAILURE FAMILY is "a correct number about the wrong population" (standards
    rule 1), and the check that catches it is: name the filter set beside the count.
    That is why this shares `_swept_path_ranges` with `swept_path_obstacle` rather
    than re-deriving the geometry -- a count filtered differently from the value it
    is meant to explain would be a NEW instance of the same defect, in the
    instrumentation built to detect it.

    Degenerate input: no points, or none surviving the filters, returns 0 -- which
    is a different fact from `swept_path_obstacle` returning None only because the
    cloud was empty, and the pair is what distinguishes them. A non-translating
    command raises `NoSweptPath` rather than returning 0, because 0 here means
    "looked and found nothing" and this function did not look.
    """
    return sum(1 for _ in _swept_path_ranges(points_xy, linear_mps, angular_rad_s,
                                             half_width_m, min_range_m, max_range_m))


def _swept_path_ranges(points_xy, linear_mps, angular_rad_s, half_width_m,
                       min_range_m, max_range_m):
    """Ranges of the points that threaten the commanded arc. ONE author for the
    swept-path geometry: `swept_path_obstacle` takes the minimum of this and
    `points_in_swept_path` takes its length, so the count can never describe a
    different set from the distance reported beside it.

    REFUSES a non-translating command rather than answering about the wrong motion.
    A stationary command and a pure pivot both have |v| below the translation
    threshold, and neither sweeps a forward corridor -- a pivot sweeps the
    footprint's corner CIRCLE, which is a different question this function does not
    model and deliberately does not try to. Until 2026-08-15 such a command fell
    through the `turning` test into the straight branch and got the corridor ahead
    of the robot: a confident, plausible, and wrong answer.

    Modelling the rotation annulus is explicitly NOT in scope here. Refusal is.
    """
    if abs(linear_mps) <= _MIN_TRANSLATION_MPS:
        raise NoSweptPath(
            f"linear_mps={linear_mps!r} is not translating, so there is no swept "
            f"path to report. A pivot sweeps the footprint's corner circle, not a "
            f"forward corridor -- ask the pivot gate, not this function."
        )
    turning = abs(angular_rad_s) > 1e-3 and abs(linear_mps) > _MIN_TRANSLATION_MPS
    if turning:
        radius = linear_mps / angular_rad_s          # +ve => centre to the left
        cx, cy = 0.0, radius
        inner, outer = abs(radius) - half_width_m, abs(radius) + half_width_m
    for x, y in points_xy:
        rng = math.hypot(x, y)
        if rng < min_range_m or rng > max_range_m:
            continue
        if turning:
            d = math.hypot(x - cx, y - cy)
            if not (inner <= d <= outer):
                continue
            # only count what is ahead along the turn, not behind the robot
            if linear_mps > 0.0 and x <= 0.0:
                continue
            if linear_mps < 0.0 and x >= 0.0:
                continue
        else:
            if abs(y) > half_width_m:
                continue
            if linear_mps >= 0.0 and x <= 0.0:
                continue
            if linear_mps < 0.0 and x >= 0.0:
                continue
        yield rng


def forward_speed_scale(nearest_m, stop_distance_m, slow_distance_m, min_forward_scale=0.0):
    """Forward-speed scale in [0, 1] for a camera obstacle at `nearest_m`:
    1.0 if clear (None) or beyond slow_distance; 0.0 at/inside stop_distance; a
    ramp from `min_forward_scale` (at stop) to 1.0 (at slow) in between. Flooring
    at `min_forward_scale` avoids scaling into the sub-breakaway creep/grind range
    (mirrors the lidar brake)."""
    if nearest_m is None:
        return 1.0
    if nearest_m <= stop_distance_m:  # stop wins ties (safe direction)
        return 0.0
    if nearest_m >= slow_distance_m:
        return 1.0
    span = slow_distance_m - stop_distance_m
    if span <= 0.0:
        return 0.0
    frac = (nearest_m - stop_distance_m) / span
    return min_forward_scale + (1.0 - min_forward_scale) * frac
