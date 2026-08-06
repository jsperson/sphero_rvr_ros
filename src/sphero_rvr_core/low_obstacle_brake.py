"""Camera low-obstacle contribution to the collision brake (pure/testable).

The lidar collision-stop supervisor is the safety authority. This adds a *purely
additive* forward-motion limit from the monocular low-obstacle cloud
(`/camera/low_obstacles`, points in base_link) so the rover also stops for things
below the 2D-lidar plane (chair legs, a shoe, a floor cable). It can only reduce
forward speed, never increase it, and never touches reverse or rotation -- so the
worst case of a camera false positive is an over-cautious stop, and a stale/absent
cloud simply yields no limit (the lidar behaviour is unchanged).

Geometry note: the detector publishes points relative to the camera origin but
labelled base_link; the camera sits ~0.06 m forward of base_link, so reported
ranges slightly UNDER-estimate the true base_link range -> the brake triggers a
touch early, which is the safe direction.
"""

import math


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
    """
    best = None
    turning = abs(angular_rad_s) > 1e-3 and abs(linear_mps) > 1e-3
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
        if best is None or rng < best:
            best = rng
    return best


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
