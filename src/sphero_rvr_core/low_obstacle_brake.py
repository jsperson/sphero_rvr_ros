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
