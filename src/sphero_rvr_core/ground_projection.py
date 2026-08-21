"""Clear-ray endpoints for camera-sourced costmap layers. One function survives.

This module was the inverse-perspective-mapping core of the monocular low-obstacle
detector (pixel_to_ground, object_height_m, ground_to_cell). That path was retired
by the 2026-08-10 rangefinder decision and deleted with bucket zero of the
2026-08-21 project review — the ToF owns low obststacles now. What remains is the one
function a LIVE consumer imports: `semantic_map_node` emits clear-ray endpoints so
Nav2's raytrace can un-mark stale camera marks (the load-bearing clearing mechanism
from the camera-low-obstacle design, docs/camera_low_obstacle_design.md).

Pure (no ROS). Image pixel u is right of centre; robot frame is x forward, y left.
"""

import math
from typing import Tuple


def clear_ray_point(u: float, cx: float, fx: float, range_m: float) -> Tuple[float, float]:
    """A ground point `range_m` away on image column `u`'s bearing, as (forward, left)
    in the robot frame.

    Used to emit "the floor is clear this far out" endpoints on bearings with no
    obstacle. Nav2 raytraces sensor->point and clears everything in between, which is
    the ONLY way a stale camera mark gets removed (a bearing that publishes nothing
    is never raytraced, so its mark persists forever). Keep `range_m` greater than
    the costmap's `obstacle_max_range` so these endpoints clear without being marked.
    """
    theta = math.atan2(u - cx, fx)  # +u is right of centre -> +theta -> -left
    return (range_m * math.cos(theta), -range_m * math.sin(theta))
