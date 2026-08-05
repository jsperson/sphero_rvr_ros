"""Inverse perspective mapping: project an image pixel onto the ground plane.

For monocular low-obstacle detection, a detected obstacle's ground-contact pixel is
mapped to a metric (forward, left) point in the robot base frame under a flat-floor
assumption, using the camera intrinsics and its pose (height above the floor + tilt).
That point can then be marked in the costmap or fed to the collision brake.

Pure (no ROS) so it can be unit-tested. Frame conventions:
* image pixel (u, v): u right, v down; principal point (cx, cy), focal (fx, fy).
* camera OPTICAL frame (OpenCV): x right, y down, z forward.
* robot/ground frame: x forward, y left, z up; camera at height `cam_height_m`,
  optical axis tilted DOWN from horizontal by `cam_tilt_down_rad` (negative = tilted
  up). The floor is z = 0.
"""

import math
from typing import Optional, Tuple


def _optical_to_robot(cam_tilt_down_rad: float):
    """Rotation (3x3, row-major tuples) taking an optical-frame vector to the robot
    ground frame, for a camera pitched DOWN by `cam_tilt_down_rad`."""
    # Level optical->robot: x_o(right)->(0,-1,0), y_o(down)->(0,0,-1), z_o(fwd)->(1,0,0)
    r0 = ((0.0, 0.0, 1.0), (-1.0, 0.0, 0.0), (0.0, -1.0, 0.0))
    # Down-tilt = rotation about robot +Y (left) by phi: x->(cos,0,-sin), z->(sin,0,cos)
    c, s = math.cos(cam_tilt_down_rad), math.sin(cam_tilt_down_rad)
    ry = ((c, 0.0, s), (0.0, 1.0, 0.0), (-s, 0.0, c))
    # R = Ry * R0
    return tuple(
        tuple(sum(ry[i][k] * r0[k][j] for k in range(3)) for j in range(3))
        for i in range(3)
    )


def pixel_to_ground(
    u: float,
    v: float,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    cam_height_m: float,
    cam_tilt_down_rad: float,
) -> Optional[Tuple[float, float]]:
    """Ground point (forward_x, left_y) in the robot base frame for pixel (u, v), or
    None if the ray points at/above the horizon (never meets the floor)."""
    # Ray in the optical frame.
    rx, ry_, rz = (u - cx) / fx, (v - cy) / fy, 1.0
    n = math.sqrt(rx * rx + ry_ * ry_ + rz * rz)
    rx, ry_, rz = rx / n, ry_ / n, rz / n
    r = _optical_to_robot(cam_tilt_down_rad)
    # Ray in robot frame.
    dx = r[0][0] * rx + r[0][1] * ry_ + r[0][2] * rz
    dy = r[1][0] * rx + r[1][1] * ry_ + r[1][2] * rz
    dz = r[2][0] * rx + r[2][1] * ry_ + r[2][2] * rz
    if dz >= -1e-9:
        return None  # not pointing downward -> no ground intersection
    t = cam_height_m / (-dz)  # camera at (0,0,h); floor z=0
    return (t * dx, t * dy)


def ground_to_cell(
    x_forward: float,
    y_left: float,
    origin_x: float,
    origin_y: float,
    res: float,
    robot_x: float = 0.0,
    robot_y: float = 0.0,
    robot_yaw: float = 0.0,
) -> Tuple[int, int]:
    """Map a base-frame ground point to a costmap cell, given the robot pose in the
    costmap frame (default: robot at the costmap origin, no rotation)."""
    cos_y, sin_y = math.cos(robot_yaw), math.sin(robot_yaw)
    wx = robot_x + x_forward * cos_y - y_left * sin_y
    wy = robot_y + x_forward * sin_y + y_left * cos_y
    return (int((wx - origin_x) / res), int((wy - origin_y) / res))
