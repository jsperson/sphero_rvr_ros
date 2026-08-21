"""Belief-basin geometry from the flight-4 bag (2026-08-20 16:19 CDT).

Questions, from the transcript's pinned facts:
  - rover STOOD at (0.866, -0.009) at 16:42:21 (tool 7 look);
  - from (0.209, 0.121) it failed 4x to return east, 16:44:27-16:49:21.
  1. What was the global-costmap cost at the former standing cell before vs
     during the failures?
  2. How big is the contiguous >=INSCRIBED(253) patch containing that cell?
  3. During the basin window, how many LIVE lidar returns land inside that
     patch (physical truth vs belief paint)?
  4. How close is the patch edge to the rover's stuck pose?
"""
import math
from collections import deque

from mcap_ros2.reader import read_ros2_messages

BAG = "/private/tmp/claude-501/-Users-jsperson-source-sphero-rvr-ros/88c047c7-739a-43f8-a371-25aeb91bb240/scratchpad/basin.mcap"
STAND = (0.866, -0.009)          # tool 7 pose (where_am_i-verified frame)
STUCK = (0.209, 0.121)           # tool 11 pose mid-failures
T_PRE = 1787262141               # ~16:42:21 (tool 7)
T_MID = 1787262480               # ~16:48:00 (between failed gotos 13 and 14)
WINDOW = (1787262267, 1787262561)  # 16:44:27 .. 16:49:21

INSCRIBED = 253


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def compose(a, b):
    """2D pose compose a∘b, each (x, y, yaw)."""
    ax, ay, at = a
    bx, by, bt = b
    return (ax + bx * math.cos(at) - by * math.sin(at),
            ay + bx * math.sin(at) + by * math.cos(at),
            at + bt)


def cell_of(grid, x, y):
    meta = grid["meta"]
    cx = int((x - meta["ox"]) / meta["res"])
    cy = int((y - meta["oy"]) / meta["res"])
    if 0 <= cx < meta["w"] and 0 <= cy < meta["h"]:
        return cx, cy
    return None


def cost_at(grid, x, y):
    c = cell_of(grid, x, y)
    return None if c is None else grid["data"][c[1] * grid["meta"]["w"] + c[0]]


def patch(grid, seed_xy):
    """Contiguous >=INSCRIBED region containing the seed, as a set of cells."""
    seed = cell_of(grid, *seed_xy)
    meta, data = grid["meta"], grid["data"]
    if seed is None or data[seed[1] * meta["w"] + seed[0]] < INSCRIBED:
        return set()
    seen, q = {seed}, deque([seed])
    while q:
        cx, cy = q.popleft()
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nx, ny = cx + dx, cy + dy
            if (nx, ny) in seen or not (0 <= nx < meta["w"] and 0 <= ny < meta["h"]):
                continue
            if data[ny * meta["w"] + nx] >= INSCRIBED:
                seen.add((nx, ny))
                q.append((nx, ny))
    return seen


# ---- sweep the bag once, keeping what the questions need -------------------
grid_pre = grid_mid = None
map_odom = []                 # (t, pose)
odom_base = []
laser_static = None
scans = []                    # (t, msg) inside the window, decimated

for m in read_ros2_messages(BAG, topics=["/global_costmap/costmap_raw", "/tf",
                                         "/tf_static", "/scan"]):
    t = m.log_time_ns / 1e9
    msg = m.ros_msg
    topic = m.channel.topic
    if topic == "/global_costmap/costmap_raw":
        grid = {"meta": {"res": msg.metadata.resolution,
                         "w": msg.metadata.size_x, "h": msg.metadata.size_y,
                         "ox": msg.metadata.origin.position.x,
                         "oy": msg.metadata.origin.position.y},
                "data": bytes(msg.data), "t": t}
        if t <= T_PRE:
            grid_pre = grid
        if t <= T_MID:
            grid_mid = grid
    elif topic in ("/tf", "/tf_static"):
        for tr in msg.transforms:
            pose = (tr.transform.translation.x, tr.transform.translation.y,
                    yaw_of(tr.transform.rotation))
            key = (tr.header.frame_id, tr.child_frame_id)
            if key == ("map", "odom"):
                map_odom.append((t, pose))
            elif key == ("odom", "base_link"):
                odom_base.append((t, pose))
            elif key == ("base_link", "laser"):
                laser_static = pose
    elif topic == "/scan" and WINDOW[0] <= t <= WINDOW[1]:
        if len(scans) == 0 or t - scans[-1][0] > 2.0:      # ~1 scan / 2 s
            scans.append((t, msg))

print(f"grids: pre@{grid_pre['t']:.0f} mid@{grid_mid['t']:.0f}; "
      f"scans kept {len(scans)}; laser_static yaw "
      f"{math.degrees(laser_static[2]):.1f} deg")

res = grid_mid["meta"]["res"]
print(f"cost at former standing pose {STAND}: "
      f"pre={cost_at(grid_pre, *STAND)}  mid={cost_at(grid_mid, *STAND)}")
print(f"cost at stuck pose {STUCK}: mid={cost_at(grid_mid, *STUCK)}")

cells = patch(grid_mid, STAND)
if cells:
    xs = [grid_mid["meta"]["ox"] + (cx + 0.5) * res for cx, cy in cells]
    ys = [grid_mid["meta"]["oy"] + (cy + 0.5) * res for cx, cy in cells]
    area = len(cells) * res * res
    dmax = max(math.hypot(x - STAND[0], y - STAND[1]) for x, y in zip(xs, ys))
    dedge = min(math.hypot(x - STUCK[0], y - STUCK[1]) for x, y in zip(xs, ys))
    print(f"patch >=253 containing the standing pose: {len(cells)} cells = "
          f"{area:.2f} m^2, max radius from standing pose {dmax:.2f} m, "
          f"nearest patch cell to stuck rover {dedge:.2f} m")
    pre_patch = patch(grid_pre, STAND)
    print(f"same-seed patch in the PRE grid: {len(pre_patch)} cells")

    # ---- physical truth: live returns landing inside the patch -------------
    def nearest(series, t):
        return min(series, key=lambda kv: abs(kv[0] - t))[1]

    inside = total = 0
    for t, scan in scans:
        base = compose(nearest(map_odom, t), nearest(odom_base, t))
        laser = compose(base, laser_static)
        angle = scan.angle_min
        for r in scan.ranges:
            if scan.range_min < r < scan.range_max:
                total += 1
                px = laser[0] + r * math.cos(laser[2] + angle)
                py = laser[1] + r * math.sin(laser[2] + angle)
                if cell_of(grid_mid, px, py) in cells:
                    inside += 1
            angle += scan.angle_increment
    print(f"lidar truth over the basin window: {inside}/{total} returns land "
          f"inside the {area:.2f} m^2 belief-lethal patch "
          f"({100.0 * inside / max(1, total):.2f}%)")
else:
    print("standing pose not in a >=253 patch at T_MID")
