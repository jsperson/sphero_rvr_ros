"""Belief-basin round 3: the door itself.

The corridor was OPEN in belief until t+221s, CLOSED for the last 71s. What
cells CHANGED to >=253 between the last open grid and the closed grids — the
door — and does live lidar during the closed period support them?
"""
import math
from collections import deque

from mcap_ros2.reader import read_ros2_messages

BAG = "/private/tmp/claude-501/-Users-jsperson-source-sphero-rvr-ros/88c047c7-739a-43f8-a371-25aeb91bb240/scratchpad/basin.mcap"
STAND = (0.866, -0.009)
STUCK = (0.209, 0.121)
WINDOW = (1787262267, 1787262561)
T_OPEN = WINDOW[0] + 221           # last open grid
T_CLOSED = WINDOW[0] + 250         # mid closed spell
INSCRIBED = 253


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def compose(a, b):
    ax, ay, at = a
    bx, by, bt = b
    return (ax + bx * math.cos(at) - by * math.sin(at),
            ay + bx * math.sin(at) + by * math.cos(at), at + bt)


grid_open = grid_closed = None
map_odom, odom_base, laser_static, scans = [], [], None, []
for m in read_ros2_messages(BAG, topics=["/global_costmap/costmap_raw", "/tf",
                                         "/tf_static", "/scan"]):
    t = m.log_time_ns / 1e9
    msg = m.ros_msg
    topic = m.channel.topic
    if topic == "/global_costmap/costmap_raw":
        if t <= T_OPEN or grid_open is None:
            grid_open = (t, msg)
        if t <= T_CLOSED or grid_closed is None:
            grid_closed = (t, msg)
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
    elif topic == "/scan" and T_OPEN + 2 <= t <= WINDOW[1]:
        if not scans or t - scans[-1][0] > 2.0:
            scans.append((t, msg))


def unpack(pair):
    t, msg = pair
    return {"meta": {"res": msg.metadata.resolution,
                     "w": msg.metadata.size_x, "h": msg.metadata.size_y,
                     "ox": msg.metadata.origin.position.x,
                     "oy": msg.metadata.origin.position.y},
            "data": bytes(msg.data), "t": t}


go, gc = unpack(grid_open), unpack(grid_closed)
assert (go["meta"]["w"], go["meta"]["h"]) == (gc["meta"]["w"], gc["meta"]["h"])
meta = gc["meta"]
res = meta["res"]
print(f"open grid @{go['t'] - WINDOW[0]:.1f}s, closed grid @"
      f"{gc['t'] - WINDOW[0]:.1f}s, {meta['w']}x{meta['h']} cells @ {res} m")

new_lethal = set()
for i, (a, b) in enumerate(zip(go["data"], gc["data"])):
    if b >= INSCRIBED > a:
        new_lethal.add((i % meta["w"], i // meta["w"]))
print(f"cells newly >=253 (the door candidates): {len(new_lethal)} "
      f"({len(new_lethal) * res * res:.2f} m^2)")

xs = [meta["ox"] + (cx + 0.5) * res for cx, cy in new_lethal]
ys = [meta["oy"] + (cy + 0.5) * res for cx, cy in new_lethal]
if new_lethal:
    d_stand = [math.hypot(x - STAND[0], y - STAND[1]) for x, y in zip(xs, ys)]
    d_stuck = [math.hypot(x - STUCK[0], y - STUCK[1]) for x, y in zip(xs, ys)]
    print(f"distance from STANDING pose: min {min(d_stand):.2f} max {max(d_stand):.2f} m")
    print(f"distance from STUCK pose:    min {min(d_stuck):.2f} max {max(d_stuck):.2f} m")

# lidar truth on the DOOR cells only, closed period only. A door cell is
# "supported" if any return lands IN it or within one cell of it (marking
# uncertainty), "unsupported" otherwise.
def nearest(series, t):
    return min(series, key=lambda kv: abs(kv[0] - t))[1]

hit = set()
total_returns = 0
for t, scan in scans:
    laser = compose(compose(nearest(map_odom, t), nearest(odom_base, t)),
                    laser_static)
    angle = scan.angle_min
    for r in scan.ranges:
        if scan.range_min < r < scan.range_max:
            total_returns += 1
            px = laser[0] + r * math.cos(laser[2] + angle)
            py = laser[1] + r * math.sin(laser[2] + angle)
            cx = int((px - meta["ox"]) / res)
            cy = int((py - meta["oy"]) / res)
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    if (cx + dx, cy + dy) in new_lethal:
                        hit.add((cx + dx, cy + dy))
        angle += scan.angle_increment
print(f"scans in closed period: {len(scans)}, returns: {total_returns}")
print(f"door cells supported by a live return (±1 cell): {len(hit)}/"
      f"{len(new_lethal)} ({100.0 * len(hit) / max(1, len(new_lethal)):.1f}%)")
print(f"UNSUPPORTED door cells (belief without physics): "
      f"{len(new_lethal) - len(hit)}")

# where are the unsupported ones?
unsup = new_lethal - hit
if unsup:
    ux = [meta["ox"] + (cx + 0.5) * res for cx, cy in unsup]
    uy = [meta["oy"] + (cy + 0.5) * res for cx, cy in unsup]
    du = [math.hypot(x - STAND[0], y - STAND[1]) for x, y in zip(ux, uy)]
    print(f"unsupported cells: {len(unsup)} ({len(unsup) * res * res:.2f} m^2), "
          f"distance from standing pose min {min(du):.2f} max {max(du):.2f} m")
