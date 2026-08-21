"""Belief-basin round 2: the corridor, not the pose.

Round 1 showed both poses in inflation gradient (207/202), not lethal. So the
question is CONNECTIVITY: through the basin window, can belief-passable cells
(<253) join the stuck pose to the former standing pose at all? And where the
blockage is, is it physically real (lidar returns) or belief-only?
"""
import math
from collections import deque

from mcap_ros2.reader import read_ros2_messages

BAG = "/private/tmp/claude-501/-Users-jsperson-source-sphero-rvr-ros/88c047c7-739a-43f8-a371-25aeb91bb240/scratchpad/basin.mcap"
STAND = (0.866, -0.009)
STUCK = (0.209, 0.121)
WINDOW = (1787262267, 1787262561)
INSCRIBED = 253


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def compose(a, b):
    ax, ay, at = a
    bx, by, bt = b
    return (ax + bx * math.cos(at) - by * math.sin(at),
            ay + bx * math.sin(at) + by * math.cos(at), at + bt)


def to_grid(meta, x, y):
    cx = int((x - meta["ox"]) / meta["res"])
    cy = int((y - meta["oy"]) / meta["res"])
    return (cx, cy) if 0 <= cx < meta["w"] and 0 <= cy < meta["h"] else None


def connected(grid, a_xy, b_xy):
    meta, data = grid["meta"], grid["data"]
    a, b = to_grid(meta, *a_xy), to_grid(meta, *b_xy)
    if a is None or b is None:
        return False
    if data[a[1] * meta["w"] + a[0]] >= INSCRIBED:
        return False
    seen, q = {a}, deque([a])
    while q:
        cx, cy = q.popleft()
        if (cx, cy) == b:
            return True
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            n = (cx + dx, cy + dy)
            if (n in seen or not (0 <= n[0] < meta["w"] and 0 <= n[1] < meta["h"])):
                continue
            if data[n[1] * meta["w"] + n[0]] < INSCRIBED:
                seen.add(n)
                q.append(n)
    return False


grids = []
map_odom, odom_base, laser_static, scans = [], [], None, []
for m in read_ros2_messages(BAG, topics=["/global_costmap/costmap_raw", "/tf",
                                         "/tf_static", "/scan"]):
    t = m.log_time_ns / 1e9
    msg = m.ros_msg
    topic = m.channel.topic
    if topic == "/global_costmap/costmap_raw" and WINDOW[0] - 60 <= t <= WINDOW[1] + 10:
        grids.append({"meta": {"res": msg.metadata.resolution,
                               "w": msg.metadata.size_x, "h": msg.metadata.size_y,
                               "ox": msg.metadata.origin.position.x,
                               "oy": msg.metadata.origin.position.y},
                      "data": bytes(msg.data), "t": t})
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
        if not scans or t - scans[-1][0] > 2.0:
            scans.append((t, msg))

in_window = [g for g in grids if WINDOW[0] <= g["t"] <= WINDOW[1]]
before = [g for g in grids if g["t"] < WINDOW[0]]
print(f"grids: {len(before)} pre-window, {len(in_window)} in window")

blocked = [g for g in in_window if not connected(g, STUCK, STAND)]
print(f"connectivity stuck->standing (<253 passable): BLOCKED in "
      f"{len(blocked)}/{len(in_window)} window grids; pre-window blocked in "
      f"{sum(not connected(g, STUCK, STAND) for g in before)}/{len(before)}")

# the blocking belief: >=253 cells within 1.0 m of the STANDING pose, worst grid
def ring_cells(grid, center, radius=1.0):
    meta, data = grid["meta"], grid["data"]
    out = set()
    for cy in range(meta["h"]):
        for cx in range(meta["w"]):
            if data[cy * meta["w"] + cx] >= INSCRIBED:
                x = meta["ox"] + (cx + 0.5) * meta["res"]
                y = meta["oy"] + (cy + 0.5) * meta["res"]
                if math.hypot(x - center[0], y - center[1]) <= radius:
                    out.add((cx, cy))
    return out

worst = max(in_window, key=lambda g: len(ring_cells(g, STAND)))
cells = ring_cells(worst, STAND)
area = len(cells) * worst["meta"]["res"] ** 2
print(f"worst grid @{worst['t']:.0f}: {len(cells)} cells >=253 within 1.0 m of "
      f"the standing pose ({area:.2f} m^2)")
pre_cells = ring_cells(before[-1], STAND) if before else set()
print(f"same ring in last pre-window grid: {len(pre_cells)} cells")

# physical truth: how many live returns land in those cells across the window
def nearest(series, t):
    return min(series, key=lambda kv: abs(kv[0] - t))[1]

inside = total = 0
for t, scan in scans:
    laser = compose(compose(nearest(map_odom, t), nearest(odom_base, t)),
                    laser_static)
    angle = scan.angle_min
    for r in scan.ranges:
        if scan.range_min < r < scan.range_max:
            total += 1
            px = laser[0] + r * math.cos(laser[2] + angle)
            py = laser[1] + r * math.sin(laser[2] + angle)
            if to_grid(worst["meta"], px, py) in cells:
                inside += 1
        angle += scan.angle_increment
print(f"lidar truth: {inside}/{total} returns ({100.0*inside/max(1,total):.2f}%) "
      f"land in the belief-blocking cells near the standing pose")

# how long did belief hold the door shut vs physical evidence?
runs = []
state = None
for g in in_window:
    ok = connected(g, STUCK, STAND)
    if state is None or ok != state[0]:
        runs.append([ok, g["t"], g["t"]])
        state = runs[-1]
    else:
        state[2] = g["t"]
print("connectivity timeline (ok, from, to):")
for ok, t0, t1 in runs:
    print(f"  {'OPEN  ' if ok else 'CLOSED'} {t0 - WINDOW[0]:6.1f}s .. "
          f"{t1 - WINDOW[0]:6.1f}s  ({t1 - t0:5.1f}s)")
