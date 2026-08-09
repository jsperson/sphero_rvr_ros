"""Render the global costmap around the robot with live CAMERA marks overlaid.

Two jobs, both with the rover stationary:

  1. Baseline check before a gap A/B -- a costmap test that doesn't verify its own
     baseline proves nothing (learned 2026-08-06). Shows which cells are actually
     free and plannable, so the goal isn't picked blind.
  2. Direct read on the "camera costmap source pinches the gap" hypothesis. Camera
     points from /camera/low_obstacles are transformed base_link -> map and drawn as
     'C'. Clear-ray endpoints (range == clear_range_m) are excluded -- they are the
     opposite of a mark. If 'C' cells sit inside the gap, the camera layer is
     narrowing it; if the gap is clear of 'C', the costmap source is exonerated.

Usage (on the Pi):
    python3 gap_costmap_view.py               # 4 m window, costmap + camera
    python3 gap_costmap_view.py --span 6 --map # also render SLAM /map for contrast

Legend: R robot  # lethal  o inscribed/inflated  . free  (space) unknown
        C camera mark      X camera mark landing on an already-lethal cell
"""
import argparse
import math

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import tf2_ros

CLEAR_RANGE = 1.8  # low_obstacle clear_range_m -- endpoints, not obstacles


def render(grid, robot, cam_cells, span, title):
    info = grid.info
    res, w, h = info.resolution, info.width, info.height
    ox, oy = info.origin.position.x, info.origin.position.y
    rc = int((robot[0] - ox) / res)
    rr = int((robot[1] - oy) / res)
    half = int(span / 2 / res)
    c0, c1 = max(0, rc - half), min(w, rc + half + 1)
    r0, r1 = max(0, rr - half), min(h, rr + half + 1)

    print(f"\n=== {title} === {w}x{h} @ {res:.3f} m, window {(c1-c0)*res:.1f} x {(r1-r0)*res:.1f} m")
    print(f"    robot ({robot[0]:.2f}, {robot[1]:.2f}) cell ({rc},{rr}); north is up, +x right")
    for r in range(r1 - 1, r0 - 1, -1):  # +y up
        line = []
        for c in range(c0, c1):
            v = grid.data[r * w + c]
            if (c, r) == (rc, rr):
                line.append("R")
            elif (c, r) in cam_cells:
                line.append("X" if v >= 99 else "C")
            elif v < 0:
                line.append(" ")
            elif v >= 100:
                line.append("#")
            elif v >= 99:
                line.append("o")
            elif v > 0:
                line.append(":")
            else:
                line.append(".")
        print(f"  y={oy + r * res:+.2f} |" + "".join(line))
    xs = "".join("|" if (c - c0) % 10 == 0 else " " for c in range(c0, c1))
    print("          " + xs)
    print(f"          x from {ox + c0 * res:+.2f} to {ox + c1 * res:+.2f} (ticks every {10*res:.1f} m)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--span", type=float, default=4.0, help="window size in metres")
    ap.add_argument("--map", action="store_true", help="also render SLAM /map")
    args = ap.parse_args()

    rclpy.init()
    n = rclpy.create_node("gap_costmap_view")
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, n)
    qos = QoSProfile(depth=1)
    qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
    qos.reliability = QoSReliabilityPolicy.RELIABLE

    got = {}
    n.create_subscription(OccupancyGrid, "/global_costmap/costmap", lambda m: got.setdefault("cm", m), qos)
    n.create_subscription(OccupancyGrid, "/map", lambda m: got.setdefault("map", m), qos)
    n.create_subscription(PointCloud2, "/camera/low_obstacles", lambda m: got.__setitem__("cloud", m), 5)

    import time
    end = time.monotonic() + 15
    while rclpy.ok() and time.monotonic() < end:
        rclpy.spin_once(n, timeout_sec=0.1)
        if "cm" in got and "cloud" in got and buf.can_transform("map", "base_link", rclpy.time.Time()):
            break
    if "cm" not in got:
        raise SystemExit("no /global_costmap/costmap")
    if not buf.can_transform("map", "base_link", rclpy.time.Time()):
        raise SystemExit("no map->base_link TF")

    t = buf.lookup_transform("map", "base_link", rclpy.time.Time()).transform
    q = t.rotation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    robot = (t.translation.x, t.translation.y)
    print(f"robot yaw {math.degrees(yaw):+.0f} deg")

    cm = got["cm"]
    res = cm.info.resolution
    ox, oy = cm.info.origin.position.x, cm.info.origin.position.y
    cam_cells, marks, clears = set(), 0, 0
    if "cloud" in got:
        for p in pc2.read_points(got["cloud"], field_names=("x", "y"), skip_nans=True):
            bx, by = float(p[0]), float(p[1])
            if abs(math.hypot(bx, by) - CLEAR_RANGE) < 0.01:
                clears += 1
                continue
            marks += 1
            wx = robot[0] + bx * math.cos(yaw) - by * math.sin(yaw)
            wy = robot[1] + bx * math.sin(yaw) + by * math.cos(yaw)
            cam_cells.add((int((wx - ox) / res), int((wy - oy) / res)))
    print(f"camera cloud: {marks} marks -> {len(cam_cells)} cells, {clears} clear-ray endpoints")
    if marks:
        rs = sorted(
            math.hypot(float(p[0]), float(p[1]))
            for p in pc2.read_points(got["cloud"], field_names=("x", "y"), skip_nans=True)
            if abs(math.hypot(float(p[0]), float(p[1])) - CLEAR_RANGE) >= 0.01
        )
        print(f"  mark ranges: min {rs[0]:.2f}  median {rs[len(rs)//2]:.2f}  max {rs[-1]:.2f} m")

    render(cm, robot, cam_cells, args.span, "GLOBAL COSTMAP (what the planner sees)")
    if args.map and "map" in got:
        render(got["map"], robot, set(), args.span, "SLAM /map (lidar only, no camera, no inflation)")

    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
