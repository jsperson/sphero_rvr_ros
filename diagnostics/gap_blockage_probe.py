"""Ask WHY the planner can't cross the gap, and who put the blockage there. No motion.

Three questions, answered against the live stack with the rover parked:

  1. Can the planner reach a goal on the far side of the gap? (ComputePathToPose --
     the planner alone, no controller, no motors, so this is a free, repeatable
     substitute for a drive.)
  2. Which cells block it? A BLOCKER is lethal in the global costmap while SLAM's
     /map calls the same cell FREE -- the costmap inventing an obstacle.
  3. Who marked them? Live /camera/low_obstacles points are transformed into the map
     frame; a blocker with a camera point on it is camera-attributed. Blockers with
     no camera point, especially outside the camera's forward wedge, are lidar's.

Usage (on the Pi):
    python3 gap_blockage_probe.py --goal 1.0 -1.6
    python3 gap_blockage_probe.py --goal 1.0 -1.6 --samples 20   # accumulate cloud
"""
import argparse
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import tf2_ros

CLEAR_RANGE = 1.8
CAM_LEFT_DEG, CAM_RIGHT_DEG = 38.0, 21.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--goal", nargs=2, type=float, metavar=("X", "Y"),
                    help="goal in the map frame")
    ap.add_argument("--forward", type=float,
                    help="goal this many metres ahead of the robot (survives a SLAM restart, "
                         "so the same command means the same physical target in both arms)")
    ap.add_argument("--left", type=float, default=0.0)
    ap.add_argument("--samples", type=int, default=15, help="camera clouds to accumulate")
    args = ap.parse_args()

    rclpy.init()
    n = rclpy.create_node("gap_blockage_probe")
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, n)
    qos = QoSProfile(depth=1)
    qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
    qos.reliability = QoSReliabilityPolicy.RELIABLE
    latest = {}
    clouds = []
    n.create_subscription(OccupancyGrid, "/global_costmap/costmap", lambda m: latest.__setitem__("cm", m), qos)
    n.create_subscription(OccupancyGrid, "/map", lambda m: latest.__setitem__("map", m), qos)
    n.create_subscription(PointCloud2, "/camera/low_obstacles", lambda m: clouds.append(m), 5)

    def spin(s):
        end = time.monotonic() + s
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(n, timeout_sec=0.05)

    spin(8.0)
    if "cm" not in latest or "map" not in latest:
        raise SystemExit("missing costmap or /map")
    t = buf.lookup_transform("map", "base_link", rclpy.time.Time()).transform
    q = t.rotation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    rx, ry = t.translation.x, t.translation.y
    if args.forward is not None:
        args.goal = [rx + args.forward * math.cos(yaw) - args.left * math.sin(yaw),
                     ry + args.forward * math.sin(yaw) + args.left * math.cos(yaw)]
    elif not args.goal:
        raise SystemExit("give --goal X Y or --forward M")
    print(f"robot ({rx:.2f}, {ry:.2f}) yaw {math.degrees(yaw):+.0f} deg -> "
          f"goal ({args.goal[0]:.2f}, {args.goal[1]:.2f})")

    cm, mp = latest["cm"], latest["map"]
    res = cm.info.resolution
    ox, oy = cm.info.origin.position.x, cm.info.origin.position.y

    # Camera mark cells, accumulated over several frames (one frame is sparse).
    cam_cells = set()
    for msg in clouds[-args.samples:]:
        for p in pc2.read_points(msg, field_names=("x", "y"), skip_nans=True):
            bx, by = float(p[0]), float(p[1])
            if abs(math.hypot(bx, by) - CLEAR_RANGE) < 0.01:
                continue
            wx = rx + bx * math.cos(yaw) - by * math.sin(yaw)
            wy = ry + bx * math.sin(yaw) + by * math.cos(yaw)
            cam_cells.add((int((wx - ox) / res), int((wy - oy) / res)))
    print(f"camera marks over {len(clouds[-args.samples:])} frames -> {len(cam_cells)} distinct cells")

    # Blockers: lethal in costmap, free in /map.
    mox, moy, mw, mh = mp.info.origin.position.x, mp.info.origin.position.y, mp.info.width, mp.info.height
    cam_blockers, lidar_blockers = [], []
    for i, v in enumerate(cm.data):
        if v < 100:
            continue
        c, r = i % cm.info.width, i // cm.info.width
        wx, wy = ox + (c + 0.5) * res, oy + (r + 0.5) * res
        mc, mr = int((wx - mox) / res), int((wy - moy) / res)
        if not (0 <= mc < mw and 0 <= mr < mh) or mp.data[mr * mw + mc] != 0:
            continue
        bear = math.degrees(math.atan2(wy - ry, wx - rx) - yaw)
        bear = (bear + 180) % 360 - 180
        rng = math.hypot(wx - rx, wy - ry)
        hit = any((c + dc, r + dr) in cam_cells for dc in (-1, 0, 1) for dr in (-1, 0, 1))
        (cam_blockers if hit else lidar_blockers).append((round(wx, 2), round(wy, 2), round(rng, 2), round(bear)))

    print(f"\nPHANTOM BLOCKERS (lethal in costmap, FREE in SLAM /map): {len(cam_blockers)+len(lidar_blockers)}")
    print(f"  camera-attributed (a live camera mark on/next to the cell): {len(cam_blockers)}")
    print(f"  not camera-attributed                                    : {len(lidar_blockers)}")
    inwedge = [b for b in lidar_blockers if -CAM_RIGHT_DEG <= b[3] <= CAM_LEFT_DEG]
    print(f"    of those, outside the camera's FOV entirely (must be lidar): {len(lidar_blockers)-len(inwedge)}")
    for name, lst in (("camera", cam_blockers), ("other", lidar_blockers)):
        if lst:
            lst.sort(key=lambda b: b[2])
            print(f"  nearest {name} blockers: " + ", ".join(f"({b[0]},{b[1]})@{b[2]}m/{b[3]:+d}deg" for b in lst[:6]))

    # Does the planner get through?
    ac = ActionClient(n, ComputePathToPose, "compute_path_to_pose")
    if not ac.wait_for_server(timeout_sec=10.0):
        raise SystemExit("compute_path_to_pose unavailable")
    g = ComputePathToPose.Goal()
    g.goal = PoseStamped()
    g.goal.header.frame_id = "map"
    g.goal.pose.position.x, g.goal.pose.position.y = args.goal
    g.goal.pose.orientation.w = 1.0
    g.use_start = False
    fut = ac.send_goal_async(g)
    while rclpy.ok() and not fut.done():
        rclpy.spin_once(n, timeout_sec=0.05)
    handle = fut.result()
    rf = handle.get_result_async()
    while rclpy.ok() and not rf.done():
        rclpy.spin_once(n, timeout_sec=0.05)
    res_msg = rf.result()
    poses = res_msg.result.path.poses if res_msg.status == 4 else []
    print(f"\nPLANNER to {tuple(args.goal)}: {'PATH FOUND' if poses else 'NO PATH'} ({len(poses)} poses)")

    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
