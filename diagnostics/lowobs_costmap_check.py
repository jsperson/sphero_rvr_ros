"""Bench proof (NO chassis, NO camera, NO motion): does `/camera/low_obstacles`
actually mark into the global costmap, and does the planner route AROUND it?

A SYNTHETIC low-obstacle cloud is published at a known base_link position, which
isolates the costmap *wiring* (topic / frame / QoS / data_type / height gate /
raytrace clearing) and the planner's response from the monocular perception. If
this passes, a real-world failure is a perception problem, not an integration one.
It also runs in the dark, unlike the colour-threshold detector.

Three phases, so marking is proven by *change*, not by a single reading:
  A BASELINE  no cloud            -> cell should be free, path ~straight
  B MARKED    cloud published     -> cell should be lethal, path should detour
  C CLEARED   cloud stopped       -> cell should return toward free

Run the harness first (lidar -> SLAM -> global costmap, static odom TF):
  ros2 launch ~/ros2_ws/src/sphero_rvr_ros/diagnostics/frontier_diag.launch.py
Then:
  python3 ~/ros2_ws/src/sphero_rvr_ros/diagnostics/lowobs_costmap_check.py
Afterwards stop the lidar motor: ros2 service call /stop_motor std_srvs/srv/Empty
"""

import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformListener

COSTMAP_TOPIC = "/global_costmap/costmap"
CLOUD_TOPIC = "/camera/low_obstacles"
MAP_FRAME = "map"
BASE_FRAME = "base_link"
# Candidate goals ahead of the robot (metres forward, metres left) -- the first one
# the planner can reach on a clear costmap is used; the obstacle then goes halfway.
GOAL_CANDIDATES = [(1.20, 0.0), (1.00, 0.0), (1.40, 0.0), (1.00, 0.40), (1.00, -0.40)]


class Checker(Node):
    def __init__(self):
        super().__init__("lowobs_costmap_check")
        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = QoSReliabilityPolicy.RELIABLE
        self._grid = None
        self.create_subscription(OccupancyGrid, COSTMAP_TOPIC, self._on_grid, qos)
        self._cloud_pub = self.create_publisher(PointCloud2, CLOUD_TOPIC, 5)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._planner = ActionClient(self, ComputePathToPose, "compute_path_to_pose")
        self._publish_cloud = False
        self._obstacle_bl = None  # (forward, left) in base_link
        self.create_timer(0.2, self._tick_cloud)

    # --- plumbing ---------------------------------------------------------
    def _on_grid(self, msg):
        self._grid = msg

    def _tick_cloud(self):
        if not self._publish_cloud or self._obstacle_bl is None:
            return
        fwd, left = self._obstacle_bl
        pts = []
        for dl in [i * 0.03 - 0.12 for i in range(9)]:      # ~0.25 m wide
            for df in [0.0, 0.04, 0.08]:                     # a little depth
                pts.append((fwd + df, left + dl, 0.0))
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = BASE_FRAME
        self._cloud_pub.publish(pc2.create_cloud_xyz32(header, pts))

    def spin(self, seconds):
        end = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_for_grid(self, timeout=40.0):
        end = time.monotonic() + timeout
        while rclpy.ok() and self._grid is None and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.2)
        return self._grid is not None

    def robot_pose(self):
        """(x, y, yaw) of base_link in the map frame."""
        end = time.monotonic() + 15.0
        while rclpy.ok() and time.monotonic() < end:
            try:
                tf = self._tf_buffer.lookup_transform(MAP_FRAME, BASE_FRAME, rclpy.time.Time())
                q = tf.transform.rotation
                yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
                return tf.transform.translation.x, tf.transform.translation.y, yaw
            except Exception:
                rclpy.spin_once(self, timeout_sec=0.2)
        return None

    # --- costmap sampling --------------------------------------------------
    def cell_at(self, mx, my):
        """Costmap value at a map-frame point, or None if out of bounds."""
        g = self._grid
        if g is None:
            return None
        res = g.info.resolution
        cx = int((mx - g.info.origin.position.x) / res)
        cy = int((my - g.info.origin.position.y) / res)
        if not (0 <= cx < g.info.width and 0 <= cy < g.info.height):
            return None
        return g.data[cy * g.info.width + cx]

    def worst_cell_near(self, mx, my, radius_m=0.12):
        """Highest (worst) costmap value within radius -- tolerates small offsets."""
        g = self._grid
        if g is None:
            return None
        res = g.info.resolution
        rad = max(1, int(radius_m / res))
        cx0 = int((mx - g.info.origin.position.x) / res)
        cy0 = int((my - g.info.origin.position.y) / res)
        worst = None
        for dy in range(-rad, rad + 1):
            for dx in range(-rad, rad + 1):
                cx, cy = cx0 + dx, cy0 + dy
                if 0 <= cx < g.info.width and 0 <= cy < g.info.height:
                    v = g.data[cy * g.info.width + cx]
                    if worst is None or v > worst:
                        worst = v
        return worst

    def wait_for_fresh_grid(self, settle_s=3.0, timeout_s=8.0):
        """Let the change settle, then take the NEXT published grid (~1 Hz)."""
        self.spin(settle_s)
        self._grid = None
        end = time.monotonic() + timeout_s
        while rclpy.ok() and self._grid is None and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self._grid is not None

    # --- planning ----------------------------------------------------------
    def plan(self, start, goal):
        """Return the path poses from ComputePathToPose, or None."""
        if not self._planner.wait_for_server(timeout_sec=10.0):
            print("  !! compute_path_to_pose action server unavailable")
            return None
        g = ComputePathToPose.Goal()
        g.use_start = True
        g.planner_id = "GridBased"
        for field, (x, y) in (("start", start), ("goal", goal)):
            ps = PoseStamped()
            ps.header.frame_id = MAP_FRAME
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.orientation.w = 1.0
            setattr(g, field, ps)
        fut = self._planner.send_goal_async(g)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=15.0)
        handle = fut.result()
        if handle is None or not handle.accepted:
            return None
        rf = handle.get_result_async()
        rclpy.spin_until_future_complete(self, rf, timeout_sec=20.0)
        res = rf.result()
        if res is None or not res.result.path.poses:
            return None
        return [(p.pose.position.x, p.pose.position.y) for p in res.result.path.poses]


def path_stats(path, start, goal):
    """(length, max lateral deviation from the straight start->goal line)."""
    if not path:
        return None, None
    length = sum(
        math.dist(path[i], path[i + 1]) for i in range(len(path) - 1)
    )
    sx, sy = start
    gx, gy = goal
    dx, dy = gx - sx, gy - sy
    seg = math.hypot(dx, dy)
    if seg < 1e-6:
        return length, 0.0
    worst = 0.0
    for px, py in path:
        # perpendicular distance from the straight line
        dev = abs(dy * (px - sx) - dx * (py - sy)) / seg
        worst = max(worst, dev)
    return length, worst


def describe(v):
    if v is None:
        return "out-of-bounds"
    if v < 0:
        return f"{v} (unknown)"
    if v >= 100:
        return f"{v} (LETHAL)"
    if v == 0:
        return f"{v} (free)"
    return f"{v} (inflation)"


def main():
    rclpy.init()
    n = Checker()
    print("waiting for global costmap + TF ...")
    if not n.wait_for_grid():
        print(f"NO COSTMAP on {COSTMAP_TOPIC} -- is frontier_diag.launch.py running?")
        return
    pose = n.robot_pose()
    if pose is None:
        print("NO map->base_link TF -- is SLAM up?")
        return
    rx, ry, ryaw = pose
    print(f"robot at map ({rx:.2f}, {ry:.2f}) yaw {math.degrees(ryaw):.0f} deg")

    def to_map(fwd, left):
        return (
            rx + fwd * math.cos(ryaw) - left * math.sin(ryaw),
            ry + fwd * math.sin(ryaw) + left * math.cos(ryaw),
        )

    # ---- Phase A: baseline, no cloud ------------------------------------
    # Find a heading whose corridor is genuinely FREE at baseline. Testing into
    # already-occupied space produces a meaningless "lethal" reading (the first
    # run of this script did exactly that and looked like a false PASS).
    print("\n=== PHASE A: BASELINE (no camera cloud) ===")
    chosen = None
    for rel_deg in [0, 30, -30, 60, -60, 90, -90, 120, -120, 150, -150, 180]:
        a = math.radians(rel_deg)
        dist = 1.20
        ray_free = True
        for s in [i * 0.10 for i in range(1, int(dist / 0.10) + 1)]:
            v = n.worst_cell_near(*to_map(s * math.cos(a), s * math.sin(a)), radius_m=0.10)
            if v is None or v != 0:
                ray_free = False
                break
        if not ray_free:
            print(f"  heading {rel_deg:+4d} deg: corridor not clear, skipping")
            continue
        goal = to_map(dist * math.cos(a), dist * math.sin(a))
        p = n.plan((rx, ry), goal)
        if not p:
            print(f"  heading {rel_deg:+4d} deg: corridor clear but NO PATH, skipping")
            continue
        ln, dev = path_stats(p, (rx, ry), goal)
        print(f"  heading {rel_deg:+4d} deg: CLEAR + path OK "
              f"({len(p)} poses, len {ln:.2f} m, max deviation {dev:.3f} m)")
        chosen = (a, dist, goal, ln, dev)
        break
    if chosen is None:
        print("  !! no clear corridor in any direction; can't run the route-around test.")
        print("     (The bench is boxed in. Re-run with open floor around the rover.)")
        return
    a, dist, goal, base_len, base_dev = chosen
    obst_bl = ((dist / 2.0) * math.cos(a), (dist / 2.0) * math.sin(a))
    obst_map = to_map(*obst_bl)
    base_cell = n.worst_cell_near(*obst_map)
    print(f"  obstacle will go at base_link fwd={obst_bl[0]:.2f} left={obst_bl[1]:+.2f} "
          f"-> map ({obst_map[0]:.2f}, {obst_map[1]:.2f})")
    print(f"  baseline cell there: {describe(base_cell)}")
    if base_cell != 0:
        print("  !! baseline cell is not free -- aborting; the test would be meaningless.")
        return

    # ---- Phase B: publish the synthetic cloud ---------------------------
    print("\n=== PHASE B: MARKED (synthetic /camera/low_obstacles published) ===")
    n._obstacle_bl = obst_bl
    n._publish_cloud = True
    # Sample repeatedly: the lidar raytraces THROUGH a sub-plane obstacle, so it can
    # clear what the camera marks. A mark that flickers (or never lands) is the real
    # failure mode here, and a single reading would hide it.
    samples = []
    for _ in range(6):
        n.wait_for_fresh_grid(settle_s=1.5)
        samples.append(n.worst_cell_near(*obst_map))
    print(f"  cell samples while publishing: {[describe(s) for s in samples]}")
    lethal_count = sum(1 for s in samples if s is not None and s >= 100)
    print(f"  lethal in {lethal_count}/{len(samples)} samples")
    marked = max((s for s in samples if s is not None), default=None)
    p2 = n.plan((rx, ry), goal)
    if p2:
        ln2, dev2 = path_stats(p2, (rx, ry), goal)
        print(f"  path with obstacle: {len(p2)} poses, len {ln2:.2f} m, max deviation {dev2:.3f} m")
    else:
        ln2 = dev2 = None
        print("  path with obstacle: NO PATH (planner refused -- obstacle blocks the only route)")

    # ---- Phase C: stop publishing, expect clearing ----------------------
    print("\n=== PHASE C: CLEARED (cloud stopped) ===")
    n._publish_cloud = False
    n.spin(10.0)
    n.wait_for_fresh_grid()
    cleared = n.worst_cell_near(*obst_map)
    print(f"  cell at obstacle: {describe(cleared)}")

    # ---- Verdict ---------------------------------------------------------
    print("\n=== VERDICT ===")
    marked_ok = lethal_count >= len(samples) - 1        # allow one transient miss
    flaky = 0 < lethal_count < len(samples) - 1
    verdict = "PASS" if marked_ok else ("FLAKY" if flaky else "FAIL")
    print(f"  [{verdict}] camera cloud MARKS the global costmap and the mark PERSISTS "
          f"(lethal in {lethal_count}/{len(samples)} samples; baseline was free)")
    if flaky:
        print("        -> the mark appears then disappears: the lidar is raytracing")
        print("           through the (sub-plane) obstacle and clearing the camera's marks.")
    if ln2 is None:
        print("  [PASS] planner REACTS: it refused the straight route once marked")
    elif dev2 is not None and base_dev is not None:
        detoured = (dev2 - base_dev) > 0.10 or (ln2 - base_len) > 0.10
        print(f"  [{'PASS' if detoured else 'CHECK'}] planner ROUTES AROUND: deviation "
              f"{base_dev:.3f} -> {dev2:.3f} m, length {base_len:.2f} -> {ln2:.2f} m")
    cleared_ok = cleared is not None and (marked is None or cleared < 100)
    print(f"  [{'PASS' if cleared_ok else 'CHECK'}] marks CLEAR when the obstacle goes away "
          f"({describe(cleared)})")

    n.destroy_node()
    rclpy.shutdown()


main()
