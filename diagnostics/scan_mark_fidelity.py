"""Are the costmap's phantom marks backed by an actual lidar return? No motion.

A PHANTOM is a cell the global costmap calls lethal (>=100) while SLAM's /map calls
it free (0) -- the same predicate gap_blockage_probe.py uses, so the counts are
comparable. On 2026-08-09 there were 33-38 of them with the camera not running at
all, steady over four minutes, which is what moved the narrow-gap suspicion from the
camera to the lidar/costmap side.

This asks the next question: for each phantom, does a live /scan return actually land
there?

  * NO scan support -> the mark is not backed by anything the sensor currently sees.
    Stale or mis-transformed accumulation. Decisive; the fix is structural.
  * scan support present -> the mark is faithful FROM THIS POSE. That does NOT clear
    the costmap: marks laid down at one pose and wrong from another need motion to
    reproduce, and this test cannot see that.

Scan points are transformed by TF (map <- laser), never by a hardcoded rotation. Raw
/scan bearing 0 points BEHIND the rover (base_link->laser yaw 178.99 deg) even though
the unit is mounted facing forward -- the sensor's zero is not at the arrow. Baking
that number in would rot on any mount change, and ignoring it once produced a bogus
"lidar is 50 cm wrong" alarm that was really the wall behind the rover.

Scans are ACCUMULATED over several sweeps before comparing, because the costmap
accumulates too. Judging accumulated marks against one instantaneous sweep would
report phantoms that are only gaps between rays.

Usage: python3 scan_mark_fidelity.py [tolerance_m] [n_scans]    (default 0.05, 30)
"""

import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
import tf2_ros

TOL = float(sys.argv[1]) if len(sys.argv) > 1 else 0.05
N_SCANS = int(sys.argv[2]) if len(sys.argv) > 2 else 30


class Probe(Node):
    def __init__(self):
        super().__init__("scan_mark_fidelity")
        self.map = None
        self.costmap = None
        self.scans = []
        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = QoSReliabilityPolicy.RELIABLE
        self.create_subscription(OccupancyGrid, "/map", self._on_map, qos)
        self.create_subscription(
            OccupancyGrid, "/global_costmap/costmap", self._on_costmap, qos
        )
        self.create_subscription(LaserScan, "/scan", lambda m: self.scans.append(m), 10)
        self.tfb = tf2_ros.Buffer()
        self.tfl = tf2_ros.TransformListener(self.tfb, self)

    def _on_map(self, m):
        self.map = m

    def _on_costmap(self, m):
        self.costmap = m


def main():
    rclpy.init()
    n = Probe()
    t0 = time.monotonic()
    while rclpy.ok() and time.monotonic() - t0 < 30:
        rclpy.spin_once(n, timeout_sec=0.3)
        if n.map and n.costmap and len(n.scans) >= N_SCANS:
            break
    if not (n.map and n.costmap):
        print("missing /map or /global_costmap/costmap")
        return
    if not n.scans:
        print("no /scan")
        return

    # map <- laser, straight from TF. Never hardcode the yaw.
    tf = None
    for _ in range(25):
        try:
            tf = n.tfb.lookup_transform("map", "laser", rclpy.time.Time())
            break
        except Exception:
            rclpy.spin_once(n, timeout_sec=0.3)
    if tf is None:
        print("no map<-laser TF")
        return
    q = tf.transform.rotation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    tx, ty = tf.transform.translation.x, tf.transform.translation.y
    print(f"map<-laser: ({tx:+.3f},{ty:+.3f}) yaw {math.degrees(yaw):+.2f} deg "
          f"(from TF, not assumed)")

    # Accumulate returns in the map frame.
    pts = []
    for s in n.scans[-N_SCANS:]:
        a = s.angle_min
        for r in s.ranges:
            if math.isfinite(r) and s.range_min <= r <= s.range_max:
                x, y = r * math.cos(a), r * math.sin(a)
                pts.append((tx + x * math.cos(yaw) - y * math.sin(yaw),
                            ty + x * math.sin(yaw) + y * math.cos(yaw)))
            a += s.angle_increment
    print(f"accumulated {len(pts)} returns over {min(N_SCANS, len(n.scans))} sweeps")

    # Bucket returns so the nearest-point search is not O(cells * points).
    cell = max(TOL, 0.05)
    buckets = {}
    for px, py in pts:
        buckets.setdefault((int(px / cell), int(py / cell)), []).append((px, py))

    def nearest(wx, wy):
        bx, by = int(wx / cell), int(wy / cell)
        best = float("inf")
        reach = int(math.ceil(0.60 / cell))
        for rad in range(reach + 1):          # grow outward, stop once found
            for dx in range(-rad, rad + 1):
                for dy in range(-rad, rad + 1):
                    if max(abs(dx), abs(dy)) != rad:
                        continue
                    for px, py in buckets.get((bx + dx, by + dy), ()):
                        d = math.hypot(px - wx, py - wy)
                        if d < best:
                            best = d
            if best <= rad * cell:
                break
        return best

    cm, mp = n.costmap, n.map
    res = cm.info.resolution
    ox, oy = cm.info.origin.position.x, cm.info.origin.position.y
    mox, moy = mp.info.origin.position.x, mp.info.origin.position.y
    mw, mh = mp.info.width, mp.info.height

    supported, unsupported = [], []
    for i, v in enumerate(cm.data):
        if v < 100:                                    # lethal only, not inflation
            continue
        c, r = i % cm.info.width, i // cm.info.width
        wx, wy = ox + (c + 0.5) * res, oy + (r + 0.5) * res
        mc, mr = int((wx - mox) / res), int((wy - moy) / res)
        if not (0 <= mc < mw and 0 <= mr < mh) or mp.data[mr * mw + mc] != 0:
            continue                                   # not a phantom
        d = nearest(wx, wy)
        (supported if d <= TOL else unsupported).append((wx, wy, d))

    total = len(supported) + len(unsupported)
    print(f"\nPHANTOMS (lethal in costmap, FREE in /map): {total}")
    if not total:
        print("  none right now — nothing to judge")
        n.destroy_node(); rclpy.shutdown(); return
    print(f"  backed by a live /scan return (<={TOL:.2f} m): {len(supported)}")
    print(f"  NOT backed by any return within {TOL:.2f} m : {len(unsupported)}")

    # A scan-backed phantom is only interesting if it is NOT just the boundary shell.
    # The costmap and /map quantize the same walls on grids with different origins, so
    # the cell touching a wall can legitimately land occupied in one and free in the
    # other. Distance to the nearest /map-occupied cell separates "the two grids
    # disagree by one cell at an edge" from "the costmap blocks open floor".
    occ_pts = []
    for i, v in enumerate(mp.data):
        if v >= 50:
            occ_pts.append((mox + (i % mw + 0.5) * res, moy + (i // mw + 0.5) * res))
    obuckets = {}
    for px, py in occ_pts:
        obuckets.setdefault((int(px / cell), int(py / cell)), []).append((px, py))

    def nearest_occ(wx, wy):
        bx, by = int(wx / cell), int(wy / cell)
        best = float("inf")
        reach = int(math.ceil(1.0 / cell))
        for rad in range(reach + 1):
            for dx in range(-rad, rad + 1):
                for dy in range(-rad, rad + 1):
                    if max(abs(dx), abs(dy)) != rad:
                        continue
                    for px, py in obuckets.get((bx + dx, by + dy), ()):
                        d = math.hypot(px - wx, py - wy)
                        if d < best:
                            best = d
            if best <= rad * cell:
                break
        return best

    if supported:
        gaps = sorted(nearest_occ(wx, wy) for wx, wy, _ in supported)
        print(f"\n  of the scan-backed phantoms, distance to the nearest "
              f"/map-OCCUPIED cell:")
        print(f"    min {gaps[0]:.3f}  median {gaps[len(gaps)//2]:.3f}  "
              f"max {gaps[-1]:.3f} m")
        shell = sum(1 for d in gaps if d <= res * 1.5)
        print(f"    within 1.5 cells of a /map obstacle (boundary shell): "
              f"{shell}/{len(gaps)}")
        standalone = sum(1 for d in gaps if d > 0.25)
        print(f"    more than 0.25 m from ANY /map obstacle (open floor): {standalone}")
        if standalone:
            print("    ^ these are the ones that matter: real returns, open floor in /map")

    if unsupported:
        ds = sorted(d for _, _, d in unsupported)
        print(f"\n  distance from unsupported cells to the NEAREST return:")
        print(f"    min {ds[0]:.3f}  median {ds[len(ds)//2]:.3f}  max {ds[-1]:.3f} m")
        for band, lo, hi in (("0.05-0.10 m (discretization)", 0.05, 0.10),
                             ("0.10-0.25 m", 0.10, 0.25),
                             ("0.25-0.60 m", 0.25, 0.60),
                             (">0.60 m (nothing near)", 0.60, 1e9)):
            k = sum(1 for d in ds if lo <= d < hi)
            if k:
                print(f"    {band:32s} {k}")
        print("\n  worst offenders (map frame):")
        for wx, wy, d in sorted(unsupported, key=lambda t: -t[2])[:8]:
            print(f"    ({wx:+.2f},{wy:+.2f})  nearest return {d:.3f} m away")

    n.destroy_node()
    rclpy.shutdown()


main()
