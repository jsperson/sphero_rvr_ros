"""Measure phantom obstacle accumulation in the global costmap, with the rover still.

A PHANTOM is a cell the global costmap calls LETHAL while SLAM's /map calls it FREE.
SLAM and the costmap obstacle layer are fed by the same lidar, so a large, growing
phantom population means the obstacle layer is marking things that are not there --
which is exactly what would pinch a narrow gap.

Attribution without killing any node: the lidar sees 360 deg, the camera only a
forward wedge (measured FOV 38 deg left / 21 deg right, marks between camera_min and
max range). So phantoms are bucketed by bearing in the ROBOT frame:
  * IN-WEDGE  = inside the camera's field of view -> camera or lidar could have done it
  * OUT-WEDGE = behind/beside the robot, where the camera physically cannot see
                -> lidar-sourced, or a stale mark left from an earlier pose
If accumulation is overwhelmingly IN-WEDGE and tracks the live camera cloud, the
camera obstacle source is the writer. If it grows everywhere, it is not the camera.

Usage (on the Pi, rover STATIONARY, attended not required -- publishes no motion):
    python3 costmap_phantom_watch.py                 # clear, then watch 90 s
    python3 costmap_phantom_watch.py --no-clear --seconds 60
"""
import argparse
import math
import time

import rclpy
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import tf2_ros

CLEAR_RANGE = 1.8
CAM_LEFT_DEG = 38.0   # measured lens coverage, left of boresight
CAM_RIGHT_DEG = 21.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=90.0)
    ap.add_argument("--interval", type=float, default=10.0)
    ap.add_argument("--no-clear", action="store_true")
    args = ap.parse_args()

    rclpy.init()
    n = rclpy.create_node("costmap_phantom_watch")
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, n)
    qos = QoSProfile(depth=1)
    qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
    qos.reliability = QoSReliabilityPolicy.RELIABLE
    latest = {}
    n.create_subscription(OccupancyGrid, "/global_costmap/costmap", lambda m: latest.__setitem__("cm", m), qos)
    n.create_subscription(OccupancyGrid, "/map", lambda m: latest.__setitem__("map", m), qos)
    n.create_subscription(PointCloud2, "/camera/low_obstacles", lambda m: latest.__setitem__("cloud", m), 5)

    def spin(s):
        end = time.monotonic() + s
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(n, timeout_sec=0.05)

    spin(6.0)
    for k in ("cm", "map"):
        if k not in latest:
            raise SystemExit(f"missing {k}")

    def robot():
        t = buf.lookup_transform("map", "base_link", rclpy.time.Time()).transform
        q = t.rotation
        return (t.translation.x, t.translation.y,
                math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))

    def phantoms():
        """Cells lethal in the costmap but free in /map, bucketed by camera visibility."""
        cm, mp = latest["cm"], latest["map"]
        rx, ry, yaw = robot()
        res = cm.info.resolution
        ox, oy = cm.info.origin.position.x, cm.info.origin.position.y
        mox, moy, mw, mh = mp.info.origin.position.x, mp.info.origin.position.y, mp.info.width, mp.info.height
        inw = out = 0
        cells = []
        for i, v in enumerate(cm.data):
            if v < 100:
                continue
            c, r = i % cm.info.width, i // cm.info.width
            wx, wy = ox + (c + 0.5) * res, oy + (r + 0.5) * res
            mc, mr = int((wx - mox) / res), int((wy - moy) / res)
            if not (0 <= mc < mw and 0 <= mr < mh):
                continue
            if mp.data[mr * mw + mc] != 0:
                continue  # SLAM agrees it is occupied or unknown -> not a phantom
            dx, dy = wx - rx, wy - ry
            rng = math.hypot(dx, dy)
            bear = math.degrees(math.atan2(dy, dx) - yaw)
            bear = (bear + 180) % 360 - 180
            if -CAM_RIGHT_DEG <= bear <= CAM_LEFT_DEG and rng <= 2.0:
                inw += 1
            else:
                out += 1
            cells.append((round(rng, 2), round(bear)))
        return inw, out, cells

    def cam_marks():
        if "cloud" not in latest:
            return 0
        return sum(
            1 for p in pc2.read_points(latest["cloud"], field_names=("x", "y"), skip_nans=True)
            if abs(math.hypot(float(p[0]), float(p[1])) - CLEAR_RANGE) >= 0.01
        )

    total_free = sum(1 for v in latest["map"].data if v == 0)
    print(f"SLAM /map free cells: {total_free}")
    i0, o0, _ = phantoms()
    print(f"BEFORE clear: phantoms in-wedge {i0}, out-wedge {o0}  (= {100*(i0+o0)/max(total_free,1):.1f}% of free floor)")

    if not args.no_clear:
        cli = n.create_client(ClearEntireCostmap, "/global_costmap/clear_entirely_global_costmap")
        if not cli.wait_for_service(timeout_sec=10.0):
            raise SystemExit("clear service unavailable")
        fut = cli.call_async(ClearEntireCostmap.Request())
        while rclpy.ok() and not fut.done():
            rclpy.spin_once(n, timeout_sec=0.05)
        print("cleared global costmap")
        spin(2.0)

    print(f"\n{'t_s':>6} {'in_wedge':>9} {'out_wedge':>10} {'cam_marks':>10}  note")
    t0 = time.monotonic()
    while rclpy.ok() and time.monotonic() - t0 < args.seconds:
        spin(args.interval)
        inw, out, cells = phantoms()
        near = [c for c in cells if c[0] <= 2.0]
        print(f"{time.monotonic()-t0:6.1f} {inw:9d} {out:10d} {cam_marks():10d}  "
              f"nearest phantom {min((c[0] for c in near), default=float('nan')):.2f} m")

    inw, out, cells = phantoms()
    print(f"\nfinal: in-wedge {inw}, out-wedge {out}")
    if cells:
        band = {}
        for rng, bear in cells:
            band[int(rng / 0.25) * 0.25] = band.get(int(rng / 0.25) * 0.25, 0) + 1
        print("phantom range histogram (m -> cells):", dict(sorted(band.items())))
    print(
        "\nRead: growth concentrated IN-WEDGE at 0.5-1.5 m implicates the camera source. "
        "Growth spread all round (or behind) is lidar/SLAM disagreement, not the camera."
    )
    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
