"""Where does raw /scan bearing 0 point in base_link? Zero motion, lidar only.

Reads the deployed base_link->laser TF rather than assuming any rotation, so the
answer does not depend on anyone being right about how the unit is bolted on.

Put an object a measured distance straight ahead of the LIDAR PIVOT, bring up
lidar.launch.py alone, and run this. It reports the raw bearing that sees the
object, the same return expressed in base_link, and which way raw bearing 0 faces.

Result 2026-08-09 (object at 0.239 m): raw bearing 0 reads the far wall at 2.335 m;
raw 180 deg reads the object at 0.228 m. So raw /scan 0 points BACKWARD in base_link
-- while the unit itself is mounted FORWARD (arrow, nameplate and cord all face the
way you would expect). The sensor's own zero simply is not at the arrow, so the
housing tells you nothing about the scan's zero. An earlier note called this "the
lidar is mounted backwards", which asserted a mounting fact from data that only ever
showed a bearing offset.

Same run validates the TF end to end: it places the object at +0.232 m dead ahead
(bearing -2.2 deg) against the 0.239 m hand measurement.

Usage: python3 scan_bearing_check.py [distance_m]     (default 0.239)
"""
import math, statistics, sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import tf2_ros

TARGET = float(sys.argv[1]) if len(sys.argv) > 1 else 0.239
N = 12

class Check(Node):
    def __init__(self):
        super().__init__("scan_bearing_check")
        self.scans = []
        self.create_subscription(LaserScan, "/scan", lambda m: self.scans.append(m), 10)
        self.tfb = tf2_ros.Buffer(); self.tfl = tf2_ros.TransformListener(self.tfb, self)

def at(scan, bearing):
    """Range at a raw laser bearing (nearest bin), or nan."""
    i = int(round((bearing - scan.angle_min) / scan.angle_increment))
    if 0 <= i < len(scan.ranges):
        v = scan.ranges[i]
        return v if math.isfinite(v) and v > 0 else float("nan")
    return float("nan")

def main():
    rclpy.init(); n = Check()
    while rclpy.ok() and len(n.scans) < N:
        rclpy.spin_once(n, timeout_sec=0.5)
    # deployed TF
    yaw = None
    for _ in range(20):
        try:
            t = n.tfb.lookup_transform("base_link", "laser", rclpy.time.Time())
            q = t.transform.rotation
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            tx, ty = t.transform.translation.x, t.transform.translation.y
            break
        except Exception:
            rclpy.spin_once(n, timeout_sec=0.3)
    print(f"deployed base_link<-laser yaw = {yaw:.6f} rad = {math.degrees(yaw):.2f} deg")

    fwd, rear, mins = [], [], []
    for s in n.scans[-N:]:
        fwd.append(at(s, 0.0)); rear.append(at(s, math.pi))
        best = None
        for i, r in enumerate(s.ranges):
            if math.isfinite(r) and r > 0.02 and (best is None or r < best[1]):
                best = (s.angle_min + i*s.angle_increment, r)
        if best: mins.append(best)

    def med(v):
        v = [x for x in v if math.isfinite(x)]
        return statistics.median(v) if v else float("nan")
    print(f"raw /scan at bearing   0 deg : {med(fwd):.3f} m")
    print(f"raw /scan at bearing 180 deg : {med(rear):.3f} m")
    if mins:
        ang = statistics.median([a for a, _ in mins]); rng = statistics.median([r for _, r in mins])
        x, y = rng*math.cos(ang), rng*math.sin(ang)
        bx = tx + x*math.cos(yaw) - y*math.sin(yaw)
        by = ty + x*math.sin(yaw) + y*math.cos(yaw)
        print(f"\nnearest return: {rng:.3f} m at RAW bearing {math.degrees(ang):+.1f} deg")
        print(f"  -> in base_link: ({bx:+.3f}, {by:+.3f}) = bearing {math.degrees(math.atan2(by,bx)):+.1f} deg")
        print(f"\nobject is at {TARGET:.3f} m straight ahead.")
        d0, d180 = abs(med(fwd)-TARGET), abs(med(rear)-TARGET)
        print(f"  |raw0 - target|   = {d0:.3f} m")
        print(f"  |raw180 - target| = {d180:.3f} m")
        print(f"  => raw bearing 0 points {'FORWARD' if d0 < d180 else 'BACKWARD'} in base_link")
    n.destroy_node(); rclpy.shutdown()

main()
