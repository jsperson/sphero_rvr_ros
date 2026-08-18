"""(A)-CLOSED-LOOP: does a contact mark stop the stack from driving into it?

The open-loop sweep was adversarial -- TF forced at the mark with no controller in the
loop. This is the fair test: the whole stock middle, a simulated chassis, a goal beyond
the mark, same heading. A/B in ONE rig: run --mark none first and prove the robot
actually drives the route, then run --mark strip and see whether anything changes. A rig
that cannot produce the drive cannot show that a mark prevented it.

PRE-REGISTERED:
  RE-CONTACT  : base_link reaches mark_x - FOOTPRINT_FRONT_M (the bumper touches the
                mark plane). In the real world that is the chair leg again.
  PROTECTED   : the robot stops short of that, or diverges laterally around it.
  INCONCLUSIVE: the control run does not drive either -- rig problem, not a finding.
"""
import argparse, math, struct, sys, threading, time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import PointCloud2, PointField
from nav2_msgs.action import NavigateToPose
from tf2_ros import Buffer, TransformListener

FOOTPRINT_FRONT_M = 0.0965
HALF_L, HALF_R = 0.098, 0.106
Z = 0.15
MQ = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.VOLATILE,
                history=QoSHistoryPolicy.KEEP_LAST, depth=10)


class Runner(Node):
    def __init__(self, mark_x, mode):
        super().__init__("closed_loop_mark_test")
        self.mark_x, self.mode = mark_x, mode
        self.track = []
        self.buf = Buffer(); TransformListener(self.buf, self)
        self.pub = self.create_publisher(PointCloud2, "/contact_marks", MQ)
        self.create_timer(0.5, self.pub_marks)
        self.create_timer(0.1, self.sample)

    def strip(self):
        if self.mode == "none":
            return []
        ys = [y / 1000.0 for y in range(int(-HALF_R * 1000), int(HALF_L * 1000) + 1, 25)]
        return [(self.mark_x, y) for y in ys]

    def pub_marks(self):
        pts = self.strip()
        m = PointCloud2(); m.header.stamp = self.get_clock().now().to_msg(); m.header.frame_id = "map"
        m.height = 1; m.width = len(pts)
        m.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                    for i, n in enumerate(("x", "y", "z"))]
        m.is_bigendian = False; m.point_step = 12; m.row_step = 12 * len(pts); m.is_dense = True
        m.data = b"".join(struct.pack("<fff", px, py, Z) for px, py in pts)
        self.pub.publish(m)

    def sample(self):
        try:
            t = self.buf.lookup_transform("map", "base_link", rclpy.time.Time())
        except Exception:
            return
        self.track.append((time.time(), t.transform.translation.x, t.transform.translation.y))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mark", choices=["none", "strip"], default="strip")
    ap.add_argument("--mark-x", type=float, default=0.60)
    ap.add_argument("--goal-x", type=float, default=1.10)
    ap.add_argument("--timeout", type=float, default=90.0)
    a = ap.parse_args()

    rclpy.init()
    n = Runner(a.mark_x, a.mark)
    ex = rclpy.executors.MultiThreadedExecutor(); ex.add_node(n)
    threading.Thread(target=ex.spin, daemon=True).start()

    print(f"mode={a.mark}  mark_x={a.mark_x}  goal_x={a.goal_x}")
    time.sleep(6)  # let the mark settle into the costmap before the goal is sent
    if not n.track:
        print("NO TF map->base_link -- rig not up. INCONCLUSIVE."); return 2
    print(f"start pose x={n.track[-1][1]:.3f} y={n.track[-1][2]:.3f}")

    ac = ActionClient(n, NavigateToPose, "navigate_to_pose")
    if not ac.wait_for_server(timeout_sec=20.0):
        print("no navigate_to_pose server. INCONCLUSIVE."); return 2
    g = NavigateToPose.Goal()
    g.pose.header.frame_id = "map"
    g.pose.header.stamp = n.get_clock().now().to_msg()
    g.pose.pose.position.x = a.goal_x
    g.pose.pose.orientation.w = 1.0
    fut = ac.send_goal_async(g)
    t0 = time.time()
    while not fut.done() and time.time() - t0 < 15:
        time.sleep(0.2)
    gh = fut.result()
    if gh is None or not gh.accepted:
        print("goal REJECTED. INCONCLUSIVE."); return 2
    res = gh.get_result_async()
    while not res.done() and time.time() - t0 < a.timeout:
        time.sleep(0.5)
    status = res.result().status if res.done() else "TIMEOUT"

    xs = [x for _, x, _ in n.track]
    ys = [y for _, _, y in n.track]
    maxx, maxy = max(xs), max(abs(v) for v in ys)
    bumper = maxx + FOOTPRINT_FRONT_M
    print(f"\nresult status      : {status}")
    print(f"start x            : {xs[0]:.3f}")
    print(f"max x reached      : {maxx:.3f}")
    print(f"max |y| deviation  : {maxy:.3f}")
    print(f"bumper reached     : {bumper:.3f}   (mark plane at {a.mark_x:.3f})")
    print(f"distance travelled : {maxx - xs[0]:.3f} m")
    if a.mark == "none":
        print("\nCONTROL RUN: the number that matters is 'distance travelled'. If the "
              "robot did not drive past the mark plane here, the marked run proves nothing.")
    else:
        if bumper >= a.mark_x:
            print(f"\nRE-CONTACT: the bumper passed the mark plane by "
                  f"{bumper - a.mark_x:.3f} m. In the room that is the leg again.")
        else:
            print(f"\nPROTECTED: the bumper stopped {a.mark_x - bumper:.3f} m short.")
    ex.shutdown(); rclpy.try_shutdown()
    return 0


sys.exit(main())
