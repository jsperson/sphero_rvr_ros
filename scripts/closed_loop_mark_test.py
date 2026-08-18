"""(A)-CLOSED-LOOP: does a contact mark stop the stack from driving into it?

The open-loop sweep was adversarial -- TF forced at the mark with no controller in the
loop. This is the fair test: the whole stock middle, a simulated chassis, a goal beyond
the mark, same heading. A/B in ONE rig: run --mark none first and prove the robot
actually drives the route, then run --mark strip and see whether anything changes. A rig
that cannot produce the drive cannot show that a mark prevented it.

PRE-REGISTERED:
  RE-CONTACT  : the body centre comes within BODY_RADIUS_M of any marked point, anywhere
                in the trajectory. In the real world that is the chair leg again.
  PROTECTED   : it never does.
  INCONCLUSIVE: the control run does not drive either -- rig problem, not a finding.

CONTACT IS 2-D, AND THE FIRST VERSION OF THIS TEST GOT THAT WRONG. It compared the
bumper's X against the mark PLANE, which is the right question only while the robot
drives straight at the mark. The moment a detour is possible, crossing that plane 0.23 m
off-axis -- beside a mark spanning 0.204 m -- is exactly what SUCCESS looks like, and the
1-D test called it RE-CONTACT and reported a failure that had not happened. The criterion
outlived its assumption: it was written when the only possible outcomes were "stops
short" and "doesn't", and it survived into a world with a third one.

THE FOUR-CLAUSE BAR for the global-costmap change, all machine-read here:
  1. FLIP          -- the marked arm goes from ABORTED to SUCCEEDED (open map).
  2. CONTROL       -- the no-mark arm still succeeds and still drives the route. This is
                      what catches `scan_layer` breaking ordinary planning, which is the
                      larger behavioural change in that batch.
  3. REAL DETOUR   -- pass --min-detour-y with the CONTROL arm's lateral deviation. A
                      marked arm that succeeds while driving straight has not routed
                      around anything; it means the mark stopped working, and without
                      this clause that reads as victory. An anti-vacuous-green: the same
                      shape as the lidar control beside the camera charter tests.
  4. HONEST REFUSAL -- in a corridor with NO detour (`--expect-no-path`, run against the
                      recorded room), the goal must FAIL and the body must never come
                      within STANDOFF_M of a marked point. Before the global costmap the
                      same obstacle in the same map produced nine RPP refusals and a
                      bumper 0.151 m from the mark; after, the planner refuses on its
                      first call and the rover never approaches at all.

                      THIS CLAUSE WAS FIRST WRITTEN AS "the wheels never turn", AND THAT
                      DESCRIBED SOMETHING THAT NEVER HAPPENED. The first corridor run
                      reported max x 0.182 in the same message that called it zero
                      motion: the PLANNER refuses before any motion, then the BT runs
                      recoveries, and recoveries move the rover. Neither the author nor
                      the reviewer reconciled the sentence with the number sitting beside
                      it. What is actually true -- and is the stronger property -- is
                      that the rover never goes near the thing it is refusing.

THE PRECONDITION, AND WHY IT IS A DISTINCT VERDICT. Clause 1 needs a detour to EXIST.
Run on `mission_20260816_171934` it could not be satisfied at any mark position -- the
free span containing the robot's path is 0.40-0.60 m at the mark site and 1.05 m at its
widest on the whole route, against ~0.51 m sterilised by one mark. A bar whose
precondition is false must not report FAILED, because that reads as "the robot did the
wrong thing" when the truth is "the question could not be asked here". So the corridor
is MEASURED from the global costmap before either arm runs, with the marks not yet
published, and a site too narrow yields INCONCLUSIVE (exit 3) -- never a pass, never a
failure. Generate the open map with `make_open_rig_map.py`.
"""
import argparse, math, struct, sys, threading, time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import PointCloud2, PointField
from nav2_msgs.msg import Costmap
from nav2_msgs.action import NavigateToPose
from tf2_ros import Buffer, TransformListener

FOOTPRINT_FRONT_M = 0.0965
HALF_L, HALF_R = 0.098, 0.106
Z = 0.15
INSCRIBED = 253

#: Circumscribed body radius from Scott's tape: hypot(max(front, rear), max(left, right))
#: = hypot(0.0995, 0.106). The BODY, not the costmap's padded footprint -- contact is a
#: fact about the robot, and the costmap's 0.1591 is what nav2 thinks about the robot.
BODY_RADIUS_M = 0.1454

#: Clause 4's bar, DERIVED rather than rounded. A contact mark stops the planner when the
#: robot's centre would come within the costmap's inscribed radius (0.1519, M1) of it, so
#: any refusal that works at all keeps the CENTRE at least that far off. Requiring the
#: body to stay a further BODY_RADIUS_M back -- 0.1519 + 0.1454 = 0.2973 -- is the
#: statement "not merely refused, but never even brought alongside": no part of the robot
#: reaches the region the mark forbids. Rounded DOWN to 0.29 so the bar is the derivation
#: rather than floating-point luck.
#:
#: For scale, the measured before/after: pre-global the rover drove until its bumper was
#: 0.151 m from the mark; post-global its closest approach in the corridor arm is ~0.42 m.
STANDOFF_M = 0.29
MQ = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.VOLATILE,
                history=QoSHistoryPolicy.KEEP_LAST, depth=10)
TL = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                history=QoSHistoryPolicy.KEEP_LAST, depth=1)


class Runner(Node):
    def __init__(self, mark_x, mode):
        super().__init__("closed_loop_mark_test")
        self.mark_x, self.mode = mark_x, mode
        self.track = []
        self.buf = Buffer(); TransformListener(self.buf, self)
        self.global_map = None
        # Marks stay OFF the wire until the precondition has been measured: the corridor
        # question is "is there room here", and a mark publishing into the costmap while
        # we measure would sterilise the very span being measured.
        self.publishing = False
        self.pub = self.create_publisher(PointCloud2, "/contact_marks", MQ)
        self.create_subscription(Costmap, "/global_costmap/costmap_raw",
                                 lambda m: setattr(self, "global_map", m), TL)
        self.create_timer(0.5, self.pub_marks)
        self.create_timer(0.1, self.sample)

    def free_span_at(self, x, y0=0.0):
        """The free y-interval CONTAINING y0 -- not the widest one in the room.

        THE TRAP THIS AVOIDS, paid for on 2026-08-18: the first corridor probe reported a
        2.05 m free span at the mark site and nearly certified the room as open. That span
        was elsewhere, behind a wall, not connected to the robot's corridor. The number
        that matters is the one the robot can actually reach.
        """
        g = self.global_map
        if g is None:
            return None
        md = g.metadata

        def cost(px, py):
            i = int(math.floor((px - md.origin.position.x) / md.resolution))
            j = int(math.floor((py - md.origin.position.y) / md.resolution))
            if not (0 <= i < md.size_x and 0 <= j < md.size_y):
                return None
            return int(g.data[j * md.size_x + i])

        c = cost(x, y0)
        if c is None or c >= INSCRIBED:
            return 0.0
        lo = hi = y0
        while (v := cost(x, lo - md.resolution)) is not None and v < INSCRIBED:
            lo -= md.resolution
        while (v := cost(x, hi + md.resolution)) is not None and v < INSCRIBED:
            hi += md.resolution
        return hi - lo

    def strip(self):
        if self.mode == "none" or not self.publishing:
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
    ap.add_argument("--min-detour-y", type=float, default=None,
                    help="marked arm must deviate laterally by MORE than this (pass the "
                         "control arm's max |y|). Without it, a SUCCEEDED marked arm "
                         "cannot be distinguished from a mark that stopped working.")
    ap.add_argument("--precondition-span", type=float, default=1.2,
                    help="required free y-span at the mark site, measured BEFORE marks "
                         "are published. Below this a detour does not exist and the run "
                         "is INCONCLUSIVE (exit 3), never FAILED.")
    ap.add_argument("--expect-no-path", action="store_true",
                    help="clause 4: assert the planner refuses from the start pose with "
                         "the wheels never turning. Skips the precondition, since a "
                         "corridor with no detour is the point of this arm.")
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

    # --- the precondition, measured before a single mark is published ------------------
    span = n.free_span_at(a.mark_x)
    if span is None:
        print("NO GLOBAL COSTMAP -- cannot measure the corridor. INCONCLUSIVE.")
        return 3
    print(f"corridor at mark_x : free y-span containing the path = {span:.2f} m "
          f"(precondition >= {a.precondition_span:.2f})")
    if a.expect_no_path:
        print("  clause 4 arm: a corridor with NO detour is the point here, so the "
              "precondition is deliberately not enforced.")
    elif span < a.precondition_span:
        print(f"\nINCONCLUSIVE: no detour exists at this mark site ({span:.2f} m of free "
              f"span against ~0.51 m sterilised by one mark). The question clause 1 asks "
              f"cannot be asked here. This is NOT a failure -- it is an unaskable "
              f"question, and reporting it as FAILED would say the robot did the wrong "
              f"thing when it was never given the chance to do the right one.")
        return 3
    n.publishing = True
    _t = time.time()
    while time.time() - _t < 6.0:
        time.sleep(0.5)

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

    # CONTACT IS 2-D, and the first version of this test got that wrong. Comparing the
    # bumper's X against the mark PLANE answers "did it cross that line", which is the
    # right question only while the robot drives straight at the mark. The moment a
    # detour is possible, crossing the plane 0.23 m off-axis -- beside a mark that spans
    # 0.204 m -- is exactly what SUCCESS looks like, and the 1-D test called it
    # RE-CONTACT. So: the closest the robot's body ever came to any marked point.
    strip_pts = n.strip() or [(a.mark_x, 0.0)]
    closest = min(
        math.hypot(x - mx, y - my)
        for _, x, y in n.track
        for mx, my in strip_pts
    )
    # Conservative on purpose: a corner-only near miss is reported as contact rather
    # than waved through.
    contacted = closest < BODY_RADIUS_M
    print(f"\nresult status      : {status}")
    print(f"start x            : {xs[0]:.3f}")
    print(f"max x reached      : {maxx:.3f}")
    print(f"max |y| deviation  : {maxy:.3f}")
    print(f"bumper reached     : {bumper:.3f}   (mark plane at {a.mark_x:.3f})")
    print(f"distance travelled : {maxx - xs[0]:.3f} m")
    if a.expect_no_path:
        # Clause 4, asserted on the WORLD: the goal must fail and the wheels must never
        # turn. Not read off "no valid path found" in a log -- a log line is a claim about
        # the planner, while zero motion is a fact about the robot.
        aborted = str(status) == "6"
        clear = closest >= STANDOFF_M
        print(f"\nCLAUSE 4 (honest refusal): status {status} "
              f"({'FAILED as required' if aborted else 'DID NOT FAIL'}); closest "
              f"approach {closest:.3f} m against a {STANDOFF_M:.2f} m standoff "
              f"({'CLEAR' if clear else 'TOO CLOSE'})")
        print(f"  (recovery motion is expected and not asserted against: travelled "
              f"{maxx - xs[0]:.3f} m)")
        if aborted and clear:
            print("  PASS -- the goal failed and the rover never came alongside the mark. "
                  "Before the global costmap the same obstacle in this same map produced "
                  "nine RPP refusals and a bumper 0.151 m away.")
        elif not aborted:
            print("  FAIL -- the goal did not fail, so the mark is not reaching the "
                  "planner in a corridor it demonstrably blocks.")
        else:
            print(f"  FAIL -- it refused, but the body still reached {closest:.3f} m of "
                  f"the mark. Refusing while approaching is the behaviour this clause "
                  f"exists to distinguish from refusing outright.")
    elif a.mark == "none":
        print("\nCONTROL RUN: the number that matters is 'distance travelled'. If the "
              "robot did not drive past the mark plane here, the marked run proves nothing.")
    else:
        print(f"\nclosest approach   : {closest:.3f} m from the body centre to the "
              f"nearest marked point (body radius {BODY_RADIUS_M:.3f})")
        if contacted:
            print(f"RE-CONTACT: the body reached within {closest:.3f} m of a marked "
                  f"point. In the room that is the leg again.")
        else:
            print(f"PROTECTED: the body never came closer than {closest:.3f} m to a "
                  f"marked point -- {closest - BODY_RADIUS_M:.3f} m of clearance.")
        if a.min_detour_y is not None:
            # Clause 3. Only meaningful on a SUCCEEDED marked arm: an aborted run has
            # nothing to route around yet, and judging its wander as a detour would
            # invent a pass out of a failure.
            succeeded = str(status) == "4"
            real = maxy > a.min_detour_y
            print(f"detour check       : max |y| {maxy:.3f} vs control {a.min_detour_y:.3f}"
                  f" -> {'REAL DETOUR' if real else 'NO DETOUR'}")
            if succeeded and not real:
                print("  !! SUCCEEDED WITHOUT DEVIATING. The robot drove the same route "
                      "it drove with no mark present, which means the mark stopped "
                      "working rather than that the planner routed around it. This is a "
                      "FAILURE of the three-clause bar wearing a green result's clothes.")
            elif succeeded and real:
                print("  three-clause bar: FLIP + REAL DETOUR satisfied on this arm "
                      "(the control clause is judged on the other arm).")
    ex.shutdown(); rclpy.try_shutdown()
    return 0


sys.exit(main())
