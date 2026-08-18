#!/usr/bin/env python3
"""RIG STEP ZERO: is footprint clearing ERASURE or SUPPRESSION?

The whole contact-mark geometry rests on one mechanic nobody in this project has
measured, and the two readings of it lead to opposite strip specs:

  H0 -- ERASURE AT BIRTH. `ObstacleLayer::updateCosts` calls `setConvexPolygonCost(
        footprint, FREE_SPACE)` on the LAYER'S OWN grid, so a mark planted under the
        robot is destroyed. A mark inside the footprint circle is a no-op that still
        reports as placed. => the strip must sit OUTSIDE the footprint, and the
        protection gap (a re-approach's bumper reaches past the leg) is unavoidable.

  H1 -- SUPPRESSION WHILE COVERED. `updateBounds` re-marks from the observation buffer
        EVERY cycle, and the publisher republishes the whole accumulated cloud at 2 Hz
        with `expected_update_rate: 0.0` (never stale). So the clearing is undone on the
        next cycle: the mark is absent from the master grid only WHILE the robot covers
        it, and materializes as the robot retreats. => the strip can sit at the TRUE
        obstacle position, own-cell safety is bought by the clearing mechanism itself,
        and inflation gives a correct robot-radius standoff around the real leg.

PRE-REGISTERED, BEFORE THE RUN, so neither reading can be fitted to the result:

  * H1 is confirmed iff the mark cell is BELOW lethal in the MASTER grid while the robot
    covers it, reaches lethal after the robot retreats, and STAYS lethal thereafter.
  * H0 is confirmed iff the mark cell never reaches lethal after retreat.
  * Anything else -- appears then vanishes, or lethal while covered -- falsifies BOTH
    and the geometry argument restarts from the measurement.

THE LAYER'S OWN GRID IS PUBLISHED SEPARATELY (`/local_costmap/touch_layer_raw`), which
settles the question directly rather than by inference: H0 predicts the mark is absent
from the LAYER grid too (it was destroyed there); H1 predicts it is present in the layer
and merely missing from the master. Reading both is the difference between watching the
mechanism and guessing at it from its output.

The run also settles numbers that both specs are derived from and that this project has
so far only ASSERTED, from the deployed binary and the deployed YAML rather than from
anyone's reading of nav2:

  M1  the published footprint polygon: vertex count, circumscribed and inscribed
      (apothem) radii, and whether `footprint_padding` has silently grown it.
  M2  the inflation layer's TRUE inscribed radius and inflation radius, read off a
      radial cost profile around one isolated mark -- the distance where cost stops
      being 253, and the distance where it reaches 0.
  M3  own-cell cost through a retreat, sampled every 5 cm. The claim under test is that
      the own cell NEVER reaches 253 (INSCRIBED_INFLATED_OBSTACLE = start pose blocked =
      D43). If inflation's inscribed radius and the footprint polygon's apothem are the
      same number -- both derived from the same polygon -- then a robot structurally
      cannot bury itself with its own mark, and that is worth knowing exactly.

TWO TRAPS THIS PROBE HIT ON THE WAY, WRITTEN DOWN SO THE NEXT RUN DOES NOT:

  1. `local_costmap` will not ACTIVATE without odom->base_link, and stops updating the
     moment that TF disappears. So the TF lives in a separate persistent publisher
     (`tf_pub.py`, pose read from /tmp/robot_x) and NOT in this script: a probe that owns
     the TF starves the thing it is measuring the instant it exits.
  2. `always_send_full_costmap: false` (the deployed value) means `costmap_raw` carries
     ONE full grid, latched TRANSIENT_LOCAL at activation, and everything after that
     arrives as deltas on `costmap_raw_updates`. A probe that reads only `costmap_raw`
     is reading a snapshot taken BEFORE the experiment began -- it will report a clean
     costmap forever and call it evidence. This one mirrors the grid and applies updates.
     THIS TRAP HAS NOW CAUGHT ITS OWN AUTHOR TWICE: the range-gate probe written later
     the same day read only the latched grid, reported that the control mark never
     reached the costmap, and cost a run before the cause was recognised. A warning in a
     docstring does not survive writing a new file in a hurry -- mirror the grid or do
     not read it.

WHAT THIS PROBE IS NOT, AND THE RESULT THAT PROVED IT. No chassis, no driver, no lidar,
no motion: the robot's pose is a TF, and "retreat" means writing a number to a file. It
measures costmap mechanics and NOT stack behaviour, and the difference turned out to be
the whole answer. This probe's approach sweep shows a mark being suppressed by footprint
clearing before inflation can refuse the robot's centre, and on its own that reads as
"contact marks cannot stop a re-approach". THAT CONCLUSION IS WRONG, and
`closed_loop_mark_test.py` is what proved it wrong: in closed loop RPP's forward
projection refuses the approach 0.151 m short of the mark, because forcing the TF at an
obstacle bypasses every safeguard between the planner and the wheels. Read this probe's
output as "what the costmap does", never as "what the robot does".

UNEXPLAINED, and deliberately not chased: in the strip sweep the own-cell cost column
reads 0 at separation 0.15 while reading 207 at 0.16-0.19 and 245 at 0.10-0.14 -- one
non-monotone row in an otherwise clean column. Left on the record rather than smoothed
away. Chase it only if a closed-loop result ever turns on it.

USAGE, on the Pi:

    echo 0.0 > /tmp/robot_x
    python3 tf_pub.py &                     # FIRST: local_costmap needs TF to activate
    ros2 launch sphero_rvr_driver bringup_stationary_test.launch.py \
        start_lidar:=false static_odom:=false
    python3 probe_touch_layer_suppression.py

`static_odom:=false` because tf_pub.py owns those transforms; two publishers of the same
transform is two answers to "where is the robot".
"""

from __future__ import annotations

import argparse
import math
import struct
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import PolygonStamped
from nav2_msgs.msg import Costmap, CostmapUpdate
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField

#: Cost values, from nav2_costmap_2d/cost_values.hpp. Restated rather than imported
#: because this is a Python probe of a C++ enum; these three are the whole vocabulary.
LETHAL = 254
INSCRIBED = 253
FREE = 0

#: The mark height the real node uses. Must land inside touch_layer's
#: min/max_obstacle_height band or the layer discards it and the probe measures nothing.
MARK_Z_M = 0.15

#: Measured body, base_link-referenced (tests/test_footprint_derivation.py).
FOOTPRINT_FRONT_M = 0.0965

#: Where tf_pub.py reads the robot's x from.
ROBOT_X_PATH = "/tmp/robot_x"

#: A mark planted far enough away to be irrelevant to the geometry under test, kept
#: lethal for the whole retreat. It is the probe's own alignment check: the rolling
#: window moves with the robot, so a grid that no longer shows this mark at this world
#: position is misaligned or stale, and every other number read from it is fiction.
REFERENCE_MARK = (0.80, 0.0)


def marks_qos() -> QoSProfile:
    """Byte-identical to contact_marker_node.marks_qos(). A probe that publishes on a
    different profile than the node it stands in for is measuring a different system."""
    return QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.VOLATILE,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=10,
    )


def costmap_qos() -> QoSProfile:
    """TRANSIENT_LOCAL, to receive the one latched full grid nav2 sent at activation.
    A VOLATILE subscriber connects to this publisher happily and receives nothing."""
    return QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=1,
    )


class CostmapMirror:
    """A live grid, rebuilt from one latched full message plus every delta since.

    Not a convenience: with `always_send_full_costmap: false` this is the ONLY way to see
    the costmap as it is now rather than as it was at activation.
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self.meta = None
        self._data: bytearray | None = None
        self.full_seen = 0
        self.updates_seen = 0

    def on_full(self, msg: Costmap) -> None:
        with self._lock:
            self.meta = msg.metadata
            self._data = bytearray(msg.data)
            self.full_seen += 1

    def seed_geometry_from(self, other: "CostmapMirror") -> bool:
        """Start an all-zero grid with another mirror's geometry.

        For the PER-LAYER topics: `touch_layer_raw` never sends a full grid (measured --
        only its `_updates` stream is live), but every layer shares the master's size,
        resolution and origin, so the master's metadata is the correct frame to apply
        layer deltas into. What this cannot recover is any layer content that predates
        the seeding; in this probe the layer starts empty, so there is none.
        """
        with other._lock:
            if other.meta is None or other._data is None:
                return False
            meta, size = other.meta, len(other._data)
        with self._lock:
            self.meta = meta
            self._data = bytearray(size)
            self.full_seen += 1
        return True

    def on_update(self, msg: CostmapUpdate) -> None:
        with self._lock:
            if self._data is None or self.meta is None:
                return
            width = self.meta.size_x
            for row in range(msg.size_y):
                dst = (msg.y + row) * width + msg.x
                src = row * msg.size_x
                self._data[dst:dst + msg.size_x] = msg.data[src:src + msg.size_x]
            self.updates_seen += 1

    def cost_at(self, x: float, y: float):
        """Cost at a world point, or None when unavailable or outside the window.

        NEVER 0 for "outside" -- "not in the window" and "free floor" are different facts
        and collapsing them is how a probe reports a clear costmap it never read.
        """
        with self._lock:
            if self._data is None or self.meta is None:
                return None
            m = self.meta
            i = int(math.floor((x - m.origin.position.x) / m.resolution))
            j = int(math.floor((y - m.origin.position.y) / m.resolution))
            if not (0 <= i < m.size_x and 0 <= j < m.size_y):
                return None
            return int(self._data[j * m.size_x + i])

    def nonzero(self) -> int:
        with self._lock:
            return 0 if self._data is None else sum(1 for c in self._data if c)


class SuppressionProbe(Node):
    def __init__(self) -> None:
        super().__init__("touch_layer_suppression_probe")
        self._lock = threading.Lock()
        self._points: list[tuple[float, float]] = []
        self._footprint: PolygonStamped | None = None
        self._robot_x = 0.0

        self.master = CostmapMirror()
        self.layer = CostmapMirror()

        self._pub = self.create_publisher(PointCloud2, "/contact_marks", marks_qos())
        # Dispatched through lambdas rather than bound methods on purpose: refresh_full()
        # REPLACES the mirror objects, and a bound method captured at subscribe time
        # would keep feeding deltas into the mirror nobody reads any more.
        self.create_subscription(
            Costmap, "/local_costmap/costmap_raw",
            lambda m: self.master.on_full(m), costmap_qos())
        self.create_subscription(
            CostmapUpdate, "/local_costmap/costmap_raw_updates",
            lambda m: self.master.on_update(m), costmap_qos())
        self.create_subscription(
            CostmapUpdate, "/local_costmap/touch_layer_raw_updates",
            lambda m: self.layer.on_update(m), costmap_qos())
        self.create_subscription(
            PolygonStamped, "/local_costmap/published_footprint",
            self._on_footprint, 1)
        # The real node's republication rate. It is part of what is under test: H1
        # depends on republication outrunning footprint clearing.
        self.create_timer(0.5, self._publish_marks)

    def _publish_marks(self) -> None:
        with self._lock:
            points = list(self._points)
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.height = 1
        msg.width = len(points)
        msg.fields = [
            PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
            for i, n in enumerate(("x", "y", "z"))
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12 * len(points)
        msg.is_dense = True
        msg.data = b"".join(struct.pack("<fff", px, py, MARK_Z_M) for px, py in points)
        self._pub.publish(msg)

    def _on_footprint(self, msg: PolygonStamped) -> None:
        with self._lock:
            self._footprint = msg

    def set_points(self, points) -> None:
        with self._lock:
            self._points = [(float(a), float(b)) for a, b in points]

    def set_robot_x(self, x: float) -> None:
        with self._lock:
            self._robot_x = float(x)
        with open(ROBOT_X_PATH, "w") as fh:
            fh.write(f"{float(x)}\n")

    def refresh_full(self, timeout_s: float = 8.0) -> bool:
        """Re-read a FULL, freshly-aligned master grid by making a new subscription.

        WHY THIS IS NEEDED AND WHY IT MIGHT NOT WORK. The local costmap is a rolling
        window: move the robot and the grid's origin moves with it, so a mirror built
        from one full message plus deltas is correctly aligned only while the robot
        stands still. Deltas carry cell indices, not an origin, so they cannot repair it.
        A new subscription is the only lever this probe has to ask for a whole grid
        again -- and whether nav2 answers with a FRESH one or replays the TRANSIENT_LOCAL
        sample latched at activation is exactly the sort of thing to verify rather than
        assume. `REFERENCE_MARK` is the check: it is planted before any of this and must
        read lethal in any grid that is genuinely current.
        """
        fresh = CostmapMirror()
        sub = self.create_subscription(
            Costmap, "/local_costmap/costmap_raw", fresh.on_full, costmap_qos())
        try:
            deadline = time.time() + timeout_s
            while time.time() < deadline and not fresh.full_seen:
                time.sleep(0.2)
        finally:
            self.destroy_subscription(sub)
        if not fresh.full_seen:
            return False
        self.master = fresh
        self.layer = CostmapMirror()
        self.layer.seed_geometry_from(fresh)
        return True

    def footprint_radii(self):
        """(vertices, min vertex radius, max vertex radius, apothem), recentred on the
        robot's current pose."""
        with self._lock:
            fp = self._footprint
            rx = self._robot_x
        if fp is None or not fp.polygon.points:
            return None
        pts = [(p.x - rx, p.y) for p in fp.polygon.points]
        radii = [math.hypot(px, py) for px, py in pts]
        apothem = min(
            _distance_to_segment(pts[k], pts[(k + 1) % len(pts)])
            for k in range(len(pts))
        )
        return len(pts), min(radii), max(radii), apothem


def _distance_to_segment(a, b) -> float:
    """Perpendicular distance from the origin to segment a-b."""
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    denom = dx * dx + dy * dy
    if denom == 0.0:
        return math.hypot(ax, ay)
    t = max(0.0, min(1.0, -(ax * dx + ay * dy) / denom))
    return math.hypot(ax + t * dx, ay + t * dy)


def _wait_for_costmap(probe: SuppressionProbe, timeout_s: float = 60.0) -> bool:
    """A probe that samples before the grid exists reads None and calls it free."""
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        if probe.master.full_seen:
            probe.layer.seed_geometry_from(probe.master)
            return True
        time.sleep(0.5)
    return False


def _settle(seconds: float = 2.0) -> None:
    """Let several costmap cycles pass. At 5 Hz, 2 s is ten cycles -- enough that a
    one-cycle race cannot masquerade as a steady state."""
    time.sleep(seconds)


def run(probe: SuppressionProbe, mark_x: float) -> int:
    print("\n=== M1: the published footprint, from the deployed binary ===")
    fp = probe.footprint_radii()
    if fp is None:
        print("  NO FOOTPRINT PUBLISHED -- cannot proceed; the polygon is half the argument.")
        return 2
    n, rmin, rmax, apothem = fp
    print(f"  vertices                : {n}")
    print(f"  vertex radius min / max : {rmin:.4f} / {rmax:.4f} m")
    print(f"  apothem (min edge dist) : {apothem:.4f} m")
    print(f"  configured robot_radius : 0.1450 m")
    print(f"  => padding applied      : {rmax - 0.145:+.4f} m at the widest vertex")

    print("\n=== M2: inflation's real radii, from a radial profile around one mark ===")
    print("  one isolated mark at (1.000, 0.000), robot parked at the origin")
    probe.set_robot_x(0.0)
    probe.set_points([(1.0, 0.0)])
    _settle(4.0)
    profile = []
    d = 0.0
    while d <= 0.45:
        profile.append((d, probe.master.cost_at(1.0 + d, 0.0)))
        d = round(d + 0.05, 3)
    for dist, cost in profile:
        print(f"    d={dist:.2f} m  master cost={cost}")
    print(f"  layer grid at the mark  : {probe.layer.cost_at(1.0, 0.0)}")
    print(f"  mirror health -- master: {probe.master.full_seen} full / "
          f"{probe.master.updates_seen} updates, {probe.master.nonzero()} nonzero cells; "
          f"layer: {probe.layer.full_seen} full / {probe.layer.updates_seen} updates, "
          f"{probe.layer.nonzero()} nonzero cells")
    inscribed_edge = max((d for d, c in profile if c is not None and c >= INSCRIBED),
                         default=None)
    zero_edge = min((d for d, c in profile if c == FREE), default=None)
    print(f"  last distance at >= INSCRIBED(253): {inscribed_edge}")
    print(f"  first distance at FREE(0)         : {zero_edge}")
    if inscribed_edge is None:
        print("  MARK NEVER REACHED THE MASTER GRID -- everything below is meaningless.")
        return 2

    print("\n=== H0 vs H1: the mark under the robot ===")
    print(f"  mark at ({mark_x:.4f}, 0.000); robot starts at x=0 covering it")
    probe.set_points([(mark_x, 0.0)])
    probe.set_robot_x(0.0)
    _settle(4.0)
    covered_master = probe.master.cost_at(mark_x, 0.0)
    covered_layer = probe.layer.cost_at(mark_x, 0.0)
    own_covered = probe.master.cost_at(0.0, 0.0)
    print(f"  while covered : master {covered_master}   LAYER {covered_layer}   "
          f"own cell {own_covered}")
    print("  (H0 predicts the LAYER is clear too -- destroyed. H1 predicts the layer "
          "holds it while the master does not.)")

    print("\n=== M3: retreat, 5 cm at a time ===")
    print(f"  a reference mark is planted at {REFERENCE_MARK} and must read LETHAL in "
          f"every row; a row where it does not is a MISALIGNED grid, not a measurement.")
    probe.set_points([(mark_x, 0.0), REFERENCE_MARK])
    _settle(3.0)
    print(f"  {'robot_x':>8}  {'master':>7}  {'layer':>6}  {'own':>5}  {'ref':>5}   note")
    materialized_at = None
    own_max = 0
    aligned_rows = 0
    for step in range(0, 11):
        rx = round(-0.05 * step, 3)
        probe.set_robot_x(rx)
        _settle(1.5)
        if not probe.refresh_full():
            print(f"  {rx:8.3f}  -- could not re-read a full grid; row skipped")
            continue
        _settle(1.5)
        mc = probe.master.cost_at(mark_x, 0.0)
        lc = probe.layer.cost_at(mark_x, 0.0)
        oc = probe.master.cost_at(rx, 0.0)
        ref = probe.master.cost_at(*REFERENCE_MARK)
        # >= INSCRIBED, not == LETHAL. A sample point lands in whichever cell contains
        # it, which is often the NEIGHBOUR of the cell the mark itself landed in -- and a
        # neighbour of a lethal cell reads 253, not 254. Demanding 254 threw away every
        # valid row on the first run and reported the experiment as unmeasured.
        aligned = ref is not None and ref >= INSCRIBED
        note = "" if aligned else "GRID STALE/MISALIGNED -- ignore this row"
        if aligned:
            aligned_rows += 1
            if mc is not None and mc >= INSCRIBED and materialized_at is None:
                materialized_at = rx
                note = "<- materializes in master"
            if oc is not None:
                own_max = max(own_max, oc)
                if oc >= INSCRIBED:
                    note += "  !! OWN CELL AT INSCRIBED (D43)"
        print(f"  {rx:8.3f}  {str(mc):>7}  {str(lc):>6}  {str(oc):>5}  {str(ref):>5}   {note}")

    if aligned_rows == 0:
        print("\n  EVERY ROW WAS STALE: a new subscription replays the grid latched at "
              "activation rather than a current one. The retreat is UNMEASURED -- M1, M2 "
              "and the covered-robot reading above still stand (robot stationary), but "
              "materialization needs a different lever.")
        return 3

    print("\n=== persistence, and the re-approach that matters ===")
    probe.set_robot_x(-0.60)
    _settle(4.0)
    probe.refresh_full()
    _settle(1.5)
    late = probe.master.cost_at(mark_x, 0.0)
    print(f"  parked well clear at -0.600 : mark cell {late}  "
          f"(reference {probe.master.cost_at(*REFERENCE_MARK)})")
    # THE QUESTION THE WHOLE PORT ASKS: after the robot comes back, is the mark still
    # there to refuse it -- or does the returning footprint clear away the thing that
    # was supposed to stop it? A mark that only survives while nobody approaches is not
    # protection.
    for back_to in (-0.30, -0.20):
        probe.set_robot_x(back_to)
        _settle(3.0)
        probe.refresh_full()
        _settle(1.5)
        c = probe.master.cost_at(mark_x, 0.0)
        print(f"  re-approached to {back_to:+.3f}    : mark cell {c}  "
              f"(reference {probe.master.cost_at(*REFERENCE_MARK)})")
        late = c if c is not None else late

    print("\n=== VERDICT (pre-registered) ===")
    if materialized_at is not None and late is not None and late >= INSCRIBED:
        print(f"  H1 CONFIRMED -- SUPPRESSION, not erasure. Absent from the master while "
              f"covered, materialized at robot_x={materialized_at:.3f}, persisted.")
        print("  => a strip may be placed at the TRUE obstacle position.")
        verdict = 0
    elif late is None or late < INSCRIBED:
        print("  H0 CONFIRMED -- ERASURE. The mark never materialized after retreat.")
        print("  => the strip must sit outside the footprint; the protection gap is real.")
        verdict = 1
    else:
        print("  NEITHER. Report the table above and re-derive before geometry lands.")
        verdict = 3

    print(f"\n  highest own-cell cost seen in the retreat: {own_max} "
          f"(INSCRIBED={INSCRIBED} is the D43 threshold)")
    if own_max >= INSCRIBED:
        print("  !! the robot CAN bury itself with its own mark -- this outranks H0/H1.")
    return verdict


def main() -> int:
    parser = argparse.ArgumentParser(description="Rig step zero: suppression vs erasure")
    parser.add_argument(
        "--mark-x", type=float, default=FOOTPRINT_FRONT_M,
        help="mark position ahead of base_link (default: the measured bumper, 0.0965)",
    )
    args = parser.parse_args()

    rclpy.init()
    probe = SuppressionProbe()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(probe)
    threading.Thread(target=executor.spin, daemon=True).start()
    try:
        if not _wait_for_costmap(probe):
            print("NO LATCHED COSTMAP on /local_costmap/costmap_raw and "
                  "/local_costmap/touch_layer_raw. Is controller_server ACTIVE, and is "
                  "tf_pub.py running? (ros2 lifecycle get /controller_server)")
            return 2
        return run(probe, args.mark_x)
    finally:
        executor.shutdown()
        probe.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    sys.exit(main())
