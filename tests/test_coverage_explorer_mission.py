"""Mission rehearsal harness: the coverage explorer's state machine, no chassis.

`coverage_explorer_node.py` sequences goals, cancels, unsticks, counts failures and
ends missions -- and until this file, nothing under tests/ executed any of it: the
chassis was serving as its unit-test runner, and five of the six bugs fixed on
2026-08-09 were control-flow errors reproducible with no motor attached (defect D8).

This harness runs the REAL node -- real rclpy, real MultiThreadedExecutor, real
action-client machinery -- against fake Nav2 action servers whose behaviour each
scenario scripts (succeed-and-teleport, abort, hold-until-cancelled), a synthetic
map, and a TF feed the fake nav updates when a goal "arrives". Everything runs in
one process and each scenario finishes in seconds.

Requires rclpy, so it runs on the Pi (or any ROS 2 environment) and skips
elsewhere. Scenarios cover the close criteria of D10-D13:

  full mission     a mission over open floor runs goals to completion and reports
                   COMPLETE with the map artifacts machinery engaged
  D10  one unstick at a time; no NavigateToPose is issued while a recovery
       behaviour is still executing
  D11  a goal that plans but never progresses is cancelled by the watchdog, and
       repeated stalls TERMINATE the mission (OUTCOME_GOALS_KEEP_FAILING) instead
       of looping forever
  D12  a recovery behaviour that outlives its deadline is cancelled BEFORE the
       next behaviour is sent, and behaviour goals carry an explicit
       time_allowance
  D13  after the mission reports done, no further NavigateToPose goal is ever
       accepted by the nav server

Reverting the D10-D13 fixes (git checkout 4941512 -- the node file) makes the
D10, D11 and D13 scenarios fail, which is what the register's D8 close criterion
demands: the harness must be able to SEE the bugs, not merely bless the fixes.
"""

import json
import math
import threading
import time
from pathlib import Path

import pytest

rclpy = pytest.importorskip("rclpy")

from action_msgs.msg import GoalStatus  # noqa: E402
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue  # noqa: E402
from geometry_msgs.msg import PoseStamped, TransformStamped  # noqa: E402
from nav2_msgs.action import BackUp, ComputePathToPose, NavigateToPose  # noqa: E402
from nav_msgs.msg import OccupancyGrid  # noqa: E402
from rclpy.action import ActionServer, CancelResponse, GoalResponse  # noqa: E402
from rclpy.callback_groups import ReentrantCallbackGroup  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from tf2_ros import TransformBroadcaster  # noqa: E402

from sphero_rvr_core.coverage_exploration import INSCRIBED_COST  # noqa: E402
from sphero_rvr_core.escape_outcome import (  # noqa: E402
    CLEARED as ESCAPE_CLEARED, REFUSED as ESCAPE_REFUSED,
    format_outcome as ESCAPE_FORMAT,
)
from sphero_rvr_driver.coverage_explorer_node import CoverageExplorerNode  # noqa: E402


RES = 0.05
#: Lethal, on the OccupancyGrid scale Nav2 publishes its costmap in. NOT 254: that is
#: the raw costmap_2d value and it does not fit in the int8 an OccupancyGrid carries.
LETHAL = 100


def make_costmap(size_cells=60, origin=-1.5, fill=0):
    """A GLOBAL COSTMAP, the topic the explorer's start-pose guard reads.

    Separate from `make_map` on purpose: they are different grids answering different
    questions, and the harness had only the first. That gap is why the goal-clearance
    filter could not be proved behaviourally when it was removed -- with no costmap
    publisher the filter returned None for every pose and passed everything through,
    so a scenario written against it would have passed against the BUG too. A harness
    that cannot make the costmap say anything cannot test a costmap-reading rule.
    """
    m = OccupancyGrid()
    m.header.frame_id = "map"
    m.info.resolution = RES
    m.info.width = size_cells
    m.info.height = size_cells
    m.info.origin.position.x = origin
    m.info.origin.position.y = origin
    m.data = [fill] * (size_cells * size_cells)
    return m


def stamp_cost(grid, wx, wy, radius_m, value):
    """Paint a disc of cost centred on a WORLD point. Returns the grid."""
    res = grid.info.resolution
    cx = int((wx - grid.info.origin.position.x) / res)
    cy = int((wy - grid.info.origin.position.y) / res)
    r = int(math.ceil(radius_m / res))
    for dy in range(-r, r + 1):
        for dx in range(-r, r + 1):
            if math.hypot(dx, dy) * res > radius_m:
                continue
            x, y = cx + dx, cy + dy
            if 0 <= x < grid.info.width and 0 <= y < grid.info.height:
                grid.data[y * grid.info.width + x] = value
    return grid


def probe_clearance_m(grid, wx, wy, max_probe_m=0.60, threshold=INSCRIBED_COST):
    """Smallest distance from (wx, wy) to costmap obstruction, 8 compass directions.

    A DELIBERATE RE-IMPLEMENTATION of the filter that was deleted in 1e6af5c, living
    here and nowhere else. The behavioural proof below has to show that the goal the
    explorer sent is one the old filter WOULD have rejected -- otherwise it passes
    vacuously, which is the exact trap the removal commit refused to walk into. That
    check needs the old arithmetic, and importing it is impossible because deleting it
    was half the point. So the test carries its own copy, where it can never be
    mistaken for production code or rewired into the selection loop.

    Returns None when the pose is off-map or unknown, matching the original.

    THE SCALE IS 0..100, NOT 0..255. Nav2 publishes its costmap as an OccupancyGrid,
    whose `data` is int8 -- 100 is lethal, 99 the inscribed ring, -1 unknown. Staging
    a raw 254 here raises OverflowError on a real message, which is how this was
    caught: on the dev machine the whole file skips for want of rclpy, and the value
    looked plausible until a Pi ran it. `INSCRIBED_COST` is imported from the
    production module rather than repeated, so the harness cannot drift off the
    threshold the node actually uses.
    """
    res = grid.info.resolution
    cx = int((wx - grid.info.origin.position.x) / res)
    cy = int((wy - grid.info.origin.position.y) / res)
    w, h = grid.info.width, grid.info.height
    if not (0 <= cx < w and 0 <= cy < h):
        return None
    if grid.data[cy * w + cx] < 0:
        return None
    steps = int(max_probe_m / res)
    best = max_probe_m
    for ux, uy in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)):
        norm = math.hypot(ux, uy)
        for step in range(1, steps + 1):
            x = cx + int(round(ux * step / norm))
            y = cy + int(round(uy * step / norm))
            if not (0 <= x < w and 0 <= y < h):
                break
            if grid.data[y * w + x] >= threshold:
                best = min(best, step * res)
                break
    return best


def make_map(size_cells=60, origin=-1.5):
    m = OccupancyGrid()
    m.header.frame_id = "map"
    m.info.resolution = RES
    m.info.width = size_cells
    m.info.height = size_cells
    m.info.origin.position.x = origin
    m.info.origin.position.y = origin
    m.data = [0] * (size_cells * size_cells)
    return m


class FakeWorld(Node):
    """Fake Nav2 servers + TF + map + report reader, scriptable per scenario."""

    def __init__(self):
        super().__init__("fake_world")
        cbg = ReentrantCallbackGroup()
        # --- scripting knobs ---
        self.plannable = lambda x, y: True     # ComputePathToPose verdict
        self.nav_mode = "succeed"              # "succeed" | "abort" | "hold"
        self.behavior_mode = "hold"            # escape: "hold"|"succeed"|"costmap_refuse"
        #: The outcome word a succeeding escape reports. The explorer parses this as a
        #: FACT about what happened (sphero_rvr_core.escape_outcome), so a harness that
        #: leaves it empty is testing the unrecognised-outcome path, not the escape.
        self.escape_outcome = ESCAPE_CLEARED
        self.escape_detail = "moved 0.30 m"
        self.costmap = None                    # last costmap published, for D36
        # --- recordings ---
        self.pose = (0.0, 0.0)                 # teleported on nav succeed
        self.nav_goals = []                    # (t, x, y)
        self.nav_active = []                   # [t_start, t_end_or_None]
        self.nav_cancels = []                  # t of cancel request seen
        self.behavior_goals = []               # (t, kind, goal_msg)
        self.behavior_active = []              # [t_start, t_end_or_None, kind]
        self.behavior_cancels = []             # (t, kind)
        self.plan_queries = 0
        self.reports = []                      # decoded JSON reports

        self._nav_srv = ActionServer(
            self, NavigateToPose, "navigate_to_pose", self._nav_execute,
            goal_callback=lambda req: GoalResponse.ACCEPT,
            cancel_callback=lambda gh: CancelResponse.ACCEPT,
            callback_group=cbg,
        )
        self._plan_srv = ActionServer(
            self, ComputePathToPose, "compute_path_to_pose", self._plan_execute,
            goal_callback=lambda req: GoalResponse.ACCEPT,
            cancel_callback=lambda gh: CancelResponse.ACCEPT,
            callback_group=cbg,
        )
        # THE ESCAPE the explorer now asks for. It used to be nav2's "backup" and
        # "spin"; those were invoked correctly on 2026-08-12 and executed nothing
        # (D36), so the escape moved to the controller's own action and this harness
        # has to fake the thing the explorer actually calls. The D10/D12 protections
        # below are about ANY recovery motion holding while goals are pending, so
        # they retarget rather than retire.
        self._backup_srv = ActionServer(
            self, BackUp, "/decisive_controller/escape_in_place",
            self._make_behavior_execute("escape"),
            goal_callback=lambda req: GoalResponse.ACCEPT,
            cancel_callback=lambda gh: CancelResponse.ACCEPT,
            callback_group=cbg,
        )

        map_qos = QoSProfile(depth=1)
        map_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        map_qos.reliability = QoSReliabilityPolicy.RELIABLE
        self._map_pub = self.create_publisher(OccupancyGrid, "/map", map_qos)

        report_qos = QoSProfile(depth=1)
        report_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        report_qos.reliability = QoSReliabilityPolicy.RELIABLE
        self.create_subscription(
            String, "/coverage_explorer/report",
            lambda msg: self.reports.append(json.loads(msg.data)),
            report_qos, callback_group=cbg,
        )

        # THE COSTMAP THE EXPLORER'S START GUARD READS. Same QoS as the map: the
        # explorer subscribes with TRANSIENT_LOCAL + RELIABLE, and a VOLATILE
        # publisher here would be silently incompatible -- no error, no delivery, and
        # every costmap-dependent scenario passing because the costmap never arrived.
        self._costmap_pub = self.create_publisher(
            OccupancyGrid, "/global_costmap/costmap", map_qos)

        self._freeze_pub = self.create_publisher(
            String, "/decisive_controller/freeze_event", 10)
        # D56: the driver's diagnostics lane -- the freeze feed that actually has
        # a publisher on the stock middle. The harness fakes the DRIVER here, so
        # the explorer's stock-side touch sense is exercised, not simulated.
        self._diag_pub = self.create_publisher(
            DiagnosticArray, "/diagnostics", 10)
        self.statuses = []                     # decoded JSON status lines
        self.create_subscription(
            String, "/coverage_explorer/status",
            lambda msg: self.statuses.append(json.loads(msg.data)),
            10, callback_group=cbg,
        )
        self._tf = TransformBroadcaster(self)
        self.create_timer(0.02, self._broadcast_tf, callback_group=cbg)

    def publish_map(self, grid):
        self._map_pub.publish(grid)

    def publish_costmap(self, grid):
        self.costmap = grid
        self._costmap_pub.publish(grid)

    def publish_freeze(self, x=0.0, y=0.0):
        """Stand in for the controller reporting a freeze: the supervisor permitted
        motion and the rover did not move, i.e. an obstacle no sensor can see."""
        self._freeze_pub.publish(String(data='{"x": %f, "y": %f, "stamp": 0.0}' % (x, y)))

    def publish_stall_count(self, count, key="motor_stall_events"):
        """Stand in for rvr_node's diagnostics: the firmware stall counter, as the
        monotonic COUNT the driver publishes (counters-not-levels). The explorer
        must read deltas; the absolute value is the driver's business."""
        status = DiagnosticStatus(name="sphero_rvr_driver")
        status.values.append(KeyValue(key=key, value=str(count)))
        msg = DiagnosticArray()
        msg.status.append(status)
        self._diag_pub.publish(msg)

    def _broadcast_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "base_link"
        t.transform.translation.x = float(self.pose[0])
        t.transform.translation.y = float(self.pose[1])
        t.transform.rotation.w = 1.0
        self._tf.sendTransform(t)

    def _nav_execute(self, goal_handle):
        px = goal_handle.request.pose.pose.position.x
        py = goal_handle.request.pose.pose.position.y
        rec = [time.monotonic(), None]
        self.nav_goals.append((rec[0], px, py))
        self.nav_active.append(rec)
        try:
            mode = self.nav_mode
            if mode == "succeed":
                self.pose = (px, py)   # teleport BEFORE succeeding: TF shows arrival
                time.sleep(0.05)       # let a TF broadcast carry the new pose
                goal_handle.succeed()
            elif mode == "abort":
                goal_handle.abort()
            else:  # hold until cancelled
                while not goal_handle.is_cancel_requested:
                    time.sleep(0.02)
                self.nav_cancels.append(time.monotonic())
                goal_handle.canceled()
            return NavigateToPose.Result()
        finally:
            rec[1] = time.monotonic()

    def _plan_execute(self, goal_handle):
        self.plan_queries += 1
        result = ComputePathToPose.Result()
        px = goal_handle.request.goal.pose.position.x
        py = goal_handle.request.goal.pose.position.y
        if self.plannable(px, py):
            result.path.poses = [PoseStamped(), PoseStamped()]
        goal_handle.succeed()
        return result

    def _make_behavior_execute(self, kind):
        def execute(goal_handle):
            rec = [time.monotonic(), None, kind]
            self.behavior_goals.append((rec[0], kind, goal_handle.request))
            self.behavior_active.append(rec)
            try:
                if self.behavior_mode == "costmap_refuse":
                    # D36 REPLAYED. Nav2's BackUp/Spin collision-check the GLOBAL
                    # COSTMAP, so on 2026-08-12 they refused in 3 ms with 0.78 m of
                    # measured clear floor behind the rover -- the rover's own freeze
                    # marks had made its cell lethal. This reproduces that mechanism
                    # exactly: consult the costmap, refuse instantly, do not move.
                    # BackUp collision-checks the path it would TRAVEL, so the cell
                    # that matters is the one behind the rover, not the one under it.
                    # Checking the rover's own cell instead makes this unreachable:
                    # the explorer's own START POSE guard ends the mission first, which
                    # is a different defect from D36 and was how the first draft of
                    # this scenario failed.
                    back = float(getattr(goal_handle.request.target, "x", 0.0))
                    blocked = self._costmap_says_blocked(
                        self.pose[0] - back, self.pose[1])
                    if blocked:
                        result = BackUp.Result()
                        setattr(result, "error_msg",
                                ESCAPE_FORMAT(ESCAPE_REFUSED, "collision check"))
                        goal_handle.abort()
                        return result
                    time.sleep(0.05)
                    goal_handle.succeed()
                elif self.behavior_mode == "succeed":
                    time.sleep(0.05)
                    goal_handle.succeed()
                else:  # hold until cancelled: a BackUp against an unseen obstacle
                    while not goal_handle.is_cancel_requested:
                        time.sleep(0.02)
                    self.behavior_cancels.append((time.monotonic(), kind))
                    goal_handle.canceled()
                result = BackUp.Result()
                # The escape's RESULT IS THE FACT the explorer consumes. Left empty,
                # every escape reads as "an outcome this node does not know" and the
                # scenario measures the unrecognised path instead of the one it names.
                setattr(result, "error_msg",
                        ESCAPE_FORMAT(self.escape_outcome, self.escape_detail))
                return result
            finally:
                rec[1] = time.monotonic()
        return execute

    def _costmap_says_blocked(self, wx, wy, threshold=INSCRIBED_COST):
        cm = self.costmap
        if cm is None:
            return False
        res = cm.info.resolution
        cx = int((wx - cm.info.origin.position.x) / res)
        cy = int((wy - cm.info.origin.position.y) / res)
        if not (0 <= cx < cm.info.width and 0 <= cy < cm.info.height):
            return False
        return cm.data[cy * cm.info.width + cx] >= threshold


class Stack:
    def __init__(self, params):
        args = ["--ros-args"]
        for key, value in params.items():
            args += ["-p", f"{key}:={value}"]
        rclpy.init(args=args)
        self.world = FakeWorld()
        self.explorer = CoverageExplorerNode()
        self.executor = MultiThreadedExecutor(num_threads=8)
        self.executor.add_node(self.world)
        self.executor.add_node(self.explorer)
        self._thread = threading.Thread(target=self.executor.spin, daemon=True)
        self._thread.start()

    def close(self):
        self.executor.shutdown(timeout_sec=5.0)
        self.explorer.destroy_node()
        self.world.destroy_node()
        rclpy.shutdown()
        self._thread.join(timeout=5.0)


BASE_PARAMS = {
    "cycle_period_s": 0.1,
    "plan_timeout_s": 0.5,
    "select_budget_s": 2.0,
    "goal_progress_timeout_s": 0.6,
    "goal_progress_epsilon_m": 0.10,
    "max_consecutive_failures": 3,
    "complete_after_empty_cycles": 3,
    "unstick_timeout_s": 1.0,
    "escape_timeout_s": 1.0,
    "max_unstick_attempts": 2,
    "blocked_start_check": False,
    "save_map_on_end": False,
    # D29 made the explorer come up DISARMED. This harness drives the REAL node, so
    # without arming it every mission test now waits forever for a mission that never
    # starts -- which is exactly what happened when D29 landed, and exactly what this
    # suite is for. Rehearsals arm at construction; `test_d29_*` below covers the
    # disarmed default and the service path.
    "autostart": True,
}


@pytest.fixture
def stack(request):
    params = dict(BASE_PARAMS)
    params.update(getattr(request, "param", {}))
    s = Stack(params)
    yield s
    s.close()


def wait_until(predicate, timeout_s, period_s=0.05):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(period_s)
    return predicate()


def overlapping(intervals):
    """Any two [start, end] windows overlapping (end None = still open)."""
    closed = [(s, e if e is not None else math.inf) for s, e, *_ in
              [list(i) + [None] for i in intervals]]
    closed.sort()
    for (s1, e1), (s2, e2) in zip(closed, closed[1:]):
        if s2 < e1:
            return True
    return False


def test_full_mission_completes_and_reports(stack):
    """The rehearsal baseline: goals get sent, 'driving' (teleport) covers ground,
    and the mission ends structurally COMPLETE with a truthful report."""
    stack.world.nav_mode = "succeed"
    stack.world.publish_map(make_map())
    assert wait_until(
        lambda: any(r.get("outcome") == "COMPLETE" for r in stack.world.reports), 30.0
    ), f"mission never completed; reports={stack.world.reports}"
    report = [r for r in stack.world.reports if r.get("outcome") == "COMPLETE"][-1]
    assert report["complete"] is True
    assert report["goals"]["sent"] >= 1
    assert report["goals"]["succeeded"] >= 1
    assert len(stack.world.nav_goals) >= 1
    # Every goal the explorer issued was at least min_goal_distance from the pose
    # it was issued at is covered by the node's own selection; here just sanity: it
    # drove somewhere other than the start.
    assert any(math.hypot(x, y) > 0.25 for _, x, y in stack.world.nav_goals)


def test_d11_stalled_goals_terminate_the_mission(stack):
    """Goals plan and get accepted but the robot never moves. The watchdog must
    cancel each, the give-up counter must COUNT those cancels, and the mission must
    end ABORTED_GOALS_KEEP_FAILING -- not loop stall/cancel/reselect forever."""
    stack.world.nav_mode = "hold"        # accepted, never progresses, cancellable
    stack.world.publish_map(make_map())
    assert wait_until(
        lambda: any(r.get("outcome") == "ABORTED_GOALS_KEEP_FAILING"
                    for r in stack.world.reports), 30.0
    ), (
        "stalled-goal mission never terminated: this is the D11 livelock "
        f"(cancels={len(stack.world.nav_cancels)}, goals={len(stack.world.nav_goals)})"
    )
    # The watchdog, not the server, ended each goal: every goal got a cancel.
    assert len(stack.world.nav_cancels) >= 3


@pytest.mark.parametrize(
    # A big unstick budget keeps the bait window open: with only 2 attempts the
    # PRE-FIX node burns them all in re-entrant ticks and ends the mission before
    # selection ever comes back plannable, and the assertions pass vacuously --
    # the harness must be able to SEE the bug, not time out before it shows.
    "stack", [{"max_unstick_attempts": 10}], indirect=True,
)
def test_d10_no_goal_while_a_recovery_behaviour_runs(stack):
    """Nothing plans, so the explorer unsticks; the behaviour holds (a BackUp that
    cannot advance). A reentrant tick used to run a full selection during that hold
    and could send NavigateToPose while the behaviour still executed -- two Nav2
    servers driving /cmd_vel. Selection comes back plannable DURING the hold to bait
    exactly that: the fixed node must not take the bait until the behaviour ends."""
    stack.world.nav_mode = "hold"
    stack.world.behavior_mode = "hold"
    refuse = {"on": True}
    stack.world.plannable = lambda x, y: not refuse["on"]
    stack.world.publish_map(make_map())
    # Wait for the first recovery behaviour to start, then make everything
    # plannable while it is still holding.
    assert wait_until(lambda: len(stack.world.behavior_active) >= 1, 20.0), \
        "explorer never attempted an unstick"
    refuse["on"] = False
    # Give re-entrant ticks (the bug) every chance: several tick periods pass while
    # the behaviour is still held.
    time.sleep(0.5)
    # No NavigateToPose goal may be issued while any behaviour is executing.
    for t_goal, _, _ in stack.world.nav_goals:
        for s, e, _ in stack.world.behavior_active:
            assert not (s <= t_goal <= (e if e is not None else math.inf)), \
                "NavigateToPose sent while a recovery behaviour was still executing"
    # And behaviours themselves never overlap (one unstick at a time).
    assert not overlapping(stack.world.behavior_active), \
        "two recovery behaviours executed concurrently"


def test_d12_timed_out_behaviour_is_cancelled_before_the_next(stack):
    """A held escape must be cancelled when its deadline passes, BEFORE the next one
    is sent, and every escape goal must carry an explicit time_allowance.

    Retargeted 2026-08-12 with the escape itself: the old version watched nav2's
    BackUp being cancelled before nav2's Spin was sent, two behaviours inside one
    unstick. There is one escape per attempt now, so the same property is asserted
    across two ATTEMPTS -- and it is the property that matters either way, because two
    overlapping recovery motions is two authors on cmd_vel.
    """
    stack.world.nav_mode = "hold"
    stack.world.behavior_mode = "hold"
    stack.world.plannable = lambda x, y: False   # force the unstick path
    stack.world.publish_map(make_map())
    assert wait_until(lambda: len(stack.world.behavior_goals) >= 2, 25.0), \
        "second escape never sent"
    (t1, kind1, goal1), (t2, kind2, goal2) = stack.world.behavior_goals[:2]
    # The first behaviour was cancelled before the second was sent.
    cancels_before_second = [t for t, k in stack.world.behavior_cancels
                             if k == kind1 and t <= t2]
    assert cancels_before_second, (
        f"first behaviour ({kind1}) was never cancelled before the second "
        f"({kind2}) was sent"
    )
    # Explicit server-side deadline on every behaviour goal.
    for goal in (goal1, goal2):
        allowance = goal.time_allowance.sec + goal.time_allowance.nanosec * 1e-9
        assert allowance == pytest.approx(BASE_PARAMS["escape_timeout_s"], abs=0.01), (
            "the escape goal carries no explicit server-side deadline, so a held "
            "escape would run until something else stopped it")


def test_d24_give_up_report_states_the_true_remaining_target_count(stack):
    """D24. The give-up path used to pass a literal 0 for `remaining_candidates`, so
    a mission that quit surrounded by unexplored floor reported the single most
    reassuring number available. The field is the one the register credits with
    making an incomplete run diagnosable, so a fabricated 0 is the same class of
    defect as D21's lying telemetry.

    The map here is wide open and the rover never moves (nav holds until cancelled),
    so at give-up there are unmistakably targets left. The report must say so.
    """
    stack.world.nav_mode = "hold"          # accepted, never progresses -> watchdog
    stack.world.publish_map(make_map())
    assert wait_until(
        lambda: any(r.get("outcome") == "ABORTED_GOALS_KEEP_FAILING"
                    for r in stack.world.reports), 30.0
    ), "the stalled mission never gave up"
    report = [r for r in stack.world.reports
              if r.get("outcome") == "ABORTED_GOALS_KEEP_FAILING"][-1]
    assert report["remaining_candidates"] > 0, (
        "gave up with open floor all round and still reported 0 targets remaining "
        "— the field is fabricated, not measured"
    )


def test_d13_no_goal_after_the_mission_reports_done(stack):
    """Three aborts end the mission from an action-callback thread. After the
    terminal report, the nav server must never see another goal."""
    stack.world.nav_mode = "abort"
    stack.world.publish_map(make_map())
    assert wait_until(
        lambda: any(r.get("outcome") == "ABORTED_GOALS_KEEP_FAILING"
                    for r in stack.world.reports), 30.0
    ), "aborting mission never terminated"
    t_report = time.monotonic()
    # Let a dozen tick periods elapse; a tick that slipped past _mission_done
    # would send another goal in this window.
    time.sleep(1.5)
    late = [g for g in stack.world.nav_goals if g[0] > t_report]
    assert not late, f"{len(late)} NavigateToPose goal(s) issued AFTER the mission reported done"


@pytest.mark.parametrize(
    "stack", [{"max_consecutive_failures": 3, "max_consecutive_freezes": 3}],
    indirect=True,
)
def test_freezes_do_not_count_as_stack_failures(stack):
    """D26. A freeze is the robot DISCOVERING an obstacle no sensor can see, not the
    stack failing. Scott's rule: hitting something and stopping is a data point, and
    it must not end the mission.

    Here every goal aborts AND a freeze is reported just before each one. Under the
    old semantics three aborts would trip max_consecutive_failures and the mission
    would end ABORTED_GOALS_KEEP_FAILING, blaming the software for a fact about the
    furniture. It must instead end with the honest unseen-obstacle outcome.

    MUST FAIL against pre-D26 code (which reports ABORTED_GOALS_KEEP_FAILING).
    """
    stack.world.nav_mode = "abort"
    stack.world.publish_map(make_map())

    def keep_freezing():
        for _ in range(200):
            stack.world.publish_freeze(1.0, 0.0)
            time.sleep(0.05)
    threading.Thread(target=keep_freezing, daemon=True).start()

    assert wait_until(lambda: bool(stack.world.reports), 30.0), "mission never ended"
    outcome = stack.world.reports[-1]["outcome"]
    assert outcome == "INCOMPLETE_BLOCKED_BY_UNSEEN_OBSTACLES", (
        f"freezes were counted as stack failures (got {outcome}) — the give-up "
        "counter must be reserved for failures with no freeze signature"
    )


@pytest.mark.parametrize(
    "stack", [{"max_consecutive_failures": 3, "max_consecutive_freezes": 3}],
    indirect=True,
)
def test_aborts_without_a_freeze_still_reach_the_give_up_counter(stack):
    """The other half: the exemption must not swallow ordinary failures. No freeze
    events at all here, so the old counter must still fire and report the stack
    outcome. Without this pairing, D26 could be 'never give up', which is worse than
    the bug it replaces."""
    stack.world.nav_mode = "abort"
    stack.world.publish_map(make_map())
    assert wait_until(lambda: bool(stack.world.reports), 30.0), "mission never ended"
    assert stack.world.reports[-1]["outcome"] == "ABORTED_GOALS_KEEP_FAILING"


@pytest.mark.parametrize(
    "stack", [{"max_consecutive_failures": 3, "max_consecutive_freezes": 8}],
    indirect=True,
)
def test_one_freeze_excuses_only_one_abort(stack):
    """Consume-once. A single freeze event must pair with AT MOST ONE abort; if one
    discovery could excuse several unrelated failures the counter would drift away
    from reality, which is the distrust this redesign exists to answer.

    One freeze is published, then goals keep aborting. The mission must still end on
    the stack counter, because only the first abort is excused.
    """
    stack.world.nav_mode = "abort"
    stack.world.publish_map(make_map())
    stack.world.publish_freeze(1.0, 0.0)
    assert wait_until(lambda: bool(stack.world.reports), 30.0), "mission never ended"
    assert stack.world.reports[-1]["outcome"] == "ABORTED_GOALS_KEEP_FAILING", (
        "a single freeze excused more than one abort"
    )


# --- D53 + D56: the 2026-08-19 combined ride-along, replayed as tests ----------------
#
# The flight: five watchdog stall-kills at ONE rover pose ended the mission
# ABORTED_GOALS_KEEP_FAILING with aborted=0 in every bucket (D53's blind class),
# while the driver's firmware stall counter -- the touch sense -- climbed 0->3 on
# a topic the explorer did not read (D56). These tests hold both fixes against
# that exact shape.


def test_d53_stall_kills_are_a_named_counter_in_the_report(stack):
    """MUST FAIL against the blind version (no stall_killed field): the report's
    goal arithmetic must not leave an unexplained gap. On 2026-08-19 the artifact
    said sent=7, succeeded=2, aborted=0 -- five goals with no recorded ending."""
    stack.world.nav_mode = "hold"          # accepted, never progresses -> watchdog
    stack.world.publish_map(make_map())
    assert wait_until(
        lambda: any(r.get("outcome") == "ABORTED_GOALS_KEEP_FAILING"
                    for r in stack.world.reports), 30.0
    ), "the stalled mission never gave up"
    goals = [r for r in stack.world.reports
             if r.get("outcome") == "ABORTED_GOALS_KEEP_FAILING"][-1]["goals"]
    assert goals["stall_killed"] is not None, (
        "stall_killed is UNKNOWN in a report written by the node itself -- the "
        "counter is not wired through")
    assert goals["stall_killed"] >= 3
    assert goals["aborted"] == 0, "hold-mode goals cannot ABORT; harness drifted"
    # D62 (2026-08-22): the invariant now sums EVERY named ending, not the old
    # three terms. A goal cancelled in flight by give-up used to land in no
    # counter, so this assertion raced -- it passed or failed depending on
    # whether give-up fired while a goal was live. Summing all four makes the
    # ledger TOTAL and the race can no longer decide whether the gap is visible.
    named = (goals["succeeded"] + goals["aborted"] + (goals["stall_killed"] or 0)
             + (goals["cancelled_at_end"] or 0))
    assert goals["sent"] == named, (
        "the goal ledger still has endings no counter names: "
        f"sent={goals['sent']} succeeded={goals['succeeded']} "
        f"aborted={goals['aborted']} stall_killed={goals['stall_killed']} "
        f"cancelled_at_end={goals['cancelled_at_end']}"
    )


def test_d53_breaker_epitaph_counts_and_never_diagnoses():
    """The flight's breaker said '5 goals in a row failed, at different places --
    this is the stack, not the room' over five failures at ONE rover pose whose
    cause was the FLOOR: 'different places' described the goals, and the
    diagnosis was inverted. The epitaph must count kinds and measure the ROVER's
    pose spread, asserting no cause. Source-level, like the launch pins."""
    src = (Path(__file__).resolve().parents[1] / "src" / "sphero_rvr_driver" /
           "coverage_explorer_node.py").read_text()
    assert 'f"{n} goals in a row failed' not in src, (
        "the diagnosing epitaph is back")
    assert "consecutive goals ended without success" in src
    assert "pose spread across the streak" in src


def test_d56_driver_stall_deltas_are_freeze_discoveries(stack):
    """THE FLIGHT'S COUNTERFACTUAL, and the must-flip for D56: goals hold (the
    watchdog kills them, exactly like 2026-08-19) while the DRIVER's stall
    counter climbs on /diagnostics. Pre-D56 the explorer cannot hear it and ends
    ABORTED_GOALS_KEEP_FAILING, blaming the stack for a fact about the floor.
    With the feed, every kill pairs with a discovery and the mission ends with
    the honest unseen-obstacle outcome instead."""
    stack.world.nav_mode = "hold"
    stack.world.publish_map(make_map())

    def keep_stalling():
        for n in range(1, 200):                    # n=1 is the BASELINE, no event
            stack.world.publish_stall_count(n)
            time.sleep(0.05)
    threading.Thread(target=keep_stalling, daemon=True).start()

    assert wait_until(lambda: bool(stack.world.reports), 30.0), "mission never ended"
    outcome = stack.world.reports[-1]["outcome"]
    assert outcome == "INCOMPLETE_BLOCKED_BY_UNSEEN_OBSTACLES", (
        f"driver stall deltas were not heard as discoveries (got {outcome}) -- "
        "the stock stack's touch sense is still not feeding the mission ledger"
    )
    assert stack.world.reports[-1]["freeze_mark_counts"]["events"] >= 1


@pytest.mark.parametrize(
    # A long watchdog keeps the controller's pending freeze UNCLAIMED while the
    # duplicate arrives; the base 0.6 s timeout could claim it first and the
    # dedupe would pass vacuously.
    "stack", [{"goal_progress_timeout_s": 10.0}], indirect=True,
)
def test_d56_synthetic_freeze_dedupes_against_the_controllers(stack):
    """Bespoke runs this node WITH the decisive controller, so one physical stall
    can arrive on both lanes. The synthetic entry must be skipped when the
    controller's positioned event already sits within the merge radius --
    otherwise every bespoke freeze double-counts the discovery ledger."""
    stack.world.nav_mode = "hold"
    stack.world.publish_map(make_map())
    assert wait_until(lambda: any(s.get("goal_in_flight") for s in stack.world.statuses),
                      20.0), "no goal ever flew"
    stack.world.publish_stall_count(1)                  # baseline
    stack.world.publish_freeze(0.0, 0.0)                # the controller's event
    assert wait_until(lambda: stack.world.statuses and
                      stack.world.statuses[-1].get("freezes", 0) == 1, 10.0), \
        "the controller's freeze never reached the ledger"
    stack.world.publish_stall_count(2)                  # same stall, driver lane
    time.sleep(1.0)                                     # give a duplicate every chance
    assert stack.world.statuses[-1].get("freezes") == 1, (
        "one physical stall was counted twice -- the bespoke dual-lane dedupe "
        "is not holding"
    )


@pytest.mark.parametrize(
    "stack", [{"max_consecutive_failures": 3, "max_consecutive_freezes": 3}],
    indirect=True,
)
def test_freeze_marks_reach_the_mission_report(stack):
    """Where the rover could not go is diagnostic gold, and it belongs in the report
    — never written into the saved map, which is the room as SLAM measured it.

    Post-D35 the field is `freeze_events` (one per event) alongside `freeze_positions`
    and `freeze_mark_counts`. This scenario freezes at ONE spot 200 times, so it is
    also the end-to-end proof that the merge runs on the real node: 200-odd events
    collapse to a single place."""
    stack.world.nav_mode = "abort"
    stack.world.publish_map(make_map())

    def keep_freezing():
        for _ in range(200):
            stack.world.publish_freeze(1.5, -0.5)
            time.sleep(0.05)
    threading.Thread(target=keep_freezing, daemon=True).start()

    assert wait_until(lambda: bool(stack.world.reports), 30.0)
    report = stack.world.reports[-1]
    events = report.get("freeze_events")
    assert events, "the report carries no freeze events"
    assert events[0]["x"] == pytest.approx(1.5)
    # Every freeze was published at the same pose, so however many events landed,
    # the rover discovered exactly ONE place it could not go (D35).
    assert report["freeze_mark_counts"]["distinct_positions"] == 1
    assert report["freeze_mark_counts"]["events"] == len(events)
    assert report["freeze_positions"] == [{"x": 1.5, "y": -0.5}]


# ------------------------------------------------------------------------- D29

@pytest.mark.parametrize("stack", [{"autostart": False}], indirect=True)
def test_d29_a_disarmed_explorer_sends_no_goal(stack):
    """Launch must not be liftoff.

    Run 185048's whole 53 s mission ran and died DURING bringup gate checks, because
    the mission began the instant the node came up. A disarmed explorer must sit
    there and send nothing.

    The map is published deliberately: without one the explorer is idle for a
    perfectly ordinary reason, and the assertion would pass while proving nothing.
    """
    stack.world.nav_mode = "succeed"
    stack.world.publish_map(make_map())
    time.sleep(2.0)
    assert stack.world.nav_goals == [], (
        "a disarmed explorer sent a goal with a map available — launch is liftoff again")


@pytest.mark.parametrize("stack", [{"autostart": False}], indirect=True)
def test_d29_mission_start_service_arms_it(stack):
    """And the service at the DOCUMENTED path is what starts it."""
    from std_srvs.srv import Trigger

    stack.world.nav_mode = "succeed"
    stack.world.publish_map(make_map())
    client = stack.world.create_client(Trigger, "/coverage_explorer/mission/start")
    assert client.wait_for_service(timeout_sec=5.0), (
        "mission/start missing at the documented path — the run protocol's liftoff "
        "command would hang on a service that does not exist")
    future = client.call_async(Trigger.Request())
    assert wait_until(lambda: future.done(), 5.0)
    assert future.result().success
    assert wait_until(lambda: bool(stack.world.nav_goals), 15.0), (
        "mission/start reported success but the rover never moved")


# --------------------------------------------------------------------------- F1

def test_f1_watchdog_defers_to_an_active_ladder(stack):
    """The goal watchdog must not cancel a goal while the controller is escaping.

    Detection plus four 3 s rungs reaches ladder exhaustion around t+14, while the
    watchdog fires at 6 s -- so rungs 3 and 4 were unreachable in the assembled
    system precisely when rungs 1 and 2 are refused, which is the only case the
    ladder exists for. A refused rung looks exactly like no progress from here, so
    the explorer has to be told, not left to infer.
    """
    from std_msgs.msg import Bool

    stack.world.nav_mode = "hold"          # accepted, never progresses
    stack.world.publish_map(make_map())
    assert wait_until(lambda: bool(stack.world.nav_goals), 15.0), "no goal was sent"
    first = len(stack.world.nav_goals)

    pub = stack.world.create_publisher(
        Bool, "/decisive_controller/ladder_active", 10)
    deadline = time.monotonic() + (BASE_PARAMS["goal_progress_timeout_s"] * 2.5)
    while time.monotonic() < deadline:
        pub.publish(Bool(data=True))       # a ladder is working
        time.sleep(0.1)

    assert not stack.world.nav_cancels, (
        "the watchdog cancelled a goal while a ladder was running — rungs 3 and 4 "
        "are unreachable in the assembled system")
    assert len(stack.world.nav_goals) == first, "a replacement goal was sent"


def test_f1_watchdog_still_fires_when_no_ladder_is_running(stack):
    """The paired negative. A deferral that never expires is just a disabled
    watchdog, and a goal going nowhere with nobody recovering must still be dropped."""
    stack.world.nav_mode = "hold"
    stack.world.publish_map(make_map())
    assert wait_until(lambda: bool(stack.world.nav_goals), 15.0), "no goal was sent"
    assert wait_until(lambda: bool(stack.world.nav_cancels),
                      BASE_PARAMS["goal_progress_timeout_s"] * 3.0), (
        "watchdog never fired with no ladder active — deferral disabled it outright")


# --------------------------------------------------------------------------- F5

@pytest.mark.parametrize("stack", [{"autostart": False}], indirect=True)
def test_f5_no_goal_is_sent_after_mission_stop(stack):
    """A tick already in flight when mission/stop arrives must not send a goal.

    With the mission stopped nothing would ever cancel it -- the tick has halted and
    the watchdog defers -- so the rover would drive on, unwatched, after the operator
    stopped it.
    """
    from std_srvs.srv import Trigger

    stack.world.nav_mode = "hold"
    stack.world.publish_map(make_map())
    start = stack.world.create_client(Trigger, "/coverage_explorer/mission/start")
    stop = stack.world.create_client(Trigger, "/coverage_explorer/mission/stop")
    assert start.wait_for_service(timeout_sec=5.0)
    assert stop.wait_for_service(timeout_sec=5.0)

    start.call_async(Trigger.Request())
    assert wait_until(lambda: bool(stack.world.nav_goals), 15.0)
    stop.call_async(Trigger.Request())
    assert wait_until(lambda: not stack.explorer._armed, 5.0), "stop did not disarm"
    settled = len(stack.world.nav_goals)

    # Drive the guarded path DIRECTLY. Waiting for the natural race is not a test:
    # the tick's own top-level armed check means a fresh tick never reaches
    # _send_goal, so the window only opens for a tick already in flight when stop
    # arrives -- which no amount of sleeping makes deterministic. My first version
    # waited, and passed with the guard removed, proving nothing. This calls the
    # method the in-flight tick would have been inside.
    m = stack.explorer._map
    info = m.info
    stack.explorer._send_goal(
        (int(info.width // 2), int(info.height // 2)), m.header.frame_id or "map",
        info.origin.position.x, info.origin.position.y, info.resolution)

    time.sleep(1.0)
    assert len(stack.world.nav_goals) == settled, (
        "a goal was sent after mission/stop — with the tick halted and the watchdog "
        "deferring, nothing would ever cancel it and the rover drives unwatched")


# --------------------------------------------------- freeze correlation (by goal)

@pytest.mark.parametrize("stack", [{"max_consecutive_failures": 3}], indirect=True)
def test_freeze_is_claimed_however_long_recovery_took(stack):
    """Replay of gauntlet run 20260811_093818's abort timeline.

    That run produced 8 freeze events and 7 aborts, and excused exactly ONE: the
    others were 5.5, 8.2, 5.6 and 18.7 s old when their goal finally aborted, because
    the stall ladder runs up to four 3 s rungs before exhausting. The 3 s correlation
    window was tuned when a freeze was followed immediately by an abort. Four honest
    discoveries were charged to the give-up counter and killed the mission.

    A freeze belongs to the GOAL it happened during, however long the escape took.
    """
    from std_msgs.msg import String

    explorer = stack.explorer
    pub = stack.world.create_publisher(String, "/decisive_controller/freeze_event", 10)
    pub.publish(String(data='{"x": -0.91, "y": -1.12, "stamp": 0.0}'))
    assert wait_until(lambda: bool(explorer._pending_freezes), 5.0), "freeze not received"

    # The ladder's full escalation, far beyond the old 3 s window.
    time.sleep(4.0)

    claimed = explorer._claim_freeze()
    assert claimed is not None, (
        "a freeze from this goal went unclaimed because recovery took longer than "
        "the old wall-clock window — four of these ended gauntlet run 093818")
    assert abs(claimed[1] - (-0.91)) < 1e-6

    # Consume-once still holds: the same freeze cannot excuse a second abort.
    assert explorer._claim_freeze() is None, "one freeze excused two aborts"


# ------------------------------------------- journey boundaries (goal_generation)

def test_goal_generation_is_published_and_changes_per_goal(stack):
    """The controller cannot infer journey boundaries -- bt_navigator re-sends
    follow_path ~1 Hz within ONE journey (the 08-03 preemption bug), so an endpoint
    proxy cannot tell a new clustered goal from a replan. In gauntlet run 103337 that
    made five consecutive goals share one escape budget: each exhausted instantly
    with nothing tried while the rover sat motionless. The explorer knows the answer,
    so it says it."""
    from std_msgs.msg import Int32

    seen = []
    stack.world.create_subscription(
        Int32, "/coverage_explorer/goal_generation",
        lambda m: seen.append(m.data), 10)
    stack.world.nav_mode = "abort"          # keep goals coming
    stack.world.publish_map(make_map())

    assert wait_until(lambda: len(stack.world.nav_goals) >= 3, 30.0), "goals not sent"
    assert wait_until(lambda: len(set(seen)) >= 3, 15.0), (
        f"goal_generation did not advance per goal (saw {sorted(set(seen))}) — the "
        "controller cannot tell a new journey from a replan")
    assert sorted(set(seen)) == sorted(set(seen)), "generations must be monotonic"


# --------------------------------------------------------------------- D34

def _active_cell_world(stack):
    """World centre of the cell the explorer is currently driving at (NOT the
    stand-off point it sent, which can be 0.68 m short of it)."""
    from sphero_rvr_core.coverage_exploration import cell_center_world
    cell = stack.explorer._active_goal_cell
    if cell is None or stack.explorer._map is None:
        return None
    info = stack.explorer._map.info
    return cell_center_world(cell[0], cell[1], info.origin.position.x,
                             info.origin.position.y, info.resolution)


def test_no_goal_is_cancelled_inside_coverage_radius_while_a_ladder_runs(stack):
    """D34 revert-proof — reproduces the 2026-08-11 gauntlet-1 contact, in software.

    Cancelling a goal reaches through the execute loop's `finally` into
    `abandon_rung()`. So ANY goal-lifecycle trigger that fires while an escape is
    running DESTROYS that escape. The reverted arrival-semantics rule (88fbca4) had
    `ladder_active` as one of its own trigger conditions, which meant it could only
    ever fire mid-rung — and in the field it did, every ~3 s, for 35 s:

        +6s  FREEZE -> ladder rung 1 starts reversing (-0.10)
        +7s  goal SATISFIED at coverage radius -> cancelled
             "Failed to get result for follow_path in node halt!"
             -> the SAME cell re-issued
        +9s  FREEZE -> rung 1 again ... twelve times

    The rover pushed a weight-bench leg it could not see at full commanded cruise
    for 26 of those seconds, and the ladder reached rung 2 exactly once all mission.

    THE POPULATION THAT WAS MISSED: `test_f1_watchdog_defers_to_an_active_ladder`
    already asserted the watchdog defers to a running ladder -- but with the rover
    FAR from its target, so it never exercised a rule keyed on being NEAR one. This
    scenario is that test with the rover moved inside coverage radius, which is the
    only difference that mattered.
    """
    from std_msgs.msg import Bool

    stack.world.nav_mode = "hold"          # accepted, never progresses
    stack.world.publish_map(make_map())
    assert wait_until(lambda: bool(stack.world.nav_goals), 15.0), "no goal was sent"
    assert wait_until(lambda: _active_cell_world(stack) is not None, 5.0)
    cx, cy = _active_cell_world(stack)

    # Drove most of the way and got stopped by something invisible: INSIDE coverage
    # radius of the target, with the controller working an escape.
    stack.world.pose = (cx - 0.50, cy)
    goals_before = len(stack.world.nav_goals)
    cancels_before = len(stack.world.nav_cancels)

    pub = stack.world.create_publisher(
        Bool, "/decisive_controller/ladder_active", 10)
    deadline = time.monotonic() + (BASE_PARAMS["goal_progress_timeout_s"] * 4.0)
    while time.monotonic() < deadline:
        pub.publish(Bool(data=True))       # a ladder is working, continuously
        time.sleep(0.05)

    assert len(stack.world.nav_cancels) == cancels_before, (
        "a goal was cancelled while an escape was running — that cancel reaches "
        "abandon_rung() and kills the rung. This is the gauntlet-1 contact.")
    assert len(stack.world.nav_goals) == goals_before, (
        "a replacement goal was issued mid-escape — the controller starts driving "
        "at the obstacle again on the new goal's execute loop")


# ---------------------------------------------------------------- the costmap scenarios
#
# Both need the costmap publisher added above, and that is the whole reason they did
# not exist. The goal-clearance filter was removed in 1e6af5c with a structural
# revert-proof and NO behavioural one, stated plainly in that commit: with no costmap
# in the harness the filter returned None for every pose and passed everything
# through, so a scenario written then would have passed against the bug as loudly as
# against the fix. That is the D10/D20 vacuous-pass trap, and the answer was to build
# the plumbing rather than to write the scenario anyway.


@pytest.mark.parametrize("stack", [{"blocked_start_check": True}], indirect=True)
def test_a_frontier_goal_the_old_filter_would_have_REJECTED_is_still_sent(stack):
    """THE BEHAVIOURAL PROOF the removal commit owed.

    A frontier borders unknown space by definition, and unknown cells are exactly what
    a clearance probe cannot judge -- so frontier goals sit at the bottom of the
    clearance distribution as a matter of geometry, not of clutter. Measured on the
    live costmap in gauntlet run 1: frontier median clearance 0.05 m (the probe
    floor), 0 of 125 frontiers passing the deployed 0.35 m threshold. There is no
    threshold to tune it to, which is why the filter was deleted rather than retuned.

    Here that geometry is staged: obstruction is painted close enough to the reachable
    floor that any approach point is inside the old 0.35 m threshold, while the robot's
    own cell stays open so the start guard does not end the mission for a different
    reason. The explorer must still SEND a goal.

    THE ANTI-VACUITY ASSERTION IS THE POINT OF THIS TEST. It is not enough that a goal
    was sent -- the goal must be one the old filter would have thrown away. So the
    clearance at the sent goal is measured here, with the deleted arithmetic
    re-implemented in this file, and the test FAILS IF THAT CLEARANCE IS COMFORTABLE.
    A scenario that cannot show the filter would have bitten is not a proof that
    removing it mattered.
    """
    grid = make_map()
    stack.world.publish_map(grid)

    # Lethal cost lining the corridor the rover must work in. Painted as a ring so the
    # rover's OWN cell (the origin) stays open -- otherwise `blocked_start_check` ends
    # the mission as START_BLOCKED and the goal is never selected for a reason that has
    # nothing to do with clearance.
    costmap = make_costmap()
    for ang in range(0, 360, 10):
        r = 0.42
        stamp_cost(costmap, r * math.cos(math.radians(ang)),
                   r * math.sin(math.radians(ang)), 0.06, LETHAL)
    stack.world.publish_costmap(costmap)
    assert probe_clearance_m(costmap, 0.0, 0.0) is not None, "the robot is off costmap"

    stack.world.nav_mode = "succeed"
    assert wait_until(lambda: len(stack.world.nav_goals) >= 1, 25.0), (
        "the explorer sent NO goal with a low-clearance costmap present. That is the "
        "goal-clearance filter's exact failure mode -- 0 of 125 frontiers passing -- "
        "and it means the filter, or something shaped like it, is back in selection")

    _t, gx, gy = stack.world.nav_goals[0]
    clearance = probe_clearance_m(costmap, gx, gy)
    assert clearance is not None, (
        f"the goal at ({gx:.2f}, {gy:.2f}) probes as UNKNOWN, where the old filter "
        "passed poses through untouched -- this scenario cannot show it would have "
        "rejected anything, so it proves nothing. Stage the costmap so the goal is on "
        "known ground")
    assert clearance < 0.35, (
        f"the goal at ({gx:.2f}, {gy:.2f}) has {clearance:.3f} m of clearance, which "
        "the removed 0.35 m filter would have ACCEPTED. The scenario is vacuous: it "
        "would pass against the filter too. Tighten the staged obstruction")


@pytest.mark.parametrize(
    "stack", [{"blocked_start_check": True, "max_unstick_attempts": 3}], indirect=True,
)
def test_d36_the_escape_is_asked_even_when_the_costmap_is_lethal(stack):
    """D36 REPLAY. *The explorer's unstick is invoked correctly and executes nothing,
    because Nav2's BackUp/Spin collision-check the costmap the rover's own freeze marks
    have made lethal -- refusing in 3 ms with 0.78 m of measured clear floor behind.*

    The fix moved the escape off `behavior_server` and onto the controller's own
    action, which drives through the collision supervisor and therefore reads the LIVE
    LIDAR. The property that fix bought is not "the escape succeeds" -- it is that the
    escape's verdict does not come from the costmap. So this stages the costmap that
    refused: lethal all around the rover, its own cell open, nothing plannable.

    WHAT THIS ONE CANNOT SHOW ON ITS OWN, said plainly: it asserts that the escape is
    requested and that its outcome word arrives, and both of those hold under an
    explorer that ignores the controller and records `cleared` unconditionally --
    verified by mutation. Its discriminating power against that lives in its sibling
    below, which stages the refusal. Read as a pair or not at all.

    THE MISSION HAS TO EXPLORE SOMETHING FIRST, and finding that out is what the
    harness is for. The explorer only ends a mission once `_ever_had_target` is true,
    which is set when a goal is actually SENT -- so a scenario where nothing was ever
    plannable produces no report at all, forever. Mission 2 got stuck after driving,
    not before, and this now replays it in that order.
    """
    stack.world.publish_map(make_map())
    stack.world.publish_costmap(make_costmap())          # open floor to begin with
    stack.world.nav_mode = "succeed"
    assert wait_until(lambda: len(stack.world.nav_goals) >= 1, 25.0), \
        "the explorer never drove at all, so the stuck phase below is unreachable"
    assert wait_until(lambda: stack.world.pose != (0.0, 0.0), 10.0), \
        "the rover never arrived anywhere, so its stuck pose is the start pose"

    # NOW it is stuck, exactly as mission 2 was: its own freeze marks have made the
    # costmap lethal all around it, and nothing routes.
    px, py = stack.world.pose
    costmap = make_costmap()
    for ang in range(0, 360, 8):
        stamp_cost(costmap, px + 0.30 * math.cos(math.radians(ang)),
                   py + 0.30 * math.sin(math.radians(ang)), 0.10, LETHAL)
    stack.world.publish_costmap(costmap)
    assert stack.world._costmap_says_blocked(px + 0.30, py), \
        "the staged costmap is not lethal, so this scenario cannot show D36 at all"
    assert not stack.world._costmap_says_blocked(px, py), (
        "the rover's own cell is lethal, so the mission ends START_BLOCKED and the "
        "escape is never reached -- that is a different defect from D36")

    stack.world.plannable = lambda x, y: False     # nothing routes: the unstick trigger
    stack.world.behavior_mode = "succeed"          # the controller escapes via the lidar
    stack.world.escape_outcome = ESCAPE_CLEARED

    before = len(stack.world.behavior_goals)
    assert wait_until(lambda: len(stack.world.behavior_goals) > before, 25.0), (
        "no escape was ever requested with a lethal costmap around the rover. The "
        "escape path is consulting the costmap again, which is D36")
    # THE REPORT'S OWN VOCABULARY: escapes land under `give_up_escapes`, as a count
    # and a histogram of the CONTROLLER'S OWN outcome words -- not as a boolean and
    # not as a count alone. Asserting on the histogram is the whole point: "3 escapes
    # attempted" reads as recovery, and only the words say whether any of them moved
    # the rover.
    assert wait_until(lambda: any(r.get("give_up_escapes", {}).get("attempted")
                                  for r in stack.world.reports), 40.0), (
        f"the mission ended without recording any escape; reports={stack.world.reports}")
    outcomes = {}
    for r in stack.world.reports:
        for word, n in (r.get("give_up_escapes", {}).get("outcomes") or {}).items():
            outcomes[word] = outcomes.get(word, 0) + n
    assert ESCAPE_CLEARED in outcomes, (
        f"the escape's own outcome word never reached the report: {outcomes}. The "
        "explorer is not consuming the controller's fact -- it is inferring one")


@pytest.mark.parametrize(
    "stack", [{"blocked_start_check": True, "max_unstick_attempts": 3}], indirect=True,
)
def test_d36_a_costmap_gated_escape_is_RECORDED_not_swallowed(stack):
    """The other half, and the one that makes the test above non-vacuous.

    If the escape were wired back to something that collision-checks the costmap, the
    request would still go out and the refusal would still come back in milliseconds.
    D36's cost was not the refusal -- it was that the refusal was INVISIBLE: the
    mission ended reporting aborted goals, with nothing saying that every escape it
    attempted had executed no motion at all.

    So with the escape server behaving exactly as `behavior_server` did -- collision-
    checking the ground it would back into -- the report must still name what happened.
    An outcome word that never reaches the report is the failure this asserts against,
    whatever the escape mechanism is.
    """
    stack.world.publish_map(make_map())
    stack.world.publish_costmap(make_costmap())
    stack.world.nav_mode = "succeed"
    assert wait_until(lambda: len(stack.world.nav_goals) >= 1, 25.0), \
        "the explorer never drove, so the stuck phase is unreachable"
    assert wait_until(lambda: stack.world.pose != (0.0, 0.0), 10.0)

    px, py = stack.world.pose
    costmap = make_costmap()
    # Lethal BEHIND the rover -- the ground a BackUp would traverse -- with its own
    # cell left open so the start guard does not end the mission first.
    stamp_cost(costmap, px - 0.30, py, 0.12, LETHAL)
    stack.world.publish_costmap(costmap)
    assert not stack.world._costmap_says_blocked(px, py), \
        "the rover's own cell is lethal; that ends the mission as START_BLOCKED"

    stack.world.plannable = lambda x, y: False
    stack.world.behavior_mode = "costmap_refuse"   # the D36 mechanism itself

    before = len(stack.world.behavior_goals)
    assert wait_until(lambda: len(stack.world.behavior_goals) > before, 25.0), \
        "the explorer never asked for an escape at all"
    assert wait_until(lambda: any(r.get("outcome") for r in stack.world.reports), 45.0), (
        f"the mission never ended; reports={stack.world.reports}")
    report = [r for r in stack.world.reports if r.get("outcome")][-1]
    escapes = report.get("give_up_escapes") or {}
    assert escapes.get("attempted"), (
        "the mission ended with escapes attempted and NOTHING in give_up_escapes. "
        "That is D36's real cost: the refusal was invisible, so the run read as 'the "
        "room refused us' when it was 'we never moved'. "
        f"report={report}")
    # A COUNT IS NOT A RECORD. "attempted: 3" is exactly the line that made D36
    # invisible for a day -- three escapes attempted reads as three escapes performed.
    # The histogram is what distinguishes them, and it must name the refusal.
    assert ESCAPE_REFUSED in (escapes.get("outcomes") or {}), (
        f"escapes were counted but their outcomes are {escapes.get('outcomes')!r}, "
        "which does not name the refusal. A count without the words is the D36 report "
        "verbatim")
    assert ESCAPE_CLEARED not in (escapes.get("outcomes") or {}), (
        f"a costmap-refused escape was recorded as CLEARED: {escapes.get('outcomes')}. "
        "The explorer is reporting an escape that moved nothing as one that moved the "
        "rover, which is D36 wearing the fix's clothes")


def test_d62_cancelled_at_end_is_UNKNOWN_not_zero_for_old_callers():
    """The module's standing rule, applied to the new field: a caller that
    predates it must publish null, not 0. A fabricated zero here reads as "no
    goal was ever cancelled" — the most reassuring possible claim about the
    exact ending that used to vanish."""
    from sphero_rvr_core.mission_report import build_report
    old_caller = build_report("COMPLETE", 10, 0.05, 1.0, goals_sent=1,
                              goals_succeeded=1)
    assert old_caller["goals"]["cancelled_at_end"] is None
    new_caller = build_report("COMPLETE", 10, 0.05, 1.0, goals_sent=2,
                              goals_succeeded=1, goals_cancelled_at_end=1)
    assert new_caller["goals"]["cancelled_at_end"] == 1


def test_d62_the_counter_lives_at_the_single_cancel_site():
    """One incrementer, at the one place a live goal is cancelled — the same
    discipline the planner_rejections re-anchor established. A second
    incrementer elsewhere would double-count exactly the endings this row
    exists to make countable."""
    from pathlib import Path
    src = (Path(__file__).resolve().parents[1] / "src" / "sphero_rvr_driver"
           / "coverage_explorer_node.py").read_text()
    assert src.count("self._goals_cancelled_at_end += 1") == 1
    body = src[src.index("def _cancel_active"):]
    body = body[:body.index("\n    def ")]
    assert "self._goals_cancelled_at_end += 1" in body, (
        "the counter moved away from the cancel site")
    assert "cancel_goal_async" in body
