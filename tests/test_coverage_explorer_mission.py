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

import pytest

rclpy = pytest.importorskip("rclpy")

from action_msgs.msg import GoalStatus  # noqa: E402
from geometry_msgs.msg import PoseStamped, TransformStamped  # noqa: E402
from nav2_msgs.action import BackUp, ComputePathToPose, NavigateToPose, Spin  # noqa: E402
from nav_msgs.msg import OccupancyGrid  # noqa: E402
from rclpy.action import ActionServer, CancelResponse, GoalResponse  # noqa: E402
from rclpy.callback_groups import ReentrantCallbackGroup  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from tf2_ros import TransformBroadcaster  # noqa: E402

from sphero_rvr_core.coverage_exploration import cell_center_world  # noqa: E402
from sphero_rvr_driver.coverage_explorer_node import CoverageExplorerNode  # noqa: E402


RES = 0.05


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
        self.behavior_mode = "hold"            # BackUp/Spin: "hold" | "succeed"
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
        self._backup_srv = ActionServer(
            self, BackUp, "backup", self._make_behavior_execute("backup"),
            goal_callback=lambda req: GoalResponse.ACCEPT,
            cancel_callback=lambda gh: CancelResponse.ACCEPT,
            callback_group=cbg,
        )
        self._spin_srv = ActionServer(
            self, Spin, "spin", self._make_behavior_execute("spin"),
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

        self._freeze_pub = self.create_publisher(
            String, "/decisive_controller/freeze_event", 10)
        self._tf = TransformBroadcaster(self)
        self.create_timer(0.02, self._broadcast_tf, callback_group=cbg)

    def publish_map(self, grid):
        self._map_pub.publish(grid)

    def publish_freeze(self, x=0.0, y=0.0):
        """Stand in for the controller reporting a freeze: the supervisor permitted
        motion and the rover did not move, i.e. an obstacle no sensor can see."""
        self._freeze_pub.publish(String(data='{"x": %f, "y": %f, "stamp": 0.0}' % (x, y)))

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
                if self.behavior_mode == "succeed":
                    time.sleep(0.05)
                    goal_handle.succeed()
                else:  # hold until cancelled: a BackUp against an unseen obstacle
                    while not goal_handle.is_cancel_requested:
                        time.sleep(0.02)
                    self.behavior_cancels.append((time.monotonic(), kind))
                    goal_handle.canceled()
                if kind == "backup":
                    return BackUp.Result()
                return Spin.Result()
            finally:
                rec[1] = time.monotonic()
        return execute


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
    """A held BackUp must be cancelled when its deadline passes, BEFORE Spin is
    sent, and both behaviour goals must carry an explicit time_allowance."""
    stack.world.nav_mode = "hold"
    stack.world.behavior_mode = "hold"
    stack.world.plannable = lambda x, y: False   # force the unstick path
    stack.world.publish_map(make_map())
    assert wait_until(lambda: len(stack.world.behavior_goals) >= 2, 25.0), \
        "second recovery behaviour never sent"
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
        assert allowance == pytest.approx(BASE_PARAMS["unstick_timeout_s"], abs=0.01)


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


@pytest.mark.parametrize(
    "stack", [{"max_consecutive_failures": 3, "max_consecutive_freezes": 3}],
    indirect=True,
)
def test_freeze_marks_reach_the_mission_report(stack):
    """Where the rover could not go is diagnostic gold, and it belongs in the report
    — never written into the saved map, which is the room as SLAM measured it."""
    stack.world.nav_mode = "abort"
    stack.world.publish_map(make_map())

    def keep_freezing():
        for _ in range(200):
            stack.world.publish_freeze(1.5, -0.5)
            time.sleep(0.05)
    threading.Thread(target=keep_freezing, daemon=True).start()

    assert wait_until(lambda: bool(stack.world.reports), 30.0)
    marks = stack.world.reports[-1].get("freeze_marks")
    assert marks, "the report carries no freeze marks"
    assert marks[0]["x"] == pytest.approx(1.5)


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


# --------------------------------------------------- arrival at coverage radius

def _active_cell_world(stack):
    """World centre of the cell the explorer is currently driving at (NOT the
    stand-off point it sent, which can be 0.68 m short of it)."""
    cell = stack.explorer._active_goal_cell
    if cell is None or stack.explorer._map is None:
        return None
    info = stack.explorer._map.info
    return cell_center_world(cell[0], cell[1], info.origin.position.x,
                             info.origin.position.y, info.resolution)


def test_a_blocked_goal_inside_coverage_radius_is_satisfied_not_failed(stack):
    """A goal whose purpose is already achieved must not be counted as a failure.

    Two contracts disagree by 0.65 m: the controller drives to within 0.10 m of the
    path end, the mission is satisfied within coverage_radius_m (0.75) of the target
    cell. Run 114626 spent that gap grinding -- four of its twelve aborts moved less
    than 0.14 m in their final 8 s, parked in front of something with the goal just
    beyond. Every one of those fed the give-up counter that ended the mission.

    Fails against HEAD before this change: the goal runs to the watchdog, aborts,
    and increments _consecutive_failures.
    """
    from std_msgs.msg import Bool

    stack.world.nav_mode = "hold"                 # accepted, never progresses
    stack.world.publish_map(make_map())
    assert wait_until(lambda: bool(stack.world.nav_goals), 15.0), "no goal was sent"
    assert wait_until(lambda: _active_cell_world(stack) is not None, 5.0)
    cx, cy = _active_cell_world(stack)
    goals_before = len(stack.world.nav_goals)

    # Drove most of the way and then got stopped by something: inside coverage
    # radius of the target, with the controller working an escape.
    stack.world.pose = (cx - 0.50, cy)
    pub = stack.world.create_publisher(
        Bool, "/decisive_controller/ladder_active", 10)
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline and stack.explorer._goals_succeeded == 0:
        pub.publish(Bool(data=True))
        time.sleep(0.05)

    assert stack.explorer._goals_succeeded >= 1, (
        "a blocked goal inside coverage radius was not credited as covered")
    assert stack.explorer._consecutive_failures == 0, (
        "a goal that achieved what the mission wanted was counted toward giving up")
    assert len(stack.world.nav_goals) > goals_before or wait_until(
        lambda: len(stack.world.nav_goals) > goals_before, 5.0), (
        "the mission did not move on to another target")


def test_a_blocked_goal_outside_coverage_radius_still_counts_as_a_failure(stack):
    """The paired negative, and the one that keeps the exemption honest. Far from
    the target with nothing recovering, a goal going nowhere is still a failure --
    otherwise this change would quietly disable the give-up counter that exists to
    catch a broken stack."""
    stack.world.nav_mode = "hold"
    stack.world.publish_map(make_map())
    assert wait_until(lambda: bool(stack.world.nav_goals), 15.0), "no goal was sent"
    assert wait_until(lambda: _active_cell_world(stack) is not None, 5.0)
    cx, cy = _active_cell_world(stack)
    assert math.hypot(cx - stack.world.pose[0], cy - stack.world.pose[1]) > 0.75, (
        "scenario precondition: the target must be outside coverage radius")

    assert wait_until(lambda: stack.explorer._consecutive_failures >= 1,
                      BASE_PARAMS["goal_progress_timeout_s"] * 6.0), (
        "a goal that went nowhere, far from its target, was not counted as failed")
    assert stack.explorer._goals_succeeded == 0


def test_a_covered_goal_that_is_still_driving_is_not_cancelled(stack):
    """THE CHURN FALSIFIER, pre-registered in the design note.

    Cancelling as soon as the target counts as covered is a bug this explorer
    already had and already fixed: coverage radius 0.75 m means a target 0.8 m away
    is covered after ~10 cm of driving, and it reissued EVERY tick -- 13 goals in
    15 s, each halting the previous follow_path until bt_navigator could not keep
    up. The satisfied-at-coverage rule must fire only when the drive is ALSO
    blocked, so a rover that is simply driving through its own coverage radius
    keeps its goal.

    Fails against a version whose trigger is 'covered' alone.
    """
    stack.world.nav_mode = "hold"
    stack.world.publish_map(make_map())
    assert wait_until(lambda: bool(stack.world.nav_goals), 15.0), "no goal was sent"
    assert wait_until(lambda: _active_cell_world(stack) is not None, 5.0)
    cx, cy = _active_cell_world(stack)
    goals_before = len(stack.world.nav_goals)
    cancels_before = len(stack.world.nav_cancels)

    # Drive: 0.15 m steps, faster than goal_progress_epsilon_m, straight through the
    # coverage boundary and on toward the target. Never blocked, never stalled.
    x0, y0 = stack.world.pose
    steps = 8
    for i in range(1, steps + 1):
        stack.world.pose = (x0 + (cx - x0) * i / (steps + 1.0),
                            y0 + (cy - y0) * i / (steps + 1.0))
        time.sleep(BASE_PARAMS["goal_progress_timeout_s"] / 3.0)

    assert math.hypot(cx - stack.world.pose[0], cy - stack.world.pose[1]) <= 0.75, (
        "scenario precondition: the rover must end up inside coverage radius")
    assert len(stack.world.nav_cancels) == cancels_before, (
        "a goal was cancelled while the rover was driving normally toward it — "
        "this is the 13-goals-in-15-s churn")
    assert len(stack.world.nav_goals) == goals_before, "a replacement goal was sent"
