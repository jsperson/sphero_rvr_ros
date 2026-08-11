"""Coverage + frontier exploration node.

Drives a coverage mission: keep sending NavigateToPose goals until every reachable
free cell has been both SEEN (no frontiers) and physically APPROACHED (the rover
drove within ``coverage_radius_m`` of it). Unlike explore_lite (which stops when
everything reachable has been *seen*), this also guarantees close physical
approach — what a camera needs for semantic inspection.

It reuses the whole nav stack: it sends NavigateToPose to bt_navigator, which uses
the planner + the decisive controller + the collision brake exactly as explore
does. Run it INSTEAD of explore_lite. Decision logic lives in the pure, tested
:mod:`sphero_rvr_core.coverage_exploration`.

Reachability comes from the PLANNER, not from a local imitation of it. The core
proposes targets nearest-first; this node asks ``ComputePathToPose`` about each and
sends the first that plans. That is one query per rejected candidate — cheap, since
it is the planner the goal would have been handed to anyway — and it removes the
whole apparatus that used to exist to paper over a hand-rolled estimate
disagreeing with the real costmap (map erosion by an inscribed radius, a
navigability-restricted flood, and a blacklist with a radius and a TTL).
"""

import json
import math
import os
import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import BackUp, ComputePathToPose, NavigateToPose, Spin
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
import tf2_ros

from sphero_rvr_core.mission_report import (
    OUTCOME_COMPLETE,
    OUTCOME_BLOCKED_BY_UNSEEN_OBSTACLES,
    OUTCOME_GOALS_KEEP_FAILING,
    OUTCOME_NO_PLANNABLE_TARGETS,
    OUTCOME_START_BLOCKED,
    build_report,
    map_yaml_text,
    occupancy_grid_to_pgm,
)
from sphero_rvr_core.coverage_exploration import (
    CoverageConfig,
    candidate_goals,
    cell_center_world,
    cell_world_grid,
    is_frontier,
    pose_clearance_m,
    robot_start_blocked,
    stamp_coverage,
    world_grid,
)


class CoverageExplorerNode(Node):
    def __init__(self):
        super().__init__("coverage_explorer")
        self.declare_parameter("coverage_radius_m", 0.75)
        self.declare_parameter("min_cluster_cells", 5)
        self.declare_parameter("include_frontiers", True)
        self.declare_parameter("free_threshold", 0)
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("cycle_period_s", 1.0)
        # How many candidates one selection may ask the planner about before giving
        # up for this cycle. Bounds the cost of a selection; the map is re-read next
        # cycle anyway, so "give up for now" is never permanent.
        self.declare_parameter("max_candidates", 12)
        self.declare_parameter("plan_timeout_s", 2.0)
        # Ceiling on ONE selection, so a slow planner cannot stall the explorer for
        # max_candidates * plan_timeout_s. Running out of budget is explicitly not
        # the same as running out of candidates.
        self.declare_parameter("select_budget_s", 6.0)
        # Matches coverage_explorer.yaml. The two drifted once already (YAML 20 vs
        # code 8, the blacklist_ttl_s pattern): production loads the YAML, but any
        # run overriding coverage_params_file silently got the premature-end
        # behaviour back. Change BOTH or neither.
        self.declare_parameter("complete_after_empty_cycles", 20)
        # Goal-progress watchdog: if a goal makes < this much progress in this many
        # seconds, cancel it -- it planned when we asked, but driving it is going
        # nowhere. Suppressed briefly afterwards so the very next selection does not
        # hand back the same cell (the planner would still say yes: planning is not
        # the thing that failed).
        self.declare_parameter("goal_progress_timeout_s", 6.0)
        self.declare_parameter("goal_progress_epsilon_m", 0.10)
        self.declare_parameter("stall_suppress_ttl_s", 45.0)
        self.declare_parameter("stall_suppress_radius_m", 0.2)
        # Give up when goals fail everywhere. Failures at different destinations mean
        # the stack is broken, not the room; grinding on is how one bad controller
        # state turns into 93 goals and a thrashing rover.
        self.declare_parameter("max_consecutive_failures", 5)
        # A FREEZE is not a failure of the stack -- it is the robot discovering an
        # obstacle no sensor can see (the supervisor permitted motion and the rover
        # did not move). Scott's rule: "Hitting an obstacle and stopping should be
        # just another data point... it should not stop the mission." So freezes are
        # exempt from max_consecutive_failures and get their own, larger budget.
        # Without a separate ceiling the exemption would be unbounded and a rover
        # wedged in a corner would freeze forever, which is why this exists.
        self.declare_parameter("max_consecutive_freezes", 5)
        # A goal nearer than this is somewhere the rover already is: it succeeds
        # without moving, the target is still a target next tick, and it gets picked
        # again forever. Never issue one.
        self.declare_parameter("min_goal_distance_m", 0.30)
        # When targets remain but none plan FROM HERE, move and retry rather than
        # ending the mission -- planning is done from the live pose, so this is
        # usually about where the rover is standing, not about the room.
        self.declare_parameter("max_unstick_attempts", 4)
        self.declare_parameter("unstick_backup_m", 0.25)
        self.declare_parameter("unstick_spin_rad", 1.57)
        self.declare_parameter("unstick_timeout_s", 12.0)
        # Start-pose guard. If the robot's OWN costmap cell is at/above inscribed
        # cost, the planner treats the start as in collision and EVERY goal returns
        # "no valid path" while Nav2's motion recoveries are all collision-blocked.
        # Without this the explorer churns goals and blacklists the whole map while
        # going nowhere (observed 2026-08-07: 0.26 m rear clearance, below
        # robot_radius + inflation_radius = 0.30 m, burned a four-minute run).
        self.declare_parameter("blocked_start_check", True)
        # A mission should end with an ANSWER, not a log line someone has to find and
        # interpret. The report is latched JSON on a topic; the map is written next to
        # it. Both are written once, at the moment the mission ends.
        self.declare_parameter("report_topic", "/coverage_explorer/report")
        self.declare_parameter("save_map_on_end", True)
        self.declare_parameter("map_save_dir", os.path.expanduser("~/.ros/missions"))
        self.declare_parameter("costmap_topic", "/global_costmap/costmap")
        # PREVENTION. Measured to INSCRIBED-cost cells, so the boundary this is
        # measured FROM already guarantees the robot fits: inscribed = obstacle +
        # robot_radius + inflation_radius (0.14 + 0.16 = 0.30 m). What remains to
        # justify is manoeuvring margin on top of that, and the honest robot-derived
        # figure is TWO COSTMAP CELLS -- 2 x 0.05 m resolution -- covering grid
        # quantization and localization jitter. Two cells is defensible in any room.
        #
        # It was 0.35 m, and that number double-counted the inflation: I derived it
        # from the same 0.14 + 0.16 = 0.30 m that the inscribed boundary had ALREADY
        # applied, so the filter demanded a second robot-and-inflation clearance
        # beyond the one the costmap encoded. Defensible in the abstract, wrong about
        # the quantity being measured.
        self.declare_parameter("min_goal_clearance_m", 0.10)
        # Defaults to FALSE: bringing the stack up must not commit the robot to
        # moving. Set true only for an unattended run that genuinely wants liftoff
        # at launch.
        self.declare_parameter("autostart", False)
        self.declare_parameter("blocked_hold_s", 5.0)

        self._config = CoverageConfig(
            coverage_radius_m=float(self.get_parameter("coverage_radius_m").value),
            min_cluster_cells=int(self.get_parameter("min_cluster_cells").value),
            include_frontiers=bool(self.get_parameter("include_frontiers").value),
            free_threshold=int(self.get_parameter("free_threshold").value),
            max_candidates=int(self.get_parameter("max_candidates").value),
            # The core must not represent a cluster by a cell this node would then
            # refuse to send a goal to (see min_goal_distance_m) -- that silences
            # the entire cluster (D14).
            min_offer_distance_m=float(self.get_parameter("min_goal_distance_m").value),
        )
        self._goal_progress_timeout_s = float(self.get_parameter("goal_progress_timeout_s").value)
        self._goal_progress_epsilon_m = float(self.get_parameter("goal_progress_epsilon_m").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._stall_ttl_s = float(self.get_parameter("stall_suppress_ttl_s").value)
        self._stall_radius_m = float(self.get_parameter("stall_suppress_radius_m").value)
        self._plan_timeout_s = float(self.get_parameter("plan_timeout_s").value)
        self._select_budget_s = float(self.get_parameter("select_budget_s").value)
        self._max_consecutive_failures = int(
            self.get_parameter("max_consecutive_failures").value)
        self._max_consecutive_freezes = int(
            self.get_parameter("max_consecutive_freezes").value)
        self._min_goal_distance_m = float(
            self.get_parameter("min_goal_distance_m").value)
        self._max_unstick = int(self.get_parameter("max_unstick_attempts").value)
        self._unstick_backup_m = float(self.get_parameter("unstick_backup_m").value)
        self._unstick_spin_rad = float(self.get_parameter("unstick_spin_rad").value)
        self._unstick_timeout_s = float(self.get_parameter("unstick_timeout_s").value)
        self._unstick_attempts = 0
        map_topic = str(self.get_parameter("map_topic").value)

        self._map = None
        self._costmap = None
        self._min_goal_clearance_m = float(
            self.get_parameter("min_goal_clearance_m").value)
        # Reported, never silent: a filter that quietly eats every candidate looks
        # exactly like a room with nothing left to explore.
        self._clearance_rejections = 0
        # Per-CYCLE, so the none-plannable warning can say where this cycle's
        # candidates died. The cumulative counter above cannot: it answers "how many
        # ever" when the question is "which gate killed the set we just had".
        self._clearance_rejected_this_cycle = 0
        # F1: last time the controller reported an active escape. See _ladder_running.
        self._ladder_active_at = None
        self._ladder_signal_max_age_s = 1.0
        self._blocked_logged = False
        self._blocked_until = 0.0  # monotonic; blocked state flickers, so hold it
        self._covered = set()      # world-grid coords the rover has driven within radius of
        # World-grid coords we should stop proposing for a while, with a monotonic
        # expiry. Written whenever a DRIVE failed there -- stalled, aborted or rejected
        # -- because all three mean something stopped the robot that the planner cannot
        # see, and re-asking the planner just gets the same yes. Not written for a
        # planner refusal: that needs no memory, since we re-ask every cycle and the
        # planner is authoritative. Narrow and expiring, unlike the old permanent
        # blacklist that wrote off 73% of free space.
        self._stalled = {}
        self._lock = threading.Lock()
        self._unplannable_last_cycle = 0
        # The tick is long: selection blocks on planner queries (up to
        # select_budget_s) and an unstick blocks on Nav2 behaviours (up to
        # ~2x unstick_timeout_s), while the timer refires every cycle_period_s on a
        # ReentrantCallbackGroup. A re-entered tick during an unstick passes the
        # active-goal checks (nothing is inflight) and runs a full selection: it can
        # send NavigateToPose while BackUp/Spin is still executing -- two Nav2
        # servers driving /cmd_vel, this stack's known way to make a control bug
        # look like a perception bug -- and burn every unstick attempt in seconds.
        # So the WHOLE tick is one critical section: a timer firing while the
        # previous tick still runs is skipped, not queued.
        self._tick_busy = threading.Lock()
        self._mission_start = time.monotonic()
        self._goals_sent = 0
        self._goals_succeeded = 0
        self._goals_aborted = 0
        self._planner_rejections = 0
        self._reported = False
        self._consecutive_failures = 0
        self._consecutive_freezes = 0
        # Unconsumed freeze events, newest last. An event pairs with AT MOST ONE
        # abort: two aborts inside the correlation window of a single freeze must
        # not both be excused, or one discovery would silently forgive an unrelated
        # failure.
        self._pending_freezes = []
        self._freeze_marks = []
        self._active_goal_cell = None
        self._active_goal_handle = None
        self._goal_inflight = False
        self._goal_start_pose = None   # (wx, wy) when the active goal was sent
        self._goal_start_time = None   # time.monotonic() when the active goal was sent
        self._mission_done = False
        # D29: ARMED STATE. Until 2026-08-10 the mission began the instant the node
        # came up, so "bringup gates, THEN go" was not expressible: run 185048's
        # entire 53 s mission ran and died DURING the gate checks, and the operator
        # watched a stopped rover with no idea a mission had happened at all. Launch
        # is no longer liftoff -- a service is.
        self._armed = bool(self.get_parameter("autostart").value)
        self._consecutive_empty = 0
        self._ever_had_target = False
        self._complete_after_empty = int(self.get_parameter("complete_after_empty_cycles").value)

        cbg = ReentrantCallbackGroup()
        map_qos = QoSProfile(depth=1)
        map_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        map_qos.reliability = QoSReliabilityPolicy.RELIABLE
        self.create_subscription(OccupancyGrid, map_topic, self._on_map, map_qos, callback_group=cbg)
        self._blocked_start_check = bool(self.get_parameter("blocked_start_check").value)
        self._blocked_hold_s = float(self.get_parameter("blocked_hold_s").value)
        if self._blocked_start_check:
            self.create_subscription(
                OccupancyGrid, str(self.get_parameter("costmap_topic").value),
                self._on_costmap, map_qos, callback_group=cbg,
            )
        # Freezes are reported by the controller, which is the only component that
        # can see both what it commanded and what the supervisor actually let out.
        self.create_subscription(
            Bool, "/decisive_controller/ladder_active", self._on_ladder_active, 10,
            callback_group=cbg)
        self.create_subscription(
            String, "/decisive_controller/freeze_event", self._on_freeze, 10,
            callback_group=cbg,
        )
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._nav = ActionClient(self, NavigateToPose, "navigate_to_pose", callback_group=cbg)
        self._planner = ActionClient(
            self, ComputePathToPose, "compute_path_to_pose", callback_group=cbg
        )
        self._backup = ActionClient(self, BackUp, "backup", callback_group=cbg)
        self._spin = ActionClient(self, Spin, "spin", callback_group=cbg)
        # Latched: the report is a one-shot event, and anything subscribing after the
        # mission ends (which is most things) must still receive it.
        report_qos = QoSProfile(depth=1)
        report_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        report_qos.reliability = QoSReliabilityPolicy.RELIABLE
        self._report_pub = self.create_publisher(
            String, str(self.get_parameter("report_topic").value), report_qos
        )
        self.create_timer(
            float(self.get_parameter("cycle_period_s").value), self._tick, callback_group=cbg
        )
        # D29: the mission-control surface. The ladder needed one anyway (a recovery
        # regime you cannot stop is not a regime), and it is what makes the run
        # protocol's "gates, THEN go" real rather than aspirational.
        # PRIVATE names ("~/"), so these resolve to /coverage_explorer/mission/*
        # rather than to bare /mission/*. Verified on the Pi, because I documented
        # the namespaced path first and the node registered the bare one: a relative
        # service name resolves against the NAMESPACE, not the node name, and the
        # stack runs in the root namespace. A run protocol whose copy-paste command
        # hangs on a nonexistent service is worse than no protocol.
        self.create_service(Trigger, "~/mission/start", self._on_mission_start,
                            callback_group=cbg)
        self.create_service(Trigger, "~/mission/stop", self._on_mission_stop,
                            callback_group=cbg)
        self.get_logger().info(
            "coverage_explorer ready (coverage + frontier mission) — "
            + ("ARMED, mission running" if self._armed else
               "DISARMED, waiting for mission/start")
        )

    def _on_ladder_active(self, msg):
        if msg.data:
            self._ladder_active_at = time.monotonic()

    def _on_map(self, msg):
        self._map = msg

    def _on_costmap(self, msg):
        self._costmap = msg

    def _start_is_blocked(self, wx, wy):
        """True only when the costmap positively says the robot's own cell is at or
        above inscribed cost. Unknown / no costmap / out of bounds -> False, so a
        missing costmap never silently halts exploration."""
        cm = self._costmap
        if not self._blocked_start_check or cm is None:
            return False
        return robot_start_blocked(
            cm.data, cm.info.width, cm.info.height,
            cm.info.origin.position.x, cm.info.origin.position.y,
            cm.info.resolution, wx, wy,
        ) is True

    def _robot_world(self, frame):
        try:
            tf = self._tf_buffer.lookup_transform(frame, self._base_frame, rclpy.time.Time())
        except Exception:
            return None
        return tf.transform.translation.x, tf.transform.translation.y

    def _robot_yaw(self, frame):
        try:
            q = self._tf_buffer.lookup_transform(
                frame, self._base_frame, rclpy.time.Time()).transform.rotation
        except Exception:
            return None
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _on_mission_start(self, _request, response):
        if self._mission_done:
            # A finished mission stays finished: restarting one in place would
            # silently reuse the coverage set and report a second mission's outcome
            # against the first one's map.
            response.success = False
            response.message = ("mission already finished — restart the node for a "
                                "fresh mission")
            return response
        if self._armed:
            response.success = True
            response.message = "already running"
            return response
        self._armed = True
        self.get_logger().info("mission STARTED by service")
        response.success = True
        response.message = "mission started"
        return response

    def _stop_requested(self):
        """True once mission/stop has disarmed us. Long-running recovery loops poll
        this so they abandon instead of finishing a manoeuvre for a stopped mission."""
        return not self._armed

    def _on_mission_stop(self, _request, response):
        """Disarm and drop any goal in flight.

        Not an emergency stop -- the collision supervisor owns stopping, and this
        does not touch it. This ends the MISSION; the rover coasts to a halt through
        the normal command path.
        """
        was = self._armed
        self._armed = False
        self._cancel_active()
        self.get_logger().info("mission STOPPED by service")
        response.success = True
        response.message = "mission stopped" if was else "was not running"
        return response

    def _tick(self):
        if not self._tick_busy.acquire(blocking=False):
            return
        try:
            self._tick_impl()
        finally:
            self._tick_busy.release()

    def _tick_impl(self):
        if self._mission_done or not self._armed:
            return
        m = self._map
        if m is None:
            return
        self._prune_stalled()
        frame = m.header.frame_id or "map"
        rp = self._robot_world(frame)
        if rp is None:
            return
        wx, wy = rp
        info = m.info
        res = info.resolution
        ox, oy = info.origin.position.x, info.origin.position.y
        w, h = info.width, info.height

        # Cover the swath along the actual path (stamp every cycle at live pose).
        stamp_coverage(self._covered, wx, wy, res, self._config.coverage_radius_m)

        # Guard: if we are wedged inside inflation, issuing goals is futile and
        # actively harmful (it blacklists the map). Say so and wait to be moved.
        if self._start_is_blocked(wx, wy):
            # The cell value flickers around the inscribed threshold, so hold the
            # blocked state briefly. Otherwise a single "clear" tick lets the
            # completion path run while the rover is in fact wedged.
            self._blocked_until = time.monotonic() + self._blocked_hold_s
            self._consecutive_empty = 0
            if not self._blocked_logged:
                self.get_logger().error(
                    "START POSE BLOCKED: the robot's own costmap cell is at/above "
                    "inscribed cost, so the planner cannot plan ANY goal and Nav2's "
                    "recoveries are collision-blocked. Move the rover to open floor "
                    "(needs > robot_radius + inflation_radius clearance, ~0.30 m "
                    "minimum; aim for 0.5 m). Not issuing goals."
                )
                self._blocked_logged = True
                # Wedged is not a mission END -- freeing the rover resumes it -- so
                # this does NOT consume the terminal report latch. But it IS the
                # answer to "what is it doing", and it is actionable, so say it on the
                # topic rather than only in a log nobody is tailing. A later terminal
                # report overwrites it on the latched topic, which is what we want.
                self._report_pub.publish(String(data=json.dumps(build_report(
                    OUTCOME_START_BLOCKED,
                    covered_cells=len(self._covered),
                    resolution=res,
                    duration_s=time.monotonic() - self._mission_start,
                    goals_sent=self._goals_sent,
                    goals_succeeded=self._goals_succeeded,
                    goals_aborted=self._goals_aborted,
                    planner_rejections=self._planner_rejections,
                ))))
            return
        if time.monotonic() < self._blocked_until:
            self._consecutive_empty = 0   # still within the hold: cannot be "done"
            return
        if self._blocked_logged:
            self.get_logger().info("start pose clear again — resuming exploration")
            self._blocked_logged = False

        rcx = int((wx - ox) / res)
        rcy = int((wy - oy) / res)

        # If the current goal's cell is no longer a target (covered en route or its
        # unknown got resolved), cancel it and reselect — don't drive to a spot we
        # already covered.
        with self._lock:
            active_cell = self._active_goal_cell
            inflight = self._goal_inflight
        if active_cell is not None:
            # COMMIT to a goal until it ends. This used to cancel as soon as the target
            # cell stopped being a target -- but coverage_radius_m is 0.75 m, so a
            # target 0.8 m away is marked covered after ~10 cm of driving, and the
            # explorer cancelled and reissued EVERY tick: 13 goals in 15 s, each one
            # halting the previous follow_path. bt_navigator could not keep up
            # ("Failed to get result for follow_path in node halt!") and the goals
            # failed. The explorer was beating itself. Reaching the target is not
            # wasted even once it counts as covered -- it is still where we were going,
            # and arriving is what lets the next selection start from there.
            # The ONLY reason to drop a live goal is that driving it is going nowhere.
            # The watchdog planned-but-no-progress case still applies.
            if self._goal_stalled(wx, wy):
                self.get_logger().warn(
                    f"coverage goal {active_cell} planned but made no progress in "
                    f"{self._goal_progress_timeout_s:.0f}s — dropping it"
                )
                self._cancel_active()
                # A watchdog cancel IS a failed drive and must count as one. Its
                # result status is CANCELED, which the result callback counts as
                # neither ABORTED nor SUCCEEDED -- so without counting it here,
                # stall -> cancel -> suppress -> reselect loops FOREVER: the
                # suppression disc expires after stall_suppress_ttl_s, the empty
                # counter resets on every send, and the give-up counter that exists
                # precisely to stop this thrash never moves. The rover sits nearly
                # still logging "made no progress" every few seconds indefinitely.
                self._note_failure(active_cell)
                if self._mission_done:
                    return  # that was the last straw; _finish already ran
            else:
                return  # making progress -> keep driving, do not reselect
        if inflight:
            return

        candidates = candidate_goals(
            m.data, w, h, ox, oy, res, rcx, rcy, self._covered, set(self._stalled), self._config
        )
        # Selection blocks on planner queries and can outlast the tick period. It
        # cannot re-enter -- the whole tick is serialized by _tick_busy -- which is
        # what keeps two selection loops from racing to send two goals: concurrent
        # goals fighting over one actuator is this stack's known way to produce
        # motion that looks like a perception failure (the 2026-08-03 follow_path
        # preemption bug).
        goal_cell = None
        goal_point = None
        exhausted = False
        self._clearance_rejected_this_cycle = 0
        budget = time.monotonic() + self._select_budget_s
        for cell in candidates:
            if time.monotonic() >= budget:
                # Out of time, not out of candidates -- the rest are unjudged, so
                # this cycle must not count as evidence of "nothing plannable".
                exhausted = True
                self.get_logger().warn(
                    f"selection budget ({self._select_budget_s:.0f}s) spent after "
                    f"{candidates.index(cell)}/{len(candidates)} candidates"
                )
                break
            gx, gy = cell
            gwx, gwy = cell_center_world(gx, gy, ox, oy, res)
            # Try the cell, then progressively closer stand-off points along the
            # line back toward the robot. Demanding the planner reach the cell
            # EXACTLY was too strict and ended missions early: frontier cells sit
            # against unknown space and walls, so they are usually inflated and
            # the planner refuses them (26 refusals then a premature end, first
            # run). The mission does not need the rover ON the cell -- coverage is
            # satisfied within coverage_radius_m of it, and seeing past a frontier
            # only needs proximity. So ask for the nearest point that both plans
            # AND still counts as covering the target.
            for awx, awy in self._approach_points(gwx, gwy, wx, wy):
                # PREVENTION (docs/stall_survival_ladder.md): never send the rover to
                # a pose it will not be able to LEAVE. Run 185048 selected, planned to
                # and reached a pose with 0.22 m on two sides, where the planner then
                # refused every onward goal and every escape was vetoed. Recovery
                # cannot help there -- at that clearance the ladder's rungs are all
                # refused too -- so the cheapest fix by far is not to go.
                #
                # A pose we cannot JUDGE (unknown, off-map, or no costmap at all ->
                # None) passes through to the planner rather than being rejected.
                # I built it the other way first, reasoning that "cannot tell" should
                # fail safe -- and the rehearsal suite showed that rejecting on None
                # makes the explorer refuse EVERY candidate whenever the costmap is
                # absent or the goal sits near unknown space, which is where coverage
                # goals live by construction. An inert explorer is not a safe one.
                # The planner still gates every candidate below, so this filter only
                # ever REMOVES poses it can positively prove are too tight.
                clearance = self._pose_clearance(awx, awy)
                if clearance is not None and clearance < self._min_goal_clearance_m:
                    self._clearance_rejections += 1
                    self._clearance_rejected_this_cycle += 1
                    continue
                if self._planner_can_reach(awx, awy, frame):
                    goal_cell, goal_point = cell, (awx, awy)
                    break
            if goal_cell is not None:
                break
        self._unplannable_last_cycle = (
            len(candidates) if goal_cell is None else candidates.index(goal_cell)
        )
        self._planner_rejections += self._unplannable_last_cycle
        if goal_cell is None and exhausted:
            return  # inconclusive cycle: leave the completion counter alone

        if goal_cell is None and candidates and self._unstick_attempts < self._max_unstick:
            # Targets remain but the planner will not route to any of them. Nearly
            # always that is about where the ROVER is standing, not about the room --
            # planning is done from the live pose, so a rover parked in inflation
            # fails every goal no matter how open the map is. Measured after one such
            # stop: the sole remaining target replied PATH in 7 ms once the rover had
            # moved.
            #
            # So get unstuck and carry on. Ending the mission here is pretending it is
            # done, which is the whole failure this outcome exists to avoid.
            self._unstick_attempts += 1
            self.get_logger().warn(
                f"{len(candidates)} target(s) left but none plannable from here "
                f"({self._clearance_rejected_this_cycle} rejected on CLEARANCE, "
                f"{self._unplannable_last_cycle} on the PLANNER) — "
                f"unsticking (attempt {self._unstick_attempts}/{self._max_unstick})"
            )
            self._unstick(toward=self._cell_world(candidates[0], ox, oy, res))
            self._consecutive_empty = 0
            return

        if goal_cell is None:
            # Debounce: don't latch "complete" on a transient/startup empty. Only
            # finish after N consecutive empties AND once it has actually explored.
            self._consecutive_empty += 1
            if self._ever_had_target and self._consecutive_empty >= self._complete_after_empty:
                self._mission_done = True
                if candidates:
                    # There IS uncovered ground the rover wants; the planner just
                    # will not route to any of it. That is not a finished mission and
                    # must never be reported as one -- it is the honest form of the
                    # 2026-08-07 false COMPLETE, and now it is a structural
                    # distinction (targets exist vs targets don't) rather than an
                    # inference from how much got blacklisted.
                    self.get_logger().warn(
                        f"exploration ENDED with {len(candidates)} target(s) still "
                        f"wanted but NONE plannable — {len(self._covered)} cells "
                        "covered, so COVERAGE IS INCOMPLETE. Where the candidates "
                        f"died THIS cycle: {self._clearance_rejected_this_cycle} "
                        f"rejected on CLEARANCE (min_goal_clearance_m="
                        f"{self._min_goal_clearance_m:.2f}), "
                        f"{self._unplannable_last_cycle} refused by the PLANNER. "
                        "Both gates thin the same set and the clearance one runs "
                        "FIRST, so a candidate it drops never reaches the planner to "
                        "be counted — without this split the log cannot say which "
                        "gate ended the mission."
                    )
                    self._finish(OUTCOME_NO_PLANNABLE_TARGETS, res, len(candidates))
                else:
                    self.get_logger().info(
                        f"coverage+frontier mission COMPLETE — {len(self._covered)} cells "
                        "covered; all reachable free space seen and within coverage radius"
                    )
                    self._finish(OUTCOME_COMPLETE, res, 0)
            return
        self._consecutive_empty = 0
        self._ever_had_target = True
        self._goal_start_pose = (wx, wy)
        self._goal_start_time = time.monotonic()
        self._send_goal(goal_cell, frame, ox, oy, res, goal_point)

    def _finish(self, outcome, resolution, remaining):
        """End the mission with an artifact: a latched JSON report, and the map on
        disk. Called once; a second call is ignored so a re-entered terminal branch
        cannot overwrite the record of what actually happened."""
        with self._lock:
            # Check-and-set atomically: _finish can be reached from the tick thread
            # and an action-callback thread in the same instant, and the report must
            # be written exactly once.
            if self._reported:
                return
            self._reported = True
        # A finished mission must not leave a goal driving. _mission_done is set from
        # action-callback threads while a tick may be mid-flight past its own check,
        # so without this cancel the goal that tick issued would run UNWATCHED: later
        # ticks return early at the top, the progress watchdog never runs again, and
        # the report says the mission is over while the rover is still moving.
        self._cancel_active()
        files = self._save_map() if bool(self.get_parameter("save_map_on_end").value) else []
        report = build_report(
            outcome,
            covered_cells=len(self._covered),
            resolution=resolution,
            duration_s=time.monotonic() - self._mission_start,
            goals_sent=self._goals_sent,
            goals_succeeded=self._goals_succeeded,
            goals_aborted=self._goals_aborted,
            planner_rejections=self._planner_rejections,
            remaining_candidates=remaining,
            map_files=files,
            freeze_marks=list(self._freeze_marks),
        )
        self._report_pub.publish(String(data=json.dumps(report)))
        self.get_logger().info(f"mission report -> {json.dumps(report)}")

    def _save_map(self):
        """Write the current /map as a map_server PGM+YAML pair.

        Serialized straight from the OccupancyGrid rather than by calling
        /map_saver/save_map, so it does not depend on another node being alive at the
        one moment it matters. Failure is logged and reported, never raised: a map
        that cannot be written must not also destroy the report explaining the run.
        """
        m = self._map
        if m is None:
            return []
        try:
            # expanduser the PARAMETER, not just the default: YAML wins over the code
            # default, and a literal "~/..." from a config file would silently create a
            # directory named "~" in the cwd rather than write to $HOME.
            d = os.path.expanduser(str(self.get_parameter("map_save_dir").value))
            os.makedirs(d, exist_ok=True)
            stem = f"mission_{time.strftime('%Y%m%d_%H%M%S')}"
            pgm_path = os.path.join(d, stem + ".pgm")
            yaml_path = os.path.join(d, stem + ".yaml")
            with open(pgm_path, "wb") as f:
                f.write(occupancy_grid_to_pgm(m.data, m.info.width, m.info.height))
            with open(yaml_path, "w") as f:
                f.write(map_yaml_text(
                    os.path.basename(pgm_path), m.info.resolution,
                    m.info.origin.position.x, m.info.origin.position.y,
                ))
            self.get_logger().info(f"map saved -> {pgm_path}")
            return [pgm_path, yaml_path]
        except Exception as exc:
            self.get_logger().error(f"map save FAILED: {exc}")
            return []

    def _await(self, future, deadline, stop_aware=False):
        """Block this callback until `future` resolves or `deadline` passes.

        Safe because the node runs on a MultiThreadedExecutor with a reentrant
        callback group, so another thread services the action response while this
        one waits. Returns the result, or None on timeout.

        N4. `stop_aware` makes the wait abandon on mission/stop. It already slices at
        10 ms, but it only ever LOOKED at the deadline -- so a recovery behaviour
        could hold this thread for the full unstick_timeout_s (12 s deployed) after
        the operator stopped the mission, which is precisely when someone is walking
        toward the robot. Off by default: a planner query should still finish, since
        it commands no motion.
        """
        while not future.done():
            if time.monotonic() >= deadline:
                return None
            if stop_aware and self._stop_requested():
                return None
            time.sleep(0.01)
        return future.result()

    def _approach_points(self, tx, ty, rx, ry):
        """The target, then stand-off points pulled back toward the robot.

        Stops short of the target by fractions of coverage_radius_m, so every point
        offered still lands within coverage range of it -- reaching any of them counts
        as covering the target, and none of them is a different errand.
        """
        d = math.hypot(tx - rx, ty - ry)
        min_d = self._min_goal_distance_m
        if d >= min_d:
            yield (tx, ty)
        if d < 1e-3:
            return
        ux, uy = (rx - tx) / d, (ry - ty) / d      # unit vector target -> robot
        for frac in (0.5, 0.9):
            back = self._config.coverage_radius_m * frac
            # Every offered point must be somewhere the rover is NOT. A stand-off
            # 0.68 m back from a target 0.70 m away is the rover's own position: the
            # goal succeeds instantly without moving, the cell is still a target next
            # tick, and it is picked again forever. Observed doing exactly that.
            if back < d - min_d:
                yield (tx + ux * back, ty + uy * back)

    def _planner_can_reach(self, wx, wy, frame):
        """Ask the PLANNER whether it can route to (wx, wy) from where we are now.

        This is the authority on reachability. A path with poses means go; anything
        else -- refused, timed out, empty path -- means try the next candidate. It
        fails CLOSED for a single candidate (we skip it) but never for the mission:
        candidates are re-proposed and re-asked next cycle, so a planner hiccup
        costs one cycle, not a permanently written-off area.
        """
        if not self._planner.server_is_ready():
            if not self._planner.wait_for_server(timeout_sec=1.0):
                self.get_logger().warn(
                    "compute_path_to_pose unavailable — cannot verify reachability"
                )
                return False
        goal = ComputePathToPose.Goal()
        goal.goal = PoseStamped()
        goal.goal.header.frame_id = frame
        goal.goal.pose.position.x = float(wx)
        goal.goal.pose.position.y = float(wy)
        goal.goal.pose.orientation.w = 1.0
        goal.use_start = False  # plan from the robot's live pose
        deadline = time.monotonic() + self._plan_timeout_s
        handle = self._await(self._planner.send_goal_async(goal), deadline)
        if handle is None or not handle.accepted:
            return False
        result = self._await(handle.get_result_async(), deadline)
        if result is None:
            # Same rule as the unstick behaviours: a query we stop waiting for must
            # also stop running, or slow planner queries pile up server-side.
            handle.cancel_goal_async()
            return False
        return len(result.result.path.poses) > 0

    def _ladder_running(self):
        """True while the controller is actively working an escape.

        Freshness-bounded on purpose: if the controller dies mid-rung the signal goes
        stale and the watchdog resumes, rather than the mission hanging forever on a
        promise from a process that is gone.
        """
        if self._ladder_active_at is None:
            return False
        return (time.monotonic() - self._ladder_active_at) <= self._ladder_signal_max_age_s

    def _goal_stalled(self, wx, wy):
        """True if the active goal has made < epsilon progress for > timeout. The
        progress reference resets whenever the rover advances, so this fires only on
        a sustained no-progress stretch (planner churning), not a slow-but-moving
        drive."""
        if self._goal_start_pose is None or self._goal_start_time is None:
            return False
        if self._ladder_running():
            # DEFER. The controller owns recovery, and a rung that is refused looks
            # exactly like no progress from here -- so firing the watchdog would
            # cancel the goal at 6 s and count a failure while the escape sequence
            # was still running, making rungs 3 and 4 unreachable in the assembled
            # system. Hold the progress clock too, so the ladder's own time does not
            # count against the goal once it finishes.
            self._goal_start_time = time.monotonic()
            return False
        moved = math.hypot(wx - self._goal_start_pose[0], wy - self._goal_start_pose[1])
        if moved >= self._goal_progress_epsilon_m:
            self._goal_start_pose = (wx, wy)
            self._goal_start_time = time.monotonic()
            return False
        return (time.monotonic() - self._goal_start_time) >= self._goal_progress_timeout_s

    def _still_target(self, m, w, h, ox, oy, res, cell):
        gx, gy = cell
        if not (0 <= gx < w and 0 <= gy < h):
            return False
        v = m.data[gy * w + gx]
        if not (0 <= v <= self._config.free_threshold):
            return False
        wg = cell_world_grid(gx, gy, ox, oy, res)
        if wg in self._stalled:
            return False
        if wg not in self._covered:
            return True
        return self._config.include_frontiers and is_frontier(
            m.data, w, h, gx, gy, self._config.free_threshold
        )

    def _send_goal(self, cell, frame, ox, oy, res, point=None):
        """Drive to `point` (the reachable stand-off we found) while still tracking
        `cell` as the target being covered. They differ whenever the cell itself was
        not directly plannable."""
        if not self._nav.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn("navigate_to_pose action server not available yet")
            return
        gx, gy = cell
        cwx, cwy = cell_center_world(gx, gy, ox, oy, res)
        wx, wy = point if point is not None else (cwx, cwy)
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = frame
        goal.pose.pose.position.x = float(wx)
        goal.pose.pose.position.y = float(wy)
        goal.pose.pose.orientation.w = 1.0
        with self._lock:
            # Checked HERE, under the lock, not only at the top of the tick:
            # _mission_done is set from action-callback threads, and a tick that was
            # already past its top check must not issue a goal for a mission that
            # has since reported itself over.
            # F5. Disarm is checked HERE too, not only at the top of the tick: a tick
            # already in flight when mission/stop arrives would otherwise send a fresh
            # goal for a stopped mission -- and with the watchdog also disarmed,
            # nothing would ever cancel it. The rover would drive on, unwatched, after
            # the operator stopped it. Same reasoning as _mission_done directly below.
            if self._mission_done or not self._armed:
                return
            self._goal_inflight = True
            self._active_goal_cell = cell
            self._goals_sent += 1
        offset = math.hypot(wx - cwx, wy - cwy)
        via = f" (standing off {offset:.2f} m)" if offset > 0.01 else ""
        self.get_logger().info(
            f"coverage goal -> cell {cell} world ({wx:.2f},{wy:.2f}){via}; "
            f"covered={len(self._covered)} planner-rejected={self._unplannable_last_cycle} "
            f"clearance-rejected={self._clearance_rejections} "
            f"stalled={len(self._stalled)}"
        )
        self._nav.send_goal_async(goal).add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        handle = future.result()
        with self._lock:
            self._goal_inflight = False
            disarmed = not self._armed
        if disarmed and handle.accepted:
            # N5. Clear the goal bookkeeping too. Leaving _active_goal_cell set meant
            # the next mission start inherited a phantom active goal, counted one
            # spurious failure against the give-up budget, and suppressed a perfectly
            # good cell for 45 s -- all from a goal that was cancelled before it ever
            # drove.
            with self._lock:
                self._active_goal_cell = None
                self._active_goal_handle = None
            self._goal_start_pose = None
            self._goal_start_time = None
            # The goal was in flight across a mission/stop and Nav2 accepted it after
            # we disarmed. Nothing else will cancel it -- the watchdog defers to the
            # ladder and the tick has stopped running -- so cancel it here or the
            # rover drives a goal for a mission that is over.
            self.get_logger().info("goal accepted after mission/stop — cancelling")
            handle.cancel_goal_async()
            return
        if not handle.accepted:
            # Same rule as an abort: a refusal we cannot see the reason for must not be
            # retried immediately at the same spot.
            with self._lock:
                cell = self._active_goal_cell
                self._active_goal_cell = None
            self.get_logger().warn(f"coverage goal {cell} REJECTED — suppressing it")
            self._note_failure(cell)
            return
        with self._lock:
            self._active_goal_handle = handle
            done = self._mission_done
        handle.get_result_async().add_done_callback(self._on_goal_result)
        if done:
            # The mission finished between our send and the server accepting it, so
            # _finish's cancel ran before there was a handle to cancel. Close the
            # gap from this side.
            handle.cancel_goal_async()

    def _on_goal_result(self, future):
        status = future.result().status
        with self._lock:
            cell = self._active_goal_cell
            self._active_goal_handle = None
            self._active_goal_cell = None
        if status == GoalStatus.STATUS_ABORTED:
            # An ABORT MUST have a consequence. Removing that was a real regression:
            # on the 2026-08-09 chassis run the explorer sent 93 goals in 127 s and 82
            # aborted, 67 of them to just two cells, because nothing recorded that a
            # drive had already failed there. The planner keeps saying yes -- it cannot
            # see whatever actually stopped the robot -- so "just re-ask the planner"
            # is an infinite retry into a physical obstacle, and each retry re-runs the
            # controller's back-off reflex. That is the back-up-drive-forward-back-up
            # loop Scott watched.
            #
            # The old blacklist was too WIDE and permanent (a 0.3 m disc, forever,
            # 2390 cells = 73% of free space). The fix for that was to narrow it, not
            # to delete it. Same narrow, expiring shape as the stall suppression.
            with self._lock:
                self._goals_aborted += 1
            self.get_logger().warn(
                f"coverage goal {cell} ABORTED — suppressing it for "
                f"{self._stall_ttl_s:.0f}s so we do not drive at it again"
            )
            self._note_failure(cell)
        elif status == GoalStatus.STATUS_SUCCEEDED:
            with self._lock:
                self._goals_succeeded += 1
                self._consecutive_failures = 0
                self._consecutive_freezes = 0
                self._unstick_attempts = 0

    def _unstick(self, toward=None):
        """Physically change where the rover is standing, then let selection retry.

        Uses Nav2's own behaviours rather than driving anything directly: back up a
        little, and if that is refused (rear blocked) spin instead. Both go through
        the collision supervisor like every other motion, so this cannot reverse into
        something. A pose change is what makes previously-unplannable targets
        plannable, which is why this belongs here and not in a recovery BT that only
        runs while a goal is active -- there is no active goal at this point.
        """
        # Back straight out ONCE. After that, TURN -- and turn toward the ground we
        # are actually trying to reach, not a fixed direction. Repeating a straight
        # reverse is what Scott watched it do: "still doing the straight back and
        # straight forward thing... if it gets stuck straight back then pivot to an
        # open area would be better." Backing up alone barely changes what the planner
        # can see; a heading change opens a different set of routes entirely.
        if self._unstick_attempts <= 1:
            order = (
                (self._backup, self._backup_goal(), "back up"),
                (self._spin, self._spin_goal(toward), "turn toward the target"),
            )
        else:
            order = (
                (self._spin, self._spin_goal(toward), "turn toward the target"),
                (self._backup, self._backup_goal(), "back up"),
            )
        for client, goal, what in order:
            if self._mission_done or self._stop_requested():
                # F5. mission/stop must abandon an unstick, not let it finish. Each
                # behaviour can run for unstick_timeout_s, so without this the rover
                # keeps manoeuvring for many seconds after the operator stopped the
                # mission -- which is exactly when someone is walking toward it.
                self.get_logger().info("unstick abandoned — mission stopped")
                return False
            if not client.wait_for_server(timeout_sec=1.0):
                continue
            deadline = time.monotonic() + self._unstick_timeout_s
            handle = self._await(client.send_goal_async(goal), deadline,
                                 stop_aware=True)
            if handle is None or not handle.accepted:
                self.get_logger().info(f"unstick: {what} refused, trying the next")
                continue
            if self._stop_requested():
                # Stopped while this behaviour was accepted and running: cancel it
                # rather than waiting out its deadline.
                handle.cancel_goal_async()
                self.get_logger().info(f"unstick: {what} cancelled — mission stopped")
                return False
            result = self._await(handle.get_result_async(), deadline,
                                 stop_aware=True)
            if result is None:
                # Timed out here does NOT mean finished there. The behaviour is
                # still executing -- a BackUp against something only the supervisor
                # can see never advances and never ends on its own (time_allowance
                # is our only server-side deadline, and it equals this one) -- and
                # sending the next behaviour on top of it means two Nav2 servers
                # driving /cmd_vel at once. Cancel, and give the cancel a moment to
                # actually stop the behaviour before anything else is sent.
                handle.cancel_goal_async()
                self._await(handle.get_result_async(), time.monotonic() + 2.0)
                self.get_logger().info(f"unstick: {what} timed out — cancelled")
                continue
            if result.status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info(f"unstick: {what} done")
                return True
            self.get_logger().info(f"unstick: {what} did not finish, trying the next")
        self.get_logger().warn("unstick: nothing worked from this pose")
        return False

    def _pose_clearance(self, wx, wy):
        """Clearance at a candidate pose, from the GLOBAL costmap.

        Returns None when it cannot be judged. The caller passes those poses to the
        planner rather than rejecting them — see the call site. This docstring said
        the opposite until the rehearsal suite proved rejecting-on-None makes the
        explorer refuse every candidate whenever the costmap is absent.
        """
        m = self._costmap
        if m is None:
            # THE FILTER IS INOPERABLE, which is NOT the same as "this pose is bad".
            # Rejecting on this branch made the explorer refuse every candidate
            # forever whenever the costmap was absent or late -- caught by the
            # rehearsal suite, which runs the real node with no costmap publisher at
            # all, and which sat there reporting "targets left but none plannable"
            # while the map was wide open. Exactly the silent-inertia failure this
            # filter's own commit message warned about.
            #
            # Passing through is safe: the planner still gates every candidate
            # afterwards, so this restores the pre-filter behaviour rather than
            # inventing a weaker one. Loud, because a prevention layer that is not
            # running must never be invisible.
            self._costmap_missing_warned = getattr(
                self, "_costmap_missing_warned", False)
            if not self._costmap_missing_warned:
                self.get_logger().warn(
                    "goal-clearance prevention INACTIVE: no costmap on "
                    f"{self.get_parameter('costmap_topic').value}. Candidates are "
                    "planner-gated only.")
                self._costmap_missing_warned = True
            return None
        return pose_clearance_m(
            m.data, m.info.width, m.info.height,
            m.info.origin.position.x, m.info.origin.position.y,
            m.info.resolution, wx, wy,
            max_probe_m=self._min_goal_clearance_m + 0.25,
        )

    def _cell_world(self, cell, ox, oy, res):
        return cell_center_world(cell[0], cell[1], ox, oy, res)

    def _backup_goal(self):
        g = BackUp.Goal()
        g.target.x = float(self._unstick_backup_m)   # BackUp treats +x as backwards
        g.speed = 0.10
        # Explicit server-side deadline. Left at 0, nav2 applies no time limit and a
        # BackUp that cannot advance runs forever; our client-side _await timeout
        # abandons it but does not stop it.
        g.time_allowance = Duration(seconds=self._unstick_timeout_s).to_msg()
        return g

    def _spin_goal(self, toward=None):
        """Turn toward `toward` (a map-frame point) when we know one, else a fixed
        quarter turn. Turning toward the target we cannot currently plan to is the
        move most likely to open a route to it -- a blind fixed spin is as likely to
        face a wall."""
        g = Spin.Goal()
        g.target_yaw = float(self._unstick_spin_rad)
        g.time_allowance = Duration(seconds=self._unstick_timeout_s).to_msg()
        if toward is not None:
            here = self._robot_world(self._map.header.frame_id or "map") if self._map else None
            yaw = self._robot_yaw(self._map.header.frame_id or "map") if self._map else None
            if here is not None and yaw is not None:
                want = math.atan2(toward[1] - here[1], toward[0] - here[0])
                delta = math.atan2(math.sin(want - yaw), math.cos(want - yaw))
                # Commit to a real turn: a few degrees changes nothing, and Spin has
                # to overcome the drivetrain's breakaway to move at all.
                if abs(delta) >= 0.35:
                    g.target_yaw = float(delta)
        return g

    def _on_freeze(self, msg):
        """A freeze the controller detected. Held until an abort claims it."""
        try:
            data = json.loads(msg.data)
            x, y = float(data["x"]), float(data["y"])
        except Exception:
            return
        with self._lock:
            # Tagged with the GOAL it arrived during, not just a timestamp. See
            # _claim_freeze for why the wall clock was the wrong key.
            self._pending_freezes.append((self._goals_sent, x, y))
            self._freeze_marks.append({"x": round(x, 3), "y": round(y, 3)})

    def _claim_freeze(self):
        """Consume the most recent unclaimed freeze FROM THIS GOAL.

        CONSUME-ONCE is still the point: a freeze event pairs with AT MOST ONE abort,
        so one discovery cannot silently forgive an unrelated failure.

        CORRELATED BY GOAL, NOT BY WALL CLOCK. It used to require the freeze to be
        within freeze_correlation_window_s (3.0 s) of the abort, which was true when a
        freeze was followed almost immediately by an abort. The stall ladder changed
        that: it runs up to four rungs at rung_budget_s each before exhausting, so a
        freeze detected at rung 1 is 12+ seconds stale by the time the goal aborts.

        Measured on gauntlet run 20260811_093818: eight freeze events, seven aborts,
        and freeze ages at abort of 0.0, 5.5, 8.2, 5.6 and 18.7 s. Exactly ONE was
        claimed. The other four ticked the give-up counter and ended the mission with
        "5 goals in a row failed, at different places -- this is the stack, not the
        room" -- doubly false, because they failed at the SAME place and the stack had
        correctly identified it as an obstacle every time.

        "During this goal" is what the rule always meant, it cannot be invalidated by
        changing how long recovery takes, and it removes a tuned constant instead of
        re-tuning one.
        """
        with self._lock:
            goal = self._goals_sent
            mine = [f for f in self._pending_freezes if f[0] == goal]
            if not mine:
                return None
            claimed = mine[-1]
            self._pending_freezes.remove(claimed)
            # Freezes from earlier goals are dead: they can never be claimed now, and
            # keeping them would let a stale discovery excuse a later unrelated abort.
            self._pending_freezes = [
                f for f in self._pending_freezes if f[0] >= goal
            ]
            return claimed

    def _note_failure(self, cell):
        """Record that driving to `cell` failed: suppress it, and give up entirely if
        goals are failing everywhere.

        The give-up half is the important one. Suppression alone still lets a broken
        stack chew through the entire map one cell at a time -- on 2026-08-09 that was
        93 goals in 127 s with the rover thrashing back and forth, because the
        controller was stuck reporting "boxed in" and would have failed a goal
        anywhere. Repeated failures regardless of destination are a broken stack, not a
        hard room, and the mission should stop and say which.
        """
        self._suppress_cell(cell)

        # DISCOVERY, NOT FAILURE. If the controller reported a freeze just before
        # this abort, the rover met an obstacle no sensor can see. That is a fact
        # about the room, so it must not feed the counter whose job is to detect a
        # broken STACK -- but it gets its own ceiling, because an unbounded exemption
        # would let a rover wedged in a corner freeze forever.
        frozen = self._claim_freeze()
        if frozen is not None:
            with self._lock:
                self._consecutive_freezes += 1
                n_freeze = self._consecutive_freezes
            self.get_logger().warn(
                f"coverage goal {cell} ended in a FREEZE at "
                f"({frozen[1]:.2f},{frozen[2]:.2f}) — an obstacle no sensor can see. "
                f"Counted as discovery ({n_freeze}/{self._max_consecutive_freezes}), "
                "not as a stack failure; the spot is marked for the planner."
            )
            if n_freeze >= self._max_consecutive_freezes:
                self.get_logger().error(
                    f"{n_freeze} freezes in a row — the room is full of things no "
                    "sensor can see. Stopping and saying so, rather than blaming the "
                    "stack."
                )
                self._mission_done = True
                res = self._map.info.resolution if self._map else 0.05
                self._finish(OUTCOME_BLOCKED_BY_UNSEEN_OBSTACLES, res,
                             self._remaining_candidates())
            return

        with self._lock:
            self._consecutive_failures += 1
            n = self._consecutive_failures
        if n >= self._max_consecutive_failures:
            self.get_logger().error(
                f"{n} goals in a row failed, at different places — this is the stack, "
                "not the room. Stopping. Check the controller (a stuck 'boxed in' "
                "state fails every goal in milliseconds) before running again."
            )
            self._mission_done = True
            res = self._map.info.resolution if self._map else 0.05
            # MEASURE what is left; do not assert 0. This path used to pass a
            # literal zero, so a mission that quit surrounded by unexplored floor
            # reported the most reassuring number available -- and both of the
            # 2026-08-10 runs did exactly that, including one that never moved an
            # inch and so had every candidate outstanding. remaining_candidates is
            # the field that makes an incomplete run diagnosable rather than merely
            # disappointing; fabricating it is the same class of defect as D21's
            # lying telemetry (D24).
            self._finish(OUTCOME_GOALS_KEEP_FAILING, res, self._remaining_candidates())

    def _remaining_candidates(self):
        """How many targets the explorer still wants, measured right now.

        Capped at `max_candidates` exactly like the completion path's count, so the
        two report paths mean the same thing: "are there targets left, and roughly
        how many", not a census. Returns None -- serialized as JSON null, i.e.
        UNKNOWN -- when the map or the robot pose is missing, because in that state
        the question genuinely cannot be answered and a 0 would be the same lie this
        exists to remove.
        """
        m = self._map
        if m is None:
            return None
        rp = self._robot_world(m.header.frame_id or "map")
        if rp is None:
            return None
        info = m.info
        res = info.resolution
        ox, oy = info.origin.position.x, info.origin.position.y
        try:
            return len(candidate_goals(
                m.data, info.width, info.height, ox, oy, res,
                int((rp[0] - ox) / res), int((rp[1] - oy) / res),
                self._covered, set(self._stalled), self._config,
            ))
        except Exception as exc:
            # Never let the count-for-the-report take the report down with it: an
            # unwritten report is strictly worse than one with an unknown field.
            self.get_logger().warn(f"could not count remaining targets: {exc}")
            return None

    def _cancel_active(self):
        """Cancel the in-flight goal. The result callback needs no special case:
        nothing is written off on a cancel."""
        with self._lock:
            handle = self._active_goal_handle
            self._active_goal_handle = None
            self._active_goal_cell = None
        if handle is not None:
            handle.cancel_goal_async()

    def _prune_stalled(self):
        """Drop expired entries so a briefly-stuck area gets retried."""
        now = time.monotonic()
        for coord in [c for c, exp in self._stalled.items() if exp <= now]:
            del self._stalled[coord]

    def _suppress_cell(self, cell):
        """Briefly stop proposing a goal that planned but would not drive."""
        if cell is None or self._map is None:
            return
        info = self._map.info
        res = info.resolution
        ox, oy = info.origin.position.x, info.origin.position.y
        gx, gy = cell
        wx, wy = cell_center_world(gx, gy, ox, oy, res)
        expiry = time.monotonic() + self._stall_ttl_s
        r = int(math.ceil(self._stall_radius_m / res))
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                self._stalled[world_grid(wx + dx * res, wy + dy * res, res)] = expiry


def main(args=None):
    rclpy.init(args=args)
    node = CoverageExplorerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
