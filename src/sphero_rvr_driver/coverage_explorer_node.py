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
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from std_msgs.msg import String
import tf2_ros

from sphero_rvr_core.mission_report import (
    OUTCOME_COMPLETE,
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
        self.declare_parameter("complete_after_empty_cycles", 8)
        # Goal-progress watchdog: if a goal makes < this much progress in this many
        # seconds, cancel it -- it planned when we asked, but driving it is going
        # nowhere. Suppressed briefly afterwards so the very next selection does not
        # hand back the same cell (the planner would still say yes: planning is not
        # the thing that failed).
        self.declare_parameter("goal_progress_timeout_s", 6.0)
        self.declare_parameter("goal_progress_epsilon_m", 0.10)
        self.declare_parameter("stall_suppress_ttl_s", 45.0)
        self.declare_parameter("stall_suppress_radius_m", 0.2)
        # Start-pose guard. If the robot's OWN costmap cell is at/above inscribed
        # cost, the planner treats the start as in collision and EVERY goal returns
        # "no valid path" while Nav2's motion recoveries are all collision-blocked.
        # Without this the explorer churns goals and blacklists the whole map while
        # going nowhere (observed 2026-08-07: 0.26 m rear clearance, below
        # robot_radius + inflation_radius = 0.30 m, burned a four-minute run).
        # A mission should end with an ANSWER, not a log line someone has to find and
        # interpret. The report is latched JSON on a topic; the map is written next to
        # it. Both are written once, at the moment the mission ends.
        self.declare_parameter("report_topic", "/coverage_explorer/report")
        self.declare_parameter("save_map_on_end", True)
        self.declare_parameter("map_save_dir", os.path.expanduser("~/.ros/missions"))
        self.declare_parameter("blocked_start_check", True)
        self.declare_parameter("costmap_topic", "/global_costmap/costmap")
        self.declare_parameter("blocked_hold_s", 5.0)

        self._config = CoverageConfig(
            coverage_radius_m=float(self.get_parameter("coverage_radius_m").value),
            min_cluster_cells=int(self.get_parameter("min_cluster_cells").value),
            include_frontiers=bool(self.get_parameter("include_frontiers").value),
            free_threshold=int(self.get_parameter("free_threshold").value),
            max_candidates=int(self.get_parameter("max_candidates").value),
        )
        self._goal_progress_timeout_s = float(self.get_parameter("goal_progress_timeout_s").value)
        self._goal_progress_epsilon_m = float(self.get_parameter("goal_progress_epsilon_m").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._stall_ttl_s = float(self.get_parameter("stall_suppress_ttl_s").value)
        self._stall_radius_m = float(self.get_parameter("stall_suppress_radius_m").value)
        self._plan_timeout_s = float(self.get_parameter("plan_timeout_s").value)
        self._select_budget_s = float(self.get_parameter("select_budget_s").value)
        map_topic = str(self.get_parameter("map_topic").value)

        self._map = None
        self._costmap = None
        self._blocked_logged = False
        self._blocked_until = 0.0  # monotonic; blocked state flickers, so hold it
        self._covered = set()      # world-grid coords the rover has driven within radius of
        # World-grid coords of goals that PLANNED but then made no progress, with a
        # monotonic expiry. Nothing else writes here: an unplannable candidate needs
        # no memory (we re-ask the planner every cycle and it is authoritative), and
        # an aborted goal is likewise just re-asked. This is only to stop a
        # stalled-but-plannable cell being reselected immediately, forever.
        self._stalled = {}
        self._lock = threading.Lock()
        self._unplannable_last_cycle = 0
        self._selecting = False
        self._mission_start = time.monotonic()
        self._goals_sent = 0
        self._goals_succeeded = 0
        self._goals_aborted = 0
        self._planner_rejections = 0
        self._reported = False
        self._active_goal_cell = None
        self._active_goal_handle = None
        self._goal_inflight = False
        self._goal_start_pose = None   # (wx, wy) when the active goal was sent
        self._goal_start_time = None   # time.monotonic() when the active goal was sent
        self._mission_done = False
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
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._nav = ActionClient(self, NavigateToPose, "navigate_to_pose", callback_group=cbg)
        self._planner = ActionClient(
            self, ComputePathToPose, "compute_path_to_pose", callback_group=cbg
        )
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
        self.get_logger().info("coverage_explorer ready (coverage + frontier mission)")

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

    def _tick(self):
        if self._mission_done:
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
            if self._still_target(m, w, h, ox, oy, res, active_cell):
                # Watchdog: the goal planned when we asked, so if it is making no
                # progress the problem is downstream of planning. Drop it and
                # suppress it briefly -- re-asking the planner would just get the
                # same yes.
                if self._goal_stalled(wx, wy):
                    self.get_logger().warn(
                        f"coverage goal {active_cell} planned but made no progress in "
                        f"{self._goal_progress_timeout_s:.0f}s — dropping it"
                    )
                    self._cancel_active()
                    self._suppress_cell(active_cell)
                else:
                    return  # valid target, making progress -> keep driving
            else:
                # Target was covered en route (or its unknown resolved) -- success.
                self._cancel_active()
        if inflight:
            return

        candidates = candidate_goals(
            m.data, w, h, ox, oy, res, rcx, rcy, self._covered, set(self._stalled), self._config
        )
        # Selection blocks on planner queries and can outlast the tick period, so it
        # must not re-enter: two selection loops would race to send two goals, and
        # concurrent goals fighting over one actuator is this stack's known way to
        # produce motion that looks like a perception failure (the 2026-08-03
        # follow_path preemption bug). One selection at a time.
        with self._lock:
            if self._selecting:
                return
            self._selecting = True
        try:
            goal_cell = None
            exhausted = False
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
                if self._planner_can_reach(gwx, gwy, frame):
                    goal_cell = cell
                    break
        finally:
            with self._lock:
                self._selecting = False
        self._unplannable_last_cycle = (
            len(candidates) if goal_cell is None else candidates.index(goal_cell)
        )
        self._planner_rejections += self._unplannable_last_cycle
        if goal_cell is None and exhausted:
            return  # inconclusive cycle: leave the completion counter alone

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
                        "covered, so COVERAGE IS INCOMPLETE. The planner refuses "
                        "every remaining candidate: check whether the rover is boxed "
                        "in, or whether the costmap is blocking ground /map calls free."
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
        self._send_goal(goal_cell, frame, ox, oy, res)

    def _finish(self, outcome, resolution, remaining):
        """End the mission with an artifact: a latched JSON report, and the map on
        disk. Called once; a second call is ignored so a re-entered terminal branch
        cannot overwrite the record of what actually happened."""
        if self._reported:
            return
        self._reported = True
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

    def _await(self, future, deadline):
        """Block this callback until `future` resolves or `deadline` passes.

        Safe because the node runs on a MultiThreadedExecutor with a reentrant
        callback group, so another thread services the action response while this
        one waits. Returns the result, or None on timeout.
        """
        while not future.done():
            if time.monotonic() >= deadline:
                return None
            time.sleep(0.01)
        return future.result()

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
            return False
        return len(result.result.path.poses) > 0

    def _goal_stalled(self, wx, wy):
        """True if the active goal has made < epsilon progress for > timeout. The
        progress reference resets whenever the rover advances, so this fires only on
        a sustained no-progress stretch (planner churning), not a slow-but-moving
        drive."""
        if self._goal_start_pose is None or self._goal_start_time is None:
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

    def _send_goal(self, cell, frame, ox, oy, res):
        if not self._nav.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn("navigate_to_pose action server not available yet")
            return
        gx, gy = cell
        wx, wy = cell_center_world(gx, gy, ox, oy, res)
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = frame
        goal.pose.pose.position.x = float(wx)
        goal.pose.pose.position.y = float(wy)
        goal.pose.pose.orientation.w = 1.0
        with self._lock:
            self._goal_inflight = True
            self._active_goal_cell = cell
            self._goals_sent += 1
        self.get_logger().info(
            f"coverage goal -> cell {cell} world ({wx:.2f},{wy:.2f}); "
            f"covered={len(self._covered)} planner-rejected={self._unplannable_last_cycle} "
            f"stalled={len(self._stalled)}"
        )
        self._nav.send_goal_async(goal).add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        handle = future.result()
        with self._lock:
            self._goal_inflight = False
        if not handle.accepted:
            # No memory needed: the next cycle re-asks the planner, which is the
            # authority on whether this is worth trying again.
            self.get_logger().warn("coverage goal REJECTED by bt_navigator")
            with self._lock:
                self._active_goal_cell = None
            return
        with self._lock:
            self._active_goal_handle = handle
        handle.get_result_async().add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future):
        status = future.result().status
        with self._lock:
            self._active_goal_handle = None
            self._active_goal_cell = None
        if status == GoalStatus.STATUS_ABORTED:
            # Also no memory: an abort says this drive failed, not that the ground is
            # unreachable. Re-proposing it costs one planner query, and if it really
            # is unreachable the planner says so for free.
            with self._lock:
                self._goals_aborted += 1
            self.get_logger().info("coverage goal aborted — reselecting")
        elif status == GoalStatus.STATUS_SUCCEEDED:
            with self._lock:
                self._goals_succeeded += 1

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
