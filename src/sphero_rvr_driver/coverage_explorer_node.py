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
"""

import math
import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from action_msgs.msg import GoalStatus
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
import tf2_ros

from sphero_rvr_core.coverage_exploration import (
    CoverageConfig,
    cell_center_world,
    cell_world_grid,
    is_frontier,
    robot_start_blocked,
    select_next_goal,
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
        self.declare_parameter("inscribed_radius_m", 0.14)
        self.declare_parameter("occupied_threshold", 50)
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("cycle_period_s", 1.0)
        # Blacklisting stops a failed goal being retried forever -- but it WAS
        # forever, and wide. Two properties made that wrong in general, not just
        # here: a planner failure is usually transient (it depends on where the robot
        # currently is and how good the map is right now), and the disc written off
        # was far larger than the thing that actually failed. Observed consequence:
        # 2390 cells blacklisted = 73% of all free space, and the mission ended with
        # 108 frontier cells still unexplored in an area that was reachable.
        #
        # Radius is derived from the ROBOT, not the room: a goal that failed says
        # nothing about ground more than roughly one footprint away, so ~robot_radius
        # (0.14) plus a small margin. TTL says how long a failure stays believable
        # before the area is worth retrying; it is a parameter because the right
        # value depends on platform speed, not on any particular space.
        self.declare_parameter("blacklist_radius_m", 0.2)
        self.declare_parameter("blacklist_ttl_s", 45.0)
        self.declare_parameter("complete_after_empty_cycles", 8)
        # Goal-progress watchdog: if a goal makes < this much progress in this many
        # seconds, cancel + blacklist it instead of letting bt_navigator churn.
        self.declare_parameter("goal_progress_timeout_s", 6.0)
        self.declare_parameter("goal_progress_epsilon_m", 0.10)
        # Start-pose guard. If the robot's OWN costmap cell is at/above inscribed
        # cost, the planner treats the start as in collision and EVERY goal returns
        # "no valid path" while Nav2's motion recoveries are all collision-blocked.
        # Without this the explorer churns goals and blacklists the whole map while
        # going nowhere (observed 2026-08-07: 0.26 m rear clearance, below
        # robot_radius + inflation_radius = 0.30 m, burned a four-minute run).
        self.declare_parameter("blocked_start_check", True)
        self.declare_parameter("costmap_topic", "/global_costmap/costmap")
        self.declare_parameter("blocked_hold_s", 5.0)

        self._config = CoverageConfig(
            coverage_radius_m=float(self.get_parameter("coverage_radius_m").value),
            min_cluster_cells=int(self.get_parameter("min_cluster_cells").value),
            include_frontiers=bool(self.get_parameter("include_frontiers").value),
            free_threshold=int(self.get_parameter("free_threshold").value),
            inscribed_radius_m=float(self.get_parameter("inscribed_radius_m").value),
            occupied_threshold=int(self.get_parameter("occupied_threshold").value),
        )
        self._goal_progress_timeout_s = float(self.get_parameter("goal_progress_timeout_s").value)
        self._goal_progress_epsilon_m = float(self.get_parameter("goal_progress_epsilon_m").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._blacklist_radius_m = float(self.get_parameter("blacklist_radius_m").value)
        self._blacklist_ttl_s = float(self.get_parameter("blacklist_ttl_s").value)
        map_topic = str(self.get_parameter("map_topic").value)

        self._map = None
        self._costmap = None
        self._blocked_logged = False
        self._blocked_until = 0.0  # monotonic; blocked state flickers, so hold it
        self._covered = set()      # world-grid coords the rover has driven within radius of
        self._blacklist = {}       # world-grid coord -> monotonic expiry time
        self._suppress_blacklist = False  # set when WE cancel on purpose
        self._lock = threading.Lock()
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
        self._prune_blacklist()
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
                # Watchdog: navigable selection should keep goals plannable, but if
                # one still makes no progress (planner churning "no valid path"),
                # cancel + blacklist it rather than let bt_navigator grind on it.
                if self._goal_stalled(wx, wy):
                    self.get_logger().warn(
                        f"coverage goal {active_cell} made no progress in "
                        f"{self._goal_progress_timeout_s:.0f}s — blacklisting"
                    )
                    self._cancel_active(voluntary=True)
                    self._blacklist_cell(active_cell)
                else:
                    return  # valid target, making progress -> keep driving
            else:
                # Target was covered en route (or its unknown resolved) -- that is
                # SUCCESS, not unreachability. Cancel without blacklisting, or every
                # win would poison a blacklist disc around itself.
                self._cancel_active(voluntary=True)
        if inflight:
            return

        goal_cell = select_next_goal(
            m.data, w, h, ox, oy, res, rcx, rcy, self._covered, set(self._blacklist), self._config
        )
        if goal_cell is None:
            # Debounce: don't latch "complete" on a transient/startup empty. Only
            # finish after N consecutive empties AND once it has actually explored.
            self._consecutive_empty += 1
            if self._ever_had_target and self._consecutive_empty >= self._complete_after_empty:
                self._mission_done = True
                blacklisted = len(self._blacklist)
                if blacklisted:
                    # Running out of candidates because they were all marked
                    # unreachable is NOT the same as having covered everything.
                    # Saying "COMPLETE" here is a lie: on 2026-08-08 the rover
                    # declared it one second after reporting START POSE BLOCKED,
                    # wedged against a chair with 2315 cells blacklisted.
                    self.get_logger().warn(
                        f"exploration ENDED with no reachable targets left — "
                        f"{len(self._covered)} cells covered but {blacklisted} "
                        "blacklisted as unreachable, so COVERAGE MAY BE INCOMPLETE. "
                        "Check whether the rover is stuck or boxed in."
                    )
                else:
                    self.get_logger().info(
                        f"coverage+frontier mission COMPLETE — {len(self._covered)} cells "
                        "covered; all reachable free space seen and within coverage radius"
                    )
            return
        self._consecutive_empty = 0
        self._ever_had_target = True
        self._goal_start_pose = (wx, wy)
        self._goal_start_time = time.monotonic()
        self._send_goal(goal_cell, frame, ox, oy, res)

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
        if wg in self._blacklist:
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
        self.get_logger().info(
            f"coverage goal -> cell {cell} world ({wx:.2f},{wy:.2f}); "
            f"covered={len(self._covered)} blacklisted={len(self._blacklist)}"
        )
        self._nav.send_goal_async(goal).add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        handle = future.result()
        with self._lock:
            self._goal_inflight = False
        if not handle.accepted:
            self.get_logger().warn("coverage goal REJECTED — blacklisting")
            with self._lock:
                cell = self._active_goal_cell
                self._active_goal_cell = None
            self._blacklist_cell(cell)
            return
        with self._lock:
            self._active_goal_handle = handle
        handle.get_result_async().add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future):
        status = future.result().status
        with self._lock:
            cell = self._active_goal_cell
            self._active_goal_handle = None
            self._active_goal_cell = None
        with self._lock:
            suppress = self._suppress_blacklist
            self._suppress_blacklist = False
        # Only a genuine ABORT means unreachable. A CANCEL that we initiated is
        # either success (covered en route) or already blacklisted by the watchdog;
        # blacklisting it again poisons map we actually covered and can drive a
        # premature "mission COMPLETE".
        if status == GoalStatus.STATUS_ABORTED or (
            status == GoalStatus.STATUS_CANCELED and not suppress
        ):
            self._blacklist_cell(cell)

    def _cancel_active(self, voluntary=False):
        """Cancel the in-flight goal. `voluntary=True` means WE decided to drop it
        (target already covered, or the watchdog is about to blacklist it
        explicitly), so the CANCELED result must not be treated as unreachable."""
        with self._lock:
            handle = self._active_goal_handle
            self._active_goal_handle = None
            self._active_goal_cell = None
            self._suppress_blacklist = voluntary
        if handle is not None:
            handle.cancel_goal_async()

    def _prune_blacklist(self):
        """Drop expired entries so a temporarily unreachable area gets retried."""
        now = time.monotonic()
        for coord in [c for c, exp in self._blacklist.items() if exp <= now]:
            del self._blacklist[coord]

    def _blacklist_cell(self, cell):
        if cell is None or self._map is None:
            return
        info = self._map.info
        res = info.resolution
        ox, oy = info.origin.position.x, info.origin.position.y
        gx, gy = cell
        wx, wy = cell_center_world(gx, gy, ox, oy, res)
        expiry = time.monotonic() + self._blacklist_ttl_s
        r = int(math.ceil(self._blacklist_radius_m / res))
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                self._blacklist[world_grid(wx + dx * res, wy + dy * res, res)] = expiry


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
