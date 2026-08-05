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
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("cycle_period_s", 1.0)
        self.declare_parameter("blacklist_radius_m", 0.3)
        self.declare_parameter("complete_after_empty_cycles", 8)

        self._config = CoverageConfig(
            coverage_radius_m=float(self.get_parameter("coverage_radius_m").value),
            min_cluster_cells=int(self.get_parameter("min_cluster_cells").value),
            include_frontiers=bool(self.get_parameter("include_frontiers").value),
            free_threshold=int(self.get_parameter("free_threshold").value),
        )
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._blacklist_radius_m = float(self.get_parameter("blacklist_radius_m").value)
        map_topic = str(self.get_parameter("map_topic").value)

        self._map = None
        self._covered = set()      # world-grid coords the rover has driven within radius of
        self._blacklist = set()    # world-grid coords of unreachable goals
        self._lock = threading.Lock()
        self._active_goal_cell = None
        self._active_goal_handle = None
        self._goal_inflight = False
        self._mission_done = False
        self._consecutive_empty = 0
        self._ever_had_target = False
        self._complete_after_empty = int(self.get_parameter("complete_after_empty_cycles").value)

        cbg = ReentrantCallbackGroup()
        map_qos = QoSProfile(depth=1)
        map_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        map_qos.reliability = QoSReliabilityPolicy.RELIABLE
        self.create_subscription(OccupancyGrid, map_topic, self._on_map, map_qos, callback_group=cbg)
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._nav = ActionClient(self, NavigateToPose, "navigate_to_pose", callback_group=cbg)
        self.create_timer(
            float(self.get_parameter("cycle_period_s").value), self._tick, callback_group=cbg
        )
        self.get_logger().info("coverage_explorer ready (coverage + frontier mission)")

    def _on_map(self, msg):
        self._map = msg

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
                return  # let it keep driving
            self._cancel_active()
        if inflight:
            return

        goal_cell = select_next_goal(
            m.data, w, h, ox, oy, res, rcx, rcy, self._covered, self._blacklist, self._config
        )
        if goal_cell is None:
            # Debounce: don't latch "complete" on a transient/startup empty. Only
            # finish after N consecutive empties AND once it has actually explored.
            self._consecutive_empty += 1
            if self._ever_had_target and self._consecutive_empty >= self._complete_after_empty:
                self._mission_done = True
                self.get_logger().info(
                    f"coverage+frontier mission COMPLETE — {len(self._covered)} cells covered; "
                    "all reachable free space seen and within coverage radius"
                )
            return
        self._consecutive_empty = 0
        self._ever_had_target = True
        self._send_goal(goal_cell, frame, ox, oy, res)

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
        # Unreachable/failed goal -> blacklist so we don't retry it forever.
        if status in (GoalStatus.STATUS_ABORTED, GoalStatus.STATUS_CANCELED):
            self._blacklist_cell(cell)

    def _cancel_active(self):
        with self._lock:
            handle = self._active_goal_handle
            self._active_goal_handle = None
            self._active_goal_cell = None
        if handle is not None:
            handle.cancel_goal_async()

    def _blacklist_cell(self, cell):
        if cell is None or self._map is None:
            return
        info = self._map.info
        res = info.resolution
        ox, oy = info.origin.position.x, info.origin.position.y
        gx, gy = cell
        wx, wy = cell_center_world(gx, gy, ox, oy, res)
        r = int(math.ceil(self._blacklist_radius_m / res))
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                self._blacklist.add(world_grid(wx + dx * res, wy + dy * res, res))


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
