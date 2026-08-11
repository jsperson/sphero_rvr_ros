"""A Nav2 FollowPath controller that drives decisively instead of tracking exactly.

Drop-in replacement for the RPP/RotationShim FollowPath controller. It provides the
same ``follow_path`` action the Nav2 behavior tree calls, but instead of continuously
correcting to hug the planned path (which makes this drivetrain pivot and grind), it
applies the pragmatic policy from :mod:`sphero_rvr_core.decisive_control`:

* aim at a lookahead point on the path,
* drive **straight** when roughly aligned (a heading deadband — do not correct what
  does not need correcting),
* **arc** while rolling for moderate course changes (keeps the tracks turning = no
  grind),
* **pivot** in place only for large heading changes, decisively (above breakaway),
* stop when within tolerance of the path end.

Commands go out on ``cmd_vel`` and pass through the independent lidar collision-stop
supervisor exactly like every other motion source. Enable it in place of
``controller_server`` (they both offer ``follow_path`` — run only one).
"""

import math
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav2_msgs.action import FollowPath
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import Bool, String
import tf2_ros

from sphero_rvr_core.stall_ladder import LadderConfig, StallLadder
from sphero_rvr_core.decisive_control import (
    DecisiveControlConfig,
    FreezeMarkSet,
    compute_drive_command,
    heading_error_to_point,
    select_target_point,
)


class DecisiveControllerNode(Node):
    def __init__(self):
        super().__init__("decisive_controller")
        self.declare_parameter("control_frequency", 10.0)
        self.declare_parameter("lookahead_m", 0.4)
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("cruise_speed_mps", 0.20)
        self.declare_parameter("heading_deadband_rad", 0.17)
        self.declare_parameter("pivot_threshold_rad", 1.22)
        self.declare_parameter("arc_gain", 1.2)
        self.declare_parameter("max_arc_angular_rad_s", 0.8)
        self.declare_parameter("pivot_rate_rad_s", 0.9)
        self.declare_parameter("goal_tolerance_m", 0.10)
        # Freeze marks: how long a place the robot could not pass stays believed.
        # Generous, because freezes are rare and a mark that expires mid-mission
        # invites the rover straight back into the same obstacle; short enough that
        # a moved chair does not haunt the map forever.
        self.declare_parameter("freeze_mark_ttl_s", 300.0)
        self.declare_parameter("freeze_mark_merge_radius_m", 0.15)
        # THE STALL SURVIVAL LADDER (docs/stall_survival_ladder.md). Replaces the
        # back-off reflex, whose single escape -- straight reverse -- the supervisor
        # refuses outright at exactly the poses where it is needed.
        self.declare_parameter("stall_time_s", 2.0)
        self.declare_parameter("progress_epsilon_m", 0.03)
        self.declare_parameter("yaw_progress_epsilon_rad", 0.10)
        self.declare_parameter("max_yaw_rate_rad_s", 0.6)
        self.declare_parameter("suppressed_cycles", 20)
        self.declare_parameter("rung_budget_s", 3.0)
        self.declare_parameter("max_ladder_invocations_per_goal", 2)
        self.declare_parameter("ladder_reverse_speed_mps", 0.10)
        self.declare_parameter("ladder_forward_speed_mps", 0.10)
        self.declare_parameter("ladder_pivot_rate_rad_s", 0.40)

        self._frequency = float(self.get_parameter("control_frequency").value)
        self._lookahead = float(self.get_parameter("lookahead_m").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._config = DecisiveControlConfig(
            cruise_speed_mps=float(self.get_parameter("cruise_speed_mps").value),
            heading_deadband_rad=float(self.get_parameter("heading_deadband_rad").value),
            pivot_threshold_rad=float(self.get_parameter("pivot_threshold_rad").value),
            arc_gain=float(self.get_parameter("arc_gain").value),
            max_arc_angular_rad_s=float(self.get_parameter("max_arc_angular_rad_s").value),
            pivot_rate_rad_s=float(self.get_parameter("pivot_rate_rad_s").value),
            goal_tolerance_m=float(self.get_parameter("goal_tolerance_m").value),
        )
        _p = self.get_parameter
        self._ladder_config = LadderConfig(
            stall_time_s=float(_p("stall_time_s").value),
            progress_epsilon_m=float(_p("progress_epsilon_m").value),
            yaw_progress_epsilon_rad=float(_p("yaw_progress_epsilon_rad").value),
            max_yaw_rate_rad_s=float(_p("max_yaw_rate_rad_s").value),
            suppressed_cycles=int(_p("suppressed_cycles").value),
            rung_budget_s=float(_p("rung_budget_s").value),
            max_invocations_per_goal=int(
                _p("max_ladder_invocations_per_goal").value),
            reverse_speed_mps=float(_p("ladder_reverse_speed_mps").value),
            forward_speed_mps=float(_p("ladder_forward_speed_mps").value),
            pivot_rate_rad_s=float(_p("ladder_pivot_rate_rad_s").value),
        )

        # Goal preemption + a stall ladder that PERSISTS across replans.
        # bt_navigator resends a fresh follow_path goal ~1 Hz as it replans; only
        # the newest may drive (older execute loops must bail, or they fight over
        # cmd_vel and stutter the motion). And the back-off reflex must track the
        # real robot over time, so the ladder lives on the node and its per-goal
        # invocation budget is only reset when the goal ENDPOINT actually moves (a new
        # destination), not on every same-destination replan -- otherwise bt_navigator
        # would clear the anti-livelock counter roughly once a second, forever.
        self._active_goal_handle = None
        self._goal_lock = threading.Lock()
        self._ladder = StallLadder(self._ladder_config)
        self._ladder_goal = None  # (x, y) endpoint the ladder is currently tracking
        self._goal_change_eps_m = 0.15

        self._cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)

        # FREEZE-AS-SENSOR. The supervisor's actual motor output is what separates
        # "it braked me for something it can see" (output zero -- normal) from "it let
        # me drive and I still did not move" (output nonzero -- something is there
        # that NO sensor on this robot can detect). Without this subscription those
        # two are indistinguishable from inside the ladder, which is why every stall
        # used to look alike.
        self._out_lock = threading.Lock()
        self._out_moving = False
        self.create_subscription(Twist, "cmd_vel_motor", self._on_motor_out, 10)
        # Which way is open, for the ladder's pivot and drive rungs. The ladder core
        # stays pure -- it does not parse scans -- so the bearing is computed here and
        # handed in.
        self._scan_lock = threading.Lock()
        self._open_bearing_rad = 0.0
        self.create_subscription(
            LaserScan, "scan", self._on_scan, qos_profile_sensor_data)
        self._freeze_marks = FreezeMarkSet(
            ttl_s=float(self.get_parameter("freeze_mark_ttl_s").value),
            merge_radius_m=float(self.get_parameter("freeze_mark_merge_radius_m").value),
        )
        # Marks go to a costmap layer of their OWN (see lean_nav2.yaml freeze_layer).
        # They cannot be another source of the existing obstacle layer: that layer
        # raytrace-clears from scan, and the whole premise is that the lidar sees
        # straight through this obstacle, so the marks would be wiped within a scan
        # or two by the one sensor blind to them.
        # PRIVATE names ("~/"). A bare relative name resolves against the NAMESPACE,
        # not the node name, so "freeze_marks" published to /freeze_marks while
        # lean_nav2.yaml's freeze_layer reads /decisive_controller/freeze_marks and
        # the explorer subscribes /decisive_controller/freeze_event. Both freeze
        # channels were DARK: marks never reached the planner, events never reached
        # the mission layer, and nothing errored -- the publisher succeeded, it just
        # spoke into an empty room. Identical to the mission-service trap fixed in
        # b2f0980; I fixed that one and did not sweep for its siblings.
        self._freeze_cloud_pub = self.create_publisher(
            PointCloud2, "~/freeze_marks", 10)
        # A separate, human- and explorer-readable event: the mission layer uses it
        # to classify an abort as DISCOVERY rather than as a failure of the stack.
        self._freeze_event_pub = self.create_publisher(
            String, "~/freeze_event", 10)
        # F1: the explorer's 6 s goal watchdog would cancel the goal at t+6 while the
        # ladder is still working -- detection ~1-2 s plus 3 s per rung puts rung 3 at
        # t+8 and exhaustion at t+14. Rungs 3 and 4 were therefore UNREACHABLE in the
        # assembled system precisely when rungs 1 and 2 are refused, which is the only
        # case the ladder exists for. The controller owns recovery now, so it says so
        # out loud and the explorer holds off while a ladder is running.
        self._ladder_active_pub = self.create_publisher(Bool, "~/ladder_active", 10)
        self.create_timer(0.5, self._publish_freeze_marks)
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._callback_group = ReentrantCallbackGroup()
        self._action_server = ActionServer(
            self,
            FollowPath,
            "follow_path",
            execute_callback=self._execute,
            goal_callback=lambda _goal: GoalResponse.ACCEPT,
            cancel_callback=lambda _goal: CancelResponse.ACCEPT,
            callback_group=self._callback_group,
        )
        self.get_logger().info("decisive_controller ready (follow_path)")

    def _on_motor_out(self, msg):
        """Track whether the SUPERVISOR is currently letting us drive.

        ANGULAR COUNTS. This tested `linear.x` alone until the ladder landed, which
        was survivable only because the old back-off reflex ignored pivots entirely.
        The ladder does not: a GRANTED pivot is motion, and reading it as "output
        zero" would fire the output-suppressed condition one second into every
        legitimate turn.
        """
        with self._out_lock:
            self._out_moving = (abs(msg.linear.x) > 1e-6
                                or abs(msg.angular.z) > 1e-6)

    def _on_scan(self, msg):
        """Remember the bearing of the widest open direction, in the ROBOT frame.

        Deliberately crude: the widest gap, not the single furthest ray. One long
        return down a doorway is not somewhere a rover fits, and escaping toward it
        is how you swap a stall for a wedge. Bearings are laser-frame here; the laser
        is mounted with ~179 deg yaw (see base_to_laser_static_tf), so the sign
        convention is handled by the caller's own TF, not assumed here.
        """
        best_bearing, best_span, span_start = 0.0, 0, None
        rng = msg.ranges
        threshold = 0.8
        for i, r in enumerate(rng):
            open_here = (r != r) or r > threshold   # NaN reads as open sky
            if open_here and span_start is None:
                span_start = i
            elif not open_here and span_start is not None:
                if i - span_start > best_span:
                    best_span = i - span_start
                    mid = (span_start + i) // 2
                    best_bearing = msg.angle_min + mid * msg.angle_increment
                span_start = None
        if span_start is not None and len(rng) - span_start > best_span:
            mid = (span_start + len(rng)) // 2
            best_bearing = msg.angle_min + mid * msg.angle_increment
        with self._scan_lock:
            self._open_bearing_rad = float(best_bearing)

    def _open_bearing(self):
        with self._scan_lock:
            return self._open_bearing_rad

    def _output_moving(self):
        with self._out_lock:
            return self._out_moving

    def _record_freeze(self, x, y, now):
        """A place the robot proved it could not pass. Publish it as an event and
        add it to the mark set the planner will see."""
        mark = self._freeze_marks.add(x, y, now)
        self.get_logger().warn(
            f"FREEZE at ({x:.2f},{y:.2f}) — the supervisor permitted motion and the "
            "robot did not move: an obstacle no sensor on this robot can see. "
            "Marking it for the planner and backing straight out."
        )
        self._freeze_event_pub.publish(String(
            data=f'{{"x": {x:.3f}, "y": {y:.3f}, "stamp": {now:.2f}}}'))
        self._publish_freeze_marks()
        return mark

    def _publish_freeze_marks(self):
        """Republish the whole live set at ~2 Hz.

        NOTE, measured rather than assumed: ceasing to publish does NOT un-mark the
        costmap. The freeze layer runs clearing:false and an ObstacleLayer never
        un-marks a cell, so a mark lasts the life of the costmap (i.e. the mission).
        Republishing keeps the layer fed for late subscribers and keeps this set the
        single source for the mission report; the TTL bounds those, not the grid.
        """
        now = time.monotonic()
        live = self._freeze_marks.live(now)
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.height = 1
        msg.width = len(live)
        msg.fields = [
            PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
            for i, n in enumerate(("x", "y", "z"))
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12 * len(live)
        msg.is_dense = True
        import struct
        # z at the lidar plane so the costmap's height filter keeps them: these
        # stand for an obstacle the lidar CANNOT see, so they must be presented at a
        # height it would have accepted.
        msg.data = b"".join(struct.pack("<fff", m.x, m.y, 0.15) for m in live)
        self._freeze_cloud_pub.publish(msg)

    def freeze_marks_for_report(self):
        """Consumed by whoever writes the mission report. These belong in the REPORT
        and never in the saved map: the map is the room as SLAM measured it; these
        are the robot's own belief about where it could not go."""
        return self._freeze_marks.as_report_list(time.monotonic())

    def _robot_pose_in(self, frame):
        """(x, y, yaw) of base_frame expressed in `frame`, or None if TF unavailable."""
        try:
            tf = self._tf_buffer.lookup_transform(frame, self._base_frame, rclpy.time.Time())
        except Exception:
            return None
        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return tf.transform.translation.x, tf.transform.translation.y, yaw

    def _stop(self):
        self._cmd_pub.publish(Twist())

    def _execute(self, goal_handle):
        path = goal_handle.request.path
        frame = path.header.frame_id or "odom"
        points = [(p.pose.position.x, p.pose.position.y) for p in path.poses]
        result = FollowPath.Result()

        if not points:
            self._stop()
            goal_handle.abort()
            return result

        goal_x, goal_y = points[-1]
        # Plain sleep rather than rclpy Rate: Rate.sleep() inside an action execute
        # callback can deadlock; TF stays fresh via the listener's own executor
        # thread (MultiThreadedExecutor).
        period = 1.0 / self._frequency if self._frequency > 0 else 0.1
        feedback = FollowPath.Feedback()

        # Become the active goal (preempting any older execute loop) and reset the
        # shared stall ladder only for a genuinely new destination — a
        # same-destination replan keeps it so it tracks the real robot.
        with self._goal_lock:
            self._active_goal_handle = goal_handle
            if (
                self._ladder_goal is None
                or math.hypot(
                    goal_x - self._ladder_goal[0], goal_y - self._ladder_goal[1]
                )
                > self._goal_change_eps_m
            ):
                # A genuinely new destination gets a fresh invocation budget; a
                # same-destination replan must NOT, or bt_navigator's ~1 Hz replan
                # would reset the anti-livelock counter forever.
                self._ladder.reset_goal()
                self._ladder_goal = (goal_x, goal_y)
            ladder = self._ladder

        try:
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    self._stop()
                    goal_handle.canceled()
                    return result
                # Superseded by a newer follow_path goal (bt_navigator replan)?
                # Bail without touching cmd_vel so only the newest loop drives —
                # concurrent loops were stuttering the motion and false-tripping
                # the back-off reflex.
                if self._active_goal_handle is not goal_handle:
                    goal_handle.abort()
                    return result

                pose = self._robot_pose_in(frame)
                if pose is None:
                    self._stop()
                    time.sleep(period)
                    continue
                robot_x, robot_y, robot_yaw = pose

                target = select_target_point(points, robot_x, robot_y, self._lookahead)
                heading_error, _ = heading_error_to_point(
                    robot_x, robot_y, robot_yaw, target[0], target[1]
                )
                distance_to_goal = math.hypot(goal_x - robot_x, goal_y - robot_y)
                command = compute_drive_command(heading_error, distance_to_goal, self._config)

                if command.mode == "arrived":
                    with self._goal_lock:
                        self._ladder_goal = None  # fresh ladder next journey
                    break

                # THE STALL SURVIVAL LADDER. One predicate for "we are not getting
                # anywhere" from any cause, one escalating sequence of escapes, and a
                # failure counted ONCE PER EXHAUSTED LADDER rather than once per
                # refused action. See docs/stall_survival_ladder.md for the two
                # missions this replaces, both of which died with escapes untried.
                now_s = time.monotonic()
                ladder_result = ladder.step(
                    x=robot_x, y=robot_y, yaw=robot_yaw, now=now_s,
                    commanding=(command.linear_mps != 0.0
                                or command.angular_rad_s != 0.0),
                    output_moving=self._output_moving(),
                    open_bearing_rad=self._open_bearing(),
                )
                self._ladder_active_pub.publish(
                    Bool(data=(ladder_result.action == "rung")))
                if ladder_result.freeze:
                    # The supervisor permitted motion and we did not move: something is
                    # physically there that no sensor on this robot can see. A DATA
                    # POINT, not a mission failure. Mark it for the planner — and keep
                    # running the ladder, because discovering an invisible obstacle
                    # does not excuse us from escaping it.
                    self._record_freeze(robot_x, robot_y, now_s)
                if ladder_result.exhausted:
                    # Say WHICH kind of dead end this was. "Genuinely wedged" means
                    # the supervisor refused every rung outright -- the room has us
                    # surrounded, and there is no bug to hunt. "Ineffective" means we
                    # were permitted to move and it did not help, which IS worth
                    # hunting. Reporting both as "aborted" is how a stack bug hides
                    # behind a tight room, and vice versa.
                    if ladder_result.genuinely_wedged:
                        self.get_logger().warn(
                            "decisive_controller: GENUINELY WEDGED — the supervisor "
                            "refused every escape (reverse, arc, pivot, forward); "
                            "the rover was never permitted to move. Aborting so the "
                            "planner can re-route. Not a stack failure."
                        )
                    else:
                        self.get_logger().warn(
                            "decisive_controller: every escape tried and none freed "
                            f"us ({ladder_result.reason}) — we WERE permitted to "
                            "move and it did not help. Aborting so the planner can "
                            "re-route."
                        )
                    self._stop()
                    goal_handle.abort()
                    return result
                if ladder_result.action == "rung":
                    self.get_logger().info(
                        f"ladder: {ladder_result.reason} "
                        f"({ladder_result.linear_x:+.2f}, "
                        f"{ladder_result.angular_z:+.2f})",
                        throttle_duration_sec=1.0,
                    )
                    twist = Twist()
                    twist.linear.x = float(ladder_result.linear_x)
                    twist.angular.z = float(ladder_result.angular_z)
                    self._cmd_pub.publish(twist)
                    feedback.distance_to_goal = float(distance_to_goal)
                    feedback.speed = float(twist.linear.x)
                    goal_handle.publish_feedback(feedback)
                    time.sleep(period)
                    continue

                twist = Twist()
                twist.linear.x = command.linear_mps
                twist.angular.z = command.angular_rad_s
                self._cmd_pub.publish(twist)

                feedback.distance_to_goal = float(distance_to_goal)
                feedback.speed = float(command.linear_mps)
                goal_handle.publish_feedback(feedback)

                time.sleep(period)
        finally:
            # Only the still-active goal halts the robot on exit. A superseded
            # goal must NOT publish a stop — the newer goal already owns cmd_vel,
            # and a stop here would punch a gap in its command stream.
            with self._goal_lock:
                if self._active_goal_handle is goal_handle:
                    self._active_goal_handle = None
                    self._stop()

        goal_handle.succeed()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = DecisiveControllerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node._stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
