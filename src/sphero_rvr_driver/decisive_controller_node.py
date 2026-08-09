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
import tf2_ros

from sphero_rvr_core.decisive_control import (
    BackOffConfig,
    DecisiveControlConfig,
    ProgressGuard,
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
        # Back-off reflex: reverse straight out of a boxed-in stall (no grind).
        self.declare_parameter("stall_time_s", 2.0)
        self.declare_parameter("progress_epsilon_m", 0.03)
        self.declare_parameter("back_off_speed_mps", 0.10)
        self.declare_parameter("back_off_distance_m", 0.25)
        self.declare_parameter("back_off_timeout_s", 3.0)
        self.declare_parameter("max_back_offs", 3)

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
        self._back_off_config = BackOffConfig(
            stall_time_s=float(self.get_parameter("stall_time_s").value),
            progress_epsilon_m=float(self.get_parameter("progress_epsilon_m").value),
            back_off_speed_mps=float(self.get_parameter("back_off_speed_mps").value),
            back_off_distance_m=float(self.get_parameter("back_off_distance_m").value),
            back_off_timeout_s=float(self.get_parameter("back_off_timeout_s").value),
            max_back_offs=int(self.get_parameter("max_back_offs").value),
        )

        # Goal preemption + a progress guard that PERSISTS across replans.
        # bt_navigator resends a fresh follow_path goal ~1 Hz as it replans; only
        # the newest may drive (older execute loops must bail, or they fight over
        # cmd_vel and stutter the motion). And the back-off reflex must track the
        # real robot over time, so the guard lives on the node and is only reset
        # when the goal ENDPOINT actually moves (a new destination), not on every
        # same-destination replan.
        self._active_goal_handle = None
        self._goal_lock = threading.Lock()
        self._guard = ProgressGuard(self._back_off_config)
        self._guard_goal = None  # (x, y) endpoint the guard is currently tracking
        self._goal_change_eps_m = 0.15

        self._cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
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
        # shared progress guard only for a genuinely new destination — a
        # same-destination replan keeps the guard so it tracks the real robot.
        with self._goal_lock:
            self._active_goal_handle = goal_handle
            if (
                self._guard_goal is None
                or math.hypot(
                    goal_x - self._guard_goal[0], goal_y - self._guard_goal[1]
                )
                > self._goal_change_eps_m
            ):
                self._guard = ProgressGuard(self._back_off_config)
                self._guard_goal = (goal_x, goal_y)
            guard = self._guard

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
                        self._guard_goal = None  # fresh guard for the next journey
                    break

                # Back-off reflex: if we are trying to translate but not actually
                # moving (boxed in against an obstacle), reverse straight out —
                # both tracks roll back together, above breakaway, so it does not
                # grind. Pivots are excluded (position is not expected to change).
                # After a few fruitless back-offs, abort so the planner re-routes.
                translating = command.mode in ("straight", "arc")
                guard_result = guard.step(robot_x, robot_y, time.monotonic(), translating)
                if guard_result.action == "abort":
                    self.get_logger().warn(
                        "decisive_controller: boxed in — backing off did not clear "
                        "it; aborting so the planner can re-route"
                    )
                    self._stop()
                    goal_handle.abort()
                    return result
                if guard_result.action == "reverse":
                    twist = Twist()
                    twist.linear.x = -abs(guard_result.reverse_speed_mps)
                    twist.angular.z = 0.0
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
