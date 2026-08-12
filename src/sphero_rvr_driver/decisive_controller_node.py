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
from std_msgs.msg import Bool, Float32, Int32, String
import tf2_ros

from sphero_rvr_core.stall_ladder import LadderConfig, StallLadder
from sphero_rvr_core.decisive_control import (
    AvoidanceConfig,
    DecisiveControlConfig,
    FreezeMarkSet,
    avoidance_heading_offset,
    camera_points_to_polar,
    compute_drive_command,
    corridor_blocker,
    heading_error_to_point,
    select_target_point,
)


def _wrap_angle(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


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
        # 0.40, matched to the supervisor's max_angular_rad_s. It was 0.8, and the
        # top half of that range has never reached a motor: the supervisor clamps
        # every command to 0.40, so an arc asking for 0.8 was executed at half what
        # it requested. A parameter whose upper half is inert is exactly how
        # "commanded" and "achieved" drifted apart in the grind-yaw guard (D32);
        # change this only together with collision_stop.yaml's max_angular_rad_s.
        self.declare_parameter("max_arc_angular_rad_s", 0.40)
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
        # COMPLETE LADDER TRAVERSALS, not repeats of one escape (turning_batch_design
        # PART TWO §9). Renamed rather than re-tuned: the old name would have carried
        # a deployed value that means something entirely different now.
        self.declare_parameter("max_ladder_traversals_per_goal", 1)
        # The OUTER bound: complete ladders per goal counted across every escalation
        # reset. The traversal budget above is per stall region and renews whenever
        # the rover genuinely gets somewhere, so without this a goal is not bounded.
        self.declare_parameter("max_total_ladder_traversals_per_goal", 4)
        self.declare_parameter("ladder_reverse_speed_mps", 0.10)
        self.declare_parameter("ladder_forward_speed_mps", 0.10)
        self.declare_parameter("ladder_pivot_rate_rad_s", 0.40)
        # GENTLE TURN-AWAY (docs/turning_batch_design.md item 1). Every default is
        # derived in AvoidanceConfig from the deployed supervisor/brake config.
        self.declare_parameter("avoid_enable", True)
        self.declare_parameter("avoid_engage_m", 0.90)
        self.declare_parameter("avoid_stop_ref_m", 0.50)
        self.declare_parameter("avoid_max_offset_rad", 0.33)
        self.declare_parameter("avoid_corridor_half_width_m", 0.18)
        self.declare_parameter("avoid_max_offset_step_rad", 0.08)
        self.declare_parameter("avoid_camera_min_range_m", 0.40)
        self.declare_parameter("avoid_camera_max_range_m", 1.20)
        self.declare_parameter("avoid_camera_topic", "/camera/low_obstacles")
        self.declare_parameter("avoid_camera_max_age_s", 0.6)

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
            max_ladder_traversals_per_goal=int(
                _p("max_ladder_traversals_per_goal").value),
            max_total_traversals_per_goal=int(
                _p("max_total_ladder_traversals_per_goal").value),
            reverse_speed_mps=float(_p("ladder_reverse_speed_mps").value),
            forward_speed_mps=float(_p("ladder_forward_speed_mps").value),
            pivot_rate_rad_s=float(_p("ladder_pivot_rate_rad_s").value),
        )
        self._avoid_enable = bool(_p("avoid_enable").value)
        self._avoid_config = AvoidanceConfig(
            engage_m=float(_p("avoid_engage_m").value),
            stop_ref_m=float(_p("avoid_stop_ref_m").value),
            max_offset_rad=float(_p("avoid_max_offset_rad").value),
            corridor_half_width_m=float(_p("avoid_corridor_half_width_m").value),
            max_offset_step_rad=float(_p("avoid_max_offset_step_rad").value),
            camera_min_range_m=float(_p("avoid_camera_min_range_m").value),
            camera_max_range_m=float(_p("avoid_camera_max_range_m").value),
        )
        self._avoid_camera_max_age_s = float(_p("avoid_camera_max_age_s").value)
        # The steering offset is per-goal state on the node, because the rate limit
        # is what keeps a 5 Hz camera from snapping a 10 Hz heading.
        self._avoid_offset_rad = 0.0

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
        self._open_bearing_at = None
        self._open_bearing_max_age_s = 1.0
        self._open_gap_min_range_m = 0.8
        # Nearest thing actually in our way, from each sensor, in the BASE frame.
        # Both are freshness-bounded: steering toward a remembered gap is how an
        # escape drives into a wall it already passed, and steering away from a
        # remembered obstacle is the same mistake with the sign flipped.
        self._lidar_blocker = None
        self._lidar_blocker_at = None
        self._camera_blocker = None
        self._camera_blocker_at = None
        self.create_subscription(
            LaserScan, "scan", self._on_scan, qos_profile_sensor_data)
        # READ-ONLY consumption of the low-obstacle cloud. This node does not brake,
        # does not veto and does not touch the detector -- the supervisor keeps every
        # one of those jobs. It reads the same points so it can start turning while
        # the brake would still let it drive, because the brake's forward scale is
        # zero at 0.50 m and by then the turn cannot be completed (design note §1.2).
        self.create_subscription(
            PointCloud2, str(self.get_parameter("avoid_camera_topic").value),
            self._on_camera_cloud, qos_profile_sensor_data)
        # AUTHORITATIVE journey boundary from the explorer. Replaces the endpoint
        # proxy: bt_navigator's ~1 Hz replans of one journey carry the SAME
        # generation, so the anti-thrash budget still accumulates within a journey,
        # while a genuinely new explorer goal changes it and earns a fresh budget.
        self._goal_generation = None
        self.create_subscription(
            Int32, "/coverage_explorer/goal_generation",
            self._on_goal_generation, 10)
        # Disc geometry (design_d25_freeze.md). Radius ~robot_radius; the ring is
        # sampled densely enough that no costmap cell inside it is missed at 0.05 m
        # resolution, and at two radii so the interior fills rather than leaving a
        # hollow annulus the planner could thread.
        self.declare_parameter("freeze_mark_radius_m", 0.14)
        self.declare_parameter("freeze_mark_disc_points", 12)
        self.declare_parameter("footprint_front_m", 0.11)
        self._mark_radius_m = float(self.get_parameter("freeze_mark_radius_m").value)
        self._mark_disc_points = int(
            self.get_parameter("freeze_mark_disc_points").value)
        self._footprint_front_m = float(
            self.get_parameter("footprint_front_m").value)
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
        # THE STEERING LAW SAYS WHAT IT DID. On its first flight (2026-08-11
        # gauntlet 1) the offset was computed and applied and published NOWHERE, so
        # when the supervising session asked "was steering engaged before the stop?"
        # the honest answer was that no artifact could say -- the law's own behaviour
        # was unobservable in the recording it was flown to produce. Published every
        # cycle including the zeros, because "it did not engage" is exactly as much
        # of an answer as "it did", and a topic that only speaks when something
        # happens cannot distinguish silence from absence.
        self._avoid_offset_pub = self.create_publisher(Float32, "~/avoid_offset", 10)
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
        # N2. The floor is what the ladder needs to see PROGRESS within its own
        # stall window, not a bare non-zero test: output slow enough that it cannot
        # move progress_epsilon_m in stall_time_s is not "the supervisor is letting
        # us drive", and counting it as permitted would vote FREEZE on a granted
        # crawl. Unreachable at the deployed config, closed structurally so it stays
        # unreachable if a scale factor ever changes.
        cfg = self._ladder_config
        lin_floor = cfg.progress_epsilon_m / cfg.stall_time_s
        ang_floor = cfg.yaw_progress_epsilon_rad / cfg.stall_time_s
        with self._out_lock:
            self._out_moving = (abs(msg.linear.x) >= lin_floor
                                or abs(msg.angular.z) >= ang_floor)

    def _on_goal_generation(self, msg):
        if self._goal_generation is not None and msg.data != self._goal_generation:
            self._ladder.reset_goal()
        self._goal_generation = msg.data

    def _on_scan(self, msg):
        """Remember the bearing of the widest open direction, IN THE ROBOT FRAME.

        N1. This used to return the laser-frame bearing while its own docstring
        claimed "the caller's TF handles the sign convention" -- and no caller did.
        The laser is mounted at ~179 deg yaw, so an unrotated bearing is very nearly
        a POINT REFLECTION of open space: a gap at robot-left arrives as robot-right,
        and rungs 3 and 4 steer confidently into the closed side. This project has a
        standing rule about exactly this ("rotate into base_link, read the rotation
        FROM TF, never hardcode"), which I broke four lines under a docstring
        admitting the frame problem existed.

        So the rotation is read from TF -- never a hardcoded 179 -- and returns None
        when TF is unavailable, because a bearing we cannot place is worse than no
        bearing: the ladder's default of "straight" is at least frame-agnostic.

        The gap search is CIRCULAR. A linear scan splits a gap that spans the end of
        the array into two half-width pieces, and for this laser that wrap point is
        very nearly dead ahead of the robot -- so the one direction most likely to be
        the way out was the one systematically biased against.
        """
        rng = msg.ranges
        n = len(rng)
        if not n:
            return

        # TF FIRST. It used to be read at the very end, which was fine when the gap
        # bearing was the only consumer; the steering blockers need the same rotation
        # and must not be skipped by the "nothing open at all" early return below --
        # a scan with no gap is precisely a scan with a blocker in it.
        yaw = self._laser_to_base_yaw(msg.header.frame_id)
        if yaw is None:
            return
        if self._avoid_enable:
            self._update_lidar_blocker(msg, yaw)

        def open_at(i):
            r = rng[i % n]
            return (r != r) or r > self._open_gap_min_range_m

        if all(open_at(i) for i in range(n)):
            start, best_len = 0, n                 # everything open: straight on
        else:
            # Start from a CLOSED ray so the walk cannot begin mid-gap, then sweep
            # once around, which makes a wrapping gap a single contiguous run.
            origin = next(i for i in range(n) if not open_at(i))
            best_len, best_start = 0, None
            run_len, run_start = 0, None
            for k in range(1, n + 1):
                i = origin + k
                if open_at(i):
                    if run_start is None:
                        run_start = i
                        run_len = 0
                    run_len += 1
                    if run_len > best_len:
                        best_len, best_start = run_len, run_start
                else:
                    run_start, run_len = None, 0
            if best_start is None:
                return                              # nothing open at all
            start = best_start
        mid = start + best_len / 2.0
        laser_bearing = msg.angle_min + mid * msg.angle_increment

        with self._scan_lock:
            self._open_bearing_rad = _wrap_angle(laser_bearing + yaw)
            self._open_bearing_at = time.monotonic()

    def _update_lidar_blocker(self, msg, laser_to_base_yaw):
        """Nearest lidar return that is actually in our way, in the BASE frame.

        Bearings are rotated by the TF yaw for the same reason the gap search is
        (N1): the laser is mounted at ~179 deg, so an unrotated bearing is very
        nearly a point reflection, and a steering law fed one would lean toward the
        obstacle instead of away from it -- confidently, and in the direction that
        makes contact more likely rather than less.
        """
        polar = []
        engage = self._avoid_config.engage_m
        r_min = max(float(msg.range_min), 1e-3)
        r_max = float(msg.range_max)
        for i, r in enumerate(msg.ranges):
            if r != r:                      # NaN: no return on this bearing
                continue
            if r < r_min or r > r_max or r >= engage:
                continue
            bearing = _wrap_angle(
                msg.angle_min + i * msg.angle_increment + laser_to_base_yaw)
            polar.append((float(r), bearing))
        blocker = corridor_blocker(polar, self._avoid_config)
        with self._scan_lock:
            self._lidar_blocker = blocker
            self._lidar_blocker_at = time.monotonic()

    def _on_camera_cloud(self, msg):
        """Nearest low-obstacle MARK in our way. Read-only; nothing here brakes.

        The range filter inside `camera_points_to_polar` is the load-bearing part:
        this cloud carries clear-ray endpoints at 1.8 m mixed in with real marks.
        """
        try:
            import sensor_msgs_py.point_cloud2 as pc2
            pts = [
                (float(p[0]), float(p[1]))
                for p in pc2.read_points(msg, field_names=("x", "y"), skip_nans=True)
            ]
        except Exception:
            pts = []
        blocker = corridor_blocker(
            camera_points_to_polar(pts, self._avoid_config), self._avoid_config)
        with self._scan_lock:
            self._camera_blocker = blocker
            self._camera_blocker_at = time.monotonic()

    def _nearest_blocker(self, max_relevant_m):
        """The nearer of the two sensors' blockers, ignoring stale ones.

        A stale camera must degrade to lidar-only steering, exactly as the camera
        BRAKE degrades to lidar-only braking on a stale cloud. Same freshness bound
        (`camera_max_age_s` 0.6) so the two layers agree about when the camera is
        speaking.
        """
        now = time.monotonic()
        with self._scan_lock:
            lidar, lidar_at = self._lidar_blocker, self._lidar_blocker_at
            camera, camera_at = self._camera_blocker, self._camera_blocker_at
        best = None
        if (lidar is not None and lidar_at is not None
                and now - lidar_at <= self._open_bearing_max_age_s):
            best = lidar
        if (camera is not None and camera_at is not None
                and now - camera_at <= self._avoid_camera_max_age_s):
            if best is None or camera[0] < best[0]:
                best = camera
        if best is not None and best[0] > max_relevant_m:
            return None
        return best

    def _laser_to_base_yaw(self, laser_frame):
        """Yaw of the laser frame in base_link, FROM TF. Never hardcoded."""
        try:
            tf = self._tf_buffer.lookup_transform(
                self._base_frame, laser_frame or "laser", rclpy.time.Time())
        except Exception:
            return None
        q = tf.transform.rotation
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _open_bearing(self):
        """Base-frame bearing of the widest gap, or **None when we do not have one**.

        Stale-bounded because steering toward a remembered gap after the rover has
        turned is how an escape drives into a wall it already passed.

        NONE, NOT 0.0. This used to answer 0.0 for "no scan yet", "scan too old" and
        "the way out is dead ahead" alike, and every consumer had to guess which one
        it meant. The ladder now DECIDES ITS ESCAPE ORDER from this value -- pivot
        toward the gap when a gap is known, reverse out along the entry path when it
        is not -- so a fabricated dead-ahead bearing would send the rover pivoting
        toward a direction nothing measured. The owner of the fact publishes the fact,
        including its absence.
        """
        with self._scan_lock:
            if (self._open_bearing_at is None
                    or time.monotonic() - self._open_bearing_at > self._open_bearing_max_age_s):
                return None
            return self._open_bearing_rad

    def _output_moving(self):
        with self._out_lock:
            return self._out_moving

    def _freeze_mark_pose(self, robot_x, robot_y, robot_yaw):
        """Where the mark goes: the frozen footprint's LEADING EDGE, not the centre.

        Implementation drift, found on 2026-08-11 and corrected against the approved
        design (design_d25_freeze.md, now committed to docs/): the mark was stamped at
        the robot's centre, so every mark sat `footprint_front_m` (0.11 m deployed)
        BEHIND the obstacle it marked, along the approach heading. The costmap got a
        point where the robot was standing rather than where the thing it hit was --
        so an approach from a slightly different angle reached the same physical
        object without ever crossing a mark. That is part of the contact-by-contact
        face-walking seen in gauntlet run 20260811_093818 against Scott's chair.
        """
        return (robot_x + self._footprint_front_m * math.cos(robot_yaw),
                robot_y + self._footprint_front_m * math.sin(robot_yaw))

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
        # GEOMETRY, per the approved design: one mark is a DISC of radius
        # ~robot_radius, not a point. Rationale from the design note: we know the
        # robot could not pass here, and we do NOT know the obstacle's true extent --
        # so mark the footprint we proved is blocked rather than guessing at the
        # object. A single point marks neither.
        #
        # z at the lidar plane so the costmap's height filter keeps them: these stand
        # for an obstacle the lidar CANNOT see, so they must be presented at a height
        # it would have accepted.
        pts = []
        for m in live:
            pts.append((m.x, m.y))
            for k in range(self._mark_disc_points):
                a = 2.0 * math.pi * k / self._mark_disc_points
                for frac in (0.5, 1.0):
                    r = self._mark_radius_m * frac
                    pts.append((m.x + r * math.cos(a), m.y + r * math.sin(a)))
        msg.width = len(pts)
        msg.row_step = 12 * len(pts)
        msg.data = b"".join(struct.pack("<fff", px, py, 0.15) for px, py in pts)
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
            if self._ladder_goal is None:
                # First journey only. Journey boundaries now arrive from the
                # explorer's goal_generation topic (see _on_goal_generation); the
                # 0.15 m endpoint proxy that used to decide this is retired, because
                # it could not tell a NEW clustered goal from a replan and starved
                # new goals of their escape budget.
                # A genuinely new destination gets a fresh invocation budget; a
                # same-destination replan must NOT, or bt_navigator's ~1 Hz replan
                # would reset the anti-livelock counter forever.
                self._ladder.reset_goal()
                self._ladder_goal = (goal_x, goal_y)
                # A new journey starts pointed at its own path, not carrying the
                # lean it had while dodging something on the last one.
                self._avoid_offset_rad = 0.0
            ladder = self._ladder

        try:  # NOTE: the finally below abandons any rung in progress (F10).
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

                # GENTLE TURN-AWAY. Lean the target heading away from whatever is in
                # the corridor, so the curve is already underway when the rover
                # reaches the distance at which the brake would otherwise zero it.
                # This changes only the heading fed to compute_drive_command: the
                # regimes below, the ladder's predicate and the supervisor's final
                # say are all untouched. `ladder.active` bypasses it because a rung
                # is what happens after normal control has already failed, and two
                # authors for one motion is the failure the ladder ended.
                if self._avoid_enable:
                    self._avoid_offset_rad = avoidance_heading_offset(
                        self._nearest_blocker(distance_to_goal),
                        self._open_bearing(),
                        self._avoid_offset_rad,
                        ladder.active,
                        self._avoid_config,
                    )
                    heading_error = _wrap_angle(heading_error + self._avoid_offset_rad)
                self._avoid_offset_pub.publish(
                    Float32(data=float(self._avoid_offset_rad)))

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
                    fx, fy = self._freeze_mark_pose(robot_x, robot_y, robot_yaw)
                    self._record_freeze(fx, fy, now_s)
                if ladder_result.exhausted:
                    # Say WHICH kind of dead end this was. "Genuinely wedged" means
                    # the supervisor refused every rung outright -- the room has us
                    # surrounded, and there is no bug to hunt. "Ineffective" means we
                    # were permitted to move and it did not help, which IS worth
                    # hunting. Reporting both as "aborted" is how a stack bug hides
                    # behind a tight room, and vice versa.
                    if ladder_result.reason == "goal_traversal_ceiling":
                        # A DIFFERENT FACT from the budget message below, and it must
                        # not borrow its wording: nothing was refused and nothing
                        # failed here. The rover escaped this goal's stalls the full
                        # number of times allowed and kept finding new ones, which is
                        # a verdict on the goal.
                        self.get_logger().warn(
                            "decisive_controller: this goal has now run "
                            f"{self._ladder_config.max_total_traversals_per_goal} "
                            "complete escape ladders in different places and keeps "
                            "stalling somewhere new. Aborting the GOAL — the escapes "
                            "worked; the destination is the problem."
                        )
                    elif ladder_result.budget_exhausted:
                        self.get_logger().warn(
                            "decisive_controller: this goal's escape budget was "
                            "already spent — NOTHING was tried on this goal, so "
                            "nothing was permitted or refused. Aborting. If this "
                            "repeats at one pose, the budget is not being reset "
                            "between goals."
                        )
                    elif ladder_result.genuinely_wedged:
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
                    # THROTTLE THE REPETITION, NEVER THE TRANSITIONS. One throttled
                    # call used to serve both, and since a rung emits ~30 `_running`
                    # lines in its 3 s budget, the one-shot `{rung}_failed->{next}`
                    # line almost always landed inside another line's 1 s shadow and
                    # was dropped. Run 114626 shows the damage: `reverse_arc_running`,
                    # `pivot_open_running` and `drive_open_running` all appear in the
                    # log and NOT ONE `_failed->` transition does, so the record shows
                    # rungs 2-4 executing with no trace of how they were entered. The
                    # escalations are the ladder's entire testimony and they are rare
                    # by construction; there is nothing to throttle.
                    message = (f"ladder: {ladder_result.reason} "
                               f"({ladder_result.linear_x:+.2f}, "
                               f"{ladder_result.angular_z:+.2f})")
                    if ladder_result.reason.endswith("_running"):
                        self.get_logger().info(message, throttle_duration_sec=1.0)
                    else:
                        self.get_logger().info(message)
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
                    # F10. Drop any rung still in progress. The ladder deliberately
                    # SURVIVES a same-destination replan -- resetting on every
                    # bt_navigator replan would clear the anti-livelock budget about
                    # once a second -- but a half-run rung must not be inherited by
                    # the next execute loop, which would resume someone else's escape
                    # against a stale reference pose and a stale clock. Only the
                    # still-active goal does this, for the same reason it owns the
                    # stop: a superseded loop must not reach into the live one.
                    self._ladder.abandon_rung()

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
