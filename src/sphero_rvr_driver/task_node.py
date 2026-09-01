"""The thin task surface: `goto`, `observe`, `query_semantic_map`. Track 2 v1.

Three tools over capabilities that are already hardware-validated, exposed as plain
ROS services and actions so they are callable with `ros2 service call` /
`ros2 action send_goal` and nothing else is required to drive the robot. That is the
Stage D acceptance in structural form: **delete the LLM and a working robot
remains.** A language model, when it arrives, is a client of this node -- it is not
inside it, and this file imports nothing that knows what a model is.

THE SAFETY BOUNDARY IS THE SHAPE OF THIS FILE. It imports no `geometry_msgs`, holds
no `Twist`, publishes no velocity, and names no `/cmd_vel*` topic. Motion is
requested exclusively as a map-frame `NavigateToPose` goal; Nav2 plans it, the
stock controller drives it, and beneath all of that the collision/STOP/ESTOP
supervisor stays the sole `/cmd_vel_motor` publisher and the final speed gate. A tool
surface that could publish a velocity would be a second controller racing the first
-- this stack's documented way to make a control bug look like a perception bug.
`tests/test_task_node.py` asserts this boundary by scanning this source, a test
lifted from the culled prompt-drive suite where it did the same job.

INTERFACE CHOICES, because they were forced and are worth stating:
* `task/goto` is an ActionServer of type `nav2_msgs/action/NavigateToPose` -- the
  same type it forwards to. That is deliberate, not lazy. The semantics match
  exactly ("go to this map pose"), it costs no new interface package, an action
  gives cancellation of a long drive for free, and `error_code`/`error_msg` carry a
  typed failure. Best of all it lets this node REUSE the caller's `PoseStamped`
  object for the downstream goal, so `geometry_msgs` never has to be imported here
  and the safety boundary above is structural rather than a promise.
* `task/observe` is a `Trigger` -- it takes no arguments, so nothing is lost.
* `task/query_semantic_map` is a `Trigger` whose arguments come from TYPED
  parameters (`query_label`, `query_radius_m`, `query_near_x/y`,
  `query_min_confidence`), read under the same lock that answers. Only `Trigger`,
  `SetBool` and `Empty` exist without generating a custom interface package, and
  none carries a string; a `.srv` of our own is the honest v2 upgrade, WHEN the LLM
  client lands and not before.
  These started as ONE `query_json` string parameter, which is the obvious design
  and is unusable: `ros2 param set` SEGFAULTS on any value starting with `{`
  (it YAML-parses the value, a `{...}` becomes a dictionary, and dictionaries are
  not a parameter type). Reproduced twice on jazzy -- a plain string sets fine,
  `{"label": "shoe"}` dumps core. Typed parameters are also simply better ROS:
  each is self-describing and range-checkable. The lock covers the SNAPSHOT read
  only; the parameters are read just before it. So one caller setting its own
  arguments then calling is consistent, but two concurrent callers can interleave
  set-then-call and must not be assumed safe.

Everything decidable without ROS -- the goal envelope, the semantic query, the
result shape -- lives in `sphero_rvr_core.task_tools`.
"""

import json
import math
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import Spin
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from nav2_msgs.srv import ClearEntireCostmap
from slam_toolbox.srv import Reset as SlamReset
from std_msgs.msg import Empty, String
from std_srvs.srv import Trigger
import tf2_ros

from sphero_rvr_core.relative_motion import (RelativeMotionError, describe,
                                             relative_goal)
from sphero_rvr_core.task_tools import (
    assemble_capabilities,
    describe_mission,
    EnvelopeError,
    GoalEnvelope,
    query_semantic_objects,
    tool_result,
    validate_goal,
)


class TaskNode(Node):
    def __init__(self):
        super().__init__("task_node")
        self.declare_parameter("max_goal_distance_m", 5.0)
        self.declare_parameter("max_query_radius_m", 10.0)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        # plan_timeout_s bounds the optional reachability precheck; goal_timeout_s
        # bounds the drive itself, so a goal that neither succeeds nor fails cannot
        # hold the action open forever.
        self.declare_parameter("plan_timeout_s", 2.0)
        self.declare_parameter("goal_timeout_s", 120.0)
        # Ask the planner before committing. Cheap -- one query against the planner
        # the goal would go to anyway -- and it turns "drove at a wall for two
        # minutes then aborted" into an immediate typed refusal. False = send blind.
        self.declare_parameter("precheck_reachable", True)
        # move_relative arguments, typed for the same reason query_* are:
        # `ros2 param set` segfaults on a JSON string, and a typed scalar is
        # self-describing and range-checkable.
        self.declare_parameter("move_distance_m", 0.0)
        self.declare_parameter("move_heading_deg", 0.0)
        # A LIVENESS BOUND, RE-DERIVED 2026-08-31 AFTER THE FIRST DERIVATION WAS
        # FOUND TO DESCRIBE A WORLD THAT CANNOT OCCUR.
        #
        # The first version reasoned: a pose stale by dt puts the goal
        # max_forward_mps * dt from where it was meant, so bound dt at
        # xy_goal_tolerance / max_forward_mps = 0.12 / 0.35 = 0.34 s. Real
        # arithmetic, real constants, and answering a question that never arises:
        # this verb REFUSES while a drive is running (it inherits task/goto's
        # one-at-a-time lock), so THE ROVER IS STATIONARY AT DISPATCH BY
        # CONSTRUCTION. A 0.35 s-old pose of a stationary rover is exactly as
        # accurate as a fresh one. The Pi found it by refusing a 5 cm contract
        # test for staleness under ordinary load.
        #
        # The guard's real subject is LIVENESS -- is this pose from a publisher
        # that is still alive -- and that is a different quantity entirely.
        # MEASURED on the rig (235 samples of map->base_link age, 1 Hz):
        #     idle       median 0.0019  p99 0.0027  max 0.109
        #     under load median 0.0524  p99 0.0534  max 0.053   (a colcon build,
        #                                                        run as the variable)
        # Worst healthy observation across both: 0.109 s. slam_toolbox republishes
        # map->odom at transform_publish_period 0.02, so the flight chain is
        # refreshed at 50 Hz and should not be slower than the rig's.
        # 0.5 s is ~4.6x the worst healthy sample, inside this project's loosest
        # existing staleness idiom (the supervisor's max_scan_stamp_age_s 0.75),
        # and a DEAD publisher crosses it within half a second. It bounds silence,
        # not motion.
        self.declare_parameter("move_pose_max_age_s", 0.5)
        self.declare_parameter("mission_start_service",
                               "/coverage_explorer/mission/start")
        self.declare_parameter("mission_stop_service",
                               "/coverage_explorer/mission/stop")
        self.declare_parameter("mission_status_topic", "/coverage_explorer/status")
        # The explorer publishes status at 1 Hz. Three missed publications is a node
        # that has stopped answering, not a slow one -- and saying so is the whole
        # value of the tool at that moment.
        self.declare_parameter("mission_status_max_age_s", 3.0)
        self.declare_parameter("observe_service", "observe")
        self.declare_parameter("observe_timeout_s", 30.0)
        self.declare_parameter("semantic_objects_topic", "/semantic_map/objects")
        # Arguments for task/query_semantic_map. See the interface note above for why
        # these are typed parameters rather than one JSON string.
        self.declare_parameter("query_label", "")          # "" = everything
        self.declare_parameter("query_radius_m", 0.0)      # 0 = unbounded
        self.declare_parameter("query_near_x", 0.0)
        self.declare_parameter("query_near_y", 0.0)
        self.declare_parameter("query_min_confidence", 0.0)

        self._envelope = GoalEnvelope(
            max_goal_distance_m=float(self.get_parameter("max_goal_distance_m").value),
            max_query_radius_m=float(self.get_parameter("max_query_radius_m").value),
        )
        self._map_frame = str(self.get_parameter("map_frame").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._plan_timeout_s = float(self.get_parameter("plan_timeout_s").value)
        self._goal_timeout_s = float(self.get_parameter("goal_timeout_s").value)
        self._observe_timeout_s = float(self.get_parameter("observe_timeout_s").value)
        self._mission_status_max_age_s = float(
            self.get_parameter("mission_status_max_age_s").value)
        self._precheck = bool(self.get_parameter("precheck_reachable").value)

        cbg = ReentrantCallbackGroup()
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._nav = ActionClient(self, NavigateToPose, "navigate_to_pose",
                                 callback_group=cbg)
        self._planner = ActionClient(self, ComputePathToPose, "compute_path_to_pose",
                                     callback_group=cbg)
        self._observe_client = self.create_client(
            Trigger, str(self.get_parameter("observe_service").value),
            callback_group=cbg,
        )

        self._objects_lock = threading.Lock()
        # Guards the mission-status snapshot only. Deliberately NOT the
        # objects lock: a 1 Hz subscriber must not queue behind a query
        # that can be waiting on a semantic-map answer.
        self._status_lock = threading.Lock()
        self._objects_json = ""
        self.create_subscription(
            String, str(self.get_parameter("semantic_objects_topic").value),
            self._on_objects, 10, callback_group=cbg,
        )

        # One goto at a time. Two concurrent gotos would put two NavigateToPose goals
        # on one robot -- the failure class the explorer's tick serialization exists
        # to prevent. A second caller is REFUSED, not queued: a queued drive would
        # start at an unpredictable later moment, which is worse than a clear "busy".
        self._goto_busy = threading.Lock()

        self._goto_server = ActionServer(
            self, NavigateToPose, "task/goto",
            execute_callback=self._execute_goto,
            goal_callback=self._accept_goto,
            cancel_callback=lambda _gh: CancelResponse.ACCEPT,
            callback_group=cbg,
        )
        # MISSION TOOLS (Track 2 v2). Forwarding, not invention: the coverage explorer
        # already owns arming, goal selection and ending, and already exposes them as
        # plain Triggers. What this adds is a NAME a language model can reach.
        self._explore_client = self.create_client(
            Trigger, str(self.get_parameter("mission_start_service").value),
            callback_group=cbg)
        self._stop_client = self.create_client(
            Trigger, str(self.get_parameter("mission_stop_service").value),
            callback_group=cbg)
        self._mission_status = None            # (monotonic_at, payload dict)
        self.create_subscription(
            String, str(self.get_parameter("mission_status_topic").value),
            self._on_mission_status, 10, callback_group=cbg)
        self.create_service(Trigger, "task/explore", self._on_explore,
                            callback_group=cbg)
        self.create_service(Trigger, "task/stop", self._on_stop,
                            callback_group=cbg)
        self.create_service(Trigger, "task/status", self._on_status,
                            callback_group=cbg)
        self.create_service(Trigger, "task/observe", self._on_observe,
                            callback_group=cbg)
        self.create_service(Trigger, "task/query_semantic_map", self._on_query,
                            callback_group=cbg)

        # --- the bridge round 1 (design_llm_verb_bridge_2026-08-20) ---------------
        # turn: a CLIENT of the supervisor's precise-turn gateway. The node checks
        # sanity (|degrees| <= 180, authoritative re-check of the contract's bound);
        # the gateway's admission remains the safety layer and its refusal comes
        # back as an ok=false the model reads like any envelope answer.
        self.declare_parameter("turn_degrees", 0.0)
        # THE SELF-CALL, and it is deliberate. `move_relative` is a coordinate
        # transform in front of `task/goto`: it computes a map-frame destination and
        # then goes through THIS NODE'S OWN goto action, so the envelope check, the
        # plannability precheck, the one-goto-at-a-time lock and every downstream gate
        # are traversed in the existing code rather than copied beside it. A copy
        # would be a second author on the motion path; this is none.
        #
        # Safe only because the executor is MultiThreaded with reentrant groups --
        # the goto's execute callback runs on another thread while this handler waits.
        # That is asserted by test, not assumed: see the Pi-only proving test.
        self._self_goto = ActionClient(self, NavigateToPose, "task/goto",
                                       callback_group=cbg)
        self._turn_client = ActionClient(self, Spin, "/collision_stop/precise_turn",
                                         callback_group=cbg)
        self.create_service(Trigger, "task/turn", self._on_turn,
                            callback_group=cbg)
        # where_am_i: owner facts, read-only -- TF pose/heading + /map metadata.
        map_qos = QoSProfile(depth=1)
        map_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        map_qos.reliability = QoSReliabilityPolicy.RELIABLE
        self._map_meta = None
        self.create_subscription(OccupancyGrid, "/map", self._on_map, map_qos,
                                 callback_group=cbg)
        self.create_service(Trigger, "task/move_relative", self._on_move_relative,
                            callback_group=cbg)
        self.create_service(Trigger, "task/where_am_i", self._on_where_am_i,
                            callback_group=cbg)
        # look_and_recognize: forwards to the recognition node's verb.
        # ENABLED 2026-08-20 (the watcher-flip pattern, this flip is the
        # reviewed commit). Receipts: the re-cert sitting passed EVERY bar on
        # the redesigned schema — 23/23 schema-valid, zero false positives,
        # zero wrong confirms, camera down after all 23, R6a refusal+alive —
        # on top of the offline calibration (all gates, iteration 1/6) and
        # the zero-FP lineage (25+ old-schema + 24-frame corpus + 7 live
        # probes, zero fabrications ever). Full ledger:
        # docs/bench_card_recognition_2026-08-19.md (RE-CERT RESULTS).
        # WATCH-ITEM CONDITION (mirrors the watcher's first-flight clause):
        # on the first mission that uses this tool, every look_and_recognize
        # result gets inspected against its photo; one fabricated match or
        # wrong confirmed reverts this default same-day.
        self.declare_parameter("recognition_tool_enabled", True)
        self.declare_parameter("recognition_target", "")
        self._recognition_params = self.create_client(
            SetParameters, "/recognition/set_parameters", callback_group=cbg)
        self._recognition_call = self.create_client(
            Trigger, "/recognition/look_and_recognize", callback_group=cbg)
        self.create_service(Trigger, "task/look_and_recognize",
                            self._on_look_and_recognize, callback_group=cbg)
        # clear_map (Scott's order 2026-08-21: pick the rover up, put it in a
        # different room, have it work). ONE action, built-in-first: slam_toolbox's
        # own Reset + Nav2's own clear_entirely services; the only custom part is
        # the /map_clear EVENT, which our own state owners (contact_marker,
        # coverage_explorer) subscribe to and honor THEMSELVES — no node reaches
        # into another's memory. Forget-and-rebuild only; no persistence, no
        # multi-map.
        self.declare_parameter("slam_reset_service", "/slam_toolbox/reset")
        self._map_clear_pub = self.create_publisher(Empty, "/map_clear", 1)
        self._slam_reset = self.create_client(
            SlamReset, str(self.get_parameter("slam_reset_service").value),
            callback_group=cbg)
        self._clear_global = self.create_client(
            ClearEntireCostmap, "/global_costmap/clear_entirely_global_costmap",
            callback_group=cbg)
        self._clear_local = self.create_client(
            ClearEntireCostmap, "/local_costmap/clear_entirely_local_costmap",
            callback_group=cbg)
        self.create_service(Trigger, "task/clear_map", self._on_clear_map,
                            callback_group=cbg)
        # capabilities: the owner's own report of which tools are BACKED right
        # now (design_capability_reporting_2026-08-20, ratified). The preamble
        # catches missing interfaces; this catches present-but-backendless
        # tools, from the SAME clients and rules the handlers use at call time.
        # NOT a promise -- every handler's call-time refusal stays the
        # authority, fail-closed, unchanged.
        self.create_service(Trigger, "task/capabilities", self._on_capabilities,
                            callback_group=cbg)

        self.get_logger().info(
            "task_node ready — tools: task/goto (action), task/observe, "
            "task/query_semantic_map, task/explore, task/stop, task/status, "
            "task/turn, task/where_am_i, task/look_and_recognize, "
            "task/capabilities, task/clear_map"
            f" (envelope {self._envelope.to_json_dict()};"
            f" recognition tool enabled="
            f"{self.get_parameter('recognition_tool_enabled').value})"
        )

    # --- plumbing -------------------------------------------------------------

    def _on_objects(self, msg):
        with self._objects_lock:
            self._objects_json = msg.data

    def _robot_xy(self):
        """Pose in the map frame, or None. None is a refusal condition for goto (see
        validate_goal) rather than something to guess around."""
        try:
            tf = self._tf_buffer.lookup_transform(
                self._map_frame, self._base_frame, rclpy.time.Time()
            )
        except Exception:
            return None
        return tf.transform.translation.x, tf.transform.translation.y

    def _robot_pose_yaw(self):
        """(x, y, yaw) in the map frame, or None -- same refusal-not-guess rule
        as _robot_xy."""
        try:
            tf = self._tf_buffer.lookup_transform(
                self._map_frame, self._base_frame, rclpy.time.Time()
            )
        except Exception:
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return t.x, t.y, yaw

    def _robot_pose_yaw_aged(self):
        """(pose, age_seconds) or (None, None).

        `lookup_transform` with `Time()` asks for the LATEST transform, which can be
        arbitrarily old and says nothing about it. A relative move computed from a
        stale pose is measured from where the rover WAS -- the same class of silent
        wrongness as every other believed-but-unchecked fact this project has paid
        for. The age is returned so the caller can refuse rather than guess.
        """
        try:
            tf = self._tf_buffer.lookup_transform(
                self._map_frame, self._base_frame, rclpy.time.Time()
            )
        except Exception:
            return None, None
        t, q = tf.transform.translation, tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        stamp = tf.header.stamp
        age = (self.get_clock().now().nanoseconds
               - (int(stamp.sec) * 10**9 + int(stamp.nanosec))) / 1e9
        return (t.x, t.y, yaw), age

    # --- the bridge round 1 handlers ------------------------------------------

    def _map_source(self):
        """Who is publishing /map, and therefore what KIND of map this is.

        ASSERT, DON'T INFER (2026-08-21 honesty seam): in the console's first
        real session the model told Scott about "the map it has built" while a
        recorded map was being served back by map_server. The model could not
        have known better — where_am_i handed it size and known-percent with no
        provenance. The map's kind is not guessable from its contents; it IS the
        identity of its publisher, so that is what gets read, live, from the
        graph at answer time (a stack restart can change it).
        """
        try:
            pubs = self.get_publishers_info_by_topic("/map")
        except Exception:
            return "unknown", "cannot tell who is publishing the map"
        names = {p.node_name for p in pubs}
        if any("slam" in n for n in names):
            return "slam", ("LIVE SLAM map — it grows as the robot moves, and "
                            "starts small after a map clear")
        if any("map_server" in n for n in names):
            return "static", ("a RECORDED map being served back — it will NOT "
                              "grow no matter how far the robot drives; do not "
                              "describe it as a map the robot has built")
        if not names:
            return "none", "no publisher on /map at all"
        return "unknown", f"published by {sorted(names)}, which is neither slam nor map_server"

    def _on_map(self, msg):
        known = sum(1 for v in msg.data if v >= 0)
        source, source_note = self._map_source()
        self._map_meta = {
            "source": source,
            "source_note": source_note,
            "size_m": [round(msg.info.width * msg.info.resolution, 2),
                       round(msg.info.height * msg.info.resolution, 2)],
            "resolution_m": round(msg.info.resolution, 3),
            "known_pct": round(100.0 * known / max(1, len(msg.data)), 1),
        }

    def _on_where_am_i(self, request, response):
        pose = self._robot_pose_yaw()
        if pose is None:
            response.success = False
            response.message = tool_result(
                False, "where_am_i",
                "no map->base_link transform — the robot does not know where it "
                "is right now, and that is the answer")
            return response
        x, y, yaw = pose
        response.success = True
        response.message = tool_result(
            True, "where_am_i", "",
            x=round(x, 3), y=round(y, 3), yaw_deg=round(math.degrees(yaw), 1),
            map=self._map_meta or "no map received yet")
        return response

    def _on_move_relative(self, request, response):
        """Body-frame move, expressed as a map-frame destination and driven through
        THIS NODE'S OWN `task/goto`.

        NO NEW MOTION AUTHORITY. Everything that could refuse a goto still refuses
        this: the envelope, the plannability precheck, the one-goto-at-a-time lock,
        the trinity and the supervisor. What is new is the EXPRESSION -- "forward
        2 m" was previously trigonometry the language agent had to do for itself.

        AND IT IS A DESTINATION, NOT A PROMISE TO DRIVE STRAIGHT. The planner and
        controller choose the route through every gate they already enforce; this
        verb only says where the rover should end up.
        """
        distance = float(self.get_parameter("move_distance_m").value)
        heading = float(self.get_parameter("move_heading_deg").value)

        pose, age = self._robot_pose_yaw_aged()
        max_age = float(self.get_parameter("move_pose_max_age_s").value)
        if pose is not None and age is not None and age > max_age:
            response.success = False
            response.message = tool_result(
                False, "move_relative",
                f"pose is {age:.2f}s old (bound {max_age:.2f}s): a relative move "
                f"computed from a stale pose is measured from where the rover WAS",
                pose_age_s=round(age, 3))
            return response
        try:
            gx, gy = relative_goal(pose, distance, heading)
        except RelativeMotionError as exc:
            response.success = False
            response.message = tool_result(False, "move_relative", str(exc))
            return response

        if not self._self_goto.wait_for_server(timeout_sec=5.0):
            response.success = False
            response.message = tool_result(
                False, "move_relative", "task/goto is not available")
            return response

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self._map_frame
        goal.pose.pose.position.x = gx
        goal.pose.pose.position.y = gy
        goal.pose.pose.orientation.w = 1.0

        deadline = time.monotonic() + float(
            self.get_parameter("goal_timeout_s").value) + 10.0
        handle = self._await(self._self_goto.send_goal_async(goal), deadline)
        if handle is None or not handle.accepted:
            # BUSY SEMANTICS ARE INHERITED, NOT REINVENTED. `_accept_goto` rejects
            # when a drive is already running; the self-call sees that rejection and
            # reports it, so this verb cannot become a second path around the
            # one-active-goal rule.
            response.success = False
            response.message = tool_result(
                False, "move_relative",
                "task/goto refused the goal (a drive may already be running)",
                x=round(gx, 3), y=round(gy, 3))
            return response

        result = self._await(handle.get_result_async(), deadline)
        if result is None:
            handle.cancel_goal_async()
            response.success = False
            response.message = tool_result(
                False, "move_relative", "the drive did not finish in time",
                x=round(gx, 3), y=round(gy, 3))
            return response

        # THE INNER REASON IS CARRIED, NOT SWALLOWED. `task/goto` already answers in
        # this project's structured form -- including D64's classification when the
        # refusal came from goal legality -- and a generic "move failed" here would
        # throw away the only sentence that says WHY.
        inner = getattr(result.result, "error_msg", "") or ""
        succeeded = result.status == GoalStatus.STATUS_SUCCEEDED
        response.success = succeeded
        if succeeded:
            response.message = tool_result(
                True, "move_relative", f"moved {describe(distance, heading)}",
                x=round(gx, 3), y=round(gy, 3))
        else:
            response.message = tool_result(
                False, "move_relative",
                f"{describe(distance, heading)} did not complete",
                x=round(gx, 3), y=round(gy, 3), goto_said=inner)
        return response

    def _on_turn(self, request, response):
        degrees = float(self.get_parameter("turn_degrees").value)
        # AUTHORITATIVE re-check of the contract's sanity bound: the schema is a
        # convenience for the model; this node is the boundary.
        if not -180.0 <= degrees <= 180.0:
            response.success = False
            response.message = tool_result(
                False, "turn", f"degrees must be within [-180, 180], got {degrees}")
            return response
        if abs(degrees) < 1.0:
            response.success = False
            response.message = tool_result(
                False, "turn", "a turn under 1 degree is a no-op; not sent")
            return response
        if not self._turn_client.wait_for_server(timeout_sec=3.0):
            response.success = False
            response.message = tool_result(
                False, "turn", "the precise-turn gateway is not available")
            return response
        goal = Spin.Goal()
        goal.target_yaw = math.radians(degrees)
        handle = self._await(self._turn_client.send_goal_async(goal),
                             time.monotonic() + 5.0)
        if handle is None or not handle.accepted:
            response.success = False
            response.message = tool_result(
                False, "turn", "the gateway did not accept the turn")
            return response
        result = self._await(handle.get_result_async(), time.monotonic() + 12.0)
        if result is not None and result.status == GoalStatus.STATUS_SUCCEEDED:
            response.success = True
            response.message = tool_result(
                True, "turn", f"turned {degrees:+.0f} degrees (firmware-settled)")
        else:
            response.success = False
            response.message = tool_result(
                False, "turn",
                "the turn was REFUSED or did not settle — the robot's own "
                "admission refuses turns near obstacles or while a stop is "
                "held. Move first, or report this to the user.")
        return response

    def _on_look_and_recognize(self, request, response):
        if not bool(self.get_parameter("recognition_tool_enabled").value):
            response.success = False
            response.message = tool_result(
                False, "look_and_recognize",
                "this tool is DISABLED until its bench certification passes "
                "(docs/bench_card_recognition_2026-08-19.md). Report that "
                "honestly; do not guess at what the camera would see.")
            return response
        target = str(self.get_parameter("recognition_target").value).strip()
        if not target:
            response.success = False
            response.message = tool_result(
                False, "look_and_recognize", "no target set")
            return response
        if not self._recognition_params.wait_for_service(timeout_sec=3.0) or \
           not self._recognition_call.wait_for_service(timeout_sec=3.0):
            response.success = False
            response.message = tool_result(
                False, "look_and_recognize",
                "the recognition node is not running")
            return response
        p = Parameter()
        p.name = "target"
        p.value = ParameterValue(type=ParameterType.PARAMETER_STRING,
                                 string_value=target)
        setp = SetParameters.Request(parameters=[p])
        if self._await(self._recognition_params.call_async(setp),
                       time.monotonic() + 5.0) is None:
            response.success = False
            response.message = tool_result(
                False, "look_and_recognize", "could not set the target")
            return response
        result = self._await(self._recognition_call.call_async(Trigger.Request()),
                             time.monotonic() + 100.0)
        if result is None:
            response.success = False
            response.message = tool_result(
                False, "look_and_recognize",
                "the camera verb timed out (camera + cloud call can take a "
                "minute; this took longer)")
            return response
        # The verb's own message is already the structured JSON answer (or a
        # loud REFUSED with its reason) -- forward it verbatim, both fields.
        response.success = result.success
        response.message = result.message
        return response

    def _await(self, future, deadline):
        """Block until `future` resolves or `deadline` passes; None on timeout.

        Safe on a MultiThreadedExecutor with a reentrant group -- another thread
        services the action response while this one waits. Same discipline as the
        explorer, including its hard-won rule: whatever you stop waiting for, you
        must also cancel (see every caller below).
        """
        while not future.done():
            if time.monotonic() >= deadline:
                return None
            time.sleep(0.01)
        return future.result()

    # --- goto -----------------------------------------------------------------

    def _accept_goto(self, goal_request):
        """Refuse rather than queue when a drive is already running."""
        if self._goto_busy.locked():
            self.get_logger().warn("task/goto refused: a goto is already running")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _fail(self, goal_handle, reason, **fields):
        result = NavigateToPose.Result()
        result.error_code = 1
        result.error_msg = tool_result(False, "goto", reason, **fields)
        goal_handle.abort()
        self.get_logger().warn(f"task/goto refused: {reason}")
        return result

    def _execute_goto(self, goal_handle):
        # The accept callback can race two near-simultaneous goals, so the lock is
        # taken here too and is the real guard; the callback only makes the common
        # case a clean rejection instead of a wait.
        if not self._goto_busy.acquire(blocking=False):
            return self._fail(goal_handle, "another goto is already running")
        try:
            pose = goal_handle.request.pose
            gx, gy = pose.pose.position.x, pose.pose.position.y
            try:
                gx, gy = validate_goal(gx, gy, self._robot_xy(), self._envelope)
            except EnvelopeError as exc:
                return self._fail(goal_handle, str(exc), x=gx, y=gy)

            if self._precheck and not self._plannable(pose):
                return self._fail(
                    goal_handle, "planner found no path to that goal", x=gx, y=gy
                )

            if not self._nav.wait_for_server(timeout_sec=5.0):
                return self._fail(goal_handle, "navigate_to_pose server unavailable")

            outbound = NavigateToPose.Goal()
            # Reuse the caller's PoseStamped verbatim: it is already the right type,
            # and passing it through is what keeps geometry_msgs out of this file.
            outbound.pose = pose
            deadline = time.monotonic() + self._goal_timeout_s
            handle = self._await(self._nav.send_goal_async(outbound), deadline)
            if handle is None or not handle.accepted:
                return self._fail(goal_handle, "navigate_to_pose rejected the goal",
                                  x=gx, y=gy)

            result_future = handle.get_result_async()
            while not result_future.done():
                if goal_handle.is_cancel_requested:
                    handle.cancel_goal_async()
                    late = self._await(result_future, time.monotonic() + 5.0)
                    # F7. A cancel can race an arrival. If the drive actually
                    # SUCCEEDED in the grace window, say so -- reporting "cancelled"
                    # when the rover is standing on the goal is a small lie that
                    # would make a caller re-drive somewhere it already is.
                    if late is not None and late.status == GoalStatus.STATUS_SUCCEEDED:
                        arrived = NavigateToPose.Result()
                        arrived.error_msg = tool_result(
                            True, "goto", "arrived just as cancel was requested",
                            x=gx, y=gy, status="SUCCEEDED")
                        goal_handle.succeed()
                        self.get_logger().info(
                            "task/goto cancel raced an arrival — reporting arrived")
                        return arrived
                    goal_handle.canceled()
                    cancelled = NavigateToPose.Result()
                    cancelled.error_msg = tool_result(
                        False, "goto", "cancelled by caller", x=gx, y=gy
                    )
                    self.get_logger().info("task/goto cancelled — downstream cancelled")
                    return cancelled
                if time.monotonic() >= deadline:
                    # Stopped waiting, so stop the drive: an abandoned goal keeps
                    # driving, which is exactly how a mission reports done and the
                    # rover keeps moving (D13).
                    handle.cancel_goal_async()
                    self._await(result_future, time.monotonic() + 5.0)
                    return self._fail(
                        goal_handle,
                        f"goal did not finish within {self._goal_timeout_s:.0f}s "
                        "— cancelled",
                        x=gx, y=gy,
                    )
                time.sleep(0.02)

            status = result_future.result().status
            if status == GoalStatus.STATUS_SUCCEEDED:
                result = NavigateToPose.Result()
                result.error_msg = tool_result(
                    True, "goto", "arrived", x=gx, y=gy, status="SUCCEEDED"
                )
                goal_handle.succeed()
                self.get_logger().info(f"task/goto arrived at ({gx:.2f},{gy:.2f})")
                return result
            name = {
                GoalStatus.STATUS_ABORTED: "ABORTED",
                GoalStatus.STATUS_CANCELED: "CANCELED",
            }.get(status, f"status_{status}")
            return self._fail(goal_handle, f"navigation {name}", x=gx, y=gy,
                              status=name)
        finally:
            self._goto_busy.release()

    def _plannable(self, pose):
        """Ask the planner whether it can route there from the live pose.

        Fails CLOSED for this one request (no path -> refuse) but never latches:
        every call re-asks, so a planner hiccup costs one tool call, not a session.
        """
        if not self._planner.server_is_ready():
            if not self._planner.wait_for_server(timeout_sec=1.0):
                self.get_logger().warn(
                    "compute_path_to_pose unavailable — skipping the precheck"
                )
                return True   # degrade to sending blind rather than block all motion
        goal = ComputePathToPose.Goal()
        goal.goal = pose
        goal.use_start = False
        deadline = time.monotonic() + self._plan_timeout_s
        handle = self._await(self._planner.send_goal_async(goal), deadline)
        if handle is None or not handle.accepted:
            return False
        result = self._await(handle.get_result_async(), deadline)
        if result is None:
            handle.cancel_goal_async()   # stop what we stopped waiting for
            return False
        return len(result.result.path.poses) > 0

    # --- observe --------------------------------------------------------------

    def _on_mission_status(self, msg):
        try:
            payload = json.loads(msg.data)
        except (ValueError, TypeError):
            return                      # a malformed status is NOT a status
        with self._status_lock:
            self._mission_status = (time.monotonic(), payload)

    def _status_freshness(self):
        """(age_s, payload) of the mission status, or (None, None) if never
        received. ONE rule shared by task/status and task/capabilities so the
        report and the handler cannot drift apart (the ratified design's
        shared-predicate requirement)."""
        with self._status_lock:
            entry = self._mission_status
        if entry is None:
            return None, None
        at, payload = entry
        return time.monotonic() - at, payload

    # --- capabilities (design_capability_reporting_2026-08-20) ---------------

    def _capability_predicates(self):
        """tool -> (ready, why): the SAME clients and rules each handler uses
        at call time, snapshotted non-blocking. The handlers keep their own
        wait-with-grace at call time -- this reports the condition, the
        handlers enforce it, and both read the same objects so they cannot
        disagree about WHICH backend a tool needs."""
        pose = self._robot_pose_yaw()
        no_pose = "no map->base_link transform — the robot does not know where it is"
        age, _payload = self._status_freshness()
        if age is None:
            status = (False, "no mission status has ever been received — the "
                             "coverage explorer is not running, or not publishing")
        elif age > self._mission_status_max_age_s:
            status = (False, f"mission status is STALE ({age:.1f}s old, limit "
                             f"{self._mission_status_max_age_s:.0f}s)")
        else:
            status = (True, None)
        objects_topic = str(self.get_parameter("semantic_objects_topic").value)
        with self._objects_lock:
            have_snapshot = bool(self._objects_json)
        recognition_on = bool(self.get_parameter("recognition_tool_enabled").value)
        explorer_why = "%s unavailable — is the coverage explorer running?"
        return {
            "goto": (self._nav.server_is_ready() and pose is not None,
                     ("navigate_to_pose server unavailable"
                      if not self._nav.server_is_ready() else no_pose)),
            # move_relative is a transform in front of task/goto, so its readiness is
            # goto's readiness plus a pose to measure FROM -- the pose is not optional
            # here the way it is for a tool that takes absolute coordinates.
            "move_relative": (self._self_goto.server_is_ready() and pose is not None,
                              ("task/goto is not available"
                               if not self._self_goto.server_is_ready() else no_pose)),
            "turn": (self._turn_client.server_is_ready(),
                     "the precise-turn gateway is not available"),
            "observe": (self._observe_client.service_is_ready(),
                        "observe service unavailable (is semantic_map running "
                        "with a camera?)"),
            "query_semantic_map": (
                have_snapshot or self.count_publishers(objects_topic) > 0,
                f"no publisher on {objects_topic} and no snapshot held"),
            "explore": (self._explore_client.service_is_ready(),
                        explorer_why % "mission/start"),
            "stop": (self._stop_client.service_is_ready(),
                     explorer_why % "mission/stop"),
            "status": status,
            "where_am_i": (pose is not None, no_pose),
            "look_and_recognize": (
                recognition_on and self._recognition_params.service_is_ready()
                and self._recognition_call.service_is_ready(),
                ("this tool is DISABLED until its bench certification passes"
                 if not recognition_on else "the recognition node is not running")),
            # The costmap clears are the tool's core; a missing slam reset is
            # reported per call (a static-map rig clears marks and costmaps and
            # says honestly that no new map will grow).
            "clear_map": (self._clear_global.service_is_ready()
                          and self._clear_local.service_is_ready(),
                          "costmap clear services unavailable — is Nav2 running?"),
        }

    def _on_capabilities(self, request, response):
        """Which tools are BACKED right now, with reasons — the owner
        volunteering at instruction start what each handler would otherwise
        refuse one paid call at a time (flight 5's discovery tax). The stamp is
        this node's clock at assembly (consensus pin): a consumer holding this
        snapshot longer than seconds can SEE that it did."""
        response.success = True
        response.message = assemble_capabilities(
            self._capability_predicates(), time.strftime("%Y-%m-%dT%H:%M:%S"))
        return response

    def _on_clear_map(self, request, response):
        """Forget the map and every belief tied to it; start fresh from here.

        ORDER IS THE CORRECTNESS ARGUMENT: the touch layer's ObstacleLayer never
        un-marks a cell (measured 2026-08-10, contact_marking docstring), so the
        owners must clear FIRST — the /map_clear event empties contact_marker's
        set so its next republish is an empty cloud — and the costmaps are wiped
        LAST, after a settle that outlasts the marker's 2 Hz republish period.
        Clearing in the other order lets one stale republish re-paint dead marks
        into a costmap that can never unlearn them.
        """
        steps = {}
        self._map_clear_pub.publish(Empty())
        steps["owners_event"] = "published /map_clear"
        # COUPLING, stated (review nit 2026-08-21): this settle must outlast
        # contact_marker's republish PERIOD (its republish_hz param, deployed
        # default 2.0 Hz -> 0.5 s) so the marker's next tick after forgetting is
        # the EMPTY cloud before the costmaps are wiped below. 1.0 s = two
        # periods at the deployed rate. If republish_hz is ever lowered below
        # ~1 Hz, this constant must grow with it — the marker owns the rate,
        # this node cannot read another node's parameter without inventing a
        # seam, so the coupling lives here in words and in the run guide.
        time.sleep(1.0)
        if self._slam_reset.wait_for_service(timeout_sec=1.0):
            reply = self._await(self._slam_reset.call_async(SlamReset.Request()),
                                time.monotonic() + 5.0)
            # slam's OWN verdict, not just an arrived reply (sitting 2026-08-21:
            # the first version called any reply "ok" — an unread result code is
            # a fabricated success waiting to happen).
            if reply is None:
                steps["slam_reset"] = "requested but NOT confirmed within 5s"
            elif reply.result == SlamReset.Response.RESULT_SUCCESS:
                steps["slam_reset"] = "ok — new map starts at the current pose"
            else:
                steps["slam_reset"] = (f"slam_toolbox REFUSED the reset "
                                       f"(result={reply.result})")
        else:
            steps["slam_reset"] = ("slam_toolbox not running — no new map will "
                                   "grow (static-map configuration?)")
        costmaps_ok = True
        for name, client in (("global_costmap", self._clear_global),
                             ("local_costmap", self._clear_local)):
            if client.wait_for_service(timeout_sec=2.0):
                reply = self._await(
                    client.call_async(ClearEntireCostmap.Request()),
                    time.monotonic() + 5.0)
                ok = reply is not None
                steps[name] = "cleared" if ok else "clear NOT confirmed within 5s"
                costmaps_ok = costmaps_ok and ok
            else:
                steps[name] = "clear service unavailable"
                costmaps_ok = False
        response.success = costmaps_ok
        response.message = tool_result(
            costmaps_ok, "clear_map",
            ("map and map-tied beliefs cleared — mapping restarts from the "
             "current pose. The fresh map starts nearly empty and grows as the "
             "robot moves; a small map right now is normal."
             if costmaps_ok else
             "clear was INCOMPLETE — see steps; beliefs may be inconsistent"),
            steps=steps)
        self.get_logger().warn(f"task/clear_map: {steps}")
        return response

    def _forward_trigger(self, client, tool, service_label, success_message):
        """Call another node's Trigger and report what actually happened.

        The forwarded call's own success/message is passed through rather than
        replaced: this node did not do the work and must not claim to know better
        than the node that did.
        """
        if not client.wait_for_service(timeout_sec=2.0):
            return tool_result(
                False, tool,
                f"{service_label} unavailable — is the coverage explorer running?"), False
        result = self._await(client.call_async(Trigger.Request()),
                             time.monotonic() + 5.0)
        if result is None:
            return tool_result(
                False, tool, f"{service_label} did not answer within 5s"), False
        if not result.success:
            return tool_result(False, tool, result.message or "refused"), False
        return tool_result(True, tool, success_message, detail=result.message), True

    def _on_explore(self, request, response):
        """Start a coverage mission of the room.

        RETURNS WHEN THE MISSION STARTS, NOT WHEN IT FINISHES, and the message says so
        in words a model will repeat. A caller that assumes otherwise reports "I have
        explored the room" about one second into a ten-minute drive, which is the most
        likely way this tool produces a confidently wrong answer.
        """
        response.message, response.success = self._forward_trigger(
            self._explore_client, "explore", "mission/start",
            "mission STARTED — the robot is now exploring. This returns immediately; "
            "the mission runs until it finishes or is stopped. Call status() to follow it.")
        return response

    def _on_stop(self, request, response):
        """End the mission and drop the goal in flight.

        NOT AN EMERGENCY STOP, and the message says that too. This disarms the mission
        and cancels the current goal; the rover coasts to a halt through the normal
        command path. The collision supervisor owns real stopping and nothing here
        touches it. Someone who says "stop" in an emergency must not be left believing
        they hit a brake.
        """
        response.message, response.success = self._forward_trigger(
            self._stop_client, "stop", "mission/stop",
            "mission STOPPED and the current goal cancelled. This is not an emergency "
            "stop: the robot coasts to a halt and the collision supervisor is untouched.")
        return response

    def _on_status(self, request, response):
        """What the robot is doing right now.

        STALENESS IS THE ANSWER WHEN IT IS THE ANSWER. A status tool that serves the
        last good value when the publisher has gone quiet converts the single most
        informative symptom a stuck rover has — silence — into a reassuring report.
        So an absent or aged status is reported as UNAVAILABLE with its age, and never
        smoothed over.
        """
        age, payload = self._status_freshness()
        if age is None:
            response.success = False
            response.message = tool_result(
                False, "status", "no mission status has ever been received — the "
                "coverage explorer is not running, or not publishing")
            return response
        if age > self._mission_status_max_age_s:
            response.success = False
            response.message = tool_result(
                False, "status",
                f"mission status is STALE ({age:.1f}s old, limit "
                f"{self._mission_status_max_age_s:.0f}s) — the explorer has stopped "
                "publishing. Do not treat the values below as current.",
                stale_age_s=round(age, 1), last_known=payload)
            return response
        response.success = True
        response.message = tool_result(
            True, "status", describe_mission(payload),
            age_s=round(age, 2), **payload)
        return response

    def _on_observe(self, request, response):
        """Pass-through to the semantic_map node's existing `observe` Trigger.

        Deliberately adds nothing: that service is hardware-validated, it costs a
        cloud VLM call per invocation, and wrapping it in retries or policy here
        would be inventing behaviour the tool surface has no business owning.
        """
        if not self._observe_client.wait_for_service(timeout_sec=2.0):
            response.success = False
            response.message = tool_result(
                False, "observe", "observe service unavailable "
                "(is semantic_map running with a camera?)"
            )
            return response
        future = self._observe_client.call_async(Trigger.Request())
        result = self._await(future, time.monotonic() + self._observe_timeout_s)
        if result is None:
            response.success = False
            response.message = tool_result(
                False, "observe",
                f"observe did not answer within {self._observe_timeout_s:.0f}s",
            )
            return response
        response.success = bool(result.success)
        response.message = tool_result(
            bool(result.success), "observe", str(result.message)
        )
        return response

    # --- query_semantic_map ---------------------------------------------------

    def _on_query(self, request, response):
        """Answer from the latest `/semantic_map/objects` snapshot.

        Arguments come from the typed query_* parameters (see the interface note at
        the top); they are read here, then the lock covers taking a consistent
        snapshot of the objects JSON while the subscription may be replacing it.
        """
        label = str(self.get_parameter("query_label").value or "") or None
        radius = float(self.get_parameter("query_radius_m").value)
        min_conf = float(self.get_parameter("query_min_confidence").value)
        # radius 0 means "no proximity filter"; near is meaningless without it, so
        # the pair is passed together or not at all (validate_query enforces that).
        near = None
        if radius > 0.0:
            near = (float(self.get_parameter("query_near_x").value),
                    float(self.get_parameter("query_near_y").value))
        with self._objects_lock:
            snapshot = self._objects_json
        try:
            found = query_semantic_objects(
                snapshot,
                label=label,
                near=near,
                radius_m=radius if radius > 0.0 else None,
                min_confidence=min_conf if min_conf > 0.0 else None,
                envelope=self._envelope,
            )
        except EnvelopeError as exc:
            response.success = False
            response.message = tool_result(False, "query_semantic_map", str(exc))
            return response
        response.success = True
        response.message = tool_result(
            True, "query_semantic_map", "", count=len(found), objects=found,
            have_map=bool(snapshot),
        )
        return response


def main(args=None):
    rclpy.init(args=args)
    node = TaskNode()
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
