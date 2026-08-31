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

AND NO CLEARANCE FILTER RUNS AHEAD OF IT. There was one, and it was removed rather
than retuned, because the defect was structural rather than numeric:

    **A frontier borders unknown space by definition.** Unknown cells are exactly
    what a clearance probe cannot judge, so frontier goals sit at the bottom of the
    clearance distribution as a matter of geometry, not of clutter. Measured on the
    live costmap during gauntlet run 1 (2026-08-11): the frontier population's median
    clearance was **0.05 m — the probe floor**, and **0 of 125 frontiers** passed the
    then-deployed 0.35 m. Even at 0.10 m only 9% passed.

A filter that rejects frontier goals makes frontier exploration impossible in any
room, on any robot, at any threshold above the grid resolution — so there is no value
to tune it to. It was neutralised to 0.0 in config mid-gauntlet as the minimal safe
diff, and this is the removal that was owed afterwards.

What that filter was FOR is still a real problem: run 185048 drove to a pose with
0.22 m on two sides and could then neither plan nor escape. That is now the stall
ladder's job (docs/stall_survival_ladder.md) -- recovery, from a pose the rover is
actually in, instead of prediction about a pose it might go to. The one prediction
that survives is `blocked_start_check`, which asks whether the robot's OWN current
cell is wedged; it is not a filter on candidates and does not have the anti-frontier
property, because the rover's own pose is not a frontier.

tests/test_goal_clearance_filter_removed.py fails if any of this comes back.
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
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import BackUp, ComputePathToPose, NavigateToPose
from sphero_rvr_core.escape_outcome import (
    CLEARED as ESCAPE_CLEARED, DECLINED as ESCAPE_DECLINED, parse_outcome,
)
from std_msgs.msg import Bool, Empty, Int32, String
from std_srvs.srv import Trigger
import tf2_ros

from sphero_rvr_core.mission_report import (
    OUTCOME_COMPLETE,
    OUTCOME_BLOCKED_BY_UNSEEN_OBSTACLES,
    OUTCOME_GOALS_KEEP_FAILING,
    OUTCOME_STOPPED_BY_OPERATOR,
    OUTCOME_NO_PLANNABLE_TARGETS,
    OUTCOME_NO_TARGETS_FROM_START,
    OUTCOME_START_BLOCKED,
    build_report,
    empty_cycle_outcome,
    map_yaml_text,
    occupancy_grid_to_pgm,
)
from sphero_rvr_core.costmap_window import extract_window, format_window
from sphero_rvr_core.coverage_exploration import (
    VIEWPOINT_STANDOFF_M,
    CoverageConfig,
    candidate_goals,
    cell_center_world,
    cell_world_grid,
    is_frontier,
    point_clears_standoff,
    robot_start_blocked,
    stamp_coverage,
    world_grid,
)


#: Every map-tied mutable field and a factory for its fresh value — ONE registry
#: driving `_on_map_clear`, pinned by tests/test_map_clear_reset_completeness.py:
#: a zero-initialised field added to __init__ without a row here (or a reasoned
#: allowlist entry in the pin) fails CI instead of surviving a map clear as
#: stale belief. That drift is exactly how D61 happened once (the done latch),
#: and the registry's own first draft proved the risk: the hand-written reset it
#: replaced was missing twelve of these rows on day one.
MAP_CLEAR_RESETS = {
    "_map": lambda: None,            # the old room's grid IS the stale belief
    "_costmap": lambda: None,
    "_covered": set,
    "_stalled": dict,
    "_blocked_logged": lambda: False,
    "_blocked_until": float,
    "_unplannable_last_cycle": int,
    "_unstick_attempts": int,
    "_mission_start": lambda: None,
    "_goals_sent": int,
    "_goals_succeeded": int,
    "_goals_aborted": int,
    "_goals_aborted_after_recovery": int,
    "_goals_aborted_without_recovery": int,
    "_goals_stall_killed": int,
    "_goals_cancelled_at_end": int,
    "_planner_rejections": int,
    "_planner_queries": int,
    "_standoff_skips": int,
    "_reported": lambda: False,
    "_consecutive_failures": int,
    "_consecutive_freezes": int,
    "_failure_streak": list,
    "_pending_freezes": list,
    "_freeze_events": list,
    "_escape_events": list,
    "_escape_poses": list,
    "_active_goal_saw_ladder": lambda: False,
    "_goal_inflight": lambda: False,
    "_goal_start_pose": lambda: None,
    "_goal_start_time": lambda: None,
    "_mission_done": lambda: False,
    "_armed": lambda: False,
    "_consecutive_empty": int,
    "_ever_had_target": lambda: False,
    "_excluded_no_viewpoint": lambda: None,
    "_last_candidate_count": lambda: None,
}


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
        # D56: the freeze feed that exists on the STOCK stack. The decisive
        # controller's freeze_event topic has no publisher there, so freezes were
        # structurally invisible and three real discoveries were booked as stack
        # failures on 2026-08-19. The DRIVER owns the physical fact (its firmware
        # stall counter, already on /diagnostics at 10 Hz) -- consume the counter's
        # DELTAS, same key contact_marker reads, and classify a mid-mission stall
        # as the discovery it is. Counters, not levels: sampling rate stays out of
        # the correctness argument.
        self.declare_parameter("diagnostics_topic", "/diagnostics")
        self.declare_parameter("stall_counter_key", "motor_stall_events")
        # Only for REPORTING: at what separation two freeze events are one place.
        # CHANGE BOTH OR NEITHER -- it must equal decisive_controller_node's
        # `freeze_mark_merge_radius_m`, because the controller has already merged at
        # its own radius before publishing, and a report claiming "6 distinct
        # positions" at a different radius describes a set nothing ever computed.
        # tests/test_freeze_mark_reporting.py fails in CI if the two literals drift.
        # This node does NOT act on the value; nothing in goal selection reads it.
        self.declare_parameter("freeze_mark_merge_radius_m", 0.15)
        # A goal nearer than this is somewhere the rover already is: it succeeds
        # without moving, the target is still a target next tick, and it gets picked
        # again forever. Never issue one.
        self.declare_parameter("min_goal_distance_m", 0.30)
        # When targets remain but none plan FROM HERE, move and retry rather than
        # ending the mission -- planning is done from the live pose, so this is
        # usually about where the rover is standing, not about the room.
        self.declare_parameter("max_unstick_attempts", 4)
        # The give-up escape. Distance is derived in the controller from the
        # freeze-mark geometry that traps the rover (mark_radius 0.14 +
        # inflation_radius 0.16); it is repeated here only as the request,
        # and tests/test_escape_geometry.py fails in CI if the two drift or
        # if a config change makes 0.30 m too short to escape a mark.
        self.declare_parameter("escape_distance_m", 0.30)
        self.declare_parameter("escape_speed_mps", 0.10)
        # Sent to the controller as the action's time_allowance, and equal to the
        # controller's own give_up_escape_timeout_s default. 0.30 m at 0.10 m/s is
        # 3.0 s of motion granted at FULL speed; the supervisor routinely SLOWS
        # rather than refuses, so a 3.0 s ceiling would report `refused` for escapes
        # that were working. Design note §4/§6(d) carries the slip-margin cost.
        self.declare_parameter("escape_timeout_s", 6.0)
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
        # D50: where the mission report JSON lands. Same directory as the map by
        # default, because the two describe the same run and separating them means
        # someone eventually archives one without the other.
        self.declare_parameter(
            "mission_report_dir", os.path.expanduser("~/.ros/missions")
        )
        # D43 dump extent. 0.60 m at the deployed 0.05 m resolution is a 25x25 square
        # -- wide enough to show clear floor AROUND a lethal patch, which is the
        # feature that separates stale occupancy from a pose offset. A tighter window
        # shows only the blob and settles nothing.
        self.declare_parameter("costmap_dump_radius_m", 0.60)
        self.declare_parameter("costmap_topic", "/global_costmap/costmap")
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
        self._escape_distance_m = float(
            self.get_parameter("escape_distance_m").value)
        self._escape_speed_mps = float(
            self.get_parameter("escape_speed_mps").value)
        self._escape_timeout_s = float(
            self.get_parameter("escape_timeout_s").value)
        # Escape bookkeeping for the report: every event, and the distinct
        # places they happened.
        self._escape_events = []
        self._escape_poses = []
        self._unstick_backup_m = float(self.get_parameter("unstick_backup_m").value)
        self._unstick_spin_rad = float(self.get_parameter("unstick_spin_rad").value)
        self._unstick_timeout_s = float(self.get_parameter("unstick_timeout_s").value)
        self._unstick_attempts = 0
        map_topic = str(self.get_parameter("map_topic").value)

        self._map = None
        # Still subscribed, and still load-bearing: `blocked_start_check` reads it to
        # answer "is the ROBOT's own cell wedged". That is a question about one pose
        # the rover is already standing in, which is why it survived the removal of
        # the goal-clearance filter -- see the note above `_select_and_send`.
        self._costmap = None
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
        # D41: THE MISSION CLOCK STARTS WHEN THE MISSION ARMS, NOT WHEN THE NODE IS
        # READY. It used to be stamped right here, in __init__, which was correct
        # until D29 made the stack come up DISARMED and moved liftoff to a service
        # call. Nothing re-anchored this, so every mission since measured from
        # node-ready and over-reported: 2.94x on 2026-08-14b (638.2 s against 217.5 s
        # armed), 4.35x on 08-15a, 3.25x on 08-15b. Every derived RATE went with it,
        # which invalidated cross-run coverage-rate comparison in both directions --
        # the project's main better-or-worse metric.
        #
        # The class lesson (standards rule 2): a re-anchoring change obliges
        # re-checking every quantity measured from the old anchor. D29 moved the
        # anchor and this was the quantity nobody re-checked.
        #
        # None until armed, deliberately. A pre-arm stamp is what made the bug
        # possible, and `None` makes an un-armed report say "no mission clock"
        # instead of quietly measuring from bringup.
        self._mission_start = None
        self._goals_sent = 0
        self._goals_succeeded = 0
        # THE ABORT COUNTER, SPLIT (design reverse_before_give_up_design.md, amendment
        # 2026-08-13). `_goals_aborted` answered two questions with one number:
        #
        #   "we tried here and could not move"  -- a ladder exhaustion. Real evidence
        #                                          that the place is hard, and what
        #                                          ABORTED_GOALS_KEEP_FAILING is meant
        #                                          to mean.
        #   "no recovery ran at all"            -- e.g. the follow_path acknowledgement
        #                                          timeout of 2026-08-13, where the
        #                                          goal aborted with the controller
        #                                          busy on the PREVIOUS goal. Evidence
        #                                          about the stack, not about the room.
        #
        # A run with enough of the second ends ABORTED_GOALS_KEEP_FAILING having proved
        # far less than the count implies, and the report could not say so because the
        # two were one number.
        #
        # WHAT THE DISCRIMINATOR ACTUALLY IS, stated plainly because it is narrower than
        # the question: it is whether the CONTROLLER reported a recovery during this
        # goal, taken from the controller's own `ladder_active` topic. That separates
        # "recovery ran and was exhausted" from "no recovery ran". It does NOT separate
        # "never got to try" from "drove normally and failed" -- both land in
        # `without_recovery`, and neither is evidence the location is unreachable, which
        # is the property the ending depends on. Naming these for the signal rather than
        # for the question is deliberate: a field called `never_tried` would be a
        # capability word that outlives the thing that made it true (D35).
        self._goals_aborted_after_recovery = 0
        self._goals_aborted_without_recovery = 0
        self._goals_aborted = 0
        # D53: the watchdog stall-kill is a terminal path like any other and gets
        # its own named counter. Its result status is CANCELED, which the result
        # callback counts as neither ABORTED nor SUCCEEDED -- so until 2026-08-19
        # this class was invisible to every public counter, and the combined
        # ride-along ended "ABORTED_GOALS_KEEP_FAILING" with aborted=0 in every
        # bucket: a report an autopsy would read as "nothing failed".
        self._goals_stall_killed = 0
        # D62: goals cancelled IN FLIGHT by mission give-up/stop/map-clear. Until
        # 2026-08-22 these landed in no counter and the ledger did not total.
        self._goals_cancelled_at_end = 0
        self._planner_rejections = 0
        # the planner_rejections counter's honest siblings (cert attempt 3's
        # conflation): queries = times the planner was actually ASKED;
        # standoff_skips = ladder poses the safety envelope filtered before any
        # planner query was spent, on candidates that yielded no goal
        self._planner_queries = 0
        self._standoff_skips = 0
        self._reported = False
        self._consecutive_failures = 0
        self._consecutive_freezes = 0
        # The current failure STREAK, one (kind, (wx, wy)) per failure, cleared on
        # success. The breaker's epitaph is keyed on THESE poses -- the 2026-08-19
        # flight's breaker said "failed at different places -- this is the stack"
        # while the rover sat at ONE place failing the same physical way five
        # times: the goals were different places, the rover was not. Count kinds,
        # measure the pose spread, diagnose nothing.
        self._failure_streak = []
        # D56 freeze feed state: last driver stall count seen (None = no baseline
        # yet -- the first sample is a baseline, never an event).
        self._last_stall_count = None
        # Unconsumed freeze events, newest last. An event pairs with AT MOST ONE
        # abort: two aborts inside the correlation window of a single freeze must
        # not both be excused, or one discovery would silently forgive an unrelated
        # failure.
        self._pending_freezes = []
        self._active_goal_generation = 0
        # One entry per freeze EVENT, in order, never merged here. Merging is the
        # report's job and it happens once, at the radius the report also states
        # (D35). Named for what it holds: this list was `_freeze_marks`, and the
        # ambiguity in that word is the whole defect.
        self._freeze_events = []
        self._active_goal_cell = None
        self._active_goal_handle = None
        # HAS THIS GOAL ALREADY BEEN GIVEN AN ENDING? D75: the ledger's invariant is
        # that `sent` equals the sum of every named ending, which is only true if the
        # endings are MUTUALLY EXCLUSIVE. They were not: a stall-kill books
        # stall_killed AND then cancels the live handle, which booked
        # cancelled_at_end for the same goal -- one goal, two endings, and the
        # tightened d53 assertion could not hold. The single-incrementer discipline
        # was honoured per COUNTER and not per GOAL. This flag is the per-goal half.
        self._active_goal_ended = False
        # DID THE CONTROLLER ATTEMPT A RECOVERY ON THIS GOAL? Set from the controller's
        # own `ladder_active` publication -- never inferred from progress, a timer, or
        # the shape of the abort. See `_goals_aborted_without_recovery`.
        self._active_goal_saw_ladder = False
        self._goal_inflight = False
        self._goal_start_pose = None   # (wx, wy) when the active goal was sent
        self._goal_start_time = None   # time.monotonic() when the active goal was sent
        self._mission_done = False
        # D29: ARMED STATE. Until 2026-08-10 the mission began the instant the node
        # came up, so "bringup gates, THEN go" was not expressible: run 185048's
        # entire 53 s mission ran and died DURING the gate checks, and the operator
        # watched a stopped rover with no idea a mission had happened at all. Launch
        # is no longer liftoff -- a service is.
        self._armed = False
        self._consecutive_empty = 0
        self._ever_had_target = False
        # UNKNOWN until the first selection cycle counts it -- same
        # no-fabricated-zero discipline as build_report's remaining_candidates.
        self._excluded_no_viewpoint = None
        # None means NOT YET COUNTED. See _publish_status: a fabricated 0 here would
        # read as "nothing left to explore".
        self._last_candidate_count = None
        self._complete_after_empty = int(self.get_parameter("complete_after_empty_cycles").value)

        cbg = ReentrantCallbackGroup()
        map_qos = QoSProfile(depth=1)
        map_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        map_qos.reliability = QoSReliabilityPolicy.RELIABLE
        self.create_subscription(OccupancyGrid, map_topic, self._on_map, map_qos, callback_group=cbg)
        # The map-clear event (task/clear_map, 2026-08-21): coverage memory
        # describes a map that no longer exists after a clear, so this node —
        # the memory's owner — forgets it and unlatches `done`. This is ALSO the
        # legitimate answer to the restart-the-node re-arm refusal (D61): the
        # refusal exists to stop a second mission reusing the first one's
        # coverage set, and after a clear there is no set to reuse.
        self.create_subscription(Empty, "/map_clear", self._on_map_clear, 1,
                                 callback_group=cbg)
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
        # D56: the freeze feed that has a live publisher on the STOCK middle. Both
        # lanes stay subscribed -- bespoke runs this node WITH the decisive
        # controller, so one physical stall can arrive on both; _on_diagnostics
        # dedupes against pending events within the merge radius.
        self.create_subscription(
            DiagnosticArray, str(self.get_parameter("diagnostics_topic").value),
            self._on_diagnostics, 10, callback_group=cbg,
        )
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._nav = ActionClient(self, NavigateToPose, "navigate_to_pose", callback_group=cbg)
        self._planner = ActionClient(
            self, ComputePathToPose, "compute_path_to_pose", callback_group=cbg
        )
        # THE GIVE-UP ESCAPE, asked of the CONTROLLER (docs/reverse_before_give_up
        # _design.md). This replaces nav2_behaviors' backup and spin, which were
        # invoked correctly here and executed nothing: their collision check reads the
        # costmap that this rover's own freeze marks have made lethal, so on
        # 2026-08-12 they refused in 3 ms with 0.78 m of measured clear floor behind
        # (D36). The controller escapes through the collision supervisor instead,
        # which reads the live lidar -- the same reason the stall ladder's rungs are
        # ordinary drive commands and not Nav2 behaviours (D16).
        self._escape = ActionClient(
            self, BackUp, "/decisive_controller/escape_in_place", callback_group=cbg)
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
        # The journey boundary, published rather than inferred. The controller has
        # no notion of an explorer goal -- it sees only follow_path from
        # bt_navigator, which re-sends ~1 Hz while replanning ONE journey (the
        # 08-03 preemption bug). It was therefore guessing "new journey?" from goal
        # endpoint distance, and clustered goals shared one escape budget: five
        # consecutive goals each exhausted instantly with nothing tried, and the
        # rover sat motionless while the give-up counter emptied (run 103337).
        # We know the answer here; say it. Namespaced (~/) per the F3 lesson.
        self._generation_pub = self.create_publisher(Int32, "~/goal_generation", 10)
        # LIVE MISSION STATUS, 1 Hz. `~/report` is latched and only exists once a
        # mission has ENDED, so until this publisher there was no way to ask a running
        # rover what it was doing -- and the task surface's `status` tool would have
        # had to infer it from goal generations or topic liveness, which is the
        # proxy-inference class this project has paid for three times. The explorer
        # knows whether it is armed; it says so.
        self._status_pub = self.create_publisher(String, "~/status", 10)
        self.create_timer(1.0, self._publish_status, callback_group=cbg)
        self.create_timer(0.5, self._publish_generation)
        self.create_service(Trigger, "~/mission/start", self._on_mission_start,
                            callback_group=cbg)
        self.create_service(Trigger, "~/mission/stop", self._on_mission_stop,
                            callback_group=cbg)
        if bool(self.get_parameter("autostart").value):
            self._arm("autostart")
        self.get_logger().info(
            "coverage_explorer ready (coverage + frontier mission) — "
            + ("ARMED, mission running" if self._armed else
               "DISARMED, waiting for mission/start")
        )

    def _arm(self, reason):
        """ONE AUTHOR FOR ARMING, and the only place the mission clock starts.

        Both routes in -- the `mission/start` service and `autostart` -- come
        through here, because D41 happened precisely when one quantity was stamped
        somewhere other than where the mission actually began. Two arming paths
        setting the clock separately would rebuild that defect with an extra place to
        forget.

        It also emits the ARM line. Disarm has logged itself since D29; arming never
        did, so the single most important instant in a run -- the one every duration
        is measured from -- was the one instant absent from the launch log, and every
        after-the-fact alignment had to infer it from goal traffic.
        """
        self._armed = True
        self._mission_start = time.monotonic()
        self.get_logger().warn(f"MISSION ARMED ({reason}) — the mission clock starts now")

    def _mission_elapsed_s(self):
        """Seconds since ARM, or 0.0 if the mission never armed.

        A report from an un-armed node is describing a mission that did not run, so
        0.0 is the honest figure; the old code would have reported however long the
        node had been up.
        """
        if self._mission_start is None:
            return 0.0
        return time.monotonic() - self._mission_start

    def _publish_generation(self):
        self._generation_pub.publish(Int32(data=int(self._active_goal_generation)))

    def _on_ladder_active(self, msg):
        if msg.data:
            self._ladder_active_at = time.monotonic()
            # STICKY FOR THE LIFE OF THE GOAL. `_ladder_running()` is freshness-bounded
            # so a dead controller cannot hang the mission, but the abort arrives on an
            # async callback that can land well after the last rung -- the 2026-08-13
            # pairing showed terminal outcomes at 0.0 s and one at +1.8 s. Sampling the
            # live signal at abort time would classify by callback latency.
            with self._lock:
                if self._active_goal_cell is not None:
                    self._active_goal_saw_ladder = True

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

    def _dump_costmap_window(self, wx, wy):
        """Write the costmap around the blocked pose, and log it too.

        BOTH destinations on purpose. The file survives the session for an autopsy;
        the log line is what a tailing operator sees at the moment it happens, and it
        is also what ends up in the launch log every archived run already keeps. A
        dump that only exists as a file is a dump nobody notices was taken.

        NEVER RAISES. This is diagnostics attached to a failure path, and a
        diagnostic that can turn "the planner is blocked" into "the explorer crashed"
        is worse than no diagnostic -- the same rule the stuck-survey follows.
        """
        try:
            cm = self._costmap
            if cm is None:
                self.get_logger().warn("COSTMAP_DUMP unavailable: no costmap received")
                return
            window = extract_window(
                cm.data, cm.info.width, cm.info.height,
                cm.info.origin.position.x, cm.info.origin.position.y,
                cm.info.resolution, wx, wy,
                float(self.get_parameter("costmap_dump_radius_m").value),
            )
            text = format_window(window, wx, wy)
            self.get_logger().warn(text)
            # The SAME directory the mission maps go to, read from the deployed
            # parameter rather than hardcoded: a dump that lands somewhere else is a
            # dump the artifact-collection step does not sweep up, and every one of
            # these runs is archived by copying that directory.
            out_dir = os.path.expanduser(str(self.get_parameter("map_save_dir").value))
            os.makedirs(out_dir, exist_ok=True)
            path = os.path.join(
                out_dir, f"blocked_{time.strftime('%Y%m%d_%H%M%S')}.txt")
            with open(path, "w") as fh:
                fh.write(text + "\n")
            self.get_logger().warn(f"COSTMAP_DUMP written to {path}")
        except Exception as exc:                  # noqa: BLE001 - see docstring
            self.get_logger().warn(f"COSTMAP_DUMP failed: {exc}")

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
        self._arm("mission/start service")
        response.success = True
        response.message = "mission started"
        return response

    def _stop_requested(self):
        """True once mission/stop has disarmed us. Long-running recovery loops poll
        this so they abandon instead of finishing a manoeuvre for a stopped mission."""
        return not self._armed

    def _publish_status(self):
        """What this node is doing, once a second, as JSON.

        EVERY FIELD IS SOMETHING THIS NODE OWNS. No field is derived from another
        component's behaviour, and nothing here is a guess -- a consumer that wants to
        know about the supervisor asks the supervisor.

        `running` is armed-and-not-done rather than "a goal is in flight": a rover
        between goals is still exploring, and reporting it idle would make every
        selection cycle look like a stall.
        """
        with self._lock:
            payload = {
                "armed": bool(self._armed),
                "done": bool(self._mission_done),
                "running": bool(self._armed and not self._mission_done),
                "goal_in_flight": self._active_goal_cell is not None,
                "goal_cell": (list(self._active_goal_cell)
                              if self._active_goal_cell is not None else None),
                "goals_sent": self._goals_sent,
                "goals_succeeded": self._goals_succeeded,
                "goals_aborted": self._goals_aborted,
                "goals_aborted_after_recovery": self._goals_aborted_after_recovery,
                "goals_aborted_without_recovery": self._goals_aborted_without_recovery,
                "goals_stall_killed": self._goals_stall_killed,
                "goals_cancelled_at_end": self._goals_cancelled_at_end,
                "planner_rejections": self._planner_rejections,
                "standoff_skips": self._standoff_skips,
                "covered_cells": len(self._covered),
                "unstick_attempts": self._unstick_attempts,
                "escapes": len(self._escape_events),
                "freezes": len(self._freeze_events),
                # UNKNOWN, not zero. Candidates are counted during selection; between
                # cycles there is no fresh number, and a stale 0 reads as "nothing left
                # to explore", which is the most reassuring possible lie (D24).
                "remaining_candidates": self._last_candidate_count,
            }
        self._status_pub.publish(String(data=json.dumps(payload)))

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
        # D50: an operator-ended mission is still a mission that happened, and until
        # 2026-08-16 this path produced NO report by any route -- it disarmed and
        # returned. _finish is idempotent (guarded by _reported), so calling it here
        # records a stop that ends a running mission and does nothing to one that has
        # already reported its own outcome.
        if was:
            self._finish(
                OUTCOME_STOPPED_BY_OPERATOR,
                self._map.info.resolution if self._map is not None else 0.0,
                self._remaining_candidates(),
            )
        response.success = True
        response.message = "mission stopped" if was else "was not running"
        return response

    def _on_map_clear(self, _msg):
        """The map is being forgotten; forget everything measured against it.

        A running mission ends first, through the SAME stop-and-report path an
        operator stop takes (a cleared-away mission still happened and still
        reports). Then MAP_CLEAR_RESETS — the registry, not a hand list — puts
        every map-tied field back to its fresh value, so the next mission/start
        arms a genuinely fresh mission. Nothing here is another node's state,
        and no other node touches ours.

        (History, kept on purpose: the first landed version of this method was
        a hand-written reset list that (a) missed twelve map-tied fields and
        (b) had spliced three service-response lines into this SUBSCRIPTION
        callback — it logged its success line and then killed the whole node
        with a NameError, and the live cert read the log line instead of
        checking the survivor. The registry and its completeness pin exist so
        neither failure shape can land again.)
        """
        was = self._armed
        self._armed = False
        self._cancel_active()
        if was:
            self._finish(
                OUTCOME_STOPPED_BY_OPERATOR,
                self._map.info.resolution if self._map is not None else 0.0,
                self._remaining_candidates(),
            )
        for name, fresh in MAP_CLEAR_RESETS.items():
            setattr(self, name, fresh())
        self.get_logger().warn(
            "MAP CLEAR — coverage memory forgotten "
            f"({len(MAP_CLEAR_RESETS)} fields reset"
            + (", running mission stopped and reported" if was else "")
            + "); a fresh mission may be armed")

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
                # D43: SNAPSHOT THE COSTMAP AT THE INSTANT OF THE BLOCK. The two
                # candidate mechanisms -- a stale SLAM static layer, or a map-frame
                # pose offset -- are indistinguishable from outside and cannot be
                # separated after the fact from a bag that never recorded the grid.
                # Taken here, paired with the scan, one dump convicts one of them.
                self._dump_costmap_window(wx, wy)
                # Wedged is not a mission END -- freeing the rover resumes it -- so
                # this does NOT consume the terminal report latch. But it IS the
                # answer to "what is it doing", and it is actionable, so say it on the
                # topic rather than only in a log nobody is tailing. A later terminal
                # report overwrites it on the latched topic, which is what we want.
                self._report_pub.publish(String(data=json.dumps(build_report(
                    OUTCOME_START_BLOCKED,
                    covered_cells=len(self._covered),
                    resolution=res,
                    duration_s=self._mission_elapsed_s(),
                    goals_sent=self._goals_sent,
                    goals_succeeded=self._goals_succeeded,
                    goals_aborted=self._goals_aborted,
                    goals_aborted_after_recovery=self._goals_aborted_after_recovery,
                    goals_aborted_without_recovery=self._goals_aborted_without_recovery,
                    goals_stall_killed=self._goals_stall_killed,
                    goals_cancelled_at_end=self._goals_cancelled_at_end,
                    planner_rejections=self._planner_rejections,
                    standoff_skips=self._standoff_skips,
                    # THE FORENSIC FIELDS, which this call site omitted until
                    # 2026-08-16. Gauntlet mission 1 ended here, and its report carried
                    # `freeze_events: []` while the status line counted 5 and the log
                    # named four positions -- the run's central evidence, absent from
                    # the artifact the run is judged by. `remaining_candidates` was
                    # likewise unset and defaulted to a fabricated 0.
                    #
                    # This is not a terminal report: being wedged is not the end of a
                    # mission, and freeing the rover resumes it. But it IS the report
                    # that gets read when a run stops here, so it has to carry what
                    # happened.
                    remaining_candidates=self._remaining_candidates(),
                    cells_excluded_no_viewpoint=self._excluded_no_viewpoint,
                    freeze_events=list(self._freeze_events),
                    freeze_mark_merge_radius_m=float(
                        self.get_parameter("freeze_mark_merge_radius_m").value),
                    escape_events=list(self._escape_events),
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
                # D75 ORDERING: the stall-kill is booked BEFORE the cancel that
                # effects it. `_cancel_active` books cancelled_at_end only for a goal
                # with no ending yet, so if the cancel ran first it would take the
                # ending that belongs here -- the cancel is the MEANS of this ending,
                # not an ending of its own.
                with self._lock:
                    self._goals_stall_killed += 1
                    self._active_goal_ended = True
                self._cancel_active()
                # A watchdog cancel IS a failed drive and must count as one. Its
                # result status is CANCELED, which the result callback counts as
                # neither ABORTED nor SUCCEEDED -- so without counting it here,
                # stall -> cancel -> suppress -> reselect loops FOREVER: the
                # suppression disc expires after stall_suppress_ttl_s, the empty
                # counter resets on every send, and the give-up counter that exists
                # precisely to stop this thrash never moves. The rover sits nearly
                # still logging "made no progress" every few seconds indefinitely.
                #
                # D53: counted PUBLICLY too. Five of these ended the 2026-08-19
                # ride-along while status and report said aborted=0 everywhere.
                self._note_failure(active_cell, self._active_goal_generation,
                                   kind="stall_killed")
                if self._mission_done:
                    return  # that was the last straw; _finish already ran
            else:
                return  # making progress -> keep driving, do not reselect
        if inflight:
            return

        selection = candidate_goals(
            m.data, w, h, ox, oy, res, rcx, rcy, self._covered, set(self._stalled),
            self._config, viewpoint_standoff_m=VIEWPOINT_STANDOFF_M,
        )
        candidates = selection.candidates
        # Latest cycle's honest residue: clusters no safety-permitted pose can
        # cover. Rides into the report so COMPLETE never silently absorbs them.
        self._excluded_no_viewpoint = selection.excluded_no_viewpoint
        # Selection blocks on planner queries and can outlast the tick period. It
        # cannot re-enter -- the whole tick is serialized by _tick_busy -- which is
        # what keeps two selection loops from racing to send two goals: concurrent
        # goals fighting over one actuator is this stack's known way to produce
        # motion that looks like a perception failure (the 2026-08-03 follow_path
        # preemption bug).
        goal_cell = None
        goal_point = None
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
            ladder_queries_before = self._planner_queries
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
                # THE REFLEX ENVELOPE GATES FIRST (cert attempt 2, 2026-08-19):
                # an approach point the planner approves can still sit inside the
                # supervisor's frontal standoff, where the rover parks at its own
                # brake and the stall kill burns the goal -- five in a row ended
                # the mission on the south wall. This is NOT the old goal-pose
                # clearance filter coming back: that one rejected TARGETS on
                # geometry and starved frontier exploration; this one only skips
                # APPROACH POINTS the safety stack provably refuses to occupy,
                # and the target keeps its other approach points.
                acx = int((awx - ox) / res)
                acy = int((awy - oy) / res)
                if not point_clears_standoff(m.data, w, h, acx, acy, res,
                                             VIEWPOINT_STANDOFF_M):
                    continue
                # THE PLANNER remains the gate on reachability -- the measurement
                # and reasoning are in this module's docstring, and the
                # revert-proof is tests/test_goal_clearance_filter_removed.py.
                if self._planner_can_reach(awx, awy, frame):
                    goal_cell, goal_point = cell, (awx, awy)
                    break
            if goal_cell is None:
                # STANDOFF-SKIP ACCOUNTING, arithmetic on purpose: the guarded
                # revert-proof pins the ladder loop's exact shape (continue-only
                # standoff gate), so skips are derived as ladder length minus
                # planner queries actually spent -- exact for a candidate that
                # yielded nothing, which is the only candidate the end message
                # narrates. planner_rejections stays what its name says.
                ladder_n = sum(1 for _ in self._approach_points(gwx, gwy, wx, wy))
                self._standoff_skips += max(
                    0, ladder_n - (self._planner_queries - ladder_queries_before))
                # THE LADDER-SQUEEZE FALLBACK (cert attempt 3, ratified pin 2):
                # when every ladder pose sits inside the safety envelope, fall
                # back to the selection's PROVEN viewpoint -- the standoff-
                # clearing cell that qualified this cluster in the first place.
                # It clears the envelope by construction; the PLANNER remains
                # the reachability gate on it, same as every pose.
                vp = selection.viewpoints.get(cell)
                if vp is not None:
                    vwx, vwy = cell_center_world(vp[0], vp[1], ox, oy, res)
                    if (math.hypot(vwx - wx, vwy - wy) >= self._min_goal_distance_m
                            and self._planner_can_reach(vwx, vwy, frame)):
                        goal_cell, goal_point = cell, (vwx, vwy)
            if goal_cell is not None:
                break
        self._last_candidate_count = len(candidates)
        self._unplannable_last_cycle = (
            len(candidates) if goal_cell is None else candidates.index(goal_cell)
        )
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
                f"{len(candidates)} target(s) left, no goal found from here "
                f"(mission totals: {self._planner_rejections} planner NOs, "
                f"{self._standoff_skips} poses inside the safety envelope) — "
                f"unsticking (attempt {self._unstick_attempts}/{self._max_unstick})"
            )
            self._unstick(toward=self._cell_world(candidates[0], ox, oy, res))
            self._consecutive_empty = 0
            return

        if goal_cell is None:
            # Debounce: don't latch a terminal outcome on a transient/startup
            # empty. The decision itself is the PURE empty_cycle_outcome (the
            # D38 fix): a mission that never had a single target now ends
            # honestly instead of sitting armed and silent forever -- the gate
            # here used to be `ever_had_target and ...`, which made "never had
            # one" un-endable by construction.
            self._consecutive_empty += 1
            outcome = empty_cycle_outcome(
                self._ever_had_target, self._consecutive_empty,
                self._complete_after_empty, len(candidates))
            if outcome == OUTCOME_NO_TARGETS_FROM_START:
                self._mission_done = True
                self.get_logger().warn(
                    f"mission ENDED without ever finding a target: no candidate "
                    f"survived selection in {self._consecutive_empty} consecutive "
                    f"cycles and no goal was ever sent ({len(self._covered)} cells "
                    "stamped from the start pose alone). D38's silent-forever "
                    "mission, now an honest end with its own name."
                )
                self._finish(OUTCOME_NO_TARGETS_FROM_START, res, 0)
            elif outcome is not None:
                self._mission_done = True
                if candidates:
                    # There IS uncovered ground the rover wants; the planner just
                    # will not route to any of it. That is not a finished mission and
                    # must never be reported as one -- it is the honest form of the
                    # 2026-08-07 false COMPLETE, and now it is a structural
                    # distinction (targets exist vs targets don't) rather than an
                    # inference from how much got blacklisted.
                    # COUNTS, NOT DIAGNOSIS (cert attempt 3, 2026-08-19): the old
                    # message asserted "refused by the PLANNER" from a counter
                    # that also absorbed standoff skips, and a mission report
                    # carried 24 planner refusals in a run whose planner log
                    # shows zero. The line now states each named counter and
                    # lets the reader convict a mechanism.
                    self.get_logger().warn(
                        f"exploration ENDED with {len(candidates)} target(s) still "
                        f"wanted and no goal found — {len(self._covered)} cells "
                        "covered, so COVERAGE IS INCOMPLETE. Mission totals: "
                        f"{self._planner_rejections} planner NOs, "
                        f"{self._standoff_skips} approach poses inside the safety "
                        f"envelope, {self._excluded_no_viewpoint} cluster(s) with "
                        "no permitted viewpoint at last selection."
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
            duration_s=self._mission_elapsed_s(),
            goals_sent=self._goals_sent,
            goals_succeeded=self._goals_succeeded,
            goals_aborted=self._goals_aborted,
            goals_aborted_after_recovery=self._goals_aborted_after_recovery,
            goals_aborted_without_recovery=self._goals_aborted_without_recovery,
            goals_stall_killed=self._goals_stall_killed,
            goals_cancelled_at_end=self._goals_cancelled_at_end,
            planner_rejections=self._planner_rejections,
            standoff_skips=self._standoff_skips,
            remaining_candidates=remaining,
            cells_excluded_no_viewpoint=self._excluded_no_viewpoint,
            map_files=files,
            freeze_events=list(self._freeze_events),
            freeze_mark_merge_radius_m=float(
                self.get_parameter("freeze_mark_merge_radius_m").value),
            escape_events=list(self._escape_events),
            escape_poses=list(self._escape_poses),
        )
        self._report_pub.publish(String(data=json.dumps(report)))
        self.get_logger().info(f"mission report -> {json.dumps(report)}")
        self._write_report_file(report)

    def _write_report_file(self, report):
        """Put the report on DISK, not only on a latched topic and in a log line.

        D50. 2026-08-16 mission 2 ended, published its report, logged it -- and left no
        report file, because writing one had always been someone else's job: a capture
        step outside this node. The numbers survived only because the status topic was
        sampled by hand before teardown. A mission whose evidence depends on somebody
        remembering to catch a topic has optional evidence.

        Failure is logged and swallowed, never raised, for the same reason `_save_map`
        does it: a report that cannot be written must not also destroy the mission it is
        trying to describe.
        """
        try:
            directory = os.path.expanduser(
                str(self.get_parameter("mission_report_dir").value)
            )
            os.makedirs(directory, exist_ok=True)
            path = os.path.join(
                directory, f"report_{time.strftime('%Y%m%d_%H%M%S')}.json"
            )
            with open(path, "w") as handle:
                json.dump(report, handle, indent=2, sort_keys=True)
            self.get_logger().info(f"mission report written -> {path}")
            return path
        except Exception as exc:
            self.get_logger().error(f"mission report file FAILED: {exc}")
            return None

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
        self._planner_queries += 1
        handle = self._await(self._planner.send_goal_async(goal), deadline)
        if handle is None or not handle.accepted:
            return False
        result = self._await(handle.get_result_async(), deadline)
        if result is None:
            # Same rule as the unstick behaviours: a query we stop waiting for must
            # also stop running, or slow planner queries pile up server-side.
            handle.cancel_goal_async()
            return False
        reachable = len(result.result.path.poses) > 0
        # planner_rejections MEANS THE PLANNER SAID NO, counted here at the one
        # place the planner actually answers -- and nowhere else. It used to
        # accumulate candidates-without-goal, which cert attempt 3 turned into
        # false narration: a report claiming 24 planner refusals in a run whose
        # planner log shows zero (every one was a standoff skip). Server
        # unavailable / timeout above deliberately do NOT count: no answer is
        # not a NO.
        if not reachable:
            self._planner_rejections += 1
        return reachable

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
            self._active_goal_saw_ladder = False
            self._active_goal_ended = False          # D75: a fresh goal, no ending yet
            self._goals_sent += 1
            # The generation THIS goal belongs to. A freeze is claimed against the
            # goal it happened during, and the abort arrives on an async result
            # callback -- by which time _goals_sent may already have advanced to the
            # next goal. Reading the live counter at claim time therefore looks for
            # the wrong generation and silently drops a legitimate discovery, which
            # is a race, not a policy. Bind it here, where it is unambiguous.
            self._active_goal_generation = self._goals_sent
            new_generation = self._goals_sent
        # Publish the boundary IMMEDIATELY, outside the lock. The 0.5 s heartbeat
        # below is for late subscribers only: relying on it alone would let the first
        # half-second of a new goal run on the previous goal's spent escape budget,
        # which is the very failure this topic exists to end.
        self._generation_pub.publish(Int32(data=int(new_generation)))
        offset = math.hypot(wx - cwx, wy - cwy)
        via = f" (standing off {offset:.2f} m)" if offset > 0.01 else ""
        self.get_logger().info(
            f"coverage goal -> cell {cell} world ({wx:.2f},{wy:.2f}){via}; "
            f"covered={len(self._covered)} planner-rejected={self._unplannable_last_cycle} "
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
                generation = self._active_goal_generation
                self._active_goal_cell = None
            self.get_logger().warn(f"coverage goal {cell} REJECTED — suppressing it")
            self._note_failure(cell, generation, kind="rejected")
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
            generation = self._active_goal_generation
            saw_ladder = self._active_goal_saw_ladder
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
                self._active_goal_ended = True   # D75
                if saw_ladder:
                    self._goals_aborted_after_recovery += 1
                else:
                    self._goals_aborted_without_recovery += 1
            self.get_logger().warn(
                f"coverage goal {cell} ABORTED "
                f"({'after recovery' if saw_ladder else 'NO recovery ran'}) — "
                f"suppressing it for "
                f"{self._stall_ttl_s:.0f}s so we do not drive at it again"
            )
            self._note_failure(cell, generation, kind="aborted")
        elif status == GoalStatus.STATUS_SUCCEEDED:
            with self._lock:
                self._goals_succeeded += 1
                self._active_goal_ended = True   # D75
                self._consecutive_failures = 0
                self._consecutive_freezes = 0
                self._failure_streak = []
                # THE ESCAPE BUDGET IS NOT REFILLED HERE. It used to be, and under the
                # old Nav2 escape that was harmless because the escape never did
                # anything. Now that the escape MOVES the rover, refilling on a
                # succeeded goal is what turns escape-replan-succeed-stick-again into
                # an unbounded loop: four full escapes per mission is the bound, and a
                # rover that needs a fifth is telling us about the room. Same shape as
                # the ladder's max_total_traversals_per_goal.

    def _unstick(self, toward=None):
        """Ask the CONTROLLER to back us out, then let selection retry.

        A pose change is what makes previously-unplannable targets plannable, and
        planning is done from the live pose -- so a rover parked in inflation fails
        every goal no matter how open the map is. Measured after one such stop: the
        sole remaining target replied PATH in 7 ms once the rover had moved.

        This used to call nav2_behaviors' backup and spin directly. They were invoked
        correctly and did nothing: on 2026-08-12 they refused in 3 ms, five episodes
        out of five, because their collision check reads the global costmap -- which
        this rover's own five freeze marks had made lethal underneath it -- while the
        lidar reported 0.78 m of clear floor behind, 3.1x the supervisor's own reverse
        gate. The mission ended NO_PLANNABLE_TARGETS sitting on floor it could have
        backed onto (D36).

        So the escape is asked of the controller, which drives it through the
        collision supervisor on live lidar. THIS NODE STILL PUBLISHES NO VELOCITY: it
        sends a request and consumes the fact that comes back.
        """
        if self._mission_done or self._stop_requested():
            self.get_logger().info("escape abandoned — mission stopped")
            return False
        if not self._escape.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn(
                "escape UNAVAILABLE: no /decisive_controller/escape_in_place server. "
                "The rover cannot back out of an unplannable pose; ending honestly "
                "rather than pretending it tried.")
            self._record_escape("unavailable")
            return False

        goal = BackUp.Goal()
        goal.target.x = float(self._escape_distance_m)   # BackUp: +x is backwards
        goal.speed = float(self._escape_speed_mps)
        goal.time_allowance = Duration(seconds=self._escape_timeout_s).to_msg()

        deadline = time.monotonic() + self._escape_timeout_s + 2.0
        handle = self._await(self._escape.send_goal_async(goal), deadline,
                             stop_aware=True)
        if handle is None or not handle.accepted:
            # DECLINED is a logic error, not a routine refusal: this node only asks
            # while it is idle with no goal outstanding, so a rejection means the two
            # nodes disagree about who is driving. Loud, counted, never retried in a
            # loop -- a quiet retry here rebuilds the give-up livelock from the other
            # side.
            self.get_logger().warn(
                "escape DECLINED by the controller — it says it is not idle while "
                "this node believes it is. That is a state disagreement between the "
                "two, not a blocked rover; counting it as a failed escape.")
            self._record_escape(ESCAPE_DECLINED)
            return False
        if self._stop_requested():
            handle.cancel_goal_async()
            self.get_logger().info("escape cancelled — mission stopped")
            return False

        result = self._await(handle.get_result_async(), deadline, stop_aware=True)
        if result is None:
            handle.cancel_goal_async()
            self._await(handle.get_result_async(), time.monotonic() + 2.0)
            self.get_logger().warn("escape timed out client-side — cancelled")
            self._record_escape("timeout")
            return False

        outcome, detail = parse_outcome(getattr(result.result, "error_msg", ""))
        if outcome is None:
            # Never coerce an unknown answer into a known one: it means the controller
            # is running code this node has not seen.
            self.get_logger().warn(
                f"escape returned an outcome this node does not know: {detail!r}")
            self._record_escape("unrecognised")
            return False
        self._record_escape(outcome)
        if outcome == ESCAPE_CLEARED:
            self.get_logger().info(f"escape: backed out — {detail}")
            return True
        # Everything else means we are still where we were. DECLINED cannot reach
        # here (a decline is a goal REJECTION, handled above), so this is `refused`
        # or `frozen`: the way out is blocked too, and the mission ends honestly.
        self.get_logger().warn(
            f"escape did not free us ({outcome}: {detail}) — the way out is blocked "
            "too")
        return False

    def _record_escape(self, outcome):
        """Events AND distinct poses, per the D35 lesson: mission 1's report said
        `freeze_marks: 9` for six places and its own author read it as nine
        obstacles."""
        with self._lock:
            self._escape_events.append(outcome)
            here = self._robot_world(self._map.header.frame_id or "map") if self._map else None
            if here is not None:
                self._escape_poses.append((round(here[0], 2), round(here[1], 2)))

    def _cell_world(self, cell, ox, oy, res):
        return cell_center_world(cell[0], cell[1], ox, oy, res)

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
            self._freeze_events.append({"x": round(x, 3), "y": round(y, 3)})

    def _on_diagnostics(self, msg):
        """D56: freezes from the DRIVER's own stall counter -- the feed with a live
        publisher on the STOCK middle, where /decisive_controller/freeze_event has
        none and three real discoveries were booked as stack failures (2026-08-19
        combined ride-along). Deltas only, per counters-not-levels: the first
        sample is a baseline, never an event, and the baseline advances even while
        disarmed so an arming later does not replay old increments. A stall with
        no mission armed is somebody carrying the rover, not a discovery.

        The freeze position is this node's own robot pose at delta time -- the
        driver owns the fact, not the location, and a stalled rover IS at the
        discovery. On bespoke one physical stall can also arrive via _on_freeze;
        the synthetic entry is skipped when a claimable pending freeze already
        sits within the merge radius (dedupe is one-directional by consensus:
        the controller's positioned event is the better record when both exist).
        """
        key = str(self.get_parameter("stall_counter_key").value)
        count = None
        for status in msg.status:
            for kv in status.values:
                if kv.key == key:
                    try:
                        count = int(kv.value)
                    except (TypeError, ValueError):
                        return
                    break
            if count is not None:
                break
        if count is None:
            return
        with self._lock:
            last = self._last_stall_count
            self._last_stall_count = count
            armed = self._armed and not self._mission_done
        if last is None or count <= last or not armed:
            return
        here = self._robot_world(self._map.header.frame_id or "map") if self._map else None
        if here is None:
            self.get_logger().warn(
                f"driver stall counter +{count - last} during the mission, but no "
                "map/pose to place it -- discovery NOT recorded")
            return
        x, y = here
        merge = float(self.get_parameter("freeze_mark_merge_radius_m").value)
        with self._lock:
            goal = self._goals_sent
            if any(g >= goal - 1 and math.hypot(x - fx, y - fy) <= merge
                   for g, fx, fy in self._pending_freezes):
                return
            self._pending_freezes.append((goal, x, y))
            self._freeze_events.append({"x": round(x, 3), "y": round(y, 3)})
        self.get_logger().warn(
            f"driver stall counter +{count - last} during goal {goal} — counted "
            f"as a FREEZE discovery at ({x:.2f},{y:.2f}), not a stack failure "
            "(D56: the touch sense feeding the mission ledger on stock)")

    def _claim_freeze(self, generation=None):
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
            goal = self._goals_sent if generation is None else generation
            # THIS goal, or the one immediately before it. A freeze detected as goal
            # N-1 ends is the same physical discovery as goal N aborting at the same
            # spot, and the two orderings are not distinguishable from here: the
            # controller publishes during a goal, but an abort arriving on an async
            # callback can be processed either side of the next send. Requiring an
            # exact generation match dropped every discovery in the D26 rehearsal,
            # whose freezes arrive between goals -- an assumption I had not noticed I
            # was making until that test refused to pass.
            #
            # Still bounded and still consume-once: a freeze cannot excuse an abort
            # two goals later, and one freeze never excuses two aborts.
            mine = [f for f in self._pending_freezes if goal - 1 <= f[0] <= goal]
            if not mine:
                return None
            claimed = mine[-1]
            self._pending_freezes.remove(claimed)
            # Freezes from earlier goals are dead: they can never be claimed now, and
            # keeping them would let a stale discovery excuse a later unrelated abort.
            self._pending_freezes = [
                f for f in self._pending_freezes if f[0] >= goal - 1
            ]
            return claimed

    def _note_failure(self, cell, generation=None, kind="failed"):
        """Record that driving to `cell` failed: suppress it, and give up entirely if
        goals are failing everywhere. `kind` names the terminal path for the
        breaker's epitaph (stall_killed / aborted / rejected) -- counted, never
        diagnosed.

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
        frozen = self._claim_freeze(generation)
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

        here = self._robot_world(self._map.header.frame_id or "map") if self._map else None
        with self._lock:
            self._consecutive_failures += 1
            n = self._consecutive_failures
            self._failure_streak.append((kind, here))
            streak = list(self._failure_streak)
        if n >= self._max_consecutive_failures:
            # D53's epitaph rule: COUNT, DON'T DIAGNOSE, and key locality on the
            # ROVER's poses, not the goals'. The 2026-08-19 breaker said "failed
            # at different places -- this is the stack, not the room" over five
            # failures at ONE rover pose (spread 0.03 m) whose cause was the
            # floor: the goals were different places, the rover was not, and the
            # line inverted the diagnosis it had no business making.
            kinds = {}
            for k, _ in streak:
                kinds[k] = kinds.get(k, 0) + 1
            counts_txt = ", ".join(f"{v} {k}" for k, v in sorted(kinds.items()))
            poses = [p for _, p in streak if p is not None]
            spread_txt = "UNKNOWN"
            if len(poses) >= 2:
                spread = max(math.hypot(px - poses[0][0], py - poses[0][1])
                             for px, py in poses[1:])
                spread_txt = f"{spread:.2f} m"
            self.get_logger().error(
                f"{n} consecutive goals ended without success ({counts_txt}); "
                f"rover pose spread across the streak: {spread_txt}. Stopping and "
                "reporting the counts — stack, room, or floor is the per-goal "
                "ledger's question, not this line's."
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
        """Cancel the in-flight goal, AND COUNT IT (D62).

        The old comment here said "nothing is written off on a cancel", which
        was true and was the defect: a goal cancelled mid-flight by give-up,
        operator stop or map clear left the ledger short by one with no name for
        the difference, and the d53 invariant raced on that gap. This is the
        ONE place a live goal is cancelled, so it is the one place the counter
        belongs -- same single-incrementer discipline the planner_rejections
        re-anchor established (cert 3, 2026-08-19).
        """
        with self._lock:
            handle = self._active_goal_handle
            ended = self._active_goal_ended
            self._active_goal_handle = None
            self._active_goal_cell = None
        if handle is not None:
            # D75: only if nothing has already named this goal's ending. A stall-kill,
            # an abort and a success each claim the goal before or as they finish; a
            # cancel that follows one of them is the MECHANISM of that ending, not a
            # second ending. Counting both is what made `sent == sum(endings)` false.
            if not ended:
                with self._lock:
                    self._goals_cancelled_at_end += 1
                    self._active_goal_ended = True
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
