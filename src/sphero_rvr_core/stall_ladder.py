"""The stall survival ladder: one no-progress condition, one escalating escape.

Design and evidence: docs/stall_survival_ladder.md.

Two missions on 2026-08-10 died of two different stall causes, each individually
terminal, and the second one gave up while it had 0.58 m of clear floor ahead and
standing permission to drive into it. The rover knew exactly one escape -- straight
reverse -- and when the supervisor refused it, the goal aborted. Five such aborts end
a mission.

This module owns the replacement contract:

  * ONE predicate for "we are not getting anywhere", covering every cause.
  * ONE escalating sequence of escapes, ORDERED BY THE STALL'S CAUSE, tried until one
    WORKS -- where "works" means the situation CHANGED, not that the robot moved.
  * The ladder REMEMBERS: a second stall in the same place resumes at the next rung,
    and the budget bounds complete traversals rather than repeats of rung 1.
  * A failure is counted ONCE PER EXHAUSTED LADDER, never once per refused action.

Version 2 (docs/turning_batch_design.md PART TWO) exists because version 1 danced: two
missions, 39 invocations, every one of them a 0.12 m straight reverse credited as an
escape, and rung 3 -- the pivot toward open floor -- never ran on hardware at all.

Pure core: no ROS, no scan parsing, no motion. The caller supplies pose, yaw, what it
commanded, what the supervisor actually emitted, and which bearing is most open; this
module decides what to do next.
"""

from dataclasses import dataclass
from typing import Optional
import math


# The four escapes. Each is an ordinary drive command through the existing
# supervisor -- deliberately NOT a Nav2 behaviour, because decisive mode removes the
# local costmap that behavior_server's Spin gate reads (D16).
REVERSE_STRAIGHT = "reverse_straight"
REVERSE_ARC = "reverse_arc"
PIVOT_OPEN = "pivot_open"
DRIVE_OPEN = "drive_open"

# ORDER IS CONDITIONED ON THE STALL'S CAUSE (design note PART TWO §9A).
#
# Reverse-first was designed for BLIND CONTACT, where nothing sees the obstacle and
# the path we came in on is the only route known to be clear (D25). It is the wrong
# first move for a stall whose cause a sensor can name: two missions, 39 invocations,
# every one of them starting at straight reverse, and rung 3 -- the pivot toward open
# floor -- never ran on hardware at all. At the second mission's death pose the
# rover's own lidar reported 2.07 m of open floor at -60 deg while it gave up five
# times in a row (§7.2).
#
# The two causes are distinguishable at the moment the ladder begins: the freeze vote
# (`output_moving` majority) already separates "the supervisor refused us, so a gate
# can see something" from "we were permitted to move and did not, so nothing can".
RUNG_ORDER = (REVERSE_STRAIGHT, REVERSE_ARC, PIVOT_OPEN, DRIVE_OPEN)
VISIBLE_RUNG_ORDER = (PIVOT_OPEN, DRIVE_OPEN, REVERSE_ARC, REVERSE_STRAIGHT)


@dataclass(frozen=True)
class LadderConfig:
    """Every threshold here is a bound on damage, not a tuning knob for this room."""

    # --- no-progress predicate -------------------------------------------------
    stall_time_s: float = 2.0
    progress_epsilon_m: float = 0.03
    # A pivot at the supervisor's own cap (max_angular_rad_s 0.4) turns 0.8 rad in the
    # 2 s stall window. An eighth of that is unambiguously "not turning" while staying
    # clear of odometry noise.
    yaw_progress_epsilon_rad: float = 0.10
    # Condition 3: the supervisor has zeroed our chosen action. At the deployed 10 Hz
    # control rate this is 2.0 s of being refused -- long enough that a momentary
    # brake is not a stall.
    #
    # It does NOT "beat the explorer's 6 s goal-drop", which is what this comment used
    # to claim: detection plus four 3 s rungs reaches exhaustion around t+14, so the
    # watchdog would cancel the goal before rungs 3 and 4 ever ran. That is fixed by
    # the explorer DEFERRING to an active ladder (see ladder_active), not by racing
    # a deadline -- a recovery whose rungs are chosen to fit inside someone else's
    # timeout is a recovery designed around the wrong constraint.
    suppressed_cycles: int = 20
    # GRIND YAW: a pivot against an invisible pin produces bursts of +/-80 deg in 0.3 s
    # (~4.6 rad/s) as the tracks slip and the estimator catches up. That is not motion,
    # but it looks like excellent yaw progress. Anything above the supervisor's cap
    # plus generous margin is physically impossible and must not count as progress.
    max_yaw_rate_rad_s: float = 0.6

    # --- anti-livelock budgets -------------------------------------------------
    # A rung gets this long to prove itself before we escalate.
    rung_budget_s: float = 3.0
    # BOUNDS COMPLETE LADDER TRAVERSALS, not repeats of one escape. This used to be
    # `max_invocations_per_goal = 2`, and with every stall restarting at rung 1 (see
    # the escalation memory below) two invocations bought the goal two identical
    # reverses and an abort -- 39 field invocations, not one of which reached rung 3.
    # One traversal now means every rung gets one honest attempt on this goal, and a
    # goal's total escape time stays bounded at roughly 4 x rung_budget_s.
    max_ladder_traversals_per_goal: int = 1
    # Escalation memory is per STALL REGION: if the rover genuinely got somewhere
    # between two stalls, the second one is a new problem and deserves the whole
    # ladder again. `robot_radius` (0.14, lean_nav2.yaml:96) is the robot's own scale,
    # and it separates the recorded populations with room to spare -- across both
    # gauntlet runs, consecutive invocations at the SAME stall are <= 0.107 m apart
    # (27 of 40) and every genuinely different place is >= 0.278 m away.
    inter_stall_progress_m: float = 0.14
    # THE OUTER BOUND, stated rather than hidden. `max_ladder_traversals_per_goal` is
    # a budget per STALL REGION, and the reset above renews it whenever the rover
    # genuinely gets somewhere -- so on its own it does not bound a goal at all: a
    # rover that escapes, drives 0.2 m, stalls again and escapes again could do that
    # forever without ever thrashing in one place. This counts complete traversals
    # MONOTONICALLY over the whole goal, across every reset, and it is the thing that
    # keeps "a stall may end a goal only after every escape has been tried" a bounded
    # contract rather than an unbounded one.
    #
    # Four distinct-place full ladders on one goal means the GOAL is cursed, not the
    # rover. Aborting it then costs nothing real -- the explorer suppresses that cell
    # and picks another -- and it is reported as itself, not as a rung that failed.
    max_total_traversals_per_goal: int = 4

    # --- rung commands ---------------------------------------------------------
    reverse_speed_mps: float = 0.10
    forward_speed_mps: float = 0.10
    pivot_rate_rad_s: float = 0.40
    # How hard rung 4 steers toward the open bearing (rad/s per rad of bearing).
    drive_open_arc_gain: float = 0.5

    # --- clearance: a rung clears only if the SITUATION CHANGED (§9B) -----------
    # Distance travelled is not change. `escape_distance_m` (0.12 m of any motion)
    # used to be the whole test, and run 20260811_211237 shows what it credits: 14
    # invocations, 14 straight reverses of 0.105-0.130 m, every one credited as an
    # escape, and 14 re-stalls within 12 s a median of 0.033 m from where the escape
    # began. Median fraction of that travel that was AXIAL: 100.0%. The rover backed
    # up 12 cm and rolled forward onto the same floor, and the ladder called it a win
    # twice per goal until the budget ran out. Scott: "backing up and rolling forward
    # shouldn't clear the ladder."
    #
    # So the test is a change that driving forward again cannot undo, and both
    # thresholds come from the robot and the DEPLOYED supervisor config:
    #
    #   * 30 deg of net heading change -- `front_stop_min/max_angle_deg` +-30
    #     (config/collision_stop.yaml:40-41) is the half-sector of the gate that
    #     stopped us, so it is exactly the turn that moves a dead-ahead blocker out
    #     of it.
    #   * 0.14 m of LATERAL displacement -- `robot_radius` (lean_nav2.yaml:96), the
    #     shift that moves the footprint's swath off the blocker. Lateral means
    #     perpendicular to the heading we stalled on; axial travel counts for
    #     nothing, which is the whole point.
    #
    # Measured against both gauntlet runs' complete populations: this credits 0 of
    # the 14 dance episodes and 0 of gauntlet 1's 28, where the distance test credits
    # all 14. Max lateral over those 14: 0.006 m. Max net heading: 6.5 deg.
    escape_yaw_rad: float = 0.524
    escape_lateral_m: float = 0.14
    # THE PIVOT RUNG IS CREDITED BY THE LIDAR, NOT BY WHEEL ODOMETRY. Its job is to
    # point the nose at the gap, so its clear test is "the open bearing has shrunk
    # into this tolerance", measured against the room by the scan the caller hands in
    # -- a rotation the tracks only pretended to make does not move the room.
    #
    # Derived: the driver ignores commanded pivot magnitude and self-regulates at a
    # measured 2.90 rad/s median (D32), so one cycle of the 10 Hz control loop is
    # ~0.29 rad and one cycle is the smallest command this stack can issue. With the
    # 1.5x margin the batch already adopted for the same reason (design note §2's
    # "~0.37 rad angular resolution" ruling) that is 0.435 -> 0.45 rad. A tolerance
    # below that floor cannot be regulated at 10 Hz; the loop overshoots and hunts.
    pivot_target_tolerance_rad: float = 0.45


@dataclass(frozen=True)
class LadderResult:
    """What the caller should do this cycle."""

    action: str                  # "drive" | "rung" | "exhausted"
    linear_x: float = 0.0
    angular_z: float = 0.0
    rung: Optional[str] = None
    reason: str = ""
    # True exactly once, on the cycle the ladder gives up: the caller aborts the goal
    # and ticks its failure counter THEN, and only then.
    exhausted: bool = False
    # True on the cycle the ladder BEGINS, when the supervisor was permitting motion
    # for most of the stall window and the robot still did not move. That is positive
    # information about the room -- something is physically there that no sensor on
    # this robot can detect -- and the caller should mark it, not count it as failure.
    # The ladder still runs: discovering an invisible obstacle does not excuse us from
    # escaping it.
    freeze: bool = False
    # Set with `exhausted` when EVERY rung was refused outright by the supervisor --
    # the rover never got a single cycle of permitted motion. That is "genuinely
    # wedged", and it is a different fact from "we manoeuvred and it did not help".
    # Reporting them as the same thing sends the next debugger looking for a bug in
    # the ladder when the honest answer is that the room had it surrounded. Same
    # honest-reporting rule as D24's UNKNOWN vs zero remaining candidates.
    genuinely_wedged: bool = False
    # Set when the ladder declined to act because this goal's escape budget was
    # already spent: NOTHING was tried, so nothing was permitted or refused. The
    # third state. Collapsing it into "ineffective" made the controller report that
    # the rover had been permitted to move and it had not helped -- blaming the
    # ROBOT for trying when it never tried. Five such lines in gauntlet run
    # 20260811_103337, against a recorder showing zero commands and one pose.
    #
    # It should now be RARE. Under the old per-invocation budget it was the ONLY
    # exhaustion either gauntlet mission ever reported (7 of 7 and 12 of 12), because
    # two invocations of a false-succeeding rung 1 spent the budget without ever
    # escalating. With a traversal budget, a goal is only refused an escape after
    # every rung has actually been tried, so `all_rungs_ineffective` and
    # `genuinely_wedged` -- the two outcomes that say something diagnostic about the
    # room -- become the normal way a ladder ends.
    budget_exhausted: bool = False


def _wrap(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class StallLadder:
    """Detects no-progress from any cause and escalates through the escapes.

    One instance per goal. The caller constructs a fresh one (or calls `reset_goal`)
    when a new goal begins, because the escape budget is per goal.
    """

    def __init__(self, config: Optional[LadderConfig] = None):
        self._cfg = config or LadderConfig()
        self.reset_goal()

    # ------------------------------------------------------------------ lifecycle
    def reset_goal(self):
        self._invocations = 0
        self._clear_monitor()
        self._end_rung()
        self._forget_escalation()
        self._last_begin_xy = None
        # Deliberately NOT in _forget_escalation: this one survives every reset the
        # ladder can perform on itself, and only a genuinely new goal clears it.
        self._total_traversals = 0

    def _forget_escalation(self):
        """Start the ladder over at the bottom: every rung untried, budget renewed.

        Only two things may do this -- a genuinely new goal (`reset_goal`, driven by
        the explorer's goal_generation) and genuine progress between two stalls.
        """
        self._tried = []
        self._traversals = 0
        self._any_rung_moved = False
        self._order = RUNG_ORDER

    def abandon_rung(self):
        """Drop any rung in progress, keeping the per-goal escape budget AND memory.

        F10. The ladder deliberately PERSISTS across a same-destination replan (see
        the controller: bt_navigator replans ~1 Hz, and resetting there would clear
        the anti-livelock counter forever). But persisting mid-RUNG is different: a
        goal that ends while rung 3 is running would leave the next execute loop
        resuming someone else's escape, with a stale reference pose and a stale
        clock. Called when a goal ends for any reason -- arrived, aborted, cancelled.

        The escalation memory deliberately survives: the rung that was interrupted
        has still been TRIED, and forgetting that is how a lifecycle event turns into
        an infinite loop of rung 1 (D34's mechanism, in the register).
        """
        self._end_rung()
        self._clear_monitor()

    def _end_rung(self):
        self._rung = None
        self._rung_started = None
        self._rung_ref = None
        self._rung_yaw = 0.0
        self._rung_last_yaw = None
        self._rung_last_t = None
        self._rung_bearing_ref = None

    def _clear_monitor(self):
        self._ref = None
        self._ref_t = None
        self._ref_yaw = None
        self._suppressed = 0
        self._last_yaw = None
        self._last_yaw_t = None
        # Tally of what the SUPERVISOR permitted during this no-progress window.
        # Majority vote rather than the final instant: an approach flapping between
        # SLOW and STOPPED would otherwise be classified on a coin toss.
        self._window = 0
        self._out_moving = 0

    @property
    def active(self) -> bool:
        return self._rung is not None

    @property
    def invocations(self) -> int:
        return self._invocations

    @property
    def tried_rungs(self) -> tuple:
        """Which escapes this goal has already had, in the order it had them."""
        return tuple(self._tried)

    # ------------------------------------------------------------------ main entry
    def step(self, *, x: float, y: float, yaw: float, now: float,
             commanding: bool, output_moving: bool,
             open_bearing_rad: Optional[float] = None) -> LadderResult:
        """One control cycle.

        `commanding`    -- the controller wants to move (its command is non-zero).
        `output_moving` -- the SUPERVISOR's actual motor output was non-zero. The gap
                           between these two is the whole point: on 2026-08-10 they
                           disagreed for 14 straight seconds and only the second one
                           was telling the truth.
        `open_bearing_rad` -- bearing of the most open direction, ROBOT frame, or
                           **None when the caller does not know one**. Supplied by the
                           caller because this module does not parse scans.

        NONE IS NOT ZERO, and the caller must not collapse them. Zero means "the way
        out is dead ahead"; None means "no scan, stale scan, or no TF to place it
        with". The controller used to return 0.0 for both, so the ladder inferred a
        bearing it had never been given -- and this module now decides the ESCAPE
        ORDER from that value, which is precisely the seam where inferring another
        component's state has cost this project three mission-killers. Unknown falls
        back to reverse-first, the order that needs no bearing at all.
        """
        cfg = self._cfg

        if self.active:
            return self._run_rung(x, y, yaw, now, output_moving, open_bearing_rad)

        # ---- condition 3: the supervisor is zeroing what we asked for -------------
        # NOTE, a deliberate difference from the old guard rather than a second drift:
        # this tallies on every COMMANDED cycle, where the old one tallied only while
        # translating. Restoring translating-only would re-blind the classifier to a
        # pinned PIVOT -- the supervisor permitting rotation that never happens is a
        # freeze, and Class A is the whole reason this module exists.
        if commanding:
            self._window += 1
            if output_moving:
                self._out_moving += 1
        if commanding and not output_moving:
            self._suppressed += 1
        else:
            self._suppressed = 0
        if self._suppressed >= cfg.suppressed_cycles:
            return self._begin(now, x, y, yaw, open_bearing_rad, "output_suppressed")

        if not commanding:
            self._clear_monitor()
            return LadderResult("drive")

        # ---- conditions 1 and 2: position and yaw both stalled -------------------
        if self._ref is None:
            self._mark(x, y, yaw, now)
            return LadderResult("drive")

        moved = math.hypot(x - self._ref[0], y - self._ref[1])
        # Reject physically impossible yaw before it can count as progress -- and
        # POISON THE REFERENCE, which the first version failed to do.
        #
        # Measured on run 142641 (the chair pin): yaw goes 105.3 -> 112.1 -> 120.3 ->
        # 137.2 -> 154.5 -> -176.6 -> -154.1 -> -152.6 and then sits there, while the
        # rover moves 15 mm. That is 102 deg in 0.6 s at up to 5.04 rad/s -- tracks
        # slipping against an obstacle while the estimator catches up, not motion.
        # Zeroing `turned` only for the BURST cycles was not enough: on the very next
        # settled cycle the burst is still inside (yaw - _ref_yaw), so it reads as
        # 1.78 rad of progress, re-arms the stall clock, and the ladder never fires.
        # The filter was defeated by the exact signature that motivated it.
        #
        # Re-marking the yaw reference at the post-burst value makes settles measure
        # only settled motion. The stall CLOCK is deliberately not reset here: a burst
        # is not progress, so it must not buy the rover more time.
        impossible = False
        if self._last_yaw is not None and now > self._last_yaw_t:
            rate = abs(_wrap(yaw - self._last_yaw)) / (now - self._last_yaw_t)
            impossible = rate > cfg.max_yaw_rate_rad_s
        self._last_yaw, self._last_yaw_t = yaw, now
        if impossible:
            self._ref_yaw = yaw
            turned = 0.0
        else:
            turned = abs(_wrap(yaw - self._ref_yaw))

        if moved >= cfg.progress_epsilon_m or turned >= cfg.yaw_progress_epsilon_rad:
            self._mark(x, y, yaw, now)
            return LadderResult("drive")

        if now - self._ref_t >= cfg.stall_time_s:
            reason = "position_and_yaw_stalled"
            return self._begin(now, x, y, yaw, open_bearing_rad, reason)

        return LadderResult("drive")

    # ------------------------------------------------------------------ internals
    def _mark(self, x, y, yaw, now):
        self._ref = (x, y)
        self._ref_yaw = yaw
        self._ref_t = now
        # ZERO THE FREEZE TALLY ON PROGRESS. The vote must cover the CURRENT
        # no-progress window only. Without this the counters run for the life of the
        # goal, so 60 s of happily granted driving banks ~1200 "permitted" votes, and
        # a later, entirely legitimate front-stop (20 refused cycles) still carries
        # the majority -- classifying a normal brake as a FREEZE and planting a
        # permanent phantom mark in the costmap that blames the room for a wall the
        # lidar can see perfectly well. The old ProgressGuard zeroed these in
        # _mark_progress; I moved the classifier and dropped the reset, so "semantics
        # unchanged" was not true.
        self._window = 0
        self._out_moving = 0

    def _order_for(self, freeze, open_bearing):
        """Which escape sequence this stall's CAUSE calls for (§9A).

        Pivot-first needs two things to be true: a gate can see what stopped us (so
        the entry path is not our only known-clear route), and there is somewhere to
        turn TO. A bearing already inside the pivot tolerance is not somewhere to turn
        to -- the gap is dead ahead and pivoting toward it is a no-op -- and an
        unknown bearing is not one either.

        A FREEZE OVERRIDES EVERYTHING, including a traversal already in progress: the
        moment nothing can explain our immobility, the retreats come first again, and
        the rung that drives FORWARD must not be reached while an untried retreat
        exists. Otherwise a traversal keeps the plan it started with -- the pivot's
        whole point is that the drive-out follows it, and re-deriving the order after
        the pivot has swung the gap to the nose would answer "nothing to pivot to,
        start again with a reverse", undoing the escape it just completed.
        """
        if freeze:
            return RUNG_ORDER
        if self._tried:
            return self._order
        if open_bearing is None:
            return RUNG_ORDER
        if abs(open_bearing) <= self._cfg.pivot_target_tolerance_rad:
            return RUNG_ORDER
        return VISIBLE_RUNG_ORDER

    def _next_rung(self, order):
        """The first escape in `order` this goal has not had yet, or None if all four
        have been tried -- which is what completes a traversal."""
        for rung in order:
            if rung not in self._tried:
                return rung
        return None

    def _begin(self, now, x, y, yaw, open_bearing, reason):
        cfg = self._cfg
        # A FREEZE is "we were allowed to drive for most of this window and still did
        # not move". If the supervisor was refusing us, the stall is explained and
        # there is nothing invisible to report.
        freeze = self._window > 0 and self._out_moving * 2 >= self._window
        self._invocations += 1

        # ESCALATION MEMORY, and the one thing that resets it. A second stall in the
        # same place is the same problem and must resume where the last one left off
        # (§9C); a second stall a real distance away is a NEW problem and gets the
        # whole ladder again. Distance from the last stall is the honest test of which
        # one happened -- and it is measured between the two stalls, never inside one
        # rung, because 0.12 m of reverse inside a rung is exactly the motion that
        # fooled the old clearance test.
        if self._last_begin_xy is not None:
            lx, ly = self._last_begin_xy
            if math.hypot(x - lx, y - ly) >= cfg.inter_stall_progress_m:
                self._forget_escalation()
        self._last_begin_xy = (x, y)

        # The cause is classified per STALL, and the order it chooses holds for as
        # long as that stall's escapes keep running. Re-deciding mid-escalation would
        # let a blind contact -- whose whole rationale is that the entry path is the
        # only route known to be clear -- switch to a plan that drives FORWARD because
        # the lidar happens to see a gap somewhere. The next stall re-derives it.
        if self._total_traversals >= cfg.max_total_traversals_per_goal:
            # Four complete ladders in four different places on ONE goal. The rover
            # has been escaping successfully and stalling again somewhere else every
            # time, which is a statement about the goal, not about the escapes.
            self._end_rung()
            return LadderResult("exhausted", reason="goal_traversal_ceiling",
                                exhausted=True, freeze=freeze, budget_exhausted=True)

        self._order = self._order_for(freeze, open_bearing)
        rung = self._next_rung(self._order)
        if rung is None or self._traversals >= cfg.max_ladder_traversals_per_goal:
            # Every escape has already had its honest attempt on this goal. Nothing is
            # tried here, so nothing is permitted or refused -- say exactly that.
            self._end_rung()
            return LadderResult("exhausted", reason="ladder_budget_exhausted",
                                exhausted=True, freeze=freeze,
                                budget_exhausted=True)
        self._start_rung(rung, now, x, y, yaw, open_bearing)
        cmd = self._rung_command(rung, open_bearing)
        return LadderResult("rung", cmd[0], cmd[1], rung,
                            f"{reason}->{rung}", freeze=freeze)

    def _start_rung(self, rung, now, x, y, yaw, open_bearing):
        self._rung = rung
        self._tried.append(rung)
        self._rung_started = now
        self._rung_ref = (x, y, yaw)
        self._rung_yaw = 0.0
        self._rung_last_yaw = None
        self._rung_last_t = None
        # Where the gap was when this rung started, so the pivot can be judged by how
        # far the ROOM has moved rather than by how far the wheels claim to have
        # turned.
        self._rung_bearing_ref = open_bearing

    def _rung_command(self, rung, open_bearing):
        cfg = self._cfg
        if rung == REVERSE_STRAIGHT:
            return (-cfg.reverse_speed_mps, 0.0)
        if rung == REVERSE_ARC:
            # Reverse arc rather than straight reverse, because `rear_hold` refuses a
            # straight reverse outright but passes ANGULAR through untouched -- so this
            # rung is granted at exactly the poses where rung 1 is refused. Measured
            # against the supervisor core at run 190528's abort geometry.
            turn = math.copysign(cfg.pivot_rate_rad_s, open_bearing or 1.0)
            return (-cfg.reverse_speed_mps, turn)
        if rung == PIVOT_OPEN:
            return (0.0, math.copysign(cfg.pivot_rate_rad_s, open_bearing or 1.0))
        # F7. Drive toward the OPEN bearing, not straight ahead at the heading we
        # stalled on. Rung 4 previously ignored open_bearing entirely, which in a
        # freeze means powered contact with the very obstacle we could not see -- the
        # rover pushing into the thing that stopped it. Arcing also gives the rung a
        # chance under a latch, where straight forward is refused outright.
        turn = max(-cfg.pivot_rate_rad_s,
                   min(cfg.pivot_rate_rad_s,
                       (open_bearing or 0.0) * cfg.drive_open_arc_gain))
        return (cfg.forward_speed_mps, turn)

    def _situation_changed(self, x, y, yaw, now, open_bearing):
        """Has this rung produced a change that driving forward again cannot undo?

        Two measures, and no third. The rung asked for something; the supervisor
        decided what actually reached the motors; the only honest question is whether
        the ROBOT'S RELATIONSHIP TO WHAT STOPPED IT is different now.
        """
        cfg = self._cfg
        rx, ry, ryaw = self._rung_ref

        # SUSTAINED, RATE-SANE yaw only, signed. _run_rung once applied no rate filter
        # at all, so a single slip burst mid-grind credited the pivot rung as
        # "cleared" and handed control back while the rover was still pinned; and
        # accumulating |step| credited ROCKING, a rover pinned against something
        # compliant oscillating within the rate-sane band, "escaping" having turned
        # nowhere. Net rotation is what an escape means.
        if self._rung_last_yaw is not None and now > self._rung_last_t:
            step = _wrap(yaw - self._rung_last_yaw)
            if abs(step) / (now - self._rung_last_t) <= cfg.max_yaw_rate_rad_s:
                self._rung_yaw += step
        self._rung_last_yaw, self._rung_last_t = yaw, now

        if self._rung == PIVOT_OPEN:
            # THE PIVOT IS JUDGED BY THE ROOM, NOT BY THE WHEELS. Its purpose is to
            # point the nose at the gap, so it is finished when the gap is in front of
            # us -- and the amount we turned to get there is read off the bearing,
            # which only a real body rotation can move. Wheel-odom yaw cannot serve
            # here: the driver self-regulates a pivot at ~2.9 rad/s, five times the
            # rate the grind filter calls physically possible (D32), so an
            # accumulator-based test can never credit a real pivot and this rung would
            # burn its whole budget spinning ~500 deg past the gap it was aiming at.
            # D32 still owns the discriminator everywhere else; this rung simply does
            # not ask the instrument that has the defect.
            if open_bearing is None or self._rung_bearing_ref is None:
                return False
            turned = abs(self._rung_bearing_ref) - abs(open_bearing)
            return (abs(open_bearing) <= cfg.pivot_target_tolerance_rad
                    and turned >= cfg.escape_yaw_rad)

        if abs(self._rung_yaw) >= cfg.escape_yaw_rad:
            return True
        # LATERAL displacement only: perpendicular to the heading we stalled on.
        # Reversing down our own entry path and driving back up it is the dance, not
        # an escape, and it is 100% axial by construction.
        dx, dy = x - rx, y - ry
        lateral = -dx * math.sin(ryaw) + dy * math.cos(ryaw)
        return abs(lateral) >= cfg.escape_lateral_m

    def _run_rung(self, x, y, yaw, now, output_moving, open_bearing):
        cfg = self._cfg
        if output_moving:
            self._any_rung_moved = True
        rung = self._rung

        if self._situation_changed(x, y, yaw, now, open_bearing):
            self._end_rung()
            self._clear_monitor()
            return LadderResult("drive", reason=f"{rung}_cleared")

        if now - self._rung_started >= cfg.rung_budget_s:
            # Refused, or granted-but-ineffective. Both mean: try something else --
            # the next escape THIS STALL'S CAUSE has not had yet.
            nxt = self._next_rung(self._order)
            if nxt is None:
                self._end_rung()
                self._traversals += 1
                self._total_traversals += 1
                wedged = not self._any_rung_moved
                return LadderResult(
                    "exhausted",
                    reason=("genuinely_wedged" if wedged else "all_rungs_ineffective"),
                    exhausted=True, genuinely_wedged=wedged)
            self._start_rung(nxt, now, x, y, yaw, open_bearing)
            cmd = self._rung_command(nxt, open_bearing)
            return LadderResult("rung", cmd[0], cmd[1], nxt, f"{rung}_failed->{nxt}")

        cmd = self._rung_command(rung, open_bearing)
        return LadderResult("rung", cmd[0], cmd[1], rung, f"{rung}_running")
