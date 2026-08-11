"""The stall survival ladder: one no-progress condition, one escalating escape.

Design and evidence: docs/stall_survival_ladder.md.

Two missions on 2026-08-10 died of two different stall causes, each individually
terminal, and the second one gave up while it had 0.58 m of clear floor ahead and
standing permission to drive into it. The rover knew exactly one escape -- straight
reverse -- and when the supervisor refused it, the goal aborted. Five such aborts end
a mission.

This module owns the replacement contract:

  * ONE predicate for "we are not getting anywhere", covering every cause.
  * ONE escalating sequence of escapes, tried in order until one WORKS.
  * A failure is counted ONCE PER EXHAUSTED LADDER, never once per refused action.

Pure core: no ROS, no scan parsing, no motion. The caller supplies pose, yaw, what it
commanded, what the supervisor actually emitted, and which bearing is most open; this
module decides what to do next.
"""

from dataclasses import dataclass
from typing import Optional
import math


# Rungs in escalation order. Each is an ordinary drive command through the existing
# supervisor -- deliberately NOT a Nav2 behaviour, because decisive mode removes the
# local costmap that behavior_server's Spin gate reads (D16).
REVERSE_STRAIGHT = "reverse_straight"
REVERSE_ARC = "reverse_arc"
PIVOT_OPEN = "pivot_open"
DRIVE_OPEN = "drive_open"
RUNG_ORDER = (REVERSE_STRAIGHT, REVERSE_ARC, PIVOT_OPEN, DRIVE_OPEN)

# Which measure tells us a rung is WORKING. A pivot that is granted but pinned moves
# no distance, and a reverse that is granted but blocked changes no heading, so asking
# the wrong question of a rung makes a refused escape look successful.
_RUNG_PROGRESS = {
    REVERSE_STRAIGHT: "position",
    REVERSE_ARC: "position",
    PIVOT_OPEN: "yaw",
    DRIVE_OPEN: "position",
}


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
    # Condition 3: the supervisor has zeroed our chosen action. At 20 Hz this is 1.0 s
    # of being refused -- long enough that a momentary brake is not a stall, short
    # enough to beat the explorer's 6 s goal-drop.
    suppressed_cycles: int = 20
    # GRIND YAW: a pivot against an invisible pin produces bursts of +/-80 deg in 0.3 s
    # (~4.6 rad/s) as the tracks slip and the estimator catches up. That is not motion,
    # but it looks like excellent yaw progress. Anything above the supervisor's cap
    # plus generous margin is physically impossible and must not count as progress.
    max_yaw_rate_rad_s: float = 0.6

    # --- anti-livelock budgets -------------------------------------------------
    # A rung gets this long to prove itself before we escalate.
    rung_budget_s: float = 3.0
    # A ladder that ESCAPES and then re-stalls on the same goal is thrash-with-motion:
    # bounded nowhere without this. Two attempts, then the goal is genuinely bad.
    max_invocations_per_goal: int = 2

    # --- rung commands ---------------------------------------------------------
    reverse_speed_mps: float = 0.10
    forward_speed_mps: float = 0.10
    pivot_rate_rad_s: float = 0.40
    # How far a successful escape must travel / turn before we call it cleared.
    escape_distance_m: float = 0.12
    escape_yaw_rad: float = 0.35


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


def _wrap(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class StallLadder:
    """Detects no-progress from any cause and escalates through the escapes.

    One instance per goal. The caller constructs a fresh one (or calls `reset_goal`)
    when a new goal begins, because the invocation budget is per goal.
    """

    def __init__(self, config: Optional[LadderConfig] = None):
        self._cfg = config or LadderConfig()
        self.reset_goal()

    # ------------------------------------------------------------------ lifecycle
    def reset_goal(self):
        self._invocations = 0
        self._clear_monitor()
        self._rung_index = None
        self._rung_started = None
        self._rung_ref = None

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
        return self._rung_index is not None

    @property
    def invocations(self) -> int:
        return self._invocations

    # ------------------------------------------------------------------ main entry
    def step(self, *, x: float, y: float, yaw: float, now: float,
             commanding: bool, output_moving: bool,
             open_bearing_rad: float = 0.0) -> LadderResult:
        """One control cycle.

        `commanding`    -- the controller wants to move (its command is non-zero).
        `output_moving` -- the SUPERVISOR's actual motor output was non-zero. The gap
                           between these two is the whole point: on 2026-08-10 they
                           disagreed for 14 straight seconds and only the second one
                           was telling the truth.
        `open_bearing_rad` -- bearing of the most open direction, robot frame, for the
                           pivot and drive rungs. Supplied by the caller because this
                           module does not parse scans.
        """
        cfg = self._cfg

        if self.active:
            return self._run_rung(x, y, yaw, now, output_moving, open_bearing_rad)

        # ---- condition 3: the supervisor is zeroing what we asked for -------------
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
        turned = abs(_wrap(yaw - self._ref_yaw))
        # Reject physically impossible yaw before it can count as progress.
        if self._last_yaw is not None and now > self._last_yaw_t:
            rate = abs(_wrap(yaw - self._last_yaw)) / (now - self._last_yaw_t)
            if rate > cfg.max_yaw_rate_rad_s:
                turned = 0.0
        self._last_yaw, self._last_yaw_t = yaw, now

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

    def _begin(self, now, x, y, yaw, open_bearing, reason):
        cfg = self._cfg
        # A FREEZE is "we were allowed to drive for most of this window and still did
        # not move". If the supervisor was refusing us, the stall is explained and
        # there is nothing invisible to report.
        freeze = self._window > 0 and self._out_moving * 2 >= self._window
        self._invocations += 1
        if self._invocations > cfg.max_invocations_per_goal:
            # Escaped before and re-stalled on the same goal. The goal is the problem.
            self._rung_index = None
            return LadderResult("exhausted", reason="ladder_budget_exhausted",
                                exhausted=True, freeze=freeze)
        self._rung_index = 0
        self._rung_started = now
        self._rung_ref = (x, y, yaw)
        cmd = self._rung_command(open_bearing)
        return LadderResult("rung", cmd[0], cmd[1], RUNG_ORDER[0],
                            f"{reason}->{RUNG_ORDER[0]}", freeze=freeze)

    def _rung_command(self, open_bearing):
        cfg = self._cfg
        rung = RUNG_ORDER[self._rung_index]
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
        return (cfg.forward_speed_mps, 0.0)

    def _run_rung(self, x, y, yaw, now, output_moving, open_bearing):
        cfg = self._cfg
        rung = RUNG_ORDER[self._rung_index]
        rx, ry, ryaw = self._rung_ref

        # Did this rung WORK? Ask the measure that matches what the rung does.
        if _RUNG_PROGRESS[rung] == "yaw":
            turned = abs(_wrap(yaw - ryaw))
            cleared = turned >= cfg.escape_yaw_rad
        else:
            cleared = math.hypot(x - rx, y - ry) >= cfg.escape_distance_m

        if cleared:
            self._rung_index = None
            self._clear_monitor()
            return LadderResult("drive", reason=f"{rung}_cleared")

        if now - self._rung_started >= cfg.rung_budget_s:
            # Refused, or granted-but-ineffective. Both mean: try something else.
            self._rung_index += 1
            if self._rung_index >= len(RUNG_ORDER):
                self._rung_index = None
                return LadderResult("exhausted", reason="all_rungs_refused",
                                    exhausted=True)
            self._rung_started = now
            self._rung_ref = (x, y, yaw)
            nxt = RUNG_ORDER[self._rung_index]
            cmd = self._rung_command(open_bearing)
            return LadderResult("rung", cmd[0], cmd[1], nxt, f"{rung}_failed->{nxt}")

        cmd = self._rung_command(open_bearing)
        return LadderResult("rung", cmd[0], cmd[1], rung, f"{rung}_running")
