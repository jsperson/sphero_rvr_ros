"""Pragmatic drive decision: straight when aligned, arc while moving, pivot only when needed.

This is the control core for a drivetrain (the RVR) that drives and arcs cleanly at
speed but grinds on slow in-place pivots. The philosophy, per the field spec:

* Point roughly at the target and, if roughly aligned, **drive straight** — do not
  correct a heading error that does not need correcting (a deadband).
* For a moderate course change, **arc**: keep both tracks rolling forward (above the
  motor breakaway) and turn while moving. Arcing does not grind.
* Only for a large heading change (or when boxed in) **pivot in place**, and do it
  decisively (a rate above breakaway), never a slow creep.

The point is: only ever turn for a reason, and prefer the motion that keeps the
motors rolling. This module is pure (no ROS) so it can be unit-tested and wired into
either a controller node or a Nav2 plugin.
"""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class DecisiveControlConfig:
    # Forward speed when driving or arcing (must be above the drivetrain breakaway).
    cruise_speed_mps: float = 0.20
    # Within this heading error, drive dead straight and DO NOT turn (the deadband).
    heading_deadband_rad: float = 0.17          # ~10 deg
    # Above this heading error, pivot in place; between deadband and this, arc.
    pivot_threshold_rad: float = 1.22           # ~70 deg
    # Arc angular = arc_gain * heading_error, capped by max_arc_angular_rad_s.
    arc_gain: float = 1.2
    max_arc_angular_rad_s: float = 0.8
    # In-place pivot rate — decisive, above breakaway (never a slow creep).
    pivot_rate_rad_s: float = 0.9
    # Stop when the target is within this distance.
    goal_tolerance_m: float = 0.10


@dataclass(frozen=True)
class DriveCommand:
    linear_mps: float
    angular_rad_s: float
    mode: str  # "arrived" | "straight" | "arc" | "pivot"


def _wrap(angle_rad: float) -> float:
    """Normalize an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def heading_error_to_point(
    robot_x: float,
    robot_y: float,
    robot_yaw_rad: float,
    target_x: float,
    target_y: float,
) -> tuple:
    """Signed heading error (+ = target is to the left/CCW) and distance to a target."""
    dx = target_x - robot_x
    dy = target_y - robot_y
    error = _wrap(math.atan2(dy, dx) - robot_yaw_rad)
    return error, math.hypot(dx, dy)


def select_target_point(path_points, robot_x: float, robot_y: float, lookahead_m: float):
    """Pick the point on the path roughly ``lookahead_m`` ahead of the robot.

    Walk forward from the path point nearest the robot until one is at least
    ``lookahead_m`` away; if the path ends first (near the goal), return its last
    point. ``path_points`` is a sequence of (x, y). Returns None for an empty path.
    A moderate lookahead is what keeps the controller from chasing an off-axis goal
    too early (it aims at the path a little ahead, not at the far endpoint).
    """
    if not path_points:
        return None
    nearest = min(
        range(len(path_points)),
        key=lambda i: (path_points[i][0] - robot_x) ** 2 + (path_points[i][1] - robot_y) ** 2,
    )
    for i in range(nearest, len(path_points)):
        px, py = path_points[i]
        if math.hypot(px - robot_x, py - robot_y) >= lookahead_m:
            return path_points[i]
    return path_points[-1]


def compute_drive_command(
    heading_error_rad: float,
    distance_to_target_m: float,
    config: DecisiveControlConfig,
) -> DriveCommand:
    """Decide the pragmatic command from the heading error and distance to the target.

    heading_error_rad is signed (+ = target to the left/CCW). The three regimes are
    the spec: aligned -> straight (no turn), moderate -> arc (turn while rolling),
    large -> decisive in-place pivot.
    """
    error = _wrap(heading_error_rad)
    magnitude = abs(error)

    if distance_to_target_m <= config.goal_tolerance_m:
        return DriveCommand(0.0, 0.0, "arrived")

    if magnitude <= config.heading_deadband_rad:
        # Roughly aligned: just drive. Do not turn.
        return DriveCommand(config.cruise_speed_mps, 0.0, "straight")

    if magnitude <= config.pivot_threshold_rad:
        # Moderate course change: arc — keep rolling, turn proportionally.
        angular = config.arc_gain * error
        angular = max(-config.max_arc_angular_rad_s, min(config.max_arc_angular_rad_s, angular))
        return DriveCommand(config.cruise_speed_mps, angular, "arc")

    # Large heading change: pivot in place, decisively (above breakaway).
    return DriveCommand(0.0, math.copysign(config.pivot_rate_rad_s, error), "pivot")


@dataclass(frozen=True)
class BackOffConfig:
    """Tunables for the back-off reflex (getting un-stuck without grinding)."""

    # No forward progress for this long (while we EXPECT to be translating) means
    # we are boxed in against an obstacle.
    stall_time_s: float = 2.0
    # Movement below this over the stall window does not count as progress.
    progress_epsilon_m: float = 0.03
    # Straight reverse speed to open room — both tracks roll back together, above
    # breakaway, so it does NOT grind (unlike an in-place pivot to wriggle out).
    back_off_speed_mps: float = 0.10
    # How far to reverse before handing back to normal control.
    back_off_distance_m: float = 0.12
    # If a back-off cannot make this distance in this long (rear likely blocked by
    # the supervisor), give up and abort so the planner re-routes.
    back_off_timeout_s: float = 3.0
    # After this many fruitless back-offs, abort the goal (let Nav2 replan /
    # explore pick another frontier) instead of shuffling in place forever.
    max_back_offs: int = 3


@dataclass(frozen=True)
class GuardResult:
    # "drive"   -> let the controller publish its normal command
    # "reverse" -> publish a straight reverse at reverse_speed_mps (skip normal)
    # "abort"   -> give up this goal so the planner re-routes
    action: str
    reverse_speed_mps: float = 0.0


class ProgressGuard:
    """Stateful (but ROS-free) back-off reflex, stepped once per control cycle.

    Feed it the robot position each cycle plus whether the controller currently
    expects to be translating (straight/arc, not pivoting or arrived). It watches
    for "commanding motion but not moving" — boxed in against an obstacle — and
    responds by backing straight out (no grind), then handing control back. If
    backing out does not help after a few tries, it says to abort so the higher
    layer re-plans. One instance per goal.
    """

    def __init__(self, config: BackOffConfig):
        self._config = config
        self._ref = None          # (x, y) of the last observed forward progress
        self._ref_t = None        # timestamp of that progress
        self._backing_off = False
        self._bo_ref = None       # (x, y) where the current back-off began
        self._bo_t = None         # timestamp the current back-off began
        self._back_off_count = 0

    def _mark_progress(self, x: float, y: float, now: float) -> None:
        self._ref = (x, y)
        self._ref_t = now

    def _give_up(self) -> "GuardResult":
        """Hand control back to the planner AND rearm.

        Aborting is this guard's way of saying "I cannot solve this, re-plan" -- it is
        a handoff, not a terminal state. Leaving the guard latched here was a real
        defect: `_backing_off` stayed true with an expired back-off clock, so every
        later cycle re-entered the same branch and returned abort within milliseconds,
        forever. On 2026-08-09 that killed 82 of 93 goals in ~70 ms each while the
        rover shuffled back and forth; the controller had decided it was boxed in once
        and could never take it back.
        """
        self._backing_off = False
        self._bo_ref = None
        self._bo_t = None
        self._ref = None
        self._ref_t = None
        self._back_off_count = 0
        return GuardResult("abort")

    def step(self, x: float, y: float, now: float, translating: bool) -> GuardResult:
        cfg = self._config

        if self._backing_off:
            backed = math.hypot(x - self._bo_ref[0], y - self._bo_ref[1])
            if backed >= cfg.back_off_distance_m:
                # Opened enough room — resume normal control with a fresh clock.
                self._backing_off = False
                self._mark_progress(x, y, now)
                return GuardResult("drive")
            if now - self._bo_t >= cfg.back_off_timeout_s:
                # Could not back up (rear blocked) — let the planner handle it.
                return self._give_up()
            return GuardResult("reverse", cfg.back_off_speed_mps)

        if not translating:
            # Pivoting or arrived: position is not expected to change, so do not
            # accrue stall time. Re-arm the progress clock on the next drive cycle.
            self._ref = None
            self._ref_t = None
            return GuardResult("drive")

        if self._ref is None:
            self._mark_progress(x, y, now)
            return GuardResult("drive")

        moved = math.hypot(x - self._ref[0], y - self._ref[1])
        if moved >= cfg.progress_epsilon_m:
            self._mark_progress(x, y, now)
            # Genuine forward progress clears the back-off tally, so it bounds ONE
            # stuck episode rather than a lifetime -- otherwise a rover that backed
            # off a few times across a long, otherwise successful mission would carry
            # that tally and abort on clear ground later. Note this is the
            # made-real-progress branch only: COMPLETING a back-off is retreat, not
            # progress, and must not reset it, or backing off and immediately
            # re-stalling would loop forever instead of escalating to abort.
            self._back_off_count = 0
            return GuardResult("drive")

        if now - self._ref_t >= cfg.stall_time_s:
            self._back_off_count += 1
            if self._back_off_count > cfg.max_back_offs:
                return self._give_up()
            self._backing_off = True
            self._bo_ref = (x, y)
            self._bo_t = now
            return GuardResult("reverse", cfg.back_off_speed_mps)

        return GuardResult("drive")
