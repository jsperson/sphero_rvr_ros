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
from typing import Optional


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


# --------------------------------------------------------------------------- #
# GENTLE TURN-AWAY (docs/turning_batch_design.md item 1)
#
# Measured on gauntlet run 114626: 86 s of a 309 s mission (28%) was spent asking
# for motion the motors never saw, and 55 s of that -- 63% -- was the CAMERA
# low-obstacle brake, whose forward scale is ZERO at <= 0.50 m. A cliff, not a band.
# Every one of the 12 aborts ran the same loop: forward -> camera cuts output ->
# 2 s suppressed -> ladder rung 1 reverses (the camera never brakes reverse, so it
# is always granted and always "clears") -> re-approach the same heading -> repeat
# -> per-goal escape budget spent -> abort. Five aborts end the mission.
#
# So the rover has to be ALREADY TURNING before it reaches the gate. And it cannot
# start at the gate: at the supervisor's angular cap (0.40 rad/s) and cruise
# (0.20 m/s) the tightest executable arc has radius v/w = 0.50 m, so displacing the
# swept corridor clear of a point dead ahead takes ~0.38 m of forward travel -- more
# than the whole brake band. The timidity is structural, and no gain fixes it; the
# reaction has to begin further out.
#
# This law only ever changes the TARGET HEADING. It emits no twist, so every regime
# below (deadband, arc, pivot) and the supervisor's final say are untouched.
# --------------------------------------------------------------------------- #


@dataclass(frozen=True)
class AvoidanceConfig:
    """Steering-away geometry, derived from the robot and the DEPLOYED supervisor
    config -- never from this room.

    `engage_m` = `stop_ref_m` + the forward travel needed to swing the corridor off
    a point dead ahead at cruise. With corridor half-width 0.18 m and arc radius
    0.50 m that travel is 0.384 m, so 0.50 + 0.38 = 0.88 -> 0.90 rounded out. It sits
    inside the camera's useful range (1.20 m), so both sensors can supply it.
    """

    # Where reacting begins. Beyond this, nothing steers.
    engage_m: float = 0.90
    # Where urgency reaches 1: the camera brake's hard-zero distance. Full steering
    # is applied at the last moment it can still be executed.
    stop_ref_m: float = 0.50
    # Ceiling on the heading offset. Chosen so the RESULTING angular cannot exceed
    # the supervisor's max_angular_rad_s (0.40): with arc_gain 1.2, 0.40 rad/s is
    # reached at a heading error of 0.33 rad. Asking for more would be a command the
    # supervisor silently clamps -- which is how a controller ends up lying to its
    # own ladder about what it requested (D32's root cause, one layer up).
    max_offset_rad: float = 0.33
    # Half-width of the corridor a blocker must be inside to count. NOT the robot's
    # own half-width (0.10): the thing that stops us is the camera brake, which
    # tests its swept path at camera_half_width_m 0.16. Steering must clear the
    # gate's corridor, not merely the body, or it stops just short of working.
    corridor_half_width_m: float = 0.18
    # Rate limit per control cycle. The camera runs ~5 Hz into a 10 Hz loop, so half
    # of all camera-driven blockers are a cycle stale; one noisy frame must not snap
    # the heading. Crossing engage -> stop_ref at cruise takes 2.0 s (20 cycles), so
    # 0.08 rad/cycle reaches full offset in 4 cycles with an order of magnitude to
    # spare.
    max_offset_step_rad: float = 0.08
    # The camera cloud's own validity window, mirrored from the deployed brake.
    camera_min_range_m: float = 0.40
    camera_max_range_m: float = 1.20


def camera_points_to_polar(points_xy, config: AvoidanceConfig):
    """(range, bearing) for camera cloud points that are actually OBSTACLE MARKS.

    This topic is NOT a pure obstacle cloud. `low_obstacle_node` also publishes
    CLEAR-RAY ENDPOINTS -- "the floor is clear this far" -- at `clear_range_m` 1.8 m,
    into the same cloud, at the same z, indistinguishable from marks by anything
    except their range. The deployed camera brake is immune only because
    `camera_max_range_m` (1.20) filters them out; a consumer that skips the filter
    steers AWAY FROM PROOF THAT THE FLOOR IS CLEAR.

    WHICH HALF IS LOAD-BEARING HERE, stated precisely because a mutation run caught
    me claiming both were:

      * MIN (0.40) is load-bearing TODAY. Below it the monocular detector is
        unreliable and the brake ignores the band; without this filter a point at
        0.30 m becomes a blocker at maximum urgency, and the steering law would be
        more credulous about this sensor than the safety layer is.
      * MAX (1.20) is defence-in-depth today and load-bearing tomorrow. At the
        deployed config `engage_m` (0.90) already excludes 1.8 m clear rays, so
        dropping the max filter changes nothing -- until someone raises the
        engagement radius, at which point every clear ray becomes an obstacle. The
        protection must not rest on a coincidence between two unrelated numbers, so
        it is stated here and pinned by a test that raises `engage_m` past the clear
        range.
    """
    out = []
    for x, y in points_xy:
        if x <= 0.0:
            continue
        rng = math.hypot(x, y)
        if rng < config.camera_min_range_m or rng > config.camera_max_range_m:
            continue
        out.append((rng, math.atan2(y, x)))
    return out


def corridor_blocker(points_polar, config: AvoidanceConfig,
                     max_relevant_m: float = float("inf")):
    """Nearest (range, bearing) that is actually in the way, or None.

    "In the way" is three tests: ahead of us, inside the engagement radius, and
    inside the corridor the robot will sweep. The corridor test is what stops the
    rover from swerving around everything it passes -- an obstacle 0.5 m away at 40
    deg sits 0.32 m off the centreline and needs no reaction at all.

    `max_relevant_m` drops blockers beyond the goal: we stop there, so curving away
    from something behind it would be steering away from the destination.
    """
    best = None
    for rng, bearing in points_polar:
        if abs(bearing) >= math.pi / 2.0:
            continue
        if rng >= config.engage_m or rng > max_relevant_m:
            continue
        if abs(rng * math.sin(bearing)) > config.corridor_half_width_m:
            continue
        if best is None or rng < best[0]:
            best = (rng, bearing)
    return best


def avoidance_heading_offset(nearest, open_bearing_rad, previous_offset_rad,
                             ladder_active, config: AvoidanceConfig) -> float:
    """How far to lean the target heading away from the nearest blocker, this cycle.

    `nearest` is `corridor_blocker`'s answer (None when the way is clear).
    `ladder_active` bypasses the whole law: while a rung is running the ladder owns
    the command, and a rung is by definition what happens after normal control has
    already failed. Steering must never second-guess an escape in progress.

    Returns a heading-error offset in radians, rate-limited from
    `previous_offset_rad` -- including on the way back to zero, so releasing is as
    smooth as engaging.
    """
    if ladder_active or nearest is None:
        target = 0.0
    else:
        rng, bearing = nearest
        span = config.engage_m - config.stop_ref_m
        urgency = 1.0 if span <= 0.0 else (config.engage_m - rng) / span
        urgency = max(0.0, min(1.0, urgency))
        # Which way to lean. A blocker off to one side is escaped by turning the
        # other way; one dead ahead has no such hint, so fall back to where the
        # widest gap is (the ladder's own open bearing, already TF-rotated into the
        # base frame by the caller). With neither signal, lean left deterministically
        # -- an arbitrary but repeatable choice beats a coin toss that makes field
        # behaviour unreproducible.
        if abs(bearing) >= 1e-3:
            away = -math.copysign(1.0, bearing)
        elif open_bearing_rad:
            away = math.copysign(1.0, open_bearing_rad)
        else:
            away = 1.0
        target = away * urgency * config.max_offset_rad

    step = config.max_offset_step_rad
    low, high = previous_offset_rad - step, previous_offset_rad + step
    return max(low, min(high, target))


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
    # True when the stall that triggered this result was classified as a FREEZE:
    # the supervisor was letting us drive and the robot still did not move. That is
    # positive evidence of an obstacle no sensor can see (below the 0.19 m lidar
    # plane and inside the camera's ~0.45 m blind zone), as opposed to the supervisor
    # braking us for something it CAN see. See ProgressGuard.step.
    freeze: bool = False


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
        # Freeze classification: how many cycles since the progress reference the
        # SUPERVISOR was actually letting us drive. Counting rather than sampling the
        # single instant the clock expires, because in a flapping SLOW/STOPPED
        # approach that instant is a coin toss.
        self._out_moving = 0
        self._window = 0

    def _mark_progress(self, x: float, y: float, now: float) -> None:
        self._ref = (x, y)
        self._ref_t = now
        self._out_moving = 0
        self._window = 0

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

    def step(self, x: float, y: float, now: float, translating: bool,
             output_moving: Optional[bool] = None) -> GuardResult:
        """One control cycle.

        `output_moving` is whether the SUPERVISOR's actual motor output was nonzero —
        i.e. whether we were permitted to drive. It is what separates the two stalls
        that look identical from inside this guard:

          * output was ZERO: the supervisor is braking for something it can see.
            Normal, expected, not news.
          * output was NONZERO and we still did not move: a FREEZE. Something is
            physically there that no sensor on this robot can detect. That is
            positive information about the room and the caller should treat it as
            discovery, not as a failure of the stack.

        SCOPE BOUNDARY, deliberate for this batch and not an oversight: a
        supervisor-braked stop at a CAMERA-VISIBLE obstacle is not classified as a
        freeze, so those goals still count toward the ordinary give-up counter. Those
        die at obstacles the camera can see but the planner cannot route around --
        the decisive-mode camera-to-planning gap -- which is a separate design
        question, to be decided on a future run's data.

        Passing None (the default) keeps the pre-freeze behaviour: no stall is ever
        classified as a freeze. That keeps every existing caller and test honest
        about what it is exercising.
        """
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

        # Tally what the supervisor permitted during this no-progress window.
        if output_moving is not None:
            self._window += 1
            if output_moving:
                self._out_moving += 1

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
            # A freeze is "we were allowed to drive for most of this window and still
            # did not move". Majority rather than the final instant: an approach that
            # flaps between SLOW and STOPPED would otherwise classify on a coin toss.
            frozen = self._window > 0 and self._out_moving * 2 >= self._window
            self._back_off_count += 1
            if self._back_off_count > cfg.max_back_offs:
                result = self._give_up()
                return GuardResult(result.action, result.reverse_speed_mps, frozen)
            self._backing_off = True
            self._bo_ref = (x, y)
            self._bo_t = now
            return GuardResult("reverse", cfg.back_off_speed_mps, frozen)

        return GuardResult("drive")


@dataclass(frozen=True)
class FreezeMark:
    """A place the robot proved it could not pass, with an expiry."""
    x: float
    y: float
    stamp: float
    expires_at: float


class FreezeMarkSet:
    """The live set of freeze marks, with TTL owned HERE rather than in the costmap.

    WHAT THE TTL ACTUALLY BOUNDS -- measured, because the intuitive reading is wrong:
    it governs what this set PUBLISHES and what reaches the mission report. It does
    NOT un-mark the costmap. The layer these feed runs with clearing:false (it must:
    the lidar sees straight through these obstacles and would otherwise erase them),
    and an ObstacleLayer never un-marks a cell it has marked. Verified on the bench
    2026-08-10: a free cell went cost 0 -> 100 on a mark and stayed 100 after
    publication stopped. In practice a mark therefore lasts the mission, which is the
    useful scope; revoking one mid-mission would need a different mechanism.

    Why they cannot simply be another source of the existing obstacle layer: that
    layer raytrace-clears from `scan`, and the entire premise of a freeze is that the
    lidar sees straight THROUGH the obstacle. The marks would be erased within a scan
    or two by the one sensor that is blind to them. They need a separate,
    non-clearing layer.

    Geometry note: a mark records the pose where motion stopped, and the consumer
    inflates it by roughly the robot radius. We know the robot could not pass HERE;
    we know nothing about the obstacle's true extent, so we mark the footprint we
    proved was blocked rather than guessing at the object.
    """

    def __init__(self, ttl_s: float = 300.0, merge_radius_m: float = 0.15):
        self._ttl_s = float(ttl_s)
        self._merge_radius_m = float(merge_radius_m)
        self._marks: list = []


    def add(self, x: float, y: float, now: float) -> FreezeMark:
        """Record a freeze. Re-freezing within `merge_radius_m` refreshes the existing
        mark rather than stacking duplicates -- a rover that retries the same spot
        three times has learned one fact, not three. (Deduplication is real and
        useful; expiry bounds publication and the report only -- see the class
        docstring.)"""
        self.prune(now)
        for i, m in enumerate(self._marks):
            if math.hypot(m.x - x, m.y - y) <= self._merge_radius_m:
                refreshed = FreezeMark(m.x, m.y, m.stamp, now + self._ttl_s)
                self._marks[i] = refreshed
                return refreshed
        mark = FreezeMark(float(x), float(y), float(now), now + self._ttl_s)
        self._marks.append(mark)
        return mark

    def prune(self, now: float) -> None:
        self._marks = [m for m in self._marks if m.expires_at > now]

    def live(self, now: float) -> list:
        self.prune(now)
        return list(self._marks)

    def as_report_list(self, now: float) -> list:
        """Plain dicts for the mission report. These belong in the REPORT, never in
        the saved map: the map is the room as SLAM measured it, while these are the
        robot's own belief about places it could not go."""
        return [{"x": round(m.x, 3), "y": round(m.y, 3), "stamp": round(m.stamp, 1)}
                for m in self.live(now)]

    def __len__(self) -> int:
        return len(self._marks)


def merge_positions(points, merge_radius_m: float) -> list:
    """The distinct PLACES behind a sequence of freeze events.

    Exists because the mission report needs the same answer the controller already
    computes, and a second implementation of "within merge_radius_m" is a second thing
    to keep in sync. So this does not re-implement the rule -- it runs the events
    through :class:`FreezeMarkSet` itself, with expiry disabled so that every event
    counts regardless of when it happened. Whatever the controller's merge does, this
    does, by construction rather than by review.

    Expiry is switched off ON PURPOSE. The set's TTL bounds what is PUBLISHED to the
    costmap layer (see the class docstring); a report is a summary of the whole
    mission, so an obstacle discovered in the first minute is still one of the places
    the rover could not go at minute nine.

    D35: the explorer appended one dict per freeze event and the report field was named
    as though it held places. Run 112721 filed nine entries for six positions --
    ``(-0.847,-1.094)`` four times -- and the run's own author read it as nine
    obstacles within the hour. Counting is the fix; naming is the other half of it.
    """
    marks = FreezeMarkSet(ttl_s=float("inf"), merge_radius_m=merge_radius_m)
    for x, y in points:
        marks.add(float(x), float(y), 0.0)
    return [(m.x, m.y) for m in marks.live(0.0)]


def escape_arc_command(speed_mps, arc_rate_rad_s, open_bearing_rad):
    """The give-up escape's commanded (linear, angular) — a reverse ARC, never a
    straight reverse.

    THE DEFECT THIS EXISTS TO KILL (D40, gauntlet 2026-08-14b). The escape used to
    command ``(-speed, 0.0)``. The supervisor's ``rear_hold`` gate zeroes ``linear_x``
    and passes ``angular_z`` through UNTOUCHED, so a straight reverse becomes exactly
    ``(0.0, 0.0)`` at every pose where the rear sector is inside
    ``reverse_stop_distance_m``. Field result: four give-up escapes, four refusals,
    "0.000 m in 6.0 s; supervisor: rear_hold", all at one pose with 7 of 12 bearings
    open and 2.18 m of floor at 4 o'clock.

    That is the UN-GRANTABLE-BY-CONSTRUCTION form of the recovery-defect family: the
    code runs, the trigger fires, and the arbiter must refuse it anyway because of the
    command's SHAPE. The check that catches it is asked of the arbiter, not the caller:
    *can this command shape ever be granted at the poses where it is meant to fire?*

    The stall ladder already knew: its REVERSE_ARC rung exists because a reverse arc
    "is granted at exactly the poses where rung 1 is refused" (stall_ladder.py). This
    function is that lesson, applied to the escape.

    DIRECTION. ``open_bearing_rad`` is the lidar-height open bearing, and it is the
    FALLBACK source by design -- the entry trail is the preferred one, because the
    trail is an all-heights eyewitness (the robot's body swept that corridor) while a
    lidar bearing is one horizontal plane at 0.19 m, blind to exactly the sub-lidar
    object class that causes freezes. The trail arrives with A1; until then this is the
    documented fallback, not a shortcut.

    A ``None`` or zero bearing still yields a turn -- sign only, magnitude fixed --
    because a reverse arc in an arbitrary direction is still grantable where a straight
    reverse is not, and "no preference" must not collapse back into the broken shape.
    """
    turn = math.copysign(abs(float(arc_rate_rad_s)), open_bearing_rad or 1.0)
    return (-abs(float(speed_mps)), turn)


def freeze_mark_pose(x, y, yaw, front_m, rear_m, reversing=False):
    """Where a freeze mark goes: the edge of the footprint that was DRIVING INTO it.

    The leading edge, not the centre: a mark at the centre sits `footprint_front_m`
    behind the obstacle it marks along the approach heading, so the costmap gets a
    point where the robot was standing rather than where the thing it hit was, and an
    approach from a slightly different angle reaches the same object without ever
    crossing a mark (the contact-by-contact face-walking of run 20260811_093818).

    AND THE EDGE DEPENDS ON THE DIRECTION OF TRAVEL. `reversing` is the commanded
    motion's sign, and it must be the COMMAND rather than anything inferred: a freeze
    while backing out means the obstacle is BEHIND, and marking the front edge then
    plants a lethal disc on clear floor ahead while leaving the real obstacle unmarked
    -- poisoning the costmap in the one situation where the rover most needs the floor
    ahead of it to stay plannable. Nothing in the shipped ladder can reach that case
    (freezes are classified only when a rung is NOT running, where the command is
    never negative), which is exactly why it survived: it becomes reachable the moment
    an escape reverses on its own.

    `rear_m` rather than `front_m` for the trailing edge, because this footprint is
    not symmetric: 0.11 m front, 0.16 m rear as deployed.
    """
    reach = -float(rear_m) if reversing else float(front_m)
    return (x + reach * math.cos(yaw), y + reach * math.sin(yaw))


class WindowedFreezeMonitor:
    """"Permitted, and going nowhere NEAR fast enough" -- over a sliding window.

    Getting this predicate right took three tries, and the two rejected ones are worth
    keeping because both sound correct:

    1. **Total travel since the escape began < epsilon.** Wrong, and wrong in exactly
       the field case: a rover pinned against something invisible CREEPS. Mission 1
       (2026-08-12), 13 straight-reverse windows against a blind contact with the
       supervisor granting 79% of cycles, achieved a mean of **0.086 m** -- nearly 3x
       `progress_epsilon_m`. The cumulative test reads that as motion forever.

    2. **The stall ladder's rolling rule** (a reference pose re-marked whenever
       progress >= epsilon). Also wrong here, and this one was a review instruction I
       followed and then measured: that same 0.086 m per 3 s is 2.9 mm per cycle, so
       the reference re-marks every ~11 cycles and the 20-cycle window never completes.
       The ladder is not wrong for the ladder -- it asks "did we get anywhere at all",
       and it classifies its freezes from a STANDING START, where creep has not begun.
       The escape asks a different question while already moving, so it needs its own
       predicate rather than a borrowed one.

    3. **Achieved travel against what the COMMAND should have achieved** -- this. The
       supervisor is permitting motion, we know what we asked for, and the honest
       question is whether the wheels are delivering it. Mission 1's pin delivered 29%
       of commanded (0.086 m of an asked-for 0.30 m in 3 s). The supervisor's own
       slowest legitimate scaling is 0.60 (camera SLOW band; lidar's is 0.70), so
       anything below **50%** of commanded is neither full speed nor a scaled grant --
       it is the tracks turning without the robot moving.

    Sliding, not reference-remarked: a window that resets on partial progress is how
    (2) hid a steady creep from itself.
    """

    def __init__(self, window_cycles: int, expected_per_cycle_m: float,
                 min_fraction: float = 0.5):
        self._window = max(2, int(window_cycles))
        self._expected = abs(float(expected_per_cycle_m))
        self._min_fraction = float(min_fraction)
        self._poses: list = []
        self._permitted: list = []

    def update(self, x: float, y: float, output_moving: bool) -> bool:
        """One control cycle. True when this is a freeze: the supervisor permitted
        motion for most of the window and the robot delivered a fraction of it."""
        self._poses.append((x, y))
        self._permitted.append(bool(output_moving))
        if len(self._poses) > self._window:
            self._poses.pop(0)
            self._permitted.pop(0)
        if len(self._poses) < self._window:
            return False
        achieved = math.hypot(self._poses[-1][0] - self._poses[0][0],
                              self._poses[-1][1] - self._poses[0][1])
        expected = self._expected * (self._window - 1)
        permitted = sum(1 for p in self._permitted if p)
        return (expected > 0.0
                and permitted * 2 >= len(self._permitted)
                and achieved < self._min_fraction * expected)
