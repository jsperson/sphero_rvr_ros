"""Stage two: rank the ways out, and never propose one the arbiter must refuse.

Stage one MEASURED (`escape_survey`). This ranks. It still commands nothing -- the
execute stage lands separately, deliberately, so these rankings can be checked
against the three recorded wedges before anything acts on them.

THE DEFECT THIS EXISTS TO KILL (D40). On 2026-08-15 the rover was cornered on three
sides, SPUN 180 DEGREES to face out, and then reversed straight back into the object
it had just turned away from -- with 1.77 m of clear floor dead ahead. It was not
choosing badly among options. It had exactly one option and never looked. So the
first rule here is structural rather than a preference, and it is Scott's, verbatim:

    "After reorienting, the selector must use the CURRENT heading -- spin-then-flee-
    backwards becomes impossible by construction."

That is why FORWARD_DRIVE is a candidate KIND that leads when the forward sector is
adequate, rather than a bearing competing on width: at 2026-08-15's pose the widest
opening and the current heading happen to agree, but at mission 2 they do not (12
o'clock reads 0.386 m while 3 o'clock reads 2.017 m) and the archive says 12 o'clock
is the answer there too. A width-only rule gets mission 2 wrong.

THE FAILURE FAMILY THIS BELONGS TO is the recovery-defect family, form 3:
UN-GRANTABLE BY CONSTRUCTION -- a recovery that runs, is reached, and is refused
every time because of its COMMAND SHAPE. The D36 give-up escape commanded
`(-v, 0.0)`; `rear_hold` zeroes linear and passes angular through untouched, so the
command became `(0.0, 0.0)` at every pose where the rear sector sat inside
`reverse_stop_distance_m`. Four attempts, four refusals, 0.000 m each.

THE CHECK THAT CATCHES THAT FAMILY IS ASKED OF THE ARBITER, NOT THE CALLER, and
`shape_is_grantable` below is that check moved one layer earlier: a candidate whose
first command the supervisor must refuse is never proposed, instead of proposed and
refused four times over 21 s.

TRUST HIERARCHY AMONG INPUTS (`VOUCH_ORDER` in `escape_survey`, unchanged here):

    TRAIL > ToF > LIDAR > UNVOUCHED

The trail is an all-heights eyewitness -- the robot's own body swept that corridor.
ToF speaks for the sub-lidar band, which is the object class that actually pins this
robot: all five freezes on 2026-08-15 were "an obstacle no sensor on this robot can
see". LIDAR speaks for one 0.19 m plane and is structurally blind to that class.
UNVOUCHED means nothing can speak, and it is RANKED LAST BUT NEVER FORBIDDEN --
most of any room is unvouched at any instant, so a rule that refused it would
deadlock a real room. It executes at reduced speed with the freeze-watch armed.

DEGENERATE INPUTS. A survey with no returns at all yields no candidates rather than
a confident forward drive; a sector with no rays in it is a REFUSAL and not a
clearance, because the deployed `sector_unknown_policy` is `blocked` and fail-closed
is respected by deriving, not by hoping.
"""

from dataclasses import dataclass
import math
from typing import Optional, Tuple

from sphero_rvr_core.bearings import clock_to_bearing_deg
from sphero_rvr_core.decisive_control import escape_arc_command
from sphero_rvr_core.escape_survey import (
    VOUCH_ORDER,
    VOUCH_TRAIL,
    VOUCH_UNVOUCHED,
    Survey,
)

__all__ = [
    "FORWARD_DRIVE",
    "ARC_TO_OPENING",
    "TRAIL_RETRACE",
    "CAUSE_FREEZE",
    "CAUSE_VISIBLE",
    "PlanGates",
    "PlanConfig",
    "ExitCandidate",
    "front_stop_distance_m",
    "rear_stop_distance_m",
    "shape_is_grantable",
    "rank_candidates",
    "format_plan",
]

FORWARD_DRIVE = "forward_drive"
ARC_TO_OPENING = "arc_to_opening"
TRAIL_RETRACE = "trail_retrace"

CAUSE_FREEZE = "freeze"
CAUSE_VISIBLE = "visible"

# Which half of the body an opening sits in decides the SHAPE that reaches it.
# Openings ahead are driven toward; openings behind are reversed toward, using the
# production arc command whose non-zero angular term is what survives `rear_hold`.
_AHEAD_CLOCKS = (10, 11, 12, 1, 2, 3, 9)
_BEHIND_CLOCKS = (4, 5, 6, 7, 8)


@dataclass(frozen=True)
class PlanGates:
    """The ARBITER's gate constants. Every field is REQUIRED -- there are no defaults.

    That is not style. A default here would be a quiet claim about the supervisor's
    deployed configuration, and this project has already flown one of those: thirteen
    fields have differed between `CollisionStopConfig`'s dataclass defaults and
    `config/collision_stop.yaml`, and a verdict flipped between them. The deployed
    `stop_distance_m` is 0.30 while the dataclass says 0.35; a plan built on 0.35
    would withhold forward moves the arbiter would have granted, which is the D40 sin
    running in the opposite direction.

    THE RUNTIME SOURCE OF THESE NUMBERS IS NOT WIRED, AND THAT IS FLAGGED RATHER THAN
    PAPERED OVER. The decisive controller does not hold a supervisor object; it
    subscribes to `/collision_stop/state` and keeps only the `reason=` token. Its
    footprint values are a hand-maintained MIRROR of the supervisor's config, which is
    precisely the assert-don't-infer failure that three defects in two days were
    bought with. The two honest sources are:

      (i)  the deployed YAML, read at test time -- what the proofs beside this module
           do, and all that is claimed today;
      (ii) the supervisor PUBLISHING its own gate constants on `/collision_stop/state`
           so the owner of the fact states the fact. That is the correct answer and it
           is queued as its own item; it is a safety-node change and cannot be
           validated on a desk.

    Until (ii) exists, nothing in the live node may construct a `PlanGates` from
    guesses. The missing constructor is the point.
    """

    stop_distance_m: float
    reverse_stop_distance_m: float
    footprint_front_m: float
    footprint_rear_m: float
    payload_margin_m: float
    measured_stop_time_s: float
    braking_distance_margin_m: float


@dataclass(frozen=True)
class PlanConfig:
    """Plan-stage thresholds, distinct from the arbiter's gates."""

    adequate_opening_m: float = 0.30
    """A candidate direction must read at least this far to be worth proposing.

    Range only. WIDTH adequacy -- "a sector wide enough for the swept path" -- is
    deliberately NOT tested here, because the swept path depends on the footprint
    extents, and those are blocked on Scott's tape measurements (design note 6b). A
    width test built on the current padded extents would be derived under a model we
    already know is wrong, which is exactly what standards rule 2 forbids.
    """

    speed_mps: float = 0.10
    arc_rate_rad_s: float = 0.40
    """Mirror the give-up escape's deployed values so a proposed shape is the shape
    that would actually be sent. A caller holding the live config should pass its
    own; these exist so the proofs can run without one."""

    unvouched_speed_scale: float = 0.5
    """Unvouched directions are never forbidden -- they execute SLOWER, with the
    freeze-watch armed, which is the existing machinery doing the job it was built
    for."""


@dataclass(frozen=True)
class ExitCandidate:
    kind: str
    clock: int
    bearing_deg: float
    range_m: Optional[float]
    vouched_by: str
    first_command: Tuple[float, float]
    reduced_speed: bool
    grantable: bool
    gate_reason: str
    rank_reason: str


def front_stop_distance_m(gates: PlanGates, speed_mps: float) -> float:
    """The supervisor's own front-stop distance for a command at this speed.

    Restated here because the plan lives in `sphero_rvr_core` and the arbiter lives
    in `sphero_rvr_driver`; importing downward would invert the layering. The
    restatement is BOUND to production by a premise tripwire that asserts numeric
    equality against `CollisionStopSupervisor._front_stop_distance` at every recorded
    pose. If the supervisor's arithmetic moves and this does not, that test goes red
    and says so -- which is the whole reason it is written as a tripwire rather than
    left as a comment.
    """
    dynamic = (gates.footprint_front_m + gates.payload_margin_m
               + _braking_distance_m(gates, speed_mps))
    return max(gates.stop_distance_m, dynamic)


def rear_stop_distance_m(gates: PlanGates, speed_mps: float) -> float:
    """The supervisor's own rear-hold distance. See `front_stop_distance_m`."""
    dynamic = (gates.footprint_rear_m + gates.payload_margin_m
               + _braking_distance_m(gates, speed_mps))
    return max(gates.reverse_stop_distance_m, dynamic)


def _braking_distance_m(gates: PlanGates, speed_mps: float) -> float:
    return abs(float(speed_mps)) * gates.measured_stop_time_s + gates.braking_distance_margin_m


def shape_is_grantable(command, survey: Survey, gates: PlanGates):
    """Would the supervisor let this command shape produce ANY motion here?

    Returns `(grantable, reason)`. **NECESSARY, NOT SUFFICIENT** -- and the asymmetry
    is deliberate. This answers only the two SECTOR gates, which a survey decides
    exactly. It cannot answer the trajectory projection, which needs the full swept
    region and defeated the arc escape at 08-14b after ~13 degrees of rotation
    (28 lidar returns sat inside the DECLARED footprint, so every refusal there was
    correct by construction). A candidate this function passes may still be refused
    in flight; that is what re-survey-after-failure is for.

    Being wrong in the two directions costs very differently, so the bias is chosen
    rather than accidental:

      * predicting REFUSED for something grantable loses a real move -- the D40 sin,
        the thing that stranded the rover in 180 degrees of open floor;
      * predicting GRANTABLE for something refused wastes one attempt, and the
        re-survey absorbs it.

    So this refuses only where the survey makes refusal CERTAIN.

    The rear branch is the un-grantable-by-construction lesson itself: `rear_hold`
    zeroes linear and passes angular through UNTOUCHED, so a reverse into a blocked
    rear delivers motion if and only if it carries a non-zero angular term. A
    straight reverse there is not a slow escape, it is no escape.
    """
    linear, angular = float(command[0]), float(command[1])

    if linear > 0.0:
        front = survey.front_sector_min_m
        if front is None:
            # `sector_unknown_policy: blocked`. No ray is not "no obstacle".
            return False, "front_sector_unknown"
        if front <= front_stop_distance_m(gates, linear):
            return False, "front_stop"

    if linear < 0.0:
        rear = survey.rear_sector_min_m
        if rear is None:
            return False, "rear_sector_unknown"
        if rear <= rear_stop_distance_m(gates, linear):
            if angular != 0.0:
                return True, "rear_hold_passes_angular"
            return False, "rear_hold"

    if linear == 0.0 and angular == 0.0:
        return False, "degenerate_command"

    return True, "no_sector_gate"


def _first_command(kind: str, clock: int, config: PlanConfig) -> Tuple[float, float]:
    """The shape that would actually go on the wire for this candidate."""
    bearing_deg = clock_to_bearing_deg(clock)
    if clock in _BEHIND_CLOCKS:
        # Production shape, not a restatement of it: `escape_arc_command` guarantees
        # the non-zero angular term that `rear_hold` passes through, and reverting it
        # to a straight reverse fails revert-proof 1c.
        return escape_arc_command(
            speed_mps=config.speed_mps,
            arc_rate_rad_s=config.arc_rate_rad_s,
            open_bearing_rad=math.radians(bearing_deg),
        )
    if clock == 12:
        return (config.speed_mps, 0.0)
    # Ahead but off-axis: drive forward while curving toward the opening. Sign follows
    # the bearing, so a LEFT opening turns left -- the mirror-pair specimens exist to
    # keep that from quietly becoming a side bias.
    turn = config.arc_rate_rad_s if bearing_deg > 0 else -config.arc_rate_rad_s
    return (config.speed_mps, turn)


def rank_candidates(survey: Survey, cause: str, gates: PlanGates,
                    config: PlanConfig = PlanConfig()) -> Tuple[ExitCandidate, ...]:
    """Rank the ways out of this pose. Ranks only -- commands nothing.

    THE ORDER, and each clause is a ruling with a pose behind it:

    1. **Cause FREEZE -> trail retrace leads.** A freeze is proof that unvouched
       space is actively HOSTILE, and the trail is the only eyewitness carrying
       all-heights evidence. Ranking a lidar-open bearing first after a freeze is
       ranking the sensor that just demonstrably missed something. Emitted ONLY when
       the trail is available AND aimed -- an unaimable candidate is the
       never-triggered form of the recovery-defect family, and this project has
       shipped that twice.
    2. **Forward drive, when the forward sector is adequate at the CURRENT heading.**
       Scott's structural requirement. This is what makes spin-then-flee-backwards
       impossible rather than merely discouraged.
    3. **Arc toward the widest adequate opening**, direction taken FROM THE SURVEY
       with both sides considered equally.
    4. **Vouched before unvouched, always** -- and unvouched last rather than never,
       at reduced speed.
    5. **Un-grantable shapes sort behind everything**, carrying the gate that would
       refuse them, so the log says what was considered and why it was not chosen.

    Footprint overlap is NOT a veto. At 08-14b, 28 returns sat inside the declared
    footprint and the projection refused everything correctly -- but the archive's
    answer at that pose is still 8 o'clock, so a rule that vetoed on overlap would
    return zero candidates at exactly the pose the acceptance is measured on. It is
    a confidence signal for the log, and the reason a trail retrace outranks live
    sensing when one exists.
    """
    candidates = []

    trail_clock = survey.trail_first_clock
    if survey.trail_available and trail_clock is not None:
        reading = survey.directions.get(trail_clock)
        command = _first_command(TRAIL_RETRACE, trail_clock, config)
        grantable, gate_reason = shape_is_grantable(command, survey, gates)
        candidates.append(ExitCandidate(
            kind=TRAIL_RETRACE,
            clock=trail_clock,
            bearing_deg=clock_to_bearing_deg(trail_clock),
            range_m=reading.range_m if reading else None,
            vouched_by=VOUCH_TRAIL,
            first_command=command,
            reduced_speed=False,
            grantable=grantable,
            gate_reason=gate_reason,
            rank_reason=("freeze_cause_trail_first" if cause == CAUSE_FREEZE
                         else "trail_is_all_heights"),
        ))

    ahead = survey.directions.get(12)
    if ahead is not None and ahead.range_m is not None and ahead.is_open:
        command = _first_command(FORWARD_DRIVE, 12, config)
        grantable, gate_reason = shape_is_grantable(command, survey, gates)
        candidates.append(ExitCandidate(
            kind=FORWARD_DRIVE,
            clock=12,
            bearing_deg=ahead.bearing_deg,
            range_m=ahead.range_m,
            vouched_by=ahead.vouched_by,
            first_command=command,
            reduced_speed=ahead.vouched_by == VOUCH_UNVOUCHED,
            grantable=grantable,
            gate_reason=gate_reason,
            rank_reason="forward_open_at_current_heading",
        ))

    for clock, reading in survey.directions.items():
        if clock == 12:
            continue
        if reading.range_m is None or reading.range_m < config.adequate_opening_m:
            continue
        command = _first_command(ARC_TO_OPENING, clock, config)
        grantable, gate_reason = shape_is_grantable(command, survey, gates)
        candidates.append(ExitCandidate(
            kind=ARC_TO_OPENING,
            clock=clock,
            bearing_deg=reading.bearing_deg,
            range_m=reading.range_m,
            vouched_by=reading.vouched_by,
            first_command=command,
            reduced_speed=reading.vouched_by == VOUCH_UNVOUCHED,
            grantable=grantable,
            gate_reason=gate_reason,
            rank_reason="widest_adequate_opening",
        ))

    if cause == CAUSE_FREEZE:
        # A freeze proves unvouched space is actively HOSTILE, so the all-heights
        # eyewitness leads and live sensing follows.
        kind_rank = {TRAIL_RETRACE: 0, FORWARD_DRIVE: 1, ARC_TO_OPENING: 2}
    else:
        # Visible cause: the sensor that CAN see the problem chooses the exit. Forward
        # still leads when it is adequate -- that is Scott's current-heading
        # requirement and it is unconditional -- then the widest adequate opening, and
        # the trail is the fallback for "no adequate live opening", which is the role
        # the design note gives it.
        kind_rank = {FORWARD_DRIVE: 0, ARC_TO_OPENING: 1, TRAIL_RETRACE: 2}

    def key(c: ExitCandidate):
        return (
            # 1. A shape the arbiter must refuse is not an exit. Behind everything.
            0 if c.grantable else 1,
            # 2. Unvouched last -- but present, and never forbidden. This is a BINARY
            #    bucket rather than the full vouching tier, because it is the ruling
            #    the design note actually makes ("the plan ranks vouched space above
            #    unvouched"). Putting the four-level tier here instead would silently
            #    override the cause ruling below: TRAIL outranks LIDAR at every tier,
            #    so a trail candidate would lead on a VISIBLE cause too, and "visible
            #    cause -> the widest adequate opening leads" would never fire. A test
            #    caught exactly that.
            1 if c.vouched_by == VOUCH_UNVOUCHED else 0,
            # 3. The cause ruling.
            kind_rank.get(c.kind, 9),
            # 4. Then evidence quality, then width.
            VOUCH_ORDER.index(c.vouched_by) if c.vouched_by in VOUCH_ORDER else len(VOUCH_ORDER),
            -(c.range_m if c.range_m is not None else 0.0),
        )

    return tuple(sorted(candidates, key=key))


def format_plan(candidates: Tuple[ExitCandidate, ...]) -> str:
    """One greppable line, beside the survey line it was ranked from.

    Same discipline as `format_survey` and for the same reason: the plan is the
    register's evidence as much as the executor's input, and three wedge autopsies
    have each cost hours reconstructing from a bag what one line would have said.
    """
    if not candidates:
        return "PLAN candidates=0 NO_EXIT_PROPOSED"
    parts = [f"PLAN candidates={len(candidates)}"]
    top = candidates[0]
    parts.append(f"first={top.kind}@{top.clock}oclock({top.rank_reason})")
    for c in candidates:
        flag = "" if c.grantable else f"!{c.gate_reason}"
        slow = "~" if c.reduced_speed else ""
        rng = "---" if c.range_m is None else format(c.range_m, ".3f")
        parts.append(
            f"{slow}{c.kind}@{c.clock}:{rng}/{c.vouched_by}"
            f"/cmd({c.first_command[0]:+.2f},{c.first_command[1]:+.2f}){flag}"
        )
    return " ".join(parts)
