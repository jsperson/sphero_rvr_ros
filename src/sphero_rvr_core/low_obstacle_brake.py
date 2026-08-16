"""Camera low-obstacle contribution to the collision brake (pure/testable).

The lidar collision-stop supervisor is the safety authority. This adds a *purely
additive* forward-motion limit from a sub-lidar obstacle cloud (points in
base_link) so the rover also stops for things below the 2D-lidar plane (chair legs,
a shoe, a floor cable). It can only reduce
forward speed, never increase it, and never touches reverse or rotation -- so the
worst case of a camera false positive is an over-cautious stop, and a stale/absent
cloud simply yields no limit (the lidar behaviour is unchanged).

WHICH PRODUCER FEEDS IT IS A DEPLOYED FACT, NOT A FACT ABOUT THIS FILE. The
consumer reads `low_obstacle_topic` from `config/collision_stop.yaml`, which is
`/tof/obstacles` today -- the ToF, not the camera. This docstring named
`/camera/low_obstacles` until 2026-08-15 and was simply wrong about the flying
system; the same stale claim still sits in `explore.launch.py`'s `start_low_obstacle`
help text. Naming a topic here at all is a config-is-a-claim hazard, so the rule is:
the module says what SHAPE it consumes (base_link points), and the deployed YAML
says where they come from.

Geometry note, and it is producer-specific: the CAMERA detector publishes points
relative to the camera origin but labelled base_link, and the camera sits ~0.06 m
forward of base_link, so those ranges slightly UNDER-estimate the true base_link
range -> the brake triggers a touch early, which is the safe direction. ToF points
are already base_link, so no such offset applies to them.
"""

import math

#: Below this the command is not TRANSLATING, and a non-translating command has no
#: forward corridor to ask about. One constant, used by both the turning test and the
#: refusal below, so the two can never disagree about where translation begins --
#: which is exactly the gap a second threshold would open.
_MIN_TRANSLATION_MPS = 1e-3


class NoSweptPath(ValueError):
    """Raised when a command has no swept path to answer about.

    RAISED, NOT RETURNED, and the distinction is the whole point. `None` already
    means "the swept path is clear" in this module, and `forward_speed_scale(None,
    ...)` returns 1.0 -- full speed. So returning None for an unanswerable query
    would not be a refusal at all; it would be the most permissive answer the API
    can give, handed out for the one input the function cannot model. Fail-open,
    wearing a refusal's clothes.

    A raise cannot destabilise the flying system, because production never reaches
    it: `_apply_low_obstacle_brake` returns early on `linear_x <= 0.0` before this
    module is called. It can only fire for a NEW caller, at development time, which
    is precisely who this guard exists for.
    """


def nearest_forward_obstacle(points_xy, half_angle_rad, min_range_m=0.0, max_range_m=float("inf")):
    """Nearest forward range among base-frame (x_forward, y_left) points inside the
    forward cone (|bearing| <= half_angle_rad, x > 0), within [min_range, max_range].
    Returns the range in metres, or None if the cone is clear."""
    best = None
    for x, y in points_xy:
        if x <= 0.0:
            continue
        rng = math.hypot(x, y)
        if rng < min_range_m or rng > max_range_m:
            continue
        if abs(math.atan2(y, x)) <= half_angle_rad:
            if best is None or rng < best:
                best = rng
    return best


def swept_path_obstacle(points_xy, linear_mps, angular_rad_s, half_width_m,
                        min_range_m=0.0, max_range_m=float("inf")):
    """Nearest obstacle on the arc the rover is ACTUALLY about to drive, or None.

    A fixed forward cone is wrong whenever the rover is turning: a differential
    drive pivots about a centre off to one side, so the *flank* leads the turn and
    sweeps ground the nose never covers. That is how a chair leg gets hit during a
    left turn while a straight-ahead cone reports "clear".

    Travelling straight (|w| tiny) the swept region is the corridor
    |y| <= half_width, x > 0. Turning, the rover follows a circle of radius
    R = v / w centred at (0, R) in the base frame; a point threatens the robot when
    its distance from that centre falls within [|R| - half_width, |R| + half_width].
    Returns the straight-line range to the nearest such point (comparable to the
    cone version, so the existing stop/slow distances still apply).

    Raises `NoSweptPath` for a non-translating command -- see `_swept_path_hits`.
    `None` here means CLEAR, so an unanswerable query cannot be reported as None
    without it reading as full speed ahead.
    """
    best = None
    for rng, _x, _y in _swept_path_hits(points_xy, linear_mps, angular_rad_s, half_width_m,
                                        min_range_m, max_range_m):
        if best is None or rng < best:
            best = rng
    return best


def nearest_swept_point(points_xy, linear_mps, angular_rad_s, half_width_m,
                        min_range_m=0.0, max_range_m=float("inf")):
    """The nearest in-corridor point as `(x, y)`, or None if the corridor is clear.

    `swept_path_obstacle` answers WHERE ALONG the path; this answers WHERE. A belief
    that has to be carried across frames needs a position, because the rover moves
    underneath it -- a scalar range would have to be re-interpreted in a frame it no
    longer belongs to, which is the frame ambiguity `zone_point` exists to prevent one
    module further up.

    Shares `_swept_path_hits` with the other two, so the point returned here is by
    construction the same point whose range `swept_path_obstacle` reports and which
    `points_in_swept_path` counted.
    """
    best = None
    for rng, x, y in _swept_path_hits(points_xy, linear_mps, angular_rad_s, half_width_m,
                                      min_range_m, max_range_m):
        if best is None or rng < best[0]:
            best = (rng, x, y)
    return None if best is None else (best[1], best[2])


def points_in_swept_path(points_xy, linear_mps, angular_rad_s, half_width_m,
                         min_range_m=0.0, max_range_m=float("inf")):
    """How many points the brake ACTUALLY CONSIDERED -- after the range window and
    the swept-path filter, not before.

    THE DEFECT THIS EXISTS TO KILL is a reporting one, and it cost two sessions.
    `/tof/state`'s `obstacle_zones` counts rule-B zones over the SENSOR'S WHOLE
    REACH; this brake acts only within [min_range, max_range] and only on the arc
    actually commanded. Nothing published the second number, so the first one stood
    in for it, and "zones cycled 0 -> 10 while cam_scale never left 1.00" read as a
    lost-detection bug in the detection-to-brake path.

    It was not. On 2026-08-15 those points sat at 0.542-1.563 m against a deployed
    `low_obstacle_max_range_m` of 0.60: out of reach, correctly ignored, and the
    brake engaged on the first frame the object was genuinely reachable. One extra
    number on the state line would have said so in five minutes.

    THE FAILURE FAMILY is "a correct number about the wrong population" (standards
    rule 1), and the check that catches it is: name the filter set beside the count.
    That is why this shares `_swept_path_hits` with `swept_path_obstacle` rather
    than re-deriving the geometry -- a count filtered differently from the value it
    is meant to explain would be a NEW instance of the same defect, in the
    instrumentation built to detect it.

    Degenerate input: no points, or none surviving the filters, returns 0 -- which
    is a different fact from `swept_path_obstacle` returning None only because the
    cloud was empty, and the pair is what distinguishes them. A non-translating
    command raises `NoSweptPath` rather than returning 0, because 0 here means
    "looked and found nothing" and this function did not look.
    """
    return sum(1 for _ in _swept_path_hits(points_xy, linear_mps, angular_rad_s,
                                           half_width_m, min_range_m, max_range_m))


def _swept_path_hits(points_xy, linear_mps, angular_rad_s, half_width_m,
                     min_range_m, max_range_m):
    """`(range, x, y)` of the points that threaten the commanded arc. ONE author for
    the swept-path geometry: `swept_path_obstacle` takes the minimum range of this,
    `points_in_swept_path` takes its length, and `nearest_swept_point` takes the
    position -- so the count, the distance and the position can never describe
    different sets.

    REFUSES a non-translating command rather than answering about the wrong motion.
    A stationary command and a pure pivot both have |v| below the translation
    threshold, and neither sweeps a forward corridor -- a pivot sweeps the
    footprint's corner CIRCLE, which is a different question this function does not
    model and deliberately does not try to. Until 2026-08-15 such a command fell
    through the `turning` test into the straight branch and got the corridor ahead
    of the robot: a confident, plausible, and wrong answer.

    Modelling the rotation annulus is explicitly NOT in scope here. Refusal is.
    """
    if abs(linear_mps) <= _MIN_TRANSLATION_MPS:
        raise NoSweptPath(
            f"linear_mps={linear_mps!r} is not translating, so there is no swept "
            f"path to report. A pivot sweeps the footprint's corner circle, not a "
            f"forward corridor -- ask the pivot gate, not this function."
        )
    turning = abs(angular_rad_s) > 1e-3 and abs(linear_mps) > _MIN_TRANSLATION_MPS
    if turning:
        radius = linear_mps / angular_rad_s          # +ve => centre to the left
        cx, cy = 0.0, radius
        inner, outer = abs(radius) - half_width_m, abs(radius) + half_width_m
    for x, y in points_xy:
        rng = math.hypot(x, y)
        if rng < min_range_m or rng > max_range_m:
            continue
        if turning:
            d = math.hypot(x - cx, y - cy)
            if not (inner <= d <= outer):
                continue
            # only count what is ahead along the turn, not behind the robot
            if linear_mps > 0.0 and x <= 0.0:
                continue
            if linear_mps < 0.0 and x >= 0.0:
                continue
        else:
            if abs(y) > half_width_m:
                continue
            if linear_mps >= 0.0 and x <= 0.0:
                continue
            if linear_mps < 0.0 and x >= 0.0:
                continue
        yield rng, x, y


def forward_speed_scale(nearest_m, stop_distance_m, slow_distance_m, min_forward_scale=0.0):
    """Forward-speed scale in [0, 1] for a camera obstacle at `nearest_m`:
    1.0 if clear (None) or beyond slow_distance; 0.0 at/inside stop_distance; a
    ramp from `min_forward_scale` (at stop) to 1.0 (at slow) in between. Flooring
    at `min_forward_scale` avoids scaling into the sub-breakaway creep/grind range
    (mirrors the lidar brake)."""
    if nearest_m is None:
        return 1.0
    if nearest_m <= stop_distance_m:  # stop wins ties (safe direction)
        return 0.0
    if nearest_m >= slow_distance_m:
        return 1.0
    span = slow_distance_m - stop_distance_m
    if span <= 0.0:
        return 0.0
    frac = (nearest_m - stop_distance_m) / span
    return min_forward_scale + (1.0 - min_forward_scale) * frac


class HoldResult:
    """What the hold concluded this frame.

    `nearest_m` is what the brake should treat as the nearest obstacle -- a live
    range, a HELD belief's range, or None for clear. `active` says whether that came
    from a belief rather than a sighting, and `reason` names why, because a hold that
    cannot explain itself on the state line is the instrumentation gap that let a zone
    count stand in for a point count for two sessions.
    """

    __slots__ = ("nearest_m", "active", "reason")

    def __init__(self, nearest_m, active, reason):
        self.nearest_m = nearest_m
        self.active = active
        self.reason = reason

    def __repr__(self):
        return (f"HoldResult(nearest_m={self.nearest_m!r}, active={self.active!r}, "
                f"reason={self.reason!r})")

    def __eq__(self, other):
        return (isinstance(other, HoldResult)
                and self.nearest_m == other.nearest_m
                and self.active == other.active
                and self.reason == other.reason)


def transport_point(point_xy, pose_delta):
    """Re-express a base_link point after the ROBOT moved by `pose_delta`.

    `pose_delta` is `(dx, dy, dyaw)`: the robot's motion expressed in its PREVIOUS
    base_link frame. The point does not move; the frame does, so this is the inverse
    transform -- translate by -d, then rotate by -dyaw.

    THE INPUT MUST BE MEASURED MOTION, NEVER COMMANDED MOTION. A command the arbiter
    zeroed would transport a belief on travel that never happened and retire it into
    the object it was protecting -- D40's lie ("the code ran, so the robot moved")
    relocated into the safety path, which is the one place it has not yet appeared.
    """
    dx, dy, dyaw = pose_delta
    ox, oy = point_xy[0] - dx, point_xy[1] - dy
    c, s = math.cos(dyaw), math.sin(dyaw)
    return (c * ox + s * oy, -s * ox + c * oy)


class BlindBandHold:
    """Holds the brake's last belief when returns vanish INSIDE the structural blind
    band, instead of reading the silence as clearance.

    THE DEFECT THIS EXISTS TO KILL (D39, confirmed in the field 2026-08-15 run 1,
    17:25:44-49). The brake tracked a table leg cleanly from 1.043 m down to a hard
    stop at 0.181 m. Then the near returns LEFT THE SENSOR'S VISIBILITY BAND with
    neither the object nor the rover moving. `swept_path_obstacle` returned None,
    `forward_speed_scale(None, ...)` returned 1.0 -- full speed -- and the rover drove
    90 mm back into the leg and struck it with the rangefinder, its foremost hardware.
    Nothing was broken: the module simply cannot distinguish "nothing is there" from
    "I can no longer see what is there", because both are None.

    THE RULE, and it is the standing one promoted into the safety path: A BELIEF IS
    RETIRED ONLY BY A LOOK THAT COULD HAVE SEEN AND DID NOT. Silence from a sensor
    that is structurally incapable of reporting at that range is not evidence of
    absence -- it is the absence of evidence, which is standards rule 10 stated about
    an instrument instead of a person.

    WHAT IS HELD IS A POINT, NOT A SCALE. Holding the output scale would freeze a
    number that no longer corresponds to any distance and could only be released by a
    timeout. Holding the obstacle's POSITION means the belief moves correctly under
    the robot (`transport_point`), the scale keeps following from the same
    `forward_speed_scale` as always, and -- the part that makes this safe to fly --
    the belief can leave the commanded corridor, so a PIVOT lifts the clamp.

    WHY THE PIVOT MATTERS (standards rule 5, asked of the ARBITER). A forward clamp
    that can only be retired by REVERSE would be un-grantable by construction: D40
    proved the arbiter refuses reverse at exactly the poses where this fires, so the
    rover would be forward-frozen for the rest of the mission -- a mission-killer
    wearing a safety fix's clothes. Two independent retirements exist instead:
    reversing carries the belief out to a range the sensor CAN report and a look then
    clears it, and rotating carries it out of the commanded corridor so it stops
    clamping at all. The second is the motion the arbiter is most willing to grant.

    FAIL DIRECTION, Scott's ruling: held-too-long is over-caution and acceptable;
    released-into-contact is the defect. So every uncertainty resolves to HOLD -- no
    fresh cloud, no pose, no answer -- and each one says which on the state line.
    """

    def __init__(self, band_outer_range_m, one_frame_closure_m, half_width_m,
                 min_range_m, max_range_m):
        #: Beyond this range the sensor COULD have reported the object, so silence
        #: there is a real look that found nothing. Below it, silence proves nothing.
        #: `one_frame_closure_m` is how far the rover can travel between two clouds
        #: the brake is willing to call fresh -- the brake's own freshness contract,
        #: not a chosen pad.
        self._visible_threshold = band_outer_range_m + one_frame_closure_m
        #: A point this close to the belief IS the belief. Same derivation reused
        #: rather than a second constant, because two tolerances would eventually
        #: disagree about what counts as the same object.
        self._reacquire_tol = one_frame_closure_m
        self._half_width = half_width_m
        self._min_range = min_range_m
        self._max_range = max_range_m
        self._belief = None

    @property
    def belief_xy(self):
        """The held point in the CURRENT base_link frame, or None."""
        return self._belief

    def update(self, points_xy, looked, linear_mps, angular_rad_s, pose_delta):
        """Advance the belief one frame and say what the brake should act on.

        `looked` is the distinction the whole mechanism turns on: True means a fresh
        cloud was examined, so an absent return is a real negative; False means no
        usable cloud existed (stale, absent, disabled) and NOTHING was learned. A
        stale cloud releasing the hold would be the released-into-contact defect in a
        second costume, so staleness holds.

        `pose_delta` is measured motion since the last call, or None when it could not
        be measured (a TF gap). None holds too -- a belief that cannot be placed
        cannot be shown to be gone.
        """
        belief = self._belief
        pose_known = pose_delta is not None
        if belief is not None and pose_known:
            belief = transport_point(belief, pose_delta)

        # The retirement test is deliberately NOT asked of the corridor. A belief the
        # rover has rotated away from is still a real object, and clearing it just
        # because it left the commanded arc would discard the belief exactly when the
        # rover is most likely to turn back toward it.
        retired = False
        if belief is not None and looked and pose_known:
            nearer = self._reacquire(points_xy, belief)
            if nearer is not None:
                belief = nearer
            elif math.hypot(belief[0], belief[1]) > self._visible_threshold:
                belief, retired = None, True

        forward = linear_mps > _MIN_TRANSLATION_MPS
        live = None
        if looked and forward:
            live = nearest_swept_point(
                points_xy, linear_mps, angular_rad_s, self._half_width,
                self._min_range, self._max_range,
            )
        if belief is None and live is not None:
            belief = live
        self._belief = belief

        if belief is None:
            return HoldResult(None, False, "retired_sight_through" if retired else "clear")

        belief_range = math.hypot(belief[0], belief[1])
        live_range = None if live is None else math.hypot(live[0], live[1])
        clamps = forward and self._in_corridor(belief, linear_mps, angular_rad_s)

        # THE NEARER OF THE TWO GOVERNS, AND A FARTHER SIGHTING NEVER RETIRES A NEARER
        # BELIEF. In run 1 a single stray return at 0.201 m arrived while the leg sat
        # believed at 0.181 m, and in the field it restored FULL commanded speed. A
        # first draft of this class handed the live reading straight back and would
        # have re-created that release at a gentler slope -- the same defect, slower.
        # Silence is not the only thing that fails to retire a belief; so does seeing
        # something else, further away.
        if clamps and (live_range is None or belief_range < live_range):
            return HoldResult(belief_range, True, self._hold_reason(looked, pose_known))
        if live_range is not None:
            return HoldResult(live_range, False, "live")
        # Still believed, but not on the path being driven -- so it must not clamp.
        # This is the pivot's retirement route and the reason the clamp is escapable.
        return HoldResult(None, True, self._hold_reason(looked, pose_known) + "_off_path")

    @staticmethod
    def _hold_reason(looked, pose_known):
        if not looked:
            return "held_no_look"
        if not pose_known:
            return "held_no_pose"
        return "vanished_in_band"

    def _reacquire(self, points_xy, belief):
        """The live point that IS the belief, or None. Snapped to rather than merely
        confirmed, so a belief that stays visible keeps tracking the object instead of
        drifting on transported pose alone.

        A REACQUISITION MAY ONLY MOVE A BELIEF NEARER, NEVER FURTHER. With the rover
        stationary, a return further out than the belief is not evidence the object
        receded -- it is evidence of partial visibility, which is the whole failure
        mode. Only MEASURED MOTION is allowed to push a belief away, via
        `transport_point`. Without this clause a thin object could be walked outward
        one stray return at a time until it left the band and retired itself, which is
        the field release rebuilt out of legal-looking steps.
        """
        belief_range = math.hypot(belief[0], belief[1])
        best = None
        for x, y in points_xy:
            d = math.hypot(x - belief[0], y - belief[1])
            if d > self._reacquire_tol or math.hypot(x, y) > belief_range:
                continue
            if best is None or d < best[0]:
                best = (d, (x, y))
        return None if best is None else best[1]

    def _in_corridor(self, belief, linear_mps, angular_rad_s):
        """Does the belief threaten the arc actually commanded? Asked through the SAME
        generator the live cloud goes through, so a belief can never be filtered by
        geometry the sighting was not -- the count-vs-value defect applied prospectively
        to a set of one.

        WITH ONE DELIBERATE DIFFERENCE: `min_range` is NOT applied. That floor exists
        to reject SENSOR READINGS the producer cannot be trusted on; a belief is not a
        reading. Passing it here would un-clamp the hold the moment the believed object
        transported inside 0.14 m -- releasing the brake nearest the obstacle, for the
        objects held closest, which is the precise defect this class exists to prevent,
        rebuilt inside it out of a validity window borrowed from the wrong population.
        `max_range` IS applied: beyond it the brake has no authority either way.
        """
        try:
            return points_in_swept_path(
                [belief], linear_mps, angular_rad_s, self._half_width,
                0.0, self._max_range,
            ) > 0
        except NoSweptPath:
            return False
