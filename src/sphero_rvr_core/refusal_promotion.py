"""Option D's brain: detect the livelock, choose the cells, discipline the firing.

Pure logic, no ROS. The watcher node (`refusal_watcher_node.py`) feeds it published
facts and publishes its one output; nothing here has authority over motion.

WHAT THIS PROMOTES AND WHY (decision stamped 2026-08-18, Scott: "go with D";
docs/design_tof_planner_visibility.md carries the full argument): a sub-lidar
obstacle the ToF sees lives only in the LOCAL costmap, the planner plans on the
GLOBAL one, and the result is a livelock the field measured precisely -- run 3c
goal 4: 16 RPP refusals, 10 recoveries, ~0.2 m of shuffling, honest abort in 19 s.
When that signature is OBSERVED (not predicted), the lethal-local/free-global delta
cells on the refused path are promoted through the touch-mark pipeline, which
already feeds both costmaps with provenance.

NAMED COSTS, carried here because this is where the mitigation lives:

* TRANSIENT-TRUE-OBSTACLE PROMOTION (position paper 5b): a person or a just-moved
  chair could accumulate refusals for a few seconds. Two structural mitigations:
  the signature requires the stall SUSTAINED over WINDOW_S (time, not a count --
  ambulatory blockages pass), and the BLINDNESS DELTA itself excludes anything the
  lidar sees (a person's legs are lethal in BOTH maps: no delta, never promotable).
  The exposure that remains is a sub-lidar transient held longer than WINDOW_S.
* MISSION-PERMANENCE-UNTIL-REVOCATION: a promoted mark is a touch mark -- nothing
  clears it. Wrong promotions cost floor for the rest of the mission; the firing
  discipline below exists to make them rare, and revocation (v2) makes them
  survivable.
* GOAL-IN-DELTA, accepted behaviour: if the goal itself sits on the sub-lidar
  obstacle, promotion makes it unplannable and the goal aborts honestly -- which is
  CORRECT (better an honest abort than the livelock), and the goal tool's trinity
  gate makes it rare. An abort right after a promotion is not a defect.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional

#: THE SIGNATURE'S CONSTANTS, derived from the day's three specimens rather than
#: taste. Goal 4 (livelock): ~0.2 m total over 19 s, 10 recoveries -- fires at
#: ~t+14 under these values. Goal 2 (healthy chatter): 1.46 m in 22 s -- the
#: displacement gate makes it structurally unfireable. Goal 1 (impossible goal):
#: 0.56 m in a 10 s life -- dead before WINDOW_S elapses.
WINDOW_S = 12.0
STALL_DISPLACEMENT_M = 0.15
MIN_RECOVERIES_IN_WINDOW = 2

#: Delta-cell selection: lethal in the LOCAL raw costmap, below inscribed in the
#: GLOBAL, on the refused path corridor, near the robot. Raw nav2 scale throughout
#: (254 lethal / 253 inscribed) -- both costmap_raw streams are on the wire.
LETHAL = 254
INSCRIBED = 253
CORRIDOR_HALF_WIDTH_M = 0.1519      # the costmap's inscribed radius (M1)
LOOKAHEAD_M = 1.0

#: Firing discipline (consensus amendment 1): one firing per goal per region, a
#: cooldown between firings, and a re-fire only when the first demonstrably did
#: not take -- a completed replan AND the delta still standing.
COOLDOWN_S = 5.0
MAX_DISCS_PER_FIRING = 3
MERGE_RADIUS_M = 0.15


@dataclass
class Grid:
    """A costmap snapshot in raw nav2 cost values. Built by the node's tracker
    (full frame replaced on costmap_raw, patched by costmap_raw_updates -- reading
    the latched frame alone is the trap `costmap-raw-is-latched-once` names)."""

    data: list
    origin_x: float
    origin_y: float
    resolution: float
    width: int
    height: int

    def at(self, x: float, y: float) -> Optional[int]:
        mx = int((x - self.origin_x) / self.resolution)
        my = int((y - self.origin_y) / self.resolution)
        if not (0 <= mx < self.width and 0 <= my < self.height):
            return None
        return self.data[my * self.width + mx]

    def apply_update(self, x0: int, y0: int, w: int, h: int, values: list) -> None:
        for row in range(h):
            dst = (y0 + row) * self.width + x0
            src = row * w
            self.data[dst:dst + w] = values[src:src + w]


@dataclass
class Firing:
    """One promotion decision: the capped cluster centroids to promote."""

    centroids: list                  # [(x, y)] in map frame, len <= MAX_DISCS
    suppressed_reason: Optional[str] = None


class LivelockWindow:
    """The signature: an active goal whose robot has stopped MOVING but not
    RECOVERING. Time-based (sustained over WINDOW_S), progress-gated, keyed on
    SPATIAL STATIONARITY -- not goal identity.

    It used to reset on every goal change (D55). That key composed the watcher
    into arithmetic impossibility on stock-explore: the explorer's progress
    watchdog kills a stalled goal at 6 s, so no sample could ever be WINDOW_S
    (12 s) old within one goal, and the 2026-08-19 flight's rover -- stationary
    within 0.03 m across FIVE consecutive goals, recoveries running, the exact
    signature this class exists to catch -- produced zero promotions BY
    CONSTRUCTION. Stationarity is a fact about the rover, not about which goal
    was asking; samples now survive goal churn and the displacement gate is what
    answers "is this a new question": a rover that MOVED cannot fire.

    Recoveries are counted per goal because that is how Nav2 hands them over --
    `number_of_recoveries` in NavigateToPose feedback RESETS on every goal, so a
    raw delta across churn goes negative and the >= min_recoveries gate would be
    exactly as un-meetable as the old time key (D55's second jaw). The window
    delta is derived from the samples themselves, per goal id (max-in-window
    minus value-at-window-start), which also keeps a straggling feedback from an
    already-replaced goal from inflating anything: its counts live under its own
    key. Zero new tunables; WINDOW_S, the stall bar, and min_recoveries carry
    unchanged from the certified constants above.

    Liveness is unchanged: feeds only arrive while a goal is active, `goal_id
    None` still drops everything, and the 2*WINDOW_S prune means a long idle gap
    self-heals -- stale samples cannot vouch for a fresh stall."""

    def __init__(self, window_s: float = WINDOW_S,
                 stall_m: float = STALL_DISPLACEMENT_M,
                 min_recoveries: int = MIN_RECOVERIES_IN_WINDOW):
        self.window_s = window_s
        self.stall_m = stall_m
        self.min_recoveries = min_recoveries
        self._samples: list = []     # (t, x, y, goal_id, recoveries)

    def _window_recoveries(self, t0: float) -> int:
        """Recoveries that happened AFTER t0, summed per goal id.

        For a goal already running at t0 the contribution is its in-window rise
        (max after t0 minus its last value at-or-before t0); a goal born inside
        the window contributes its max outright. Per-goal keys mean a stale
        feedback from a replaced goal can never bank onto another goal's count.
        """
        before: dict = {}
        after: dict = {}
        for ts, _x, _y, g, r in self._samples:
            if ts <= t0:
                before[g] = r                      # chronological: last one wins
            else:
                after[g] = max(after.get(g, 0), r)
        return sum(mx - before.get(g, 0) for g, mx in after.items())

    def feed(self, t: float, x: float, y: float, recoveries: int,
             goal_id: Optional[str]) -> bool:
        """True when the signature holds NOW. No goal, no signature."""
        if goal_id is None:
            self._samples = []
            return False
        self._samples.append((t, x, y, goal_id, int(recoveries)))
        self._samples = [s for s in self._samples if s[0] >= t - 2 * self.window_s]
        past = [s for s in self._samples if s[0] <= t - self.window_s]
        if not past:
            return False
        _t0, x0, y0, _g0, _r0 = past[-1]
        displacement = math.hypot(x - x0, y - y0)
        return (displacement < self.stall_m
                and self._window_recoveries(t - self.window_s) >= self.min_recoveries)


def corridor_points(plan_xy: list, robot_xy: tuple,
                    lookahead_m: float = LOOKAHEAD_M,
                    step_m: float = 0.05) -> list:
    """Points along the plan within lookahead of the robot -- the refused
    corridor's spine. The plan is the planner's own output; consuming it is
    assert-don't-infer at the seam."""
    rx, ry = robot_xy
    out = []
    for px, py in plan_xy:
        if math.hypot(px - rx, py - ry) <= lookahead_m:
            out.append((px, py))
    return out


def blindness_delta_cells(local: Grid, global_: Grid, plan_xy: list,
                          robot_xy: tuple,
                          corridor_half_width_m: float = CORRIDOR_HALF_WIDTH_M) -> list:
    """Cells LETHAL in local, BELOW INSCRIBED in global, on the refused corridor.

    The free-in-global gate is the person-excluder: anything the lidar sees is
    lethal in BOTH maps and produces no delta. What survives is precisely the
    planner's blindness -- the thing being promoted.
    """
    spine = corridor_points(plan_xy, robot_xy)
    if not spine:
        return []
    res = local.resolution
    seen = set()
    out = []
    steps = int(math.ceil(corridor_half_width_m / res))
    for sx, sy in spine:
        for ox in range(-steps, steps + 1):
            for oy in range(-steps, steps + 1):
                x = sx + ox * res
                y = sy + oy * res
                if math.hypot(x - sx, y - sy) > corridor_half_width_m:
                    continue
                key = (round(x / res), round(y / res))
                if key in seen:
                    continue
                seen.add(key)
                lv = local.at(x, y)
                gv = global_.at(x, y)
                if lv == LETHAL and gv is not None and gv < INSCRIBED:
                    out.append((x, y))
    return out


def cluster_and_cap(points: list, merge_radius_m: float = MERGE_RADIUS_M,
                    max_discs: int = MAX_DISCS_PER_FIRING) -> Firing:
    """Greedy-cluster the delta cells; refuse a firing that wants too many discs.

    More than `max_discs` clusters is a WALL, not an obstacle -- promoting it
    would sterilise a span the way one mark closed the 0.51 m corridor, and a
    wall the lidar cannot see at 19 cm is a finding for a human, not a costmap.
    """
    clusters: list = []              # [ [sum_x, sum_y, n, cx, cy] ]
    for x, y in points:
        for c in clusters:
            if math.hypot(x - c[3], y - c[4]) <= merge_radius_m:
                c[0] += x
                c[1] += y
                c[2] += 1
                c[3] = c[0] / c[2]
                c[4] = c[1] / c[2]
                break
        else:
            clusters.append([x, y, 1, x, y])
    if not clusters:
        return Firing([], suppressed_reason="no delta cells")
    if len(clusters) > max_discs:
        return Firing([], suppressed_reason=(
            f"{len(clusters)} clusters exceed the {max_discs}-disc cap -- this is "
            f"a wall, not an obstacle; refusing to sterilise it"))
    return Firing([(c[3], c[4]) for c in clusters])


#: The sensor's rate band floor and the WORST measured real-obstacle detection
#: duty: the thin-target flicker class detects in ~15% of frames (the "15% class",
#: run-3d era characterisation). At 6.5 Hz that is ~one return per second from the
#: flickeriest obstacle we have ever measured as real.
TOF_RATE_FLOOR_HZ = 6.5
FLICKER_DETECTION_FRACTION = 0.15

#: Evidence freshness (consensus round after transient-short FAILED on the rig):
#: a promotion candidate must be backed by a raw ToF return seen within this many
#: seconds, inside MERGE_RADIUS_M of the centroid.
#:
#: DERIVED, not chosen: the flickeriest measured real obstacle produces a return
#: every 1 / (TOF_RATE_FLOOR_HZ x FLICKER_DETECTION_FRACTION) ~ 1.03 s, so
#: F = 3.0 s admits it with ~3x margin while expiring a departed obstacle in three
#: seconds. The guard test re-derives this; change the sensor's rate constants and
#: F re-derives or fails a test, not a mission.
#:
#: WHY THE GATE EXISTS: tof_layer clearing needs successor returns to raytrace old
#: cells, so an obstacle that leaves in silence STRANDS ITS PAINT -- the local
#: costmap keeps refusing, the livelock window sustains on schedule, and an
#: ungated watcher faithfully promotes STALE evidence into a permanent mark
#: (transient-short, first rig run: marks at ~T after a 6 s obstacle had already
#: gone). The gate makes the watcher trust RETURNS, not paint. The robot may still
#: sit pinned on the stale paint -- that is tof_layer's own backlogged persistence
#: defect (the run-3d dead-band/FOV-transition item), not D's to solve; D's duty
#: is only that a transient never becomes PERMANENT.
#:
#: PIVOT-SWEEP DYNAMIC, accepted behaviour: during recovery spins the cone sweeps
#: off the obstacle and freshness can lapse -- promotion is then REFUSED until the
#: cone re-confirms on a later sweep. A temporary refusal that self-corrects is
#: correct (promote only what was recently SEEN); it costs latency, not
#: correctness, and pairs with the two-phase bar's next-plan latency contract.
EVIDENCE_FRESHNESS_S = 3.0


class RecentReturns:
    """A small ring of recent raw ToF returns in map frame, for the freshness gate."""

    def __init__(self, keep_s: float = 4.0 * EVIDENCE_FRESHNESS_S, cap: int = 2000):
        self.keep_s = keep_s
        self.cap = cap
        self._pts: list = []         # (t, x, y)

    def add(self, t: float, x: float, y: float) -> None:
        self._pts.append((t, x, y))
        if len(self._pts) > self.cap:
            del self._pts[: len(self._pts) - self.cap]

    def prune(self, now: float) -> None:
        self._pts = [p for p in self._pts if p[0] >= now - self.keep_s]

    def fresh_near(self, now: float, x: float, y: float,
                   radius_m: float = MERGE_RADIUS_M,
                   freshness_s: float = EVIDENCE_FRESHNESS_S) -> bool:
        cutoff = now - freshness_s
        return any(t >= cutoff and math.hypot(px - x, py - y) <= radius_m
                   for t, px, py in self._pts)


def freshness_verdict(returns: RecentReturns, now: float, centroids: list) -> tuple:
    """(fresh_centroids, stale_centroids). EVERY centroid must be individually
    backed -- one live cluster must not launder a stale one through the firing."""
    fresh, stale = [], []
    for cx, cy in centroids:
        (fresh if returns.fresh_near(now, cx, cy) else stale).append((cx, cy))
    return fresh, stale


class FiringDiscipline:
    """Consensus amendment 1: promotions are EVENTS, and events are rationed.

    One firing per goal per region; COOLDOWN_S between any two firings; a re-fire
    for the same goal+region only if a replan has COMPLETED since the last firing
    and the delta still stands (the first promotion demonstrably did not take).
    Every suppression is reported so the log shows the decision, not silence.
    """

    def __init__(self, cooldown_s: float = COOLDOWN_S,
                 merge_radius_m: float = MERGE_RADIUS_M):
        self.cooldown_s = cooldown_s
        self.merge_radius_m = merge_radius_m
        self._fired: dict = {}       # (goal_id, region_key) -> fire time
        self._last_fire_t: Optional[float] = None
        self._plans_seen_since_fire = 0

    def _region_key(self, centroids: list):
        gx = sum(c[0] for c in centroids) / len(centroids)
        gy = sum(c[1] for c in centroids) / len(centroids)
        r = self.merge_radius_m
        return (round(gx / r), round(gy / r))

    def note_plan(self) -> None:
        self._plans_seen_since_fire += 1

    def allow(self, t: float, goal_id: str, centroids: list) -> tuple:
        """(allowed, reason). Reason is always set; a silent gate is no gate."""
        if not centroids:
            return False, "nothing to promote"
        if self._last_fire_t is not None and t - self._last_fire_t < self.cooldown_s:
            return False, (f"cooldown: {t - self._last_fire_t:.1f}s since last "
                           f"firing (< {self.cooldown_s}s)")
        key = (goal_id, self._region_key(centroids))
        if key in self._fired:
            if self._plans_seen_since_fire < 1:
                return False, ("already fired for this goal+region and no replan "
                               "has completed -- the first promotion has not been "
                               "given its chance")
            return True, ("re-fire: replan completed and the delta persists -- "
                          "the first promotion demonstrably did not take")
        return True, "first firing for this goal+region"

    def record(self, t: float, goal_id: str, centroids: list) -> None:
        self._fired[(goal_id, self._region_key(centroids))] = t
        self._last_fire_t = t
        self._plans_seen_since_fire = 0
