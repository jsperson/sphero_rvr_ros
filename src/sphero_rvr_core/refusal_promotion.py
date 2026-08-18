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
    RECOVERING. Time-based (sustained over WINDOW_S), progress-gated, reset on
    every goal change."""

    def __init__(self, window_s: float = WINDOW_S,
                 stall_m: float = STALL_DISPLACEMENT_M,
                 min_recoveries: int = MIN_RECOVERIES_IN_WINDOW):
        self.window_s = window_s
        self.stall_m = stall_m
        self.min_recoveries = min_recoveries
        self._goal_id: Optional[str] = None
        self._samples: list = []     # (t, x, y, recoveries)

    def feed(self, t: float, x: float, y: float, recoveries: int,
             goal_id: Optional[str]) -> bool:
        """True when the signature holds NOW. No goal, no signature."""
        if goal_id is None:
            self._goal_id, self._samples = None, []
            return False
        if goal_id != self._goal_id:
            self._goal_id, self._samples = goal_id, []
        self._samples.append((t, x, y, recoveries))
        self._samples = [s for s in self._samples if s[0] >= t - 2 * self.window_s]
        past = [s for s in self._samples if s[0] <= t - self.window_s]
        if not past:
            return False
        t0, x0, y0, r0 = past[-1]
        displacement = math.hypot(x - x0, y - y0)
        return displacement < self.stall_m and (recoveries - r0) >= self.min_recoveries


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
