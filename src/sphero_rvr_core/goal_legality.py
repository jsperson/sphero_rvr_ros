"""Why a goal is illegal, and where the nearest legal one is. Pure.

D64, filed 2026-08-22 after Scott stood in the room for 154 s while an operator
(me) guessed at goals the trinity kept refusing. Every one of those refusals was
CORRECT — the gate is a safety property and this module does not touch it. What
was missing is that the gate already KNOWS why, and already has everything
needed to point at a legal cell, and threw both away.

THE CLASSIFICATION IS THE POINT, not the search. "SLAM-unknown" was the same
sentence for two situations that call for opposite operator actions:

  BEYOND_HORIZON  the map simply ends here — drive closer and this cell becomes
                  legal. Waiting helps; the map grows as the rover moves.
  IN_SHADOW       a mapped obstacle sits between the robot and this cell, so it
                  lies in the obstacle's sensor shadow. **No goal behind an
                  obstacle can EVER be legal until the rover moves to see past
                  it.** Waiting does NOT help. Two of 2026-08-22's five refusals
                  were this, and both cost minutes of guessing.
  INFLATED        mapped free but inside an inflation band. The nearest cost-0
                  cell is usually centimetres away, and now gets named.

Pure (no ROS): grids arrive as plain (values, width, height, resolution,
origin_x, origin_y) tuples, so the classifier and the search are unit-testable
against hand-built rooms.
"""

from dataclasses import dataclass
import math
from typing import Optional, Tuple

#: Verdict codes. LEGAL is the only one that flies.
LEGAL = "LEGAL"
OFF_GRID = "OFF_GRID"
BEYOND_HORIZON = "BEYOND_HORIZON"
IN_SHADOW = "IN_SHADOW"
OCCUPIED = "OCCUPIED"
INFLATED = "INFLATED"

#: Operator sentences. Each says what to DO, because that is what was missing.
ADVICE = {
    LEGAL: "mapped free, cost 0",
    OFF_GRID: "outside the mapped grid entirely — no map data here at all",
    BEYOND_HORIZON: ("beyond the mapped horizon — the map ends short of this "
                     "cell. Drive closer and it can become legal; the map grows "
                     "as the rover moves"),
    IN_SHADOW: ("in the SHADOW of a mapped obstacle — something between the "
                "robot and this cell blocks the sensor. NO goal behind it can "
                "become legal until the rover MOVES to see past it. Waiting "
                "will not help"),
    OCCUPIED: "occupied in /map — something is there",
    INFLATED: ("mapped free but inside an inflation band — the planner may route "
               "here but the goal must sit on cost-0 floor"),
}


@dataclass(frozen=True)
class Grid:
    """A read-only occupancy-style grid. `values` is any indexable sequence."""
    values: object
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float

    def index(self, x: float, y: float) -> Optional[int]:
        cx = int((x - self.origin_x) / self.resolution)
        cy = int((y - self.origin_y) / self.resolution)
        if not (0 <= cx < self.width and 0 <= cy < self.height):
            return None
        return cy * self.width + cx

    def at(self, x: float, y: float):
        i = self.index(x, y)
        return None if i is None else self.values[i]

    def cell_centre(self, cx: int, cy: int) -> Tuple[float, float]:
        return (self.origin_x + (cx + 0.5) * self.resolution,
                self.origin_y + (cy + 0.5) * self.resolution)


@dataclass(frozen=True)
class Verdict:
    code: str
    reason: str
    #: Nearest legal alternative as (x, y), or None if the search found none.
    nearest: Optional[Tuple[float, float]] = None
    #: Distance from the REQUESTED goal to `nearest`, metres.
    nearest_offset_m: Optional[float] = None

    @property
    def legal(self) -> bool:
        return self.code == LEGAL

    @property
    def improves_with_motion(self) -> bool:
        """True when driving somewhere else can make THIS cell legal later.

        The operator-facing difference between the two unknown cases: a horizon
        recedes as you drive, a shadow only moves if you move around it — and
        neither is a reason to keep retyping the same goal.
        """
        return self.code in (BEYOND_HORIZON, IN_SHADOW)


def _ray_blocked(map_grid: Grid, x0: float, y0: float, x1: float, y1: float):
    """Is there a mapped-occupied cell strictly between the two points?

    Sampled at half-resolution steps, which cannot miss a cell wider than one
    cell. Returns the first blocking point, or None.
    """
    dist = math.hypot(x1 - x0, y1 - y0)
    if dist <= 0.0:
        return None
    step = map_grid.resolution * 0.5
    n = max(1, int(dist / step))
    for i in range(1, n):
        t = i / n
        px, py = x0 + (x1 - x0) * t, y0 + (y1 - y0) * t
        v = map_grid.at(px, py)
        if v is not None and v > 0:
            return (px, py)
    return None


def classify(map_grid: Grid, cost_grid: Grid, robot_xy, goal_xy) -> Verdict:
    """Why this goal is (il)legal. Same three checks the trinity makes, with the
    unknown case SPLIT by whether an obstacle explains it."""
    gx, gy = goal_xy
    map_val = map_grid.at(gx, gy)
    cost_val = cost_grid.at(gx, gy)
    if map_val is None or cost_val is None:
        return Verdict(OFF_GRID, ADVICE[OFF_GRID])
    if map_val == -1:
        blocker = _ray_blocked(map_grid, robot_xy[0], robot_xy[1], gx, gy)
        if blocker is not None:
            d = math.hypot(blocker[0] - robot_xy[0], blocker[1] - robot_xy[1])
            return Verdict(IN_SHADOW,
                           f"{ADVICE[IN_SHADOW]} (the blocker is {d:.2f} m out, "
                           f"on this bearing)")
        return Verdict(BEYOND_HORIZON, ADVICE[BEYOND_HORIZON])
    if map_val != 0:
        return Verdict(OCCUPIED, f"{ADVICE[OCCUPIED]} (value {map_val})")
    if cost_val != 0:
        return Verdict(INFLATED, f"{ADVICE[INFLATED]} (cost {cost_val} here)")
    return Verdict(LEGAL, ADVICE[LEGAL])


def nearest_legal(map_grid: Grid, cost_grid: Grid, robot_xy, goal_xy,
                  max_offset_m: float = 1.5):
    """The legal cell closest to the REQUESTED goal, or None.

    Ranked by distance from the request, so the proposal is the smallest change
    to what the operator asked for. Ties break toward the robot (a shorter drive
    over a longer one). Bounded by `max_offset_m`: a proposal a long way from the
    request is not the same errand, and silently flying it would be the tool
    deciding where to go.
    """
    gx, gy = goal_xy
    best = None
    span = int(max_offset_m / map_grid.resolution) + 1
    gi_x = int((gx - map_grid.origin_x) / map_grid.resolution)
    gi_y = int((gy - map_grid.origin_y) / map_grid.resolution)
    for cy in range(gi_y - span, gi_y + span + 1):
        if not (0 <= cy < map_grid.height):
            continue
        for cx in range(gi_x - span, gi_x + span + 1):
            if not (0 <= cx < map_grid.width):
                continue
            if map_grid.values[cy * map_grid.width + cx] != 0:
                continue
            wx, wy = map_grid.cell_centre(cx, cy)
            if cost_grid.at(wx, wy) != 0:
                continue
            offset = math.hypot(wx - gx, wy - gy)
            if offset > max_offset_m:
                continue
            key = (round(offset, 4), math.hypot(wx - robot_xy[0], wy - robot_xy[1]))
            if best is None or key < best[0]:
                best = (key, (wx, wy), offset)
    return None if best is None else (best[1], best[2])


def assess(map_grid: Grid, cost_grid: Grid, robot_xy, goal_xy,
           max_offset_m: float = 1.5) -> Verdict:
    """classify(), plus a proposal when the answer is no."""
    verdict = classify(map_grid, cost_grid, robot_xy, goal_xy)
    if verdict.legal:
        return verdict
    found = nearest_legal(map_grid, cost_grid, robot_xy, goal_xy, max_offset_m)
    if found is None:
        return verdict
    (nx, ny), offset = found
    return Verdict(verdict.code, verdict.reason, (nx, ny), offset)
