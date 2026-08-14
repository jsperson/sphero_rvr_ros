"""Coverage + frontier exploration: keep going until every reachable free cell has
been both SEEN (no frontiers left) and physically APPROACHED (the rover has driven
within a coverage radius of it).

Frontier exploration (explore_lite) stops when everything reachable has been *seen*
by the lidar — it may never drive near a spot it only glanced at from across the
room. A coverage mission additionally requires the rover to get within
``coverage_radius_m`` of every reachable free cell (what a close-range camera needs
for semantic inspection). This module unifies both: a cell is a *target* if it is
uncovered OR a frontier, and the mission is done only when no reachable target
remains.

Pure (no ROS) so it can be unit-tested and wired into a node. Works on an
occupancy grid (``occ``: -1 unknown, 0 free, >0 occupied) plus a ``covered`` set of
world-grid coordinates that the node grows as the rover drives. Coverage is tracked
in a framing-independent world grid (quantized at the map resolution) so a shifting
SLAM map origin does not invalidate it.

This module answers "what is worth going to, nearest first" and deliberately not
"can the rover get there" — the planner owns that question and is the only thing
that can answer it correctly.
"""

from dataclasses import dataclass
from collections import deque
import math
from typing import Tuple


@dataclass(frozen=True)
class CoverageConfig:
    # The rover "covers" a cell by driving within this distance of it.
    coverage_radius_m: float = 0.75
    # Ignore target clusters smaller than this many cells (sensor/edge noise) so
    # the rover does not chase a lone speck across the room.
    min_cluster_cells: int = 5
    # Also pursue frontiers (free next to unknown), so the mission discovers new
    # space as well as covering known space. False = cover known space only.
    include_frontiers: bool = True
    # A cell counts as free when 0 <= value <= this (0 = strict trinary map).
    free_threshold: int = 0
    # How many candidates to offer per selection. The caller asks the PLANNER about
    # each in turn and takes the first that yields a path, so this bounds how many
    # planner queries one selection can cost. Reachability is not decided here.
    max_candidates: int = 12
    # Never offer a cluster cell closer to the robot than this (world metres).
    # The caller refuses goals inside its own minimum goal distance, so a cluster
    # represented by its very nearest cell can be silently unofferable in full: a
    # frontier arc 0.25 m away is a real target the mission wants, but its nearest
    # cell yields no goal and the cluster then contributes NOTHING -- the likely
    # end-of-mission cause in D14. When the nearest cell is inside this distance,
    # the representative moves outward to the nearest cluster cell beyond it; a
    # cluster entirely inside is skipped for THIS pose (distances are re-measured
    # from the live pose every cycle, so moving re-offers it). 0.0 = old behaviour.
    min_offer_distance_m: float = 0.0


def world_grid(wx: float, wy: float, res: float) -> Tuple[int, int]:
    """Framing-independent integer coordinate of a world point on a `res` grid."""
    return (math.floor(wx / res), math.floor(wy / res))


def cell_center_world(cx: int, cy: int, origin_x: float, origin_y: float, res: float):
    """World coordinate of the center of map cell (cx, cy)."""
    return (origin_x + (cx + 0.5) * res, origin_y + (cy + 0.5) * res)


def stamp_coverage(covered: set, robot_wx: float, robot_wy: float, res: float, radius_m: float) -> None:
    """Mark every world-grid cell within `radius_m` of the rover as covered."""
    r_cells = int(math.ceil(radius_m / res))
    gx0, gy0 = world_grid(robot_wx, robot_wy, res)
    r2 = radius_m * radius_m
    for dy in range(-r_cells, r_cells + 1):
        for dx in range(-r_cells, r_cells + 1):
            gx, gy = gx0 + dx, gy0 + dy
            # cell center of this world-grid cell
            cxw, cyw = (gx + 0.5) * res, (gy + 0.5) * res
            if (cxw - robot_wx) ** 2 + (cyw - robot_wy) ** 2 <= r2:
                covered.add((gx, gy))


def _is_free(occ, idx: int, free_threshold: int) -> bool:
    v = occ[idx]
    return 0 <= v <= free_threshold


def is_frontier(occ, w: int, h: int, cx: int, cy: int, free_threshold: int = 0) -> bool:
    """A free cell with at least one 4-neighbor that is unknown (occ < 0)."""
    idx = cy * w + cx
    if not _is_free(occ, idx, free_threshold):
        return False
    for nx, ny in ((cx - 1, cy), (cx + 1, cy), (cx, cy - 1), (cx, cy + 1)):
        if 0 <= nx < w and 0 <= ny < h and occ[ny * w + nx] < 0:
            return True
    return False


def cell_world_grid(cx: int, cy: int, origin_x: float, origin_y: float, res: float) -> Tuple[int, int]:
    """World-grid coordinate of map cell (cx, cy) — framing-independent, so a
    shifting SLAM origin doesn't invalidate `covered`/`suppressed` (both are keyed
    by world grid, not map cell)."""
    wx, wy = cell_center_world(cx, cy, origin_x, origin_y, res)
    return world_grid(wx, wy, res)


def candidate_goals(
    occ,
    w: int,
    h: int,
    origin_x: float,
    origin_y: float,
    res: float,
    robot_cx: int,
    robot_cy: int,
    covered: set,
    suppressed: set,
    config: CoverageConfig,
) -> list:
    """Propose target cells in nearest-first order. Does NOT decide navigability.

    A *target* is a free cell that is uncovered OR (if enabled) a frontier, and is
    not suppressed. ``covered`` and ``suppressed`` are sets of WORLD-GRID coords
    (see ``cell_world_grid``), not map cells, so they survive a shifting SLAM
    origin. "Nearest" is by a flood over free space from the robot, which also
    excludes cells walled off from the robot — map connectivity is a fact the
    occupancy grid can state. A target is only proposed if it belongs to a cluster
    of at least ``min_cluster_cells`` (to skip sensor noise), and at most one cell
    per cluster is proposed.

    Whether a proposal is actually *drivable* is the planner's question, and only
    the planner can answer it: this module previously eroded the map by an inscribed
    radius and flooded over the result as a stand-in, which disagreed with the real
    costmap often enough to need a blacklist, a TTL and a watchdog to contain the
    disagreement. The caller now asks ``ComputePathToPose`` about these candidates in
    order and takes the first that plans. Returns up to ``max_candidates`` cells.
    """
    if not (0 <= robot_cx < w and 0 <= robot_cy < h):
        return []

    def is_free(cx: int, cy: int) -> bool:
        return _is_free(occ, cy * w + cx, config.free_threshold)

    def is_target(cx: int, cy: int) -> bool:
        if not is_free(cx, cy):
            return False
        wg = cell_world_grid(cx, cy, origin_x, origin_y, res)
        if wg in suppressed:
            return False
        if wg not in covered:
            return True
        if config.include_frontiers and is_frontier(occ, w, h, cx, cy, config.free_threshold):
            return True
        return False

    def cluster_cells(cx: int, cy: int, seen_cluster: set) -> list:
        """Flood connected target cells (4-connected) from (cx, cy); return them."""
        stack = [(cx, cy)]
        seen_cluster.add((cx, cy))
        cells = []
        while stack:
            x, y = stack.pop()
            cells.append((x, y))
            for nx, ny in ((x - 1, y), (x + 1, y), (x, y - 1), (x, y + 1)):
                if 0 <= nx < w and 0 <= ny < h and (nx, ny) not in seen_cluster and is_target(nx, ny):
                    seen_cluster.add((nx, ny))
                    stack.append((nx, ny))
        return cells

    def representative(cells: list):
        """The cell to offer for a cluster: the nearest one the caller can USE.

        The flood reaches a cluster at its nearest cell, but a cell closer than
        min_offer_distance_m yields no goal at the caller (a goal where the rover
        already stands succeeds without moving, forever), so offering it silences
        the whole cluster. Offer the nearest cell at a USABLE distance instead --
        with one cell of margin, since the caller measures from its live world
        pose and this measures from the robot's cell. None when every cell is
        inside: unofferable from THIS pose, and re-measured after any move.
        """
        if config.min_offer_distance_m <= 0.0:
            return cells[0]
        min_cells = config.min_offer_distance_m / res + 1.0
        best = None
        best_d = math.inf
        for x, y in cells:
            d = math.hypot(x - robot_cx, y - robot_cy)
            if d >= min_cells and d < best_d:
                best, best_d = (x, y), d
        return best

    # BFS over free space from the robot, collecting one representative per
    # big-enough target cluster in nearest-first order. The robot's own cell is
    # always enqueued (it may sit in near-wall cost the planner dislikes, which is
    # the planner's business, not this flood's).
    visited = bytearray(w * h)
    visited[robot_cy * w + robot_cx] = 1
    dq = deque([(robot_cx, robot_cy)])
    checked_cluster: set = set()
    found: list = []
    while dq:
        cx, cy = dq.popleft()
        if is_target(cx, cy) and (cx, cy) not in checked_cluster:
            cells = cluster_cells(cx, cy, checked_cluster)
            if len(cells) >= config.min_cluster_cells:
                rep = representative(cells)
                if rep is not None:
                    found.append(rep)
                    if len(found) >= config.max_candidates:
                        return found
        for nx, ny in ((cx - 1, cy), (cx + 1, cy), (cx, cy - 1), (cx, cy + 1)):
            if 0 <= nx < w and 0 <= ny < h:
                nidx = ny * w + nx
                if not visited[nidx] and _is_free(occ, nidx, config.free_threshold):
                    visited[nidx] = 1
                    dq.append((nx, ny))
    return found


# Nav2 publishes its costmap as an OccupancyGrid scaled 0..100 (-1 unknown), where
# 100 is lethal and 99 is the inscribed ring — a robot centre in either means the
# footprint is in collision.
INSCRIBED_COST = 99


def robot_start_blocked(costmap_data, width, height, origin_x, origin_y, resolution,
                        robot_x, robot_y, threshold=INSCRIBED_COST):
    """True if the robot's OWN cell is at/above inscribed cost in the costmap.

    When it is, the planner treats the START pose as in collision and returns "no
    valid path" for *every* goal, and Nav2's motion recoveries are all
    collision-blocked — so an explorer will churn goals and blacklist the whole map
    while going nowhere. Observed 2026-08-07: starting with 0.26 m rear clearance,
    below `robot_radius + inflation_radius` (0.14 + 0.16 = 0.30 m), produced exactly
    that and burned a four-minute run.

    Returns None when the robot is outside the costmap or the cell is unknown, so a
    caller can distinguish "definitely wedged" from "cannot tell".
    """
    if resolution <= 0.0 or width <= 0 or height <= 0:
        return None
    cx = int((robot_x - origin_x) / resolution)
    cy = int((robot_y - origin_y) / resolution)
    if not (0 <= cx < width and 0 <= cy < height):
        return None
    value = costmap_data[cy * width + cx]
    if value < 0:
        return None
    return value >= threshold
