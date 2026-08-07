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
"""

from dataclasses import dataclass
from collections import deque
import math
from typing import Optional, Tuple


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
    # NAVIGABILITY: only target/traverse free cells at least this far from an
    # obstacle (the planner's inscribed radius). Cells closer than this become
    # lethal on the global costmap, so targeting them (or routing a target behind a
    # passage narrower than 2x this) yields "no valid path" churn. Matching the
    # planner's robot_radius makes coverage's reachability agree with the planner.
    inscribed_radius_m: float = 0.14
    # A map cell counts as an obstacle for the navigability check when value >= this.
    occupied_threshold: int = 50


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
    shifting SLAM origin doesn't invalidate `covered`/`blacklist` (both are keyed
    by world grid, not map cell)."""
    wx, wy = cell_center_world(cx, cy, origin_x, origin_y, res)
    return world_grid(wx, wy, res)


def compute_navigable(occ, w: int, h: int, inscribed_cells: int, occupied_threshold: int, free_threshold: int):
    """Bytearray mask (1=navigable) of free cells at least `inscribed_cells` from
    any obstacle — a stand-in for where the planner won't hit lethal cost. Both
    coverage targets and reachability flood are restricted to these, so coverage's
    idea of "reachable" matches the planner's (no targets behind too-narrow
    passages, no targets in soon-to-be-lethal near-wall cells)."""
    nav = bytearray(w * h)
    r = inscribed_cells
    r2 = r * r
    for cy in range(h):
        for cx in range(w):
            idx = cy * w + cx
            if not (0 <= occ[idx] <= free_threshold):
                continue  # not free
            blocked = False
            for dy in range(-r, r + 1):
                if blocked:
                    break
                ny = cy + dy
                if ny < 0 or ny >= h:
                    continue
                base = ny * w
                for dx in range(-r, r + 1):
                    if dx * dx + dy * dy > r2:
                        continue
                    nx = cx + dx
                    if 0 <= nx < w and occ[base + nx] >= occupied_threshold:
                        blocked = True
                        break
            if not blocked:
                nav[idx] = 1
    return nav


def select_next_goal(
    occ,
    w: int,
    h: int,
    origin_x: float,
    origin_y: float,
    res: float,
    robot_cx: int,
    robot_cy: int,
    covered: set,
    blacklist: set,
    config: CoverageConfig,
) -> Optional[Tuple[int, int]]:
    """Pick the nearest reachable target cell, or None if the mission is complete.

    A *target* is a free cell that is uncovered OR (if enabled) a frontier, and is
    not blacklisted. ``covered`` and ``blacklist`` are sets of WORLD-GRID coords
    (see ``cell_world_grid``), not map cells, so they survive a shifting SLAM
    origin. "Nearest" and "reachable" are by a flood over free space from the robot
    (so cells walled off from the robot are never chosen). A target is only returned
    if it belongs to a cluster of at least ``min_cluster_cells`` (to skip noise).
    Returns the goal cell (cx, cy), or None when no reachable target remains — the
    coverage+frontier mission is done.
    """
    if not (0 <= robot_cx < w and 0 <= robot_cy < h):
        return None

    inscribed_cells = max(0, int(round(config.inscribed_radius_m / res)))
    navigable = compute_navigable(
        occ, w, h, inscribed_cells, config.occupied_threshold, config.free_threshold
    )

    def is_target(cx: int, cy: int) -> bool:
        idx = cy * w + cx
        if not navigable[idx]:
            return False
        wg = cell_world_grid(cx, cy, origin_x, origin_y, res)
        if wg in blacklist:
            return False
        if wg not in covered:
            return True
        if config.include_frontiers and is_frontier(occ, w, h, cx, cy, config.free_threshold):
            return True
        return False

    def cluster_size(cx: int, cy: int, seen_cluster: set) -> int:
        """Flood connected target cells (4-connected) starting at (cx, cy)."""
        stack = [(cx, cy)]
        seen_cluster.add((cx, cy))
        size = 0
        while stack:
            x, y = stack.pop()
            size += 1
            for nx, ny in ((x - 1, y), (x + 1, y), (x, y - 1), (x, y + 1)):
                if 0 <= nx < w and 0 <= ny < h and (nx, ny) not in seen_cluster and is_target(nx, ny):
                    seen_cluster.add((nx, ny))
                    stack.append((nx, ny))
        return size

    # BFS over NAVIGABLE space from the robot; the first target (nearest) whose
    # cluster is big enough wins. Flooding over navigable-only cells makes
    # reachability match the planner: targets behind passages narrower than the
    # robot are never reached, so they are never selected -> no "no valid path"
    # churn. The robot's own start cell is always enqueued (it may sit just inside
    # the inscribed margin), but expansion only follows navigable cells.
    visited = bytearray(w * h)
    start = robot_cy * w + robot_cx
    visited[start] = 1
    dq = deque([(robot_cx, robot_cy)])
    checked_cluster: set = set()
    while dq:
        cx, cy = dq.popleft()
        if is_target(cx, cy) and (cx, cy) not in checked_cluster:
            if cluster_size(cx, cy, checked_cluster) >= config.min_cluster_cells:
                return (cx, cy)
        for nx, ny in ((cx - 1, cy), (cx + 1, cy), (cx, cy - 1), (cx, cy + 1)):
            nidx = ny * w + nx
            if 0 <= nx < w and 0 <= ny < h and not visited[nidx] and navigable[nidx]:
                visited[nidx] = 1
                dq.append((nx, ny))
    return None


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
