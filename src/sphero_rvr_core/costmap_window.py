"""A readable square of the costmap around a pose, for the moment it matters.

D43: on 2026-08-15 `START POSE BLOCKED` fired on floor that measured OPEN on all
twelve bearings at a minimum of 0.410 m -- three times the inscribed radius -- with
no freeze marks in range (run 2's block predates its first mark by 17.5 s; run 1's
marks were 1.3 m away and TTL-expired). The costmap said the robot's own cell was
lethal and every other instrument said the floor was clear.

THE MECHANISM IS CONVICTED AS OF 2026-08-16, AND IT WAS NEITHER CANDIDATE.

The two suspects were a SLAM static layer carrying stale occupancy, or a map-frame
pose offset putting "own cell" on genuinely occupied ground. On gauntlet mission 1
this dump fired eight times and the answer was simpler and worse: the block was
TRUE. The rover had buried its own cell in inflation by planting five freeze marks
that cannot be cleared (`clearing: false`), each inflating to a ~0.56 m sterilised
disc. Costmap fidelity was never the problem; our own marks were.

AND THIS FILE GOT ITS OWN ANSWER BACKWARDS WHILE REPORTING IT. The window prints
`centre_blocked` and renders `#` for at/above-inscribed cells. Both used a local
`INSCRIBED_COST = 253` -- the raw `costmap_2d` scale -- against a
`nav_msgs/OccupancyGrid` whose values stop at 100 and encode inscribed as 99. So on
the night it mattered, the instrument printed `centre_blocked=False` beside a gate
that had correctly refused, and drew a robot buried in inscribed cost as a field of
`+`. The constant is now imported rather than copied, and pinned by
`tests/test_costmap_window_scale.py`.

So this is not a debugging convenience. It is the ONE measurement that turned a
standing suspicion -- open since 2026-08-10 -- into a conviction, and it had to be
taken at the moment, by the robot, because nothing else could. It is also proof that
an instrument needs its own adversary: this one was believed for four minutes on the
strength of a number it had computed wrongly.

Pure: a flat costmap array in, text out. No ROS, so the replay and the tests bind on
a machine with no rclpy.
"""

from typing import Optional

from sphero_rvr_core.coverage_exploration import INSCRIBED_COST

#: RE-EXPORTED, NOT REDEFINED, and the difference cost a gauntlet mission.
#:
#: This module used to carry its own copy, with the reasoning: "Duplicated from
#: `coverage_exploration` rather than imported so this module stays standalone; the
#: value is a Nav2 constant, not a tuning choice." Both halves of that sentence are
#: true and together they produced the defect: it IS a Nav2 constant, but there are
#: TWO of them. The raw `costmap_2d` scale runs 0..255 with 253 = inscribed; the
#: `nav_msgs/OccupancyGrid` that Nav2 PUBLISHES runs -1..100 with 99 = inscribed.
#: The copy here was 253, against a grid whose maximum value is 100.
#:
#: WHAT IT COST (2026-08-16 gauntlet mission 1). The explorer's gate reads the
#: OccupancyGrid and compared correctly against 99, so `START POSE BLOCKED` was
#: TRUE -- the rover really had buried its own cell. The D43 auto-dump, built
#: specifically to convict that mechanism, compared 99 >= 253, printed
#: `centre_blocked=False`, and rendered every inscribed cell as `+` instead of `#`.
#: The instrument built to find the truth reported its exact inverse, on its first
#: flight, with the whole batch watching.
#:
#: The standalone-ness this bought was worth nothing: `coverage_exploration` is pure,
#: imports nothing but the standard library, and does not import this module -- so
#: there is no cycle and never was. One constant, one author.
__all__ = ["INSCRIBED_COST", "CostmapWindow", "extract_window", "format_window"]


class CostmapWindow:
    """A square of cells centred on a world pose, with the pose's own cell marked."""

    def __init__(self, cells, radius_cells, centre_cx, centre_cy, resolution,
                 centre_value):
        self.cells = cells                  # list of rows, TOP row first for printing
        self.radius_cells = radius_cells
        self.centre_cx = centre_cx
        self.centre_cy = centre_cy
        self.resolution = resolution
        self.centre_value = centre_value    # None = unknown or out of bounds

    @property
    def centre_is_blocked(self) -> Optional[bool]:
        if self.centre_value is None:
            return None
        return self.centre_value >= INSCRIBED_COST


def extract_window(costmap_data, width, height, origin_x, origin_y, resolution,
                   world_x, world_y, radius_m) -> Optional[CostmapWindow]:
    """Cells within `radius_m` of (world_x, world_y), or None if unreadable.

    Indexing matches `coverage_exploration.robot_start_blocked` exactly --
    `cy * width + cx`, origin at the grid's lower-left, row-major -- because a dump
    that indexed differently from the check that fired would describe a different
    cell than the one the planner refused, and the whole point is to see THAT cell.
    That is the assert-don't-infer rule applied to an instrument: it must read the
    map the same way its subject does.

    Cells outside the grid are `None` rather than 0. Off-map is not free space, and a
    dump that padded with zeros would draw open floor around a robot sitting at the
    map's edge -- inventing exactly the evidence the dump exists to gather.
    """
    if resolution <= 0.0 or width <= 0 or height <= 0 or radius_m < 0.0:
        return None
    cx = int((world_x - origin_x) / resolution)
    cy = int((world_y - origin_y) / resolution)
    radius_cells = int(radius_m / resolution)

    rows = []
    # Top row first: +y is up in the world, and a dump printed upside-down is how a
    # reader concludes the obstacle is behind the robot when it is in front. The
    # project has made a mirrored-convention error in both the control path and the
    # analysis layer already; this one is stated rather than assumed.
    for dy in range(radius_cells, -radius_cells - 1, -1):
        row = []
        for dx in range(-radius_cells, radius_cells + 1):
            gx, gy = cx + dx, cy + dy
            if 0 <= gx < width and 0 <= gy < height:
                row.append(costmap_data[gy * width + gx])
            else:
                row.append(None)
        rows.append(row)

    if 0 <= cx < width and 0 <= cy < height:
        centre = costmap_data[cy * width + cx]
        centre_value = None if centre < 0 else centre
    else:
        centre_value = None

    return CostmapWindow(rows, radius_cells, cx, cy, resolution, centre_value)


def format_window(window: Optional[CostmapWindow], world_x, world_y) -> str:
    """The window as text a human reads and a grep finds.

    One character per cell, because the shape is the evidence: a lethal blob AT the
    robot with clear cells around it reads as stale occupancy, while the robot's
    marker sitting on the edge of real structure reads as a pose offset. A table of
    numbers hides that; a picture does not.

        #  at/above inscribed (the planner refuses these)
        +  occupied but below inscribed
        .  free
        ?  unknown (-1)
        ' ' off the map entirely -- NOT free
        R  the robot's own cell, drawn over whatever it holds

    The legend travels WITH the dump. A dump whose legend lives in a source file is a
    dump that gets misread six months later by someone reading it out of an archive.
    """
    if window is None:
        return "COSTMAP_DUMP unavailable (no costmap, or a degenerate grid)"

    lines = [
        f"COSTMAP_DUMP world=({world_x:.3f},{world_y:.3f}) "
        f"cell=({window.centre_cx},{window.centre_cy}) "
        f"res={window.resolution:.3f} radius_cells={window.radius_cells} "
        f"centre_value={window.centre_value} "
        f"centre_blocked={window.centre_is_blocked} "
        f"inscribed={INSCRIBED_COST}",
        "legend: # >=inscribed  + occupied  . free  ? unknown  (space) off-map  R robot",
    ]
    mid = window.radius_cells
    for y, row in enumerate(window.cells):
        chars = []
        for x, value in enumerate(row):
            if y == mid and x == mid:
                chars.append("R")
            elif value is None:
                chars.append(" ")
            elif value < 0:
                chars.append("?")
            elif value >= INSCRIBED_COST:
                chars.append("#")
            elif value > 0:
                chars.append("+")
            else:
                chars.append(".")
        lines.append("  " + "".join(chars))
    return "\n".join(lines)
