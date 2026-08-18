#!/usr/bin/env python3
"""Generate the open map the mark-detour acceptance runs on.

WHY A SYNTHETIC MAP, AND WHY THAT IS NOT CHEATING. The three-clause bar asks whether the
planner ROUTES AROUND a contact mark. That question needs a detour to exist. Run on
`mission_20260816_171934` it cannot be asked at all: measured 2026-08-18, the free y-span
containing the robot's own path is 0.40-0.60 m where the mark sits and 1.05 m at its
widest anywhere on the route, while a body-width strip plus the measured 0.1519 m
inscribed radius sterilises ~0.51 m. There was never a way past. The bar was sound; its
PRECONDITION was false in that room.

So the acceptance certifies PLANNER BEHAVIOUR against a known geometry, rather than
certifying one recorded room's furniture -- which is what no-room-specific-solutions asks
for. The parameters are kept realistic so the numbers transfer: same 0.05 m resolution as
every costmap in this project, and the room is walled rather than infinite so the raycast
scan returns real ranges inside `obstacle_max_range` instead of nothing.

A GENERATOR RATHER THAN A COMMITTED PGM: a binary blob cannot be reviewed, and every
number that matters here (size, resolution, origin, where the walls are) should be
readable in the diff.

    python3 make_open_rig_map.py --out-dir /tmp
    ros2 launch sphero_rvr_driver sim_closed_loop.launch.py map_yaml:=/tmp/open_rig_room.yaml
"""

from __future__ import annotations

import argparse
from pathlib import Path

#: Metres. 6 x 6 gives >= 2.5 m of clear floor either side of a mark at the origin --
#: comfortably past the 1.2 m precondition, with room for the planner to choose either
#: side rather than being funnelled into the only gap.
SIZE_M = 6.0
RESOLUTION_M = 0.05
#: Wall thickness in cells. Two cells so a raycast cannot slip between them diagonally.
WALL_CELLS = 2

FREE = 254        # PGM value read as free at the default free_thresh
OCCUPIED = 0      # read as occupied


def build_pgm(size_m: float, resolution_m: float, wall_cells: int) -> tuple[bytes, int]:
    n = int(round(size_m / resolution_m))
    rows = []
    for j in range(n):
        if j < wall_cells or j >= n - wall_cells:
            rows.append(bytes([OCCUPIED] * n))
        else:
            row = bytearray([FREE] * n)
            for k in range(wall_cells):
                row[k] = OCCUPIED
                row[n - 1 - k] = OCCUPIED
            rows.append(bytes(row))
    header = f"P5\n{n} {n}\n255\n".encode()
    return header + b"".join(rows), n


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--out-dir", default="/tmp")
    ap.add_argument("--name", default="open_rig_room")
    ap.add_argument("--size-m", type=float, default=SIZE_M)
    ap.add_argument("--resolution", type=float, default=RESOLUTION_M)
    a = ap.parse_args()

    out = Path(a.out_dir)
    pgm, n = build_pgm(a.size_m, a.resolution, WALL_CELLS)
    (out / f"{a.name}.pgm").write_bytes(pgm)

    # Origin centres the room on (0,0): the sim pins map->odom to identity and the robot
    # starts at the odom origin, so the robot starts in the middle with clear floor all
    # round -- which is also the >0.30 m start clearance this project needs before
    # anything plans at all.
    half = a.size_m / 2.0
    (out / f"{a.name}.yaml").write_text(
        f"image: {a.name}.pgm\n"
        f"mode: trinary\n"
        f"resolution: {a.resolution}\n"
        f"origin: [{-half}, {-half}, 0.0]\n"
        f"negate: 0\n"
        f"occupied_thresh: 0.65\n"
        f"free_thresh: 0.196\n"
    )
    print(f"wrote {out / (a.name + '.pgm')} ({n}x{n} cells, {a.size_m} m at "
          f"{a.resolution} m/cell) and {out / (a.name + '.yaml')}")
    print(f"clear floor from {-half + WALL_CELLS * a.resolution:+.2f} to "
          f"{half - WALL_CELLS * a.resolution:+.2f} m on both axes")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
