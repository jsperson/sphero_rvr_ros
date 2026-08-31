#!/usr/bin/env python3
"""Extract D60's two costmap frames from the flight-4 bag, as the falsifier's input.

    python3 scripts/extract_d60_frames.py --bag ~/bag_20260820_161932 --out-dir artifacts/d60_falsifier

WHY THE FRAMES AND NOT A SYNTHETIC ROOM. D60 is the corridor that was OPEN for 221 s
and then painted shut mid-run by contact-mark promotions -- 23 LETHAL seeds amplified
x19 by the 0.30 m inflation into 447 cells, with 0 of those cells supported by any of
23,027 live lidar returns. A synthetic pocket would test a shape we chose. These are
the recorded grids: the room as the rover actually believed it, immediately before and
after the closure.

THE NUMBERS THIS MUST REPRODUCE, or the extraction is wrong and must not be used:
447 cells newly >=253, of which 23 newly 254. Both are checked here and the script
refuses to write on a mismatch. They come from the archive's own analysis
(`artifacts/belief_basin_geometry_2026-08-20/basin_geometry4.py`), re-run against this
bag on 2026-08-31 and reproducing exactly.

WINDOW AND PREDICATE ARE THE ARCHIVE'S, NOT THIS FILE'S. `WINDOW0` is the artifact's
t+0; T_OPEN/T_CLOSED are its +221/+250; "newly blocked" is its `b >= 253 > a`. A first
attempt at this used +240 and `b == 254 != a`, which counted 52 cells that merely
HARDENED from 253 to 254 as new seeds -- already-blocked floor scored as new door.
The aggregate still landed within 1% of correct, which is exactly why a matching total
must never close a question about parts.

Output is ASCII PGM (P2) rather than binary: a costmap that cannot be read in a diff
is a blob, and this one is evidence.
"""

from __future__ import annotations

import argparse
import pathlib

import numpy as np

WINDOW0 = 1787262267.0          # artifact t+0 (basin_geometry4.py)
T_OPEN, T_CLOSED = WINDOW0 + 221, WINDOW0 + 250
EXPECT_NEWLY_BLOCKED = 447
EXPECT_NEWLY_LETHAL = 23


def load_frames(bag: str):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from nav2_msgs.msg import Costmap

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id="mcap"),
                rosbag2_py.ConverterOptions("", ""))
    reader.set_filter(rosbag2_py.StorageFilter(topics=["/global_costmap/costmap_raw"]))
    open_f = closed_f = None
    while reader.has_next():
        _, data, stamp = reader.read_next()
        t = stamp / 1e9
        msg = deserialize_message(data, Costmap)
        md = msg.metadata
        frame = (t, md.resolution, md.origin.position.x, md.origin.position.y,
                 np.asarray(msg.data, np.uint8).reshape(md.size_y, md.size_x))
        if t <= T_OPEN or open_f is None:
            open_f = frame
        if t <= T_CLOSED or closed_f is None:
            closed_f = frame
    return open_f, closed_f


def write_pgm(path: pathlib.Path, grid: np.ndarray, comment: str) -> None:
    height, width = grid.shape
    with open(path, "w") as handle:
        handle.write("P2\n")
        for line in comment.splitlines():
            handle.write(f"# {line}\n")
        handle.write(f"{width} {height}\n255\n")
        for row in grid:
            handle.write(" ".join(str(int(v)) for v in row) + "\n")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--bag", required=True)
    ap.add_argument("--out-dir", required=True)
    args = ap.parse_args()

    open_f, closed_f = load_frames(args.bag)
    if open_f is None or closed_f is None:
        raise SystemExit("no /global_costmap/costmap_raw frames in that bag")
    if open_f[2:4] != closed_f[2:4] or open_f[4].shape != closed_f[4].shape:
        raise SystemExit("the costmap window rolled between the two frames; a cell-index "
                         "diff would compare different floor")

    a, b = open_f[4], closed_f[4]
    newly = (b >= 253) & (a < 253)
    newly_lethal = (b == 254) & (a < 253)
    if int(newly.sum()) != EXPECT_NEWLY_BLOCKED or int(newly_lethal.sum()) != EXPECT_NEWLY_LETHAL:
        raise SystemExit(
            f"extraction does not reproduce the archive: {int(newly.sum())} newly "
            f">=253 (want {EXPECT_NEWLY_BLOCKED}), {int(newly_lethal.sum())} newly 254 "
            f"(want {EXPECT_NEWLY_LETHAL}). Refusing to write frames nothing can trust.")

    out = pathlib.Path(args.out_dir)
    out.mkdir(parents=True, exist_ok=True)
    header = (f"D60 flight-4 global costmap, {args.bag}\n"
              f"resolution {open_f[1]} m, origin ({open_f[2]}, {open_f[3]})\n"
              f"values are nav2 costmap costs: 0 free, 253 inscribed, 254 lethal, 255 unknown")
    write_pgm(out / "d60_open.pgm", a, header + f"\nOPEN frame, artifact t+{open_f[0]-WINDOW0:.1f} s")
    write_pgm(out / "d60_closed.pgm", b, header + f"\nCLOSED frame, artifact t+{closed_f[0]-WINDOW0:.1f} s")
    (out / "geometry.txt").write_text(
        f"resolution {open_f[1]}\norigin_x {open_f[2]}\norigin_y {open_f[3]}\n"
        f"open_t {open_f[0]-WINDOW0:.1f}\nclosed_t {closed_f[0]-WINDOW0:.1f}\n"
        f"newly_blocked {int(newly.sum())}\nnewly_lethal {int(newly_lethal.sum())}\n")
    print(f"wrote {out}/d60_open.pgm, d60_closed.pgm, geometry.txt")
    print(f"reproduced the archive: {int(newly.sum())} newly >=253, "
          f"{int(newly_lethal.sum())} newly 254")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
