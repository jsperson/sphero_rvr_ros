#!/usr/bin/env python3
"""Bench item J's analysis: does the ToF agree with the lidar about the same surface?

WHY THIS EXISTS AS A SCRIPT. The check that found the frame bug was run once, by hand,
on a Pi, and the script did not survive the session -- only its CSV did. That CSV was
enough to re-derive the answer, but only because it happened to carry both components
of each return. The next capture should not depend on that luck.

    python3 diagnostics/tof_lidar_frame_check.py <probe.csv> [more.csv ...]

INPUT: the J probe CSV, one row per (frame, column):

    epoch,col,tof_ground_m,tof_z_m,lidar_min_m,disagreement_m,scan_age_s

WHAT IT ANSWERS, and the two are different questions:

  1. IS THE FRAME RIGHT?  A missing translation shows up as a CONSTANT disagreement
     across a wide range span -- that is how the 0.10 m `mount_x_m` omission was found
     (median 0.10 m nearer, constant from 0.72 m to 1.90 m; a sensor artifact would
     have scaled). So the script reports the residual AT NEAR AND FAR RANGE separately.
     A frame error is flat; an optical or cone effect grows with range.

  2. WHAT MARGIN DOES RULE B NEED?  Design 9.8's criterion: the per-column disagreement
     on a BARE WALL must stay inside the chosen margin in >= 99% of frames, and the
     margin is then set FROM that distribution. That is a per-column number and a
     uniform stand-in for it has already produced one retracted finding.

WHAT IT DOES NOT DO: it does not choose the margin. It prints the distribution the
margin must be chosen from.

THE ROW IS RECOVERED, NOT ASSUMED. `tof_z_m` is untouched by an x-offset bug, so the
ray length follows from it exactly (r = (z - mount_height) / uz), and the row is
whichever candidate reproduces the recorded `tof_ground_m`. That means this script can
re-analyse a CSV written by BROKEN code and report what the fixed geometry would have
said -- which is what the frame fix's acceptance gate needed.
"""
import csv
import math
import statistics
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from sphero_rvr_core.tof_frame import TofConfig, ZONES, zone_ray  # noqa: E402

#: A floor return is not looking at the wall the lidar reports, so including it
#: measures the floor. Returns at or below this height are floor.
FLOOR_Z_M = 0.05
#: How closely a candidate row must reproduce the recorded ground range to be believed.
IDENT_TOLERANCE_M = 0.005


def identify(col, ground_recorded, z, cfg):
    """(row, ray_length) for the zone that produced this sample, or None."""
    best = None
    for row in range(ZONES):
        ux, uy, uz = zone_ray(row, col, cfg)
        if abs(uz) < 1e-9:
            continue
        r = (z - cfg.mount_height_m) / uz
        if r <= 0:
            continue
        err = abs(r * math.hypot(ux, uy) - ground_recorded)
        if best is None or err < best[0]:
            best = (err, row, r)
    if best is None or best[0] > IDENT_TOLERANCE_M:
        return None
    return best[1], best[2]


def load(path, cfg):
    samples, unusable = [], 0
    for rec in csv.DictReader(open(path)):
        try:
            col = int(rec["col"])
            recorded = float(rec["tof_ground_m"])
            z = float(rec["tof_z_m"])
            lidar = float(rec["lidar_min_m"])
        except (KeyError, ValueError):
            unusable += 1
            continue
        if not math.isfinite(lidar) or lidar <= 0:
            unusable += 1
            continue
        found = identify(col, recorded, z, cfg)
        if found is None:
            unusable += 1
            continue
        row, r = found
        ux, uy, _uz = zone_ray(row, col, cfg)
        fixed = math.hypot(cfg.mount_x_m + r * ux, r * uy)
        samples.append({"col": col, "row": row, "z": z, "lidar": lidar,
                        "recorded": recorded, "fixed": fixed})
    return samples, unusable


def report(path, cfg):
    samples, unusable = load(path, cfg)
    print(f"\n{'=' * 76}\n{Path(path).name}   "
          f"{len(samples)} usable samples, {unusable} unusable\n{'=' * 76}")
    if not samples:
        print("  nothing to analyse")
        return

    wall = [s for s in samples if s["z"] > FLOOR_Z_M]
    print(f"same-surface samples: {len(wall)}   "
          f"floor returns excluded: {len(samples) - len(wall)}")
    if not wall:
        print("  every return is floor -- this probe cannot test the frame. A frame "
              "check needs the ToF and the lidar looking at the SAME surface.")
        return

    print("\n col |   n  | lidar  | as recorded |  disagree | re-analysed |  disagree")
    print(" " + "-" * 74)
    for col in range(ZONES):
        c = [s for s in wall if s["col"] == col]
        if not c:
            continue
        print(f"  {col}  | {len(c):4d} | {statistics.median(s['lidar'] for s in c):.4f} "
              f"|   {statistics.median(s['recorded'] for s in c):.4f}    "
              f"|  {statistics.median(s['lidar'] - s['recorded'] for s in c):+.4f} "
              f"|   {statistics.median(s['fixed'] for s in c):.4f}    "
              f"|  {statistics.median(s['lidar'] - s['fixed'] for s in c):+.4f}")

    resid = [s["lidar"] - s["fixed"] for s in wall]
    print(f"\noverall median disagreement:  as recorded "
          f"{statistics.median(s['lidar'] - s['recorded'] for s in wall):+.4f} m  ->  "
          f"re-analysed {statistics.median(resid):+.4f} m")

    # THE FRAME TEST. Split by range: a missing translation is CONSTANT, an optical or
    # cone effect SCALES. This is the distinction that identified the bug, and it is
    # the one that confirms the fix.
    ranges = sorted(s["lidar"] for s in wall)
    cut = ranges[len(ranges) // 2]
    near = [s["lidar"] - s["fixed"] for s in wall if s["lidar"] <= cut]
    far = [s["lidar"] - s["fixed"] for s in wall if s["lidar"] > cut]
    if near and far:
        print(f"  near half (<= {cut:.2f} m): {statistics.median(near):+.4f} m")
        print(f"  far  half (>  {cut:.2f} m): {statistics.median(far):+.4f} m")
        print("  VERDICT: " + (
            "flat across range -- a residual TRANSLATION remains, the frame is still "
            "wrong somewhere"
            if abs(statistics.median(far) - statistics.median(near)) < 0.01
            and abs(statistics.median(near)) > 0.02
            else "scales with range or is negligible -- consistent with cone/optical "
                 "effects, not with a missing translation"))

    # THE MARGIN DISTRIBUTION, per column, which is what design 9.8 requires.
    print("\nper-column |disagreement| for a margin (design 9.8: >=99% inside it):")
    for col in range(ZONES):
        c = sorted(abs(s["lidar"] - s["fixed"]) for s in wall if s["col"] == col)
        if not c:
            continue
        p99 = c[min(len(c) - 1, int(0.99 * len(c)))]
        print(f"  col {col}: n={len(c):4d}  median {statistics.median(c):.4f}  "
              f"p99 {p99:.4f}  max {max(c):.4f}")
    # NO POOLED NUMBER IS PRINTED HERE, AND THAT IS THE POINT. Design 9.8's trap: the
    # background is PER COLUMN, and a single figure across the frame asks a question no
    # lidar poses. A column facing a real sub-lidar object legitimately disagrees by a
    # large amount -- that is rule B working, not noise -- and pooling it with the wall
    # columns produces a "margin" that is neither. Column 4 of J_probe2 is exactly that
    # case (a return at z = 0.119 m, below the 0.1905 m lidar plane, 0.49 m nearer than
    # the lidar), and a pooled p99 there reads 0.74 m against wall columns at 0.03 m.
    #
    # So the margin is read off the BARE-WALL columns, and deciding which columns those
    # are is the operator's job with the scene in front of them, not the script's.
    print("\n  NO pooled margin is printed: the background is per column (design 9.8).")
    print("  Read the margin from the columns that faced BARE WALL in this scene. A")
    print("  column with a large, consistent disagreement is a candidate real object --")
    print("  check its z against the lidar plane before calling it noise.")


def main(argv):
    if len(argv) < 2:
        print(__doc__)
        return 2
    cfg = TofConfig()
    print(f"geometry: mount_x_m={cfg.mount_x_m} mount_height_m={cfg.mount_height_m} "
          f"pitch={cfg.mount_pitch_deg} zone_deg_v={cfg.zone_deg_v}")
    for path in argv[1:]:
        report(path, cfg)
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
