"""Replay the low-obstacle pipeline over saved frames. No ROS, no robot, no sun.

D27 was found by blocking the sun and watching 72 brake-band detections on bare floor
collapse to 0. That single-variable experiment is the acceptance test for any fix --
but it needs late-afternoon sun, which is not a dependency a regression test can have.
So the matched frame pair from 2026-08-10 becomes a permanent fixture: identical
scene, identical pose, identical stack, differing ONLY in illumination.

This replays the WHOLE chain the node runs -- resize to proc_width, spans, height
gate, ground projection, range gate -- and reports the number that matters: how many
points land in the CAMERA BRAKE'S ACTIVE BAND (camera_min_range .. camera_stop_distance,
0.40-0.50 m deployed). Points outside that band do not stop the rover.

Replaying only `detect_floor_boundary` is NOT a valid test and will mislead you: every
column of a normal room has wall or door above the floor, so it flags 800/800 columns
in both frames. The discrimination lives downstream in the height gate. (I made this
mistake first; it is recorded so the next person does not.)

Usage:
    python3 low_obstacle_replay.py <frame.png> [frame2.png ...]
    python3 low_obstacle_replay.py --pair          # the D27 fixtures, if present
"""

import argparse
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from sphero_rvr_core.floor_obstacle_detection import detect_obstacle_spans
from sphero_rvr_core.ground_projection import object_height_m, pixel_to_ground

# The deployed camera calibration (rvr_pi_camera3_800x600) and node defaults. Kept
# literal so the replay reproduces the field configuration rather than whatever a
# local YAML happens to say.
K_FULL = (680.559811904291, 679.9742509979817, 538.1721064472238, 299.2447559724674)
FULL_W = 800
PROC_W = 200
CAM_H = 0.1143
TILT = -0.0524            # 3 deg UP
FLOOR_BAND_FRAC = 0.12
MIN_RUN = 4
MIN_H, MAX_H = 0.020, 0.20
MIN_R, MAX_R = 0.05, 2.0
# The collision supervisor's camera brake only acts on points in this band.
BRAKE_MIN, BRAKE_MAX = 0.40, 0.50


def _resize(img, out_w, out_h):
    """Nearest-neighbour resize with numpy, so the replay needs no OpenCV.

    The node uses cv2.resize (bilinear). Nearest is a slightly different
    resampling, which can shift a boundary by a pixel; it does NOT change the
    conclusion this fixture tests (whether a light edge produces brake-band points
    at all), and it keeps the fixture runnable on any machine. Stated because a
    reader comparing counts against the live node deserves to know they will not
    match to the unit.
    """
    h, w = img.shape[:2]
    ys = (np.arange(out_h) * (h / out_h)).astype(int).clip(0, h - 1)
    xs = (np.arange(out_w) * (w / out_w)).astype(int).clip(0, w - 1)
    return img[ys][:, xs]


def replay(img_rgb, adaptive=True, color_thresh=40.0):
    """Return (all_points, brake_band_points) in robot base frame metres."""
    h0, w0 = img_rgb.shape[:2]
    s = PROC_W / float(w0)
    small = _resize(img_rgb, PROC_W, max(1, int(h0 * s)))
    fx, fy, cx, cy = (v * s for v in K_FULL)

    spans = detect_obstacle_spans(
        small,
        floor_band_frac=FLOOR_BAND_FRAC,
        color_thresh=None if adaptive else color_thresh,
        min_run=MIN_RUN,
    )
    pts, band = [], []
    for u, span in enumerate(spans):
        if span is None:
            continue
        contact, top = span
        g = pixel_to_ground(u, contact, fx, fy, cx, cy, CAM_H, TILT)
        if g is None:
            continue
        fwd, left = g
        rng = math.hypot(fwd, left)
        est_h = object_height_m(contact - top, rng, fy)
        if not (MIN_H <= est_h <= MAX_H):
            continue                     # tall -> the lidar's job
        if not (MIN_R <= fwd <= MAX_R):
            continue
        pts.append((fwd, left))
        if BRAKE_MIN <= rng <= BRAKE_MAX:
            band.append((fwd, left))
    return pts, band


def report(path, **kw):
    from PIL import Image

    img = np.asarray(Image.open(path).convert("RGB")).astype(np.uint8)
    pts, band = replay(img, **kw)
    bearings = [math.degrees(math.atan2(l, f)) for f, l in band]
    span = ("%+.1f..%+.1f" % (min(bearings), max(bearings))) if bearings else "-"
    print("  %-44s obstacle_pts=%4d   BRAKE-BAND=%3d   bearings %s"
          % (os.path.basename(path), len(pts), len(band), span))
    return len(band)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("frames", nargs="*")
    ap.add_argument("--pair", action="store_true",
                    help="replay the D27 sun-lit / sun-blocked fixture pair")
    args = ap.parse_args()

    frames = list(args.frames)
    if args.pair:
        d = os.path.expanduser("~/Downloads")
        frames = [os.path.join(d, "rvr_cloudcheck_183001.png"),
                  os.path.join(d, "rvr_sunblocked_183447.png")]
    if not frames:
        ap.error("give frame paths or --pair")

    print("low-obstacle replay — deployed config, brake band %.2f-%.2f m"
          % (BRAKE_MIN, BRAKE_MAX))
    counts = [report(f) for f in frames if os.path.exists(f)]
    if len(counts) == 2:
        print("\n  sun-lit -> blocked brake-band change: %d -> %d" % (counts[0], counts[1]))
        print("  A CORRECT DETECTOR SHOWS ~0 IN BOTH. The sun-lit count IS the defect.")


if __name__ == "__main__":
    main()
