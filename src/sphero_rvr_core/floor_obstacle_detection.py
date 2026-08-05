"""Monocular floor-boundary obstacle detection (first cut).

Under a flat-floor assumption: sample the floor's colour from a band at the bottom
of the image, then for each column scan upward for where the pixels stop looking
like floor -- that transition is an obstacle's ground-contact. Paired with
`ground_projection.pixel_to_ground`, each contact pixel becomes a metric point in
front of the robot for the costmap / brake.

This is a deliberately simple, fragile-but-useful v1 (colour-threshold, flat floor);
the design doc calls out a depth camera for robust low-obstacle safety. Pure (numpy
only) so it can be unit-tested.
"""

import numpy as np


def floor_reference(img, floor_band_frac=0.12, sample_center_frac=0.6):
    """Median colour of a bottom, horizontally-centered band (assumed floor)."""
    h, w = img.shape[:2]
    band = max(1, int(h * floor_band_frac))
    c0 = int(w * (1.0 - sample_center_frac) / 2.0)
    c1 = w - c0
    patch = img[h - band : h, c0:c1].reshape(-1, img.shape[2]).astype(np.float32)
    return np.median(patch, axis=0)


def adaptive_threshold(dist, floor_band_frac, percentile=95.0, k=2.0, bounds=(25.0, 60.0)):
    """A colour-distance threshold derived from the floor band's own distribution:
    `k` * the `percentile`-th distance among (assumed-floor) band pixels, clamped to
    `bounds`. Tracks lighting/carpet variation so a fixed number isn't needed."""
    h = dist.shape[0]
    band = max(1, int(h * floor_band_frac))
    return float(np.clip(k * np.percentile(dist[h - band : h, :], percentile), bounds[0], bounds[1]))


def detect_floor_boundary(
    img,
    floor_band_frac=0.12,
    sample_center_frac=0.6,
    color_thresh=40.0,
    min_run=4,
    min_rise=None,
    rise_frac=0.7,
):
    """For each image column, the pixel ROW of the lowest stable non-floor pixel
    (an obstacle's ground contact), or None if the column is clear floor.

    `img`: (H, W, C) uint8. `color_thresh`: fixed colour-distance threshold, or None
    to derive one adaptively from the floor band (robust to lighting). `min_run`:
    consecutive non-floor pixels required above a contact (rejects single-pixel
    noise). `min_rise`: if set, instead require >= `rise_frac` of the `min_rise`-pixel
    window above the contact to be non-floor -- tolerates small floor-coloured specks
    on a real object (e.g. laces) while still rejecting thin seams/reflections; a
    thin-but-tall obstacle (chair leg) still passes. Returns a list length W.
    """
    h, w = img.shape[:2]
    ref = floor_reference(img, floor_band_frac, sample_center_frac)
    dist = np.linalg.norm(img.astype(np.float32) - ref, axis=2)  # (H, W)
    thr = adaptive_threshold(dist, floor_band_frac) if color_thresh is None else float(color_thresh)
    over = dist > thr  # non-floor mask
    win = int(min_rise) if min_rise else int(min_run)
    need = float(rise_frac) if min_rise else 1.0  # 1.0 => strict consecutive run
    result = [None] * w
    for x in range(w):
        col = over[:, x]
        # Scan up from the bottom; the contact is the lowest row whose window above
        # is (mostly) non-floor.
        for y in range(h - 1, win - 2, -1):
            if col[y] and float(np.mean(col[y - win + 1 : y + 1])) >= need:
                result[x] = y
                break
    return result
