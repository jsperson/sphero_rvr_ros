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


def detect_floor_boundary(img, floor_band_frac=0.12, sample_center_frac=0.6, color_thresh=40.0, min_run=4):
    """For each image column, the pixel ROW of the lowest stable non-floor pixel
    (an obstacle's ground contact), or None if the column is clear floor.

    `img`: (H, W, C) uint8. `min_run`: how many consecutive non-floor pixels above
    the contact are required (rejects single-pixel noise). Returns a list length W.
    """
    h, w = img.shape[:2]
    ref = floor_reference(img, floor_band_frac, sample_center_frac)
    dist = np.linalg.norm(img.astype(np.float32) - ref, axis=2)  # (H, W)
    over = dist > color_thresh  # non-floor mask
    result = [None] * w
    for x in range(w):
        col = over[:, x]
        # Scan up from the bottom; the contact is the lowest row that begins a run
        # of >= min_run non-floor pixels (going upward).
        for y in range(h - 1, min_run - 2, -1):
            if col[y] and bool(np.all(col[y - min_run + 1 : y + 1])):
                result[x] = y
                break
    return result
