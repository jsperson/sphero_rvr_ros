"""Raycast a 2D occupancy grid — the geometry half of the simulated lidar.

Pure functions, no ROS, so the thing that decides what the robot "sees" is testable on any
machine. The node wrapper (`sphero_rvr_driver.sim_raycast_scan`) supplies the pose from TF
and publishes the result; everything that can be wrong about the geometry is here.

Loaded from a real recorded mission map, so the simulated room is **the actual room** —
door gap, cluttered corner and all — and a collision abort in simulation happens where one
happened in life.
"""

from __future__ import annotations

import math
import re
from dataclasses import dataclass
from typing import List, Tuple

import numpy as np


@dataclass(frozen=True)
class OccupancyMap:
    """An occupancy grid: True where a cell blocks a beam."""

    blocked: np.ndarray  # [height, width] bool
    resolution: float
    origin_x: float
    origin_y: float

    @property
    def height(self) -> int:
        return int(self.blocked.shape[0])

    @property
    def width(self) -> int:
        return int(self.blocked.shape[1])


def parse_map_yaml(text: str) -> dict:
    """Minimal map_server YAML reader — enough fields, no dependency."""
    out: dict = {}
    for key in ("image", "mode"):
        match = re.search(rf"^{key}\s*:\s*(\S+)", text, re.M)
        if match:
            out[key] = match.group(1)
    for key in ("resolution", "occupied_thresh", "free_thresh", "negate"):
        match = re.search(rf"^{key}\s*:\s*([-\d.eE]+)", text, re.M)
        if match:
            out[key] = float(match.group(1))
    match = re.search(r"^origin\s*:\s*\[([^\]]+)\]", text, re.M)
    if match:
        out["origin"] = [float(v) for v in match.group(1).split(",")]
    return out


def parse_pgm(data: bytes) -> np.ndarray:
    """Binary P5 PGM -> [height, width] uint8, as map_server writes."""
    if not data.startswith(b"P5"):
        raise ValueError("not a binary P5 PGM")
    fields: List[int] = []
    index = 2
    while len(fields) < 3:
        while index < len(data) and data[index : index + 1].isspace():
            index += 1
        if data[index : index + 1] == b"#":
            while index < len(data) and data[index] != 0x0A:
                index += 1
            continue
        start = index
        while index < len(data) and not data[index : index + 1].isspace():
            index += 1
        fields.append(int(data[start:index]))
    index += 1  # single whitespace after maxval
    width, height, _maxval = fields
    pixels = np.frombuffer(data[index : index + width * height], dtype=np.uint8)
    return pixels.reshape((height, width))


def load_map(yaml_text: str, pgm_bytes: bytes) -> OccupancyMap:
    """Build an OccupancyMap using map_server's own thresholding convention.

    PGM rows are stored top-down while the grid's origin is bottom-left, so the image is
    flipped. Getting that backwards mirrors the world, which is the N1 class of bug and
    exactly the kind a simulator is supposed not to introduce.
    """
    meta = parse_map_yaml(yaml_text)
    image = parse_pgm(pgm_bytes).astype(np.float32) / 255.0
    if meta.get("negate", 0.0):
        occupancy = image
    else:
        occupancy = 1.0 - image
    blocked = occupancy >= float(meta.get("occupied_thresh", 0.65))
    blocked = np.flipud(blocked)
    origin = meta.get("origin", [0.0, 0.0, 0.0])
    return OccupancyMap(
        blocked=np.ascontiguousarray(blocked),
        resolution=float(meta["resolution"]),
        origin_x=float(origin[0]),
        origin_y=float(origin[1]),
    )


def raycast(
    grid: OccupancyMap,
    x: float,
    y: float,
    yaw: float,
    angles: np.ndarray,
    max_range: float,
    step: float = 0.0,
) -> np.ndarray:
    """Ranges (m) from world pose (x, y, yaw) along `angles` measured from `yaw`.

    Vectorised over beams: every beam marches together, so 720 beams at 10 Hz is
    comfortable on the Pi. A beam that leaves the grid or reaches `max_range` returns
    `inf`, which is what a real scanner reports for "nothing there".
    """
    if step <= 0.0:
        step = grid.resolution * 0.5
    steps = max(1, int(math.ceil(max_range / step)))

    world_angles = angles + yaw
    dx = np.cos(world_angles)
    dy = np.sin(world_angles)

    ranges = np.full(angles.shape, np.inf, dtype=np.float64)
    alive = np.ones(angles.shape, dtype=bool)

    for index in range(1, steps + 1):
        distance = index * step
        px = x + dx * distance
        py = y + dy * distance
        col = ((px - grid.origin_x) / grid.resolution).astype(np.int32)
        row = ((py - grid.origin_y) / grid.resolution).astype(np.int32)

        inside = (
            (col >= 0) & (col < grid.width) & (row >= 0) & (row < grid.height) & alive
        )
        if not inside.any():
            if not alive.any():
                break
            continue

        hit = np.zeros(angles.shape, dtype=bool)
        idx = np.nonzero(inside)[0]
        hit[idx] = grid.blocked[row[idx], col[idx]]
        newly = hit & alive
        if newly.any():
            ranges[newly] = distance
            alive &= ~newly
        if not alive.any():
            break

    return ranges
