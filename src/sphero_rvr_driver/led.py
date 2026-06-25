"""LED helpers for bounded ROS-facing convenience surfaces."""

from __future__ import annotations


def _clamp_channel(value: float) -> int:
    return max(0, min(255, int(round(float(value)))))


def normalize_rgb255(r: float, g: float, b: float) -> tuple[int, int, int]:
    """Clamp numeric RGB inputs to 0..255 integer channels."""
    return (_clamp_channel(r), _clamp_channel(g), _clamp_channel(b))
