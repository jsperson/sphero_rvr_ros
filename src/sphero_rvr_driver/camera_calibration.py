"""Camera calibration checks for semantic localization inputs.

The helpers in this module stay ROS-import-free so launch/config tests can run on
development hosts. They accept any CameraInfo-like object with width, height, k,
d, and distortion_model attributes, including sensor_msgs/msg/CameraInfo at
runtime.
"""

from __future__ import annotations

from collections.abc import Iterable
from typing import TypeVar

CameraInfoLike = TypeVar("CameraInfoLike")


def _float_sequence(value: object) -> list[float]:
    if value is None:
        return []
    if not isinstance(value, Iterable):
        return []
    return [float(item) for item in value]


def camera_info_is_configured(camera_info: object) -> bool:
    """Return True only when CameraInfo carries usable intrinsic calibration.

    camera_ros can publish an empty CameraInfo when no calibration URL/file is
    configured. Semantic localization must not silently project detections from
    zero dimensions or an all-zero K matrix, because that produces confident
    nonsense.
    """

    width = int(getattr(camera_info, "width", 0) or 0)
    height = int(getattr(camera_info, "height", 0) or 0)
    k = _float_sequence(getattr(camera_info, "k", getattr(camera_info, "K", [])))
    distortion_model = str(getattr(camera_info, "distortion_model", "") or "")

    if width <= 0 or height <= 0:
        return False
    if len(k) != 9 or not any(abs(value) > 1e-12 for value in k):
        return False
    if abs(k[0]) <= 1e-12 or abs(k[4]) <= 1e-12 or abs(k[8]) <= 1e-12:
        return False
    if not distortion_model.strip():
        return False
    return True


def require_configured_camera_info(
    camera_info: CameraInfoLike, *, context: str = "camera projection"
) -> CameraInfoLike:
    """Return ``camera_info`` or raise when intrinsics are empty/unconfigured."""

    if camera_info_is_configured(camera_info):
        return camera_info
    raise ValueError(
        f"camera intrinsics are unconfigured for {context}: "
        "CameraInfo width/height, K, and distortion_model must be populated "
        "from a measured calibration file before semantic localization runs"
    )
