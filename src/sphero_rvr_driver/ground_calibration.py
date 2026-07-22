"""Ground-distance calibration analysis for persisted live-route evidence."""

from __future__ import annotations

import math
from typing import Any, Mapping


class GroundCalibrationError(ValueError):
    """Terminal evidence cannot support a ground calibration sample."""


def analyze_ground_sample(
    payload: Mapping[str, Any],
    *,
    actual_distance_m: float,
) -> dict[str, Any]:
    """Compare authoritative encoder/odom evidence with a tape measurement.

    ``payload`` may be either the raw route result or the web artifact wrapper
    containing a ``result`` object. The calculation never mutates robot config;
    repeated accepted samples are still required before adopting a scale.
    """

    try:
        measured = float(actual_distance_m)
    except (TypeError, ValueError) as exc:
        raise GroundCalibrationError("actual_distance_m must be positive and finite") from exc
    if not math.isfinite(measured) or measured <= 0.0:
        raise GroundCalibrationError("actual_distance_m must be positive and finite")

    result = payload.get("result", payload)
    if not isinstance(result, Mapping):
        raise GroundCalibrationError("terminal result must be an object")
    try:
        left_delta = abs(int(result["left_encoder_delta_counts"]))
        right_delta = abs(int(result["right_encoder_delta_counts"]))
    except (KeyError, TypeError, ValueError) as exc:
        raise GroundCalibrationError("terminal encoder deltas are required") from exc
    if left_delta <= 0 or right_delta <= 0:
        raise GroundCalibrationError("terminal encoder deltas must show motion")

    try:
        odom_distance_m = float(result["measured_distance_m"])
    except (KeyError, TypeError, ValueError) as exc:
        raise GroundCalibrationError("terminal measured_distance_m is required") from exc
    if not math.isfinite(odom_distance_m) or odom_distance_m < 0.0:
        raise GroundCalibrationError("terminal measured_distance_m must be finite and non-negative")

    left_counts_per_meter = left_delta / measured
    right_counts_per_meter = right_delta / measured
    mean_counts_per_meter = (left_delta + right_delta) / (2.0 * measured)
    track_ratio = left_delta / right_delta
    track_mismatch_pct = 100.0 * abs(left_delta - right_delta) / ((left_delta + right_delta) / 2.0)
    odom_error_pct = 100.0 * (odom_distance_m - measured) / measured
    settled = result.get("terminal_settled") is True
    collision_clear = str(result.get("collision_state", "")).upper() == "CLEAR"
    terminal_reason = str(result.get("terminal_reason", ""))
    # A settled target_error run is intentionally useful: the controller's
    # error is exactly what the tape measurement calibrates.  Everything else
    # is fail-closed instead of trying to enumerate every possible unsafe or
    # interrupted terminal reason.
    eligible = bool(
        settled
        and collision_clear
        and track_mismatch_pct <= 10.0
        and terminal_reason in {"complete", "target_error"}
    )
    return {
        "schema": "sphero_rvr.ground_calibration_sample.v1",
        "source_sha": str(result.get("source_sha", "unknown")),
        "route_id": str(result.get("route_id", "")),
        "terminal_reason": terminal_reason,
        "terminal_settled": settled,
        "collision_state": str(result.get("collision_state", "UNKNOWN")),
        "actual_distance_m": measured,
        "odom_distance_m": odom_distance_m,
        "odom_error_pct": odom_error_pct,
        "left_encoder_delta_counts": left_delta,
        "right_encoder_delta_counts": right_delta,
        "left_counts_per_meter": left_counts_per_meter,
        "right_counts_per_meter": right_counts_per_meter,
        "mean_counts_per_meter": mean_counts_per_meter,
        "left_right_ratio": track_ratio,
        "track_mismatch_pct": track_mismatch_pct,
        "eligible_for_calibration_set": eligible,
        "adoption_note": "Use the median of repeated eligible ground samples; never adopt one run.",
    }
