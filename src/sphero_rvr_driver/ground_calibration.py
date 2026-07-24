"""Ground-distance calibration analysis for persisted live-route evidence."""

from __future__ import annotations

import math
import statistics
from typing import Any, Mapping, Sequence


GROUND_SAMPLE_SCHEMA = "sphero_rvr.ground_calibration_sample.v1"
GROUND_SET_SCHEMA = "sphero_rvr.ground_calibration_set.v1"
MINIMUM_GROUND_SAMPLES = 3
MAX_SET_DEVIATION_PCT = 5.0


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
        "schema": GROUND_SAMPLE_SCHEMA,
        "source_sha": str(result.get("source_sha", "unknown")),
        "mission_id": str(payload.get("mission_id", result.get("mission_id", ""))),
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


def aggregate_ground_samples(samples: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
    """Produce a fail-closed repeatability report from analyzed ground samples.

    A set becomes eligible for human config review only when it contains at
    least three distinct, individually eligible mission executions from one
    exact source SHA and every mean counts-per-meter value is within five
    percent of the median. Identical approved proposals intentionally share a
    route ID, so mission ID is the repeat identity when it is available. No
    config file is read or changed here.
    """

    if len(samples) < MINIMUM_GROUND_SAMPLES:
        raise GroundCalibrationError(
            f"at least {MINIMUM_GROUND_SAMPLES} analyzed ground samples are required"
        )

    normalized: list[dict[str, Any]] = []
    for index, sample in enumerate(samples, start=1):
        if not isinstance(sample, Mapping) or sample.get("schema") != GROUND_SAMPLE_SCHEMA:
            raise GroundCalibrationError(f"sample {index} has an unsupported schema")
        source_sha = str(sample.get("source_sha", "")).strip()
        mission_id = str(sample.get("mission_id", "")).strip()
        route_id = str(sample.get("route_id", "")).strip()
        if not source_sha or source_sha == "unknown":
            raise GroundCalibrationError(f"sample {index} is missing an exact source SHA")
        if not route_id:
            raise GroundCalibrationError(f"sample {index} is missing a route_id")
        numeric: dict[str, float] = {}
        for field in (
            "actual_distance_m",
            "left_counts_per_meter",
            "right_counts_per_meter",
            "mean_counts_per_meter",
            "track_mismatch_pct",
            "odom_error_pct",
        ):
            try:
                value = float(sample[field])
            except (KeyError, TypeError, ValueError) as exc:
                raise GroundCalibrationError(f"sample {index} has invalid {field}") from exc
            if not math.isfinite(value):
                raise GroundCalibrationError(f"sample {index} has invalid {field}")
            numeric[field] = value
        if numeric["actual_distance_m"] <= 0.0 or any(
            numeric[field] <= 0.0
            for field in ("left_counts_per_meter", "right_counts_per_meter", "mean_counts_per_meter")
        ):
            raise GroundCalibrationError(f"sample {index} contains non-positive calibration evidence")
        normalized.append(
            {
                "source_sha": source_sha,
                "mission_id": mission_id,
                "route_id": route_id,
                "sample_identity": (
                    f"mission:{mission_id}" if mission_id else f"route:{route_id}"
                ),
                "eligible": sample.get("eligible_for_calibration_set") is True,
                **numeric,
            }
        )

    source_shas = sorted({sample["source_sha"] for sample in normalized})
    mission_ids = [sample["mission_id"] for sample in normalized]
    route_ids = [sample["route_id"] for sample in normalized]
    sample_identities = [sample["sample_identity"] for sample in normalized]
    means = [sample["mean_counts_per_meter"] for sample in normalized]
    median_mean = statistics.median(means)
    deviations = [100.0 * abs(value - median_mean) / median_mean for value in means]
    max_deviation_pct = max(deviations)
    rejection_reasons = []
    if len(source_shas) != 1:
        rejection_reasons.append("samples do not share one exact source SHA")
    if len(set(sample_identities)) != len(sample_identities):
        rejection_reasons.append("mission execution identities are not unique")
    if not all(sample["eligible"] for sample in normalized):
        rejection_reasons.append("one or more samples failed the individual safety/evidence gate")
    if max_deviation_pct > MAX_SET_DEVIATION_PCT:
        rejection_reasons.append(
            f"counts-per-meter deviation exceeds {MAX_SET_DEVIATION_PCT:.1f}%"
        )
    eligible = not rejection_reasons
    return {
        "schema": GROUND_SET_SCHEMA,
        "sample_count": len(normalized),
        "source_sha": source_shas[0] if len(source_shas) == 1 else None,
        "mission_ids": mission_ids,
        "route_ids": route_ids,
        "actual_distance_m": [sample["actual_distance_m"] for sample in normalized],
        "median_left_counts_per_meter": statistics.median(
            sample["left_counts_per_meter"] for sample in normalized
        ),
        "median_right_counts_per_meter": statistics.median(
            sample["right_counts_per_meter"] for sample in normalized
        ),
        "median_mean_counts_per_meter": median_mean,
        "minimum_mean_counts_per_meter": min(means),
        "maximum_mean_counts_per_meter": max(means),
        "maximum_median_deviation_pct": max_deviation_pct,
        "maximum_track_mismatch_pct": max(
            sample["track_mismatch_pct"] for sample in normalized
        ),
        "maximum_absolute_odom_error_pct": max(
            abs(sample["odom_error_pct"]) for sample in normalized
        ),
        "eligible_for_config_review": eligible,
        "suggested_odom_counts_per_meter": median_mean if eligible else None,
        "rejection_reasons": rejection_reasons,
        "adoption_note": (
            "Human review is still required; compare floor, payload, battery, heading drift, "
            "and the longer-distance validation before editing config."
        ),
    }
