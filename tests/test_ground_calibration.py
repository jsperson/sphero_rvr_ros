from __future__ import annotations

import pytest

from sphero_rvr_driver.ground_calibration import GroundCalibrationError, analyze_ground_sample


def _terminal_result(**overrides):
    result = {
        "source_sha": "calibration-sha",
        "route_id": "route-1",
        "terminal_reason": "complete",
        "terminal_settled": True,
        "collision_state": "CLEAR",
        "measured_distance_m": 0.24,
        "left_encoder_delta_counts": 1000,
        "right_encoder_delta_counts": 1020,
    }
    result.update(overrides)
    return {"mission_id": "mission-1", "result": result}


def test_ground_sample_computes_independent_track_scale_and_error():
    sample = analyze_ground_sample(_terminal_result(), actual_distance_m=0.25)

    assert sample["left_counts_per_meter"] == pytest.approx(4000.0)
    assert sample["right_counts_per_meter"] == pytest.approx(4080.0)
    assert sample["mean_counts_per_meter"] == pytest.approx(4040.0)
    assert sample["odom_error_pct"] == pytest.approx(-4.0)
    assert sample["track_mismatch_pct"] == pytest.approx(100 * 20 / 1010)
    assert sample["eligible_for_calibration_set"] is True


def test_ground_sample_keeps_failed_target_error_eligible_when_settled_and_clear():
    sample = analyze_ground_sample(
        _terminal_result(terminal_reason="target_error", measured_distance_m=0.28),
        actual_distance_m=0.26,
    )

    assert sample["terminal_reason"] == "target_error"
    assert sample["eligible_for_calibration_set"] is True


@pytest.mark.parametrize(
    "overrides",
    (
        {"terminal_settled": False},
        {"collision_state": "BLOCKED"},
        {"terminal_reason": "stopped"},
        {"terminal_reason": "stop_requested"},
        {"terminal_reason": "estop_latched"},
        {"terminal_reason": "cancelled"},
        {"left_encoder_delta_counts": 800, "right_encoder_delta_counts": 1000},
    ),
)
def test_ground_sample_rejects_unsafe_or_track_mismatched_runs_from_set(overrides):
    sample = analyze_ground_sample(_terminal_result(**overrides), actual_distance_m=0.25)
    assert sample["eligible_for_calibration_set"] is False


@pytest.mark.parametrize("actual", (0.0, -1.0, float("nan"), float("inf")))
def test_ground_sample_rejects_invalid_actual_distance(actual):
    with pytest.raises(GroundCalibrationError, match="actual_distance_m"):
        analyze_ground_sample(_terminal_result(), actual_distance_m=actual)


def test_ground_sample_requires_nonzero_encoder_evidence():
    with pytest.raises(GroundCalibrationError, match="show motion"):
        analyze_ground_sample(
            _terminal_result(left_encoder_delta_counts=0),
            actual_distance_m=0.25,
        )
