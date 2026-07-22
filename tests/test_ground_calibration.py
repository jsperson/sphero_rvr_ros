from __future__ import annotations

import pytest

from sphero_rvr_driver.ground_calibration import (
    GroundCalibrationError,
    aggregate_ground_samples,
    analyze_ground_sample,
)


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


def _analyzed_sample(route_id: str, mean_counts: float, *, source_sha: str = "calibration-sha"):
    return {
        "schema": "sphero_rvr.ground_calibration_sample.v1",
        "source_sha": source_sha,
        "route_id": route_id,
        "actual_distance_m": 0.25,
        "left_counts_per_meter": mean_counts - 20.0,
        "right_counts_per_meter": mean_counts + 20.0,
        "mean_counts_per_meter": mean_counts,
        "track_mismatch_pct": 1.0,
        "odom_error_pct": -2.0,
        "eligible_for_calibration_set": True,
    }


def test_ground_set_uses_median_only_after_three_consistent_distinct_safe_samples():
    report = aggregate_ground_samples(
        [
            _analyzed_sample("route-1", 4300.0),
            _analyzed_sample("route-2", 4350.0),
            _analyzed_sample("route-3", 4400.0),
        ]
    )

    assert report["median_mean_counts_per_meter"] == pytest.approx(4350.0)
    assert report["suggested_odom_counts_per_meter"] == pytest.approx(4350.0)
    assert report["eligible_for_config_review"] is True
    assert report["rejection_reasons"] == []


@pytest.mark.parametrize(
    ("samples", "reason"),
    (
        (
            [
                _analyzed_sample("route-1", 4300.0),
                _analyzed_sample("route-2", 4350.0, source_sha="other-sha"),
                _analyzed_sample("route-3", 4400.0),
            ],
            "source SHA",
        ),
        (
            [
                _analyzed_sample("route-1", 4300.0),
                _analyzed_sample("route-1", 4350.0),
                _analyzed_sample("route-3", 4400.0),
            ],
            "not unique",
        ),
        (
            [
                _analyzed_sample("route-1", 4000.0),
                _analyzed_sample("route-2", 4350.0),
                _analyzed_sample("route-3", 4700.0),
            ],
            "deviation exceeds",
        ),
    ),
)
def test_ground_set_withholds_suggestion_for_mixed_duplicate_or_inconsistent_evidence(samples, reason):
    report = aggregate_ground_samples(samples)

    assert report["eligible_for_config_review"] is False
    assert report["suggested_odom_counts_per_meter"] is None
    assert any(reason in item for item in report["rejection_reasons"])


def test_ground_set_withholds_suggestion_when_one_sample_failed_individual_gate():
    samples = [
        _analyzed_sample("route-1", 4300.0),
        _analyzed_sample("route-2", 4350.0),
        _analyzed_sample("route-3", 4400.0),
    ]
    samples[1]["eligible_for_calibration_set"] = False

    report = aggregate_ground_samples(samples)

    assert report["eligible_for_config_review"] is False
    assert report["suggested_odom_counts_per_meter"] is None
    assert "safety/evidence" in report["rejection_reasons"][0]


def test_ground_set_requires_at_least_three_samples():
    with pytest.raises(GroundCalibrationError, match="at least 3"):
        aggregate_ground_samples(
            [_analyzed_sample("route-1", 4300.0), _analyzed_sample("route-2", 4350.0)]
        )
