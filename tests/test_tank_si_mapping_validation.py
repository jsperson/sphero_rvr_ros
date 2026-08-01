import json
from pathlib import Path

import pytest

from sphero_rvr_driver.tank_si_mapping_validation import (
    TURN_PATH_RAW_DUTY,
    TURN_PATH_TANK_SI,
    TimedEncoderCounts,
    analyze_trial,
    analyze_turn_trial,
    main,
    summarize_turn_paths,
    summarize_turn_trials,
)


def _sample(stamp: float, left: int, right: int) -> TimedEncoderCounts:
    return TimedEncoderCounts(stamp, left, right)


def test_analysis_accepts_calibrated_005_speed_and_bounded_stop_travel() -> None:
    result = analyze_trial(
        commanded_mps=0.05,
        start=_sample(10.0, 1000, 2000),
        end=_sample(12.0, 1434, 2434),
        stopped=_sample(12.75, 1454, 2454),
        counts_per_meter=4337.768,
        speed_tolerance_fraction=0.20,
        max_post_stop_travel_m=0.01,
    )

    assert result.measured_mps == pytest.approx(0.0500269, rel=1e-4)
    assert result.post_stop_travel_m == pytest.approx(0.0046107, rel=1e-4)
    assert result.speed_within_tolerance is True
    assert result.stop_within_limit is True


def test_analysis_rejects_ten_x_speed_and_excess_stop_travel() -> None:
    result = analyze_trial(
        commanded_mps=0.05,
        start=_sample(1.0, 0, 0),
        end=_sample(3.0, 4338, 4338),
        stopped=_sample(3.75, 4555, 4555),
        counts_per_meter=4337.768,
        speed_tolerance_fraction=0.20,
        max_post_stop_travel_m=0.01,
    )

    assert result.measured_mps == pytest.approx(0.5, rel=1e-3)
    assert result.relative_error == pytest.approx(9.0, rel=1e-3)
    assert result.post_stop_travel_m > 0.01
    assert result.speed_within_tolerance is False
    assert result.stop_within_limit is False


def test_analysis_handles_signed_int32_encoder_rollover() -> None:
    result = analyze_trial(
        commanded_mps=0.05,
        start=_sample(0.0, 2**31 - 200, 2**31 - 200),
        end=_sample(2.0, -(2**31) + 234, -(2**31) + 234),
        stopped=_sample(2.75, -(2**31) + 234, -(2**31) + 234),
        counts_per_meter=4337.768,
        speed_tolerance_fraction=0.20,
        max_post_stop_travel_m=0.01,
    )

    assert result.measured_mps == pytest.approx(0.0500269, rel=1e-4)
    assert result.stop_within_limit is True


def _turn_result(
    commanded: float,
    left_delta: int,
    right_delta: int,
    command_path: str = TURN_PATH_TANK_SI,
):
    return analyze_turn_trial(
        command_path=command_path,
        commanded_angular_rad_s=commanded,
        start=_sample(10.0, 1000, 2000),
        end=_sample(11.0, 1000 + left_delta, 2000 + right_delta),
        stopped=_sample(11.75, 996 + left_delta, 2004 + right_delta),
        counts_per_meter=4337.768,
        wheel_track_m=0.2507,
        max_post_stop_travel_m=0.01,
    )


def test_turn_analysis_calculates_yaw_rate_and_sustained_symmetric_turn() -> None:
    result = _turn_result(0.6, -326, 326)

    assert result.commanded_left_mps == pytest.approx(-0.07521)
    assert result.commanded_right_mps == pytest.approx(0.07521)
    assert result.measured_angular_rad_s == pytest.approx(0.5995, rel=1e-3)
    assert result.relative_error < 0.01
    assert result.post_stop_yaw_rad == pytest.approx(0.00736, rel=1e-3)
    assert result.post_stop_track_travel_m < 0.001
    assert result.counter_rotating is True
    assert result.clean_pivot is True
    assert result.opposing_track_motion is True
    assert result.sustained is True
    assert result.stalled is False
    assert result.smooth is True
    assert result.stop_within_limit is True


def test_turn_analysis_classifies_real_but_asymmetric_motion_and_stall() -> None:
    stalled = _turn_result(0.4, -20, 20)
    asymmetric = _turn_result(0.6, -50, 326)
    smooth = _turn_result(0.8, -435, 435)
    summary = summarize_turn_trials((stalled, asymmetric, smooth))

    assert stalled.measured_angular_rad_s < 0.10
    assert stalled.stalled is True
    assert asymmetric.sustained is True
    assert asymmetric.track_asymmetry_fraction > 0.25
    assert asymmetric.smooth is False
    assert summary == {
        "first_sustained_angular_rad_s": 0.6,
        "first_smooth_angular_rad_s": 0.8,
        "stalled_angular_rates_rad_s": [0.4],
    }


def test_turn_analysis_reports_raw_duty_and_path_comparison() -> None:
    tank_si = _turn_result(0.6, 0, 0)
    raw_duty = _turn_result(0.6, -326, 326, TURN_PATH_RAW_DUTY)
    summary = summarize_turn_paths((tank_si, raw_duty))

    assert tank_si.commanded_left_mps == pytest.approx(-0.07521)
    assert tank_si.commanded_left_duty is None
    assert raw_duty.commanded_left_mps is None
    assert raw_duty.commanded_left_duty == -57
    assert raw_duty.commanded_right_duty == 57
    assert raw_duty.counter_rotating is True
    assert raw_duty.clean_pivot is True
    assert summary["clean_pivot_paths"] == [TURN_PATH_RAW_DUTY]
    assert summary["path_summaries"][TURN_PATH_TANK_SI][
        "actuates_clean_pivot"
    ] is False
    assert summary["path_summaries"][TURN_PATH_RAW_DUTY][
        "first_counter_rotating_angular_rad_s"
    ] == 0.6


def test_default_cli_is_no_motion_and_does_not_open_serial(capsys) -> None:
    assert main([]) == 0
    output = capsys.readouterr().out
    assert json.loads(output.splitlines()[0])["plan"]["motion"] == "DISABLED"
    assert "MOTION_SKIPPED" in output


def test_default_turn_cli_is_bounded_no_motion_diagnostic(capsys) -> None:
    assert main(["--mode", "turn"]) == 0
    plan = json.loads(capsys.readouterr().out.splitlines()[0])["plan"]

    assert plan["motion"] == "DISABLED"
    assert plan["mode"] == "turn"
    assert plan["wheels_up_precheck"] == "NOT_CONFIRMED"
    assert set(plan["driver_paths"]) == {"tank_si", "raw_duty"}
    assert plan["angular_rates_rad_s"] == [0.2, 0.4, 0.6, 0.8, 1.0]
    assert plan["diagnostic_max_angular_rad_s"] == 1.0
    assert plan["hard_raw_turn_duty_ceiling"] == 96
    assert plan["max_nonzero_pulse_s"] == 0.75
    assert plan["warmup_s"] == 0.25
    assert plan["measurement_s"] == 0.5


@pytest.mark.parametrize(
    "argv",
    (
        ["--speeds", "0.06"],
        ["--speeds", "0.03"],
        ["--armed"],
        ["--armed", "--attended", "--surface", "floor"],
        ["--mode", "turn", "--armed", "--attended"],
        ["--mode", "turn", "--surface", "floor"],
        ["--mode", "turn", "--angular-rates", "1.01"],
        ["--mode", "turn", "--angular-rates", "0.6", "0.4"],
        ["--mode", "turn", "--warmup", "0.3"],
    ),
)
def test_cli_rejects_unbounded_or_unacknowledged_motion(argv) -> None:
    with pytest.raises(SystemExit):
        main(argv)


def test_hardware_harness_uses_real_driver_set_velocity_path() -> None:
    source = Path(__file__).resolve().parents[1].joinpath(
        "src", "sphero_rvr_driver", "tank_si_mapping_validation.py"
    ).read_text(encoding="utf-8")

    assert "RVRDriver(" in source
    assert "RVRDriver.VELOCITY_CONTROL_NATIVE_TANK_SI" in source
    assert "RVRDriver.VELOCITY_CONTROL_RAW_MOTOR" in source
    assert "driver.set_velocity(" in source
    assert "commands.drive_tank_si_units" not in source
    assert "commands.drive_tank_normalized" not in source
    assert "driver.raw_motors(" not in source
