import json
from pathlib import Path

import pytest

from sphero_rvr_driver.tank_si_mapping_validation import (
    TimedEncoderCounts,
    analyze_trial,
    main,
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


def test_default_cli_is_no_motion_and_does_not_open_serial(capsys) -> None:
    assert main([]) == 0
    output = capsys.readouterr().out
    assert json.loads(output.splitlines()[0])["plan"]["motion"] == "DISABLED"
    assert "MOTION_SKIPPED" in output


@pytest.mark.parametrize(
    "argv",
    (
        ["--speeds", "0.06"],
        ["--speeds", "0.03"],
        ["--armed"],
        ["--armed", "--attended", "--surface", "floor"],
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
    assert "driver.set_velocity(" in source
    assert "commands.drive_tank_si_units" not in source
    assert "driver.raw_motors(" not in source
