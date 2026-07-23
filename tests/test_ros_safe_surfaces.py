import math

import pytest

from sphero_rvr_core.responses import EncoderCounts
from sphero_rvr_core.state import RVRState, VelocityCommand
from sphero_rvr_driver.diagnostics import (
    BatterySnapshot,
    DiagnosticTelemetry,
    diagnostic_key_values,
    summarize_state,
)
from sphero_rvr_driver.led import normalize_rgb255
from sphero_rvr_driver.odometry import DifferentialOdomConfig, DifferentialOdomTracker
from sphero_rvr_driver.rvr_node import RVRNodeConfig


def test_differential_odom_tracker_integrates_encoder_counts():
    tracker = DifferentialOdomTracker(
        DifferentialOdomConfig(counts_per_meter=1000.0, wheel_track_m=0.25)
    )

    assert tracker.update(EncoderCounts(left=100, right=100), stamp=1.0) is None
    sample = tracker.update(EncoderCounts(left=200, right=300), stamp=3.0)

    assert sample is not None
    assert sample.frame_id == "odom"
    assert sample.child_frame_id == "base_link"
    assert sample.x == pytest.approx(0.14701, abs=1e-5)
    assert sample.y == pytest.approx(0.02980, abs=1e-5)
    assert sample.yaw == pytest.approx(0.4)
    assert sample.linear_mps == pytest.approx(0.075)
    assert sample.angular_rad_s == pytest.approx(0.2)


def test_differential_odom_tracker_wraps_heading_and_moves_in_current_heading():
    tracker = DifferentialOdomTracker(
        DifferentialOdomConfig(counts_per_meter=100.0, wheel_track_m=0.5)
    )
    tracker.update(EncoderCounts(left=0, right=0), stamp=0.0)

    sample = tracker.update(EncoderCounts(left=-400, right=400), stamp=1.0)
    assert sample is not None
    assert -math.pi <= sample.yaw <= math.pi

    moved = tracker.update(EncoderCounts(left=-300, right=500), stamp=2.0)
    assert moved is not None
    assert abs(moved.y) > 0.01


def test_diagnostic_key_values_include_safe_telemetry_without_identifiers():
    fields = diagnostic_key_values(
        RVRState(
            connected=True,
            emergency_stopped=False,
            latest_velocity=VelocityCommand(0.1, -0.2),
            fail_safe_active=True,
            fail_safe_reason="safe stop delivery failed",
            motor_transport_write_count=12,
            motion_transport_write_count=10,
            last_motor_command_id=0x01,
            last_motor_sequence_id=42,
            last_motor_payload_hex="02df01df",
            last_motor_transport_write_epoch_s=123.5,
            last_motion_transport_write_epoch_s=123.4,
        ),
        DiagnosticTelemetry(
            battery=BatterySnapshot(percentage=42, voltage=7.4),
            battery_voltage_state="low",
            motor_fault=True,
            motor_stall_event_count=2,
            last_motor_stall_index=1,
            last_motor_stall_active=True,
            left_motor_temperature_c=41.25,
            left_motor_thermal_status=0,
            right_motor_temperature_c=42.5,
            right_motor_thermal_status=1,
            motor_diagnostics_notification_enabled=True,
            firmware_version="1.2.3",
            board_revision=7,
            processor_name="nRF52840",
            core_uptime_s=12345,
        ),
    )

    assert fields["connected"] == "true"
    assert fields["emergency_stopped"] == "false"
    assert fields["fail_safe_active"] == "true"
    assert fields["fail_safe_reason"] == "safe stop delivery failed"
    assert fields["linear_mps"] == "0.100"
    assert fields["angular_rad_s"] == "-0.200"
    assert fields["battery_percent"] == "42"
    assert fields["battery_voltage"] == "7.400"
    assert fields["battery_voltage_state"] == "low"
    assert fields["motor_fault"] == "true"
    assert fields["motor_transport_write_count"] == "12"
    assert fields["motion_transport_write_count"] == "10"
    assert fields["last_motor_command_id"] == "0x01"
    assert fields["last_motor_sequence_id"] == "42"
    assert fields["last_motor_payload_hex"] == "02df01df"
    assert fields["last_motor_transport_write_epoch_s"] == "123.500000"
    assert fields["last_motion_transport_write_epoch_s"] == "123.400000"
    assert fields["motor_stall_event_count"] == "2"
    assert fields["last_motor_stall_index"] == "1"
    assert fields["last_motor_stall_active"] == "true"
    assert fields["left_motor_temperature_c"] == "41.250"
    assert fields["left_motor_thermal_status"] == "0"
    assert fields["right_motor_temperature_c"] == "42.500"
    assert fields["right_motor_thermal_status"] == "1"
    assert fields["motor_diagnostics_notification_enabled"] == "true"
    assert fields["firmware_version"] == "1.2.3"
    assert fields["board_revision"] == "7"
    assert fields["processor_name"] == "nRF52840"
    assert fields["core_uptime_s"] == "12345"
    assert "mac_address" not in fields
    assert "stats_id" not in fields


def test_summarize_state_reports_motor_fault_as_error():
    summary = summarize_state(
        RVRState(connected=True), DiagnosticTelemetry(motor_fault=True)
    )

    assert summary.level == "ERROR"
    assert "motor fault" in summary.message


def test_summarize_state_reports_fail_safe_as_error_before_motor_fault():
    summary = summarize_state(
        RVRState(connected=True, fail_safe_active=True, fail_safe_reason="stale velocity command: injected failure"),
        DiagnosticTelemetry(motor_fault=True),
    )

    assert summary.level == "ERROR"
    assert "fail-safe" in summary.message
    assert "stale velocity command" in summary.message


def test_rgb_helper_clamps_0_to_255_for_safe_led_surfaces():
    assert normalize_rgb255(-1, 127.9, 300) == (0, 128, 255)


def test_node_config_declares_safe_surface_defaults():
    config = RVRNodeConfig()

    assert config.odom_publish_period == 0.1
    assert config.odom_counts_per_meter == 4337.768
    assert config.odom_wheel_track_m == 0.2507
    assert config.odom_frame_id == "odom"
    assert config.base_frame_id == "base_link"
    assert config.odom_publish_tf is True
    assert config.odom_pose_xy_covariance == 0.05
    assert config.odom_pose_yaw_covariance == 0.25
    assert config.odom_twist_linear_covariance == 0.10
    assert config.odom_twist_angular_covariance == 0.50
    assert config.ambient_light_publish_period == 2.0
    assert config.diagnostics_metadata_period == 30.0
    assert config.motor_diagnostics_poll_period == 0.5
