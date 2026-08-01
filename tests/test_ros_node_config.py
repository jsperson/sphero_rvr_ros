from pathlib import Path

from sphero_rvr_driver.rvr_node import RVRNodeConfig, create_driver
from sphero_rvr_core.driver import RVRDriver
from sphero_rvr_core.fake_transport import FakeTransport


def test_default_node_config_uses_pi_serial_alias_and_floor_turn_motor_duty():
    config = RVRNodeConfig()

    assert config.serial_port == "/dev/ttyAMA0"
    assert config.baud_rate == 115200
    assert config.max_raw_motor_duty == 160
    assert config.max_linear_raw_motor_duty == 64
    assert config.max_angular_raw_motor_duty == 255
    assert config.pivot_raw_motor_duty == 24
    assert config.velocity_control_mode == RVRDriver.VELOCITY_CONTROL_RAW_MOTOR
    assert config.safety_dispatch_timeout_s == 0.10
    assert config.battery_publish_period == 5.0
    assert config.temperature_publish_period == 2.0
    assert config.diagnostics_publish_period == 1.0
    assert config.motor_diagnostics_poll_period == 0.5
    assert config.odom_counts_per_meter == 4337.768


def test_create_driver_passes_base_driver_safety_limits():
    transport = FakeTransport(auto_ack=True)
    config = RVRNodeConfig(
        control_period=0.02,
        cmd_vel_timeout=0.25,
        max_linear_mps=0.3,
        max_angular_rad_s=0.6,
        max_raw_motor_duty=42,
        max_linear_raw_motor_duty=21,
        max_angular_raw_motor_duty=84,
        pivot_raw_motor_duty=12,
        velocity_control_mode=RVRDriver.VELOCITY_CONTROL_RAW_MOTOR,
        odom_wheel_track_m=0.222,
        safety_dispatch_timeout_s=0.18,
    )

    driver = create_driver(config, transport=transport)

    assert isinstance(driver, RVRDriver)
    assert driver._control_period == 0.02
    assert driver._command_timeout == 0.25
    assert driver._max_linear_mps == 0.3
    assert driver._max_angular_rad_s == 0.6
    assert driver._max_raw_motor_duty == 42
    assert driver._max_linear_raw_motor_duty == 21
    assert driver._max_angular_raw_motor_duty == 84
    assert driver._pivot_raw_motor_duty == 12
    assert driver._velocity_control_mode == RVRDriver.VELOCITY_CONTROL_RAW_MOTOR
    assert driver._wheel_track_m == 0.222
    assert driver._safety_dispatch_timeout_s == 0.18

def test_checked_in_rvr_yaml_preserves_floor_turn_motor_duty():
    config_text = Path(__file__).resolve().parents[1].joinpath("config", "rvr.yaml").read_text()

    assert "max_raw_motor_duty: 160" in config_text
    assert "max_linear_raw_motor_duty: 64" in config_text
    assert "max_angular_raw_motor_duty: 255" in config_text
    assert "velocity_control_mode: raw_motor" in config_text
    assert "safety_dispatch_timeout_s: 0.20" in config_text
    assert "motor_diagnostics_poll_period: 0.5" in config_text
    assert "odom_counts_per_meter: 4337.768" in config_text


def test_ros_node_exposes_clear_fail_safe_service():
    node_source = Path(__file__).resolve().parents[1].joinpath("src", "sphero_rvr_driver", "rvr_node.py").read_text()

    assert 'create_service(Trigger, "clear_fail_safe", self._on_clear_fail_safe)' in node_source
    assert "driver.clear_fail_safe_fault()" in node_source


def test_ros_node_enables_and_polls_motor_diagnostics():
    node_source = Path(__file__).resolve().parents[1].joinpath(
        "src",
        "sphero_rvr_driver",
        "rvr_node.py",
    ).read_text()

    assert "enable_motor_stall_notify()" in node_source
    assert "enable_motor_fault_notify()" in node_source
    assert "enable_motor_thermal_protection_status_notify()" in node_source
    assert "get_thermal_protection_status()" in node_source
    assert "get_battery_voltage()" in node_source
