from sphero_rvr_driver.rvr_node import RVRNodeConfig, create_driver
from sphero_rvr_core.driver import RVRDriver
from sphero_rvr_core.fake_transport import FakeTransport


def test_default_node_config_uses_pi_serial_alias_and_safe_motor_duty():
    config = RVRNodeConfig()

    assert config.serial_port == "/dev/ttyAMA0"
    assert config.baud_rate == 115200
    assert config.max_raw_motor_duty == 64
    assert config.battery_publish_period == 5.0
    assert config.diagnostics_publish_period == 1.0


def test_create_driver_passes_base_driver_safety_limits():
    transport = FakeTransport(auto_ack=True)
    config = RVRNodeConfig(
        control_period=0.02,
        cmd_vel_timeout=0.25,
        max_linear_mps=0.3,
        max_angular_rad_s=0.6,
        max_raw_motor_duty=42,
    )

    driver = create_driver(config, transport=transport)

    assert isinstance(driver, RVRDriver)
    assert driver._control_period == 0.02
    assert driver._command_timeout == 0.25
    assert driver._max_linear_mps == 0.3
    assert driver._max_angular_rad_s == 0.6
    assert driver._max_raw_motor_duty == 42
