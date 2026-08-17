"""The simulated chassis must be impossible to start by accident.

A fake robot that a config file can switch on is a fake robot that will eventually be
switched on during a real run -- and it would be *silent*, because everything above the
transport is production code that cannot tell it is talking to a model. The node would
publish confident odometry for a rover sitting perfectly still.

So the selector is not a boolean. It is an impossible device path, and these tests assert
nothing that flies ever names it. Same pattern as the camera charter and the stationary
test launch: the safe thing is the one that cannot be started by accident.
"""

from pathlib import Path

import pytest

from sphero_rvr_driver.rvr_node import SIMULATED_CHASSIS_PORT, RVRNodeConfig, create_driver

ROOT = Path(__file__).resolve().parents[1]


def test_the_selector_is_not_a_boolean_and_not_a_plausible_path():
    assert SIMULATED_CHASSIS_PORT == "SIMULATED_CHASSIS_NOT_A_REAL_ROBOT"
    assert not SIMULATED_CHASSIS_PORT.startswith("/"), (
        "a selector that looks like a device path invites being typed by accident"
    )
    assert SIMULATED_CHASSIS_PORT not in ("true", "True", "1", "yes")


def test_the_default_config_talks_to_a_real_device():
    assert RVRNodeConfig().serial_port == "/dev/ttyAMA0"
    assert RVRNodeConfig().serial_port != SIMULATED_CHASSIS_PORT


@pytest.mark.parametrize(
    "relative",
    [
        "launch/explore.launch.py",
        "launch/supervised_rvr.launch.py",
        "launch/rvr.launch.py",
        "launch/mapping.launch.py",
        "launch/bringup_stationary_test.launch.py",
        "config/rvr.yaml",
        "config/lean_rvr_tank_si.yaml",
        "config/lean_nav2.yaml",
        "config/lean_nav2_stock.yaml",
        "scripts/launch_and_arm.py",
    ],
)
def test_nothing_that_flies_names_the_simulated_chassis(relative):
    path = ROOT / relative
    if not path.exists():
        pytest.skip(f"{relative} not present on this branch")
    assert SIMULATED_CHASSIS_PORT not in path.read_text(), (
        f"{relative} selects the simulated chassis. Everything above the transport is "
        "production code and cannot tell it is talking to a model -- the node would "
        "publish confident odometry for a rover that is not moving."
    )


def test_selecting_the_simulator_actually_yields_the_simulator():
    # The guard above is only worth anything if the selector really works; otherwise this
    # file would be protecting a mechanism that does not exist (failure mode 1 of the
    # verification-adversary taxonomy).
    from sphero_rvr_core.chassis_sim import SimTransport

    driver = create_driver(RVRNodeConfig(serial_port=SIMULATED_CHASSIS_PORT))
    assert isinstance(driver._dispatcher._transport, SimTransport)


def test_a_normal_config_yields_a_serial_transport():
    from sphero_rvr_core.serial_transport import SerialTransport

    driver = create_driver(RVRNodeConfig(serial_port="/dev/ttyAMA0"))
    assert isinstance(driver._dispatcher._transport, SerialTransport)
