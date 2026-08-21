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

#: A file "names the simulator" if it contains the literal value OR imports the symbol.
#: Checking only the value was a blind spot: the sim launch imports the constant (which is
#: the GOOD pattern -- no duplicated literal), so a value-only search saw nothing, and a
#: flight launch could have done the same and slipped past the guard.
SELECTOR_TOKENS = (SIMULATED_CHASSIS_PORT, "SIMULATED_CHASSIS_PORT")


def _names_the_simulator(text: str) -> bool:
    return any(token in text for token in SELECTOR_TOKENS)


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
        "config/lean_nav2_stock.yaml",
        "scripts/launch_and_arm.py",
    ],
)
def test_nothing_that_flies_names_the_simulated_chassis(relative):
    path = ROOT / relative
    if not path.exists():
        pytest.skip(f"{relative} not present on this branch")
    assert not _names_the_simulator(path.read_text()), (
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


def test_exactly_one_launch_names_the_simulator_and_it_is_the_sim_rig():
    """Otherwise this file guards a list instead of a property.

    The parametrized test above checks known flight launches by name; that cannot see a
    NEW launch someone adds. This one asserts the global shape: exactly one launch file
    in the tree selects the simulator, and it is the one whose docstring says it is not a
    flight launch.
    """
    naming = [
        path.name
        for path in sorted((ROOT / "launch").glob("*.py"))
        if _names_the_simulator(path.read_text())
    ]
    assert naming == ["sim_closed_loop.launch.py"], (
        f"launches naming the simulated chassis: {naming}. Exactly one may, and it must "
        "be the closed-loop rig."
    )
    source = (ROOT / "launch" / "sim_closed_loop.launch.py").read_text()
    assert "NOT A FLIGHT LAUNCH" in source


# --- the fake scan is the most dangerous thing in this repo -------------------------

def test_the_clear_scan_node_refuses_without_explicit_consent():
    """A fake all-clear /scan would blind the collision supervisor into granting every
    command. On a real robot that is worse than a bad controller: it removes the reflex
    layer that has never failed. So it refuses to start unless asked explicitly."""
    # Read at source level: importing the module needs rclpy, which is absent on the
    # Mac, and this guard must run everywhere the suite runs.
    source = (ROOT / "src" / "sphero_rvr_driver" / "sim_clear_scan.py").read_text()

    assert "SimClearScanRefused" in source
    assert "declare_parameter(CONSENT_PARAMETER, False)" in source, (
        "the consent parameter must DEFAULT to false"
    )
    assert 'CONSENT_PARAMETER = "i_understand' in source, (
        "the parameter name should make an operator read what they are agreeing to"
    )
    assert "raise SimClearScanRefused" in source, "it must actually refuse"


@pytest.mark.parametrize(
    "relative",
    [
        "launch/explore.launch.py",
        "launch/supervised_rvr.launch.py",
        "launch/rvr.launch.py",
        "launch/mapping.launch.py",
        "launch/bringup_stationary_test.launch.py",
        "scripts/launch_and_arm.py",
    ],
)
def test_no_flight_launch_publishes_a_fake_scan(relative):
    path = ROOT / relative
    if not path.exists():
        pytest.skip(f"{relative} not present")
    text = path.read_text()
    assert "sim_clear_scan" not in text and "sim_raycast_scan" not in text, (
        f"{relative} starts the fake clear-scan publisher. That blinds the supervisor "
        "into granting every command -- the reflex layer is the part of this stack that "
        "has never failed, and this would switch it off."
    )


def test_the_raycast_scan_node_also_refuses_without_consent():
    source = (ROOT / "src" / "sphero_rvr_driver" / "sim_raycast_scan.py").read_text()
    assert "SimScanRefused" in source and "raise SimScanRefused" in source
    assert "declare_parameter(CONSENT_PARAMETER, False)" in source


def test_the_sim_rig_uses_the_REAL_laser_mount_not_identity():
    """A simulator that mounts the lidar straight exercises a robot we do not own.

    The real base_link->laser yaw is 3.1239668 rad (~179 deg) -- raw scan angle 0 points
    BEHIND the rover. Getting that wrong would produce a plausible scan of a mirrored
    world, and every collision decision downstream would be confidently wrong.
    """
    source = (ROOT / "launch" / "sim_closed_loop.launch.py").read_text()
    assert "3.1239668018215028" in source, "the sim must use the measured mount yaw"
    assert '"--x", "0.004500"' in source
