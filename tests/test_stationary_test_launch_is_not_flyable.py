"""The stationary test's static TF must never appear in a launch that can fly.

A static odom->base_link is TRUE while the rover is parked with the chassis off, and a
lie the instant anything drives -- a lie that Nav2, the costmaps and any progress checker
would all consume as fact. So the safety property is not "remember not to use it": it is
that no flight launch can reference it at all.

Same pattern as the camera charter: the safe thing is the one that cannot be started by
accident.
"""

from pathlib import Path

import pytest

LAUNCH_DIR = Path(__file__).resolve().parents[1] / "launch"

#: Every launch here can put the robot on the floor under its own power.
FLIGHT_LAUNCHES = [
    "explore.launch.py",
    "supervised_rvr.launch.py",
    "rvr.launch.py",
    "mapping.launch.py",
]

STATIONARY_LAUNCH = "bringup_stationary_test.launch.py"
MARKER = "STATIONARY_TEST_STATIC_ODOM"


def _read(name: str) -> str:
    path = LAUNCH_DIR / name
    if not path.exists():
        pytest.skip(f"{name} is not present in this tree")
    return path.read_text()


def test_the_stationary_launch_declares_itself_with_the_marker():
    source = _read(STATIONARY_LAUNCH)
    assert MARKER in source, (
        "the stationary launch must carry the marker every guard greps for; without it "
        "this whole file silently guards nothing"
    )


@pytest.mark.parametrize("launch", FLIGHT_LAUNCHES)
def test_no_flight_launch_references_the_stationary_static_tf(launch):
    source = _read(launch)
    assert MARKER not in source, (
        f"{launch} references the stationary test's static odom->base_link. That "
        "transform is true only while the rover is parked with its chassis off; in a "
        "launch that can drive it is a fabricated pose consumed as fact by the costmaps, "
        "the planner and the progress checker."
    )


@pytest.mark.parametrize("launch", FLIGHT_LAUNCHES)
def test_no_flight_launch_publishes_a_static_odom_to_base_link_by_any_route(launch):
    # Stronger than the marker check: the marker could be removed while the same node is
    # added by hand. Look for the shape of the thing, not only its label.
    source = _read(launch).replace('"', " ").replace("'", " ").replace(",", " ")
    if "static_transform_publisher" not in source:
        return
    tokens = source.split()
    for index, token in enumerate(tokens):
        if token == "odom" and index + 1 < len(tokens) and tokens[index + 1] == "base_link":
            raise AssertionError(
                f"{launch} publishes a static odom->base_link. Odometry is a MEASUREMENT; "
                "a launch that can drive must get it from the driver, not from a constant."
            )


def test_the_stationary_launch_does_not_start_the_driver_or_the_explorer():
    # It must not be able to command the wheels even if a chassis were powered on.
    source = _read(STATIONARY_LAUNCH)
    for forbidden in ("rvr_node", "coverage_explorer", "explore_lite", "decisive_controller"):
        assert forbidden not in source, (
            f"the stationary test launch starts {forbidden}, which can command motion. "
            "This launch exists to prove wiring, not to drive."
        )
