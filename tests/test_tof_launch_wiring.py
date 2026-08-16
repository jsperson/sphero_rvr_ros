"""The ToF brake's producer must be in the launch, and pointed at the right topic.

Two defects meet here.

ONE: `tof_node` was in no launch file. The supervisor ships with
`low_obstacle_brake_enable: true` reading `/tof/obstacles`, and a brake with no
producer FAILS OPEN -- `_apply_low_obstacle_brake` returns the command untouched when
no cloud has ever arrived, which is right for a sensor that died and
indistinguishable from one never started. So every ToF run was one forgotten command
away from believing it had a sub-lidar brake it did not have.

TWO: the launch file's own help text said the CAMERA node fed the brake. It has not
since the ToF took over, so enabling it would publish to a topic nothing reads. That
is standards Appendix A2 -- a module states the SHAPE it consumes, the deployed
config states the SOURCE -- and prose is the one claim nothing gates on, which is why
this file exists.

Asserted structurally against the sources, because launching anything needs ROS and
this machine has none.
"""

import re
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
LAUNCH = ROOT / "launch" / "explore.launch.py"
CONFIG = ROOT / "config" / "collision_stop.yaml"
SETUP = ROOT / "setup.py"


@pytest.fixture(scope="module")
def launch_src():
    return LAUNCH.read_text()


def _decl_block(src, name):
    """The `DeclareLaunchArgument` block for `name`.

    Anchored on `DeclareLaunchArgument(` rather than on the bare name, because every
    argument's name also appears earlier in a `LaunchConfiguration(...)` line -- and
    slicing from that first hit reads a block belonging to nothing, which is how a
    test quietly stops examining what it claims to. (It did, on the first run.)
    """
    match = re.search(
        r"DeclareLaunchArgument\(\s*\n\s*\"" + re.escape(name) + r"\"(.*?)\n            \),",
        src, re.S)
    assert match, f"no DeclareLaunchArgument block for {name!r}"
    return match.group(1)


def _deployed_params():
    yaml = pytest.importorskip("yaml")
    raw = yaml.safe_load(CONFIG.read_text())

    def walk(node):
        if isinstance(node, dict):
            if "ros__parameters" in node:
                return node["ros__parameters"]
            for value in node.values():
                found = walk(value)
                if found:
                    return found
        return None

    params = walk(raw)
    assert params
    return params


def test_the_brakes_producer_is_actually_launched(launch_src):
    """REVERT-PROOF. Fails against a launch file with no tof node."""
    assert re.search(r'executable="tof"', launch_src), (
        "no tof node in explore.launch.py; the sub-lidar brake has no producer and "
        "will fail open without saying so"
    )
    assert re.search(r"^\s+tof,\s*$", launch_src, re.M), (
        "the tof node is declared but never added to the launch description, which "
        "is the unreachable-code form of the same defect"
    )


def test_it_defaults_ON_because_the_brake_it_feeds_defaults_ON(launch_src):
    """The two defaults must agree, or the stack ships with a brake wired to nothing.

    This binds the launch default to the DEPLOYED supervisor config rather than to a
    preference, so if the brake is ever disabled by default the mismatch surfaces
    here instead of as a quiet capability loss in a room.
    """
    params = _deployed_params()
    brake_enabled = bool(params.get("low_obstacle_brake_enable"))
    block = _decl_block(launch_src, "start_tof")
    default_on = 'default_value="true"' in block

    assert default_on == brake_enabled, (
        f"low_obstacle_brake_enable={brake_enabled} but start_tof default_on="
        f"{default_on}. A brake enabled with no producer fails OPEN silently."
    )


def test_the_brake_reads_the_topic_the_launched_node_publishes():
    """PREMISE TRIPWIRE. Survives its own mutation, on purpose.

    Asserts the DEPLOYED `low_obstacle_topic` is the ToF's, not the camera's. If it
    ever moves back to `/camera/low_obstacles`, starting `tof` by default stops
    helping and the help text this commit corrected becomes true again -- and both
    facts need re-deciding together rather than one drifting.
    """
    topic = _deployed_params().get("low_obstacle_topic")
    assert topic == "/tof/obstacles", (
        f"the brake reads {topic!r}; this launch wiring and its help text both assume "
        f"/tof/obstacles, and they must move together"
    )


def test_the_camera_cannot_be_started_as_a_navigation_sensor(launch_src):
    """SCOTT'S CHARTER, 2026-08-16, pinned: *"We want to use the camera to obtain
    intelligence about the environment (object locations, faces, etc) but it should not
    be involved in the safety stack or direct navigation."*

    This test used to guard the *help text* of a `start_low_obstacle` argument, because
    the prose falsely claimed the camera fed the collision brake. The argument is now
    gone entirely, which is the stronger guarantee: the monocular detector cannot be
    started from this launch at all, and its console entry point is removed, so there is
    no path to run it. A test about wording is replaced by a test about capability.

    What it cost while it ran (gauntlet mission 1): ~34% of a CPU on a Pi that hit load
    10.7 and starved the ToF to 5.4 Hz -- under the rate its own staleness bound is
    derived from -- to publish a cloud the brake had stopped reading.
    """
    assert "start_low_obstacle" not in launch_src, (
        "the monocular camera detector is startable again; Scott's charter is that the "
        "camera is an intelligence sensor, never a navigation or safety one")
    assert 'executable="low_obstacle"' not in launch_src, (
        "the monocular low_obstacle node is back in the launch")

    setup = (Path(__file__).resolve().parents[1] / "setup.py").read_text()
    assert "low_obstacle_node:main" not in setup, (
        "the low_obstacle console entry point is back; without the launch arg this is "
        "the only other way to start it, so both must stay gone")


def test_the_camera_is_not_in_default_bringup(launch_src):
    """The camera driver is started on demand by Track 2's observe path, never by the
    motion stack. `camera.launch.py` stays a separate, explicit launch.

    Checked on the ACTUAL launch arguments, not by grepping for the string
    "camera.launch" -- that phrase appears in this file only inside a comment
    explaining that the camera is deliberately *not* started here, so a string test
    would fail on its own explanation and pressure someone into deleting the
    explanation. (Second time tonight; see tests/test_costmap_window_scale.py.)
    """
    assert '"start_camera": "false"' in launch_src, (
        "the included mapping launch is no longer explicitly told not to start the "
        "camera; the charter requires it stay out of default bringup")


def test_the_launched_executable_exists_in_the_entry_points():
    """A launch file naming an executable setup.py does not install fails at runtime
    on the robot, which is the worst place to discover a typo."""
    entry_points = SETUP.read_text()
    assert "tof = sphero_rvr_driver.tof_node:main" in entry_points


def test_the_node_defaults_still_match_the_flying_configuration():
    """PREMISE TRIPWIRE, bound to RECORDED EVIDENCE rather than to a config file.

    No parameters are passed to the tof node, which is only correct while its declared
    defaults ARE what flies. That was verified against the robot's OWN WORDS in the
    2026-08-15b bag, where `/tof/state` reported:

        stop_distance_m=0.45 rules=rule_a+b rule_b=pinned margin_m=0.06

    If any of these drift, the launch silently starts a differently-configured sensor
    than the one every recording was made with -- and the night rule B was silently
    OFF with every test green is exactly what that looks like from outside.
    """
    from sphero_rvr_core.tof_frame import TofConfig

    cfg = TofConfig()
    assert cfg.stop_distance_m == pytest.approx(0.45)
    assert cfg.disagreement_margin_m == pytest.approx(0.06)
    assert cfg.rule_b_enable is True
