"""The stock-middle prototype config, checked offline against this robot's own facts.

NOT A CLAIM THAT THE PROTOTYPE WORKS. It has never been flown and the chassis was off
when it was written. These tests assert only that the config does not contain the
defects we already paid for -- which is the most a config can be held to without a
robot, and is exactly what "an exhibit beside the decision" should survive.

Everything asserted here traces to a recorded incident, not to taste.
"""

import re
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
STOCK = ROOT / "config" / "lean_nav2_stock.yaml"
FLOWN = ROOT / "config" / "collision_stop.yaml"

#: Measured on gauntlet mission 1: 41 consecutive commanded pure rotations at exactly
#: this rate produced 0-1 mm of motion. Breakaway is therefore ABOVE it.
KNOWN_INEFFECTIVE_RAD_S = 0.4
#: Chosen by the decisive controller to be above breakaway, per its own documentation.
KNOWN_EFFECTIVE_RAD_S = 0.9


def cfg():
    """The config's DIRECTIVES ONLY, with comment lines stripped.

    THIS HELPER IS THE POINT OF A LESSON LEARNED THREE TIMES IN ONE NIGHT. A guard
    that greps raw text fails on the file's own explanation of the defect it forbids
    -- the costmap-window 253 guard, the camera launch guard, and three of the guards
    below all did exactly that on first writing. The pressure such a guard creates is
    to DELETE THE EXPLANATION to get green, which destroys the most valuable comment
    in the file. Assert on what the machine reads, not on what the human wrote.
    """
    return "\n".join(line for line in STOCK.read_text().splitlines()
                      if not line.lstrip().startswith("#"))


def raw():
    """Full text INCLUDING comments -- for assertions that are about the prose."""
    return STOCK.read_text()


def scalar(text, name):
    m = re.search(rf"^\s*{name}:\s*([-\d.]+)", text, re.M)
    assert m, f"{name} not found in the stock config"
    return float(m.group(1))


# --- D45: no angular constant may sit in the dead band ------------------------------

@pytest.mark.parametrize("name", [
    "rotate_to_heading_angular_vel",
    "min_rotational_vel",
])
def test_no_angular_constant_is_below_the_known_ineffective_rate(name):
    """THE DEFECT THAT KILLED MISSION 1. The supervisor clamped pivots to 0.4 rad/s,
    the motors could not execute it, the robot did not move, and the freeze classifier
    blamed an obstacle that did not exist -- then planted a mark that buried the robot.

    Nav2's own Spin defaults `min_rotational_vel` to 0.4, so this is not a trap unique
    to us; it is the trap the ecosystem parameterised for bases exactly like this one.
    """
    value = scalar(cfg(), name)
    assert value > KNOWN_INEFFECTIVE_RAD_S, (
        f"{name}={value} is at or below the rate measured to produce NO MOTION on this "
        f"drivetrain (0.400 rad/s, 41 consecutive commands, 0-1 mm)")
    assert value >= KNOWN_EFFECTIVE_RAD_S, (
        f"{name}={value} is below the only rate documented as above breakaway (0.9)")


def test_the_rotation_shim_is_not_used():
    """The shim's purpose is to stop and rotate in place to face the path, and it is
    what ground the motors. RPP's own `use_rotate_to_heading` gives the same behaviour
    with ONE authority over the rotation rate instead of two."""
    text = cfg()
    assert "RotationShimController" not in text
    assert "use_rotate_to_heading: true" in text


def test_the_angular_constants_are_measured_and_cite_the_measurement():
    """This guard was written when every angular constant was provisional, and it
    demanded four MEASURE-FIRST markers. The sweep has since run
    (03_validation/breakaway_2026-08-16), so the invariant it protects has INVERTED: the
    angular constants must now be derived, and must say where from. A guard that still
    demanded 'MEASURE-FIRST' would be enforcing a state we deliberately left."""
    text = raw()

    assert "MEASURE-FIRST" not in text, (
        "an angular constant is still marked MEASURE-FIRST, but the measurement exists"
    )
    assert "breakaway_2026-08-16" in text, "the config must cite the run it derives from"
    assert "pivot_curve" in text, "and the module that owns the curve"


def test_no_angular_constant_asks_for_a_rate_the_drivetrain_cannot_produce():
    """THE WHOLE POINT. 0.4 was stock's old rotation rate and Nav2's Spin default; 0.9
    was the 'above breakaway' replacement. Measured: the rate curve jumps from exactly
    zero to ~0.8-1.5 rad/s, so NEITHER is producible by any duty, and the slowest clean
    in-place rotation is 3.55 rad/s at the deployed pivot_min_duty of 28.

    A config that asks for an unproducible rate is not merely mis-tuned -- it is asking
    for something the driver will silently substitute, which is how three layers came to
    hold opinions none of them executed."""
    import re

    from sphero_rvr_core import pivot_curve as pc

    floor = pc.minimum_clean_rate(28)
    text = raw()

    rotational_keys = (
        "rotate_to_heading_angular_vel",
        "min_rotational_vel",
        "max_rotational_vel",
    )
    found = {}
    for key in rotational_keys:
        match = re.search(rf"^\s*{key}:\s*([0-9.]+)", text, re.M)
        assert match, f"{key} is missing from the stock config"
        found[key] = float(match.group(1))

    for key, value in found.items():
        assert value >= floor, (
            f"{key} = {value} is below the slowest clean pivot this drivetrain can make "
            f"({floor:.2f} rad/s). The driver would raise it silently; the config would "
            "be fiction."
        )
    assert found["max_rotational_vel"] <= pc.maximum_clean_rate(45) + 1e-6, (
        "max_rotational_vel exceeds what the deployed duty band can deliver"
    )


def test_constants_the_curve_does_not_cover_are_marked_UNMEASURED():
    """The curve measured IN-PLACE PIVOTS. Accelerations, linear breakaway and arc rates
    are different regimes, and deriving them from this curve would be the raw-motor
    error class again. They must be labelled, not quietly inherited."""
    text = raw()
    assert text.count("UNMEASURED") >= 3, (
        "acceleration limits and linear breakaway must be marked UNMEASURED rather than "
        "silently derived from a pivot measurement"
    )
    assert "run_card_arc_rate_FUTURE" in text, (
        "the arc gap must point at its close path, not just carry a label"
    )


# --- D42: marks must be points, and the lidar must not erase them -------------------

def test_touch_marks_live_in_a_layer_the_lidar_cannot_clear():
    """RAYTRACE CLEARING IS 2D PER LAYER. A lidar ray at the 0.19 m scan plane passes
    over a chair leg; in a shared layer it would clear that leg's cell -- erasing the
    one obstacle class this robot cannot see. Two instances, and the lidar has no
    authority in the second."""
    text = cfg()
    assert "scan_layer" in text and "touch_layer" in text
    touch = text[text.index("touch_layer:"):text.index("inflation_layer:")]
    assert "clearing: false" in touch, "touch marks must not be lidar-clearable"
    assert "/scan" not in touch, "the lidar must have no observation role in touch_layer"
    assert "/tof/points" in touch, (
        "the ToF must be the touch layer's clearing source -- it is the only sensor "
        "that can actually observe the sub-lidar band it clears")


def test_the_tof_clears_only_within_its_honest_envelope():
    """A sensor may only clear where it can see. The ToF's structural blind band ends
    at 0.167 m (blind_band_outer_range_m) and rule B reaches ~0.60 m; clearing outside
    that would be clearing on evidence it cannot supply -- the D39 lesson applied to
    the costmap instead of the brake."""
    text = cfg()
    touch = text[text.index("touch_layer:"):text.index("inflation_layer:")]
    assert re.search(r"raytrace_min_range:\s*0\.17", touch)
    assert re.search(r"raytrace_max_range:\s*0\.6", touch)


def test_no_denoise_layer():
    """DenoiseLayer removes small obstacle groups. A touch mark IS a small obstacle
    group."""
    assert "DenoiseLayer" not in cfg()


def test_inflation_is_the_only_place_the_robot_radius_is_applied():
    """D42's double-booking: we painted marks as 0.14 m robot-radius discs and then let
    the costmap inflate them AGAIN, sterilising ~0.56 m per touch. Marks are points
    now; the radius enters once."""
    text = cfg()
    assert re.search(r"inflation_radius:\s*0\.16", text)
    assert "robot_radius: 0.14" in text or "robot_radius: 0.145" in text


# --- D36 / D40: the recoveries must actually be able to run -------------------------

def test_a_local_costmap_exists_for_the_recoveries_to_check_against():
    """D36 measured stock recoveries refusing in 2 ms. They collision-check against the
    local costmap, and the flown stack has none -- explore.launch.py drops
    controller_server "and with it Nav2's local costmap". The recoveries were present,
    wired, and structurally unable to succeed."""
    text = cfg()
    assert "local_costmap:" in text
    behavior = text[text.index("behavior_server:"):]
    assert "local_costmap_topic" in behavior


def test_progress_checker_asks_about_translation_not_rotation():
    """PoseProgressChecker credits in-place rotation as progress. On mission 1 the rover
    rotated 1.8-2.2 degrees over 2-3 s while going nowhere; that reads as progress, and
    nothing trips for the whole time allowance. The honest question for a robot that is
    supposed to be going somewhere is whether it translated."""
    text = cfg()
    assert "SimpleProgressChecker" in text
    assert "PoseProgressChecker" not in text


def test_yaw_tolerance_is_not_tightened_below_one_control_cycle():
    """Bounded from BELOW, which surprises people. The tightest achievable yaw tolerance
    is about one control cycle of rotation (pivot_rate / controller_frequency).
    Tightening past it makes the goal unreachable: overshoot, correct, overshoot --
    which on this drivetrain is grinding."""
    text = cfg()
    yaw = scalar(text, "yaw_goal_tolerance")
    rate = scalar(text, "rotate_to_heading_angular_vel")
    freq = scalar(text, "controller_frequency")
    assert yaw >= rate / freq, (
        f"yaw_goal_tolerance {yaw} is tighter than one control cycle of rotation "
        f"({rate}/{freq} = {rate/freq:.3f} rad)")
    assert yaw >= 0.25


# --- the seam that will kill this prototype if it is forgotten ----------------------

def test_the_reverse_seam_is_documented_as_an_open_risk():
    """D40 WEARING STOCK CLOTHES, and the prototype's most likely failure.

    Nav2's BackUp recovery commands reverse. Our collision supervisor holds reverse
    whenever the rear sector is inside `reverse_stop_distance_m` (0.25) -- and mission 1
    sat at rear 0.243 m, seven millimetres inside it, refusing reverse 61 times in one
    minute. If the supervisor is not taught to permit BackUp, every stock reverse
    recovery dies exactly the way our bespoke ones did, and we will conclude stock is no
    better.

    This is asserted against the RECKONING DOC rather than the config, because the fix
    is in the supervisor and is NOT part of this prototype. It must not be discovered
    on carpet.
    """
    doc = (ROOT / "docs" / "navigation_reckoning.md").read_text()
    assert "BackUp" in doc and "reverse_stop_distance_m" in doc, (
        "the reverse-seam risk must be written down before this prototype is flown")
