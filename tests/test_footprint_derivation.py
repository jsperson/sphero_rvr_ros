"""The footprint, derived from Scott's tape rather than remembered.

Standards rule 9: the footprint is a claim about where the robot ENDS, every stop
distance is derived from it, and unlike the other safety constants it changes when
someone picks up a screwdriver. This file is what notices.

Every number here traces to one of three things: the tape (2026-08-15), the published
base_link -> laser transform, or the deployed YAML. Nothing is transcribed from a
README, and the arithmetic that moves the tape into base_link is written out rather
than pre-applied, because that transform is where the first error lived.
"""

import math
import re
from pathlib import Path

import pytest

from sphero_rvr_driver.collision_stop import CollisionStopConfig

ROOT = Path(__file__).resolve().parents[1]
CFG = ROOT / "config" / "collision_stop.yaml"
LIDAR_LAUNCH = ROOT / "launch" / "lidar.launch.py"

# --- Scott's tape, 2026-08-15, verbatim ---------------------------------------------
# "front: 0.092m, right 0.095m, back: 0.104 (this involved full compression of the
#  power cable), left: 0.109m"
# Measured from directly below the LIDAR SPIN AXIS, and front is to the RANGEFINDER'S
# LEADING EDGE (lens/housing/bracket, whichever reaches furthest) per standards rule 9.
TAPE_FRONT, TAPE_RIGHT, TAPE_REAR, TAPE_LEFT = 0.092, 0.095, 0.104, 0.109

#: The rear tape is a MINIMUM -- taking it required compressing the power cable, which
#: at rest extends further. Stated rather than measured because every flip-check below
#: is insensitive to it across 0-20 mm, so a second measurement would change nothing.
CABLE_ALLOWANCE_M = 0.015


def _launch_default(name):
    """The published base_link -> laser translation, read from the launch file that
    publishes it. Read rather than restated: the tape's origin IS this transform, and a
    copy of it here could drift from the one the TF tree actually broadcasts."""
    m = re.search(rf'"{name}",\s*\n\s*default_value="([-0-9.]+)"', LIDAR_LAUNCH.read_text())
    assert m, f"{name} is not declared in lidar.launch.py"
    return float(m.group(1))


def _yaml_float(name):
    m = re.search(rf"^\s*{name}:\s*([0-9.]+)", CFG.read_text(), re.M)
    assert m, f"{name} is not in the deployed config"
    return float(m.group(1))


def tape_in_base_link():
    """(front, rear, left, right) extents referenced to base_link.

    THE Y OFFSET IS NOT COSMETIC AND IT REVERSES A SIGN. The laser sits 11 mm to the
    RIGHT of base_link (+y is left), so the tape's left reading loses 11 mm and its
    right reading gains 11 mm. Raw tape says left is bigger by 14 mm; base_link says
    RIGHT is bigger by 8 mm. Anything that consumed the raw tape as base_link extents
    would have padded the wrong side of the robot.
    """
    lx, ly = _launch_default("laser_x"), _launch_default("laser_y")
    return (TAPE_FRONT + lx, TAPE_REAR - lx, abs(ly + TAPE_LEFT), abs(ly - TAPE_RIGHT))


def pivot_radius(front, rear, left, right, payload):
    """The circumscribed corner radius a pivot sweeps -- the supervisor's own formula
    (`collision_stop.py`, the pure-pivot branch), not a second author of it."""
    return math.hypot(max(front, rear), max(left, right)) + payload


# --- the transform ------------------------------------------------------------------

def test_the_tape_origin_is_the_lidar_spin_axis_not_base_link():
    assert _launch_default("laser_x") == pytest.approx(0.0045)
    assert _launch_default("laser_y") == pytest.approx(-0.011)


def test_extents_referenced_to_base_link():
    front, rear, left, right = tape_in_base_link()
    assert front == pytest.approx(0.0965)
    assert rear == pytest.approx(0.0995)     # cable compressed: a MINIMUM
    assert left == pytest.approx(0.098)
    assert right == pytest.approx(0.106)


def test_the_y_offset_reverses_the_left_right_asymmetry():
    """Pinned as a PHYSICAL claim, in the style bearings.py was given after the mirrored
    clock error. If this ever stops holding, someone has consumed the raw tape."""
    _, _, left, right = tape_in_base_link()
    assert TAPE_LEFT > TAPE_RIGHT          # the tape reads left-bigger
    assert right > left                    # base_link reads right-bigger
    assert (right - left) == pytest.approx(0.008, abs=1e-9)


# --- what the deployed config must equal --------------------------------------------

def test_the_deployed_footprint_is_the_measured_one():
    front, rear, left, right = tape_in_base_link()
    assert _yaml_float("footprint_front_m") == pytest.approx(front)
    assert _yaml_float("footprint_rear_m") == pytest.approx(rear + CABLE_ALLOWANCE_M)
    assert _yaml_float("footprint_left_m") == pytest.approx(left)
    assert _yaml_float("footprint_right_m") == pytest.approx(right)


def test_no_dataclass_default_disagrees_with_the_deployed_config():
    """THREE SETS OF FOOTPRINT DEFAULTS EXISTED SIMULTANEOUSLY until 2026-08-15: the
    YAML (0.11/0.16/0.10/0.10), `CollisionStopConfig` (0.22/0.16/0.14/0.14, payload
    0.05) and `SurveyConfig` (0.11/0.16/0.10/0.10). A bench probe that forgot to load
    the YAML answered from a robot twice the real size -- the deployed-config trap with
    a second and third author. The YAML governs; the defaults may not silently differ.
    (SurveyConfig, the third author, died with the bespoke escape family
    2026-08-21 -- one fewer place for this trap to live.)"""
    for name in ("footprint_front_m", "footprint_rear_m",
                 "footprint_left_m", "footprint_right_m"):
        deployed = _yaml_float(name)
        assert getattr(CollisionStopConfig(), name) == pytest.approx(deployed), name
    assert CollisionStopConfig().payload_margin_m == pytest.approx(
        _yaml_float("payload_margin_m"))


def test_the_front_extent_reaches_the_rangefinder_not_the_chassis():
    """Standards rule 9's actual requirement. `TofConfig.mount_x_m` (0.10) is the
    sensor's OPTICAL ORIGIN and the housing reaches past it; the measured nose is
    0.0965. That the two are within 4 mm of each other is the whole hazard -- a
    footprint that ends behind its own foremost sensor is a robot that will touch what
    it cannot see, which is exactly what happened on 2026-08-15."""
    from sphero_rvr_core.tof_frame import TofConfig
    front, _, _, _ = tape_in_base_link()
    assert abs(front - TofConfig().mount_x_m) < 0.005


def test_the_fitted_mount_constants_are_not_corrected_by_the_tape():
    """CALIBRATION IS NOT MATTER, and this is here to stop a well-meaning fix.

    `mount_x_m = 0.10` sits AHEAD of the measured nose 0.0965 and is not wrong: fitted
    constants reproduce READINGS in the sensor's own frame, the tape describes MATTER.
    Same class as the fitted `mount_height_m = 0.139` against Scott's ~0.11 tape. The
    tape governs extents and stop margins; the fitted constants govern point placement.
    'Fixing' the frame fit with this tape would misplace every point the ToF publishes.
    """
    from sphero_rvr_core.tof_frame import TofConfig
    assert TofConfig().mount_x_m == 0.10
    assert TofConfig().mount_height_m == 0.139


# --- flip-check E: does mission 2's pivot verdict survive? ---------------------------

#: The object that vetoed mission 2's pivot, from the 08-14 autopsy's by-bearing table.
MISSION2_OBJECT_M = 0.165


def test_mission2_pivot_verdict_survives_the_new_extents():
    """DECLARED, NOT DISCOVERED. The verdict HOLDS -- and it holds ON THE MARGIN, not
    on geometry, which is the sentence that must survive editing.

        declared before  hypot(0.16,   0.10 ) + 0.02 = 0.2087   refuse by 43.7 mm
        measured, bare   hypot(0.0995, 0.106) + 0.00 = 0.1454   GRANT  (!)
        measured + pad   hypot(0.1145, 0.106) + 0.02 = 0.1760   refuse by 11.0 mm

    Strip `payload_margin_m` and the corner radius falls to 0.1454 m against a 0.165 m
    object and the pivot becomes grantable. The refusal that cost mission 2 is carried
    entirely by a 20 mm pad.
    """
    front, rear, left, right = tape_in_base_link()
    payload = _yaml_float("payload_margin_m")

    bare = pivot_radius(front, rear + CABLE_ALLOWANCE_M, left, right, 0.0)
    assert bare < MISSION2_OBJECT_M, "bare geometry would GRANT the pivot"

    full = pivot_radius(front, rear + CABLE_ALLOWANCE_M, left, right, payload)
    assert full > MISSION2_OBJECT_M
    assert (full - MISSION2_OBJECT_M) == pytest.approx(0.011, abs=5e-4)


@pytest.mark.parametrize("allowance", (0.0, 0.005, 0.010, 0.015, 0.020))
def test_the_verdict_is_insensitive_to_the_cable_allowance(allowance):
    """Why no second measurement was requested. Across the whole plausible range of the
    uncompressed cable the verdict never flips -- at zero allowance it refuses by only
    0.4 mm, so the allowance buys margin but does not decide anything."""
    front, rear, left, right = tape_in_base_link()
    radius = pivot_radius(front, rear + allowance, left, right,
                          _yaml_float("payload_margin_m"))
    assert radius > MISSION2_OBJECT_M


# --- the stop distances, re-derived under the new extents ---------------------------

def test_the_front_physics_term_is_no_longer_binding():
    """RULE 2: a new envelope obliges re-checking every constant derived under the old
    one. The effective hard stop is max(stop_distance_m, front + payload + braking).
    The physics term is now 0.1365 m against a deployed stop_distance_m of 0.30, so the
    threshold governs by 0.164 m -- and the config comment that claimed the opposite
    (and quoted a 0.12 stop and a 0.045 braking margin that no longer exist) is why
    nobody had noticed."""
    front, _, _, _ = tape_in_base_link()
    physics = front + _yaml_float("payload_margin_m") + _yaml_float("braking_distance_margin_m")
    assert physics == pytest.approx(0.1365)
    assert _yaml_float("stop_distance_m") > physics


def test_the_reverse_gate_is_governed_by_its_threshold_not_the_footprint():
    """Declared, deliberately unchanged, and flagged. D40's finding was that reverse is
    refused at poses physics allows; an over-large reverse threshold is one route to
    that, and this test states the size of it rather than leaving it implicit. Moving a
    reverse gate is a behavioural change needing its own evidence -- the footprint
    re-derivation exposed this, it did not derive it."""
    _, rear, _, _ = tape_in_base_link()
    physics = (rear + CABLE_ALLOWANCE_M + _yaml_float("payload_margin_m")
               + _yaml_float("braking_distance_margin_m"))
    assert physics == pytest.approx(0.1545)
    assert _yaml_float("reverse_stop_distance_m") - physics == pytest.approx(0.0955, abs=5e-4)


def test_the_contact_reconstruction_agrees_with_the_tape():
    """The cleanest cross-validation this project has produced: run 1's leg sat at
    0.091 m from base_link at contact (bag reconstruction: last trustworthy reading
    0.181 m, minus 90 mm of travel after the brake released) against a measured nose of
    0.0965 m. Two entirely independent instruments, 5.5 mm apart.

    NOT 1 mm. That figure compares the bag's base_link number against the tape's
    spin-axis number -- two origins 4.5 mm apart -- and is flattered by exactly the
    error being corrected here.
    """
    front, _, _, _ = tape_in_base_link()
    leg_at_contact_m = 0.181 - 0.090
    assert abs(front - leg_at_contact_m) == pytest.approx(0.0055, abs=5e-4)
