"""Per-clause proofs for the ToF holding the low-obstacle brake.

The empty-diffstat contract is suspended for the supervisor's config in this batch --
that is the design's own section 0 rule -- so these per-clause proofs are what replaces
it. Each reads the DEPLOYED yaml and drives the SAME functions the node calls, so a
clause proven here is proven about the thing that flies rather than about a copy of it.
"""
import math
import os
import re
import sys
from pathlib import Path

import dataclasses
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from sphero_rvr_core.tof_frame import (  # noqa: E402
    ObstacleDetector, TofConfig, ZONES, zone_point,
)

ROOT = Path(__file__).resolve().parents[1]
CFG = ROOT / "config" / "collision_stop.yaml"

_CITATION = re.compile(
    r"^\s*#\s*RULE B PINNED BY:\s*(?!<|\.\.\.|$)(\S[^\n]{7,})", re.M)
"""A citation must NAME something. The first version matched the bare token, so
the config's own sentence explaining the requirement satisfied it -- prose
defeating the guard it described. It must now carry at least eight characters of
actual content and must not be a `<placeholder>` or an ellipsis."""



def _cfg(name):
    m = re.search(rf"^\s*{name}:\s*(\S+)", CFG.read_text(), re.M)
    assert m, f"{name} missing from the deployed config"
    return m.group(1)


def _f(name):
    return float(_cfg(name))


def _tof_cloud_ground_ranges(frames, cfg, column_min=None):
    """Ground ranges the wired path would actually publish, over a run of frames.

    Takes a SEQUENCE because rule B carries an N-of-M confirmation window: feeding one
    frame at a time through a fresh detector can never confirm anything, and a test that
    did so would report "no obstacle" for a target the sensor sees perfectly. That is
    the mistake this helper's signature exists to prevent.
    """
    det = ObstacleDetector(cfg)
    out = []
    for frame in frames:
        for row, col in det.update(frame, column_min)["obstacles"]:
            p = zone_point(row, col, frame[row * ZONES + col], cfg)
            if p:
                g = math.hypot(p[0], p[1])
                # THE SUPERVISOR'S RANGE WINDOW IS PART OF THE WIRING. Leaving it out
                # made this helper report phantoms the brake could never have seen --
                # the detector publishes everything it concludes, and the consumer
                # decides what is in range. A "through the real wiring" proof that skips
                # the consumer's own filter is not one.
                if _f("low_obstacle_min_range_m") <= g <= _f("low_obstacle_max_range_m"):
                    out.append(g)
    return out


def test_the_brake_is_wired_to_the_ToF_and_enabled():
    assert _cfg("low_obstacle_topic") == "/tof/obstacles"
    assert _cfg("low_obstacle_brake_enable") == "true"


def test_the_brake_acts_on_a_REAL_recorded_obstacle():
    """END TO END on recorded frames: a real 5 cm object at 0.50 m, through the shipped
    detector, produces a cloud whose nearest point falls inside the deployed range
    window.

    Uses the tilt session's own frames rather than a synthetic obstacle, so the clause
    is about what this sensor really returns.

    THE MARGIN IS DECLARED, NOT INHERITED, and that is the whole lesson of this test's
    second version. It used to run at the shipped `disagreement_margin_m` of 0.10 m and
    pass, because every ToF ground range was 0.10 m short and every apparent
    disagreement was inflated by the frame error to 0.178 m. Corrected, the real
    disagreement on this object is 0.089 m and the shipped guess does not reach it --
    so the wiring proof, run at the shipped margin, would now assert that the brake
    cannot see the object it was built for. That is a true statement about an UNPINNED
    constant, and it belongs in its own test (below) rather than silently deciding
    whether the wiring works. What this proof is about is the PATH: detector to cloud
    to the consumer's range window.
    """
    sys.path.insert(0, os.path.dirname(__file__))
    from fixtures.tof_recorded_frames import TILT_BOX_050, TILT_BOX_050_WALL_M

    # The object is rule B's case (row 3 lies outside rule A's band), so the proof needs
    # rule B enabled AND the lidar background it adjudicates against -- the wall behind
    # the object, spot-measured at 0.678 m in the same session, in base_link.
    cfg = dataclasses.replace(
        TofConfig(), rule_b_enable=True, disagreement_margin_m=0.05)
    ranges = _tof_cloud_ground_ranges(TILT_BOX_050, cfg, [TILT_BOX_050_WALL_M] * 8)
    assert ranges, "the recorded object produced no obstacle points through the wired path"
    nearest = min(ranges)
    assert nearest <= _f("low_obstacle_max_range_m"), (
        f"the recorded object sits at {nearest:.3f} m, outside the deployed range window "
        "-- the brake could not see it even with rule B pinned")
    # THE WINDOW HAS TO ACTUALLY REACH IT, which is what the max_range re-derivation
    # bought. The object sits at ~0.58 m of base_link ground range; the old 0.55 m
    # window would have clipped it, and clipping is silent.
    assert nearest > 0.55, (
        f"the recorded object now lands at {nearest:.3f} m, inside the OLD 0.55 m "
        "window -- so this assertion no longer demonstrates that widening it mattered, "
        "and the max_range derivation needs redoing rather than trusting")


def test_the_pinned_margin_reaches_the_object_it_was_argued_from():
    """REWRITTEN 2026-08-14 AROUND THE MEASURED VALUE, which is what the previous
    version of this test told its reader to do the day bench item J pinned the margin.

    The history matters and is why this test exists at all. Design 9.4 justified rule B
    on this object with "a disagreement of 0.178 m, nearly twenty times the margin rule
    A had to spare". That 0.178 was 0.678 (a LIDAR range from base_link) minus 0.500 (a
    ToF SENSOR reading) -- subtracted across two frames. In one frame the object's real
    disagreement is 0.089 m, and the UNPINNED 0.10 m guess sat above it: at the shipped
    margin, rule B would not have fired on the object it was argued from.

    J pinned the margin at 0.06 m from a measured bare-wall distribution whose worst
    phantom-direction sample was 0.0396 m. So this object is now reachable with 48% of
    margin to spare, and that headroom is the point: the false-fire side is ours to
    measure, but how close an object sits to the wall behind it is the room's choice.
    A margin set as high as the detection data tolerates would keep missing this
    geometry.
    """
    sys.path.insert(0, os.path.dirname(__file__))
    from fixtures.tof_recorded_frames import TILT_BOX_050, TILT_BOX_050_WALL_M

    shipped = TofConfig()
    assert shipped.rule_b_enable, "rule B is gated; this test is about the pinned state"
    assert shipped.disagreement_margin_m == 0.06, (
        f"the pinned margin has moved to {shipped.disagreement_margin_m}. It was set "
        "FROM bench item J's measured distribution; moving it needs a new measurement "
        "and a new citation, not an edit")

    fired = _tof_cloud_ground_ranges(TILT_BOX_050, shipped, [TILT_BOX_050_WALL_M] * 8)
    assert fired, (
        "rule B does NOT reach the recorded object at the pinned 0.06 m margin. That "
        "object's real disagreement is 0.089 m, so either the margin moved above it "
        "or the geometry changed -- both are regressions of the thing J bought")

    # AND THE OLD GUESS STILL MISSES IT, so this test measures the margin rather than
    # the detector. Without this line it would pass against any margin at all.
    guess = dataclasses.replace(shipped, disagreement_margin_m=0.10)
    assert not _tof_cloud_ground_ranges(TILT_BOX_050, guess, [TILT_BOX_050_WALL_M] * 8), (
        "the old 0.10 m guess now reaches this object too, so the pinning made no "
        "observable difference here and this proof no longer distinguishes them")


def test_clear_floor_produces_NO_brake_input():
    """The paired negative, on recorded clear-floor frames. A brake that fires on floor
    is worse than no brake: it stops the mission instead of the robot."""
    sys.path.insert(0, os.path.dirname(__file__))
    from fixtures.tof_recorded_frames import TILT_CLEAR

    assert not _tof_cloud_ground_ranges(TILT_CLEAR, TofConfig()), (
        "clear floor produced brake input with the shipped rule-A-only configuration")

    # RULE B IS DELIBERATELY NOT EXERCISED HERE, and the reason is a correction.
    #
    # An earlier version fed rule B a UNIFORM synthetic background ([1.90] * 8) and
    # reported 188 "phantoms" on this segment. They were not phantoms. The frame is not
    # one surface: columns 0-4 read ~2.0 m (a wall) and columns 5-7 read ~1.05 m (a real
    # object to the rover's right -- the side clutter Scott flagged while recording).
    # At 1.05 m that object sits 0.03-0.14 m up, BELOW the 0.19 m lidar plane, so rule B
    # flagging it is very likely CORRECT.
    #
    # A uniform background is not a stand-in for a real one; `scan_min_by_column` exists
    # precisely because the background varies per column. Supplying a flat number asks
    # the detector a question no lidar would ever pose, and its answer means nothing.
    # Nothing in this recording can settle rule B's false-fire rate, because no
    # synchronised scan was ever captured beside it -- which is bench item J, and is a
    # better argument for J than the phantom claim it replaces.


def test_stale_tof_degrades_to_lidar_only():
    """REVERT-PROOF 1, through the REAL wiring. The staleness bound must be short enough
    that acting on the oldest tolerated frame cannot consume the approach margin.

    AMENDED with the 2026-08-19 speed raise (cert amendment, PM-reviewed with
    the batch): at the 0.35 cruise the UNSCALED form fails (0.105 > 0.098) and
    the margin holds only through the brake's own in-window speed scaling —
    forward_speed_scale at the window's outer edge caps the speed the stale
    frame can be acted on at. The scale credit is imported from the production
    formula, never restated, and the naive form is asserted to still fail so
    this pin can't silently become vacuous."""
    from sphero_rvr_core.low_obstacle_brake import forward_speed_scale
    max_age = _f("low_obstacle_max_age_s")
    cruise = _f("max_forward_mps")
    stop = _f("low_obstacle_stop_distance_m")
    reach = 0.298
    approach = reach - stop
    scale_at_edge = forward_speed_scale(
        reach, stop, _f("low_obstacle_slow_distance_m"),
        _f("low_obstacle_min_forward_scale"))
    travel = scale_at_edge * max_age * cruise
    assert travel < approach, (
        f"a cloud aged to the {max_age} s limit allows {travel:.3f} m of travel even "
        f"scale-credited, which exceeds the {approach:.3f} m between detection and the "
        "stop threshold -- the staleness bound is longer than the margin it protects")
    assert max_age * cruise > approach, (
        "the unscaled form fits again -- the scale credit is no longer load-bearing; "
        "simplify this pin and the yaml derivation together")
    assert max_age <= 0.4, (
        f"low_obstacle_max_age_s is {max_age}; the ToF runs at 6.5-7.6 Hz, so anything "
        "beyond ~0.4 s is several frames of silence treated as fresh")


def test_the_stop_threshold_sits_between_the_physics_and_the_detection_limit():
    """The derivation, pinned against the deployed numbers rather than remembered."""
    stop = _f("low_obstacle_stop_distance_m")
    need = _f("footprint_front_m") + _f("payload_margin_m") + _f("braking_distance_margin_m")
    assert stop > need, (
        f"stop threshold {stop} is inside the {need:.3f} m the rover physically needs")
    assert stop > 0.175, (
        f"stop threshold {stop} does not clear the config comment's more conservative "
        "0.175 m figure; prefer the conservative one while the floor margin is provisional")
    assert stop < 0.298, (
        f"stop threshold {stop} is beyond rule A's 0.298 m reach -- the brake would be "
        "asked to stop for things it cannot see")


def test_the_slow_band_covers_the_whole_detectable_range():
    """If detection begins inside the slow band, the rover is already easing off for the
    entire approach instead of braking flat from cruise."""
    assert _f("low_obstacle_slow_distance_m") >= 0.298, (
        "the slow band starts nearer than rule A's reach, so obstacles appear already "
        "inside it and the first the rover knows is a hard stop")
    assert _f("low_obstacle_slow_distance_m") > _f("low_obstacle_stop_distance_m")


def test_rule_b_is_live_ONLY_WITH_a_pinning_citation():
    """The gate's whole point was that WIRING the sensor does not silently enable every
    rule it has. Bench item J unlocked it on 2026-08-14, so the assertion inverts --
    but the CONDITION does not: enabled still requires the citation, and this test now
    guards the pairing rather than the off-state.

    Written this way deliberately. The lazy edit when a gate opens is to delete the
    test that asserted it was shut; what that throws away is the rule that opening it
    needs evidence. A future config that re-enables rule B after someone removes the
    citation fails here.
    """
    text = CFG.read_text()
    if TofConfig().rule_b_enable:
        assert _CITATION.search(text), (
            "rule B is ENABLED with no '# RULE B PINNED BY:' citation in the deployed "
            "config. The flag and the evidence for it travel together or not at all")
    else:
        # Still a legal state -- a re-gating after a bad field result is a config
        # change, not a regression -- but it must not be silent.
        assert "RULE B" in text, "rule B is gated and the config does not say so"


def test_the_range_window_matches_the_sensor_not_the_camera():
    """The camera's 0.40-1.20 m window would have discarded rule A's entire output --
    every point it produces lies below 0.40 m. Carrying those numbers over would have
    wired the brake and left it inert."""
    lo, hi = _f("low_obstacle_min_range_m"), _f("low_obstacle_max_range_m")
    assert lo < 0.297 < hi, (
        f"the deployed window {lo}-{hi} m does not contain rule A's 0.298 m reach")
    assert lo <= 0.1517, (
        f"min range {lo} is ABOVE the 0.1517 m nearest range this geometry can report "
        "in the true frame, so it silently discards the closest real returns -- the "
        "0.052 m figure it used to be checked against was the broken frame's")
    assert hi >= 0.598, (
        f"max range {hi} clips rule B's 0.598 m reach in the true base_link frame "
        "(0.498 was the same reach measured before the mount offset was applied)")
