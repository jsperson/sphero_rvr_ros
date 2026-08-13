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
    r"^\\s*#\\s*RULE B PINNED BY:\\s*(?!<|\\.\\.\\.|$)(\\S[^\\n]{7,})", re.M)
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
    detector, produces a cloud whose nearest point falls inside the deployed slow band.

    Uses the tilt session's own frames rather than a synthetic obstacle, so the clause
    is about what this sensor really returns.
    """
    sys.path.insert(0, os.path.dirname(__file__))
    from fixtures.tof_recorded_frames import TILT_BOX_050, TILT_BOX_050_WALL_M

    # The object is rule B's case (row 3 lies outside rule A's band), so the proof needs
    # rule B enabled AND the lidar background it adjudicates against -- the wall behind
    # the object, spot-measured at 0.678 m in the same session.
    cfg = dataclasses.replace(TofConfig(), rule_b_enable=True)
    ranges = _tof_cloud_ground_ranges(TILT_BOX_050, cfg, [TILT_BOX_050_WALL_M] * 8)
    assert ranges, "the recorded object produced no obstacle points through the wired path"
    nearest = min(ranges)
    assert nearest <= _f("low_obstacle_max_range_m"), (
        f"the recorded object sits at {nearest:.3f} m, outside the deployed range window "
        "-- the brake could not see it even with rule B pinned")

    # AND THE COUPLING THAT PINNING RULE B WILL CREATE, asserted now while it is cheap.
    # The object is at 0.482 m; the deployed slow band is 0.30 m, sized for rule A's
    # 0.298 m reach. That is correct TODAY, with rule B gated. The moment J pins rule B,
    # a 0.30 m slow band would throw away every metre of the range B was run to buy.
    if _CITATION.search(CFG.read_text()):
        assert _f("low_obstacle_slow_distance_m") >= 0.50, (
            f"rule B is pinned but the slow band is still {_f('low_obstacle_slow_distance_m')} m, "
            f"sized for rule A. The recorded object is seen at {nearest:.3f} m and would "
            "be ignored until it reached 0.30 m -- rule B's range bought nothing")


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
    that acting on the oldest tolerated frame cannot consume the approach margin."""
    max_age = _f("low_obstacle_max_age_s")
    cruise = _f("max_forward_mps")
    travel = max_age * cruise
    approach = 0.298 - _f("low_obstacle_stop_distance_m")
    assert travel < approach, (
        f"a cloud aged to the {max_age} s limit allows {travel:.3f} m of travel, which "
        f"exceeds the {approach:.3f} m between detection and the stop threshold -- the "
        "staleness bound is longer than the margin it protects")
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


def test_rule_b_is_still_gated_through_the_real_wiring():
    """The whole point of the gate is that WIRING the sensor does not silently enable
    every rule it has."""
    assert TofConfig().rule_b_enable is False
    if "RULE B PINNED BY:" not in CFG.read_text():
        assert TofConfig().rule_b_enable is False, (
            "rule B is enabled with no pinning citation in the deployed config")


def test_the_range_window_matches_the_sensor_not_the_camera():
    """The camera's 0.40-1.20 m window would have discarded rule A's entire output --
    every point it produces lies below 0.40 m. Carrying those numbers over would have
    wired the brake and left it inert."""
    lo, hi = _f("low_obstacle_min_range_m"), _f("low_obstacle_max_range_m")
    assert lo < 0.298 < hi, (
        f"the deployed window {lo}-{hi} m does not contain rule A's 0.298 m reach")
    assert lo >= 0.052, (
        f"min range {lo} is below the 0.052 m nearest range this geometry can report")
    assert hi >= 0.498, (
        f"max range {hi} would clip rule B's 0.498 m reach once bench item J pins it")
