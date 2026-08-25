"""The arithmetic `config/collision_stop.yaml` states in prose, restated as tests.

WHY THIS FILE EXISTS. That config is ~360 lines and most of them are essay. Three
separate times it has asserted arithmetic from a configuration that no longer
existed -- it says so itself at L94-95 ("every number in it was from a config that
no longer exists"). Prose rots silently; a test fails loudly. Standards rule 2 is
the general form: a new envelope obliges re-checking every constant derived under
the old one, and nothing in a comment block re-checks itself.

WHAT IS AND IS NOT HERE.
  * Every operand is read from the DEPLOYED yaml via `yaml.safe_load`, never from a
    `CollisionStopConfig` default. `test_escape_geometry` was the right shape
    pointed at the wrong era's config, and the footprint trap (three sets of
    defaults live at once) is the same failure with three authors.
  * Geometry operands come from the shipped `TofConfig` and are DERIVED here, not
    transcribed. Where an existing test pins the same quantity against a copied
    literal, this file derives it instead and says so on the test.
  * Ordering constraints already enforced in `CollisionStopConfig.__post_init__`
    (slow > stop, release > stop, stamp age >= scan age) are NOT duplicated. A
    production invariant outranks a test of the same invariant; what is pinned here
    is the SIZE of each margin, which `__post_init__` says nothing about.
  * No deployed value is changed by this file, and none should be changed to make it
    pass. Every constant below is on the safety path.
"""

import math
from pathlib import Path

import pytest
import yaml

from sphero_rvr_core.tof_frame import (
    TofConfig, blind_band_outer_range_m, zone_point,
)

ROOT = Path(__file__).resolve().parents[1]
CFG = ROOT / "config" / "collision_stop.yaml"
RVR_NODE = ROOT / "src" / "sphero_rvr_driver" / "rvr_node.py"

SUP = yaml.safe_load(CFG.read_text())["lidar_collision_stop_supervisor"]["ros__parameters"]


def _f(name):
    """A float from the DEPLOYED config, or a loud failure. Not a dataclass default:
    that substitution is what made a bench probe answer from a robot twice the real
    size (see tests/test_footprint_derivation.py)."""
    assert name in SUP, f"{name} is not in the deployed config"
    return float(SUP[name])


# --- B-1: the near edge of the ToF window, DERIVED rather than transcribed ----------

def nearest_reportable_ground_range_m(cfg: TofConfig) -> float:
    """The nearest ground range this geometry can report, in the TRUE base_link frame.

    Every zone at its `min_valid_mm` floor; the nearest of those is the near edge of
    what the sensor can physically produce. Derived here because the config states it
    as a bare 0.1517 and `tests/test_tof_brake_wiring.py` pins against that same
    copied literal -- so today the number is believed by two files and computed by
    none. A geometry change would move the truth and leave both agreeing.
    """
    best = math.inf
    for row in range(8):
        for col in range(8):
            p = zone_point(row, col, cfg.min_valid_mm, cfg)
            if p:
                best = min(best, math.hypot(p[0], p[1]))
    assert math.isfinite(best), "no zone produced a point at min_valid_mm"
    return best


def test_the_near_edge_discards_nothing_the_sensor_can_report():
    """L277-289. The near edge must sit BELOW the nearest reportable range, and the
    direction is the whole point: a near edge set too high silently drops the closest
    obstacles, which are the ones there is least time for. The old 0.08 was derived
    from the broken frame's 0.052 m and sat ABOVE the true 0.1517.
    """
    nearest = nearest_reportable_ground_range_m(TofConfig())
    assert nearest == pytest.approx(0.1517, abs=5e-4), (
        f"the nearest reportable ground range moved to {nearest:.4f} m; the config "
        "comment's 0.1517 and every window derived from it need re-deriving")
    assert _f("low_obstacle_min_range_m") < nearest, (
        "the near edge is at or above the nearest range the sensor can report, so the "
        "window discards real returns -- and the ones it discards are the closest")


def test_the_near_edge_allowance_is_the_stated_twelve_millimetres():
    """The comment claims '~12 mm of allowance for calibration drift before it would
    start discarding again'. Pinned as a figure so a geometry change that eats the
    allowance is visible before it eats the returns."""
    allowance = nearest_reportable_ground_range_m(TofConfig()) - _f("low_obstacle_min_range_m")
    assert allowance == pytest.approx(0.0117, abs=1e-3)


# --- B-2: the staleness chain, and it crosses a FILE boundary -----------------------

def _driver_cmd_vel_timeout_s() -> float:
    """The driver's own watchdog, read from the driver. L19 of the config asserts an
    ordering against this number and nothing checks it -- the two constants live in
    different files owned by different nodes, which is exactly where the project's
    seam defects come from."""
    import re
    m = re.search(r"^\s*cmd_vel_timeout:\s*float\s*=\s*([0-9.]+)", RVR_NODE.read_text(), re.M)
    assert m, "rvr_node.py no longer declares cmd_vel_timeout"
    return float(m.group(1))


def test_the_supervisor_forces_zero_before_the_driver_watchdog_fires():
    """L16-20: 'Receipt age still forces zero at 0.30 s, before the 0.5 s driver
    watchdog.' If this inverts, the driver's failsafe becomes the first thing to act
    on a dead scan and the supervisor's staleness policy is dead code -- the
    unreachable-recovery family, arrived at through a config ordering.

    `__post_init__` already enforces max_scan_stamp_age_s >= max_scan_age_s, so only
    the CROSS-FILE half is pinned here.
    """
    assert _f("max_scan_age_s") < _driver_cmd_vel_timeout_s(), (
        f"max_scan_age_s {_f('max_scan_age_s')} no longer precedes the driver's "
        f"{_driver_cmd_vel_timeout_s()} s cmd_vel watchdog")


def test_the_provenance_ceiling_is_a_ceiling_and_not_the_stream_clock():
    """L17-20 distinguishes two clocks that a reader will otherwise merge. The stamp
    ceiling is source-provenance sanity; the receipt age is what forces zero. Pinned
    as the strict inequality, because equality would make them one clock."""
    assert _f("max_scan_stamp_age_s") > _f("max_scan_age_s")


# --- B-3: the ToF brake's range window ---------------------------------------------

def test_the_brake_window_covers_the_whole_band_it_acts_on():
    """L290-295. The brake takes no action beyond `low_obstacle_max_range_m`, so any
    obstacle inside the slow band but outside the window is detected and then
    discarded. The invariant is `max_range >= slow_distance`.

    IT HOLDS AT EXACT EQUALITY, and that is the finding this test exists to keep
    visible: both are 0.60. The comment above the constant describes headroom that
    the 2026-08-14 widening of `low_obstacle_slow_distance_m` (0.30 -> 0.60) spent in
    full. A further widening of the slow band alone would break this, and would look
    free to anyone reading only the prose.
    """
    assert _f("low_obstacle_max_range_m") >= _f("low_obstacle_slow_distance_m"), (
        "the slow band now extends past the range window, so the brake eases off for "
        "obstacles it will then refuse to see")


def test_the_brake_window_headroom_is_zero_and_that_is_declared():
    """DECLARED, NOT DISCOVERED, in the style of the mission-2 pivot verdict. If
    someone buys headroom back this fails and is the right thing to update; what must
    not happen is the margin changing while a comment still claims 0.30 m of it."""
    headroom = _f("low_obstacle_max_range_m") - _f("low_obstacle_slow_distance_m")
    assert headroom == pytest.approx(0.0, abs=1e-9)


# --- B-4: the hysteresis gap -------------------------------------------------------

def test_the_release_gap_is_the_stated_ten_centimetres():
    """L133-139. `__post_init__` already refuses release <= stop, so the ordering is a
    production invariant and is not re-tested here. What is unpinned is the SIZE:
    0.40 against 0.30 is 0.10 m of backing to clear the latch. Too small and it
    chatters; too large and a short back-off cannot re-open clearance, which is the
    'protection without progress' shape."""
    gap = _f("release_distance_m") - _f("stop_distance_m")
    assert gap == pytest.approx(0.10, abs=1e-9)


# --- B-5: where the rover actually comes to rest ------------------------------------

def braking_distance_m() -> float:
    """Travel after the stop fires, from the deployed config: cruise x measured stop
    time + margin. The supervisor's own operands, not a second author of them."""
    return _f("max_forward_mps") * _f("measured_stop_time_s") + _f("braking_distance_margin_m")


def test_the_rest_position_clears_the_measured_nose():
    """L155-163's claim, which nothing pins today: braking at the 0.35 cruise is
    ~0.11 m, so an obstacle that trips the stop at `stop_distance_m` ends up ~0.19 m
    from base_link -- about 0.09 m off the measured nose.

    `tests/test_speed_raise.py` pins that the physics term stays under
    `stop_distance_m`; that is a different claim. It says the threshold governs. This
    says the robot does not touch the thing, which is the claim a person cares about.
    """
    braking = braking_distance_m()
    assert braking == pytest.approx(0.1075, abs=5e-4)

    rest_from_base_link = _f("stop_distance_m") - braking
    assert rest_from_base_link == pytest.approx(0.19, abs=5e-3)

    clearance = rest_from_base_link - _f("footprint_front_m")
    assert clearance > 0.0, (
        f"the rover comes to rest {-clearance:.3f} m INSIDE its own measured nose -- "
        "the stop distance no longer covers braking at the deployed cruise")

    # THE DERIVED FIGURE IS 0.0960 m, and the config says '~0.09'. That is a 6 mm
    # understatement, and it is left alone deliberately: it errs toward LESS clearance
    # than the rover actually has, which is the safe direction for a margin someone
    # reasons about. Pinned as the exact value plus the direction, so a future drift
    # that makes the prose OPTIMISTIC fails here even though the '~' would still cover
    # it grammatically.
    assert clearance == pytest.approx(0.0960, abs=5e-4)
    assert clearance >= 0.09, (
        "the nose clearance fell below the ~0.09 m the config claims; the prose now "
        "overstates the margin instead of understating it")


# --- B-6/B-7: the sectors ----------------------------------------------------------

def test_the_slow_cone_strictly_contains_the_stop_cone():
    """L79-84. The stop cone must sit INSIDE the slow cone, or an obstacle enters the
    hard stop without ever having been eased off for -- the rover brakes flat from
    cruise with no warning band. The deployed pair is +-35 around +-30; nothing
    validates the ordering, and inverting two lines of yaml would do it silently."""
    assert _f("front_slow_min_angle_deg") < _f("front_stop_min_angle_deg")
    assert _f("front_slow_max_angle_deg") > _f("front_stop_max_angle_deg")


def test_the_front_cones_are_symmetric_about_straight_ahead():
    """Both cones are stated as +-N. An asymmetric front cone would bias every stop
    decision to one side of the robot and would read as a steering fault, not a
    config fault."""
    for lo, hi in (("front_stop_min_angle_deg", "front_stop_max_angle_deg"),
                   ("front_slow_min_angle_deg", "front_slow_max_angle_deg")):
        assert _f(lo) == pytest.approx(-_f(hi))


def test_the_spin_sectors_mirror_each_other_and_clear_the_front_cone():
    """L86-89. Left and right spin sectors are mirror images, and neither may overlap
    the front slow cone -- a spin sector that reached into the front cone would let a
    single return veto both a turn and a forward crawl, which is the pose no motion
    can leave."""
    lo_l, hi_l = _f("left_spin_min_angle_deg"), _f("left_spin_max_angle_deg")
    lo_r, hi_r = _f("right_spin_min_angle_deg"), _f("right_spin_max_angle_deg")
    assert (lo_l, hi_l) == pytest.approx((-hi_r, -lo_r))
    assert lo_l > _f("front_slow_max_angle_deg"), (
        "the left spin sector overlaps the front slow cone")
    assert hi_r < _f("front_slow_min_angle_deg"), (
        "the right spin sector overlaps the front slow cone")


# --- B-8: the hold's arming threshold ----------------------------------------------

def test_the_hold_arms_at_the_stated_threshold():
    """L329-334. The threshold is band + closure, both operands read live in the node
    so the deployed config cannot disagree with the sensor. That is the right design
    AND the reason nothing pinned the figure -- there is no constant to pin, so the
    number lived only in prose, and the prose drifted (the 2026-08-19 speed raise
    moved it 0.227 -> 0.272 and one sentence of the block kept the old value).

    `tests/test_blind_band_hold.py` pins the inequalities this threshold must satisfy.
    This pins the FIGURE the config states, which is what actually rotted.
    """
    band = blind_band_outer_range_m(TofConfig())
    closure = _f("max_forward_mps") * _f("low_obstacle_max_age_s")
    assert band == pytest.approx(0.1672, abs=5e-4)
    assert closure == pytest.approx(0.105, abs=5e-4)
    assert band + closure == pytest.approx(0.272, abs=1e-3)


def test_the_hold_arms_further_out_than_the_stop_it_backstops():
    """The pair's whole safety argument: the hold's trigger is a LOST BELIEF, not a
    stopping distance, so its threshold is deliberately larger than the stop
    distance. Run 1's leg was lost at 0.181 m and drove 90 mm into the object."""
    threshold = blind_band_outer_range_m(TofConfig()) + _f("max_forward_mps") * _f(
        "low_obstacle_max_age_s")
    assert threshold > _f("low_obstacle_stop_distance_m")
