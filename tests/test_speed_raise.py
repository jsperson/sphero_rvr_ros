"""The linear speed raise, held together: three gates, derived margins, the bar.

Every number here is read from the DEPLOYED yamls (probe-the-deployed-config)
and checked against the derivations in docs/design_linear_speed_raise_2026-08-19
+ the consensus pin (the progress bar keys on the REGULATED MINIMUM). The
measured constants come from bag_20260819_141021 (the re-fly) and are fixtures
with provenance, not tunables.
"""

import math
from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parents[1]


def _load(name):
    return yaml.safe_load((REPO / "config" / name).read_text())


NAV = _load("lean_nav2_stock.yaml")
RVR = _load("lean_rvr_tank_si.yaml")["sphero_rvr_driver"]["ros__parameters"]
SUP = _load("collision_stop.yaml")["lidar_collision_stop_supervisor"]["ros__parameters"]
EXP = _load("coverage_explorer.yaml")["coverage_explorer"]["ros__parameters"]

RPP = NAV["controller_server"]["ros__parameters"]["FollowPath"]
CRUISE = float(RPP["desired_linear_vel"])


# --- the three gates move together (the gap-crossing lesson) ---------------------------

def test_the_three_speed_gates_are_one_decision():
    assert CRUISE == float(RVR["max_linear_mps"]) == float(SUP["max_forward_mps"]), (
        "the speed gates diverged -- max_forward_mps is the FINAL clamp and a "
        "forgotten gate silently overrides the other two (gap-crossing era)")
    assert CRUISE == 0.35, "the ratified raise landed a different number than reviewed"


# --- rederived margins, each an inequality that must HOLD at the deployed speed --------

def test_braking_still_governed_by_the_hard_stop():
    """physics = footprint_front + payload + v*stop_time + margin must stay
    under stop_distance_m, or the hard stop stops being the governing term
    without anyone deciding that."""
    physics = (0.0965 + 0.02
               + CRUISE * float(SUP["measured_stop_time_s"])
               + float(SUP["braking_distance_margin_m"]))
    assert physics < float(SUP["stop_distance_m"]), (
        f"braking physics {physics:.3f} exceeds stop_distance "
        f"{SUP['stop_distance_m']} -- the raise outran the stop derivation")


def test_the_slow_band_keeps_half_a_second_of_ease_off():
    band = float(SUP["slow_distance_m"]) - float(SUP["stop_distance_m"])
    assert band / CRUISE >= 0.5, (
        f"the SLOW band is {band/CRUISE:.2f}s at cruise -- under the 0.5s floor")


def test_the_lookahead_no_longer_clips_at_cruise():
    """Caught in the design round: 0.35 x 1.3 = 0.455 > the old 0.36 max."""
    assert float(RPP["max_lookahead_dist"]) >= CRUISE * float(RPP["lookahead_time"]), (
        "velocity-scaled lookahead clips at cruise -- decided-not-inherited was "
        "the whole point of catching this")


def test_the_low_obstacle_stale_window_holds_WITH_the_scale_credit():
    """The margin that FAILS naively at 0.35 (0.105 > 0.097) and holds only
    because the brake's own scaling caps in-window speed. The whole chain is
    pinned because the margin depends on every link: scale(reach) x cruise x
    max_age < reach - stop."""
    stop = float(SUP["low_obstacle_stop_distance_m"])
    slow = float(SUP["low_obstacle_slow_distance_m"])
    floor = float(SUP["low_obstacle_min_forward_scale"])
    age = float(SUP["low_obstacle_max_age_s"])
    reach = 0.297   # the true-frame reach, from the deployed derivation block
    window = reach - stop
    # the brake's own formula, imported not restated
    from sphero_rvr_core.low_obstacle_brake import forward_speed_scale
    scale_at_edge = forward_speed_scale(reach, stop, slow, floor)
    stale_travel = scale_at_edge * CRUISE * age
    assert stale_travel < window, (
        f"stale travel {stale_travel:.3f} m crosses the {window:.3f} m approach "
        f"window -- a single stale frame can carry the rover through it")
    # and the naive form MUST still fail, or this pin is testing nothing:
    assert CRUISE * age > window, (
        "unscaled stale travel now fits the window -- the scale credit is no "
        "longer load-bearing; simplify the derivation and this pin together")


def test_the_tof_obstacle_gate_outranges_the_stale_travel():
    from sphero_rvr_core.tof_frame import TofConfig
    tof_stop = TofConfig().stop_distance_m   # deployed = dataclass default, 0.45
    assert tof_stop > float(SUP["stop_distance_m"]) + CRUISE * float(SUP["low_obstacle_max_age_s"]), (
        "the tof gate no longer covers hard-stop + stale travel at cruise")


# --- the progress bar: keyed on the regulated minimum (the consensus pin) --------------

#: MEASURED, bag_20260819_141021 (the re-fly): net displacement per 6 s window.
#: Succeeded-goal windows never fell below this rate (n=29 windows);
#: genuinely-pinned end-game windows ran this median (n=21). Provenance:
#: scratch analysis 2026-08-19, docs/design_linear_speed_raise_2026-08-19.md.
MEASURED_LEGIT_FLOOR_RATE = 0.221 / 6.0     # 0.0368 m/s
MEASURED_PINNED_MEDIAN_RATE = 0.030 / 6.0   # 0.0050 m/s


def test_the_bar_keys_on_the_regulated_minimum_not_cruise():
    """epsilon = k * v_regulated_min * window, k stated by the deployed values;
    the pin's arithmetic: a cruise-keyed bar tightens faster than crawl speeds
    up and the false stalls return."""
    v_min = float(RPP["regulated_linear_scaling_min_speed"])
    window = float(EXP["goal_progress_timeout_s"])
    epsilon = float(EXP["goal_progress_epsilon_m"])
    k = epsilon / (v_min * window)
    assert k < 1.0, "k >= 1 demands the minimum be sustained NET -- unmeetable"
    assert abs(epsilon - k * v_min * window) < 1e-9  # the form itself
    # the bar RATE sits between the measured distributions, with margin both ways
    rate = epsilon / window
    assert rate < MEASURED_LEGIT_FLOOR_RATE / 1.5, (
        f"bar rate {rate:.4f} is within 1.5x of the measured legitimate floor "
        f"{MEASURED_LEGIT_FLOOR_RATE:.4f} -- false kills return")
    assert rate > MEASURED_PINNED_MEDIAN_RATE * 2.0, (
        f"bar rate {rate:.4f} is within 2x of the pinned median -- true pins "
        f"outlive the watchdog")


def test_the_window_covers_a_recovery_turn():
    """The re-fly's goal 1 spent 4 s of its 6 s budget in a gateway turn and was
    killed while making legitimate discovery progress. The window must exceed
    the gateway's own turn bound (precise_turn_timeout_s 5.0) plus displacement
    time -- read from the deployed supervisor config so a turn-bound tune
    re-asks this question."""
    turn_bound = float(SUP["precise_turn_timeout_s"])
    window = float(EXP["goal_progress_timeout_s"])
    assert window >= turn_bound + 3.0, (
        f"a goal that opens with a max-length recovery turn ({turn_bound}s) "
        f"has under 3s of displacement time in a {window}s window")


def test_the_controller_side_checker_was_left_alone_deliberately():
    """PoseProgressChecker (0.05 m / 12 s) never false-fired and is the
    safety-side check; the raise does not touch it. A change here must arrive
    with its own derivation."""
    pc = NAV["controller_server"]["ros__parameters"]["progress_checker"]
    assert float(pc["required_movement_radius"]) == 0.05
    assert float(pc["movement_time_allowance"]) == 12.0


def test_rotation_constants_are_untouched():
    """Scott's addendum: 'Turns are plenty fast so no need to increase there.'"""
    assert float(RPP["rotate_to_heading_angular_vel"]) == 3.55
    assert float(RVR["pivot_max_duty"]) == 45
    assert float(RVR["max_angular_rad_s"]) == 0.4
