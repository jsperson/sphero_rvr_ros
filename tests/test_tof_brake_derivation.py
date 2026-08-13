"""Derived constants for a ToF brake — and the reason there isn't one yet.

CI form of the three-clause derivation:
  (a) the stop distance must be >= braking distance at cruise, plus margin
  (b) it must be <= the sensor's reliable detection range
  (c) if (a) and (b) conflict the answer is LOWER CRUISE, never a stretched sensor claim

Computing (b) honestly is what stopped this batch. The numbers are below and they are
read from the DEPLOYED yaml and the shipped geometry, never from a remembered figure.
"""
import math
import re
from pathlib import Path

import dataclasses
import pytest

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from sphero_rvr_core.tof_frame import (  # noqa: E402
    TofConfig, _floor_reading_m, rule_a_rows, zone_point,
)

CFG = Path(__file__).resolve().parents[1] / "config" / "collision_stop.yaml"


def _yaml_float(name):
    m = re.search(rf"^\s*{name}:\s*([0-9.]+)", CFG.read_text(), re.M)
    assert m, f"{name} is not in the deployed config"
    return float(m.group(1))


def rule_a_reach_m(cfg: TofConfig) -> float:
    """Furthest GROUND RANGE at which rule A can flag anything.

    Not the floor intersection -- the floor margin sits between them. A zone fires only
    below `floor - floor_margin`, so the obstacle has to be that much nearer than the
    floor, and THAT is what caps the range. Reading the floor distance as the detection
    range overstates it by the whole margin.
    """
    best = 0.0
    for row in rule_a_rows(cfg):
        floor = _floor_reading_m(row, 3, cfg)
        trigger = floor - cfg.floor_margin_m
        if trigger <= cfg.min_valid_mm / 1000.0:
            continue
        p = zone_point(row, 3, int(trigger * 1000), cfg)
        if p:
            best = max(best, math.hypot(p[0], p[1]))
    return best


def stop_requirement_m() -> float:
    """The distance in which the rover must PHYSICALLY stop, from the deployed config.

    NOT `stop_distance_m`. That is a THRESHOLD chosen to exceed this with margin, and
    comparing a detection range against it asks the wrong question -- which is exactly
    the mistake this file was written to prevent a second time.
    """
    return (_yaml_float("footprint_front_m")
            + _yaml_float("payload_margin_m")
            + _yaml_float("braking_distance_margin_m"))


def test_rule_a_alone_IS_a_viable_short_range_brake():
    """The corrected derivation (design 11.2-11.3). The first version of this test
    asserted the opposite and was wrong twice over.

    ERROR 1 -- it treated the lidar's 0.30 m stop as covering this obstacle class. It
    does not. A 5 cm rail is invisible to the lidar at EVERY distance; that is why this
    layer exists. Rule A is not 2 mm inside an existing stop, it is the only detection
    the class gets.

    ERROR 2 -- it compared against a threshold instead of a requirement. `stop_distance_m`
    is chosen to exceed the physical stop with margin. The requirement is the footprint
    plus payload plus braking sum, and rule A clears it comfortably.

    Both errors pushed the same way and nearly cancelled a sound batch.
    """
    reach = rule_a_reach_m(TofConfig())
    need = stop_requirement_m()

    assert reach > need, (
        f"rule A reaches {reach:.3f} m against a {need:.3f} m stop requirement -- it can "
        "no longer stop clean, so re-derive 11.3 rather than assuming it still holds")
    assert reach - need > 0.10, (
        f"rule A's margin over the stop requirement fell to {reach - need:.3f} m; the "
        "derivation assumed a comfortable 0.12-0.15 m and should be redone")


def test_the_stop_requirement_is_not_the_lidar_threshold():
    """Pins the operand itself, because substituting the threshold is the specific error
    that produced a retracted conclusion. They are different numbers and must stay
    different -- if they ever coincide, this test stops being able to catch the
    substitution and says so."""
    need = stop_requirement_m()
    threshold = _yaml_float("stop_distance_m")
    assert need != pytest.approx(threshold, abs=0.01), (
        f"the physical stop requirement ({need:.3f}) and the lidar's threshold "
        f"({threshold:.3f}) now coincide, so confusing them is undetectable here. "
        "Re-establish the distinction before trusting any range derivation")
    assert need < threshold, (
        "the lidar's threshold no longer exceeds the physical requirement it exists to "
        "cover -- that is a lidar-core problem, not a ToF one, and it outranks this file")


def test_rule_b_buys_range_not_existence():
    """Row 3 reaches 0.498 m against rule A's 0.298 m -- 0.20 m, a full second at the
    0.20 m/s cruise. That is an earlier and gentler stop, and room to steer rather than
    halt. It is an upgrade to the brake, NOT a precondition for having one, and the
    difference decides whether bench item J gates the batch or merely improves it."""
    cfg = TofConfig()
    wide = dataclasses.replace(cfg, stop_distance_m=0.70)      # admits row 3
    assert 3 in rule_a_rows(wide) and 3 not in rule_a_rows(cfg)
    gain = rule_a_reach_m(wide) - rule_a_reach_m(cfg)
    assert gain > 0.15, (
        f"the range rule B buys fell to {gain:.3f} m; if it is no longer worth a second "
        "of warning, bench item J's priority needs re-arguing")


def test_rule_b_authority_requires_a_pinning_citation():
    """SCOPED DOWN after the 11.2 correction. This used to block the ToF brake entirely,
    on the retracted premise that rule-A-only was inert. It is not -- rule A may fly
    without bench item J.

    What still needs the citation is RULE B's own authority: its disagreement margin has
    no recorded data behind it, and a threshold that looks like every other threshold is
    how a provisional number becomes a fact nobody re-examines. So the gate now sits on
    the claim it honestly covers, and not one step wider.
    """
    text = CFG.read_text()
    topic = re.search(r"^\s*low_obstacle_topic:\s*(\S+)", text, re.M)
    enable = re.search(r"^\s*low_obstacle_brake_enable:\s*(\w+)", text, re.M)
    assert topic and enable

    # The gate lives on the DETECTOR, not on this consumer -- rule B decides what it
    # publishes, and a consumer-side gate would leave the detector claiming obstacles
    # nobody may act on. So the citation is checked here and the gate itself in
    # TofConfig, which is the thing that actually decides.
    if topic.group(1) == "/tof/obstacles" and enable.group(1) == "true":
        pinned = re.search(r"^\s*#\s*RULE B PINNED BY:", text, re.M)
        assert pinned or not TofConfig().rule_b_enable, (
            "the ToF has brake authority with rule B ENABLED and no pinning citation. "
            "Rule A alone is fine to fly (design 11.3); rule B is not, while its margin "
            "rests on no recorded data at all. Either run bench item J and cite the "
            "capture as '# RULE B PINNED BY: ...' in this config, or leave "
            "TofConfig.rule_b_enable False and fly rule A honestly")


def test_the_shipped_gate_is_real_and_defaults_OFF():
    """The guard above cites `rule_b_enable`. A guard that names a parameter nobody
    implemented is worse than no guard: it reads as protection and enforces nothing.

    That is the unreachable-recovery pattern in test form, and the first draft of this
    file had exactly it -- the citation named a `low_obstacle_rule_b_enable` key that
    existed in no config and no code.
    """
    cfg = TofConfig()
    assert hasattr(cfg, "rule_b_enable"), "the gate the guard cites does not exist"
    assert cfg.rule_b_enable is False, (
        "rule B ships ENABLED. Its disagreement margin has never been measured against "
        "a synchronised scan; bench item J is what changes that")

    from sphero_rvr_core.tof_frame import lidar_disagreement
    frame = [4000] * 64
    for col in (2, 3, 4):
        frame[3 * 8 + col] = 500                      # the recorded object, 0.50 m
    assert lidar_disagreement(frame, cfg, [3.0] * 8) == [], (
        "rule B produced obstacles while gated off -- the flag is declared but not "
        "consulted, which is a gate in name only")
    live = dataclasses.replace(cfg, rule_b_enable=True)
    assert lidar_disagreement(frame, live, [3.0] * 8), (
        "rule B produces nothing even when ENABLED, so the test above proves nothing")


def test_the_state_token_describes_CONFIGURATION_not_capability():
    """`rule_a_only` / `rule_a+b`, never `short_range` or `full`.

    A capability word drifts the moment a constant moves -- "short_range" would have
    been written when rule A's reach was believed to be redundant with the lidar, and
    would still read "short_range" after the correction that made it a real brake. A
    configuration word stays true by construction. Report what IS; let the reader judge
    what it means.
    """
    from sphero_rvr_core.tof_frame import ObstacleDetector
    gated = ObstacleDetector(TofConfig()).update([4000] * 64, None)
    live = ObstacleDetector(dataclasses.replace(TofConfig(), rule_b_enable=True)).update(
        [4000] * 64, None)
    assert gated["rules"] == "rule_a_only"
    assert live["rules"] == "rule_a+b"
    for banned in ("short", "full", "degraded", "limited", "safe"):
        assert banned not in gated["rules"] and banned not in live["rules"], (
            f"the token contains the capability word {banned!r}; it will outlive the "
            "numbers that made it true")


def test_the_stop_distance_would_be_derived_not_chosen():
    """Clause (a), recorded so the number is never picked by feel later.

    The braking distance at cruise is not theorised here -- the LIDAR's proven-in-flight
    0.30 m at the same chassis and the same 0.20 m/s cruise is the empirical figure, and
    it already contains the physical stop. A ToF brake inherits it plus the difference in
    sensor period (7.6 Hz against 10 Hz is ~6 mm at cruise), giving a floor near 0.31 m.
    """
    cruise = _yaml_float("max_forward_mps")
    lidar_stop = _yaml_float("stop_distance_m")
    tof_hz, lidar_hz = 7.6, 10.0
    extra = cruise * (1.0 / tof_hz - 1.0 / lidar_hz)
    floor_m = lidar_stop + extra

    assert 0.30 < floor_m < 0.33, (
        f"the derived braking floor moved to {floor_m:.3f} m (cruise {cruise}, lidar "
        f"stop {lidar_stop}); re-check the derivation before using it")
    assert floor_m > rule_a_reach_m(TofConfig()), (
        "rule A's reach now EXCEEDS the braking floor, which means the geometry or the "
        "margin changed enough that a rule-A-only brake may finally be viable -- "
        "re-derive rather than assuming the conclusion above still holds")
