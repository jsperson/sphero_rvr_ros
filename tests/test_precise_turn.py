"""The precise-turn primitive and its gateway gate, chassis-off testable halves.

The firmware heading loop itself is bench-card territory (docs/
bench_card_2026-08-19.md); what IS testable tonight: the driver primitive's
refusals and packet shape against the fake transport, the pure admission gate,
the heading math, and the source-level wiring guards. Un-runnable-by-construction
is the spine: every path to a motor refuses until the card flips the flag.
"""

import asyncio
import math
import re
from pathlib import Path

import pytest

from sphero_rvr_core import sensor_streaming
from sphero_rvr_core.driver import RVRDriver
from sphero_rvr_core.fake_transport import FakeTransport
from sphero_rvr_driver.collision_stop import CollisionState, precise_turn_admission

REPO = Path(__file__).resolve().parents[1]
NODE_SRC = (REPO / "src" / "sphero_rvr_driver" / "collision_stop_node.py").read_text()
RVR_SRC = (REPO / "src" / "sphero_rvr_driver" / "rvr_node.py").read_text()


def make_sample(yaw_deg):
    yaw = math.radians(yaw_deg)
    return sensor_streaming.ImuSample(
        orientation=(0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)),
        angular_velocity=(0.0, 0.0, 0.0),
        linear_acceleration=(0.0, 0.0, 0.0),
        is_valid=True,
    )


# --- the pure heading math -------------------------------------------------------------

def test_firmware_heading_wraps_to_0_360():
    assert sensor_streaming.firmware_heading_deg(make_sample(30.0)) == pytest.approx(30.0)
    assert sensor_streaming.firmware_heading_deg(make_sample(-30.0)) == pytest.approx(330.0)
    assert sensor_streaming.firmware_heading_deg(make_sample(370.0)) == pytest.approx(10.0, abs=1e-6)


# --- the driver primitive, against the fake transport ----------------------------------

def run(coro):
    return asyncio.get_event_loop_policy().new_event_loop().run_until_complete(coro)


def make_driver(**kwargs):
    return RVRDriver(FakeTransport(), **kwargs)


def test_unverified_bench_flag_refuses_before_anything_else():
    """Un-runnable-by-construction: ships false, and the refusal happens before
    the IMU is even consulted -- a motor verb must not run on an unmeasured
    safety seam (estop preemption is bench item i)."""
    driver = make_driver()   # precise_turn_verified defaults False
    with pytest.raises(RuntimeError, match="bench card not verified"):
        run(driver.turn_by_degrees(90.0))


def test_stale_or_absent_imu_refuses_loudly():
    """A turn computed from a stale yaw is a turn to a made-up place -- the
    instrument-death pattern: loud refusal, never silence."""
    driver = make_driver(precise_turn_verified=True)
    with pytest.raises(RuntimeError, match="no fresh IMU yaw"):
        run(driver.turn_by_degrees(90.0))


@pytest.mark.asyncio
async def test_a_fresh_sample_produces_the_heading_command():
    """The packet shape: drive_with_heading(speed=0, target) where target is
    current + SIGN*delta wrapped to [0,360). SIGN is the ASSUMED -1 until bench
    item (iii) measures it -- pinned here so a silent flip fails a test."""
    from sphero_rvr_core.safety import now_seconds

    driver = make_driver(precise_turn_verified=True)
    await driver.connect()
    try:
        driver._imu_sample = make_sample(10.0)
        driver._imu_sample_at = now_seconds()
        current, target = await driver.turn_by_degrees(90.0)
        assert current == pytest.approx(10.0)
        assert driver.PRECISE_TURN_HEADING_SIGN == -1.0
        assert target == pytest.approx((10.0 - 90.0) % 360.0)   # 280: left, CW-positive frame
    finally:
        await driver.disconnect()


@pytest.mark.asyncio
async def test_wraparound_at_the_seam():
    from sphero_rvr_core.safety import now_seconds

    driver = make_driver(precise_turn_verified=True)
    await driver.connect()
    try:
        driver._imu_sample = make_sample(350.0)
        driver._imu_sample_at = now_seconds()
        _, target = await driver.turn_by_degrees(-30.0)      # right 30 in a CW frame: +30
        assert target == pytest.approx(20.0)
    finally:
        await driver.disconnect()


# --- the admission gate ------------------------------------------------------------------

CORNER = 0.209


def test_admission_refuses_without_the_bench_flag():
    ok, reason = precise_turn_admission(False, CollisionState.CLEAR, 1.0, CORNER)
    assert not ok and "bench card" in reason


def test_admission_requires_clear_not_stopped_not_stale():
    for state in (CollisionState.STOPPED, CollisionState.SENSOR_STALE,
                  CollisionState.ESTOPPED, CollisionState.STARTUP):
        ok, reason = precise_turn_admission(True, state, 1.0, CORNER)
        assert not ok and state.value in reason


def test_admission_fails_closed_on_no_reading():
    """D18's rule: no reading means NO, never 'probably fine'."""
    ok, reason = precise_turn_admission(True, CollisionState.CLEAR, None, CORNER)
    assert not ok and "fails closed" in reason


def test_admission_uses_the_corner_circle():
    ok, _ = precise_turn_admission(True, CollisionState.CLEAR, CORNER - 0.01, CORNER)
    assert not ok
    ok, reason = precise_turn_admission(True, CollisionState.CLEAR, CORNER + 0.05, CORNER)
    assert ok and reason == "admitted"


# --- the wiring, source-level -------------------------------------------------------------

def test_the_gateway_is_the_only_door_and_every_exit_stops_the_driver():
    """cancel == stop, timeout == stop, safety transition == stop: each of the
    gateway's non-success exits must route through the driver-stop path."""
    body_start = NODE_SRC.index("def _execute_precise_turn")
    body = NODE_SRC[body_start:NODE_SRC.index("def _decide_and_publish")]
    assert "precise_turn_admission(" in body
    assert 'finish_stopped("cancel requested", canceled=True)' in body
    assert "timeout" in body and "_driver_stop_client" in body
    assert "ESTOPPED" in body and "STOPPED" in body


def test_the_driver_lane_is_thin_and_reports_refusals():
    """rvr_node's side must stay non-blocking (single-threaded executor) and must
    surface driver refusals on the event topic rather than swallowing them."""
    assert "/rvr_driver/precise_turn_cmd" in RVR_SRC
    assert "/rvr_driver/precise_turn_event" in RVR_SRC
    assert 'String(data=f"REFUSED {exc}")' in RVR_SRC
    assert "turn_by_degrees" in RVR_SRC


def test_both_yamls_ship_the_flag_false():
    for name in ("lean_rvr_tank_si.yaml", "collision_stop.yaml"):
        text = (REPO / "config" / name).read_text()
        m = re.search(r"precise_turn_bench_verified:\s*(\w+)", text)
        assert m, f"{name} lost the bench flag"
        assert m.group(1) == "false", (
            f"{name} ships the bench flag {m.group(1)} -- it flips only in a "
            f"reviewed commit citing the measured card")
