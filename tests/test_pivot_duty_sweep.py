"""Offline tests for `diagnostics/pivot_duty_sweep.py` -- the attended breakaway tool.

The tool commands motors on an attended robot with a human at the power switch, so the
whole point of this file is that nothing about it gets debugged while Scott stands over
the rover. Every decision path runs here against a fake driver: the refusals, the
instrument-death handling, the ladder's early stop, the validity verdict in both
directions, and the scale conversion the whole measurement is expressed in.

The fake driver refuses `set_velocity` outright, because a sweep that reached the rate
path would be measuring the closed-loop pivot controller -- the thing under test --
instead of the duty.
"""

from __future__ import annotations

import importlib.util
import os
import sys
from dataclasses import dataclass
from pathlib import Path
from types import SimpleNamespace

import pytest

REPO_ROOT = Path(__file__).parents[1]
SWEEP_PATH = REPO_ROOT / "diagnostics" / "pivot_duty_sweep.py"


def _load_module():
    spec = importlib.util.spec_from_file_location("pivot_duty_sweep", SWEEP_PATH)
    module = importlib.util.module_from_spec(spec)
    sys.modules["pivot_duty_sweep"] = module
    spec.loader.exec_module(module)
    return module


sweep = _load_module()


# ======================================================================================
# The scale conversion -- one place, pinned. Two scales are what made the first autopsy
# quote a ceiling that was not the deployed one.
# ======================================================================================


def test_raw255_to_tank127_pins_the_documented_figures():
    # "angular duty <=128 does not move at all, 140-160 breaks away then bogs" is on the
    # RAW-MOTOR 0-255 scale; in-place pivots go out on drive_tank_normalized's +/-127.
    assert sweep.raw255_to_tank127(128) == 64
    assert sweep.raw255_to_tank127(140) == 70
    assert sweep.raw255_to_tank127(160) == 80
    assert sweep.raw255_to_tank127(255) == 127
    assert sweep.raw255_to_tank127(0) == 0


def test_tank127_to_raw255_pins_both_deployed_pivot_ceilings():
    assert sweep.tank127_to_raw255(32) == 64  # rvr_node dataclass default
    assert sweep.tank127_to_raw255(45) == 90  # config/lean_rvr_tank_si.yaml, explore.launch
    assert sweep.tank127_to_raw255(23) == 46
    assert sweep.tank127_to_raw255(28) == 56
    assert sweep.tank127_to_raw255(127) == 255


def test_both_deployed_ceilings_sit_below_the_documented_no_move_duty():
    # The claim that makes this measurement worth a robot: BOTH shipped ceilings convert
    # to less than the raw duty the driver's own comment says produces no motion at all.
    for ceiling in (32, 45):
        assert sweep.tank127_to_raw255(ceiling) < sweep.DOCUMENTED_NO_MOVE_RAW255


def test_scale_conversions_round_trip_within_one_count():
    for raw in range(0, 256, 7):
        assert abs(sweep.tank127_to_raw255(sweep.raw255_to_tank127(raw)) - raw) <= 1


# ======================================================================================
# The ladder and its bounds
# ======================================================================================


def test_default_ladder_starts_below_production_and_climbs_past_breakaway():
    ladder = sweep.default_ladder()
    assert ladder == sorted(set(ladder)), "ladder must ascend strictly"
    assert ladder[0] < sweep.PRODUCTION_PIVOT_MIN_DUTY
    assert ladder[-1] > sweep.raw255_to_tank127(sweep.DOCUMENTED_BREAKAWAY_RAW255[1])
    # It must actually sample the band production runs in, or the run cannot say what
    # the deployed ceilings do.
    for ceiling in (23, 28, 32, 45):
        assert ceiling in ladder
    # ...and the region the raw-motor comment calls breakaway.
    assert sweep.raw255_to_tank127(140) in ladder


def test_default_config_validates():
    config = sweep.SweepConfig().validated()
    assert config.duties == tuple(sweep.default_ladder())


@pytest.mark.parametrize(
    "kwargs, fragment",
    [
        ({"duties": (12, 200)}, "1..127"),
        ({"duties": (12, 40, 30, 90)}, "ascend"),
        ({"duties": (23, 40, 90)}, "BELOW the production pivot floor"),
        ({"duties": (12, 20, 40)}, "top of the"),
        ({"burst_s": 4.0}, "burst-s"),
        ({"settle_s": 1.0}, "settle-s"),
        ({"control_period": 0.0}, "control-period"),
    ],
)
def test_config_refuses_out_of_bounds_requests(kwargs, fragment):
    with pytest.raises(sweep.RefusalError) as excinfo:
        sweep.SweepConfig(**kwargs).validated()
    assert fragment in str(excinfo.value)


def test_the_run_cards_own_invocation_is_accepted():
    # docs/run_card_breakaway_2026-08-16.md section 2 prints this exact ladder and these
    # exact bounds. If a bound tightens here, the card's command must stop working in a
    # test rather than in front of the robot.
    config = sweep.SweepConfig(
        duties=sweep.parse_duties("20,28,32,40,50,60,70,80,90,100"),
        burst_s=2.0,
        settle_s=3.0,
    ).validated()

    assert config.duties[0] == 20
    assert config.duties[-1] == 100


def test_burst_and_settle_bounds_are_the_damaging_case_not_a_preference():
    # Sustained sub-moving duty is what powered the rover down twice. The tool may not
    # be asked for a long burst or a short cool-down.
    assert sweep.MAX_BURST_S <= 3.0
    assert sweep.MIN_SETTLE_S >= 2.0
    sweep.SweepConfig(burst_s=sweep.MAX_BURST_S, settle_s=sweep.MIN_SETTLE_S).validated()


# ======================================================================================
# Refusals that happen before anything touches the host
# ======================================================================================


def test_main_refuses_without_arm_before_it_looks_at_the_host(monkeypatch):
    def explode(*_args, **_kwargs):
        raise AssertionError("the --arm gate must come first")

    monkeypatch.setattr(sweep, "scan_port_holders", explode)
    monkeypatch.setattr(sweep, "config_from_args", explode)
    lines = []

    code = sweep.main([], out=lines.append)

    assert code == sweep.EXIT_REFUSED
    assert any("--arm not given" in line for line in lines)


def test_main_prints_the_safety_preamble_before_the_refusal():
    lines = []
    sweep.main([], out=lines.append)
    assert "HAND ON THE POWER SWITCH" in lines[0]
    assert "Rotation in place only" in lines[0]


def test_scan_port_holders_finds_another_process_holding_the_port(tmp_path):
    port = tmp_path / "ttyFAKE"
    port.write_text("")
    proc = tmp_path / "proc"
    for pid, target in ((4242, port), (4243, tmp_path / "other")):
        fd_dir = proc / str(pid) / "fd"
        fd_dir.mkdir(parents=True)
        (tmp_path / "other").write_text("")
        os.symlink(target, fd_dir / "3")
    (proc / "notapid").mkdir()

    holders = sweep.scan_port_holders(str(port), proc_root=str(proc), self_pid=1)

    assert [pid for pid, _ in holders] == [4242]


def test_scan_port_holders_ignores_our_own_fds(tmp_path):
    port = tmp_path / "ttyFAKE"
    port.write_text("")
    fd_dir = tmp_path / "proc" / "999" / "fd"
    fd_dir.mkdir(parents=True)
    os.symlink(port, fd_dir / "5")

    assert sweep.scan_port_holders(str(port), proc_root=str(tmp_path / "proc"), self_pid=999) == []


def test_scan_port_holders_returns_none_when_it_cannot_know(tmp_path):
    # No /proc means the exclusivity claim is unverifiable, and main() must refuse
    # rather than assume. An unverified single-authority claim is the failure this
    # whole measurement exists to avoid.
    assert sweep.scan_port_holders("/dev/ttyAMA0", proc_root=str(tmp_path / "absent")) is None


# ======================================================================================
# Thresholds derived from the instrument, not from a room
# ======================================================================================


def test_thresholds_rise_with_the_gyros_own_measured_noise():
    assert sweep.moving_threshold(0.0) == sweep.MOVING_RAD_S_FLOOR
    assert sweep.response_delta(0.0) == sweep.RESPONSE_DELTA_RAD_S_FLOOR
    # A noisy IMU raises its own bar instead of manufacturing motion.
    assert sweep.moving_threshold(0.10) == pytest.approx(0.50)
    assert sweep.response_delta(0.10) == pytest.approx(0.30)


# ======================================================================================
# The verdict, both directions
# ======================================================================================


def _step(index, duty, rate, **kwargs):
    return sweep.StepResult(
        index=index,
        duty=duty,
        mean_abs_yaw_rate=rate,
        peak_abs_yaw_rate=rate,
        sample_count=20,
        **kwargs,
    )


def test_verdict_reports_the_lowest_rotating_duty_when_responses_differ():
    steps = [
        _step(0, 32, 0.00),
        _step(1, 45, 0.01),
        _step(2, 70, 0.60),
        _step(3, 76, 0.95),
    ]

    verdict = sweep.evaluate(steps, noise_floor=0.0)

    assert verdict.status == "MOVING_DUTY_FOUND"
    assert verdict.moving_duty == 70


def test_verdict_is_invalid_when_adjacent_rotating_duties_read_the_same():
    # The run card's rule: the only evidence a duty took effect is a measurably
    # different achieved rate. Identical rates mean something upstream is clamping,
    # and then the apparent moving duty is not a measurement.
    steps = [
        _step(0, 32, 0.00),
        _step(1, 70, 0.60),
        _step(2, 76, 0.61),
    ]

    verdict = sweep.evaluate(steps, noise_floor=0.0)

    assert verdict.status == "SWEEP_INVALID"
    assert verdict.moving_duty is None
    assert "clamping" in verdict.reason


def test_verdict_is_invalid_when_a_rotating_step_has_no_confirming_neighbour():
    steps = [_step(0, 70, 0.60)]

    verdict = sweep.evaluate(steps, noise_floor=0.0)

    assert verdict.status == "SWEEP_INVALID"
    assert "no second rotating step" in verdict.reason


def test_verdict_says_nothing_rotated_rather_than_inventing_a_floor():
    steps = [_step(i, duty, 0.02, motor_writes=40) for i, duty in enumerate((32, 45, 70, 100))]

    verdict = sweep.evaluate(steps, noise_floor=0.0)

    assert verdict.status == "NO_ROTATION_IN_RANGE"
    assert verdict.moving_duty is None
    assert "160 motor packets written" in verdict.reason


def test_a_silent_sweep_with_no_packets_written_convicts_the_command_path_not_the_robot():
    # An all-zero table cannot distinguish a dead drivetrain from a dead command path.
    # The write counter is what tells them apart, and the verdict must say which.
    steps = [_step(i, duty, 0.0, motor_writes=0) for i, duty in enumerate((32, 45, 70, 100))]

    verdict = sweep.evaluate(steps, noise_floor=0.0)

    assert verdict.status == "NO_ROTATION_IN_RANGE"
    assert "dead COMMAND PATH" in verdict.reason
    assert "says nothing about the hardware" in verdict.reason


def test_verdict_uses_the_noise_derived_threshold_not_the_floor():
    steps = [_step(0, 70, 0.30), _step(1, 76, 0.31)]

    # With a quiet gyro 0.30 rad/s is rotation; with a gyro whose at-rest noise is
    # 0.10 rad/s the same reading is indistinguishable from noise.
    assert sweep.evaluate(steps, noise_floor=0.0).status != "NO_ROTATION_IN_RANGE"
    assert sweep.evaluate(steps, noise_floor=0.10).status == "NO_ROTATION_IN_RANGE"


def test_production_readout_names_the_ceiling_that_is_below_the_floor():
    verdict = sweep.evaluate([_step(0, 70, 0.6), _step(1, 76, 0.9)], noise_floor=0.0)

    text = sweep.verdict_for_production(verdict)

    assert text.count("CEILING IS BELOW THE FLOOR") == 2  # both 32 and 45 are below 70
    assert "45" in text and "32" in text


async def test_production_readout_reopens_the_autopsy_when_the_ceiling_is_adequate():
    verdict = sweep.evaluate([_step(0, 20, 0.6), _step(1, 23, 0.9)], noise_floor=0.0)

    text = sweep.verdict_for_production(verdict)

    assert "CEILING IS BELOW THE FLOOR" not in text
    assert "another cause" in text


# ======================================================================================
# The sweep itself, against a fake driver
# ======================================================================================


COUNTS_PER_METER = 4337.768
WHEEL_TRACK_M = 0.2507


@dataclass
class FakeImuSample:
    angular_velocity: tuple
    is_valid: bool = True


class FakeClock:
    def __init__(self):
        self.t = 0.0
        self.hooks = []

    def now(self):
        return self.t

    async def sleep(self, dt):
        self.t += float(dt)
        for hook in self.hooks:
            hook()


class FakeDriver:
    """Records commands, emits gyro samples on a tick, and advances fake encoders.

    Encoder counts are generated from the SAME rate the gyro reports, scaled through
    the production odometry kinematics, so a test that wants wheel/body disagreement
    has to ask for it explicitly.
    """

    def __init__(
        self,
        clock,
        response,
        *,
        battery=80,
        emit=True,
        stop_emitting_after_first_command=False,
        translation_m_per_tick=0.0,
        wheel_slip_factor=1.0,
        motor_fault=False,
        writes_reach_the_transport=True,
        spin_up_ticks=0,
    ):
        # Ticks a burst takes to reach its steady rate. The real drivetrain took ~0.4 s of
        # a 2 s burst on 2026-08-16, and that ramp is what made the steady-half gyro mean
        # and the whole-burst encoder rate incomparable. Zero means an instant step.
        self.spin_up_ticks = spin_up_ticks
        self._ticks_at_duty = 0
        self.writes_reach_the_transport = writes_reach_the_transport
        self.transport_writes = 0
        self.clock = clock
        self.response = response
        self.battery = battery
        self.emit = emit
        self.stop_emitting_after_first_command = stop_emitting_after_first_command
        self.translation_m_per_tick = translation_m_per_tick
        self.wheel_slip_factor = wheel_slip_factor
        self.motor_fault = motor_fault
        self.commands = []
        self.imu_callback = None
        self.imu_interval_ms = None
        self.left_counts = 1000
        self.right_counts = -1000
        self.last_duty = 0
        self.tick_dt = 0.05
        clock.hooks.append(self.tick)

    # -- the API the sweep is allowed to use ---------------------------------------

    async def get_battery_percentage(self):
        return self.battery

    def set_imu_callback(self, callback):
        self.imu_callback = callback

    async def enable_imu_streaming(self, interval_ms):
        self.imu_interval_ms = interval_ms

    async def disable_imu_streaming(self):
        self.imu_interval_ms = None

    async def drive_tank_normalized(self, left, right):
        self.commands.append((round(self.clock.now(), 4), left, right))
        if self.writes_reach_the_transport:
            self.transport_writes += 1
        if self.stop_emitting_after_first_command and (left or right):
            self.emit = False
        self.last_duty = right

    async def get_encoder_counts(self):
        return SimpleNamespace(left=self.left_counts, right=self.right_counts)

    def get_state(self):
        return SimpleNamespace(
            motor_stall_triggered=False,
            motor_fault=self.motor_fault,
            motor_transport_write_count=self.transport_writes,
        )

    async def set_velocity(self, *_args, **_kwargs):
        raise AssertionError(
            "the sweep must never command a RATE: the pivot path discards it and the "
            "closed-loop controller is the thing under test"
        )

    # -- the simulation -------------------------------------------------------------

    def tick(self):
        rate = self.response(abs(self.last_duty))
        if self.last_duty:
            self._ticks_at_duty += 1
            if self.spin_up_ticks:
                rate *= min(1.0, self._ticks_at_duty / float(self.spin_up_ticks))
        else:
            self._ticks_at_duty = 0
        if self.emit and self.imu_callback is not None:
            self.imu_callback(FakeImuSample(angular_velocity=(0.0, 0.0, rate)))
        wheel_rate = rate * self.wheel_slip_factor
        spin_counts = wheel_rate * WHEEL_TRACK_M * COUNTS_PER_METER * self.tick_dt / 2.0
        drive_counts = self.translation_m_per_tick * COUNTS_PER_METER
        if self.last_duty:
            self.left_counts += int(round(-spin_counts + drive_counts))
            self.right_counts += int(round(spin_counts + drive_counts))


def knee_at(duty_knee, slope=0.02, base=0.5):
    def response(duty):
        return 0.0 if duty < duty_knee else base + slope * (duty - duty_knee)

    return response


async def run_sweep(driver, clock, **config_kwargs):
    """Run the whole ladder on a fake clock.

    These stay native-async rather than calling `asyncio.run` from a sync test: on
    Python 3.9 `asyncio.run` leaves the main thread with no current event loop, and the
    next sync test that builds a FakeTransport dies on it. A test file that quietly
    breaks a different file's tests is its own kind of defect.
    """
    config = sweep.SweepConfig(**config_kwargs).validated()
    runner = sweep.SweepRunner(
        driver, config, now=clock.now, sleep=clock.sleep, out=lambda _line: None
    )
    return config, await runner.run()


async def test_sweep_finds_the_knee_and_stops_exactly_one_step_beyond():
    clock = FakeClock()
    driver = FakeDriver(clock, knee_at(70))

    _config, report = await run_sweep(driver, clock)

    duties = [step.duty for step in report.steps]
    assert duties[-2:] == [70, 76]
    assert 84 not in duties, "the ladder must not climb into the bog after confirmation"
    assert report.verdict.status == "MOVING_DUTY_FOUND"
    assert report.verdict.moving_duty == 70


async def test_sweep_commands_opposing_treads_and_stops_after_every_burst():
    clock = FakeClock()
    driver = FakeDriver(clock, knee_at(70))

    _config, report = await run_sweep(driver, clock)

    moving = [(left, right) for _t, left, right in driver.commands if right]
    assert moving, "no motor command was ever sent"
    for left, right in moving:
        assert left == -right, "an in-place pivot drives the treads in opposition"
        assert abs(right) <= sweep.TANK_FULL_SCALE
    assert driver.commands[-1][1:] == (0, 0), "the last packet of the run must be a stop"

    # Every burst must be STOPPED before the next duty is commanded. Bounded bursts with
    # a real cool-down between them are the safety envelope, not a formatting detail:
    # sustained sub-moving duty is the case that has powered this rover down.
    sequence = [(left, right) for _t, left, right in driver.commands]
    collapsed = [cmd for i, cmd in enumerate(sequence) if i == 0 or cmd != sequence[i - 1]]
    for current, following in zip(collapsed, collapsed[1:]):
        if current != (0, 0):
            assert following == (0, 0), (
                f"duty {current} was followed straight by {following} with no stop between"
            )
    commanded_duties = {right for _left, right in collapsed if right}
    assert commanded_duties == {step.duty for step in report.steps}


async def test_sweep_never_uses_the_rate_path():
    clock = FakeClock()
    driver = FakeDriver(clock, knee_at(70))

    await run_sweep(driver, clock)  # FakeDriver.set_velocity raises if it is ever called

    assert driver.imu_interval_ms is None or driver.imu_interval_ms >= 33


async def test_sweep_refuses_when_the_imu_stream_never_arrives():
    clock = FakeClock()
    driver = FakeDriver(clock, knee_at(70), emit=False)

    with pytest.raises(sweep.RefusalError) as excinfo:
        await run_sweep(driver, clock)

    assert "INSTRUMENT DEAD" in str(excinfo.value)
    assert not any(right for _t, _left, right in driver.commands), (
        "a dead instrument must be found BEFORE any duty is commanded"
    )


async def test_sweep_refuses_below_the_battery_floor_without_commanding_anything():
    clock = FakeClock()
    driver = FakeDriver(clock, knee_at(70), battery=24)

    with pytest.raises(sweep.RefusalError) as excinfo:
        await run_sweep(driver, clock)

    assert "24%" in str(excinfo.value)
    assert driver.commands == []


async def test_sweep_aborts_when_the_instrument_dies_mid_burst():
    clock = FakeClock()
    driver = FakeDriver(clock, knee_at(12), stop_emitting_after_first_command=True)

    _config, report = await run_sweep(driver, clock)

    assert report.aborted is not None
    assert "INSTRUMENT DEAD mid-sweep" in report.aborted
    assert driver.commands[-1][1:] == (0, 0), "an abort still stops the motors"


async def test_sweep_aborts_when_the_pivot_walks():
    clock = FakeClock()
    # 6 cm of straight-line travel across a 2 s burst: over the run card's 5 cm rule.
    driver = FakeDriver(clock, knee_at(12), translation_m_per_tick=0.06 / 40.0)

    _config, report = await run_sweep(driver, clock)

    assert report.aborted is not None
    assert "translated" in report.aborted
    # The burst was really measured, so it stays in the record; the abort is a statement
    # about safety, not a reason to discard a reading.
    assert len(report.steps) == 1
    assert report.steps[0].encoder_translation_m > sweep.TRANSLATION_ABORT_M
    assert 16 not in [step.duty for step in report.steps], "no step after the abort"


def _runner_with_interrupt(clock, driver, when):
    """Build a runner whose abort flag is set by `when(clock, driver)` on each tick."""
    config = sweep.SweepConfig().validated()
    runner = sweep.SweepRunner(
        driver, config, now=clock.now, sleep=clock.sleep, out=lambda _line: None
    )

    def hook():
        if when(clock, driver):
            runner.request_abort("operator sent SIGINT")

    clock.hooks.append(hook)
    return config, runner


async def test_an_interrupt_mid_burst_stops_the_motors_rather_than_unwinding():
    # THE CASE THAT MATTERS. A bare KeyboardInterrupt would unwind through the awaits and
    # leave the FIRMWARE HOLDING THE LAST DUTY -- the tank command latches, and this tool
    # is the only thing sending it. The signal must land as a flag the BURST loop reads,
    # so the wind-down goes through the ordinary stop.
    clock = FakeClock()
    driver = FakeDriver(clock, knee_at(70))
    _config, runner = _runner_with_interrupt(clock, driver, lambda _c, d: bool(d.last_duty))

    report = await runner.run()

    assert report.aborted == "operator sent SIGINT"
    assert driver.commands[-1][1:] == (0, 0), "an interrupted burst still stops the motors"
    interrupted_duty = {right for _t, _left, right in driver.commands if right}
    assert interrupted_duty == {12}, "it stopped inside the first burst, not after it"
    assert report.steps == [], "the interrupted burst was never measured, so it is not a step"


async def test_an_interrupt_during_the_cool_down_keeps_the_step_already_measured():
    clock = FakeClock()
    driver = FakeDriver(clock, knee_at(70))
    config, runner = _runner_with_interrupt(
        clock, driver, lambda c, d: c.now() > 6.0 and not d.last_duty
    )

    report = await runner.run()

    assert report.aborted == "operator sent SIGINT"
    assert report.steps, "steps measured before the interrupt are kept"
    # An interrupted run is not a lost run: the artifact still records what was measured.
    assert "ABORTED: operator sent SIGINT" in sweep.render_csv(config, report, "stamp")


async def test_sweep_records_a_clamped_drivetrain_as_invalid_rather_than_as_a_floor():
    clock = FakeClock()
    driver = FakeDriver(clock, knee_at(70, slope=0.0))  # same rate at every moving duty

    _config, report = await run_sweep(driver, clock)

    assert report.verdict.status == "SWEEP_INVALID"
    assert report.verdict.moving_duty is None


async def test_sweep_reports_no_rotation_when_nothing_in_the_ladder_moves():
    clock = FakeClock()
    driver = FakeDriver(clock, lambda _duty: 0.0)

    _config, report = await run_sweep(driver, clock)

    assert [step.duty for step in report.steps] == list(sweep.default_ladder())
    assert report.verdict.status == "NO_ROTATION_IN_RANGE"
    # The packets did go out, so this run is evidence about the drivetrain.
    assert all(step.motor_writes > 0 for step in report.steps)
    assert "did reach the wire" in report.verdict.reason


async def test_a_sweep_whose_packets_never_reach_the_transport_says_so():
    clock = FakeClock()
    driver = FakeDriver(clock, lambda _duty: 0.0, writes_reach_the_transport=False)

    _config, report = await run_sweep(driver, clock)

    assert all(step.motor_writes == 0 for step in report.steps)
    assert "dead COMMAND PATH" in report.verdict.reason


async def test_sweep_logs_encoder_yaw_alongside_the_gyro():
    clock = FakeClock()
    driver = FakeDriver(clock, knee_at(70))

    _config, report = await run_sweep(driver, clock)

    moving = [s for s in report.steps if s.mean_abs_yaw_rate > 0.1]
    assert moving, "expected the knee steps to rotate"
    for step in moving:
        assert step.encoder_yaw_rate is not None
        assert abs(abs(step.encoder_yaw_rate) - step.mean_abs_yaw_rate) < 0.10


async def test_wheel_body_disagreement_is_reported_when_the_wheels_slip():
    clock = FakeClock()
    # Wheels turn twice as fast as the body: the slip signature the gyro exists to catch.
    driver = FakeDriver(clock, knee_at(70), wheel_slip_factor=2.0)

    _config, report = await run_sweep(driver, clock)

    text = sweep.wheel_body_disagreement(report.steps)
    assert "DISAGREE" in text


async def test_wheel_body_agreement_is_reported_when_they_match():
    clock = FakeClock()
    driver = FakeDriver(clock, knee_at(70))

    _config, report = await run_sweep(driver, clock)

    assert "agree within" in sweep.wheel_body_disagreement(report.steps)


async def test_sweep_aborts_on_a_firmware_motor_fault():
    clock = FakeClock()
    driver = FakeDriver(clock, knee_at(12), motor_fault=True)

    _config, report = await run_sweep(driver, clock)

    assert report.aborted is not None
    assert "MOTOR FAULT" in report.aborted


async def test_baseline_measures_the_gyro_noise_with_the_motors_untouched():
    clock = FakeClock()
    driver = FakeDriver(clock, knee_at(70))
    config = sweep.SweepConfig().validated()
    runner = sweep.SweepRunner(
        driver, config, now=clock.now, sleep=clock.sleep, out=lambda _line: None
    )

    await runner.preflight()
    commands_before = list(driver.commands)
    await runner.baseline()

    assert driver.commands == commands_before, "the baseline must command nothing"
    assert runner.report.noise_floor == pytest.approx(0.0, abs=1e-9)


# ======================================================================================
# The artifact
# ======================================================================================


async def test_csv_header_carries_the_configuration_the_number_is_only_valid_under():
    clock = FakeClock()
    driver = FakeDriver(clock, knee_at(70), battery=41)
    config, report = await run_sweep(driver, clock)

    text = sweep.render_csv(config, report, "2026-08-16 03:00:00")
    header = [line for line in text.splitlines() if line.startswith("#")]
    joined = "\n".join(header)

    assert "battery_pct=41" in joined
    assert "burst_s=2.0" in joined and "settle_s=3.0" in joined
    assert "duties=" + ",".join(str(d) for d in config.duties) in joined
    assert "git_sha=" in joined
    assert "gyro_noise_rad_s=" in joined
    assert "verdict=MOVING_DUTY_FOUND" in joined

    body = [line for line in text.splitlines() if not line.startswith("#")]
    assert body[0].split(",") == sweep.CSV_COLUMNS
    assert len(body) > 1
    for row in body[1:]:
        assert len(row.split(",")) == len(sweep.CSV_COLUMNS)


async def test_csv_rows_cover_burst_and_settle_and_carry_both_duty_scales():
    clock = FakeClock()
    driver = FakeDriver(clock, knee_at(70))
    config, report = await run_sweep(driver, clock)

    text = sweep.render_csv(config, report, "stamp")
    rows = [line.split(",") for line in text.splitlines() if not line.startswith("#")][1:]
    phases = {row[sweep.CSV_COLUMNS.index("phase")] for row in rows}
    assert {"baseline", "burst", "settle"} <= phases

    for row in rows:
        duty = int(row[sweep.CSV_COLUMNS.index("duty_tank127")])
        raw = int(row[sweep.CSV_COLUMNS.index("duty_raw255_equiv")])
        assert raw == sweep.tank127_to_raw255(duty)


async def test_render_table_states_the_verdict_and_the_battery():
    clock = FakeClock()
    driver = FakeDriver(clock, knee_at(70), battery=37)
    _config, report = await run_sweep(driver, clock)

    text = sweep.render_table(report)

    assert "MOVING_DUTY_FOUND" in text
    assert "BATTERY = 37%" in text


# ======================================================================================
# Curve mode (--no-early-stop): mapping the production band the knee-finder cannot reach
#
# 2026-08-16 measured breakaway at tank 10-12, BELOW every deployed constant (23/28/32/45).
# The knee-finder stops one step past the knee, so no ordinary run can ever sample the
# band the config actually commands. Curve mode removes the early stop -- and because the
# early stop was also what kept the rover out of the bog, it is replaced by a hard cap.
# ======================================================================================


def test_curve_mode_is_off_unless_asked_for():
    assert sweep.SweepConfig().no_early_stop is False
    assert sweep.SweepConfig().validated().no_early_stop is False


def test_curve_mode_unlocks_the_production_band_the_knee_finder_refuses():
    band = (23, 28, 32, 45)

    config = sweep.SweepConfig(duties=band, no_early_stop=True).validated()
    assert config.duties == band

    # The very same ladder is refused by the knee-finder, twice over: it starts at the
    # production floor and it stops below the documented breakaway region. That is the
    # whole reason curve mode has to exist rather than being a different --duties string.
    with pytest.raises(sweep.RefusalError):
        sweep.SweepConfig(duties=band).validated()


def test_curve_mode_refuses_to_run_the_default_knee_finder_ladder():
    # The default ladder climbs to 100. With nothing stopping it at the knee, that is a
    # march through the bog, so curve mode will not accept a ladder by omission.
    with pytest.raises(sweep.RefusalError) as excinfo:
        sweep.SweepConfig(no_early_stop=True).validated()
    assert "requires an explicit --duties" in str(excinfo.value)


def test_curve_mode_refuses_to_climb_above_the_highest_deployed_duty():
    with pytest.raises(sweep.RefusalError) as excinfo:
        sweep.SweepConfig(duties=(23, 45, 70), no_early_stop=True).validated()
    assert str(sweep.CURVE_MODE_MAX_DUTY) in str(excinfo.value)


def test_the_curve_mode_cap_is_the_highest_duty_production_can_command():
    # If a deployed ceiling ever rises, the cap follows it -- and if someone raises the
    # cap on its own, this test says why that is a different decision.
    assert sweep.CURVE_MODE_MAX_DUTY == sweep.PRODUCTION_PIVOT_MAX_DUTY


def test_curve_mode_keeps_the_ordinary_ladder_bounds_that_are_not_about_the_knee():
    for duties in ((23, 200), (45, 23), (0, 23)):
        with pytest.raises(sweep.RefusalError):
            sweep.SweepConfig(duties=duties, no_early_stop=True).validated()


async def test_curve_mode_runs_every_rung_instead_of_stopping_past_the_knee():
    clock = FakeClock()
    driver = FakeDriver(clock, knee_at(12))

    _config, report = await run_sweep(
        driver, clock, duties=(10, 12, 16, 23, 28, 32, 45), no_early_stop=True
    )

    assert [step.duty for step in report.steps] == [10, 12, 16, 23, 28, 32, 45]
    rotating = [s for s in report.steps if s.mean_abs_yaw_rate > 0.1]
    assert len(rotating) > 2, "curve mode exists to produce a curve, not a knee"


async def test_the_knee_finder_still_stops_when_curve_mode_is_off():
    # The contrast test: same ladder, same drivetrain, early stop restored.
    clock = FakeClock()
    driver = FakeDriver(clock, knee_at(12))

    _config, report = await run_sweep(driver, clock, duties=(10, 12, 16, 23, 28, 32, 80))

    assert [step.duty for step in report.steps] == [10, 12, 16]


async def test_curve_mode_still_aborts_when_the_pivot_walks():
    # Removing the early stop must not remove the abort that would have caught the bog.
    clock = FakeClock()
    driver = FakeDriver(clock, knee_at(12), translation_m_per_tick=0.06 / 40.0)

    _config, report = await run_sweep(
        driver, clock, duties=(10, 12, 16, 23, 28, 32, 45), no_early_stop=True
    )

    assert report.aborted is not None and "walks" in report.aborted
    assert [step.duty for step in report.steps][-1] != 45, "the abort must end the ladder"


async def test_curve_mode_still_aborts_on_a_firmware_motor_fault():
    clock = FakeClock()
    driver = FakeDriver(clock, knee_at(12), motor_fault=True)

    _config, report = await run_sweep(
        driver, clock, duties=(10, 12, 16, 23, 45), no_early_stop=True
    )

    assert report.aborted is not None and "MOTOR FAULT" in report.aborted


def test_curve_mode_announces_itself_because_the_preamble_cannot(monkeypatch):
    # The safety preamble is printed before the arguments are parsed, so it always
    # describes the knee-finder. An operator who is told "it stops at the knee" and then
    # watches it not stop has been misled by the tool.
    monkeypatch.setattr(sweep, "scan_port_holders", lambda _port: [])
    # Stop before any hardware, without building a coroutine nothing will await.
    monkeypatch.setattr(sweep, "sweep_with_hardware", lambda *_a, **_k: None)
    monkeypatch.setattr(sweep, "asyncio", SimpleNamespace(run=lambda _c: sweep.EXIT_OK))
    lines = []

    sweep.main(["--arm", "--no-early-stop", "--duties", "23,28,45"], out=lines.append)

    assert any("CURVE MODE" in line and "does NOT stop at the knee" in line for line in lines)


# ======================================================================================
# The wheel-vs-body window (D32) -- the run-1 self-catch
#
# 2026-08-16 run 1 printed "DISAGREE by 0.212 rad/s ... slip". It was comparing the gyro's
# STEADY-HALF mean against an encoder rate differenced over the WHOLE burst. Recomputed on
# one window the two agreed to 0.08%. The bug was in the comparison, not the robot.
# ======================================================================================


async def test_step_records_the_whole_burst_gyro_as_well_as_the_steady_half():
    clock = FakeClock()
    driver = FakeDriver(clock, knee_at(70, base=1.2), spin_up_ticks=20)

    _config, report = await run_sweep(driver, clock)

    moving = [s for s in report.steps if s.mean_abs_yaw_rate > 0.1]
    assert moving
    for step in moving:
        assert step.full_burst_mean_abs_yaw_rate is not None
        # Spin-up drags the whole-burst mean below the steady half. If these are ever
        # equal the ramp is not being modelled and this file proves nothing.
        assert step.full_burst_mean_abs_yaw_rate < step.mean_abs_yaw_rate


async def test_a_spinning_up_burst_is_not_reported_as_slip():
    # THE REGRESSION. Wheels track the body exactly (slip factor 1.0); only the ramp
    # differs between the two windows. The old comparison called this slip.
    clock = FakeClock()
    driver = FakeDriver(clock, knee_at(70, base=1.2), spin_up_ticks=20)

    _config, report = await run_sweep(driver, clock)

    text = sweep.wheel_body_disagreement(report.steps)
    assert "agree within" in text, text
    assert "DISAGREE" not in text

    # And prove the artifact was real: comparing the steady half instead -- what the tool
    # used to do -- manufactures a gap far outside tolerance.
    worst = max(
        abs(abs(s.encoder_yaw_rate) - s.mean_abs_yaw_rate)
        for s in report.steps
        if s.encoder_yaw_rate is not None and s.mean_abs_yaw_rate > 0.1
    )
    assert worst > 0.10


async def test_real_slip_is_still_caught_when_the_burst_ramps():
    # The fix must not blind the detector: same ramp, but the wheels genuinely overrun.
    clock = FakeClock()
    driver = FakeDriver(
        clock, knee_at(70, base=1.2), spin_up_ticks=20, wheel_slip_factor=2.0
    )

    _config, report = await run_sweep(driver, clock)

    assert "DISAGREE" in sweep.wheel_body_disagreement(report.steps)


def test_wheel_body_says_nothing_rather_than_comparing_the_wrong_window():
    # A step from before the whole-burst field existed must be skipped, not silently
    # compared against the steady half again.
    step = _step(0, 70, 1.2, encoder_yaw_rate=0.9)
    assert step.full_burst_mean_abs_yaw_rate is None
    assert sweep.wheel_body_disagreement([step]) == ""


def test_wheel_body_disagreement_names_both_causes_and_claims_neither():
    step = _step(0, 70, 1.2, encoder_yaw_rate=0.4, full_burst_mean_abs_yaw_rate=1.2)
    text = sweep.wheel_body_disagreement([step])

    assert "DISAGREE" in text
    assert "slip" in text and "scale error" in text
    assert "whole-burst window" in text
    # The disclaimer is the point. Run 1 asserted "slip" from a number that could not
    # tell slip from a scale error, and the tool still cannot. It must say so.
    assert "does not distinguish" in text


async def test_csv_step_summary_carries_both_gyro_windows():
    clock = FakeClock()
    driver = FakeDriver(clock, knee_at(70, base=1.2), spin_up_ticks=20)
    config, report = await run_sweep(driver, clock)

    text = sweep.render_csv(config, report, "stamp")
    summary_header = [
        line for line in text.splitlines() if line.startswith("# step summary:")
    ][0]
    assert "full_burst_mean_abs_yaw_rate" in summary_header

    columns = summary_header.split(":", 1)[1].strip().split(",")
    summary_rows = [
        line[1:].strip().split(",")
        for line in text.splitlines()
        if line.startswith("# ") and line[2:3].isdigit()
    ]
    assert summary_rows, "expected per-step summary rows"
    for row in summary_rows:
        assert len(row) == len(columns)


async def test_csv_header_records_which_mode_produced_the_table():
    clock = FakeClock()
    driver = FakeDriver(clock, knee_at(12))
    config, report = await run_sweep(
        driver, clock, duties=(10, 12, 16, 23, 45), no_early_stop=True
    )

    text = sweep.render_csv(config, report, "stamp")

    assert "no_early_stop=1" in text and "curve mode" in text
