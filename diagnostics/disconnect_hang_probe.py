#!/usr/bin/env python3
"""Reproduce and locate the RVRDriver.disconnect() hang. Mac-side, no hardware.

WHY THIS EXISTS. D31 filed `test_driver_safety::
test_stale_velocity_command_causes_validated_raw_motor_off_packet` as "a SAFETY test
that flakes under CPU load", first sighted 2026-08-10 and re-sighted 2026-08-13. The
load framing was wrong, and it hid a real defect for three days.

Measured 2026-08-13 with this probe, idle Mac, 10 cores, Python 3.9.6:

    **6 hangs in 360 runs = 1.67%**, over six independent batches of 60
    (1, 2, 1, 1, 0, 1 -- so one batch of 60 saw none at all)
    healthy disconnect for comparison: 0.27-5.79 ms
    one early hang was given a 60 s bound and blew through it

    LOADED (24 busy processes on 10 cores): 0 hangs in 8 runs, disconnect 3-19 ms

**The hang is bimodal and NOT load-correlated** -- it appeared on the idle machine and
did not appear under saturation. 0.6 ms versus >60 s is not a scheduling distribution.

Note the batch of 60 that saw zero: at ~1.7%, a single probe run is not a reliable
detector and a clean one proves nothing. Run several hundred before believing an
absence -- which is exactly why this survived as "a flake" for three days.
The 1.0 s `asyncio.wait_for` around the teardown was converting an occasional hard hang
into something that looked exactly like a timing flake, and everyone (including the
register, and the session that first re-diagnosed it tonight) read the symptom as load
sensitivity because the first sighting happened to occur under load.

WHERE IT PARKS, from `--stacks`:

    disconnect()    driver.py:210   await self._control_task
    _control_loop() driver.py:670   await asyncio.sleep(self._control_period)

`disconnect()` calls `self._control_task.cancel()` and then awaits it. For the control
task to still be pending *at the top-of-loop sleep* after that cancel, the
CancelledError must have been delivered and then lost -- so the loop went round again
and parked in a sleep that nothing will ever interrupt. `disconnect()` awaits it with
no timeout of its own, so the process waits forever.

THE SUSPECTED MECHANISM, stated as a hypothesis and NOT acted on: `dispatcher.py:187`
is `return await asyncio.wait_for(future, timeout=timeout)`, and this host runs
**Python 3.9.6**. `asyncio.wait_for` in that era has a known cancellation race -- when
the inner future resolves in the same loop iteration as the outer cancellation,
`wait_for` consumes the CancelledError and returns the result normally, leaving the
enclosing task un-cancelled. The control loop reaches that call on every ack, so a
cancel landing in the same iteration as an ack would produce precisely this state, at
precisely this sort of rate.

That is a hypothesis with a matching signature, not a measurement. It is written down
so the next session starts from a candidate instead of from the symptom, and it MUST be
confirmed before anything is changed -- including on the Pi, whose Python version this
probe has not checked and which may not share the race.

NOT FIXED, DELIBERATELY. This is the driver's shutdown path: the thing that cancels the
control loop and sends the last stop. It was found at the end of a long session, under
a standing instruction to bank rather than wrestle safety-path work, and a fix wants
its own batch with a version check on both hosts, a revert-proof, and a reviewer. What
matters on the real robot is bounded: a hung `disconnect()` leaves the control task
alive and sleeping with `_desired_velocity` already cleared, so it commands nothing --
it fails to complete a shutdown rather than failing to stop the rover. Worth confirming
against the real transport, which this probe does not use.

CAPTURED 2026-08-16: THE "INTERMITTENT FLAKE" AND THE HANG ARE THE SAME DEFECT.

148 runs of the whole `tests/test_driver_safety.py` file, Mac, Python 3.9.6:

    11 HANGS   (7.4%) -- always at test #29, `test_stale_stop_transient_failure_
                          retries_safe_stop_without_fail_safe`, at its UNBOUNDED
                          `await driver.disconnect()`. Every hang log stops at
                          exactly 28 progress dots. Never any other test.
     3 FAILURES (2.0%) -- always test #12, `test_stale_velocity_command_causes_
                          validated_raw_motor_off_packet`, at line 354's
                          `await asyncio.wait_for(driver.disconnect(), timeout=1.0)`,
                          raising `asyncio.exceptions.TimeoutError`.

**Both faces are `disconnect()`.** The only difference is whether the call site bounds
it: 2 of the 19 `disconnect()` calls in that file use a 1.0 s `wait_for`, and those are
the ones that produce a red test instead of a hung process. This is precisely what the
paragraph above predicted, so the "flakes under CPU load" framing is now dead twice
over -- it is not load, and it is not a flake; it is this hang, seen through a timeout.

The captured traceback also shows the `wait_for` race directly:

    fut = <Task finished name='Task-78' coro=<RVRDriver.disconnect() done, ...> result=None>

`wait_for` raised TimeoutError against a task that had **already finished with
result=None**: the timeout fired while disconnect was still pending, and the coroutine
then completed during `_cancel_and_wait`, so `fut.result()` did not raise CancelledError
and Python 3.9 fell through to `raise exceptions.TimeoutError()` (tasks.py:496). Whether
disconnect would have returned on its own or only because the cancel unblocked it, this
log cannot say.

A SHARPER CLUE THAN "1.67% OF DISCONNECTS", offered as a hypothesis and NOT confirmed:
the rate is not uniform across call sites. Seventeen unbounded `disconnect()` calls in
that file, and exactly ONE ever hangs. The three tests that drive a short
`command_timeout` (the stale-command safe-stop path) are #12, #29 and #30 -- and #12 and
#29 are the two that misbehave. #30 is the odd one out and never misbehaved in 148 runs;
it is also the only one that ends with `_fail_safe_active` True, which makes the control
loop `continue` at the top and park in a plain `asyncio.sleep` rather than inside a
`wait_for` on a dispatcher round-trip. That fits the mechanism above: the cancel has to
land on a `wait_for`, not on a sleep, to get lost. **This is inference from which tests
do and do not misbehave -- the control loop's state at cancel time was never
instrumented.** Instrumenting it is the first move for whoever takes the fix.

REPRODUCTION (the whole file is required -- isolating test #29 gave 5/5 clean):

    for i in $(seq 1 150); do
      timeout 90 python3 -m pytest tests/test_driver_safety.py -p no:cacheprovider \
          -q --tb=long -rf > /tmp/d31.log 2>&1
      rc=$?; [ $rc -ne 0 ] && echo "run $i rc=$rc" && break
    done

Expect roughly 1 red run in 10; rc=124 is the hang, rc=1 is the same defect timing out.

USAGE
    python3 diagnostics/disconnect_hang_probe.py            # 40 runs, report the rate
    python3 diagnostics/disconnect_hang_probe.py 100        # more runs
    python3 diagnostics/disconnect_hang_probe.py 40 --stacks  # dump task stacks on hang
"""

import asyncio
import io
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from sphero_rvr_core.driver import RVRDriver          # noqa: E402
from sphero_rvr_core.fake_transport import FakeTransport  # noqa: E402
from sphero_rvr_core.packet import Packet             # noqa: E402

RAW_OFF = bytes([0, 0, 0, 0])
HANG_BOUND_S = 5.0


async def _one(dump_stacks: bool):
    """One connect / go-stale / disconnect cycle. Returns disconnect seconds, or None
    if it hung past HANG_BOUND_S."""
    transport = FakeTransport(auto_ack=True)
    driver = RVRDriver(
        transport=transport,
        control_period=0.01,
        command_timeout=0.03,
        velocity_control_mode=RVRDriver.VELOCITY_CONTROL_NATIVE_TANK_SI,
    )
    await driver.connect()
    await driver.set_velocity(linear_mps=0.2, angular_rad_s=0.0)

    loop = asyncio.get_running_loop()
    deadline = loop.time() + 5.0
    while not any(
        Packet.decode(raw).command_id == driver.commands.CID_RAW_MOTORS
        and Packet.decode(raw).payload == RAW_OFF
        for raw in transport.writes
    ):
        if loop.time() >= deadline:
            raise AssertionError("the stale-command stop was never sent")
        await asyncio.sleep(0.001)

    started = loop.time()
    task = asyncio.ensure_future(driver.disconnect())
    _done, pending = await asyncio.wait([task], timeout=HANG_BOUND_S)
    if pending:
        if dump_stacks:
            _dump()
        task.cancel()
        return None
    return loop.time() - started


def _dump():
    print("\n*** HANG — where every live task is parked ***")
    for task in asyncio.all_tasks():
        if task is asyncio.current_task():
            continue
        buf = io.StringIO()
        task.print_stack(file=buf)
        lines = [l.strip() for l in buf.getvalue().splitlines()
                 if "sphero_rvr_core" in l or "await" in l]
        print(f"  {task.get_name()}:")
        for line in lines[-4:]:
            print(f"      {line}")
    print()


async def main():
    runs = int(sys.argv[1]) if len(sys.argv) > 1 and sys.argv[1].isdigit() else 40
    dump = "--stacks" in sys.argv

    hangs, times = 0, []
    for i in range(runs):
        elapsed = await _one(dump)
        if elapsed is None:
            hangs += 1
            print(f"  run {i:3d}: HANG (>{HANG_BOUND_S:.0f} s)")
            if dump:
                break
        else:
            times.append(elapsed)

    print(f"\npython {sys.version.split()[0]}")
    print(f"hangs: {hangs} of {runs} ({100.0*hangs/runs:.1f}%)")
    if times:
        print(f"healthy disconnect: min {min(times)*1000:.2f} ms, "
              f"max {max(times)*1000:.2f} ms, n={len(times)}")
    print("\nA nonzero hang rate here is the D31 defect, not a flaky test.")


if __name__ == "__main__":
    asyncio.run(main())
