"""Blind contact: the robot was told to move, the driver wrote packets, and it stalled.

D48. On 2026-08-16 the freeze classifier logged *"an obstacle no sensor on this robot can
see"* while `motor_stall` sat on `/diagnostics` saying otherwise. **The robot's touch sense
existed and was published, and nothing consumed it.** This module is the consumer.

The decision is deliberately three-part, and each part exists because leaving it out
produces a specific wrong answer this project has already made:

  1. **commanded** -- the stack asked for motion. Without this, a stationary robot with a
     hand on its wheel reads as contact.
  2. **the driver actually wrote motor packets** -- the write counter advanced. Without
     this, a dead command path reads as contact, which is the mistake mission 1's autopsy
     made in the other direction: it convicted an actuator ceiling on evidence that could
     not distinguish "did not move" from "was never told".
  3. **the firmware reported a stall** -- by COUNTER, not by level. Diagnostics publish the
     driver's status at 1 Hz, so a stall that starts and clears inside a second never
     appears in the flag. A monotonic counter cannot miss one however brief.

`motion_observed` is the falsifier: if the wheels turned, whatever happened was not a
robot pinned against something.

WHAT THIS MODULE IS NOT: a BT node. Nav2's behaviour-tree conditions are C++ plugins
(BehaviorTree.CPP) and this package is pure Python with no ament_cmake target, so the
plugin's packaging is a separate decision -- see `docs/blind_contact_bt_node_TODO.md`.
This is the decision itself, in the layer that owns the facts, testable against recorded
missions with no robot present.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

#: Below this the command is not a request to move. Matches the driver's own idea of a
#: zero command rather than inventing a second one.
COMMANDED_EPSILON = 1e-6

#: Wheel motion above this counts as "it moved", so not pinned. Deliberately small: the
#: chair-leg episode moved 1.49 cm in 7.76 s while being commanded at 0.4 rad/s.
MOTION_EPSILON_MPS = 0.01
MOTION_EPSILON_RAD_S = 0.05


@dataclass(frozen=True)
class DriverSample:
    """One observation of the driver's own account of itself.

    Field for field, this is what `/diagnostics` already carries -- no new plumbing, and
    nothing inferred across a seam.
    """

    t: float
    commanded_linear_mps: float
    commanded_angular_rad_s: float
    motor_transport_write_count: int
    motor_stall_events: int
    motor_stall_active: bool = False
    observed_linear_mps: float = 0.0
    observed_angular_rad_s: float = 0.0


@dataclass(frozen=True)
class BlindContactVerdict:
    is_blind_contact: bool
    reason: str
    stall_events: int = 0
    motor_writes: int = 0
    commanded_s: float = 0.0


def _commanded(sample: DriverSample) -> bool:
    return (
        abs(sample.commanded_linear_mps) > COMMANDED_EPSILON
        or abs(sample.commanded_angular_rad_s) > COMMANDED_EPSILON
    )


def _moved(sample: DriverSample) -> bool:
    return (
        abs(sample.observed_linear_mps) > MOTION_EPSILON_MPS
        or abs(sample.observed_angular_rad_s) > MOTION_EPSILON_RAD_S
    )


def evaluate(window) -> BlindContactVerdict:
    """Decide whether a window of driver samples shows blind contact.

    ``window`` is any iterable of :class:`DriverSample` in time order. Two samples are
    enough; the counters are read as deltas across the window, so the answer does not
    depend on the sampling rate -- which is the entire point.
    """
    samples = list(window)
    if len(samples) < 2:
        return BlindContactVerdict(False, "need at least two samples to read a delta")

    first, last = samples[0], samples[-1]
    commanded_s = sum(
        b.t - a.t for a, b in zip(samples, samples[1:]) if _commanded(a)
    )
    stall_events = last.motor_stall_events - first.motor_stall_events
    writes = last.motor_transport_write_count - first.motor_transport_write_count
    stalled = stall_events > 0 or any(s.motor_stall_active for s in samples)

    if not commanded_s:
        return BlindContactVerdict(
            False, "nothing was commanded in this window", stall_events, writes
        )
    if writes <= 0:
        # THE MISSION-1 TRAP, refused explicitly. No packets reached the wire, so the
        # robot was never actually told anything -- whatever this is, it is a command-path
        # fault and calling it contact would convict the room for the driver's silence.
        return BlindContactVerdict(
            False,
            "commanded, but the driver wrote NO motor packets -- this is a dead command "
            "path, not contact",
            stall_events,
            writes,
            commanded_s,
        )
    if any(_moved(s) for s in samples):
        return BlindContactVerdict(
            False,
            "the wheels turned, so the robot was not pinned",
            stall_events,
            writes,
            commanded_s,
        )
    if not stalled:
        return BlindContactVerdict(
            False,
            "commanded and written but the firmware reported no stall -- unexplained, and "
            "NOT to be called contact on absence of evidence",
            stall_events,
            writes,
            commanded_s,
        )
    return BlindContactVerdict(
        True,
        f"commanded for {commanded_s:.2f} s, {writes} motor packets written, "
        f"{stall_events} stall event(s) reported, no wheel motion -- the robot is pushing "
        "on something it cannot see",
        stall_events,
        writes,
        commanded_s,
    )


def windows(samples, span_s: float):
    """Yield consecutive windows spanning at least ``span_s`` of sample time."""
    samples = list(samples)
    start = 0
    for end in range(1, len(samples)):
        while samples[end].t - samples[start].t > span_s and start + 1 < end:
            start += 1
        if samples[end].t - samples[start].t >= span_s:
            yield samples[start : end + 1]


def first_contact(samples, span_s: float = 2.0) -> Optional[BlindContactVerdict]:
    """The first window in ``samples`` that reads as blind contact, or None."""
    for window in windows(samples, span_s):
        verdict = evaluate(window)
        if verdict.is_blind_contact:
            return verdict
    return None
