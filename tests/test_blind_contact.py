"""D48: the touch sense gets a consumer, and it is proven against a real contact.

The fixture is gauntlet mission 2's own driver status, 654 samples straight out of the
recorded bag (`tests/fixtures/mission2_driver_status.json`). It contains one real event:
Scott watched the rover push into a chair leg at 17:17:50, and the firmware said so.

A NOTE ON THE FIXTURE'S RESOLUTION, because it matters for what these tests prove: the bag
predates the stall COUNTER, so it carries only the 1 Hz flag. The counter is therefore
reconstructed from flag transitions, which UNDERCOUNTS -- any stall shorter than the
sampling interval is simply absent from this recording. That is the very deficiency the
counter exists to remove, so these tests demonstrate the decision on the coarsest input it
will ever see. A future mission's bag will carry `motor_stall_events` directly and the
reconstruction can be deleted.
"""

import json
from pathlib import Path

import pytest

from sphero_rvr_core.blind_contact import (
    DriverSample,
    evaluate,
    first_contact,
    windows,
)

FIXTURE = Path(__file__).parent / "fixtures" / "mission2_driver_status.json"


def _mission_samples():
    """Mission 2's driver status as DriverSamples, with the counter reconstructed."""
    raw = json.loads(FIXTURE.read_text())
    samples = []
    events = 0
    previous_stall = False
    for row in raw:
        if row["stall"] and not previous_stall:
            events += 1
        previous_stall = row["stall"]
        samples.append(
            DriverSample(
                t=row["t"],
                commanded_linear_mps=row["cmd_vx"],
                commanded_angular_rad_s=row["cmd_wz"],
                motor_transport_write_count=row["writes"],
                motor_stall_events=events,
                motor_stall_active=row["stall"],
                observed_linear_mps=row["odom_vx"],
                observed_angular_rad_s=row["odom_wz"],
            )
        )
    return samples


# ======================================================================================
# The real episode
# ======================================================================================


def test_the_fixture_is_the_recorded_mission_and_holds_exactly_one_stall():
    samples = _mission_samples()
    assert len(samples) == 654
    stalled = [s for s in samples if s.motor_stall_active]
    assert [s.t for s in stalled] == [219.0, 220.0], (
        "the fixture must still be mission 2's chair-leg episode"
    )
    assert samples[-1].motor_stall_events == 1


def test_it_fires_on_the_chair_leg_push():
    verdict = first_contact(_mission_samples(), span_s=2.0)

    assert verdict is not None, (
        "the one real contact in this mission must be detected -- this is D48's whole "
        "close criterion"
    )
    assert verdict.is_blind_contact
    assert verdict.stall_events >= 1
    assert verdict.motor_writes > 0
    assert "pushing on something it cannot see" in verdict.reason


def test_it_stays_quiet_across_every_window_that_is_not_the_contact():
    samples = _mission_samples()
    firing = [
        w for w in windows(samples, 2.0) if evaluate(w).is_blind_contact
    ]

    assert firing, "expected the contact itself to fire"
    # Every firing window must contain a sample the firmware actually flagged. A detector
    # that fires anywhere else in a 654-sample mission is crying wolf, and a touch sense
    # that cries wolf gets ignored exactly like the freeze classifier did.
    for window in firing:
        assert any(s.motor_stall_active for s in window), (
            f"window {window[0].t}-{window[-1].t} fired with no stall in it"
        )


def test_the_quiet_majority_of_the_mission_is_quiet():
    samples = _mission_samples()
    quiet = [s for s in samples if s.t < 200.0]
    assert first_contact(quiet, span_s=2.0) is None


# ======================================================================================
# The three parts of the decision, each with the wrong answer it prevents
# ======================================================================================


def _pair(*, writes_after=40, events_after=1, **overrides):
    """Two samples 2 s apart: the first at zero counters, the second at the given ones.

    `writes_after`/`events_after` are the DELTAS the verdict reads; everything else is
    passed straight to both samples. Keeping them out of the dataclass kwargs is the whole
    job of this helper -- the first version folded them in and every test using it died on
    an unexpected keyword.
    """
    base = dict(
        commanded_linear_mps=0.0,
        commanded_angular_rad_s=0.4,
        observed_linear_mps=0.0,
        observed_angular_rad_s=0.0,
        motor_stall_active=False,
    )
    base.update(overrides)
    first = DriverSample(
        t=0.0, motor_transport_write_count=0, motor_stall_events=0, **base
    )
    second = DriverSample(
        t=2.0,
        motor_transport_write_count=writes_after,
        motor_stall_events=events_after,
        **base,
    )
    return [first, second]


def test_a_stationary_robot_nobody_commanded_is_not_contact():
    window = _pair(commanded_angular_rad_s=0.0, writes_after=40, events_after=1)
    verdict = evaluate(window)
    assert not verdict.is_blind_contact
    assert "nothing was commanded" in verdict.reason


def test_a_dead_command_path_is_not_contact():
    # THE MISSION-1 TRAP. Commanded, stalled-looking, but no packets reached the wire.
    # Calling this contact would convict the room for the driver's silence.
    window = _pair(writes_after=0, events_after=1)
    verdict = evaluate(window)
    assert not verdict.is_blind_contact
    assert "dead command" in verdict.reason


def test_a_robot_that_moved_is_not_pinned():
    window = _pair(writes_after=40, events_after=1)
    window[1] = DriverSample(
        t=2.0,
        commanded_linear_mps=0.0,
        commanded_angular_rad_s=0.4,
        motor_transport_write_count=40,
        motor_stall_events=1,
        observed_angular_rad_s=2.9,
    )
    verdict = evaluate(window)
    assert not verdict.is_blind_contact
    assert "wheels turned" in verdict.reason


def test_no_stall_is_not_called_contact_on_absence_of_evidence():
    window = _pair(writes_after=40, events_after=0)
    verdict = evaluate(window)
    assert not verdict.is_blind_contact
    assert "no stall" in verdict.reason


def test_the_counter_catches_a_stall_the_flag_never_shows():
    # THE POINT OF D48'S CLOSE CRITERION. Both samples show the flag FALSE -- the stall
    # started and cleared between them -- and the counter still convicts. A level sampled
    # at 1 Hz cannot do this, and the one stall this project has recorded survived only
    # because it happened to last 2 s.
    window = _pair(writes_after=40, events_after=1)
    assert not any(s.motor_stall_active for s in window)
    assert evaluate(window).is_blind_contact


def test_one_sample_cannot_produce_a_verdict():
    single = _pair()[:1]
    assert not evaluate(single).is_blind_contact
    assert "two samples" in evaluate(single).reason


@pytest.mark.parametrize("span", [1.0, 2.0, 5.0])
def test_the_answer_does_not_depend_on_the_sampling_rate(span):
    # Deltas, not levels: doubling the sample density must not change the verdict.
    samples = _mission_samples()
    dense = []
    for a, b in zip(samples, samples[1:]):
        dense.append(a)
        dense.append(
            DriverSample(
                t=(a.t + b.t) / 2.0,
                commanded_linear_mps=a.commanded_linear_mps,
                commanded_angular_rad_s=a.commanded_angular_rad_s,
                motor_transport_write_count=a.motor_transport_write_count,
                motor_stall_events=a.motor_stall_events,
                motor_stall_active=a.motor_stall_active,
                observed_linear_mps=a.observed_linear_mps,
                observed_angular_rad_s=a.observed_angular_rad_s,
            )
        )
    assert (first_contact(samples, span) is not None) == (
        first_contact(dense, span) is not None
    )


# ======================================================================================
# The driver actually counts. Mutation testing found this file did not check it.
# ======================================================================================


def test_the_driver_counts_stall_transitions_not_levels():
    """`_handle_motor_stall` must COUNT, or the whole module above consumes a constant.

    Caught by mutation: deleting the increment in driver.py left every test in this file
    green, because they all exercised the pure decision and none touched the producer.
    """
    from types import SimpleNamespace

    from sphero_rvr_core.driver import RVRDriver
    from sphero_rvr_core.fake_transport import FakeTransport

    driver = RVRDriver(transport=FakeTransport(auto_ack=False))
    assert driver.get_state().motor_stall_events == 0

    # A stall begins: one event.
    driver._handle_motor_stall(SimpleNamespace(motor_index=0, is_triggered=True))
    assert driver.get_state().motor_stall_events == 1
    assert driver.get_state().motor_stall_triggered is True
    assert driver.get_state().last_motor_stall_epoch_s is not None

    # Still stalled: the LEVEL repeats, the COUNT must not.
    driver._handle_motor_stall(SimpleNamespace(motor_index=0, is_triggered=True))
    assert driver.get_state().motor_stall_events == 1

    # Cleared, then stalled again: a second event.
    driver._handle_motor_stall(SimpleNamespace(motor_index=0, is_triggered=False))
    assert driver.get_state().motor_stall_triggered is False
    assert driver.get_state().motor_stall_events == 1
    driver._handle_motor_stall(SimpleNamespace(motor_index=0, is_triggered=True))
    assert driver.get_state().motor_stall_events == 2


def test_the_counter_is_published_on_diagnostics():
    from sphero_rvr_driver.diagnostics import diagnostic_key_values
    from sphero_rvr_core.state import RVRState

    values = diagnostic_key_values(RVRState(motor_stall_events=7))
    assert values["motor_stall_events"] == "7", (
        "a counter nobody publishes is a counter nobody can consume -- D48's whole shape"
    )
