"""The escape seam's vocabulary — pinned, because it travels as a STRING.

docs/reverse_before_give_up_design.md §5. The outcome of a give-up escape rides in
`nav2_msgs/action/BackUp`'s free-text `error_msg`, because a borrowed message type
cannot be given new enum constants and a whole interface package for four words is
more machinery than the words are worth. That makes the prefix a contract between two
nodes, and a contract carried in a string is exactly the kind that rots silently — so
it is asserted here rather than hoped for.
"""

import pytest

from sphero_rvr_core.escape_outcome import (
    CLEARED, DECLINED, FROZEN, OUTCOMES, REFUSED, format_outcome, parse_outcome,
)


def test_every_outcome_survives_a_round_trip():
    for outcome in OUTCOMES:
        got, detail = parse_outcome(format_outcome(outcome, "0.31 m"))
        assert got == outcome
        assert detail == "0.31 m"


def test_an_outcome_with_no_detail_still_parses():
    assert parse_outcome(format_outcome(CLEARED)) == (CLEARED, "")


def test_an_unknown_message_is_not_coerced_into_a_known_outcome():
    """The load-bearing one. An `error_msg` this module does not understand means the
    other node is running code this one has not seen; guessing which outcome it
    "probably" meant is how a seam rots into a lie. It must come back as None."""
    for alien in ("", "backup failed", "cleared_ish: 0.3", "Collision Ahead",
                  "success", "aborted: whatever"):
        outcome, raw = parse_outcome(alien)
        assert outcome is None, f"{alien!r} was accepted as the outcome {outcome!r}"
        assert raw == alien.strip()


def test_a_typo_at_the_producer_is_caught_at_format_time():
    """Both halves of the seam import this module, so a bad word fails where it is
    written rather than at the consumer, days later, in a mission report."""
    with pytest.raises(ValueError):
        format_outcome("clered", "0.3 m")


def test_declined_is_part_of_the_vocabulary_and_distinct():
    """`declined` is a LOGIC ERROR (the explorer only asks while idle), which is
    exactly why it needs a word of its own rather than being folded into `refused`:
    one means the room is blocking us, the other means the two nodes disagree about
    who is driving, and they call for opposite responses."""
    assert DECLINED in OUTCOMES
    assert len({CLEARED, REFUSED, FROZEN, DECLINED}) == 4
