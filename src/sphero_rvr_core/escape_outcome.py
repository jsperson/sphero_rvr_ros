"""The vocabulary the controller and the explorer use to talk about ONE escape.

Design: docs/reverse_before_give_up_design.md §5.

The explorer asks the controller to back out of a place where nothing plans, and the
controller answers with a FACT about what happened. The fact travels in the action
result, and this module is the only definition of what those words mean, imported by
both sides, so the two cannot drift into disagreeing about the same string. That is
the same rule that produced the `_open_bearing` `None` fix: the owner publishes the
fact, and the seam has one definition, not two hopeful ones.

Why the words are packed into `error_msg` rather than a purpose-built field: the
transport is `nav2_msgs/action/BackUp`, whose goal is exactly the shape this escape
needs (a target point, a speed, a time allowance) and whose result carries `error_code`
plus a free-text `error_msg`. Adding an enum constant to somebody else's message type
is not possible, and adding an interface package to a pure-Python ament package to gain
four strings is more machinery than the four strings are worth. So the prefix in
`error_msg` IS the contract, and it is pinned by a test rather than by hope.
"""

CLEARED = "cleared"      # moved the distance asked for; the pose has actually changed
REFUSED = "refused"      # the supervisor zeroed the command: something is behind us
FROZEN = "frozen"        # permitted and immobile: blind contact BEHIND, mark planted
DECLINED = "declined"    # the controller was not idle -- see below, this is a bug

OUTCOMES = (CLEARED, REFUSED, FROZEN, DECLINED)

# `declined` is NOT an ordinary outcome. The explorer only ever asks while it is idle
# with no goal outstanding, so a decline means the two nodes disagree about which of
# them is driving. It is reported so it can be SEEN -- logged loudly, counted as a
# failed escape, never silently retried -- because a quiet retry here rebuilds the
# give-up livelock from the other side.


def format_outcome(outcome: str, detail: str = "") -> str:
    """`error_msg` for one escape: an outcome word, then human detail after a colon."""
    if outcome not in OUTCOMES:
        raise ValueError(f"unknown escape outcome {outcome!r}; expected {OUTCOMES}")
    return f"{outcome}: {detail}" if detail else outcome


def parse_outcome(error_msg: str):
    """(outcome, detail) from an `error_msg`, or (None, raw) if it is not ours.

    Unrecognised text is NEVER coerced into a known outcome. A message this module
    does not understand means the other side is running code this one has not seen,
    and guessing which outcome it "probably" meant is how a seam rots quietly.
    """
    raw = (error_msg or "").strip()
    head, _, detail = raw.partition(":")
    head = head.strip()
    if head in OUTCOMES:
        return head, detail.strip()
    return None, raw
