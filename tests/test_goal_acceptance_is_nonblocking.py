"""Goal ACCEPTANCE must never be able to block. Structural guard, from the
ack-starvation analysis (docs/frame_fix_handover.md §8).

WHAT THIS IS NOT. The analysis was assigned with a reproducing test attached: fake
action client, ladder busy, measure ack latency. That test is NOT here and could not
be written on this host — there is no rclpy on the Mac, so there is no real
`ActionServer`, no executor and no acknowledgement to time. A hand-rolled model of
`MultiThreadedExecutor` would only ever prove things about the model, which is the
one thing nobody needs to know.

What replaced it is a guard over the property the analysis actually established:
**`follow_path` acceptance is cheap and lock-free, and that is why it did not starve.**
The field evidence refuted the ranked candidate (the ladder does not starve the ack —
a 47 s continuous ladder episode in the same run produced zero timeouts), so the
useful thing to protect going forward is the reason acceptance is safe today, because
that is what a future change could take away.

THE PLAUSIBLE FUTURE REGRESSION IS RIGHT NEXT DOOR. `_escape_goal_callback` — the
other goal callback on this node — DOES take `self._goal_lock`, and `_execute` holds
that same lock. Today it is held only across a couple of assignments, so nothing
blocks. But the shape that candidate 1 describes ("the ladder holds a lock the
goal-acceptance path also needs") is one careless edit away from being true, and it
would present exactly as the symptom this analysis chased: an acknowledgement timeout
under load, blamed on the ladder, with the real cause a lock nobody meant to widen.
"""

import ast
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CONTROLLER = ROOT / "src" / "sphero_rvr_driver" / "decisive_controller_node.py"

BLOCKING_CALLS = {"sleep", "join", "wait", "acquire", "spin_until_future_complete",
                  "call", "lookup_transform", "wait_for_server", "get_result"}


def _func(name):
    tree = ast.parse(CONTROLLER.read_text())
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == name:
            return node
    raise AssertionError(f"{name} not found in {CONTROLLER.name}")


def _calls(node):
    out = set()
    for n in ast.walk(node):
        if isinstance(n, ast.Call):
            f = n.func
            out.add(f.attr if isinstance(f, ast.Attribute) else
                    f.id if isinstance(f, ast.Name) else "")
    return out


def test_follow_path_acceptance_takes_no_lock():
    """The acceptance path must not be able to queue behind the execute loop.

    `_execute` holds `self._goal_lock` while it swaps the active goal handle. If
    acceptance ever waits on that same lock, then a busy controller delays its own
    acknowledgements — the intra-node version of the defect the field evidence
    refuted, newly created by us.
    """
    fn = _func("_follow_path_goal_callback")
    withs = [n for n in ast.walk(fn) if isinstance(n, ast.With)]
    assert not withs, (
        "_follow_path_goal_callback now acquires something with a `with` block. "
        "Goal acceptance must stay lock-free: bt_navigator times out in ~1 s."
    )


def test_follow_path_acceptance_cannot_block_on_anything():
    """No sleeps, no waits, no service or TF calls. The whole callback is one bool
    read and a log line, and it must stay that cheap."""
    fn = _func("_follow_path_goal_callback")
    blocking = _calls(fn) & BLOCKING_CALLS
    assert not blocking, (
        f"_follow_path_goal_callback can now block on {sorted(blocking)}"
    )


def test_the_escape_acceptance_lock_stays_a_hair_thin_critical_section():
    """`_escape_goal_callback` legitimately reads shared state under `_goal_lock`.
    The requirement is that it does NOTHING ELSE in there — no logging, no publishing,
    no action calls — so the lock cannot be held while something slow happens.

    Pinned because this is the one place on the node where an acceptance path and the
    execute loop contend for the same lock, i.e. the only place candidate 1's shape
    could become real.
    """
    fn = _func("_escape_goal_callback")
    withs = [n for n in ast.walk(fn) if isinstance(n, ast.With)]
    assert len(withs) == 1, "expected exactly one locked region in _escape_goal_callback"

    inner = _calls(withs[0]) - {""}
    assert not inner, (
        f"the escape acceptance holds _goal_lock across calls to {sorted(inner)} — "
        "the critical section must stay a plain state read"
    )


def test_both_action_servers_are_on_the_reentrant_group():
    """The other half of the refutation, pinned so it cannot silently regress.

    Candidate 3 was "the callback-group assignment does not actually cover goal
    acceptance". It does: both servers are constructed with the reentrant group. If a
    future edit drops that kwarg, acceptance falls back to the node's DEFAULT
    mutually-exclusive group, where it would serialize behind every scan, odom and TF
    callback — and candidate 3 would become true after the fact.
    """
    tree = ast.parse(CONTROLLER.read_text())
    servers = [n for n in ast.walk(tree)
               if isinstance(n, ast.Call)
               and isinstance(n.func, ast.Name) and n.func.id == "ActionServer"]
    assert len(servers) == 2, f"expected 2 ActionServers, found {len(servers)}"

    for call in servers:
        groups = [kw for kw in call.keywords if kw.arg == "callback_group"]
        assert groups, "an ActionServer no longer names its callback_group"
        val = groups[0].value
        assert isinstance(val, ast.Attribute) and val.attr == "_callback_group", (
            "an ActionServer is on a different group than the reentrant one"
        )

    assigned = [n for n in ast.walk(tree)
                if isinstance(n, ast.Call) and isinstance(n.func, ast.Name)
                and n.func.id == "ReentrantCallbackGroup"]
    assert assigned, "_callback_group is no longer a ReentrantCallbackGroup"
