"""The preflight script's DISCIPLINES, which are testable off-robot even though its
gates are not.

The gates need ROS, an I2C sensor and a chassis, so their verdicts can only be earned
on the Pi. But the rules the script exists to encode -- kill by explicit PID never by
pattern, chassis-alive first, "could not tell" is not "fine" -- are structure and
arithmetic, and every one of them was bought with a session. They get checked here so
that the only thing left unverified on this machine is the part that genuinely needs
hardware.
"""

import os
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "preflight_pi.py"


@pytest.fixture(scope="module")
def preflight():
    sys.path.insert(0, str(ROOT / "scripts"))
    try:
        import preflight_pi
    finally:
        sys.path.pop(0)
    return preflight_pi


@pytest.fixture(scope="module")
def source():
    return SCRIPT.read_text()


def test_nothing_pattern_kills(source):
    """`pkill -f <pattern>` MATCHES YOUR OWN SSH COMMAND LINE and has killed the
    operator's session mid-teardown four times. A preflight is run over SSH by
    definition, so this is the one script where that trap is guaranteed to fire.

    Checks INVOCATIONS, not the word. The first version of this test failed on the
    script's own warning against pkill, which would have taught the next reader to
    delete the warning to get the suite green -- a test that punishes documenting a
    trap is worse than no test.
    """
    executing = ("sh(", "subprocess", "Popen", "os.system", "check_output")
    for lineno, line in enumerate(source.splitlines(), 1):
        for forbidden in ("pkill", "killall", "pgrep -f"):
            if forbidden in line and any(tok in line for tok in executing):
                pytest.fail(
                    f"line {lineno} EXECUTES {forbidden!r}: {line.strip()!r} -- it "
                    f"matches the operator's own ssh command line and has ended four "
                    f"sessions"
                )


def test_the_kill_list_can_never_contain_our_own_process(preflight):
    """The same trap in the form it would actually take here: a PID list that was not
    filtered. Handing this function our own PID must be a no-op, not suicide."""
    own = preflight._own_process_tree()
    assert os.getpid() in own, "our own PID must be in the exclusion set"

    targeted = preflight.kill_pids([os.getpid()])
    assert targeted == [], "the preflight tried to kill the process running it"

    # PID 1 and below are never targets either.
    assert preflight.kill_pids([0, 1]) == []


def test_the_exclusion_set_covers_ancestors_not_just_self(preflight):
    """The SSH session is the PARENT, so excluding only `getpid()` would still kill
    the shell the operator is sitting in."""
    own = preflight._own_process_tree()
    assert len(own) >= 2, f"only {own} excluded; ancestors are not being walked"
    assert os.getppid() in own


def test_chassis_alive_is_the_FIRST_gate_that_touches_hardware(preflight):
    """Order is the point of this script. The chassis being off is invisible from
    everywhere except the serial link, and odom, the SLAM anchor, the map and every
    goal cascade from it -- each failing in a way that looks like its own problem."""
    pre = [g.name for g in preflight.GATES if g.stage == "pre"]
    assert "chassis_alive" in pre
    hardware = [n for n in pre if n in ("chassis_alive", "lidar_not_occluded", "tof_state")]
    assert hardware[0] == "chassis_alive"
    # ...but the cheap checks that make its verdict trustworthy come first.
    assert pre.index("serial_port_free") < pre.index("chassis_alive"), (
        "probing the chassis while something else holds the port produces a false "
        "dead-chassis verdict"
    )


def test_the_probes_own_teardown_is_gated_immediately_after_it(preflight):
    """A previous version of this probe orphaned an rvr_node holding /dev/ttyAMA0,
    produced 874 'dispatcher reader failed' and no odom -- indistinguishable from a
    dead chassis. The probe nearly manufactured the symptom it detects, so the
    teardown is a GATE rather than an assumption."""
    pre = [g.name for g in preflight.GATES if g.stage == "pre"]
    assert pre.index("port_free_after_probe") == pre.index("chassis_alive") + 1


def test_the_port_is_the_authority_not_a_missing_pid(source):
    """`ros2 run` spawns the node as a CHILD; killing the wrapper leaves the node
    holding the device. So the verdict comes from fuser, not from a PID being gone."""
    assert "def port_holders" in source
    assert "fuser" in source
    body = source[source.index("def gate_port_free_after_probe"):
                  source.index("def gate_lidar_not_occluded")]
    assert "port_holders()" in body, (
        "the post-probe gate must ask the PORT, not check that a pid disappeared"
    )


def test_the_chassis_probe_signals_the_GROUP_not_the_wrapper(source):
    body = source[source.index("def gate_chassis_alive"):
                  source.index("def gate_port_free_after_probe")]
    assert "start_new_session=True" in body, (
        "without its own session the group signal would reach our own processes"
    )
    assert "killpg" in body, "killing the wrapper alone leaves the node on the port"


def test_the_graph_is_asked_of_ros_not_of_ps(source):
    """`ros2 node list` is the question; a node can be in `ps` and absent from the
    graph, and vice versa. Trusting `ps` has produced wrong verdicts before."""
    body = source[source.index("def ros_nodes"):source.index("def topic_once")]
    assert "ros2 node list" in body
    assert " ps " not in body and "'ps'" not in body


def test_scan_reads_are_not_truncated(source):
    """`ros2 topic echo` TRUNCATES arrays, and a truncated scan cannot answer an
    occlusion question."""
    body = source[source.index("def topic_once"):source.index("# ----", source.index("def topic_once"))]
    assert "--full-length" in body


def test_the_tof_gate_reads_the_state_line_and_not_the_yaml(source):
    """`/tof/state` once said `rules=rule_a_only` with the pinned margin sitting in
    the config and every test green -- one gate away from flying the first rule-B
    mission with rule B off."""
    body = source[source.index("def gate_tof_state"):
                  source.index("def gate_supervisor_latch_cleared")]
    assert "/tof/state" in body
    for yaml_ish in ("yaml", "collision_stop.yaml", "get_parameter"):
        assert yaml_ish not in body, (
            f"the ToF gate consults {yaml_ish}; it must gate on the sensor's own words"
        )


def test_clear_estop_is_called_BEFORE_reset(source):
    """An ordering written down nowhere and discovered live. `reset` refuses while the
    estop is latched, producing a plausible failure that sends the operator looking in
    the wrong place."""
    body = source[source.index("def gate_supervisor_latch_cleared"):]
    body = body[:body.index("# ----")]
    assert body.index("clear_estop") < body.index("/collision_stop/reset")


def test_UNKNOWN_exits_non_zero_because_could_not_tell_is_not_fine(preflight, monkeypatch):
    """A preflight that shrugs is a preflight that gets believed. Every gate that
    could not be evaluated must block bringup exactly like a failure."""
    unknown = preflight.Gate(
        "synthetic", "pre", lambda: preflight.Result(preflight.UNKNOWN, "cannot tell"))
    monkeypatch.setattr(preflight, "GATES", [unknown])
    assert preflight.main(["--stage", "pre"]) == 1


def test_a_clean_stage_exits_zero(preflight, monkeypatch):
    ok = preflight.Gate("synthetic", "pre",
                        lambda: preflight.Result(preflight.PASS, "fine"))
    monkeypatch.setattr(preflight, "GATES", [ok])
    assert preflight.main(["--stage", "pre"]) == 0


def test_a_raising_gate_becomes_UNKNOWN_rather_than_crashing_the_run(preflight, monkeypatch):
    """One broken gate must not cost the operator the other seven verdicts."""
    def boom():
        raise RuntimeError("gate exploded")

    monkeypatch.setattr(preflight, "GATES", [
        preflight.Gate("boom", "pre", boom),
        preflight.Gate("fine", "pre", lambda: preflight.Result(preflight.PASS, "ok")),
    ])
    assert preflight.main(["--stage", "pre"]) == 1
    assert preflight.GATES[1].result.verdict == preflight.PASS


def test_every_failure_path_tells_the_operator_what_to_do(preflight):
    """A gate that fails without a remedy is a gate ignored on the third run."""
    for gate in preflight.GATES:
        src = gate.run.__doc__ or ""
        assert src.strip(), f"{gate.name} has no docstring explaining itself"
        assert gate.why, f"{gate.name} has no `why`"


def test_an_installed_file_with_no_source_is_caught_as_an_orphan(preflight, monkeypatch, tmp_path):
    """THE ORPHAN, PLANTED AND CAUGHT.

    `colcon build` copies source->install and never deletes, so every return from a
    branch that added a module leaves that module behind. The file-by-file comparison
    walks SOURCE files and checks each has a twin -- a file that exists only in the
    install tree is structurally invisible to it, and the gate said PASS over exactly
    that on 2026-08-31, twice, for the same file.

    This plants a fake orphan and asserts the gate now fails with a remedy that names
    the cause. Without it the check would be prose in a docstring.
    """
    src = tmp_path / "src" / "sphero_rvr_ros"
    (src / "src" / "sphero_rvr_core").mkdir(parents=True)
    (src / "src" / "sphero_rvr_core" / "real.py").write_text("x = 1\n", encoding="utf-8")
    install = tmp_path / "install" / "sphero_rvr_core"
    install.mkdir(parents=True)
    (install / "real.py").write_text("x = 1\n", encoding="utf-8")

    monkeypatch.setattr(preflight, "SRC_TREE", src)
    monkeypatch.setattr(preflight, "INSTALL_TREE", tmp_path / "install")
    monkeypatch.setattr(preflight, "_deployed_roots", lambda: [tmp_path / "install"])

    clean = preflight.gate_installed_tree_matches()
    assert clean.verdict == preflight.PASS, (
        f"a matching tree did not pass: {clean.detail} -- the orphan check must not "
        f"fire on a healthy install")

    # now the orphan: present in install, absent from source
    (install / "ghost.py").write_text("gone_from_source = True\n", encoding="utf-8")
    dirty = preflight.gate_installed_tree_matches()
    assert dirty.verdict == preflight.FAIL, "the planted orphan was not caught"
    assert "ghost.py" in dirty.detail, f"the orphan was not named: {dirty.detail}"
    assert "no source" in dirty.detail
    assert dirty.remedy and "colcon never deletes" in dirty.remedy, (
        "the remedy must name the cause, or the next person re-runs the build that "
        "cannot fix it")


def test_a_borrowed_file_left_staged_is_caught_and_ordinary_dirt_is_not(preflight, monkeypatch, tmp_path):
    """THE BORROW, PLANTED AND CAUGHT -- and the near-miss it comes from.

    `git checkout <commit> -- <path>` WRITES THE INDEX, so the reflexive restore
    `git checkout -- <path>` reads from the index and hands the borrowed version back.
    On 2026-08-31 that left the pre-fix version of a test file on the Pi while every
    command reported success, one step before a ten-run measurement of that very file.

    The gate must catch the borrow WITHOUT firing on ordinary unstaged edits, or it
    becomes the check everyone skips. Both halves are asserted here, because a gate that
    fails on everything is the same as a gate that fails on nothing.
    """
    import subprocess

    repo = tmp_path / "repo"
    repo.mkdir()

    def git(*args):
        return subprocess.run(["git", "-C", str(repo), *args],
                              capture_output=True, text=True, check=True)

    git("init", "-q", ".")
    git("config", "user.email", "t@t")
    git("config", "user.name", "t")
    (repo / "f.py").write_text("VALUE = 'old'\n", encoding="utf-8")
    git("add", "-A")
    git("commit", "-qm", "one")
    old = git("rev-parse", "--short", "HEAD").stdout.strip()
    (repo / "f.py").write_text("VALUE = 'new'\n", encoding="utf-8")
    git("add", "-A")
    git("commit", "-qm", "two")

    monkeypatch.setattr(preflight, "SRC_TREE", repo)

    clean = preflight.gate_worktree_is_head()
    assert clean.verdict == preflight.PASS, (
        f"a clean checkout did not pass: {clean.detail}")

    # ORDINARY DIRT must NOT fire the gate: it is ' M' in porcelain, not 'M '.
    (repo / "f.py").write_text("VALUE = 'edited by hand'\n", encoding="utf-8")
    dirty = preflight.gate_worktree_is_head()
    assert dirty.verdict == preflight.PASS, (
        f"the gate fired on an ordinary unstaged edit ({dirty.detail}) -- a gate that "
        f"fails on everything is skipped, and then it catches nothing")
    git("checkout", "--", "f.py")

    # THE BORROW: the file is now the OLD version, staged, worktree clean.
    git("checkout", old, "--", "f.py")
    assert (repo / "f.py").read_text(encoding="utf-8") == "VALUE = 'old'\n"
    borrowed = preflight.gate_worktree_is_head()
    assert borrowed.verdict == preflight.FAIL, (
        "the planted borrow was not caught -- this is the exact state that nearly got "
        "measured as if it were HEAD")
    assert "f.py" in borrowed.detail, f"the borrowed file was not named: {borrowed.detail}"
    assert borrowed.remedy and "git checkout HEAD --" in borrowed.remedy, (
        "the remedy must name the restore that actually works, because the obvious one "
        "is the one that silently does nothing")

    # AND THE REFLEXIVE RESTORE MUST NOT CLEAR IT -- the whole point of the trap.
    git("checkout", "--", "f.py")
    assert (repo / "f.py").read_text(encoding="utf-8") == "VALUE = 'old'\n", (
        "`git checkout -- <path>` restored from HEAD; if git ever changes this, the "
        "gate is still correct but this norm needs rewriting")
    assert preflight.gate_worktree_is_head().verdict == preflight.FAIL

    # the remedy the gate prints must be the one that works
    git("checkout", "HEAD", "--", "f.py")
    assert (repo / "f.py").read_text(encoding="utf-8") == "VALUE = 'new'\n"
    assert preflight.gate_worktree_is_head().verdict == preflight.PASS


def test_the_source_check_runs_BEFORE_the_install_check(preflight):
    """The chain is what proves anything, and the chain is the list order.

    `worktree_is_head` says the source tree is HEAD. `installed_tree_matches` says the
    install tree is the source tree, in both directions (its orphan scan is the return
    half). Neither alone says the deployed artifact derives from the SHA -- the Pi runs
    `install/`, not `src/`, so the first gate on its own proves a fact about a tree that
    is not the one under test.

    Run in the other order the pair still passes and still means less, which is exactly
    the kind of thing that survives a review. Asserted rather than assumed, and named by
    worker sphero-rvr-ros-4f, who asked whether anything held the two together.
    """
    names = [g.name for g in preflight.GATES if g.stage == "pre"]
    assert "worktree_is_head" in names and "installed_tree_matches" in names
    assert names.index("worktree_is_head") < names.index("installed_tree_matches"), (
        "the source check must precede the install check, or the pair proves "
        "install == source == some-tree rather than install == source == HEAD")
