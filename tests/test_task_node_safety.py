"""The task node's safety boundary, asserted by scanning its source.

SEPARATE FILE ON PURPOSE. The rclpy harness (`test_task_node.py`) skips wholesale on
a machine without ROS, and `pytest.importorskip` skips the entire module regardless
of where it sits -- so a boundary test living there would silently not run on the
Mac, which is where most commits are made. The one property of this node that must
never regress unnoticed is the one test that needs no ROS at all, so it lives here
and runs everywhere.

Lifted from the culled prompt-drive suite (`c5e87d2~1:tests/test_prompt_drive.py:
407-419`), retargeted at `task_node.py`. There it proved the NL executor could only
publish a route-request string; here it proves the task surface can only ask Nav2 for
a pose. Same job: keep a tool surface from becoming a second controller, which is
this stack's documented way to make a control bug look like a perception bug.
"""

from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
TASK_NODE = REPO_ROOT / "src" / "sphero_rvr_driver" / "task_node.py"
TASK_CLIENT = REPO_ROOT / "src" / "sphero_rvr_driver" / "task_client.py"
EXPLORER = REPO_ROOT / "src" / "sphero_rvr_driver" / "coverage_explorer_node.py"


def _executable_body(text: str) -> str:
    """The source minus comments and the module docstring.

    The docstring NAMES the forbidden symbols in order to explain the boundary, and
    a scan that cannot tell prose from code would either fail on the explanation or
    force the explanation out of the file. Comments are stripped for the same reason.
    """
    body = text.split('"""', 2)[-1]
    return "\n".join(
        line for line in body.splitlines() if not line.strip().startswith("#")
    )


def test_task_node_never_touches_velocity_or_motor_topics():
    body = _executable_body(TASK_NODE.read_text())
    assert "Twist" not in body
    assert "from geometry_msgs" not in body
    assert "import geometry_msgs" not in body
    assert "/cmd_vel" not in body
    assert "cmd_vel_motor" not in body
    assert "Serial" not in body


def test_task_node_publishes_nothing_at_all():
    """It is a client and a server, never a publisher. Nothing it can do reaches a
    topic that anything downstream drives on."""
    assert "create_publisher" not in _executable_body(TASK_NODE.read_text())


def test_task_node_is_wired_as_a_real_entry_point():
    """A boundary on code nobody can run proves nothing."""
    setup = (REPO_ROOT / "setup.py").read_text()
    assert "task_node = sphero_rvr_driver.task_node:main" in setup


def test_task_node_exposes_exactly_the_NINE_tools_and_no_more():
    """A surface that quietly grows is how a thin node stops being thin, so the tool
    set is a WHITELIST and adding to it is a deliberate edit here.

    WIDENED 3 -> 6 on 2026-08-14 for Track 2 v2 (`explore`, `stop`, `status`).
    WIDENED 6 -> 9 on 2026-08-20 for the LLM-verb bridge round 1, PM-ratified
    (design_llm_verb_bridge_2026-08-20): `turn` (a client of the supervisor's
    precise-turn gateway -- admission stays the safety layer), `where_am_i`
    (owner facts, read-only), `look_and_recognize` (LANDS DISABLED behind the
    recognition bench card; the enable flag flips in its own reviewed diff).
    Widened rather than relaxed, both times; a TENTH tool, whatever it is,
    fails this test.

    Still out of scope and still asserted: `set_search_classes`, `capture_photo`, and
    anything that would give this surface its own motion authority. An e-stop tool in
    particular is NOT here -- it answers to the collision supervisor, not to the task
    layer, and it needs its own review rather than an entry in a list.
    """
    source = TASK_NODE.read_text()
    tools = ("task/goto", "task/observe", "task/query_semantic_map",
             "task/explore", "task/stop", "task/status",
             "task/turn", "task/where_am_i", "task/look_and_recognize")
    for tool in tools:
        assert f'"{tool}"' in source, f"{tool} is no longer exposed"

    body = _executable_body(source)
    import re
    declared = set(re.findall(r'"(task/[a-z_]+)"', body))
    assert declared == set(tools), (
        f"the tool surface is {sorted(declared)}, not {sorted(tools)}. Adding a tool "
        "is a decision, not an implementation detail -- name it here or remove it")

    for forbidden in ("set_search_classes", "capture_photo", "task/estop",
                      "task/emergency_stop"):
        assert forbidden not in body, (
            f"{forbidden!r} appeared in the tool surface. An emergency stop answers "
            "to the collision supervisor and does not belong on this node without "
            "its own review")


def test_task_node_carries_no_transport_or_model_layer():
    """v1 is a ROS surface. No MCP, no HTTP, no provider SDK -- those are a client's
    business, and the acceptance test is that deleting the client leaves a working
    robot."""
    body = _executable_body(TASK_NODE.read_text())
    for forbidden in ("import requests", "http", "socket", "openai", "anthropic",
                      "mcp", "stdio"):
        assert forbidden not in body.lower()


# --- the same boundary, applied to the LLM client ---------------------------
# The client is where a language model's output first becomes an action, so it gets
# the identical scan. A model cannot ask for a velocity if nothing in the path can
# express one.

def test_task_client_never_touches_velocity_or_motor_topics():
    body = _executable_body(TASK_CLIENT.read_text())
    assert "Twist" not in body
    assert "from geometry_msgs" not in body
    assert "import geometry_msgs" not in body
    assert "/cmd_vel" not in body
    assert "cmd_vel_motor" not in body
    assert "Serial" not in body


def test_task_client_publishes_nothing():
    assert "create_publisher" not in _executable_body(TASK_CLIENT.read_text())


def test_task_client_only_reaches_the_task_tool_interfaces():
    """It may talk to task_node and nothing else. A client that could call
    /navigate_to_pose directly would bypass the envelope entirely — and the
    bridge round 1 additions widen the surface it must NOT touch: the
    precise-turn gateway and the recognition node are task_node's to call,
    never the client's (the same envelope-bypass class, new doors)."""
    body = _executable_body(TASK_CLIENT.read_text())
    assert '"task/goto"' in body
    assert '"task/observe"' in body
    assert '"task/query_semantic_map"' in body
    assert '"task/turn"' in body
    assert '"task/look_and_recognize"' in body
    # The bare interfaces are the envelope bypass; only task_node may use them.
    assert '"navigate_to_pose"' not in body
    assert '"compute_path_to_pose"' not in body
    assert '"/observe"' not in body
    assert "collision_stop/precise_turn" not in body
    assert "/recognition/" not in body


def test_the_recognition_tool_lands_disabled_behind_the_bench():
    """The watcher-flip pattern, applied: the flag defaults FALSE, the refusal
    cites the bench card, and the flip is a reviewed one-line diff when the
    card passes — never an ambient enable."""
    node_body = TASK_NODE.read_text()
    assert '"recognition_tool_enabled", False' in node_body
    assert "bench_card_recognition_2026-08-19" in node_body


def test_turn_reaches_only_the_gateway():
    """task/turn is a CLIENT of the supervisor's precise-turn gateway and of
    nothing else motor-capable: the admission stays the safety layer, and the
    node's own re-check of the sanity bound is asserted present (the schema is
    a convenience; the node is the boundary)."""
    node_body = TASK_NODE.read_text()
    assert '"/collision_stop/precise_turn"' in node_body
    assert "-180.0 <= degrees <= 180.0" in node_body


def test_task_client_is_removable_and_nothing_depends_on_it():
    """Stage D acceptance, mechanically: delete the client and the robot is
    unchanged. Nothing in the driver package may import it, and no launch file may
    start it."""
    driver = REPO_ROOT / "src" / "sphero_rvr_driver"
    for path in driver.glob("*.py"):
        if path.name == "task_client.py":
            continue
        assert "task_client" not in path.read_text(), (
            f"{path.name} imports the LLM client — it must be removable"
        )
    for launch in (REPO_ROOT / "launch").glob("*.py"):
        assert "task_client" not in launch.read_text(), (
            f"{launch.name} starts the LLM client — it must be opt-in and removable"
        )


def test_task_client_carries_no_provider_sdk():
    """One provider, reached through the repo's existing helper. A vendor SDK here
    would be a new dependency and a second way to hold a key."""
    body = _executable_body(TASK_CLIENT.read_text())
    for forbidden in ("import openai", "import anthropic", "from openai",
                      "from anthropic", "mcp"):
        assert forbidden not in body.lower()


def test_the_explorer_asks_for_the_give_up_escape_and_never_drives_it():
    """REVERT-PROOF 6 (docs/reverse_before_give_up_design.md).

    The give-up escape moved from nav2_behaviors into the decisive controller, and
    the shortest path from "the explorer needs the rover to back up" to a working
    rover is for the explorer to publish a Twist itself. That would put a second
    author on cmd_vel — the failure the whole controller/supervisor split exists to
    prevent — and it would bypass the collision supervisor entirely.

    So the boundary is asserted structurally, here, where it runs without ROS: the
    explorer may ASK (an action client) and must never DRIVE.
    """
    body = _executable_body(EXPLORER.read_text())
    assert "Twist" not in body
    assert "cmd_vel" not in body
    # NOTE, deliberately narrower than the task node's rule above: this node DOES
    # import geometry_msgs.PoseStamped, because sending NavigateToPose a destination
    # is its job. Banning the package here would be cargo-culting the task node's
    # boundary rather than stating this one. The property that matters is that it
    # never expresses a VELOCITY.
    assert "PoseStamped" in body, "the explorer stopped sending poses entirely?"
    # And it must still be ASKING someone: a boundary satisfied by deleting the
    # escape entirely would pass every assertion above.
    assert "escape_in_place" in body, (
        "the explorer no longer requests the give-up escape at all — the boundary "
        "holds, but the rover is back to giving up without trying to move")
