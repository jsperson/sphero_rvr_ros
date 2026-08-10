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


def test_task_node_exposes_exactly_the_three_v1_tools():
    """v1 is three tools. explore start/stop, set_search_classes and capture_photo
    are explicitly out of scope, and a surface that quietly grows is how a thin node
    stops being thin."""
    source = TASK_NODE.read_text()
    for tool in ("task/goto", "task/observe", "task/query_semantic_map"):
        assert f'"{tool}"' in source
    body = _executable_body(source)
    for forbidden in ("set_search_classes", "capture_photo", "task/explore"):
        assert forbidden not in body


def test_task_node_carries_no_transport_or_model_layer():
    """v1 is a ROS surface. No MCP, no HTTP, no provider SDK -- those are a client's
    business, and the acceptance test is that deleting the client leaves a working
    robot."""
    body = _executable_body(TASK_NODE.read_text())
    for forbidden in ("import requests", "http", "socket", "openai", "anthropic",
                      "mcp", "stdio"):
        assert forbidden not in body.lower()
