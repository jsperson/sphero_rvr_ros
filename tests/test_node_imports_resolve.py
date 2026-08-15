"""Every name a ROS node imports from `sphero_rvr_core` must EXIST in that module.

Bought on 2026-08-15. Inserting one import block into the middle of another's
parenthesised name list silently re-homed three names -- `WindowedFreezeMonitor`,
`heading_error_to_point`, `select_target_point` -- onto a module that does not define
them. The file still compiled: `py_compile` checks syntax, and a wrong-module import
is perfectly good syntax. It would have failed at `import` time on the robot, which
is the worst place to find it, and the dev machine cannot catch it by importing the
node because there is no `rclpy` here.

So the check is STRUCTURAL: parse the node sources, collect every
`from sphero_rvr_core.X import (a, b, c)`, and assert each name is actually defined in
module X. No ROS required, and it fails on the laptop instead of on carpet.

Same family as the AST guard that keeps the excised goal-clearance filter from
returning: when the thing you need to prove is about code SHAPE, read the shape.
"""

import ast
import glob
import importlib
import os

import pytest

DRIVER_DIR = os.path.join(
    os.path.dirname(__file__), "..", "src", "sphero_rvr_driver"
)
NODE_SOURCES = sorted(glob.glob(os.path.join(DRIVER_DIR, "*.py")))


def core_imports(path):
    """[(module, name, lineno)] for every `from sphero_rvr_core.X import ...`."""
    with open(path) as fh:
        tree = ast.parse(fh.read(), filename=path)
    found = []
    for node in ast.walk(tree):
        if isinstance(node, ast.ImportFrom) and node.module:
            if node.module.startswith("sphero_rvr_core"):
                for alias in node.names:
                    found.append((node.module, alias.name, node.lineno))
    return found


def test_there_are_node_sources_to_check():
    """Guard the guard: a glob that silently matches nothing would make every
    assertion below vacuous."""
    assert NODE_SOURCES, "no driver sources found — this test would be vacuous"
    assert any(core_imports(p) for p in NODE_SOURCES), (
        "no sphero_rvr_core imports found in any node — the parse is not working"
    )


@pytest.mark.parametrize("path", NODE_SOURCES, ids=lambda p: os.path.basename(p))
def test_every_core_import_resolves_to_a_real_name(path):
    problems = []
    for module_name, name, lineno in core_imports(path):
        try:
            module = importlib.import_module(module_name)
        except Exception as exc:                      # noqa: BLE001
            problems.append(f"{os.path.basename(path)}:{lineno} cannot import "
                            f"{module_name}: {exc}")
            continue
        if name != "*" and not hasattr(module, name):
            problems.append(
                f"{os.path.basename(path)}:{lineno} imports {name!r} from "
                f"{module_name}, which does not define it"
            )
    assert not problems, "\n".join(problems)
