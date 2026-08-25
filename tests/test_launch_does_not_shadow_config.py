"""A launch override that disagrees with the YAML makes the YAML a lie.

2026-08-25. `supervised_rvr.launch.py` passes the supervisor's parameters as
``[collision_stop.yaml, {three overrides}]``. In ROS 2 launch the LATER entry
wins, so those three fields come from LaunchConfiguration defaults and the file
is ignored for them. Measured on the running rover: ``front_slow_*_angle_deg``
published +/-45 while the deployed YAML said +/-35.

The cost was not the mismatch, it was that nothing could SEE the mismatch.
``4bb920d`` (2026-08-02, "trim timid brake") narrowed the corridor to +/-35 in
the YAML -- a deliberate tune, 23 days dead on arrival, because ``57e26be``
(2026-07-23) had already added the override at +/-45.

This guard asserts the EFFECTIVE value of every key -- YAML, then any launch
override applied on top -- equals the YAML's own value. It is deliberately
written over all 67 keys rather than over the override list, so that deleting
the overrides does not make it pass vacuously (Appendix A: a guard that would
pass on an empty file is measuring the wrong surface), and so that a NEW
shadowing override added later fails it too.

Reads what the machine reads: the launch file via ``ast``, the config via YAML.
No prose is matched (Appendix A5).
"""

from __future__ import annotations

import ast
import pathlib

import pytest

yaml = pytest.importorskip("yaml")

REPO = pathlib.Path(__file__).resolve().parents[1]
LAUNCH = REPO / "launch" / "supervised_rvr.launch.py"
CONFIG = REPO / "config" / "collision_stop.yaml"
NODE_EXECUTABLE = "lidar_collision_stop_supervisor"


def _config_values():
    doc = yaml.safe_load(CONFIG.read_text())
    node = doc[NODE_EXECUTABLE]
    return dict(node["ros__parameters"])


def _launch_argument_defaults(tree):
    """name -> default_value, for every DeclareLaunchArgument in the file."""
    defaults = {}
    for call in (n for n in ast.walk(tree) if isinstance(n, ast.Call)):
        if getattr(call.func, "id", None) != "DeclareLaunchArgument":
            continue
        if not call.args or not isinstance(call.args[0], ast.Constant):
            continue
        name = call.args[0].value
        for kw in call.keywords:
            if kw.arg == "default_value" and isinstance(kw.value, ast.Constant):
                defaults[name] = kw.value.value
    return defaults


def _launch_configuration_bindings(tree):
    """local variable -> LaunchConfiguration name."""
    bindings = {}
    for node in ast.walk(tree):
        if not isinstance(node, ast.Assign) or len(node.targets) != 1:
            continue
        target = node.targets[0]
        value = node.value
        if not isinstance(target, ast.Name) or not isinstance(value, ast.Call):
            continue
        if getattr(value.func, "id", None) != "LaunchConfiguration":
            continue
        if value.args and isinstance(value.args[0], ast.Constant):
            bindings[target.id] = value.args[0].value
    return bindings


def _supervisor_overrides(tree, bindings):
    """param name -> LaunchConfiguration name, for the supervisor Node only."""
    for call in (n for n in ast.walk(tree) if isinstance(n, ast.Call)):
        if getattr(call.func, "id", None) != "Node":
            continue
        kw = {k.arg: k.value for k in call.keywords}
        executable = kw.get("executable")
        if not isinstance(executable, ast.Constant) or executable.value != NODE_EXECUTABLE:
            continue
        overrides = {}
        params = kw.get("parameters")
        if not isinstance(params, ast.List):
            return overrides
        for entry in params.elts:
            if not isinstance(entry, ast.Dict):
                continue
            for key, value in zip(entry.keys, entry.values):
                if not isinstance(key, ast.Constant):
                    continue
                for inner in ast.walk(value):
                    # a variable bound to LaunchConfiguration(...) earlier ...
                    if isinstance(inner, ast.Name) and inner.id in bindings:
                        overrides[key.value] = bindings[inner.id]
                        break
                    # ... or LaunchConfiguration("x") written inline. The first
                    # version of this guard handled only the former and a
                    # mutation test walked straight past it.
                    if (isinstance(inner, ast.Call)
                            and getattr(inner.func, "id", None) == "LaunchConfiguration"
                            and inner.args
                            and isinstance(inner.args[0], ast.Constant)):
                        overrides[key.value] = inner.args[0].value
                        break
        return overrides
    raise AssertionError(f"no Node(executable={NODE_EXECUTABLE!r}) in {LAUNCH}")


def _effective_values():
    tree = ast.parse(LAUNCH.read_text())
    bindings = _launch_configuration_bindings(tree)
    overrides = _supervisor_overrides(tree, bindings)
    defaults = _launch_argument_defaults(tree)
    effective = dict(_config_values())
    for param, argument in overrides.items():
        if argument in defaults:
            effective[param] = float(defaults[argument])
    return effective


def test_the_supervisor_yaml_is_passed_at_all():
    """Non-vacuity: the guard below is meaningless if the file never reaches the node."""
    tree = ast.parse(LAUNCH.read_text())
    for call in (n for n in ast.walk(tree) if isinstance(n, ast.Call)):
        if getattr(call.func, "id", None) != "Node":
            continue
        kw = {k.arg: k.value for k in call.keywords}
        executable = kw.get("executable")
        if isinstance(executable, ast.Constant) and executable.value == NODE_EXECUTABLE:
            source = ast.dump(kw.get("parameters", ast.Constant(None)))
            assert "collision_stop_config" in source, (
                "the supervisor node no longer receives collision_stop.yaml at all"
            )
            return
    raise AssertionError("supervisor node not found")


def test_no_launch_override_shadows_the_deployed_config():
    """Every key's EFFECTIVE value must equal the YAML's value for that key."""
    config = _config_values()
    effective = _effective_values()
    disagreements = {
        name: (config[name], effective[name])
        for name in sorted(config)
        if effective[name] != config[name]
    }
    assert not disagreements, (
        "launch defaults shadow the deployed config -- the YAML says one thing and "
        "the node runs another:\n"
        + "\n".join(
            f"  {name}: yaml {was!r} -> running {now!r}"
            for name, (was, now) in disagreements.items()
        )
        + "\nFix the launch (or the YAML), never the test."
    )


def test_the_guard_covers_every_key_not_just_the_overridden_ones():
    """Mutation tripwire: this must not become a check over an empty override set."""
    assert len(_config_values()) >= 60, (
        "the guard's population collapsed -- it would pass vacuously"
    )
