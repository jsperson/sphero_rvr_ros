"""Every launch and config file in the tree must be INSTALLED, or it does not exist.

`setup.py` lists launch and config files explicitly rather than globbing them. That is a
fine choice -- explicit is better than magic -- but it has one failure mode, and this file
closes it: a file added to the repo and forgotten in the manifest is invisible to
`ros2 launch` and to `get_package_share_directory`, while looking completely present in
git.

Found the hard way on 2026-08-16: `config/lean_nav2_stock.yaml` had been on the prototype
branch since it was written and was never in the manifest, so the stock-middle config
**could not have been loaded by anything** even if someone had tried to fly it. The file
was real, reviewed, and unreachable.
"""

import ast
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
SETUP = ROOT / "setup.py"


def _manifest_entries() -> set:
    """Every relative path listed in setup.py's data_files, by AST rather than by regex."""
    tree = ast.parse(SETUP.read_text())
    entries = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Constant) and isinstance(node.value, str):
            if "/" in node.value and not node.value.startswith("share/"):
                entries.add(node.value)
    return entries


@pytest.mark.parametrize("subdir, suffix", [("launch", ".py"), ("config", ".yaml")])
def test_every_file_in_the_tree_is_in_the_install_manifest(subdir, suffix):
    manifest = _manifest_entries()
    missing = []
    for path in sorted((ROOT / subdir).glob(f"*{suffix}")):
        relative = f"{subdir}/{path.name}"
        if relative not in manifest:
            missing.append(relative)

    assert not missing, (
        f"{missing} exist in the tree but are not in setup.py's data_files. "
        "An uninstalled launch or config file cannot be loaded by ros2 launch or found "
        "by get_package_share_directory -- it is invisible to the robot while looking "
        "present in git."
    )


@pytest.mark.parametrize("subdir, suffix", [("launch", ".py"), ("config", ".yaml")])
def test_the_manifest_does_not_list_files_that_no_longer_exist(subdir, suffix):
    # The other direction: a stale entry breaks the BUILD, which is louder, but it is
    # cheap to catch here rather than on the Pi.
    stale = [
        entry
        for entry in _manifest_entries()
        if entry.startswith(f"{subdir}/") and entry.endswith(suffix)
        and not (ROOT / entry).exists()
    ]
    assert not stale, f"setup.py lists {stale}, which are not in the tree"
