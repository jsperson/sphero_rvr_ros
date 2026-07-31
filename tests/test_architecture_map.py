from __future__ import annotations

import re
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]


def test_repo_status_is_only_a_pointer_to_canonical_vault_status() -> None:
    status = (REPO_ROOT / "STATUS.md").read_text(encoding="utf-8")

    assert len(status.splitlines()) == 1
    assert "Obsidian vault" in status
    assert "Projects/Sphero RVR ROS/Current Status.md" in status
    assert "docs/architecture_map.md" in status


def test_architecture_map_is_the_read_first_single_seam_map() -> None:
    agents = (REPO_ROOT / "AGENTS.md").read_text(encoding="utf-8")
    readme = (REPO_ROOT / "README.md").read_text(encoding="utf-8")
    phase_zero = (
        REPO_ROOT / "docs" / "hierarchical_exploration.md"
    ).read_text(encoding="utf-8")

    for text in (agents, readme):
        assert "docs/architecture_map.md" in text
        assert "Current Status.md" in text
    assert "only maintained" in phase_zero
    assert "current ownership/seam table" in phase_zero


def test_architecture_map_tracks_fixed_binding_names() -> None:
    architecture = (
        REPO_ROOT / "docs" / "architecture_map.md"
    ).read_text(encoding="utf-8")
    binding = (
        REPO_ROOT
        / "src"
        / "sphero_rvr_driver"
        / "hierarchical_physical_binding.py"
    ).read_text(encoding="utf-8")

    for constant in (
        "AUTHORITY_TOPIC",
        "GOAL_DISPATCH_TOPIC",
        "NAV2_ACTION",
        "PRIVATE_NAV2_CMD_TOPIC",
        "SUPERVISOR_REQUEST_TOPIC",
        "MOTOR_TOPIC",
    ):
        match = re.search(
            rf"^{constant}\s*=\s*[\"']([^\"']+)[\"']",
            binding,
            flags=re.MULTILINE,
        )
        assert match is not None
        assert f"`{match.group(1)}`" in architecture

    for token in (
        "hierarchical_mission_controller",
        "hierarchical_nav2_adapter",
        "live_route_runner",
        "lidar_collision_stop_supervisor",
        "sphero_rvr_driver",
        "SmacPlanner2D",
        "DWBLocalPlanner",
    ):
        assert token in architecture
