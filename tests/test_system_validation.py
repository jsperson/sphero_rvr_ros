from __future__ import annotations

import json
from pathlib import Path

import pytest

from sphero_rvr_driver.live_route_runner import LiveRouteConfig
from sphero_rvr_driver.system_validation import (
    REQUIRED_CURRENT_SHA_TOPICS,
    CorpusManifestError,
    build_current_sha_corpus_manifest,
    main,
    parse_rosbag2_metadata,
    validate_current_sha_corpus_manifest,
    validate_emergency_dispatch_latency,
    validate_fake_route_execution_corpus,
)

REPO_ROOT = Path(__file__).resolve().parents[1]


def test_system_validation_runs_without_a_hosted_workflow(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    assert main(["--repo-root", str(tmp_path)]) == 0
    assert capsys.readouterr().out == "system validation checks passed\n"


def test_current_sha_corpus_manifest_requires_all_no_motion_topics_and_exact_sha(tmp_path: Path) -> None:
    bag = tmp_path / "current-sha" / "rosbag"
    bag.mkdir(parents=True)
    (bag / "metadata.yaml").write_text(
        "\n".join(
            [
                "rosbag2_bagfile_information:",
                "  starting_time:",
                "    nanoseconds_since_epoch: 1784595456000000000",
                "  duration:",
                "    nanoseconds: 5000000000",
                "  topics_with_message_count:",
                *[
                    f"    - topic_metadata:\n        name: {topic}\n        type: example_msgs/msg/Fake\n      message_count: {10 if topic != '/tf_static' else 1}"
                    for topic in REQUIRED_CURRENT_SHA_TOPICS
                ],
            ]
        )
        + "\n"
    )

    manifest = build_current_sha_corpus_manifest(
        run_id="current-sha-no-motion",
        bag_path=bag,
        git_sha="cd15437cc04360a693111dc8e9a771ab3c507228",
        environment={"host": "sphero-pi-2", "ros_distro": "jazzy", "hardware_motion": False},
        frame_graph={"odom": ["base_link"], "base_link": ["laser", "camera_link"]},
        cleanup={"processes_terminated": True, "serial_owners_after": []},
        clock_basis="system_utc_and_ros_header_stamps",
    )

    data = validate_current_sha_corpus_manifest(manifest, expected_sha="cd15437cc04360a693111dc8e9a771ab3c507228")

    assert data["git_sha"] == "cd15437cc04360a693111dc8e9a771ab3c507228"
    assert data["no_motion"] is True
    assert data["topic_counts"]["/scan"] == 10
    assert data["topic_counts"]["/tf_static"] == 1
    assert data["topic_rates_hz"]["/odom"] == pytest.approx(2.0)
    assert data["frame_graph"]["base_link"] == ["laser", "camera_link"]


def test_manifest_validation_rejects_missing_tf_or_odom_channel(tmp_path: Path) -> None:
    bag = tmp_path / "bad" / "rosbag"
    bag.mkdir(parents=True)
    (bag / "metadata.yaml").write_text(
        "rosbag2_bagfile_information:\n"
        "  duration:\n"
        "    nanoseconds: 1000000000\n"
        "  topics_with_message_count:\n"
        "    - topic_metadata:\n"
        "        name: /scan\n"
        "      message_count: 5\n"
    )

    manifest = build_current_sha_corpus_manifest(
        run_id="bad",
        bag_path=bag,
        git_sha="abc123",
        environment={"host": "sphero-pi-2", "hardware_motion": False},
        frame_graph={"base_link": ["laser"]},
        cleanup={"processes_terminated": True, "serial_owners_after": []},
    )

    with pytest.raises(CorpusManifestError, match="missing required topic"):
        validate_current_sha_corpus_manifest(manifest, expected_sha="abc123")


def test_rosbag2_metadata_parser_extracts_counts_and_rates() -> None:
    metadata = parse_rosbag2_metadata(
        "rosbag2_bagfile_information:\n"
        "  duration:\n"
        "    nanoseconds: 2000000000\n"
        "  topics_with_message_count:\n"
        "    - topic_metadata:\n"
        "        name: /odom\n"
        "      message_count: 20\n"
    )

    assert metadata.duration_seconds == 2.0
    assert metadata.topic_counts == {"/odom": 20}
    assert metadata.topic_rates_hz == {"/odom": 10.0}


def test_fake_route_execution_corpus_catches_false_terminal_success_and_latency_regressions() -> None:
    corpus = validate_fake_route_execution_corpus(LiveRouteConfig(collision_state_max_age_s=0.30))

    assert corpus["complete_route"]["status"] == "complete"
    assert corpus["stale_collision"]["status"] == "blocked"
    assert corpus["stale_collision"]["terminal_reason"] == "stale_collision_state"
    assert corpus["wrong_direction"]["status"] == "failed"
    assert corpus["estop"]["terminal_reason"] == "estopped"
    assert corpus["collision_veto"]["terminal_reason"] == "collision_veto"

    with pytest.raises(AssertionError, match="emergency dispatch latency"):
        validate_emergency_dispatch_latency([0.002, 0.003, 0.050], max_p95_seconds=0.010)

    assert validate_emergency_dispatch_latency([0.001, 0.002, 0.003], max_p95_seconds=0.010)["p95_seconds"] <= 0.010


def test_system_validation_entry_point_is_installed() -> None:
    setup_text = (REPO_ROOT / "setup.py").read_text()

    assert "rvr_system_check = sphero_rvr_driver.system_validation:main" in setup_text
    assert "docs/system_validation.md" in setup_text


def test_operator_runbook_uses_executable_module_command_and_canonical_route_topics() -> None:
    runbook = (REPO_ROOT / "docs" / "system_validation.md").read_text()

    assert "python -m sphero_rvr_driver.system_validation --repo-root ." in runbook
    assert "`/mission_api/v2/live_route/request`" in runbook
    assert "`/mission_api/v2/live_route/status`" in runbook
    assert "`/live_route/command`" not in runbook
    assert "`/live_route/status`" not in runbook
