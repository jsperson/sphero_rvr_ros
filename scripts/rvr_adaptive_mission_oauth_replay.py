#!/usr/bin/env python3
"""Real ChatGPT-OAuth Adaptive mission adaptive movement replay acceptance."""

from __future__ import annotations

import argparse
import json
import subprocess
import tempfile
import time

from sphero_rvr_driver.mission_web import AdaptiveMissionAdapter
from sphero_rvr_driver.adaptive_mission_controller import (
    CodexOAuthAdaptiveMissionIntentProvider,
)


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Run a real OAuth planner through repeated Adaptive mission movement "
            "revisions using the replay executor"
        )
    )
    parser.add_argument("--model", default=None)
    parser.add_argument("--reasoning-effort", default="low")
    parser.add_argument("--provider-timeout", type=float, default=120.0)
    parser.add_argument("--replay-timeout", type=float, default=600.0)
    args = parser.parse_args()

    source_sha = (
        subprocess.run(
            ["git", "rev-parse", "HEAD"],
            check=True,
            text=True,
            stdout=subprocess.PIPE,
        )
        .stdout.strip()
    )
    provider = CodexOAuthAdaptiveMissionIntentProvider(
        model=args.model,
        reasoning_effort=args.reasoning_effort,
        timeout_s=args.provider_timeout,
    )
    prompt = (
        "Explore this simulated room by choosing each next action from the "
        "fresh typed world snapshot. Gather evidence that includes at least "
        "one nonzero translation, at least one rotation, and one explicit "
        "observation. Choose the order and magnitudes from current clearances "
        "and progress. Once all three kinds of evidence are present, stop."
    )
    with tempfile.TemporaryDirectory(prefix="rvr-adaptive-mission-oauth-replay-") as temp:
        adapter = AdaptiveMissionAdapter(
            provider,
            database=f"{temp}/mission.sqlite3",
            source_sha=source_sha,
            deployed_sha=source_sha,
            operator="oauth-replay-local-operator",
            allow_loopback_test_approval=True,
        )
        try:
            proposed = adapter.propose(prompt, "adaptive_mission_explore")
            if proposed["mission"]["state"] != "PROPOSED":
                raise RuntimeError(
                    "real provider did not produce a adaptive mission proposal: "
                    f"{proposed['mission']}"
                )
            adapter.approve(proposed["approval"]["required_phrase"])
            deadline = time.monotonic() + args.replay_timeout
            while time.monotonic() < deadline:
                terminal = dict(adapter.snapshot())
                if terminal["mission"]["terminal"]:
                    break
                time.sleep(0.05)
            else:
                adapter.cancel()
                raise RuntimeError("Adaptive mission OAuth replay did not terminate in time")
        finally:
            adapter.close()

    result = terminal["mission"]["result"]
    revisions = result.get("intent_revisions", [])
    final_snapshot = result.get("final_snapshot", {})
    progress = final_snapshot.get("progress", {})
    evidence = {
        "schema": "sphero_rvr.adaptive_mission_oauth_replay.v1",
        "status": terminal["mission"]["state"],
        "terminal_reason": terminal["mission"]["terminal_reason"],
        "provider": result.get("provider", {}),
        "proposal_digest": proposed["proposal"]["proposal_digest"],
        "source_sha": source_sha,
        "interpreted_objective": proposed["proposal"][
            "interpreted_objective"
        ],
        "intent_revisions": [
            {
                "revision": item["revision"],
                "snapshot_id": item["snapshot_id"],
                "action": item["action"],
                "distance_m": item["distance_m"],
                "angle_deg": item["angle_deg"],
                "rationale": item["rationale"],
                "movement": item.get("execution", {}).get("movement", {}),
            }
            for item in revisions
        ],
        "world_snapshot_ids": [
            item.get("snapshot_id", "")
            for item in result.get("world_snapshots", [])
        ],
        "progress": progress,
        "approval": result.get("approval", {}),
        "limits": result.get("limits", {}),
        "motion_authority": result.get("motion_authority"),
        "physical_execution_enabled": result.get(
            "physical_execution_enabled"
        ),
        "motor_topic_publisher": result.get("motor_topic_publisher"),
    }
    print(json.dumps(evidence, indent=2, sort_keys=True))

    actions = [item.get("action") for item in revisions]
    accepted = (
        terminal["mission"]["state"] == "COMPLETE"
        and terminal["mission"]["terminal_reason"] == "planner_stop"
        and len(revisions) >= 4
        and "move_distance" in actions
        and "turn_angle" in actions
        and "observe" in actions
        and actions[-1] == "stop"
        and float(progress.get("cumulative_translation_m", 0.0)) > 0.0
        and float(progress.get("cumulative_rotation_deg", 0.0)) > 0.0
        and int(progress.get("observation_count", 0)) > 0
        and len(set(evidence["world_snapshot_ids"])) == len(
            evidence["world_snapshot_ids"]
        )
        and not result.get("motion_authority")
        and not result.get("physical_execution_enabled")
    )
    return 0 if accepted else 1


if __name__ == "__main__":
    raise SystemExit(main())
