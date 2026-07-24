#!/usr/bin/env python3
"""Real ChatGPT-OAuth Stage D smoke with an observation-only replay executor."""

from __future__ import annotations

import argparse
import json
import subprocess
import time

from sphero_rvr_driver.stage_d_controller import (
    CodexOAuthStageDIntentProvider,
    ReplayStageDExecutor,
    StageDApprovalEnvelope,
    StageDController,
    StageDIntent,
    StageDLimits,
)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Run a real OAuth Stage D observe/stop loop with motion disabled"
    )
    parser.add_argument("--model", default=None)
    parser.add_argument("--reasoning-effort", default="low")
    parser.add_argument("--provider-timeout", type=float, default=120.0)
    parser.add_argument("--smoke-timeout", type=float, default=300.0)
    args = parser.parse_args()

    limits = StageDLimits()
    provider = CodexOAuthStageDIntentProvider(
        model=args.model,
        reasoning_effort=args.reasoning_effort,
        timeout_s=args.provider_timeout,
    )
    executor = ReplayStageDExecutor(
        limits=limits,
        motion_permitted=False,
    )
    mission_id = f"stage-d-oauth-smoke-{int(time.time())}"
    prompt = (
        "Perform a no-motion Stage D smoke: inspect the current typed safety "
        "snapshot once, then stop. Do not request translation or rotation."
    )
    snapshot = executor.snapshot(mission_id)
    first_raw = provider.choose(prompt, snapshot)
    now = time.time()
    first = StageDIntent.validated(
        first_raw,
        revision=1,
        snapshot=snapshot,
        issued_at_s=now,
        provider_id=provider.provider_id,
        model_id=provider.model_id,
        limits=limits,
    )
    source_sha = (
        subprocess.run(
            ["git", "rev-parse", "HEAD"],
            check=True,
            text=True,
            stdout=subprocess.PIPE,
        )
        .stdout.strip()
    )
    proposal = StageDApprovalEnvelope(
        mission_id=mission_id,
        lease_id=f"{mission_id}-lease",
        prompt=prompt,
        interpreted_objective=first.interpreted_objective,
        source_sha=source_sha,
        deployed_sha=source_sha,
        provider_id=provider.provider_id,
        model_id=provider.model_id,
        reasoning_effort=provider.reasoning_effort,
        executor_mode=executor.mode,
        starting_snapshot_id=str(snapshot["snapshot_id"]),
        first_intent=first_raw,
        limits=limits,
    ).proposal()
    controller = StageDController(
        mission_id=mission_id,
        prompt=prompt,
        proposal_digest=str(proposal["proposal_digest"]),
        operator="oauth-smoke-local-operator",
        authenticated=True,
        authentication_source="explicit-local-oauth-smoke",
        approved_at_s=now,
        first_snapshot=snapshot,
        first_intent=first,
        provider=provider,
        executor=executor,
        limits=limits,
    )
    controller.start()
    deadline = time.monotonic() + args.smoke_timeout
    try:
        while time.monotonic() < deadline:
            result = controller.snapshot()
            if result["terminal"]:
                break
            time.sleep(0.05)
        else:
            controller.cancel()
            raise RuntimeError("Stage D OAuth smoke did not terminate in time")
    finally:
        controller.close()

    revisions = result["intent_revisions"]
    nonzero = [
        item
        for item in revisions
        if item["action"] in {"move_distance", "turn_angle"}
        or item["distance_m"] != 0.0
        or item["angle_deg"] != 0.0
    ]
    evidence = {
        "schema": "sphero_rvr.stage_d_oauth_smoke.v1",
        "status": result["status"],
        "terminal_reason": result["terminal_reason"],
        "provider": {
            "provider_id": provider.provider_id,
            "model_id": provider.model_id,
            "reasoning_effort": provider.reasoning_effort,
        },
        "proposal_digest": proposal["proposal_digest"],
        "source_sha": source_sha,
        "intent_actions": [item["action"] for item in revisions],
        "provider_calls": result["inference"],
        "motion_intent_count": len(nonzero),
        "motion_authority": result["motion_authority"],
        "physical_execution_enabled": result["physical_execution_enabled"],
        "motor_topic_publisher": "lidar_collision_stop_supervisor",
    }
    print(json.dumps(evidence, indent=2, sort_keys=True))
    if (
        result["status"] != "complete"
        or result["terminal_reason"] != "planner_stop"
        or nonzero
        or result["motion_authority"]
        or result["physical_execution_enabled"]
    ):
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
