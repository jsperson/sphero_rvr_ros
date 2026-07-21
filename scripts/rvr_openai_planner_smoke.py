#!/usr/bin/env python3
"""No-hardware first-party OpenAI Responses smoke for the Mission API v2 planner.

This script deliberately refuses to use OpenRouter or any non-OpenAI endpoint.
It performs a bounded replay-only planner run: the live model may request only
mission_api.v2 tool calls, executed by fake adapters, and no ROS/hardware surface
is opened.
"""

from __future__ import annotations

import json
import os
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from sphero_rvr_driver.mission_api import MissionValidationError
from sphero_rvr_driver.mission_api import FakeCapabilityAdapters, MissionBudgets
from sphero_rvr_driver.mission_planner import OpenAICompatiblePlannerProvider, IterativeMissionPlanner


def main() -> int:
    if not os.environ.get("OPENAI_API_KEY"):
        print(
            json.dumps(
                {
                    "status": "blocked",
                    "reason": "OPENAI_API_KEY is not configured; refusing to substitute OpenRouter or fake live-provider evidence.",
                },
                sort_keys=True,
            )
        )
        return 2

    try:
        provider = OpenAICompatiblePlannerProvider(max_retries=1, timeout_s=45.0)
        planner = IterativeMissionPlanner(
            provider=provider,
            adapters=FakeCapabilityAdapters(),
            budgets=MissionBudgets(max_steps=2, max_runtime_s=60.0, max_travel_m=None),
            max_iterations=3,
            registry_version="mission_api.v2.no_hardware_openai_smoke",
        )
        manifest = planner.run(
            "No-hardware smoke: query status telemetry using mission_api.v2, then return a structured complete decision. "
            "Do not request motion, ROS topics, camera devices, shell, files, credentials, or physical execution."
        )
    except MissionValidationError as exc:
        print(json.dumps({"status": "failed", "reason": str(exc)}, sort_keys=True))
        return 1

    payload = manifest.to_json_dict()
    payload["status"] = "passed" if payload["provider_id"] == "openai" and payload["api_surface"] == "responses" else "failed"
    print(json.dumps(payload, sort_keys=True))
    return 0 if payload["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
