#!/usr/bin/env python3
"""Benchmark OAuth adaptive planning against recorded real mission snapshots."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sqlite3
import time
from typing import Any, Mapping

from sphero_rvr_driver.adaptive_mission_controller import (
    AdaptiveMissionIntent,
    AdaptiveMissionLimits,
    CodexOAuthAdaptiveMissionIntentProvider,
    choose_validated_adaptive_intent,
)


DEFAULT_MODELS = (
    "gpt-5.6-sol",
    "gpt-5.6-terra",
    "gpt-5.6-luna",
)


def _recorded_cases(
    database: Path,
    mission_id: str,
) -> list[dict[str, Any]]:
    connection = sqlite3.connect(
        f"file:{database.expanduser().resolve()}?mode=ro", uri=True
    )
    try:
        row = connection.execute(
            "SELECT result_json FROM prompt_missions WHERE mission_id = ?",
            (str(mission_id),),
        ).fetchone()
    finally:
        connection.close()
    if row is None:
        raise RuntimeError(f"recorded mission is unavailable: {mission_id}")
    result = json.loads(row[0])
    snapshots = result.get("world_snapshots", [])
    if not isinstance(snapshots, list) or not snapshots:
        raise RuntimeError("recorded mission has no world snapshots")
    stall = next(
        (
            item
            for item in snapshots
            if isinstance(item, Mapping)
            and (item.get("last_execution") or {})
            .get("navigation_outcome", {})
            .get("reason")
            == "stall"
            and (item.get("last_execution") or {})
            .get("navigation_outcome", {})
            .get("recoverable")
            is True
        ),
        None,
    )
    if stall is None:
        raise RuntimeError("recorded mission has no recoverable stall snapshot")
    clear = next(
        (
            item
            for item in reversed(snapshots)
            if isinstance(item, Mapping)
            and (item.get("last_execution") or {})
            .get("navigation_outcome", {})
            .get("reason")
            == "complete"
        ),
        snapshots[0],
    )
    return [
        {
            "name": "clear_exploration",
            "prompt": (
                "Explore and map the room from this fresh typed snapshot. "
                "Choose one bounded action that makes objective progress."
            ),
            "snapshot": clear,
        },
        {
            "name": "fixed_obstacle_stall_recovery",
            "prompt": (
                "Explore and map the room. Recover safely from the recorded "
                "settled stall using the current typed and visual evidence."
            ),
            "snapshot": stall,
        },
        {
            "name": "object_directed_progress",
            "prompt": (
                "Find and approach the detected shoe only through validated "
                "clear floor, then reassess."
            ),
            "snapshot": stall,
        },
    ]


def _case_valid(
    name: str,
    raw: Mapping[str, Any],
    intent: AdaptiveMissionIntent,
    metric: Mapping[str, Any],
) -> tuple[bool, list[str]]:
    failures: list[str] = []
    if not metric.get("success"):
        failures.append("instrumented cycle failed")
    if name == "fixed_obstacle_stall_recovery":
        if intent.action == "move_distance" and intent.distance_m >= 0.0:
            failures.append("repeated forward motion after stall")
        rationale = intent.rationale.casefold()
        if not any(
            phrase in rationale
            for phrase in (
                "fixed",
                "obstacle",
                "obstruction",
                "minor surface",
                "indeterminate",
            )
        ):
            failures.append("rationale omitted visual obstacle assessment")
        if metric.get("image_attached") is not True:
            failures.append("stall recovery omitted image pixels")
    if name == "object_directed_progress":
        if intent.objective_status not in {
            "in_progress",
            "needs_observation",
        }:
            failures.append("object-directed objective made no progress")
        if metric.get("image_attached") is not True:
            failures.append("object-directed objective omitted image pixels")
    if name == "clear_exploration" and intent.objective_status not in {
        "in_progress",
        "needs_observation",
    }:
        failures.append("clear exploration made no objective progress")
    if str(raw.get("snapshot_id", "")) != intent.snapshot_id:
        failures.append("decision was not snapshot-bound")
    return not failures, failures


def _percentile(values: list[float], fraction: float) -> float:
    ordered = sorted(float(value) for value in values)
    if not ordered:
        return 0.0
    index = max(0, min(len(ordered) - 1, int(len(ordered) * fraction + 0.999999) - 1))
    return round(ordered[index], 3)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--database",
        default="~/.local/state/sphero_rvr/missions.sqlite3",
    )
    parser.add_argument("--mission-id", required=True)
    parser.add_argument("--model", action="append", dest="models")
    parser.add_argument("--reasoning-effort", default="low")
    parser.add_argument(
        "--integration",
        choices=("app-server", "exec"),
        default="app-server",
    )
    parser.add_argument("--legacy-full-input", action="store_true")
    parser.add_argument("--repetitions", type=int, default=3)
    parser.add_argument("--timeout", type=float, default=60.0)
    args = parser.parse_args()
    if args.repetitions < 1:
        parser.error("--repetitions must be positive")

    cases = _recorded_cases(Path(args.database), args.mission_id)
    results: list[dict[str, Any]] = []
    limits = AdaptiveMissionLimits()
    for model in args.models or DEFAULT_MODELS:
        provider = CodexOAuthAdaptiveMissionIntentProvider(
            model=model,
            reasoning_effort=args.reasoning_effort,
            timeout_s=args.timeout,
            limits=limits,
            integration=args.integration,
            compact_input=not args.legacy_full_input,
        )
        try:
            for repetition in range(1, args.repetitions + 1):
                for case in cases:
                    try:
                        raw, intent = choose_validated_adaptive_intent(
                            provider,
                            str(case["prompt"]),
                            case["snapshot"],
                            revision=1,
                            issued_at_s=time.time(),
                            limits=limits,
                        )
                        first = intent.to_json_dict()
                        deterministic = AdaptiveMissionIntent.validated(
                            raw,
                            revision=1,
                            snapshot=case["snapshot"],
                            issued_at_s=first["issued_at_s"],
                            provider_id=provider.provider_id,
                            model_id=provider.model_id,
                            limits=limits,
                            supervised_collision_escape=bool(
                                (
                                    case["snapshot"].get("last_execution")
                                    or {}
                                )
                                .get("navigation_outcome", {})
                                .get("reason")
                                == "collision_veto"
                                and raw.get("action") == "move_distance"
                                and float(raw.get("distance_m", 0.0)) < 0.0
                            ),
                        ).to_json_dict()
                        metric = provider.latency_history()[-1]
                        valid, failures = _case_valid(
                            str(case["name"]), raw, intent, metric
                        )
                        result = {
                            "model_id": model,
                            "reasoning_effort": args.reasoning_effort,
                            "integration": args.integration,
                            "case": case["name"],
                            "repetition": repetition,
                            "action": intent.action,
                            "objective_status": intent.objective_status,
                            "schema_valid": True,
                            "deterministic_validation": first == deterministic,
                            "behavior_valid": valid,
                            "failures": failures,
                            "latency": metric,
                        }
                    except Exception as exc:
                        history = provider.latency_history()
                        metric = (
                            history[-1]
                            if history
                            else {
                                name: 0.0
                                for name in (
                                    "prompt_image_preparation_ms",
                                    "oauth_client_startup_ms",
                                    "inference_ms",
                                    "validation_ms",
                                    "total_ms",
                                )
                            }
                        )
                        result = {
                            "model_id": model,
                            "reasoning_effort": args.reasoning_effort,
                            "integration": args.integration,
                            "case": case["name"],
                            "repetition": repetition,
                            "action": "",
                            "objective_status": "",
                            "schema_valid": False,
                            "deterministic_validation": False,
                            "behavior_valid": False,
                            "failures": [
                                f"{exc.__class__.__name__}: {exc}"
                            ],
                            "latency": metric,
                        }
                    results.append(result)
                    print(json.dumps({"result": result}, sort_keys=True))
        finally:
            provider.close()

    summaries: list[dict[str, Any]] = []
    for model in args.models or DEFAULT_MODELS:
        selected = [item for item in results if item["model_id"] == model]
        summary: dict[str, Any] = {
            "model_id": model,
            "reasoning_effort": args.reasoning_effort,
            "integration": args.integration,
            "cycles": len(selected),
            "all_schema_valid": all(item["schema_valid"] for item in selected),
            "all_deterministic": all(
                item["deterministic_validation"] for item in selected
            ),
            "all_behavior_valid": all(
                item["behavior_valid"] for item in selected
            ),
        }
        for phase in (
            "prompt_image_preparation_ms",
            "oauth_client_startup_ms",
            "inference_ms",
            "validation_ms",
            "total_ms",
        ):
            values = [float(item["latency"][phase]) for item in selected]
            summary[f"{phase}_p50"] = _percentile(values, 0.50)
            summary[f"{phase}_p95"] = _percentile(values, 0.95)
        summaries.append(summary)
    report = {
        "schema": "sphero_rvr.adaptive_planner_benchmark.v1",
        "recorded_mission_id": args.mission_id,
        "summaries": summaries,
        "selected_model": next(
            (
                item["model_id"]
                for item in summaries
                if item["all_schema_valid"]
                and item["all_deterministic"]
                and item["all_behavior_valid"]
            ),
            None,
        ),
    }
    print(json.dumps({"report": report}, sort_keys=True))
    return 0 if report["selected_model"] is not None else 1


if __name__ == "__main__":
    raise SystemExit(main())
