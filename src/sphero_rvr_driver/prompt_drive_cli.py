"""Command-line entry point for proposal-only and approved prompt driving."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
from typing import Any, Mapping, Optional, Sequence

from .mission_api import MissionValidationError
from .prompt_drive import (
    ALLOWED_REASONING_EFFORTS,
    OpenAIPromptDriveProvider,
    PromptDrivePlanner,
    PromptDriveProposal,
    approval_phrase,
    approved_live_route,
    build_prompt_drive_manifest,
    render_prompt_drive_proposal,
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="rvr_prompt_drive",
        description="Use a real OpenAI model to propose a bounded rover route. Default: proposal only, no ROS or motion.",
    )
    parser.add_argument("prompt", help="Quoted natural-language driving request")
    parser.add_argument("--model", default=None, help="First-party OpenAI model (default: OPENAI_MODEL or project default)")
    parser.add_argument(
        "--reasoning-effort",
        choices=ALLOWED_REASONING_EFFORTS,
        default="high",
        help="OpenAI reasoning effort for route interpretation (default: high)",
    )
    parser.add_argument(
        "--execute",
        action="store_true",
        help="After proposal, enable interactive approval and submit the route to the live ROS runner",
    )
    parser.add_argument("--operator", default="local-operator", help="Operator label recorded in the manifest")
    parser.add_argument("--manifest-dir", default="artifacts/prompt_drive", help="Directory for safe run manifests")
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    provider = OpenAIPromptDriveProvider(model=args.model, reasoning_effort=args.reasoning_effort)
    planner = PromptDrivePlanner(provider)
    try:
        proposal = planner.propose(args.prompt)
    except MissionValidationError as exc:
        print(json.dumps({"status": "failed", "error": str(exc)}, sort_keys=True), file=sys.stderr)
        return 2

    print(render_prompt_drive_proposal(proposal))
    if not proposal.executable:
        path = _write_manifest(args.manifest_dir, proposal, build_prompt_drive_manifest(proposal, status="model_rejected"))
        print(f"Manifest: {path}")
        return 2
    if not args.execute:
        path = _write_manifest(args.manifest_dir, proposal, build_prompt_drive_manifest(proposal, status="proposal_only"))
        print("\nNo route was published; proposal-only is the default.")
        print(f"Manifest: {path}")
        return 0
    if not sys.stdin.isatty():
        manifest = build_prompt_drive_manifest(
            proposal,
            status="approval_refused",
            error="physical execution requires an interactive terminal",
        )
        path = _write_manifest(args.manifest_dir, proposal, manifest)
        print(f"Physical execution refused: interactive terminal required. Manifest: {path}", file=sys.stderr)
        return 3

    expected = approval_phrase(proposal)
    print("\nWARNING: approval will submit this route for physical motion through the live collision-supervised runner.")
    print("Confirm the rover area is clear and STOP/ESTOP controls are available.")
    print(f"Type exactly: {expected}")
    try:
        supplied = input("> ")
    except (EOFError, KeyboardInterrupt):
        manifest = build_prompt_drive_manifest(
            proposal,
            status="approval_refused",
            error="operator approval was interrupted",
        )
        path = _write_manifest(args.manifest_dir, proposal, manifest)
        print(f"\nPhysical execution refused: approval interrupted. Manifest: {path}", file=sys.stderr)
        return 3
    try:
        route = approved_live_route(proposal, supplied, operator=args.operator)
    except MissionValidationError as exc:
        manifest = build_prompt_drive_manifest(proposal, status="approval_refused", error=str(exc))
        path = _write_manifest(args.manifest_dir, proposal, manifest)
        print(f"Physical execution refused: {exc}. Manifest: {path}", file=sys.stderr)
        return 3

    try:
        # Delayed import is a safety property: proposal-only and failed approval
        # paths do not import ROS, create a node, or publish a route.
        from .prompt_drive_ros import RosLiveRouteExecutor

        result = RosLiveRouteExecutor().execute(route)
        status = "complete" if str(result.get("status", "")).lower() == "complete" else "terminal_failure"
        manifest = build_prompt_drive_manifest(
            proposal,
            status=status,
            operator=args.operator,
            approved=True,
            route=route,
            execution_result=result,
        )
        path = _write_manifest(args.manifest_dir, proposal, manifest)
        print(json.dumps(result, indent=2, sort_keys=True))
        print(f"Manifest: {path}")
        return 0 if status == "complete" else 4
    except KeyboardInterrupt:
        manifest = build_prompt_drive_manifest(
            proposal,
            status="execution_interrupted",
            operator=args.operator,
            approved=True,
            route=route,
            error="operator interrupted execution; cancellation requested by ROS client",
        )
        path = _write_manifest(args.manifest_dir, proposal, manifest)
        print(f"\nExecution interrupted. Verify the rover is stationary. Manifest: {path}", file=sys.stderr)
        return 130
    except Exception as exc:
        manifest = build_prompt_drive_manifest(
            proposal,
            status="execution_failed",
            operator=args.operator,
            approved=True,
            route=route,
            error=str(exc),
        )
        path = _write_manifest(args.manifest_dir, proposal, manifest)
        print(f"Execution failed: {exc}. Manifest: {path}", file=sys.stderr)
        return 4


def _write_manifest(directory: str, proposal: PromptDriveProposal, manifest: Mapping[str, Any]) -> Path:
    target_dir = Path(directory)
    target_dir.mkdir(parents=True, exist_ok=True)
    path = target_dir / f"{proposal.proposal_digest[:12]}.json"
    path.write_text(json.dumps(dict(manifest), indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return path


if __name__ == "__main__":
    raise SystemExit(main())
