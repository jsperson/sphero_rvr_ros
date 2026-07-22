#!/usr/bin/env python3
"""Analyze a downloaded mission terminal artifact against tape measurement."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src"))

from sphero_rvr_driver.ground_calibration import (  # noqa: E402
    GroundCalibrationError,
    analyze_ground_sample,
)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Compare live-route terminal encoder evidence with measured ground travel."
    )
    parser.add_argument("terminal_json", type=Path)
    distance = parser.add_mutually_exclusive_group(required=True)
    distance.add_argument("--actual-distance-m", type=float)
    distance.add_argument("--actual-distance-in", type=float)
    parser.add_argument("--output", type=Path, help="optional JSON output file")
    args = parser.parse_args()

    try:
        payload = json.loads(args.terminal_json.read_text())
        actual_distance_m = (
            args.actual_distance_m
            if args.actual_distance_m is not None
            else args.actual_distance_in * 0.0254
        )
        result = analyze_ground_sample(payload, actual_distance_m=actual_distance_m)
    except (OSError, json.JSONDecodeError, GroundCalibrationError) as exc:
        parser.error(str(exc))

    rendered = json.dumps(result, indent=2, sort_keys=True) + "\n"
    if args.output is not None:
        args.output.write_text(rendered)
    print(rendered, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
