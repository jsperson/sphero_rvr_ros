#!/usr/bin/env python3
"""Aggregate analyzed ground samples into a repeatability report."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src"))

from sphero_rvr_driver.ground_calibration import (  # noqa: E402
    GroundCalibrationError,
    aggregate_ground_samples,
)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Check repeated ground samples and report a review-only median scale."
    )
    parser.add_argument("sample_json", type=Path, nargs="+")
    parser.add_argument("--output", type=Path, help="optional JSON output file")
    args = parser.parse_args()

    try:
        samples = [json.loads(path.read_text()) for path in args.sample_json]
        result = aggregate_ground_samples(samples)
    except (OSError, json.JSONDecodeError, GroundCalibrationError) as exc:
        parser.error(str(exc))

    rendered = json.dumps(result, indent=2, sort_keys=True) + "\n"
    if args.output is not None:
        args.output.write_text(rendered)
    print(rendered, end="")
    return 0 if result["eligible_for_config_review"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
