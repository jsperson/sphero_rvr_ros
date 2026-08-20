#!/usr/bin/env python3
"""Offline prompt calibration against the FROZEN recognition corpus.

Design + consensus: docs/design_recognition_schema_redesign_2026-08-20.md §6,
PIN 2. The corpus manifest (frames + ground truth + gates + iteration budget)
was committed before the first iteration; this runner replays the banked bench
photos through the CURRENT build_prompt/parse pipeline and scores every gate.
No robot, no staging — the camera already did its half on the bench.

The iteration budget is enforced here as well as by discipline: each evaluated
run appends to a ledger next to the results file, and once the manifest's
budget is spent the runner refuses — the round goes back to the PM instead of
warm-iterating (model-polishing drift is the failure mode PIN 2 guards).

Run on the Pi (key + photos in place):
    cd ~/ros2_ws/src/sphero_rvr_ros
    PYTHONPATH=src python3 diagnostics/recognition_prompt_calibration.py
"""

from __future__ import annotations

import argparse
import datetime
import json
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from sphero_rvr_core.recognition import build_prompt, parse_recognition_reply  # noqa: E402
from sphero_rvr_core.vlm_client import query_vlm  # noqa: E402

IDENTITY_HONEST = ("confirmed", "unverified")


def evaluate(frames: list, results: dict) -> dict:
    """Gate verdicts from per-frame results. Pure — unit-testable without a
    single API call."""
    gates = {"schema_valid": True, "absent_zero_fp": True, "bottle_detect": True,
             "bottle_identity_honest": True, "controller": True}
    for f in frames:
        r = results.get(f["file"])
        if r is None or r.get("parse_error"):
            gates["schema_valid"] = False
            continue
        cls = f["class"]
        if cls == "absent" and r["match"]:
            gates["absent_zero_fp"] = False
        elif cls == "bottle":
            if not r["match"]:
                gates["bottle_detect"] = False
            elif r["identity"] not in IDENTITY_HONEST:
                gates["bottle_identity_honest"] = False
        elif cls == "controller":
            # AMENDED 2026-08-20 (PM ruling, see manifest _amendments): the
            # target string is brand-qualified, so unverified at standoff is
            # honest — the gate mirrors the bottle gate.
            if not r["match"] or r["identity"] not in IDENTITY_HONEST:
                gates["controller"] = False
        # picture: recorded, never gated
    return gates


def main() -> int:
    ap = argparse.ArgumentParser()
    here = os.path.dirname(os.path.abspath(__file__))
    ap.add_argument("--manifest", default=os.path.join(
        here, "recognition_calibration_manifest_2026-08-20.json"))
    ap.add_argument("--photo-dir", default=os.path.expanduser("~/recognitions"))
    ap.add_argument("--api-key-file",
                    default=os.path.expanduser("~/.config/synthetic/api_key"))
    ap.add_argument("--base-url", default="https://api.synthetic.new/v1")
    ap.add_argument("--model", default="syn:large:vision")
    ap.add_argument("--max-tokens", type=int, default=1500)
    ap.add_argument("--timeout", type=float, default=60.0)
    ap.add_argument("--out-dir", default=os.path.expanduser("~/recognition_calibration"))
    ap.add_argument("--label", default="", help="short note for this iteration")
    args = ap.parse_args()

    with open(args.manifest) as fh:
        manifest = json.load(fh)
    frames, budget = manifest["frames"], int(manifest["iteration_budget"])

    os.makedirs(args.out_dir, exist_ok=True)
    ledger_path = os.path.join(args.out_dir, "iterations.jsonl")
    spent = 0
    if os.path.exists(ledger_path):
        with open(ledger_path) as fh:
            spent = sum(1 for line in fh if line.strip())
    if spent >= budget:
        print(f"REFUSED: iteration budget spent ({spent}/{budget}) — PIN 2: "
              f"the round returns to the PM, it does not warm-iterate.")
        return 2

    key = open(args.api_key_file).read().strip()
    results, ledger_lines = {}, []
    for f in frames:
        path = os.path.join(args.photo_dir, f["file"])
        with open(path, "rb") as fh:
            jpeg = fh.read()
        try:
            reply = query_vlm(args.base_url.rstrip("/"), key, args.model,
                              build_prompt(f["target"]), jpeg,
                              max_tokens=args.max_tokens, timeout=args.timeout,
                              json_mode=True)
            parsed = parse_recognition_reply(reply)
            results[f["file"]] = parsed
            line = (f"{f['class']:<10} {f['file'][-10:-4]} match={parsed['match']!s:<5} "
                    f"identity={parsed['identity']!s:<10} conf={parsed['confidence']:.2f} "
                    f"{parsed['description'][:60]}")
        except ValueError as exc:
            results[f["file"]] = {"parse_error": str(exc)}
            line = f"{f['class']:<10} {f['file'][-10:-4]} PARSE ERROR: {exc}"
        print(line, flush=True)
        ledger_lines.append(line)

    gates = evaluate(frames, results)
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    record = {"stamp": stamp, "iteration": spent + 1, "label": args.label,
              "model": args.model, "gates": gates,
              "passed": all(gates.values()), "results": results}
    out_path = os.path.join(args.out_dir, f"iteration_{spent + 1}_{stamp}.json")
    with open(out_path, "w") as fh:
        json.dump(record, fh, indent=1)
    with open(ledger_path, "a") as fh:
        fh.write(json.dumps({"stamp": stamp, "iteration": spent + 1,
                             "label": args.label, "gates": gates,
                             "passed": all(gates.values())}) + "\n")

    print(f"\n=== iteration {spent + 1}/{budget} ===")
    for name, ok in gates.items():
        print(f"  {name:<24} {'PASS' if ok else 'FAIL'}")
    print(f"{'ALL GATES PASS' if all(gates.values()) else 'ITERATION FAILS'} "
          f"— record: {out_path}")
    return 0 if all(gates.values()) else 1


if __name__ == "__main__":
    sys.exit(main())
