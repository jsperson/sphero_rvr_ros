# Milestone 6 Phase 4 evidence and integration

Phase 4 closes the replay evidence gap left by Phase 3: it runs at least four
consecutive authenticated semantic-goal decisions and measures each call's
real wall latency. The recorded-map traversal before the prefetch window is
accelerated, but the planning window advances at exactly one replay second per
wall second. This permits an honest comparison between the measured provider
latency, the 13.691-second dispatch threshold, and a 25-second long leg at
0.10 m/s without claiming physical motion.

## Evidence contract

The structured result records:

- every provider dispatch, completion, validated semantic decision, rationale,
  and server-resolved path;
- per-call wall latency plus min, p50, p95, and max for the run;
- atomic and `planning_resume` handoffs, controller-session count, and every
  replay-derived motor-zero interval;
- decisions per metre, reduction against the old 0.25 m decision cadence, and
  recorded-replay coverage change;
- a separate 0.5 m / 5 second short-hop characterization using every measured
  latency sample.

The short-hop table is deliberately counterfactual. It says exactly which
measured calls would enter `wait_planning` and for how long; it never turns a
long-leg pass into a small-room continuity claim.

## Persistence and browser evidence

Each proposal, initial goal, handoff checkpoint, and terminal result is stored
by `MissionService`. The exact mission can be reopened from the SQLite evidence
database by mission ID:

```bash
python3 -m sphero_rvr_driver.mission_web \
  --mode hierarchical-phase4-replay \
  --replay-database artifacts/phase4_real_provider_replay/evidence.sqlite3 \
  --phase4-mission-id HIERARCHICAL_PHASE4_MISSION_ID
```

The view is read-only. It renders recorded frontiers, the server-owned path,
semantic detections, selected goals and rationales, prefetch/handoff events,
latency distribution, controller sessions, pause intervals, decisions per
metre, and coverage. The terminal artifact endpoint returns the same persisted
result.

## Authority boundary

Phase 4 starts no launch file, ROS node, serial transport, driver, motion
topic, or physical executor. All persisted checkpoints require
`motion_authority=false` and `physical_execution_enabled=false`. Provider
structured output still contains semantic IDs only; deterministic code owns
geometry and revalidation.

Phase 2 accuracy remains provisional until surveyed live-sensor calibration.
Pi no-motion WFD and command-ownership evidence remain mandatory before any
physical phase. Missing drop-off sensing continues to prohibit physical
hierarchical exploration.

## Validation

Focused:

```bash
python3 scripts/run_pytest_bounded.py --timeout 60 -- -vv \
  tests/test_hierarchical_phase4_replay.py \
  tests/test_hierarchical_goal_selection.py \
  tests/test_mission_web.py \
  tests/test_driver_safety.py \
  tests/test_package_metadata.py
```

Full:

```bash
python3 scripts/run_pytest_bounded.py --timeout 90 -- -vv
```

The exact candidate SHA, real-provider measurements, browser checks, test
durations, and cleanup result are recorded in the pull-request handoff.
