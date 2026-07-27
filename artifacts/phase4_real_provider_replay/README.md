# Phase 4 real-provider replay evidence

This directory contains the committed Milestone 6 Phase 4 evidence:

- `report.json` is the reviewable structured result.
- `evidence.sqlite3` is the durable `MissionService` database. The exact
  `mission_id` in `report.json` reopens the same result in the web console.

The replay uses the recorded Phase 1 SLAM map and the authenticated, toolless
semantic-goal provider. It has no ROS graph, serial access, live sensors, motor
publisher, or physical authority. Provider wall latency is real. Motion,
coverage, path, and motor-zero intervals are replay-derived.

Regenerate only from a clean reviewed source SHA:

```bash
python3 -m sphero_rvr_driver.hierarchical_phase4_replay \
  artifacts/phase3_semantic_goal_replay/fixture.json \
  --database artifacts/phase4_real_provider_replay/evidence.sqlite3 \
  --output artifacts/phase4_real_provider_replay/report.json \
  --source-sha "$(git rev-parse HEAD)" \
  --reasoning-effort low
```

Reopen the committed mission by the exact ID recorded in `report.json`:

```bash
python3 -m sphero_rvr_driver.mission_web \
  --mode hierarchical-phase4-replay \
  --replay-database artifacts/phase4_real_provider_replay/evidence.sqlite3 \
  --phase4-mission-id hierarchical-phase4-real-20260726
```

The committed run executed source SHA
`72a63c249a61327f0e35123220ece95aaf64b6ad` with `gpt-5.6-sol` at low
reasoning effort. Four calls completed. Latency was 7.929764–14.545232 seconds,
with p50 10.768940 seconds and four-sample interpolated p95 14.348098 seconds.
All three 25-second long-leg handoffs were atomic in one controller session.
All four samples would require `wait_planning` on the 0.5 m / 5 second short
hop, with replay-derived motor-zero intervals of 2.929764–9.545232 seconds.
