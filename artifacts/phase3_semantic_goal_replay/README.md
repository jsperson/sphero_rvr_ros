# Phase 3 semantic-goal replay evidence

`fixture.json` drives the deterministic recorded-map acceptance evaluator. It
contains the recorded latency profile, canonical long legs, honest short hop,
semantic track, false authority flags, and physical-phase carryovers.

`oauth_smoke.json` records one separate, tool-disabled structured-output call.
It contains no credential, image, live sensor, ROS, serial, or motor data. The
single latency is not a replacement p95 and is not proof of continuous
physical operation.

Run:

```bash
PYTHONPATH=src python3 -m \
  sphero_rvr_driver.hierarchical_phase3_replay_validation \
  artifacts/phase3_semantic_goal_replay/fixture.json
```
