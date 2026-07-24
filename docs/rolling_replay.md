# Rolling LLM replay

Milestone 1 / Stage B is the browser-visible, no-motion proof that the LLM is
the navigation-decision driver rather than a one-time fixed-route generator.

## Runtime shape

`RollingReplayMissionAdapter` persists the prompt, proposal digest, approval,
checkpoints, revisions, and terminal result through the replay-mode
`MissionService`. `RollingReplayEngine` owns a continuously ticking simulated
world. `CodexOAuthRollingIntentProvider` runs in a separate worker and returns
one structured response per fresh snapshot.

While a provider call is in flight, the engine continues to update:

- lidar-authoritative Stage 1 localization and pose;
- simulated traveled path and current bounded command;
- camera frames and detections;
- stable shoe and unknown-face tracks;
- semantic-map observations, confidence, uncertainty, and evidence IDs.

The provider output contains a goal direction, normalized steering, speed
limit, safe corridor, observation focus, viewpoint, rationale, and finite
lease. Deterministic validation binds it to the exact snapshot, checks numeric
limits, requires the clear corridor when an obstacle is present, and requires a
new viewpoint for the uncertain visual track. One lock-protected assignment
replaces the previous valid intent. Compatible nonzero intents do not pass
through a zero command.

Localization freshness, lease expiry, collision, STOP, ESTOP, and cancellation
remain independent stop paths. Stage B carries `motion_authority=false` and
`physical_execution_enabled=false` in every checkpoint and terminal result.
The module has no ROS, serial, sensor-device, or physical-adapter dependency.

Stage D retains this repeatedly revised snapshot/intent shape but uses the
separate `StageDExecutor` protocol. Its production adapter is exact-SHA gated
and default-off; Stage B rolling replay never acquires that adapter or authority.

## Real-provider demonstration

Run only on loopback:

```bash
PYTHONPATH=src python3 -m sphero_rvr_driver.mission_web \
  --mode rolling-replay \
  --host 127.0.0.1 \
  --port 8876 \
  --replay-database /tmp/rvr-stage-b.sqlite3 \
  --replay-reasoning-effort none
```

The local Codex CLI must already be authenticated with ChatGPT OAuth. The
server refuses to substitute the scripted test provider.
`ScriptedRollingIntentProvider` exists only for bounded deterministic tests.

The acceptance mission is:

> Explore this room, identify and map the shoes and any recognized people,
> inspect uncertain findings from another viewpoint, then stop safely.

The terminal result must show at least three real intent revisions, positive
motion updates during provider calls, zero artificial compatible-intent gaps,
an obstacle steering change, an uncertainty-driven observation change,
continuous object and face tracking, semantic-map updates, and deterministic
stale-localization stop.

## Evidence fields

The browser and terminal JSON expose:

- the latest versioned world snapshot and every provider decision snapshot;
- current finite leased intent and all accepted revisions/rationales;
- provider-call state and movement-update overlap;
- complete traveled path and command samples;
- camera detections and evidence-linked semantic tracks;
- obstacle, telemetry, STOP, ESTOP, and terminal state;
- concurrency and acceptance metrics.

Run focused verification through the repository's bounded runner:

```bash
python3 scripts/run_pytest_bounded.py --timeout 60 -- -vv \
  tests/test_rolling_replay.py tests/test_mission_web.py tests/test_mission_service.py
```
