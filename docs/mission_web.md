# Mock/replay mission web console

`src/sphero_rvr_driver/mission_web.py` is the first integrated map-driven browser slice for the rover product. It combines natural-language mission entry, typed prompt-drive proposal review, explicit digest-bound simulation approval, mission and safety state, event history, and a fixture-backed room map in one responsive page.

This slice is intentionally local and mock/replay-only. It does not connect to ROS, serial devices, the rover, OpenAI, Codex OAuth, or a live mission executor.

## Run locally

From an editable development install:

```bash
python -m sphero_rvr_driver.mission_web --host 127.0.0.1 --port 8765
```

Or use the installed console command:

```bash
rvr_mission_web --host 127.0.0.1 --port 8765
```

Open `http://127.0.0.1:8765/`. The server rejects non-loopback bind addresses in this slice.

## Demonstrated flow

1. Enter a natural-language mission and choose a deterministic replay outcome.
2. The server uses the existing `PromptDrivePlanner` to validate a typed proposal or rejection.
3. The page shows provider/model identity, summary, bounded segments, trusted motion limits, and the full proposal digest.
4. An executable proposal remains in `PROPOSED` until the operator types the exact `APPROVE <digest>` phrase.
5. The existing `approved_live_route()` contract verifies the unchanged digest. The resulting route is discarded: the mock adapter has no executor.
6. The server-owned fixture advances through progress and a truthful terminal state while the map, safety strip, and event history update.

The seven fixture outcomes are:

- successful completion;
- model rejection;
- cancellation;
- STOP;
- ESTOP;
- collision blocked;
- stale telemetry blocked.

## Typed service boundary

The browser talks to `MissionWebAdapter`, not to ROS or motor endpoints:

```text
browser
  -> GET/POST /api/web/*
  -> MissionWebAdapter
  -> MockReplayMissionAdapter (this slice)
  -> PromptDrivePlanner + mission_api.v2 state vocabulary
```

Routes:

```text
GET  /api/web/state
GET  /api/web/scenarios
POST /api/web/mission/propose
POST /api/web/mission/approve
POST /api/web/mission/advance
POST /api/web/mission/cancel
```

The adapter reports all of these properties in every snapshot:

- `mode: mock/replay`;
- `fixture_only: true`;
- `live_execution_enabled: false`;
- `direct_ros_commands_allowed: false`;
- `credentials_accepted: false`.

Direct motor, generic write, arbitrary ROS, `/cmd_vel`, and `/cmd_vel_motor` routes are not exposed. The browser stores no credentials or mission state in local or session storage.

## Map contract

The snapshot includes a typed `map` payload with:

- map-frame bounds and rover pose;
- proposed route;
- traveled path derived from replay progress;
- obstacle fixtures;
- evidence-linked semantic object markers.

The responsive page renders these layers as an inline SVG. The map is a visualization of server-provided state; it does not infer rover state or perform planning in JavaScript.

## Future Pi adapter

A future live adapter belongs behind a Pi-hosted, authenticated mission service. It must preserve the same browser-facing contract while binding proposals, approvals, durable mission state, and observability snapshots to the canonical service. It must not place model credentials in the browser or expose ROS/motor routes.

That future integration requires a separate reviewed slice. This module deliberately provides no live adapter, no physical execution flag, and no remote bind mode.

## Validation

Focused validation:

```bash
python3 scripts/run_pytest_bounded.py --timeout 60 -- -vv tests/test_mission_web.py tests/test_package_metadata.py
```

The tests cover proposal and rejection contracts, exact approval, every fixture outcome, map layers, responsive/static safety properties, forbidden routes, and a complete loopback HTTP flow.
