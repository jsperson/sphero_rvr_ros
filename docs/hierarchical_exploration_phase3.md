# Milestone 6 Phase 3 replay evidence

## Verdict and scope

Phase 3 promotes the model from short motion primitives to one bounded
semantic goal at a time. Deterministic code owns geometry, map resolution,
route selection, freshness, revalidation, event priority, and all safety
decisions. This is a ROS-free recorded-map replay implementation. It does not
start Nav2, live sensors, the driver, or any physical publisher.

The implementation begins at merged Phase 2 baseline
`9ddc2b64dc9c626e07df5e4945297ffea28fdb2a`. The pull-request handoff records
the exact candidate SHA.

## Semantic goal contract

The structured response permits exactly one action:

- `go_to_frontier(frontier_id)`;
- `inspect(track_id)`;
- `search_region(region_id, target_classes)`;
- `return_to_start()`;
- `wait()`; or
- `finish(outcome, evidence_ids)`.

Every decision binds the exact mission, snapshot, decision generation, and
event generation. Selected candidate or evidence IDs must be present in the
bounded snapshot and cited in the rationale. Extra arguments fail closed.
There are no model fields for pose, route, path, velocity, acceleration,
clearance, leases, ROS names, motor commands, or code.

The OAuth adapter uses the same nested semantic contract with a Draft-07
structured-output schema and no tools. Deterministic validation still runs
after schema enforcement.

## Deterministic resolution and Next-Best-View

Frontier IDs resolve to project-owned WFD approach cells. `inspect` resolves
to a deterministic Next-Best-View sampled from free recorded-map cells. The
sampler rejects insufficient clearance, unreachable cells, occluded sight
lines, and out-of-range views, then ranks accepted poses by uncertainty gain,
view diversity, route cost, and stable identity. The model never receives or
returns the selected pose.

Before a result can enter the Phase 1 follower, it is revalidated against
mission/objective/event generation, map identity, localization freshness,
safety state, route budget, clearance, frontier/track signature, viewpoint,
and evidence availability. A map-revision-only change is accepted when the
stable target signature and all other gates still hold.

## Async prefetch and event replanning

One background decision starts when remaining ETA is at or below:

```text
recorded p95 + margin = 12.691 s + 1.000 s = 13.691 s
```

A revalidated result can extend the active Phase 1
`NavigateThroughPoses` model without restarting its controller session.
Arrival before a valid result is ready produces an explicit zero
`wait_planning` state; the replay never fabricates zero-pause continuity.
The p95 is a dispatch threshold, not a minimum hold: a real provider result is
collected as soon as its future completes. Only the deterministic acceptance
fixture applies the recorded p95 as a virtual completion time.

Stable new detections and invalidated frontiers preempt into
`wait_planning`, increment the event generation once, invalidate older
in-flight results, and dispatch a fresh decision. Confidence, stability count,
and hysteresis coalesce noisy duplicate events. STOP, ESTOP, collision state,
cancellation, stale motion evidence, and lease loss remain higher priority
than the provider. A supervisor veto produces zero even while the provider
thread is blocked.

## Quantitative replay evidence

The canonical fixture uses the committed Phase 1 recorded map and four
2.5-metre semantic legs at the `0.10 m/s` envelope:

| Gate | Result |
|---|---:|
| Modeled distance | `10.0 m` |
| Semantic motion decisions | `4` (`0.4 decisions/m`) |
| Legacy 0.25-m primitive loop | `4.0 decisions/m` |
| Decision reduction | `10×` |
| Atomic long-leg handoffs | `3` |
| Controller sessions across long legs | `1` |
| Short-hop arrival | `5.0 s`, zero `wait_planning` |
| Short-hop modeled result readiness | `12.691 s`, then session `2` |

The deterministic acceptance run uses real background futures with a scripted
provider and a virtual replay clock. It proves concurrency, stale-result
discard, handoff, event, and safety semantics; it does not claim that four
live model calls were completed under the recorded p95.

A separate tool-disabled OAuth smoke used `gpt-5.6-sol` at low reasoning and
returned a valid snapshot-bound `inspect(shoe-track-01)` decision. Its single
observed end-to-end latency was `11.019812 s`. One sample does not replace the
recorded latency distribution, but it reinforces the carryover: small-room
short hops can still pause, and deeper lookahead or lower provider latency is
needed before claiming continuous physical exploration.

## Safety and carryovers

The existing ownership chain is unchanged:

```text
Nav2 private request -> live_route_runner -> /cmd_vel
  -> collision_stop_node -> /cmd_vel_motor
```

Phase 3 changes no launch, Nav2 bridge, supervisor, or physical execution
default. The canonical evaluator explicitly reports all authority flags false.

Before any physical phase:

- repeat and retain Raspberry Pi no-motion WFD and command-ownership evidence;
- recalibrate Phase 2 localization tolerances against surveyed physical
  ground truth; and
- add a credible drop-off sensing boundary.

Until then, physical hierarchical exploration remains prohibited.

## Validation

Run the focused suite through the repository's bounded verbose runner:

```bash
python3 scripts/run_pytest_bounded.py --timeout 60 -- -vv \
  tests/test_hierarchical_goal_selection.py \
  tests/test_hierarchical_exploration.py
```

Run the committed replay evaluator:

```bash
PYTHONPATH=src python3 -m \
  sphero_rvr_driver.hierarchical_phase3_replay_validation \
  artifacts/phase3_semantic_goal_replay/fixture.json
```

The evaluator exits nonzero if any semantic-schema, long-leg handoff,
decision-reduction, short-hop honesty, event-replan, stale-result,
supervisor-veto, authority, or carryover gate fails.
