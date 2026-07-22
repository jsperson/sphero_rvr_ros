# Product direction

## End state

`sphero_rvr_ros` is evolving from a ROS driver and replay toolkit into an LLM-supervised semantic rover product.

The target user experience is a map-driven web application with text interaction. An operator can see the rover and its known environment, submit a mission in ordinary language, watch the plan and evidence update on the map, redirect or stop the mission, and inspect the final semantic map and coverage report.

Representative missions include:

- map a room or update an existing map;
- identify supported objects and place evidence-linked markers on the map;
- search for a particular object class or instance;
- revisit uncertain observations from another viewpoint;
- navigate between selected points or regions while avoiding obstacles;
- return to the starting location or stop safely when progress cannot be proven.

## System shape

```text
map-centric web UI + text conversation
  -> persistent mission and conversation service
  -> bounded LLM supervisory planner
  -> typed world state and capability registry
  -> policy, approval, budget, and reachability validation
  -> deterministic mapping/navigation/perception executors
  -> independent collision, STOP, and ESTOP boundary
  -> rover and sensors
```

The LLM selects mission-level objectives: which reachable region to explore, when to inspect, whether to revisit an uncertain object, when coverage is sufficient, and when to return or stop. It does not publish motor commands, clear ESTOP, mint approval, expand budgets, or claim unobserved coverage.

## Delivery sequence

1. Prove the core hypothesis: a real LLM driving the physical rover from a text prompt through typed bounded tools, explicit operator approval, measured execution, and the existing collision boundary.
2. Deploy one persistent mission owner with truthful live status.
3. Add bounded durable submit/status/cancel clients.
4. Add adaptive LLM exploration over independently generated and validated candidate objectives.
5. Bind timestamped observation, useful object detection, map projection, tracking, and searched-coverage accounting.
6. Complete a shoe-mapping vertical slice in replay and supervised physical operation.
7. Deliver the live map-driven conversational web interface.
8. Prove generality with point-to-point navigation and a second semantic mission.

Shoe mapping is the first acceptance slice, not a hard-coded product boundary. Object class, world state, navigation objective, evidence, and artifact generation must remain typed and replaceable.

## Current implementation focus

The repository now integrates the real Pi-local ChatGPT OAuth planner, typed
`move_distance` and `turn_angle` proposals, durable mission lifecycle, exact
digest-bound approval records, truthful live status, and a map-driven browser
adapter. The Pi deployment is intentionally proposal-only: it has no physical
executor authority and does not substitute replay fixtures for missing live map
or sensor evidence.

The current acceptance step is the exact-SHA no-motion Pi deployment and real
browser-to-OAuth proposal path. The next separately gated step is measured
physical validation: resolve route-local versus absolute-odometry evidence, then
perform restrained 10 cm, 45-degree, and multi-step stages with fresh operator
approval for each exact proposal digest.

## Delivery discipline

- Keep one implementation package active at a time.
- End each package with executable or physical acceptance evidence.
- Reopen security architecture only when authority or an external control boundary changes.
- Use focused bounded tests during iteration; treat a full-suite timeout as test infrastructure, not a product failure.
- Preserve truthful partial, blocked, cancelled, stopped, and estopped outcomes instead of forcing nominal completion.
