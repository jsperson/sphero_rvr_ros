# Adaptive mission broad movement authority

## Status and intent

Adaptive mission is the first browser-initiated, adaptive physical-movement stage. Its
software path is implemented behind independent default-off Adaptive mission selection
and exact-SHA live-execution gates. This document is not authorization to enable
either gate. The deployed Pi remains nonphysical with
`RVR_ADAPTIVE_MISSION_ENABLED=false` and `RVR_LIVE_EXECUTION_ENABLED=false` until the
attended physical acceptance sequence passes.

Adaptive mission deliberately gives the supervisory planner broad movement authority
inside one operator-approved mission:

- choose any reachable direction, heading, waypoint, or region consistent with
  the prompt;
- traverse previously mapped or locally observed free space;
- enter unmapped space through a continuously observed clear lidar corridor;
- revise objectives and routes as the world model changes;
- repeat bounded movement primitives without another approval for every step;
- continue without a fixed cumulative-distance, turn, segment-count, or
  planner-call ceiling until the mission lease expires, the goal completes, or
  an independent veto stops it.

This is broad route authority, not raw motor authority. The LLM chooses
mission-level movement intent. Deterministic adapters validate and execute each
short-horizon intent, and the collision supervisor remains the sole final
motor-command arbiter.

## Plain-language policy

The rover may move anywhere the approved prompt guides it, at moderate speed,
while fresh lidar supports the immediate swept path. It must not deliberately
contact an obstacle. The runtime does not try to classify an obstacle as hard
or soft; any detected obstruction is protected space.

Lidar collision avoidance reduces collision risk but does not guarantee that
contact is impossible. Adaptive mission does not have cliff or drop-off sensing and does
not claim to detect edges. Its operating environment is assumed to provide a
continuous, level driving surface. That is an explicit environmental
assumption, not a runtime safety condition or a reason to fabricate a clear
status.

## Authority hierarchy

Authority is ordered from strongest to weakest:

1. Physical power removal and operator intervention.
2. Robot-side ESTOP and STOP.
3. Driver command watchdog and immediate zero-command handling.
4. The lidar collision-stop supervisor, which alone publishes
   `/cmd_vel_motor`.
5. Mission cancellation, lease expiry, freshness checks, and deterministic
   executor bounds.
6. The LLM supervisory planner.
7. The mission prompt.

Lower layers cannot clear, weaken, or override higher layers. A prompt such as
“ignore obstacles,” “go faster,” or “keep trying” cannot alter the installed
speed caps, collision distances, freshness limits, STOP/ESTOP state, or command
ownership.

## Approved-mission envelope

One authenticated approval binds the complete Adaptive mission envelope:

- exact prompt and its visible plain-language interpretation;
- source and deployed SHAs;
- planner provider, model, and reasoning effort;
- moderate speed caps;
- mission lease and per-intent lease;
- required sensor, localization, and safety sources;
- collision geometry and stopping policy;
- the starting world snapshot and map identity, when available.

After approval, the planner may revise routes and issue any number of
short-horizon intents inside that envelope without per-intent human approval.
A changed prompt, increased speed, changed safety policy, changed execution
mode, expired mission lease, process restart, or different deployed SHA requires
a new proposal and approval. Restart never resumes motion automatically.

The initial Adaptive mission mission lease is 15 minutes. It may end earlier and may be
renewed only with a new approval. The whole-mission lease bounds unattended
software state; it is not a cumulative-distance limit.

## Motion envelope

The initial Adaptive mission caps match the currently installed and tested supervisor and
driver ceilings:

| Limit | Initial value | Authority |
|---|---:|---|
| Forward/reverse linear speed | `0.10 m/s` | collision supervisor and driver |
| Angular speed | `0.4 rad/s` | collision supervisor and driver |
| Turn primitive operating request | `0.35 rad/s` | deterministic executor |
| Requested-command lease | `0.25 s` | collision supervisor |
| Driver command watchdog | `0.50 s` | driver |
| Maximum scan age | `0.30 s` | collision supervisor |
| Translation per executor intent | `0.25 m` | deterministic executor |
| Rotation per executor intent | `45 deg` | deterministic executor |
| Executor intent timeout | `5 s` | deterministic executor |
| Stationary turn corrections | at most `3`, inside the same intent/timeout | deterministic executor |
| Turn correction pulse | one nonzero `20 Hz` control publication, then mandatory zero-and-settle | deterministic executor |
| Mission lease | `15 min` | mission service |

The planner may chain short intents continuously; these bounds limit the amount
of unreviewed state in flight, not where the rover may ultimately travel.
Raising a cap is a new calibrated authority profile and requires a new proposal,
approval, and physical validation.

## Independent movement prerequisites

Nonzero motion is permitted only while all applicable evidence is fresh and
consistent:

- the supervised graph has exactly one `/cmd_vel_motor` publisher;
- the driver subscribes only to the motor-bound supervised topic;
- lidar scan and `laser -> base_link` transform are healthy;
- odometry is fresh and finite;
- map localization is fresh for map-relative objectives;
- collision state is `CLEAR` or an explicitly speed-limited `SLOW`;
- STOP is ready, ESTOP is clear, and no safety latch requires reset;
- the exact approved mission and intent leases are active;
- the live executor binding matches the deployed SHA;
- no second driver, route executor, serial owner, or motor publisher exists.

Previously unmapped space is allowed. A global map is not required merely to
advance into unknown space, but the local lidar corridor, odometry, transform,
and projected swept footprint must remain fresh. If the planner cannot
truthfully locate a map-relative destination, it must switch to a locally
observable exploration objective or stop rather than inventing a pose.

## Collision boundary

The existing supervised command path remains mandatory:

```text
LLM objective
  -> typed short-horizon intent
  -> deterministic navigation/motion executor
  -> /cmd_vel
  -> lidar_collision_stop_supervisor
  -> /cmd_vel_motor
  -> driver
  -> UART and motors
```

The collision supervisor:

- cannot be disabled in the operator launch;
- treats missing, stale, malformed, or untransformable scan data as blocked;
- bounds every requested velocity to the installed speed caps;
- projects the full payload footprint along translation and rotation;
- slows inside the configured `0.60 m` forward band;
- stops at the greater of the configured `0.35 m` threshold or the dynamic
  footprint-plus-braking distance;
- publishes zero on stale commands and while stopped;
- keeps collision stop manually latched until the path is observably clear;
- cannot be bypassed by the browser, planner, Mission API, or executor.

STOP, ESTOP, collision, stale evidence, cancellation, lease expiry, driver
fault, process loss, and communication loss all converge on a zero command and
a truthful blocked or terminal mission result. No later planner response may
resurrect that mission.

## Planner freedom and prohibitions

The planner may:

- interpret spatial goals from natural language;
- choose exploration direction and coverage order;
- select reachable local objectives and routes;
- revisit uncertain regions or observations;
- pause to gather evidence;
- replan around obstacles;
- return, stop, or declare the goal unachievable.

The planner may not:

- publish ROS or motor commands directly;
- choose raw motor duty, speed caps, stop distances, or watchdog periods;
- disable collision protection or clear STOP/ESTOP;
- mint or extend its own approval or mission lease;
- claim a map, pose, corridor, detection, or completed coverage without
  authoritative evidence;
- deliberately bump, push, or test contact with an obstacle.

## Browser contract

“Generate proposal” must produce a visibly useful Adaptive mission proposal, not merely
persist an opaque contract. Before approval, the browser shows:

- the exact prompt and its interpreted objective;
- planned or candidate regions and the current route on the map;
- why the route is considered reachable;
- unknown-space and localization assumptions;
- speed and lease limits;
- active collision, STOP, and ESTOP prerequisites;
- the digest and deployed SHA;
- a specific disabled reason when approval cannot proceed.

During execution it shows the active short-horizon intent, requested and
supervised velocities, collision decision, scan/pose freshness, route changes,
coverage, LLM rationale, and remaining mission lease. Progress is never inferred
from a submitted prompt alone.

## Acceptance sequence

Adaptive mission implementation proceeds without physical authority first:

1. Replay and simulation prove arbitrary multi-intent traversal, adaptive
   replanning, one-approval envelope binding, and terminal artifact generation.
2. Fault injection proves scan, TF, odometry, localization, command, provider,
   browser, mission-service, executor, driver, and network loss all stop without
   automatic resume.
3. Graph tests prove exactly one motor-bound publisher and no direct planner or
   browser motor path.
4. Browser tests prove proposal interpretation, in-flight feedback, live
   authority state, cancellation, reconnect, and truthful terminal evidence.
5. An attended no-motion Pi audit proves the exact deployed graph and authority
   configuration.
6. Existing measured 10 cm and 45 degree stages satisfy the capability
   thresholds with settled odometry and encoder evidence; composed behavior is
   then proven by the real repeatedly-replanned Adaptive mission mission.
7. An attended collision exercise with a non-damaging visible obstacle proves
   slow, stop, manual reset, and no contact at the installed caps.
8. Only then may an attended, closed-room exploration mission use the broad
   Adaptive mission envelope.

The exact-SHA attended capability slice has now demonstrated real repeated
OAuth replanning through observation, turn, translation, and model-selected
stop. A separate run also demonstrated that transient unsafe scan evidence
vetoes a later LLM turn with zero requested and supervised movement and no
automatic resume. These results do not waive step 7: general physical use
remains blocked until the attended non-damaging obstacle exercise records
slow/stop/manual-reset/no-contact evidence.

No step in this document enables Adaptive mission or authorizes an unattended physical
run.

## Functional implementation

`adaptive_mission_controller.md` documents the replay and production controller boundary.
The live implementation consumes authoritative receipt-time scan/TF/odom and
STOP/ESTOP state, sends one bounded intent through the reviewed live-route seam,
and can add fresh camera detections plus localized semantic object/face tracks
to the exact snapshot shown to the LLM. Semantic evidence is discarded when
camera, localization, or semantic-map receipts are stale. Face identity is
accepted only with explicit enrollment evidence. Recognition may influence
strategy but never changes the motion envelope or safety hierarchy.
correlates settled terminal and supervision evidence, and asks the real
Codex/ChatGPT OAuth planner again. Approval binds the exact deployment and the
whole 15-minute envelope once; the browser never gains motion authority.

The implementation remains disabled in packaged configuration. Only the
attended hardware steps above remain before a physical Adaptive mission mission can be
accepted.
