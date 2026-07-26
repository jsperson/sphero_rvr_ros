# Adaptive mission closed-loop controller

`sphero_rvr_driver.adaptive_mission_controller` is the first functional Adaptive mission
vertical slice. It runs against a replay executor by default and keeps
`live_execution_enabled=false`, `physical_execution_enabled=false`, and
`motion_authority=false`. The controller does not import ROS or expose a
publisher. The production service can now bind the same controller to the
disabled-by-default `PhysicalAdaptiveMissionExecutor`.

## Closed-loop boundary

```text
prompt
  -> supervised persistent Codex app-server using ChatGPT OAuth
  -> fresh ephemeral thread with compact typed world evidence
  -> one move_distance / turn_angle / observe / stop intent
  -> deterministic snapshot, type, lease, and magnitude validation
  -> AdaptiveMissionExecutor
  -> collision-supervisor decision
  -> requested versus supervised movement evidence
  -> updated typed world snapshot
  -> Codex chooses again
```

The `AdaptiveMissionExecutor` protocol is the physical-integration seam.
`ReplayAdaptiveMissionExecutor` uses it without ROS or hardware.
`PhysicalAdaptiveMissionExecutor` consumes the mission node's receipt-time scan/TF,
odometry, collision, STOP, and ESTOP evidence and maps exactly one movement
intent to one `LiveRouteRequest`. `RosLiveRouteExecutor` submits that request to
`live_route_runner`, which is the bounded `/cmd_vel` publisher. The
`lidar_collision_stop_supervisor` remains the sole `/cmd_vel_motor` publisher.
The adapter accepts completion only when route, source SHA, segment correlation,
settlement, measurements, and requested-versus-supervised evidence agree.

The OAuth provider owns one supervised `codex app-server` process and starts a
new ephemeral thread for every snapshot-bound decision. It never reuses model
conversation state. App-server startup verifies that the Codex account type is
ChatGPT; credentials remain in Codex's persisted OAuth session and are neither
read nor logged by the rover. The child process receives a minimal allowlist of
non-secret runtime variables. A timeout, cancellation, malformed schema,
authentication mismatch, or unrecoverable server failure fails closed. A
crashed server may be restarted once within the original decision deadline.

Every planning cycle emits
`sphero_rvr.adaptive_planning_latency.v1` metrics for prompt/image preparation,
OAuth client startup, inference, deterministic validation, and total latency.
The record includes only model/runtime identifiers, snapshot ID, input length,
image attachment reason, restart count, and status; it contains no credential
material.

The model input is a typed decision projection rather than the complete
executor record. It retains STOP/ESTOP and collision state, source freshness,
scan/odometry age, drop-off-detection unavailability, signed translation
clearance, pose, progress, last bounded intent and navigation outcome, camera
freshness, and camera detections. Bulky route samples, duplicate recognition
records, receipt timestamps, and local image paths are excluded. Image pixels
are attached only for a recoverable stall/collision decision or an
object-directed objective. Every such decision requires a fresh frame- and
SHA-bound image and otherwise fails closed.

## Recorded-snapshot latency benchmark

The no-motion benchmark uses three cases from recorded real mission
`adaptive-mission-live-39f168fbc4a748609a6a80b48a76b247`: clear exploration,
the fixed-obstacle recoverable stall, and an object-directed shoe objective.
Each case runs three times. Success requires schema validity, a second
deterministic validation producing an identical intent, objective progress,
safe obstacle behavior, and an image attachment for both recovery and
object-directed decisions.

The high-effort Sol legacy baseline at runtime SHA
`ef2ca33d` used:

```bash
PYTHONPATH=src python3 scripts/rvr_adaptive_planner_benchmark.py \
  --mission-id adaptive-mission-live-39f168fbc4a748609a6a80b48a76b247 \
  --integration exec --legacy-full-input --reasoning-effort high \
  --model gpt-5.6-sol --repetitions 3 --timeout 60
```

All nine baseline decisions passed. Total latency was 13,400 ms p50 and
22,464 ms p95. The first fixed-obstacle recovery took 19,358 ms.

The final compact-input comparison at runtime SHA `a803d81e` used:

```bash
PYTHONPATH=src python3 scripts/rvr_adaptive_planner_benchmark.py \
  --mission-id adaptive-mission-live-39f168fbc4a748609a6a80b48a76b247 \
  --integration app-server --reasoning-effort low \
  --model gpt-5.6-sol --model gpt-5.6-terra --model gpt-5.6-luna \
  --repetitions 3 --timeout 60
```

| Model | Total p50 | Total p95 | Valid decisions |
|---|---:|---:|---:|
| Sol | 8,804 ms | 16,033 ms | 9/9 |
| Terra | 7,267 ms | 10,994 ms | 9/9 |
| Luna | 6,567 ms | 12,691 ms | 9/9 |

The deterministic selector chooses the lowest valid p50 and then p95, so
Adaptive mission uses Luna at low effort. This reduces total p50 by 51.0% and
p95 by 43.5% from the high-Sol ephemeral baseline. The same set covers initial
objective interpretation and did not justify a separate stronger model for
that first decision. Sol remains the general Prompt Drive default; the
Adaptive mission service selects Luna explicitly.

## Approval and limits

The proposal digest binds the prompt and its interpretation, source and
deployed SHAs, provider/model/reasoning effort, executor mode, starting
snapshot, first intent, safety policy, mission lease, and all limits. One
server-authenticated operator approval starts the complete mission; later
revisions do not require per-intent approval. In served mode the HTTP boundary
accepts the operator only from the `Tailscale-User-Login` identity injected by
Tailscale Serve, records `tailscale-serve` as its authentication source, and
binds that principal to the persisted proposal digest. A client-supplied
identity header on an ordinary loopback server is not trusted.

The mission lease defaults to 900 seconds and the deployment configures that
default/maximum with `RVR_ADAPTIVE_MISSION_LEASE_S` up to the reviewed
900-second ceiling. The browser may select a positive duration no greater than
that maximum. The selected
value is bound into the proposal digest, approval expiration, provider
authority, UI, and persisted result. There is no cumulative translation,
rotation, intent-count, or provider-call budget inside the active lease. Every
individual intent remains bounded to 0.25 m, 45 degrees, and 5 seconds. Speed
ceilings remain 0.10 m/s and 0.4 rad/s.

Approval starts the fixed supervised graph, which owns camera/lidar telemetry
for the whole active lease and across every replan. The browser cannot toggle
telemetry independently while that lease owns the graph. A model `stop` ends
movement for the current objective but leaves the approved session safely idle,
with telemetry running and ready for another objective. Lease expiry, explicit
cancellation, a safety terminal, or restart recovery ends the session and
verifies graph and telemetry shutdown.

The approving operator may submit a new objective while the lease is active.
This updates the running controller at a replanning boundary without creating
a second lease, changing the original expiry, restarting the graph, or
interrupting telemetry. An in-flight response for an older objective is
discarded and fresh typed evidence is reacquired before another model call.

Hardware STOP, ESTOP, collision veto, stale evidence, cancellation, executor timeout,
mission-lease expiry, persistence failure, process restart, and provider or
network failure are terminal. The controller never automatically resumes.
Restart recovery is owned by `MissionService`, which changes a persisted
approved or running mission to `recovery_required`.

Lidar is the collision model. Drop-offs remain outside that model; the world
snapshot explicitly reports that drop-off detection is unavailable.

## Moving semantic perception

Adaptive mission reuses the stationary camera/SLAM semantic producer without giving that
producer motion authority. `adaptive_mission_perception.launch.py` composes real lidar,
camera, moving `slam_toolbox`, and semantic tracking with the existing
collision-supervised rover graph. Its default is `start_rvr:=false`; physical
movement additionally requires the reviewed deployment gates and an explicit
`start_rvr:=true start_live_route_runner:=true` attended launch.

The physical executor projects fresh evidence into every planner snapshot:

- camera detections and frame identity;
- localized object and face tracks with map coordinates and uncertainty;
- explicitly enrolled face identities and separate unknown-face tracks;
- camera, semantic-map, and localization freshness plus revision provenance.

Localized tracks are withheld unless all three receipts are fresh and each
track has a recent producer-owned `last_seen_s`. Semantic evidence uses a
separate one-second objective-evidence deadline appropriate for the 3 Hz camera;
the stricter 0.30-second scan/TF/odom movement gates are unchanged. The moving
producer refreshes its map-to-rover TF pose on each processed camera frame so
track coordinates do not wait for the slower occupancy-map publication. A face
name is downgraded to `unknown` unless
`recognized_from_enrollment=true` and `enrollment_evidence_ids` is nonempty.
These fields can influence the LLM's choice of `observe`, turn direction, and
bounded movement, but they never authorize motion or override lidar, STOP,
ESTOP, lease, or executor validation.
The replay test exercises `observe → turn_angle → move_distance → stop` from a
shoe track that appears in a fresh post-observation snapshot.

## Real-provider replay

Run the browser slice behind authenticated Tailscale HTTPS:

```bash
PYTHONPATH=src python3 -m sphero_rvr_driver.mission_web \
  --mode adaptive-mission-replay \
  --host 127.0.0.1 \
  --port 8877 \
  --public-origin https://sphero-pi-2.example-tailnet.ts.net \
  --replay-database /tmp/rvr-adaptive-mission.sqlite3 \
  --replay-reasoning-effort low
```

The Codex CLI must report `Logged in using ChatGPT`. Generate a proposal,
review the interpreted objective and first intent, enter the displayed
digest-bound phrase once, and watch each updated snapshot, rationale, requested
movement, supervised movement, and terminal outcome.

For repository browser-harness testing only, omit the public origin and add
`--allow-loopback-test-approval`. That mode remains replay-only, records
`explicit-loopback-test-mode` rather than a production authentication source,
and must not be exposed through a served operator console.

The no-motion OAuth smoke uses the same controller and executor boundaries with
`motion_permitted=false`:

```bash
PYTHONPATH=src python3 scripts/rvr_adaptive_mission_oauth_smoke.py \
  --reasoning-effort low
```

The smoke requires an `observe` followed by `stop`, verifies that no movement
intent was accepted, and prints provider/model, proposal digest, calls, and
authority flags.

The adaptive acceptance run lets the real model choose the order and magnitude
of translation, rotation, and observation from each updated snapshot, then
requires a model-selected stop:

```bash
PYTHONPATH=src python3 scripts/rvr_adaptive_mission_oauth_replay.py \
  --reasoning-effort low
```

It succeeds only after at least four real provider decisions, nonzero
translation and rotation, an explicit observation, distinct world snapshots,
and a final `stop`, while both physical authority flags remain false.

## Physical integration and remaining acceptance

The live mission service now implements that binding, including authenticated
Tailscale approval, exact source/deployed/reviewed SHA equality, durable
checkpoints, correlated cancellation, and restart-to-`recovery_required`.
`RVR_ADAPTIVE_MISSION_ENABLED=false`, `RVR_LIVE_EXECUTION_ENABLED=false`, and a blank
reviewed SHA remain the installed defaults; merely installing this code grants
no motion authority.

The deterministic movement primitive uses separately measured stopping
horizons. Translation reserves 0.25 seconds and bounds its rate estimate at the
requested speed. Rotation reserves 0.10 seconds, caps the primitive request at
the lowest demonstrated breakaway request of 0.35 rad/s, and bounds
authoritative measured yaw progress with a reviewed 3.5 rad/s estimator
ceiling. The installed raw-motor mapping produced 3.21 rad/s of measured yaw
from that request; reusing the translation horizon then produced a truthful
30.506-degree undershoot. Requests of 0.25 and 0.15 rad/s both stalled. The
turn controller now retains the latest measured rate across the intervening
20 Hz control tick so it can issue zero between 10 Hz odometry samples. The
stale-odometry gate bounds that projection. A later exact-SHA trace settled
8.330 degrees short because the drivetrain response was materially slower.
After verified stationary evidence, the same correlated turn intent may
therefore issue at most three single-control-tick corrective pulses within its
original 5-second timeout. Overshoot, collision, STOP/ESTOP, cancellation, stale evidence,
timeout, or another terminal result can never re-engage motion. The terminal
manifest reports the correction count. The collision supervisor and driver
still own the absolute 0.4 rad/s ceiling. The rover must become stationary and
finish within the 0.03 m translation or capability-oriented 10-degree turn
terminal bound. The wider turn bound is not a collision or safety tolerance.

The exact-SHA `45ae1adc3942850bd43b9a31679869d3339d309a` attended
capability run completed the real physical loop with one authenticated
approval and four independent OAuth provider calls:
`observe → turn_angle(30°) → move_distance(0.10 m) → stop`. The turn settled
at 26.133 degrees and the translation at 0.117339 m. Both route terminals were
stationary, collision `CLEAR`, and within the capability bounds; requested and
supervised motion agreed. The turn crossed a floor seam, so its magnitude is
capability evidence rather than a calibration measurement. The final progress
recorded 26.133 degrees rotation, 0.117339 m translation, one observation, and
`planner_stop`. Independent STOP, two zero motor samples, unchanged encoder
samples, relock, and restart-preserved completion all passed. The sealed
evidence is under
`/home/jsperson/rvr_runs/adaptive_mission_capability_retry_45ae1ad_20260724T204000Z`.

The immediately preceding exact-SHA attempt independently proved the veto path.
After `observe` and a completed 0.194449 m translation, the real provider chose
a 30-degree turn. The collision supervisor reported `SENSOR_STALE` with
`rear_unknown`; submission was blocked with requested and supervised motion
both zero, and the mission never resumed when the sector later returned
`CLEAR`. Its sealed evidence is under
`/home/jsperson/rvr_runs/adaptive_mission_capability_45ae1ad_20260724T203000Z`.

Remaining physical acceptance is the attended non-damaging obstacle exercise
for collision slow/stop/manual-reset/no-contact evidence, followed by an
attended moving-perception mission proving that real camera/SLAM tracks remain
fresh and influence real OAuth revisions while the rover moves. The
deterministic semantic-fusion, collision, and stale-veto suites already pass;
the physical gates in `adaptive_mission_authority.md` still apply.
