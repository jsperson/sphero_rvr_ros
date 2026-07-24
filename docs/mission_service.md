# Persistent mission service

`MissionService` is the single local owner for durable Mission API execution
state. It retains session latches, cumulative runtime state, mission progress,
source/deployed SHAs, and an append-only SQLite event stream. `MissionServiceServer`
can place that owner behind a user-only Unix-domain socket.

The service also owns process-local executor bindings. Each declared tool can be
bound to a concrete cooperative executor with an execution mode, credential
namespace, heartbeat deadline, and structured health evidence. The capability
report exposes `declared`, `bound`, `healthy`, `mode`, `credential_namespace`,
`availability`, `fresh`, heartbeat age/deadline, evidence, and a fail-closed health
reason. Missing, stale, unhealthy, mode-crossed, or namespace-crossed bindings are
rejected before a mission enters `running`.

Bindings cover the registry's status telemetry; pause/cancel/STOP/ESTOP; bounded
route/progress and motion primitives; map/localization; observation/detection; and
artifact executors as those adapters are available. Separately bound replay
executors are dispatched by tool id. Live execution remains restricted to one
reviewed physical adapter authority.

`live_mission_service` is the first installed production owner. It constructs the
persistent service in live mode, binds a read-only status executor and a
route/progress executor, and consumes `/odom`, `/collision_stop/state`,
`/mission_api/v2/control_state`, and `/mission_api/v2/live_route/status`. It publishes truthful capability evidence on
`/mission_api/v2/service/status` and exposes the same payload through the Trigger
service `/mission_api/v2/service/read_status`. The node never publishes Twist,
sensor commands, or direct hardware commands. It instantiates a route-request
publisher only when the exact-SHA configuration gate enables the deterministic
route executor.

The generic route/progress executor is deliberately bound but unhealthy with
`motion_authority=false` and `route_submission_enabled=false`. The status executor
reports `NO_MOTION_OFFLINE`, `NO_MOTION_MONITORING`, or a live STOP/ESTOP/cancel
state. Missing or stale STOP/ESTOP evidence is `UNKNOWN`, never `READY` or
`CLEAR`. Missing and stale ROS inputs stay visible in evidence and never become
physical authority. `source_sha` and `deployed_sha` must be injected as ROS
parameters or `RVR_SOURCE_SHA`/`RVR_DEPLOYED_SHA`; startup fails if either is
missing.

`ExecutorBinding` objects are supplied through `executor_bindings` or
`bind_executor()`. `heartbeat_executor()` refreshes evidence without changing
authority, and `capabilities()` reports current state. The local service socket
exposes the same report through the `capabilities` operation. It also owns the
durable prompt lifecycle used by the Pi planner and web adapter: `received`,
`planning`, `proposed` or `rejected`, `approved`, `queued`, `running`, and a
truthful terminal or `recovery_required` state. Planning and execution run on
separate worker threads so status and cancellation remain available.

The packaged user-service units start this owner and the loopback web adapter, but
they do not start a rover driver, route runner, collision node, or sensor. The
installed helper deliberately does not enable either unit. Physical route
execution remains default-disabled. Setting `RVR_LIVE_EXECUTION_ENABLED=true`
installs only the existing `RosLiveRouteExecutor`, and startup rejects that
setting unless `RVR_LIVE_EXECUTION_REVIEWED_SHA` exactly matches the deployed
and running source SHA. This configuration gate does not approve a mission: the server still
requires fresh odometry, collision CLEAR, STOP READY, ESTOP CLEAR, and the
authenticated operator's exact proposal-digest phrase. Readiness is checked
before approval is persisted and again before the route is submitted.

Stage D is selected separately with `RVR_STAGE_D_ENABLED=true`. Its installed
default is false, and selection alone creates no route transport when live
execution remains false. When both reviewed gates are enabled, MissionService
owns the repeatedly replanned controller and persists the proposal, one
Tailscale-authenticated 900-second approval, every world-snapshot/intent
checkpoint, and the terminal result. The approval digest binds the prompt,
source/deployed SHA, provider identity, physical executor mode, starting
snapshot, first intent, speed and per-intent limits, lease, and installed lidar
safety policy. A restart converts approved, queued, running, or
cancel-requested Stage D work to `recovery_required`; it never recreates an
executor lease.

The LLM motion envelope is separately Pi-owned and configurable. The calibration
profile remains three motion calls, 0.5 m cumulative translation, 0.5 m per
translation, and 45 seconds at 0.08 m/s and 30 deg/s. A reviewed attended-room
profile may widen that envelope up to the hard service ceilings of eight calls,
2.0 m cumulative translation, 0.75 m per translation, and 120 seconds. These
values are loaded from `RVR_PLANNING_*`, displayed with the proposal, included
in its digest, and validated again by `PromptDriveLimits`; model output cannot
raise them or select speed, timeout, ROS, motor, credential, or safety settings.

The canonical database target is resolved and its owner lock acquired before the
database is opened or recovery runs, so symlink aliases cannot create a second
owner. The socket is mode `0600`, accepts one bounded JSON request per connection,
and has no TCP listener. Constructing the service never starts or resumes a mission.

## Restart and recovery

On startup, every nonterminal persisted mission becomes `recovery_required`; its
session cancel latch is set and `auto_resume` remains false. Proposal, approval,
invocation, and the `running` transition are committed before runtime execution.
Any exception after that transition is treated as an unproven-quiescence failure:
the mission becomes `recovery_required` and the session is latched.

Executor bindings are intentionally never stored in SQLite. Replay mode creates
process-local bindings for its explicitly supplied in-process adapter set for
backward compatibility. Live mode starts with no bindings unless the owner process
supplies fresh bindings again, so restart cannot restore physical authority or
resume motion.

The event table rejects UPDATE and DELETE. Its records retain proposal, approval,
invocation, observation, artifact, terminal reason, source SHA, and deployed SHA
evidence needed to reconstruct execution. Both SHA values are required constructor
inputs from reviewed package/build provenance; runtime working-directory Git state
is never consulted.
