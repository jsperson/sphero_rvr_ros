# Persistent mission service

`MissionService` is the single local owner for durable Mission API execution
state. It retains session latches, cumulative runtime state, mission progress,
source/deployed SHAs, and an append-only SQLite event stream. `MissionServiceServer`
can place that owner behind a user-only Unix-domain socket.

This bounded slice provides the service lifecycle and state store only. Executor
health/capability binding, MCP/planner integration, user-facing client/CLI,
packaged daemon entry points, ROS launch, and deployment are intentionally deferred
to successor tasks.

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

The event table rejects UPDATE and DELETE. Its records retain proposal, approval,
invocation, observation, artifact, terminal reason, source SHA, and deployed SHA
evidence needed to reconstruct execution. Both SHA values are required constructor
inputs from reviewed package/build provenance; runtime working-directory Git state
is never consulted.
