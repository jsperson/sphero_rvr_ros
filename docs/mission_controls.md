# Authenticated Mission API controls and physical start gate

`src/sphero_rvr_driver/mission_controls.py` adds the VS08B control layer above the VS07 Mission API state machine and the VS03 supervised coordinator contract. It is ROS-free, dependency-free, and deliberately not a generic ROS bridge.

The controls expose only authenticated versioned Mission API actions:

```text
POST /api/mission/start
POST /api/mission/pause
POST /api/mission/cancel
```

They do not expose raw motor, generic write, arbitrary ROS-topic, or direct movement endpoints. Mission execution still follows the safe Mission API/coordinator path; the independent collision/STOP/ESTOP path remains outside the browser UI.

## Authenticated and authorized principals

Every browser/control action requires a `MissionPrincipal` with the matching permission:

- `mission:start`
- `mission:pause`
- `mission:cancel`

Missing principals and missing permissions fail closed with `MissionControlError`. Denials are still appended to `audit_log` with:

- `api_version`, currently `mission_api.v1`
- action and actor
- decision (`denied`, `allowed`, or `latched`)
- reason
- execution mode
- physical gate state
- linked Mission API event log at the time of the decision

## Physical start approval

Replay/mock controls can start without physical approval because they do not claim motor authority.

A motor-capable mission start must use `MissionExecutionMode.PHYSICAL` and include a `PhysicalStartApproval`:

- `approved_by`
- `approved_at`
- `gate_id`
- optional reason

Without that explicit gate, `MissionControlSession.start()` rejects the request and records a denied audit event. With the gate present, the session advances the Mission API state machine through `start_requested` and `validated`, preserving audit linkage to those mission events.

## Cancel and pause

`MissionControlSession.pause()` applies the Mission API `pause_requested` event. `MissionControlSession.cancel()` latches `CANCELLED` through the Mission API result path. Both require their own permissions and write audit entries linked to the Mission API event log.

Browser cancel/STOP is an operator control, not the sole safety mechanism. The robot-side STOP/ESTOP/collision supervisor remains independent and visible.

## Robot-side safety visibility

`MissionControlSession.robot_safety_event()` records independent robot-side safety transitions such as:

- `estop` -> `ESTOPPED`
- `blocked` -> `BLOCKED`

These events use actor `robot-side-supervisor` and decision `latched` in `audit_log`, making it clear that the browser UI did not own the only safety path.

## Static controls shell

`build_static_controls_bundle()` returns a minimal HTML shell with:

- Start replay mission
- Request physical start
- Pause
- Cancel / STOP mission

The shell advertises `mission_api.v1` and repeats that the robot-side STOP/ESTOP/collision supervisor remains independent. It intentionally contains no direct motor route names or generic ROS bridge hooks.
