# Sphero RVR notification/event capability audit

This audit defines the unsolicited packet semantics needed before adding dispatcher notification routing. It is based on `sphero-sdk==0.3.4.post2`, especially `SpheroRvrAsync`, generated command modules, and the current repo packet/dispatcher/driver code.

## Protocol model for unsolicited notifications

Official SDK notification callbacks are registered with the serial DAL through `on_command(did, cid, target, handler, outputs=...)`, not through response matching. The SDK handler keys callback workers as `(did, cid, msg.source)`. Treat RVR notifications as unsolicited **command packets** from the robot:

- `is_response`: false.
- `requests_response`: false in normal use; the SDK wrapper can return success internally, but this driver should not send an ACK unless hardware testing proves the robot requires one.
- `has_source`: true; `source` is the robot component that emitted the event (`BT=1` or `MCU=2`).
- `has_target`: normally false for device-to-host notifications. If hardware sends both target and source, demux must still key on `(did, cid, source)`.
- `device_id` / `command_id`: event DID/CID from the tables below.
- `sequence_id`: not useful for matching. Do not require a pending request with the same sequence.
- `payload`: event-specific byte payload parsed by `responses.py`-style typed parsers.

Current `Dispatcher._read_loop()` only checks `_pending[(did, cid, seq)]`; unmatched packets are silently ignored. Notification support should add a callback/subscriber registry keyed by `(did, cid, source)` with optional `(did, cid, None)` fallback, then route unmatched command packets to that registry before dropping/logging.

## Cross-cutting implementation semantics

### Enable, subscribe, unsubscribe

Use separate concepts:

1. **Enable command**: sends a firmware command such as `enable_motor_fault_notify(True)`. This changes robot-side event emission.
2. **Local subscription**: registers a Python callback / ROS publisher path for unsolicited packets. This changes only host-side routing.
3. **Unsubscribe/disable policy**: when the last local subscriber leaves, send the paired enable command with `False` for firmware-gated events. For sleep events and other non-gated notifications, only unregister the local callback.

Recommended core API shape:

- `enable_*_notify(enabled=True)` remains a command method.
- `subscribe_*_notify(callback) -> unsubscribe_handle` registers callbacks.
- ROS event publishers should lazily enable firmware notifications on first subscriber and disable on last subscriber where an enable command exists.
- `disconnect()` must disable high-rate or safety-relevant firmware events best-effort, unregister callbacks, then stop the dispatcher.

### Parser and state-cache behavior

- Parsers belong in `src/sphero_rvr_core/responses.py` and should return frozen dataclasses for multi-field payloads.
- Dispatch should parse before invoking typed driver callbacks; parse failures should be logged/dropped without crashing the reader loop.
- Maintain a latest-value cache for stateful safety/diagnostic events: battery voltage state, motor stall, motor fault, color detection, IR receive code, and motor thermal protection.
- Do not cache raw streaming service bytes by default. They are high-rate and token-scoped; publish or deliver them to subscribers and drop if nobody is listening.
- Callback failures must be isolated. One bad subscriber should not kill the dispatcher or starve other subscribers.
- Backpressure policy: bounded per-event queues for ROS/high-rate events. Drop oldest or latest explicitly; never let streaming/color events grow unbounded.

### ROS topic guidance

Proposed topics are intentionally narrow and typed:

- `/rvr/events/battery_voltage_state`
- `/rvr/events/motor_stall`
- `/rvr/events/motor_fault`
- `/rvr/events/gyro_max`
- `/rvr/events/ir_message`
- `/rvr/events/color_detection`
- `/rvr/events/motor_thermal_protection`
- `/rvr/events/sleep` for will/did sleep lifecycle notifications

Avoid exposing raw streaming bytes as a generic ROS topic until a typed streaming configuration exists; otherwise it becomes a firehose with a cute robot attached.

## Notification/event matrix

| Event | Enable/disable command | Event packet `(DID,CID,source)` | Payload parser shape | State cache / topic / subscriber behavior | Fake transport fixtures | Safety implications |
|---|---|---|---|---|---|---|
| Will sleep | No firmware enable in SDK. Register local callback with `on_will_sleep_notify(handler)`; unregister locally to disable host callback. | Power `DID=0x13`, `CID=0x19`, `source=BT(1)`. No pending response sequence. | Empty payload -> `SleepEvent(kind="will_sleep")` or enum/string marker. | Cache latest sleep lifecycle event timestamp; publish `/rvr/events/sleep` with phase `will_sleep`; notify subscribers even if no state poll is active. | Inject unsolicited `Packet(DID_POWER,0x19,seq,payload=b"",source=TARGET_BT,flags=FLAG_HAS_SOURCE).encode()` while no request is pending; assert callback fires. | Robot may stop responding soon. Driver should stop velocity/control loops and mark state degraded before sleep. |
| Did sleep | No firmware enable in SDK. Register local callback with `on_did_sleep_notify(handler)`; unregister locally to disable. | Power `0x13`, `CID=0x1A`, `source=BT(1)`. | Empty payload -> `SleepEvent(kind="did_sleep")`. | Cache sleeping/offline status; publish `/rvr/events/sleep` with phase `did_sleep`; trigger reconnect/wake policy only if configured. | Inject unsolicited power `0x1A` from BT and verify no pending request is required. | Treat as connection/safety boundary: stop motors in local desired state, suspend polling, avoid command spam until wake/reconnect. |
| Battery voltage state change | `enable_battery_voltage_state_change_notify(is_enabled)` sends Power `DID=0x13`, `CID=0x1B`, `target=BT(1)`, payload `isEnabled:bool`. | Power `0x13`, `CID=0x1C`, `source=BT(1)`. | Reuse `parse_battery_voltage_state(payload)` -> `BatteryVoltageState(state:uint8,state_name)`. | Cache latest voltage state; publish diagnostics/event topic; subscribers can choose to auto-stop or warn on low/critical. Disable firmware notify when no subscribers remain. | Assert enable writes payload `01`/`00`; inject `payload=b"\x02"`/`b"\x03"` event and verify state cache and callback. | Low/critical battery can brown out motors and serial. Critical should suppress new motion commands unless policy explicitly allows. |
| Motor stall | `enable_motor_stall_notify(is_enabled)` sends Drive `DID=0x16`, `CID=0x25`, `target=MCU(2)`, payload `isEnabled:bool`. | Drive `0x16`, `CID=0x26`, `source=MCU(2)`. | `MotorStallEvent(motor_index:uint8, is_triggered:bool)` from 2 bytes. | Cache per-motor stall state; publish `/rvr/events/motor_stall`; callback should include edge-triggered true/false transitions. Disable on last subscriber. | Enable write fixture; inject `payload=bytes([motor, triggered])`; verify request/response packets still match while event is routed. | Stall can mean blocked tread or overloaded drivetrain. On trigger, default safety should stop or cap velocity until cleared. |
| Motor fault | `enable_motor_fault_notify(is_enabled)` sends Drive `DID=0x16`, `CID=0x27`, `target=MCU(2)`, payload `isEnabled:bool`. | Drive `0x16`, `CID=0x28`, `source=MCU(2)`. | `MotorFaultEvent(is_fault:bool)`; can mirror `parse_motor_fault_state`. | Cache current fault state; publish `/rvr/events/motor_fault`; update diagnostics. Disable on last subscriber. | Enable write fixture; inject `payload=b"\x01"` then `b"\x00"`; assert callback/state transitions. | Fault=true is safety-critical. Stop motion immediately, reject velocity commands while faulted unless an explicit recovery path clears policy. |
| Gyro max | `enable_gyro_max_notify(is_enabled)` sends Sensor `DID=0x18`, `CID=0x0F`, `target=MCU(2)`, payload `isEnabled:bool`. | Sensor `0x18`, `CID=0x10`, `source=MCU(2)`. | `GyroMaxEvent(flags:uint8)` bitmask; keep raw flags until axis bit meanings are verified. | Cache last event timestamp/flags; publish `/rvr/events/gyro_max`; usually diagnostic only. Disable on last subscriber. | Enable write fixture; inject `payload=b"\x07"`; verify parser preserves bitmask. | Indicates rotation exceeded sensor range; odometry/heading estimates may be invalid. Consider resetting/flagging localization, not necessarily stopping motors. |
| Robot-to-robot IR message received | `enable_robot_infrared_message_notify(is_enabled)` sends Sensor `DID=0x18`, `CID=0x3E`, `target=MCU(2)`, payload `isEnabled:bool`. | Sensor `0x18`, `CID=0x2C`, `source=MCU(2)`. | `InfraredMessageEvent(infrared_code:uint8)`. | Cache latest IR code with timestamp; publish `/rvr/events/ir_message`; subscribers may implement robot-to-robot behaviors. Disable on last subscriber. | Enable write fixture; inject `payload=bytes([code])`; assert code reaches callback/topic without touching legacy `DID_IR=0x1C` commands. | IR events can trigger autonomous behaviors if user code listens. Keep core event passive; do not map directly to motion. |
| Color detection interval notify | `enable_color_detection_notify(is_enabled, interval, minimum_confidence_threshold)` sends Sensor `DID=0x18`, `CID=0x35`, `target=BT(1)`, payload `bool,uint16 interval,uint8 confidence`. | Sensor `0x18`, `CID=0x36`, `source=BT(1)`. | Reuse/rename `DetectedColor(red:uint8, green:uint8, blue:uint8, confidence:uint8, color_classification_id:uint8)`. | Cache latest color reading; publish `/rvr/events/color_detection`; interval can be high-rate, so use bounded queue/drop policy. Disable on last subscriber. | Enable write fixture must check big-endian interval (`>BHB`); inject 5-byte color payload, including `classification_id=0xFF` unknown case. | Color IDs may drive app decisions but not direct safety. Beware high interval rates causing callback/ROS pressure. Unknown `0xFF` should not be treated as a real color. |
| Current detected color one-shot result | Trigger command `get_current_detected_color_reading()` sends Sensor `DID=0x18`, `CID=0x37`, `target=BT(1)`, no direct outputs. No separate disable. | Result arrives as color detection notify: Sensor `0x18`, `CID=0x36`, `source=BT(1)`. | Same `DetectedColor` parser as interval notify. | Implement as request-that-awaits-next-color-event or service future with timeout; also publish/update cache like ordinary color event. | Send command `0x37`, inject subsequent `0x36` unsolicited event; assert awaiting one-shot resolves and normal subscribers still receive event. | Avoid racing one-shot calls with interval stream; correlate by first next event only, not sequence ID. |
| Streaming service data | Start/stop via `configure_streaming_service(token, configuration, target)`, `start_streaming_service(period,target)`, `stop_streaming_service(target)`, `clear_streaming_service(target)`. Event delivery effectively disabled by `stop`/`clear`; no `enable_*_notify` method. | Sensor `0x18`, `CID=0x3D`, `source=target` (`BT=1` or `MCU=2`, caller selected). | `StreamingServiceData(token:uint8, sensor_data:bytes)`; first byte token, rest variable-length bytes. | Do not global-cache. Route by token plus source where possible; bounded queues; no generic ROS topic until typed stream schemas exist. Stop/clear on disconnect. | Configure/start write fixtures; inject variable-length payload `bytes([token])+data`; verify demux by source and token and bounded queue/drop behavior. | High-rate stream can starve callbacks and serial reader. Needs backpressure before enabling in ROS. Misconfigured streams can hide safety events by flooding. |
| Motor thermal protection status | `enable_motor_thermal_protection_status_notify(is_enabled)` sends Sensor `DID=0x18`, `CID=0x4C`, `target=MCU(2)`, payload `isEnabled:bool`. | Sensor `0x18`, `CID=0x4D`, `source=MCU(2)`. | Reuse `parse_thermal_protection_status(payload)` -> `ThermalProtectionStatus(left_temp:float,left_status:uint8,right_temp:float,right_status:uint8)`. Payload is `>fBfB` (10 bytes). | Cache latest thermal status; publish diagnostics/event topic; disable on last subscriber. | Enable write fixture; inject `struct.pack(">fBfB", left_temp,left_status,right_temp,right_status)`; verify cache/callback. | Thermal limiting means motor output may be reduced or unsafe. Warn/stop at high statuses; avoid fighting firmware protection with repeated commands. |

## Dispatcher requirements before implementation

1. Add `Dispatcher.subscribe(did, cid, source, callback/parser)` and an unsubscribe handle. The registry should support multiple subscribers per event.
2. Change `_read_loop()` order:
   - Decode packet.
   - If it matches pending response by `(did,cid,seq)`, complete the future.
   - Else if it is an unsolicited command/event, route by `(did,cid,source)` with `(did,cid,None)` fallback.
   - Else log/drop with rate limiting.
3. Never block `_read_loop()` on user callbacks. Schedule callbacks as tasks or push parsed events into bounded queues.
4. Preserve request/response behavior with concurrent traffic: a response and an event sharing DID/CID but different response flag/sequence must not steal each other.
5. Make parser failures and callback failures visible through diagnostics/logging, not fatal to serial I/O.
6. On `Dispatcher.stop()`, cancel pending request futures, clear notification subscribers, and drain/close event queues.

## Minimum parser additions

Current parsers already cover several payloads but should be named/typed for events:

- Add `SleepEvent` dataclass or enum marker for empty sleep payloads.
- Add `MotorStallEvent(motor_index:int, is_triggered:bool)`.
- Add `MotorFaultEvent(is_fault:bool)` or reuse bool parser behind a typed event wrapper.
- Add `GyroMaxEvent(flags:int)`.
- Add `InfraredMessageEvent(infrared_code:int)`.
- Add `StreamingServiceData(token:int, sensor_data:bytes)`.
- Reuse `BatteryVoltageState`, `DetectedColor`, and `ThermalProtectionStatus`.

## Test fixture checklist

Fake transport needs first-class unsolicited packet helpers so tests do not hand-roll framing in every file:

```python
def unsolicited_packet(did, cid, source, payload=b"", seq=0x7F):
    return Packet(
        did,
        cid,
        seq,
        payload=payload,
        target=None,
        source=source,
        flags=FLAG_HAS_SOURCE,
    ).encode()
```

Core test cases to add before wiring ROS:

- Event packet with no pending request invokes subscribed callback.
- Event packet does not complete an unrelated pending request with the same DID/CID but different seq.
- Pending response still completes while an event arrives before/after it.
- Multiple subscribers receive the parsed event; unsubscribed callbacks stop receiving.
- Callback exception is logged and does not kill `_read_loop()`.
- Parser error is logged/dropped and does not kill `_read_loop()`.
- Last subscriber disables firmware notifications for gated events.
- Disconnect unregisters callbacks and disables/stops high-rate streams best-effort.

## Safety policy summary

- **Motor fault/stall**: stop or suppress motion on active fault/stall until policy clears it. These are not decorative notifications.
- **Motor thermal protection**: publish prominently; reduce/suppress motion at elevated statuses and avoid retry storms.
- **Battery low/critical**: warn at low, suppress new motion or begin controlled shutdown at critical.
- **Sleep events**: transition driver state to unavailable/sleeping and stop local velocity intent.
- **Color/IR events**: keep passive by default. They may drive user behaviors, but the core driver should never translate them directly into motion.
- **Streaming events**: require bounded queues and typed stream configuration before ROS exposure; otherwise they can DOS the driver from inside the toy. Which is rude, but possible.
