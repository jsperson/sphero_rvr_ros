# BENCH CARD — 2026-08-19 morning (one sitting of Scott's switch time, ~20 min)

Consolidated from the 2026-08-18 night ladder. Chassis ON BLOCKS or clear floor
per item; each item says which. Every item is a MEASUREMENT with a written-down
pass bar — nothing here flips a flag on vibes. Bring the stack up with
`launch_and_arm.py --stack stock` (watcher flag irrelevant for the bench).

## (i) ESTOP PREEMPTS AN IN-FLIGHT FIRMWARE HEADING TURN — blocks, THE must-pass

The precise-turn primitive commands `drive_with_heading(0, target)`: the firmware
holds the heading until countermanded. The machinery says estop overrides it
(motor-capable command class); NOTHING says so from measurement. On blocks:
start a turn via the gateway with the bench flag TEMPORARILY true in a param
override (`--ros-args -p precise_turn_bench_verified:=true` on BOTH nodes for
this sitting only — the YAML stays false until the whole card passes), then
`ros2 service call /collision_stop/estop` mid-turn.
**PASS: treads stop within ~0.2 s and STAY stopped.** FAIL = the primitive is
dead on arrival and option B (our-layer gyro loop) revives from the memo.

## (ii) SUPERVISOR-ISSUED DRIVER STOP KILLS THE HOLD — blocks

Same setup; mid-turn, `ros2 service call /collision_stop/stop`. This is the path
the gateway's cancel/timeout/violation exits use — bench proves the stop actually
ends the firmware hold. **PASS: treads stop and stay stopped.**

## (iii) TURN ACCURACY + THE SIGN — clear floor, ~0.5 m radius

`PRECISE_TURN_HEADING_SIGN` is ASSUMED −1 (firmware heading compass-like,
CW-positive). First single turn: command +90 (left). **Record which way it
physically turns** — that one bit settles the constant. Then 10× 90° each
direction via the gateway; Scott eyeballs, odom records.
**PASS: mean |error| ≤ 5°, worst ≤ 10°, correct direction every time.**
(The straight-API era's remembered accuracy is the hypothesis under test.)

## (iv) YAW-REFERENCE DRIFT — blocks, passive

Note firmware heading (event line prints it) at card start and card end
(~20 min). **Record the drift; no pass bar** — it sizes the future absolute
turn_to_heading verb's sync problem. > ~5°/session means the offset needs
periodic re-capture; < that, a bringup-time constant suffices.

## (v) BATTERY-CONDITIONED PIVOT-CURVE RE-CHECK — clear floor, 30 s

The 5.85-vs-2.64 rad/s non-reproduction is now MOSTLY explained (the stale-flap
chopped the pivot), but the battery hypothesis was never closed measured. One
sweep: `python3 diagnostics/pivot_duty_sweep.py` (or three pivots at duty 45 via
the existing tooling) at current battery. **PASS: peak rate within ~10% of the
5.852 rad/s curve point.** Outside that → the curve gains a battery condition
and the register a row.

## (vi) ITEM-2 FIELD BAR: THE FLAP UNDER REAL LIDAR — clear floor, one pivot

The rig's residual flap was environment-confounded; the field decides. One fast
pivot (Spin at 3.55, ~4 s), then read the state line ONCE:
`slot_scan_gaps` / `slot_tick_overruns` deltas across the pivot + count
SENSOR_STALE transitions in the recorded state stream.
**PASS (item 2 closes for good): ≤ 2 stale transitions during the pivot.**
Still flapping → item 2 reopens WITH field data (the counters are the evidence).

## (vii) RIDE-ALONG NOTES, free while the stack is up

The placement fix (d2f96e9) field-confirms on any contact that plants a mark
with `path=fallback` + staleness in the log. No dedicated staging: it rides
whatever happens.

## (viii) BT-ROUTED SPIN REACHES THE GATEWAY — blocks *(batch (a), design PASS 2026-08-19)*

Stack up with `use_precise_turn_spin:=true` plus the sitting-only bench param
override from item (i). Send a NavigateToPose goal the rover on blocks cannot
progress (any goal — treads spin free, no displacement), and let the BT escalate
to its recovery round.
**PASS: the gateway's event lane shows the STARTED → settled/stopped sequence
for a Spin goal originating from bt_navigator, and behavior_server's log shows
NO spin activation.** FAIL = the retarget is cosmetic; find where the goal
actually went before anything flips.

## (ix) ADMISSION REFUSAL FALLS THROUGH LOUDLY — blocks *(batch (a))*

Same staging, but with a `precise_turn_bench_verified:=false` param override.
*(CORRECTED 2026-08-19: the shipping value is TRUE — f4c840a flipped it after
items (i)–(iii) passed on 2026-08-18 night. This item deliberately stages the
REFUSAL state to prove it is safe-and-loud, since it can no longer occur by
default.)* Force the same recovery round.
**PASS: the refusal reason appears on the event lane, NO motion happens in the
spin slot, and the BT proceeds to BackUp within one recovery round.** This is
the exact state the stack would be in if the routing arg ever flipped without
the bench flag — the bench proves that state is safe-and-loud, not silent.

## After the card

**STATUS CORRECTION 2026-08-19 (git fact, found via the deployed yaml):**
items (i)–(iii) PASSED on 2026-08-18 night and `f4c840a` already flipped
`precise_turn_bench_verified: true` in BOTH yamls, with the measured numbers
and Scott's verbatim passes recorded in the yaml comment (estop preempt;
stop-kills-hold; 21/21 turns, mean |err| ≤ 2.2°). The paragraph below is
retained as the rule that WAS followed, not a pending action.

~~If (i)–(iii) pass: flip `precise_turn_bench_verified: true` in BOTH yamls in
one reviewed commit that cites this card's measured numbers, and record the
sign measurement at `PRECISE_TURN_HEADING_SIGN`.~~ Done (f4c840a). If a later
sitting ever fails a re-run, the flag flips BACK with the failure cited.

REMAINING: if (viii)–(ix) pass, one reviewed commit flips
`use_precise_turn_spin` default true in `explore.launch.py` — now the ONE
remaining lock (the admission gate behind it is already open). If (viii) or
(ix) fails, the routing arg stays false: flights continue on behavior_server's
spin exactly as before the batch, and the failure goes to the register.
