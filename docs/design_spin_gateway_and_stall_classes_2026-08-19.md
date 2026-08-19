# Design note: Spin through the firmware gateway + stall classes for the touch sense

*Batch (a) + D57, design round — no code moves on this document. It moves on PM
review, and nothing motion-capable flies before the bench card passes and Scott
stages it. Both surfaces are safety-adjacent: (a) touches the supervisor seam,
(57) touches the sole author of permanent marks.*

Ground facts this design stands on (all source- or bag-verified today): the
2026-08-19 flight's dead pivots commanded max rate (duty 28–45, the curve's own
ceiling) into scrub friction; three firmware motor stalls; two FALSE contact
marks planted in the gap from rotation stalls; Spin recovery ran at 5.83 rad/s
(= duty 45) and stalled like everything else.

> **CORRECTION 2026-08-19 (post-design, Scott's ground truth):** the
> "location-specific" framing in this note is wrong — the floor is uniform (no
> grippy spot); duty 45 is marginal for from-rest pivots EVERYWHERE on it, and
> the stalls landed where hard from-rest pivots were demanded. Nothing in Part
> 1 or Part 2 changes. **The duty-band-extension fallback named below is
> WITHDRAWN by Scott's ruling** ("I worry about any fixed duty. This is just
> one room.") — feedback, not calibration: the in-path residual now routes to
> the firmware-native-velocity-loop investigation, and the curve demotes over
> time to a feed-forward hint. See the run card's dated correction for the
> full wording.

> **SECOND CORRECTION 2026-08-19 (git fact, found via the deployed yaml):**
> this note was written believing bench items (i)–(iii) were pending. They
> PASSED on 2026-08-18 night — `f4c840a` flipped `precise_turn_bench_verified`
> to true in both yamls with the measured receipts (estop preempt; stop kills
> hold; 21/21 turns, mean |err| ≤ 2.2°) — and the flag was open the whole day
> this note was reviewed in. Both sessions missed it against their own
> records; the deployed config outranks both of us, and probing the yaml en
> route to an unrelated batch is what caught it. The "two locks flip together"
> rationale was sound engineering written against wrong facts: in the real
> state there is ONE remaining lock (`use_precise_turn_spin`), and it alone is
> sufficient — no Spin goal flows to the gateway while it is false. (i)–(iii)
> stand as passed on a verified basis, not vibes: since f4c840a the turn verb,
> gateway, admission, collision_stop*, and rvr_node have ZERO diff — the only
> driver.py delta is D31's disconnect/teardown fix, which never touches the
> turn path. The remaining sitting is (viii)–(ix) only.

---

## Part 1 — batch (a): Spin recovery through the precise-turn gateway

### What already exists (verified at source, collision_stop_node.py)

The supervisor serves a **nav2_msgs/action/Spin** ActionServer at
`/collision_stop/precise_turn`: admission gate (`precise_turn_admission` —
refuses un-benched flag, unsafe supervisor state, corner proximity), fires the
firmware heading loop via `/rvr_driver/precise_turn_cmd`, honors `target_yaw`
as a relative delta, publishes `angular_distance_traveled` feedback, and EVERY
non-success exit (cancel, timeout, supervisor state change) stops the driver
first. The firmware manages torque against resistance — the built-in that
attacks the flight's root cause, per built-in-first.

### The integration is a CONFIG retarget, not code

nav2's BT is designed for exactly this: the `Spin` BT node takes a
`server_name`. Stock currently loads nav2's own
`navigate_to_pose_w_replanning_and_recovery.xml`, whose Spin child calls
behavior_server's `spin` (the open-loop duty-45 path that stalled).

**Proposal:** a repo-owned copy of that standard XML —
`behavior_trees/navigate_to_pose_stock_precise_turn.xml` — identical except the
Spin recovery child gets `server_name="/collision_stop/precise_turn"` (and a
`server_timeout` consistent with the 1000 ms defaults-by-decision block in
lean_nav2_stock.yaml). Installed via setup.py (the install-manifest guard test
already exists). Zero new nodes, zero custom plugins; the same
action-interface-retargeting nav2 ships for.

### Gating (the watcher's own pattern: default OFF until cleared)

New launch arg `use_precise_turn_spin`, default **false** → bt_navigator loads
the standard XML exactly as today; **true** → loads the retargeted copy. The
flip is one launch-arg default change AFTER the bench card passes, reviewed
like the watcher flip was. Two locks total: the launch arg (routing) and
`precise_turn_bench_verified` (admission) — flipped in the same post-bench
batch so there is no state where Spin routes to a gateway that must refuse it.

Failure semantics while gated or refused: the BT Spin child returns FAILURE →
the recovery round-robin proceeds to BackUp/Wait, exactly as when today's spin
achieves nothing — no capability lost, and the refusal is loud (supervisor
event + reason).

### Deliberate contract notes

- The gateway enforces its own `precise_turn_timeout_s` (5.0 s) and ignores the
  BT goal's `time_allowance` (10 s default) — the tighter, measured bound wins;
  documented rather than silently reconciled.
- `spin_dist` stays the standard 1.57 rad; the sign bit is bench item (iii)'s
  first-turn measurement, not an assumption.

### Named residual (kept honest)

In-path RotationShim pivots remain open-loop at duty ≤ 45 — the g3/g4 class.
Intercepting them means a custom controller plugin (the custom-code end of
built-in-first, ruled out for v1). The composed system now covers them:
a shim-stall becomes a D56 freeze discovery (mark + reselect), and when the BT
does escalate, Spin arrives with firmware force. Duty-band extension stays the
bench-measured fallback if the field shows shim-stall frequency staying high.

### Bench card additions (draft — lands on the card after design PASS)

- **(viii) BT-ROUTED SPIN REACHES THE GATEWAY — blocks.** Stack up with
  `use_precise_turn_spin:=true` + bench param override; force a recovery (goal
  onto blocks that cannot progress); PASS: gateway event lane shows the
  STARTED/settled sequence for a Spin goal whose id originates from
  bt_navigator, and behavior_server's spin logs show NO activation.
- **(ix) ADMISSION REFUSAL FALLS THROUGH LOUDLY — blocks.** Same, with
  `precise_turn_bench_verified:=false`: PASS: BT proceeds to BackUp within one
  recovery round, no motion from the spin slot, refusal reason in the event
  lane.

---

## Part 2 — D57: translation stalls mark, rotation stalls must corroborate

### The defect mechanism

contact_marker pairs a firmware stall delta with the commanded motion — but it
records only `linear.x` (`_on_cmd`), so a pure-rotation stall (cmd vx = 0.000,
wz = 3.55 — the flight's exact trace) is indistinguishable from a forward
contact and plants a permanent FRONT mark. Two false marks in the gap resulted.

### The classifier (pure, shared, derived)

A pure function in `sphero_rvr_core.contact_marking`:

```
classify_stall(vx, wz) -> TRANSLATION | ROTATION | IDLE
```

- TRANSLATION: `|vx| >= eps_v` — the rover was commanded to move through space;
  a stall is strong contact evidence (the d45bd24 boot class).
- ROTATION: `|vx| < eps_v <= |wz|·k` — commanded rotation only; the flight
  proved floor grip produces this signature with nothing there.
- IDLE: neither — a stall with no commanded motion is a phantom; error log, no
  mark ever.

`eps_v` is derived from the deployed config, not the room: the smallest
translation the stock controller can command (lean_nav2_stock's RPP minimum
speed), taken at implementation time from the yaml the robot flies
(probe-the-deployed-config). No new tunable — a derivation with a stated
source, pinned by a test that reads the same yaml.

### Marking policy per class

- **TRANSLATION → mark exactly as today** (front/rear edge by sign of vx).
- **ROTATION → mark ONLY with corroboration:** fresh ToF returns inside the
  would-be mark disc, reusing the rig-certified freshness machinery from
  refusal_promotion (`ReturnsRing` + `freshness_verdict` — trust returns, not
  paint; horizon re-derives from the sensor rate). Marker subscribes
  /tof/points (already on the wire, already in the bag). No fresh returns → NO
  MARK, loud event with the reason, counted in a named
  `rotation_stalls_unmarked` counter in the marker's placements report — the
  stall stays fully visible (diagnostics counter unchanged, so the D56 explorer
  lane still hears it; only the PAINT is withheld).
- Placement honesty, recorded as rationale: rotation-stall geometry is also
  ambiguous (the obstruction is at a tread corner, not the front edge), a
  second independent reason rotation paint is weak evidence.

### Interaction with D56 (explorer freeze lane) — decision point for you

v1 proposal: the explorer's freeze accounting stays as landed (all stall deltas
count as discoveries). Rationale: with (a) landed, gateway Spin powers through
floor grip and rotation stalls should become rare; a floor-grip stall ending a
mission as BLOCKED_BY_UNSEEN_OBSTACLES is still more truthful than
GOALS_KEEP_FAILING, and its ceiling (5) bounds it; a third "authority stall"
bucket today is scaffolding ahead of evidence. If the field shows
rotation-stall storms surviving (a), the shared classifier is already in core
and the bucket is a small follow-up. Flag if you want the bucket now.

### Close-criteria tests (from the D57 row, both directions)

- Replay the flight's cmd/stall traces (real numbers from the bag, in fixtures)
  → **0 marks** (falsifier: current code plants 2).
- Replay the d45bd24 boot class (translation stall) → **1 mark**, same centre
  as today's geometry.
- Rotation stall WITH fresh in-disc ToF returns → 1 mark (corroboration
  admits).
- Freshness horizon and radius pins reuse the existing refusal_promotion test
  fixtures — one authority, not a second copy.

---

## Batch order within (a), if this note passes

1. D57 marker distinction + classifier + replay tests (no motion semantics
   change beyond withholding weak paint — flyable immediately, and it protects
   the NEXT flight regardless of when the bench happens).
2. BT retarget + launch arg + wiring/pin tests (inert until the arg flips).
3. Bench card update (items viii/ix). Scott schedules the sitting; the two
   locks flip together in a reviewed batch only after the card passes.

Rollback story: every piece is behind a default-false arg, an admission flag,
or is strictly paint-narrowing; each reverts with one commit and no code.
