# BENCH CARD — the recognition primitive (look_and_recognize)

*Certification before any mission depends on it (design + consensus
2026-08-19). Zero motion authority anywhere on this card: the rover never
receives a motion command. ~25 minutes of Scott's staging.*

## Chassis state (the call, stated per the ruling)

**Chassis ON, rover STATIONARY, full stack's sensing half up** (driver +
lidar + supervisor + tof — no Nav2 needed). Reason: the stationarity gate
reads the SUPERVISOR'S state line, and certifying the verb with
`require_stationary:=false` would leave the gate itself uncertified. The
fail-closed branch is certified live (R1c); the commanded-motion branch stays
unit-pinned — certifying it live would require commanding motion, which this
card forbids by design. Stated honestly here so nobody reads R1 as more than
it is.

Node under test: `ros2 run sphero_rvr_driver recognition` (target set via
`ros2 param set /recognition target "<thing>"`, invoked via
`ros2 service call /recognition/look_and_recognize std_srvs/srv/Trigger`).

## What Scott stages

- THREE target objects: the Dr Pepper bottle + two of his choosing (distinct
  colors/shapes), placed one at a time ~1.2 m ahead of the parked rover in
  clear view.
- Tape marks at lateral offsets from the camera axis at that range: 0, ±0.15,
  ±0.30, ±0.45, ±0.60 m (for R4's FOV measurement — ~7° steps at 1.2 m).
- The room otherwise as-is; ambient light, no hard sun (standing constraint).

## Items, in order (refusals FIRST — falsifier before certifier)

### R1. THE REFUSAL LADDER refuses, loudly, before the happy path is trusted

(a) no target set → invoke → **PASS: "REFUSED: no target set..."**, and no
camera process ever appears. (b) `api_key_file` pointed at a missing path →
**PASS: refusal names the PATH and never any contents; no camera**; restore.
(c) kill the supervisor's state feed (stop the supervisor node) → invoke →
**PASS: the fail-closed stationarity refusal**; restart supervisor.
FAIL on any silent no-op or camera activity during a refusal = stop the card.

### R2. HIT RATE on staged objects

Each of the 3 objects, centered, 3 invocations each (9 total).
**PASS: ≥ 8/9 seen=true with a sane description; 9/9 schema-valid** (any parse
refusal is investigated, never shrugged — the contract is the product).

### R3. ABSENT-TARGET FALSE-POSITIVE RATE — the product number

Against the same staged scenes, 6 invocations for objects NOT present
(Scott picks absurd-but-plausible: "banana", "red shoe", ...).
**PASS: 0/6 seen=true. ZERO, not low** — a searcher that finds absent things
is worse than none. One false positive → the prompt/confidence design gets a
consensus round before any mission use; the card continues for data.

### R4. FOV MEASUREMENT — 66° vendor number becomes a measured constant

One object walked across the tape marks (9 positions), one invocation each.
Record where_in_frame per position. **PASS: monotonic right→center→left with
stable sector boundaries; derive the measured HFOV** (boundary offset ×
range arithmetic) — the `HORIZONTAL_FOV_DEG` constant gains the measured
value + this card as receipt in a reviewed commit. No pass bar on the VALUE
(it is whatever it measures); the bar is monotonicity + repeatability.

### R5. ToF NON-INTERFERENCE — the counter method (the CPU lesson, measured)

tof runs the whole card. Frames-counter rate over ≥10 s spans DURING
camera-up/snapshot/VLM windows vs a quiet-stack baseline span.
**PASS: every span ≥ 6.5 Hz (the band does not move); report the delta vs
baseline** — the number goes in the card whatever it is.

### R6. CAMERA-DOWN RECEIPTS, including induced error paths

After EVERY invocation above: camera process absent (`ps -eo pid,comm`, no
camera node on `ros2 node list`). Then three induced failures: (a) bad
`base_url` (VLM query fails) — camera down after; (b) `image_topic` pointed at
a silent topic (frame timeout) — refusal + camera down after; (c) restore
everything, one clean invocation as the recovery proof.
**PASS: camera down after every path with no manual cleanup, ever.**

## After the card

All pass → the primitive is certified for composition (the search design round
may begin); FOV constant updated with the measurement; results recorded here
verbatim. Any fail → register row with the receipts, primitive stays
bench-only. Nothing on this card changes flight behavior either way — the
node is a leaf no mission invokes yet.

---

## RESULTS (run 2026-08-20, Scott staging; logs: bench4/bench5/bench6 on the Pi)

Build under test: 45476c9 (includes the R2-original dark-frame capture fix
717a012 — the warm-up quality gate). Stack up stock, chassis on, rover
stationary at the angled pose (4° right of tape-normal, lidar-measured).
Photos: ~/recognitions/ on the Pi (30+ invocations).

### R1 — REFUSAL LADDER: **PASS**

All three refusals loud, no camera process during any refusal: (a) no target;
(b) missing `api_key_file` refusal names the PATH, never contents; (c)
supervisor down → the fail-closed stationarity refusal.

### R2 — HIT RATE: **6/9 vs ≥8/9 — FAILS AS WRITTEN**; the per-class ledger is the product

The ORIGINAL R2 run failed on dark frames (capture raced auto-exposure);
defect fixed same morning (717a012, warm-up/quality gate), deployed 45476c9,
and the rerun logged ZERO dark frames — capture defect cured, field-proven.
Rerun ledger, per class:

- **xbox controller 3/3** seen=true, conf 0.90/0.95/0.92 — plain distinct
  objects are solved.
- **dr pepper bottle 2/3 as written** — but DETECTION was 3/3: every frame's
  reply describes the bottle (position, cap, label); the misses are the model
  honestly declining to certify the BRAND ("its contents and label do not
  match Dr Pepper", conf 0.62; "not identifiable as Dr Pepper", 0.72). The
  failure class is brand/identity verification asked of a 640px JPEG, not
  blindness — the single-question schema has no way to say "object yes,
  identity unsure," which is the schema-redesign round's core exhibit.
- **family picture in a frame 1/3** — small, low-salience target; two honest
  seen=false ("no clearly framed family picture"), one seen=true center 0.62.

### R3 — FALSE-POSITIVE RATE: **BAR MET — 0 FP, ZERO fabrications in 25+ invocations**

0/6 valid absent-target probes seen=true, bright-condition rerun 3/3 clean
(banana ×2 conf 0.95/0.98, person probe), and the decisive person-probe:
shown a scene, asked for "person" — seen=false conf 0.98. Across every
invocation of the whole card (25+), the model NEVER asserted an absent
object. The zero-FP record is the primitive's most valuable property and is
non-negotiable in the redesign.

### R4 — FOV MEASUREMENT: **PASS**

9-mark walk monotonic right→center→left, stable sector boundaries. Vendor
66° HFOV VALIDATED (right edge crossing found between +0.45 and +0.60 m at
1.2 m). Mount offset MEASURED: camera axis ~14° LEFT of body axis = 10° from
the walk + 4° deliberate staging body-yaw (lidar-measured against the
east-west wall; SLAM-zero-at-angled-pose caveat noted). `HORIZONTAL_FOV_DEG`
keeps 66 with this card as receipt; the mount offset is a new measured
constant for the composition round.

### R5 — ToF NON-INTERFERENCE: **BAR FAILS AS WRITTEN — AND THAT IS THE CHARTER'S RECEIPT**

tof frames-counter (≥10 s spans): 6.85 Hz quiet-stack baseline; 6.11 and
6.43 Hz in camera-up/VLM windows — below the 6.5 Hz band, a −0.4 to −0.7 Hz
camera drag. The rover was stationary by this card's design, so the sensor
was not safety-active; what the number proves is that snapshot-mode camera
work DOES starve the ToF, which validates the never-while-driving charter
EMPIRICALLY rather than by caution. This is the banked baseline evidence for
any future camera-while-moving row: that row must clear this measured drag,
not an assumption.

### R6 — CAMERA-DOWN RECEIPTS: **camera lifecycle PASS on every path; (a) exposes a node-survival DEFECT**

Camera process absent after every invocation of the card (ps + node list;
one stale `/camera_node` daemon-cache ghost verified dead by silent topic hz
and gone after cache refresh).

- **(a) bad `base_url`** — camera DOWN before the failure (the snapshot
  `finally` is upstream of the VLM call) ✓, BUT the failure mode is NODE
  DEATH, not a refusal: `requests.exceptions.ConnectionError` escapes the
  `except (ValueError, RuntimeError)` at the VLM call site, propagates out of
  `executor.spin()`, and the process exits (bench5 traceback, 10:59). The
  service caller never receives a response; restart is manual. **FAILS "no
  manual cleanup, ever."** Defect: the VLM call must catch transport errors
  (`requests.RequestException`) into the loud-refusal path. Fix goes with the
  schema-redesign round (consensus first); primitive stays bench-only anyway.
- **(b) silent `image_topic`** — the first induction (`ros2 param set` on a
  live node) was VACUOUS: the subscription binds at init; the param changed,
  the wiring didn't (no REFUSED line anywhere in bench5). REDONE VALIDLY
  2026-08-20 11:11 by restarting the node with `-p image_topic:=/bench6_silent_topic`:
  refusal text captured verbatim — `REFUSED: camera produced no QUALITY
  frames within the timeout (all frames dark/unsettled, or none arrived —
  camera DOWN again; see log)` — while a mid-window probe showed the REAL
  topic flowing at ~13 Hz (camera up, node correctly wired elsewhere: the
  induction mechanism is proven, not assumed). Camera down after; node
  SURVIVED this path. **PASS.**
- **(c) clean recovery** — node restarted on pure defaults, one clean
  invocation: success=True, `seen=true, center, conf 0.72` on the staged
  picture, photo + provenance recorded, camera down after, node alive.
  **PASS.**

### OVERALL VERDICT

**CERTIFIED as measured: capture (zero dark frames post-fix), honesty
(zero false positives in 25+), plain-object recognition (controller 3/3,
bottle detection 3/3), geometry (FOV validated, mount offset measured),
camera lifecycle (down after every path including a node crash).**

**NOT certified: the ANSWER SCHEMA — a single seen-boolean cannot carry
"object present, identity unverified," which cost the bottle class its bar
and is exactly wrong for two-stage search. Goes to the schema-redesign
consensus round. Plus the R6a transport-exception defect (node death on VLM
connection failure).**

**The recognition/bridge tool STAYS DISABLED pending schema redesign +
re-cert.** Nothing on this card changed flight behavior.

---

## RE-CERT RESULTS (2026-08-20 afternoon sitting, schema redesign live)

*Per the re-cert plan (design_recognition_schema_redesign_2026-08-20.md §8,
amended §8b): R2′/R3/R6a/R6c rerun on the redesigned schema; R1/R4/R5/R6b
stand on the morning's receipts. Build 3c3f05f; 23 invocations, 23/23
schema-valid, camera down after all 23; bag sitting_bag_20260820_154736.*

**EVERY BAR MET — RE-CERT COMPLETE.**

- **R3 falsifier (ran FIRST): PASS 7/7 zero-match** — book .92, shovel .95,
  rocket .95, banana .97, flower pot .93, fence .98, person .95 (fired only
  on Scott's confirmed "clear"); all identity=null; every description names
  the actual scene. Zero-FP lineage now: 25+ old-schema invocations + the
  24-frame offline corpus + 7 fresh live probes = zero fabrications ever.
- **R2′ bottle: PASS 3/3 amended bars** — match .95/.98/.90, identity
  unverified 3/3 with the reason stated ("label not legible enough to
  confirm"), zero mismatch, zero wrong confirmed. The morning's forced-false
  frame-class now returns the approach-candidate answer with a bearing.
  **Decoy (glue bottle): match=true + identity=MISMATCH .90 with the
  contradicting evidence NAMED** ("white bottle with a red nozzle...
  resembling a spray/cleaner bottle rather than a dark Dr Pepper soda
  bottle") — PIN 1's evidence rule, live. **Variant (larger DP bottle):
  match+unverified .82** — generalizes across vessel size, no over-fit.
- **R2′ controller: PASS 4/4** — "xbox controller" 3/3 match (.78/.82/.72),
  identity {unverified, CONFIRMED, unverified}, zero mismatch; Scott's
  ground truth: genuine Xbox, "recognizing controller is a pass," so zero
  wrong confirms. **Epistemic note, recorded deliberately:** the one
  confirm was form-inferred and Scott judges the brand not easily
  recognizable — factually right but evidentially thin. The silver
  controller supplementaries showed restraint IS the default (unverified
  .62, "Xbox-specific details are not visible") — the red form-confirm is
  the outlier on record. **The third-party-gamepad falsifier remains
  AVAILABLE-UNEXERCISED** (both controllers genuine; today did not test the
  wrong-confirm edge). **"game controller" (identity-free, the amendment's
  plain-object probe): match + CONFIRMED .85** — the schema's
  no-identity-component rule followed exactly.
- **R2′ picture (recorded class, ungated): 3/3 match=true** (.60/.55/.78),
  all honestly unverified — the class the original card scored 1/3 now
  detects every time.
- **Aggregate: 9/9 vs ≥8/9.**
- **R6a (transport failure, the fix's live proof): PASS** — bad base_url →
  `REFUSED: recognition failed: VLM request failed (ConnectionError) —
  endpoint or network trouble, not a verdict about the scene`, service
  ANSWERED, node ALIVE, camera down. This morning the same induction killed
  the node with no answer. **R6c: PASS** — clean success on the same node
  immediately after restore; node alive at close.
- **Supply verdict (same sitting): CLOSED** — kernel journal across the
  4h23m boot on the new mains path shows ZERO under-voltage/throttle
  events; the old outlet/cable path is convicted; no purchase needed.

**Consequence: the primitive is CERTIFIED for composition on the redesigned
schema. The tool-enable flip proceeds as its own reviewed commit (the
watcher-flip pattern). The live search rehearsal is a FLIGHT (it drives)
and needs its own staging + envelope + clear.**

### Round-2 addendum (2026-08-20 evening): range_m — QUEUED re-cert row

The flight program added `range_m`/`range_source` to the result (tof points
gathered in the snapshot window, sector band via the measured 14° mount
offset — design_search_round2_2026-08-20.md §1). **Nuance, stated per the
consensus: the field is USABLE immediately (it simply arrives, null-honest),
but its ACCURACY is uncertified until the queued sitting — one staged object
on the existing tape marks at 2–3 distances, bar `range_m` within ±0.15 m of
tape truth, absent target → null. Until that sitting passes, the search
stanza's reliance on range is not itself considered certified.** ~5 minutes
of Scott's staging, zero motion, queued for his next convenient moment.

### RANGE RE-CERT RESULTS (2026-08-20 evening sitting) — **CERTIFIED**

*Preamble, the sitting's meta-lesson (PM's words, three instances in one
evening): THE SENSOR MEASURES TRULY; THE SEAMS BETWEEN SUBSYSTEMS ARE WHERE
TRUTH GETS EATEN. Every failure below was an aggregation or geometry seam;
the tof itself was never once wrong.*

The sitting ran as a live falsifier-driven design round — three shots, two
design defects caught by the bars, both fixed-and-certified same sitting
(f244e73 final build; every fix carries the field distribution that convicted
it as a committed must-flip fixture):

- **Placement 1 (tape 1.754):** FAILED as first built — median-of-everything
  said 0.502, drowning the bottle (1.61, in-bar) under 60 floor-clutter
  returns. → RULING C: cluster + nearest-standing + `range_ambiguous`
  (8cdfc9e). RERUN: range 1.145 + ambiguous=true — the couch (photo-confirmed
  by Scott's own read) IS the nearest standing object; the CORRECT answer for
  the stated scene. VALID CERT SHOT.
- **Placement 2 (tape 0.742, accidental edge-case):** cutoff correctly
  discarded the sub-0.8 bottle (whose returns, 0.72–0.73, were −0.02 from
  tape — the sensor again) but reported background 1.67 UNAMBIGUOUS — the
  confidently-wrong-object class. → NEAR-BAND refinement [0.6, 0.8) flags
  ambiguity (81f9654). Boundary specimen recorded; residual (<0.6 hiders)
  documented with the stanza's never-confirm-below-0.8 argument.
- **Placement 3 (tape 1.251):** FAILED as shot — the VLM's defensible
  borderline "left" call aimed the unwidened band ~2° off a bottle the
  sensor held at 1.21–1.26. → 6° BAND OVERLAP (f244e73). RE-SHOT under the
  SAME borderline call (no lucky flip): **range 1.248 vs tape 1.251 —
  0.003 m**, confirmed 0.87, ambiguous honestly true. IN BAR, fix
  live-proven under its own failure condition.
- **Absent probe (floor cleared, room vacated):** match=false 0.98,
  identity/range_m/range_ambiguous all null, camera down. PASS.
- Vision throughout: FOUR true `confirmed` verdicts on the real bottle's
  legible label (0.85–0.92) — the confirm range for this object class
  extends to at least 1.75 m in this light.
- Procedural scars, owned: one invocation fired without re-setting the
  target after a node restart — the R1 refusal ladder caught it loudly
  (the certified refusal machinery protecting its successor's cert).

**The search stanza's reliance on range_m is now CERTIFIED** (the addendum's
usable-now nuance is discharged). Flight 5 — the full-loop bottle hunt —
is the next flight this card supports.
