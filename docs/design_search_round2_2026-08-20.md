# DESIGN NOTE — search round 2 (for consensus, before code)

*2026-08-20 evening, after the four-flight rehearsal program. Scott's build
authorization ("investigate the situation and code up any fixes necessary")
gates through PM consensus on this note, per protocol. Items land as separate
reviewed batches with tests; the re-cert asks are stated per item.*

## 0. CORRECTION TO THE FLIGHT-4 RECORD (dated 2026-08-20, supersedes the live narration)

The live narration called the flight's ending "D60's signature — an
enterable-unexitable pocket" and "the rover pinned in the furniture nook."
**Scott's eyewitness contradicted this ("It's not in a corner... about 2 feet
from the bottle") and the post-flight belief probe proves him right.** Live
costmap composition (full + `_updates`, per the latched-once doctrine), rover
stationary at (0.138, 0.091):

- The rover's own cell costs **224** in both costmaps — high, NOT inscribed.
  Physically it stands on open floor.
- But the belief around it: nearest lethal cell **0.175 m** away, nearest
  ≥inscribed **0.036 m**, **176 lethal cells within 1.0 m** (local). The
  close-range excursion into the furniture cluster painted dense lethal cells
  at 20–40 cm range, and the 0.30 m inflation radius flooded the neighborhood.
- The four failed motions decompose exactly: goals (0.8,−0.5) and (0.8,0.0)
  sit IN inscribed/lethal belief (planner: no path — including the return to
  the rover's own entry corridor, now inside swollen inflation); goals
  (1.2,−0.2) and (1.0,0.3) are cost-139/0 FREE but ABORTED — the controller
  refusing to carry a path out through the gradient hugging the start (the
  inflation-gradient lesson's shape).
- The kicker: **(0.0, 0.3) — where the rover successfully stood five minutes
  earlier — is now LETHAL in the live local costmap.**

**Corrected diagnosis: not a map pocket (D60 as filed), but a SELF-MADE
BELIEF BASIN — the start-clearance crisis, self-inflicted by a blind-range
approach.** Chain: no range on the candidate → guessed leg length → overshot
into the cluster → close-range paint + inflation → basin → no escape
primitive. The confirm-look miss reconciles the same way: at the look pose
the bottle was ~0.48 m away and ~8° off the camera axis — nominally in
frame — but the photo shows the desk-stand base and bin filling the view:
occlusion plus near-field floor cutoff at half-metre range. Scott's "I would
have expected a closer initial approach and then stop" is answered: the leg
was not too short — it was UNMEASURED, and it landed too deep.

D60 keeps its filed meaning (map-geometry pockets); tonight's specimen files
under the start-clearance/self-paint family instead, with full costmap bags.

## 1. RANGE AT THE LOOK — the substantial fix (both symptoms, one root)

The look result carries bearing but no range; every approach leg is a guess.
The tof already stares forward with a ~45° cone that covers the camera's
axis.

**Design:**
- The recognition node subscribes `/tof/points` (PointCloud2 in `tof_link`,
  static TF to `base_link` published by the tof node).
- GEOMETRY, stated precisely: the camera axis is 14° LEFT of the body axis
  (bench-measured constant). `where_in_frame` maps to a camera-frame sector
  (center ±11°, left/right centered ±22°); transform the sector's band to
  the body frame by ADDING the +14° (left = positive yaw) mount offset, then
  select tof points whose body-frame azimuth falls in the band.
- Range = median planar distance of selected points captured DURING the
  snapshot window (freshness: only points whose stamps fall inside the
  camera-up interval; no stored history). No points in the band → null.
- Result fields: `"range_m": <float|null>, "range_source": "tof"|null` —
  added to `build_result` (provenance doctrine: a measurement, stamped by
  its source). TOOL_SCHEMAS UNCHANGED; the bridge forwards verbatim.
- SYSTEM_PROMPT look description gains: "when the answer includes range_m,
  the object is that many metres ahead along the bearing — use it to place
  the approach stop." Stanza approach bullet gains the minimum-confirm-range
  lesson: "look again from about a metre away — closer than ~0.8 m a floor
  object drops out of the camera's view" (tonight's measured miss).
- **Re-cert ask (the one Scott sitting, ~5 min):** staged object on the
  existing tape marks; assert `range_m` within ±0.15 m of tape truth at 2–3
  distances; absent target → null. Refusal ladder untouched → no R1 rerun.

## 2. ENDGAME SAY RULE — the wordless-budget fix (flights 1 and 4)

The model NEVER SEES the budget — `build_user_turn` carries instruction +
history only, so "when one call remains, say" cannot be followed. Fix in two
halves:
- `build_user_turn` gains one line when a budget is passed: "You have N tool
  call(s) remaining. When 1 remains you MUST finish with say — report what
  you know." (Loop change: `run_instruction` passes `budget.remaining`.)
- Stanza rule: "An instruction that ends without say wastes everything it
  learned. Ending honestly with partial findings beats spending the last
  call on one more tool."
- Canned transcripts: budget-1 turn must produce say (scripted model obeys;
  the test pins the PROMPT carries the count and the loop threads it).

## 3. DISCOVERY TAX + REDUNDANT RE-QUERY — stanza lines (receipts: 3 flights)

- "Some configurations do not run explore/observe/status. A tool that
  reports unavailable will not become available this instruction — do not
  call it again; search with look, turn, goto and where_am_i."
- "Do not repeat a query that answered empty unless something has changed."
- Canned transcripts: a script that retries an unavailable tool is the
  ANTI-transcript (documented as what the prompt forbids); the positive
  transcript goes straight from one unavailable result to the look.

## 4. BASE-CAP ECONOMICS — argued with tonight's 5/5

Evidence: every substantive decision (post-look, post-failure) escalated
1500→4500 and succeeded there; every trivial early call fit 1500. Each
escalation re-sends the full prompt (~2–4k tokens late-mission) — real but
modest cost, and zero crashes.

Options: (i) raise base to 4500 — every trivial call pays; (ii) keep the
ladder as-is — one wasted base attempt per hard decision; (iii) STICKY
ESCALATION: the caller passes a small mutable state (`sticky={}`) and
query_text records the cap that last succeeded, starting there for the rest
of the instruction; task_client scopes the dict per instruction.
**Recommendation: (iii)** — first hard decision pays the ladder once, the
rest of the mission starts escalated, trivial early calls stay cheap, and
the mechanism remains visible in the log. (ii) is the do-nothing fallback;
the cost data does not justify (i).

## 5. CONFIG-MATCHED TOOL SUBSET — design only (build if PM wants it cheap)

ToolRunner probes its Trigger clients at startup (0.5 s wait each) and
prepends one line to the instruction: "Unavailable in this configuration:
explore, observe, status." Kills the discovery tax at the source; overlaps
with item 3's prompt line (either alone may suffice — PM's call whether both
land or item 3 goes first and this waits for receipts).

## 6. STANDING, NOT THIS ROUND

- **Escape primitive** (the supervisor-owned recovery for belief basins /
  D52/D60 family) — tonight adds a fully-bagged specimen; remains the
  supervisor design item.
- Camera-while-moving, orientation-robustness (unchanged out-of-scope).

## 7. ORDER OF LANDING (each its own reviewed batch)

(2)+(3) first — stanza+loop+transcripts, no hardware, no re-cert; then (4iii)
with its unit tests; then (1) with its bench ask queued for Scott's next
5-minute sitting; (5) per PM ruling. Correction §0 lands with this note.
