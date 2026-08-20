# DESIGN NOTE — recognition answer schema redesign (for consensus, before code)

*2026-08-20. Trigger: bench card verdict (docs/bench_card_recognition_
2026-08-19.md RESULTS). Status: PROPOSAL — nothing here is built; the
recognition/bridge tool stays DISABLED throughout this round and flips only
in a reviewed commit after re-cert.*

## 1. The exhibit — what the card proved about the current schema

The card certified the primitive's **capture** (zero dark frames post-fix),
**honesty** (zero false positives in 25+ invocations, person-probe 0.98
decisive), **plain-object recognition** (controller 3/3 ≥0.90), and
**geometry** (FOV validated, mount offset measured). What failed was not the
eyes — it was the ANSWER SHAPE:

- Every bottle frame was DETECTED (3/3 replies describe the bottle, its cap,
  its label, its position) but 2 of 3 were scored `seen=false` because the
  model honestly declined to certify the Dr-Pepper BRAND from a 640px JPEG
  ("its contents and label do not match Dr Pepper", conf 0.62).
- The single `seen` boolean has no way to say "an object of that kind is
  right there; the identity is unverified." Honest doubt and blindness are
  the same word. That is exactly backwards for a searcher, whose next move
  on those two answers is different: approach the candidate vs keep looking.

The zero-FP record shows the model's honesty is the asset. The schema wastes
it.

## 2. Proposed schema — object-match and identity-verification split

Reply schema (VLM contract, `parse_recognition_reply`, `build_result`):

```json
{"match": true/false,
 "identity": "confirmed" | "unverified" | "mismatch" | null,
 "where_in_frame": "left" | "center" | "right" | null,
 "confidence": 0.0-1.0,
 "description": "one short sentence"}
```

- **`match`** — an object of the target's base kind is visible (a bottle,
  when asked for a Dr Pepper bottle; a controller, when asked for an Xbox
  controller). This is the DETECTION question the model already answers
  3/3 on every staged class it can resolve.
- **`identity`** — the qualifier verdict, only meaningful when `match=true`:
  - `confirmed`: the distinguishing features (brand, wording, specific
    identity) are visibly established;
  - `unverified`: right kind of object, identity not resolvable from here —
    THE APPROACH-CANDIDATE ANSWER, today's forced-false case;
  - `mismatch`: the visible evidence contradicts the identity (a Coke label
    when asked for Dr Pepper);
  - `null`: required when `match=false`; also the answer when the target
    carries no identity component ("a bottle" — nothing to verify).
- `where_in_frame` rules unchanged: required iff `match=true` (a candidate
  with no place is unusable — same doctrine, new field name).
- `confidence` is the model's confidence in the `match` answer (one number;
  identity carries its own three-way honesty and needs no second scalar —
  keeping the schema at one confidence avoids invented precision).
- Bearing/provenance in `build_result` unchanged, keyed on `match=true`.

Strictness doctrine unchanged: parse-or-raise, contradiction rules enforced
(`match=false` with a `where` refuses; `match=true` with `identity=null` on
an identity-bearing target refuses — the prompt states whether the target
carries an identity component... no. Simpler and testable: `match=true`
requires `identity` in {confirmed, unverified, mismatch}; targets without an
identity component expect `confirmed` — "it is what you asked for" — so the
rule needs no target parsing. Stated in the prompt.)

## 3. The zero-FP record — how it is preserved (non-negotiable)

The false-positive definition MOVES to the new field and the bar stays ZERO:
**`match=true` when no object of the target's base kind is present is a
false positive; the bar is 0 in however many probes we run.**

Why the split does not loosen it: under the old schema, honest doubt pushed
answers toward `seen=false` — the SAFE direction for FP but the wrong one
for search. The new risk direction is a too-generous `match` ("anything
vaguely bottle-shaped"). Three mitigations, all testable:

1. The prompt defines `match` as: an object a person would call by the
   target's base noun, with the supporting evidence NAMED in `description`
   (the card's replies already do this unprompted).
2. `identity=confirmed` that is wrong is a SECOND zero-bar: zero false
   confirmations, ever — a searcher that announces the wrong bottle found
   is D24 with a camera. `unverified` is always the honest out.
3. The R3-class falsifier reruns before any bar is claimed (§5), including
   absent-BASE-KIND probes (ask for "dr pepper bottle" in a scene with no
   bottle of any sort staged — `match` must be false).

## 4. Per-class bars (replacing the 8/9 aggregate)

Re-cert bars, each class 3 invocations at the staging range:

| class (exemplar) | bar |
|---|---|
| plain distinct object (xbox controller) | `match` 3/3, no wrong `confirmed` |
| branded object (dr pepper bottle) | `match` 3/3; `identity` ∈ {confirmed, unverified} 3/3; `mismatch`/false only if Scott stages a decoy |
| small/low-salience (framed picture) | `match` ≥2/3 recorded honestly; this class is pixel-starved at 1.2 m and is the APPROACH-REQUIRED exemplar — the fix is stage-two range, not prompt pressure |
| aggregate | ≥8/9 `match` across the nine |
| absent probes (R3 rerun) | 0 `match=true` in all probes — one false = round reopens |

## 5. Designed FOR two-stage search (candidate → approach → confirm)

The searcher's loop consumes the fields directly:

1. **Candidate**: `match=true` from standoff → a candidate at
   `bearing_deg`/`where_in_frame`; `identity=unverified` does NOT end the
   search — it queues an approach.
2. **Approach**: drive to the safety-envelope standoff of the candidate's
   bearing (VIEWPOINT_STANDOFF_M doctrine; approach is Nav2's job, not this
   round's).
3. **Confirm**: re-invoke at close range; `identity=confirmed` is the
   terminal "found"; `mismatch` prunes the candidate and the search resumes;
   repeated `unverified` at close range is reported honestly as "an object
   like it, identity unverifiable."

**Bridge updated same round, stays disabled:** `task_agent.SYSTEM_PROMPT`'s
`look_and_recognize` description gains the two-field answer contract
(match = something of that kind is visible; identity = whether it is
verifiably the named thing; unverified means get closer or say so).
`TOOL_SCHEMAS` is UNCHANGED (the call still takes one `target` string —
only the answer shape changes). `task_node` forwards verbatim as today.
`recognition_tool_enabled` stays false; the flip is its own reviewed commit
after re-cert passes.

## 6. Prompt calibration with the R3-rerun falsifier

Calibration happens OFFLINE FIRST against the banked corpus: 30+ photos in
`~/recognitions/` with known ground truth (bottle/controller/picture/absent
scenes, bright and marginal). The new prompt is iterated against replayed
frames on the Mac-side of the seam (photos + `query_vlm`, no robot, no
staging) until it clears, pre-registered before the live rerun:

- all banked bottle frames → `match=true`, identity ∈ {confirmed,
  unverified}, zero `match=false`;
- all banked absent-target scenes → `match=false` (zero FP on the corpus);
- banked controller frames → `match=true` + `confirmed`;
- banked picture frames → recorded, no gate (the pixel-starved class).

Falsifier-before-certifier: the live R3 rerun (fresh absent probes, Scott
picks the words) runs BEFORE the live hit-rate items in the re-cert
sitting, same order doctrine as the original card.

## 7. Bundled fix: the R6a transport-exception defect (same round)

`recognition_node._look` catches `(ValueError, RuntimeError)` around the
VLM call; `requests.exceptions.ConnectionError` (transport family) escaped,
killed the executor, and took the node down — bench5's receipt, R6a's FAIL.
Fix in this round because re-cert reruns R6a anyway: catch
`requests.RequestException` at the call site into the loud-refusal path
(refusal names the failure class, never the URL's credentials — there are
none in it, but the doctrine is stated), plus a unit test that a
transport-raising `query_vlm` yields REFUSED and a live node. R6a's re-cert
expectation becomes: loud refusal text captured, camera down, NODE ALIVE.

## 8. Re-cert plan — which items rerun

| item | rerun? | why |
|---|---|---|
| R1 refusal ladder | (a)/(b)/(c) NO | untouched by this round |
| R2 → R2′ hit rate | YES, per-class bars (§4) | schema + prompt changed; needs Scott staging (3 objects), ~10 min |
| R3 absent probes | YES — the falsifier, runs FIRST | zero-FP bar re-proven on `match` |
| R4 FOV/geometry | NO | constants and sectors untouched |
| R5 ToF counter | NO | nothing touches timing; the charter receipt is banked |
| R6a transport failure | YES | now expected to PASS (refusal + alive) after §7 |
| R6b silent topic | NO | mechanism proven 2026-08-20, untouched |
| R6c clean recovery | YES | one clean invocation closes the sitting |

Scott's staging is ONE short batch (R2′ objects + R3 probe words + present
for R6a/c), ~15 min, zero motion — requested via the PM when the offline
calibration (§6) has already passed.

## 8b. CALIBRATION RESULT (2026-08-20, appended after PM ruling)

**COMPLETE AT 1 OF 6 ITERATIONS — the committed prompt cleared every gate
with zero prompt changes.** Frozen corpus (24 frames, manifest
`diagnostics/recognition_calibration_manifest_2026-08-20.json`), run on the
Pi against `syn:large:vision`, record
`~/recognition_calibration/iteration_1_20260820_113341.json`:

- absent 7/7 `match=false` (0.90–0.98) — **the zero-FP record survives the
  split**;
- bottle 10/10 `match=true`, 10/10 `identity=unverified`, zero false
  mismatches — including BOTH forced-false exhibits (104430 → match 0.80,
  105052 → match 0.94): the defect this round exists for is fixed on its own
  evidence;
- controller 3/3 `match=true`, identity unverified/confirmed/unverified —
  PASS under the AMENDED gate (PM ruling, recorded verbatim in the
  manifest's `_amendments`: "xbox controller" is brand-qualified as a fact
  of the frozen target string; the design note's §4 table had misclassified
  it as the plain exemplar, and demanding confirmed on ~60 px of logo-free
  pixels would reward exactly the false confidence the zero-wrong-confirmed
  bar forbids);
- picture 4/4 `match=true` unverified 0.55–0.70 (recorded, ungated) — the
  class the old schema scored 1-of-3-plus-1 now reports the object every
  time with honest identity doubt;
- schema-valid 24/24, zero parse breaches.

**Sitting-plan addition (the amendment's live coverage patch):** one fresh
invocation with the identity-free target **"game controller"** on the staged
controller — the true plain-object confirmed-probe (expectation:
`match=true`, `identity=confirmed` per the no-identity-component rule).

## 8c. RE-CERT RESULT (2026-08-20 afternoon — the round closes)

**Every bar met; full ledger on the bench card (RE-CERT RESULTS section).**
Highlights against this note's own asks: the zero-FP bar held live (7/7
absent probes zero-match), both §4 identity bars held (zero mismatch on
true targets, zero wrong confirms — the one brand-confirm was factually
correct per ground truth, with an epistemic thinness note recorded), the
decoy produced mismatch-with-named-evidence (PIN 1 live on both branches),
the §8b plain-object probe returned match+confirmed exactly per the
no-identity-component rule, and the §7 fix proved itself live
(refusal+alive where the morning had node death). The third-party-gamepad
falsifier is noted AVAILABLE-UNEXERCISED. The round's remaining step is
the tool-enable flip (reviewed commit, watcher-flip pattern); the live
search rehearsal is out of this round — it drives, so it is a flight.

## 9. Explicitly OUT OF SCOPE this round

- **Orientation-robustness** (objects at odd poses/rotations): the camera
  mount is fixed (declined 2026-08-09) and the two-stage design treats hard
  views as approach-and-relook, not prompt heroics.
- **Camera-while-moving**: charter says never; R5's measured −0.4–0.7 Hz
  ToF drag is the banked baseline any future proposal must clear. Not here.

## 10. Consensus asks (the decisions this note needs)

1. `identity` vocabulary: three values + null as specified (or collapse
   `mismatch` into `unverified` and lose the pruning signal)?
2. Per-class bars as tabled in §4 (notably: picture class recorded-not-gated,
   branded class allowed `unverified` at standoff)?
3. Bundling the §7 exception fix into this round: yes/no?
4. Offline-corpus calibration before the live sitting (§6): ratify the
   pre-registered gates?
