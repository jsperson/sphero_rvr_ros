# Engineering standards

Rules this project bought with real failures. Each one is here because ignoring it cost a
mission, a chassis run, or a wrong verdict — the incident is named so the rule can be argued
with on evidence rather than obeyed on authority.

Collected 2026-08-14; until then these lived scattered across design notes and defect rows,
which meant each was re-learned by whoever next tripped over it.

---

## 1. Operands, not arithmetic

**When a number turns out wrong, the arithmetic is usually right and the OPERANDS are wrong.**
Name the population and the denominator before quoting any figure.

Four times in 48 hours this project computed a correct number about the wrong set: a coverage
"best ever" measured against best-since-reset rather than best-ever (twice — 8.683, then 10.285
against 7.188 when 10.128 was the real record); a churn falsifier that passed by measuring the
wrong population; a rule keyed on being *near* a target, proven only *far* from one.

**In practice:** state the set and the denominator in the same sentence as the number. "Best of
the four post-reset runs" and "best ever" are different claims, and only one of them is usually
true.

## 2. A new envelope or anchor obliges re-checking every constant derived under the old one

**Changing a model silently invalidates every quantity measured against it.** The change looks
local; the blast radius is not.

* The ToF envelope model was re-derived on 2026-08-14 morning, and nothing prompted a re-check
  of `low_obstacle_stop_distance_m` — which had been derived under the old one and now sat
  inside the sensor's blind zone (D39). The brake could not fire for the object height it
  existed to stop for, and *released* as the object vanished.
* D29 made the stack come up disarmed, moving the mission's start event. Nothing re-anchored
  `duration_s`, which still measured from node-ready — a 2.94× over-report (D41) that
  invalidated cross-run coverage-*rate* comparison in both directions for every run since.

Note the second is not a sensor envelope at all. **The rule is about any re-anchoring change**,
not about ToF.

**In practice:** when a model, anchor, or lifecycle event moves, enumerate what was derived
under the old one and re-check each. The list is usually short and never empty.

## 3. Every guard needs an existence check AND a defeat check

A guard can fail three ways, and mutation testing catches only the first two:

1. **The mechanism is absent** — it was never wired up.
2. **The check is defeatable** — it passes things it should refuse.
3. **The input is fabricable** — the guard is real and correct, and is fed synthetic data that
   cannot exercise it.

**Synthetic inputs need the same adversary as code.** A fixture that cannot produce the failure
makes a green test meaningless, and green is what everyone reads.

## 4. Fail-closed is respected by DERIVING, not by vetoing everything

The correct conservatism is to compute what a command actually does and refuse on *that*. A gate
that refuses every rotation because one bearing is close is fail-closed in the same sense that a
brick is a safe car.

The mirror failure is equally live: a guard that fails closed can silently reject *everything* —
the 2026-08-10 goal-clearance filter rejected 125 of 125 frontier candidates when no costmap was
present. **An inert component must warn loudly, never just decline.**

## 5. The recovery-defect family — three forms, three checks

A recovery mechanism exists, reviews cleanly, and never produces motion. Four instances, three
distinct forms:

| form | failure | the check |
|---|---|---|
| **unreachable** | the code path never runs | does this path execute? |
| **never-triggered** | it runs; its condition never occurs | does the trigger fire in real run logs? |
| **un-grantable by construction** | it runs, is reached, and is refused every time because of its **command shape** | **can this command shape ever be granted at the poses where it is meant to fire?** |

* *Unreachable*: the pivot controller sat below a raw-motor branch — a parameter sweep that
  changed nothing was the signature.
* *Never-triggered*: the explorer's `_unstick` was gated on selection failure while every real
  mission died on drive failure. Zero invocations, four tuned parameters.
* *Un-grantable*: the D36 give-up escape commanded `(-v, 0.0)`; `rear_hold` zeroes linear and
  passes angular through, so the command became `(0.0, 0.0)` at every pose where the rear sector
  was inside `reverse_stop_distance_m`. Four attempts, four refusals, 0.000 m each (2026-08-14b).

**Form 3 evades the other two checks** — the code runs, the trigger fires, and the refusal
happens in a *different module*. **Ask the check of the ARBITER, not the caller.**

Corollary: a proven primitive for the same problem may already exist elsewhere in the codebase
(the stall ladder's `REVERSE_ARC` rung encoded this exact lesson months earlier). Look before
inventing.

## 6. A config file is a claim; the robot's state line is the robot

Gate on what the robot says about itself, never on what the config says it should be.

On 2026-08-14 `/tof/state` reported `rule_b=UNPINNED margin_m=0.1 rules=rule_a_only` with the
pinned margin sitting in the config and every test green — `tof_node` declared its own literal
parameter defaults, which overrode `TofConfig`. One gate away from flying "the first rule-B
mission" with rule B off. It was caught because the run card gates on the state line's own words.

Same discipline for tests: probe the **deployed** YAML, not dataclass defaults. Thirteen fields
have differed between the two, and a verdict flipped between them.

**And a gate must measure the quantity it claims to.** `/tof/state`'s `rate_hz` is
`frames / elapsed` since node start — a cumulative average that cannot report the current rate
and reads below band for minutes after startup. A genuinely slow sensor would be
indistinguishable from a healthy one warming up.

## 7. Cut segments on the data's own signature, never on relayed marks

Session marks relayed by a human carry human reaction time plus relay latency. Segment boundaries
come from step changes in the recorded data itself.

Related: pair observations to the recording before comparing them. A cardinal-direction note and
a costmap dump describe the same bearing only if the robot did not rotate in between — on
2026-08-10 it had, and neither reading could confirm or refute the other. Nothing was wrong with
either observation; they simply could not be compared.

## 8. Revert-proofs must fail against the code they indict

A proof that has never failed is not a proof. Write the test first, run it against the unfixed
code, and record that it failed. Then fix, and record that it passes.

Mutation is the check on the check: change the fix back and confirm the proof goes red. Four
false proofs were caught in a single night by this rule.

Bind the proof to **production code**, not to a restatement of it — a test that hardcodes the
shape it is meant to be verifying proves only its own consistency.

**Corollary (2026-08-14): when a proof cannot bind to production without restating it, the
minimum structure that lets it bind IS in scope** — flagged loudly, never done silently. D40's
revert-proof needed the escape's command shape, but the node imports `rclpy` unguarded and cannot
be imported by any dev-machine test; extracting the shape into a pure core function was the only
way for the proof to test what actually goes on the wire. Extracting for testability is
legitimate; extending capability under cover of it is not, and the difference belongs in the
commit message.

**Docstring standard for safety-adjacent pure functions.** A function the safety path depends on
carries, in its own docstring: the defect it exists to kill, the failure family it belongs to,
the check that catches that family, any trust hierarchy among its inputs, and the degenerate-input
guard. The next person to touch it should not need the design note to know why it is shaped that
way.

---

## Appendix A: the premise tripwire — a test that SHOULD survive its own mutation

Most tests in a revert-proof set go red when the fix is reverted. A **premise tripwire** does not,
and that is its job: it asserts the *external contract the fix depends on*, not the fix.

D40's example: `test_the_straight_reverse_the_escape_used_to_send_is_ungrantable_here` asserts that
the supervisor's `rear_hold` zeroes **both** axes for a straight reverse at the recorded pose. It
passes with the fix and passes with the fix mutated away, because it is a statement about
`rear_hold`, not about the escape. If it ever goes red, `rear_hold`'s contract has moved and the
revert-proof beside it is silently measuring something else.

**Write one whenever a fix is justified by another component's behaviour.** Name it so its
survival under mutation reads as intent rather than as a weak test, and say in the docstring what
its going red would mean. Without it, a change to the depended-on component turns a real proof
into a tautology with no alarm.

The same idea covers recorded evidence: assert that the replayed geometry still sits inside the
deployed threshold it is supposed to trip. If a config moves, the test should say "this pose no
longer reproduces the failure" rather than quietly passing for a new reason.

## Appendix B: operational traps that look like bugs

Not standards, but they have each cost a session and are invisible from a log:

* **`git pull` prints `Updating <old>..<new>` while aborting.** Trust `git rev-parse`, never the
  pull's narration.
* **`pkill -f <pattern>` matches your own SSH command line** and has killed the operator's
  session mid-teardown four times. Build a PID list, filter your own session out, kill by
  explicit PID.
* **`ros2 run` spawns the node as a CHILD.** Killing the wrapper leaves the node alive holding
  its device. Verify the *port* is free, not that a pid is gone.
* **`ros2 topic echo` truncates arrays** — use `--full-length` for anything with ranges.
* **Same-byte-length Python edits can run a stale `.pyc`.** Verify tiny fixes with a must-flip
  test or clear `__pycache__`.
