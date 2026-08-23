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

## Appendix A2: a module states the SHAPE it consumes; the deployed config states the SOURCE

**Naming a topic, device, or producer in source prose is a config-is-a-claim hazard**,
because the claim is unenforced and outlives the arrangement it described.

`low_obstacle_brake.py`'s module docstring said the layer reads `/camera/low_obstacles`.
The deployed `collision_stop.yaml` has pointed it at `/tof/obstacles` since the ToF took
over the brake, so the file confidently described a system that had not existed for some
time — and the same stale sentence still sat in `explore.launch.py`'s
`start_low_obstacle` help text, where it told an operator that enabling the camera would
feed the brake. It would not; it would publish to a topic nothing reads. Landed
2026-08-15 (`b8bc0b9`).

**In practice:** the module says what it consumes — "base_link points", "an 8x8 frame of
millimetre readings" — and the deployed YAML says where that comes from. Where a
producer-specific fact genuinely matters (the camera's ~0.06 m origin offset), attach it
to the producer by name and say it applies only to that one.

**The wider rule:** any sentence in source that a config file can falsify is a claim
under standards rule 6, and it is worse than a wrong constant, because nothing gates on
prose. The test that catches it does not exist — a reader does. So prefer not writing
the claim.

## Appendix A3: the ruling names the INTENT; the implementation owns the SEMANTICS

Two corrections on 2026-08-15, both in the same direction, both worth keeping as a
pair because they show where a review's authority ends.

**One.** The ruling was: a non-translating command should be refused by returning
`None` with a reason. But in `low_obstacle_brake`, `None` already means *the swept
path is clear*, and `forward_speed_scale(None, ...)` returns 1.0 — full speed. So the
prescribed form would have handed out the most permissive answer in the API for the
one input the function cannot model: **fail-open wearing a refusal's clothes**,
achieving the exact opposite of what was asked for. It raises instead. Mutating the
raise back to the `None` form kills all five refusal cases, which settles it
empirically rather than by argument.

**Two.** The same ruling said `v == 0`. The function's own `turning` test needs
`|v| > 1e-3`, so exact-zero would have left the band `0 < |v| <= 1e-3` still falling
into the straight branch and still answering a near-pivot with a forward corridor —
the identical defect at 1e-4 m/s. The guard uses the *same constant* the turning test
uses, so the two cannot disagree about where translation begins.

**The rule:** a ruling is binding on WHAT should be true — here, "a command with no
swept path must be refused, and refusal is the scope, not annulus modelling." It is
not binding on a form whose local semantics the reviewer could not see. When the two
conflict, implement the intent, say plainly that you deviated and why, and make the
deviation falsifiable — a mutation that shows the prescribed form failing is worth
more than the argument for it.

The failure mode this prevents is the obedient one: implementing a form that reads
correct, passes review because it matches what was asked, and is fail-open in a
module the reviewer was not holding in their head.

## 9. The footprint is a claim about where the robot ENDS; hardware changes re-open it

**Every stop distance is derived from the declared footprint, so the footprint is a
safety constant — and unlike the others, it changes when someone picks up a
screwdriver, with nothing in the repo to notice.**

On 2026-08-15 the rover's rangefinder physically struck a table leg (Scott, attended).
Reconstructed from the bag: the leg sat at **≈0.091 m from `base_link`** at contact,
against a declared `footprint_front_m` of **0.110 m** and a ToF optical origin
(`mount_x_m`) of **0.100 m**. The declared front allows only **10 mm** over the
sensor's optical reference, so any lens, housing or bracket protruding further makes
the rangefinder the foremost hardware — outside the footprint every gate is derived
from. Compounding it, `min_valid_mm = 60` converts through the mount geometry to a
floor of **x ≥ 0.152 m** on anything that sensor can ever report: a **42 mm band ahead
of the bumper that is structurally invisible, with the foremost hardware inside it**.

This is standards rule 2 (a new model obliges re-checking every constant derived under
the old one) arriving through a door nothing watches: **the model that moved was
physical**. Adding a forward-mounted sensor silently re-derived "where the robot ends"
and no constant was re-checked.

**In practice:** measure extents to the FOREMOST HARDWARE, not to the chassis, and
re-measure whenever anything is mounted, moved, or re-bracketed. State beside the
footprint constants what physical part each one is measured to. A sensor that reaches
past the footprint is a sensor that will touch what it cannot see.

## 10. An instrument that recorded nothing does not outrank a human who watched it

**Absence of evidence in a window you chose is not evidence of absence.**

The same night, autopsy #2 concluded "no contact appears in this recording" and offered
that against Scott's direct report of a collision. Both halves were wrong. The contact
was real, and it was in the OTHER run — the analysis had searched the episode keyed to a
relayed timestamp rather than every close approach in both bags, exactly as Scott
suspected (*"perhaps you're looking at the wrong part of the recording"*).

Note the honest claim available at the time was **"no contact in the six seconds I
examined"**, which is much weaker and would have prompted the wider search immediately.
The stronger phrasing was not supported by the work done, and it was the phrasing that
got weighed against an eyewitness.

**In practice:** when an instrument disagrees with an attended observation, the
instrument is the hypothesis and the observation is the datum. Widen the window and
the population FIRST — every episode, both directions, all runs — and state the search
extent in the same sentence as the negative result, or do not report the negative.

## 11. A belief is a different population from a reading

**Every filter written for sensor readings must be re-justified before it touches a
remembered one — and the same goes for every rule about what may update it.**

The D39 hold (2026-08-15) gives the brake a memory: when returns vanish inside the
sensor's structurally invisible band, it keeps believing the obstacle instead of reading
silence as clearance. Two defects were written into the first draft, caught by the
mutation discipline, and they are the same mistake twice:

* **A validity floor applied to a belief.** The corridor test passed
  `low_obstacle_min_range_m` (0.14 m) through, exactly as the live path does. That floor
  exists to reject *readings the producer cannot be trusted on*. Applied to a belief, it
  drops the object out of the corridor as soon as it transports inside 0.14 m — **releasing
  the brake for the objects held closest**. The original defect, rebuilt inside the class
  written to prevent it, out of a window borrowed from the wrong population.
* **A farther sighting retiring a nearer belief.** The draft handed each live reading
  straight back. Run 1's stray 0.201 m return — the one that restored full commanded speed
  seconds before the collision — would have overwritten the 0.181 m belief. With the rover
  stationary, a return further out is not evidence the object receded; it is evidence of
  partial visibility, which is the failure mode itself. Left in, a thin object could be
  walked outward one legal-looking return at a time until it retired itself.

The clause that resolves the second is the mechanism's soul: **only MEASURED MOTION may
push a belief away.** A reacquisition can pull a belief nearer; nothing a sensor says can
push it further.

Note that neither defect is visible from the live path's tests, and neither would have
failed a review that asked "does this match how the live path filters?" — matching the
live path is what produced both.

**In practice:** when a module gains state, enumerate every filter, threshold and update
rule it inherited and say out loud what population each was derived for. Then mutate the
new state away and check the revert-proof still dies on the field symptom; both of these
were found that way and neither was found by reading.

## Appendix A4: an archive cannot always tell two rules apart

**A test written over recorded specimens passes when the specimens cannot distinguish
the rule from its absence — and it looks like the strongest kind of test there is.**

The 2c execute stage (2026-08-15) has two neighbouring rules: a FREEZE retires the
whole direction, a REFUSAL retires only that command shape at that clock. Both were
tested by replaying the three archived wedge poses.

Both tests passed. Neither could have failed. **On all three specimens every clock
carries exactly one candidate**, so retiring "the direction" and retiring "the shape at
that clock" remove the identical thing, and the two rules produce byte-identical
sequences. The freeze rule could have been deleted entirely with the suite still green.

The fix was not more specimens — no archived pose has the needed shape. It was to
construct the one pose where the rules diverge (two candidate kinds at a single clock)
and then **mutate each rule and require that it is killed by its own test and only its
own**: conflating freeze into refusal must kill the freeze test and leave the refusal
test green, and the reverse must do the reverse. A rule whose mutation kills nothing is
untested; a rule whose mutation kills *both* tests means the tests are measuring one
thing twice.

This is the sibling of "one configuration cannot separate hypotheses" (a level bench
hid three sensor-model errors because the constraint was correlated). Same shape, moved
from measurement into testing: **replaying real data is not automatically a stronger
test — it is only stronger when the data can tell the answers apart.**

**In practice:** for any pair of neighbouring rules, name the input feature that makes
them diverge and check the fixtures actually vary it. If none does, construct the case
and say in the test that it is constructed and why. Then mutate both ways.

## Appendix A6: the fifth way a verification artifact lies

The taxonomy was: (1) the cited mechanism does not exist, (2) the check is defeatable
without doing the work, (3) the input is fabricated in a way reality never produces, and
(4) the check never ran at all. **2026-08-17 added a fifth: THE CHECK DAMAGED THE THING IT
CHECKED, AND LEFT NO TRACE.** A killed mutation run left its edit in an untracked source
file; the tooling that normally reports such things (`git status`) is structurally unable
to see untracked content, so the instrument that would have caught it was the instrument
that reassured me. See rules 12 and 13.

## Appendix A5: a guard that greps prose fails on its own explanation

**Three times in one night (2026-08-16), a guard written to forbid a defect went red on
the comment explaining that defect — and the only way to make it green was to delete the
explanation.**

The pattern, each time:

* `test_costmap_window_scale.py` forbade the literal `253`. The module's docstring now
  explains, at length, why 253 was wrong. Red.
* `test_tof_launch_wiring.py` forbade `"camera.launch"` in the motion-stack launch. That
  phrase appears there only inside a comment saying the camera is deliberately *not*
  started. Red.
* `test_stock_middle_config.py` forbade `RotationShimController`, `DenoiseLayer` and
  `PoseProgressChecker`. All three appear in the config's comments explaining why they
  are absent. Red, red, red.

**The failure mode is not the red test — it is what the red test makes you do.** The
cheapest way to green is to remove the sentence that records why the defect existed,
which is usually the most valuable line in the file. A guard that punishes documentation
will, eventually, get the documentation deleted.

**In practice:** assert on what the machine reads, not on what the human wrote.

* YAML → strip `#` lines before matching.
* Python → parse with `ast` and inspect literals, assignments and call keywords.
* Launch files → assert on the actual declared arguments and node actions.

And when the assertion genuinely *is* about the prose — "this comment must still say
what the node does not do" — say so explicitly and read the raw text on purpose, in a
separate helper with a name that admits it.

**The tell:** if a guard would pass on an empty file, or would pass more easily after
deleting a comment, it is measuring the wrong surface.

## 12. A killed mutation run is a DIRTY-SOURCE event, and git will not tell you

**Bought 2026-08-17.** A mutation harness edits source in place and restores it after each
run. I killed one that had hung, and it left its mutation applied: a live `and False` in
`chassis_sim.py` that disabled the simulator's encoder path. **The file was UNTRACKED, so
`git status` showed nothing** — and I had run my usual tree check and *been reassured by
it*. I found the corruption only by debugging a test failure that made no sense. Had I
committed at that moment I would have shipped a silently crippled model, and every
closed-loop result produced from it would have been confident garbage.

**The rule:** a mutation harness must verify source integrity against its backup after
**every** restore (`cmp`, not faith), and abort loudly on mismatch. **A tree check that
consults only `git status` is blind to exactly the files mutation testing touches most —
new ones.** Killing an in-place-mutation run is a dirty-source event and must be treated
like one: verify before anything else, including before believing a test result.

## 13. A test that CAN hang is a test that WILL hang

**Same incident.** The mutation that stopped answering encoder polls made a test block
forever on a bare `await queue.get()` rather than fail. **A hung mutation run wearing a
perfect score is the worst artifact in this repo's taxonomy** — it reports `killed=N` while
proving nothing, and the only tell is the clock.

**The rule:** every `await` in test code gets a timeout. `asyncio.wait_for`, always. A
test's job under mutation is to *fail*, and a test that cannot fail promptly cannot do it.

## 14. Name the endings you REASONED, not only the ones you MEASURED

**Bought 2026-08-18.** The §3a retest card pre-registered two possible endings, both drawn
from measured behaviour: an approach-stall near clutter, and a hunt signature. The flight
ended in neither — it ended in **repeated contact with a chair leg**, because the stock
middle runs with **no touch response at all**: the freeze/touch classifier lives in the
decisive controller, which that configuration does not start, and the D48 consumer had been
deliberately banked.

**That fact was written on the certification page itself.** It was known, reasoned, and
documented — and still left off the card, because the endings named were the ones that had
shown up in data.

**The rule:** when a card names expected endings, include the ones derivable from **known
structural facts**, not only those observed in a previous run. **A structural gap recorded
in your own documentation is a named ending by definition.** The test: read the
certification's "what is NOT certified" section and ask what each unproven clause looks
like when it fails in the field — those are endings, and naming them costs one line each.

## 15. Flights go through the guarded tool, not a hand-rolled command

**Same day, same flight.** The bag topic list was hand-rolled at the shell and dropped
`/tof/obstacles` — the one topic the contact analysis most needed. `scripts/launch_and_arm.py`
records it, and has a **guard test** over that exact list, written the day before after
`/diagnostics` went unrecorded and cost an autopsy its answer.

**The lesson is not "remember the topic".** It is that a guarded tool exists precisely so
the list is not retyped from memory, and bypassing it discards the guard silently. If the
tool lacks a flag a flight needs, **add the flag as a reviewed change** — do not step around
the tool. Third member of the manifest family (uninstalled config, unrecorded
`/diagnostics`, unrecorded `/tof/obstacles`), and the first where the protection already
existed and was walked past.

## 16. A skip-decorator hides a file from PARSING, not just from running

Found 2026-08-22 by a syntax error that reached the Pi: a duplicated keyword
argument in `coverage_explorer_node.py` passed **1,393 green tests on the Mac**
and failed at `colcon build`. The reason is structural, not careless — every
driver-module test begins `pytest.importorskip("rclpy")`, so on a host without
ROS those files are never imported, and a file that is never imported is never
PARSED. A whole package's syntax was unverified on the machine where the work
happens, and had been for months.

**The rule: any test population gated on an unavailable import is blind to
syntax in the files it skips. Something ROS-free must parse them.**
`tests/test_driver_sources_are_wellformed.py` does exactly that, and its
must-flip was run in both directions (the exact duplicate re-inserted, seen to
fail, restored, seen to pass) before it was trusted. The same file pins that
every `build_report` call site carries the same counter set — the generalised
form of the defect, since a report that lies by omission at ONE call site
(START_BLOCKED, until 2026-08-16) is the same class.

## 17. A PIPE SWALLOWS THE EXIT CODE, so `&&` cannot protect a landing

Stated wrong on the first attempt (2026-08-22) and corrected the same night by
review, because a norm with a wrong mechanism is itself a lie-generator. The
first draft blamed `&&`. That is exactly backwards: `&&` works — a failing
`pytest` returns non-zero and the next command never runs.

**What actually failed was the PIPE.** The command was:

```
python3 -m pytest tests/ -q 2>&1 | tail -1 && git add -A && git commit ...
```

A pipeline's exit status is its LAST command's. `tail` succeeds at tailing a
report of failures, so the pipeline exits 0, and `&&` cheerfully proceeds to
commit a red suite. The protective construction was defeated by the convenience
of trimming output — and the trimming is why the failure was invisible: `-q |
tail -1` prints one summary line that says "1 failed" while the shell reports
success.

**The rules, both of them:**
1. **Landing commands take a BARE exit code.** Never put a pipe between a
   verification command and the `&&` that guards a commit, push or deploy. Read
   the output in a separate step, or use `set -o pipefail` where the shell
   supports it.
2. **Read the number before you land.** A summary line that says "1 failed" is
   only useful to someone who looks at it; the commit does not.

Landed a red test on 2026-08-22 (D62), caught in the same breath, amended,
force-with-lease on an unreviewed tip. The defect was one character of
convenience.

**AND THE REASON IT BIT A THIRD TIME IS THE LESSON UNDER THE LESSON.** The
pipe-eats-exit-code rule had been learned twice before — and lived only in one
participant's memory, never in this file. Nobody else could read it, so
everybody else re-learned it. **A rule that lives in one head will be
re-discovered by every head that does not have it, at full price each time.**
If a lesson is worth a norm, it goes in the repo where the next person — or the
next session — will actually meet it. Three occurrences is the receipt for this
one.

## 18. A log query over ssh can match its own reflection

2026-08-22, investigating the Pi's shutdown hang: `journalctl | grep "failed set
request"` reported USB errors on a boot where the device had never been used.
They were not errors. **`tailscaled` logs the full command line of every ssh
session it starts, so journald contained tailscaled's record of the grep itself,
and the pattern matched its own text.** The same effect inflated shutdown-marker
counts on a boot that had never shut down.

The cost was not noise, it was a REVERSED FINDING: the self-echo made a
three-boot correlation look perfect (USB errors ⇒ hung shutdown). Filtering the
echo and widening to six boots broke it — two boots carried the errors and shut
down normally — so a report already sent had to be withdrawn.

**Rules:** use `journalctl -k` when the fact is a kernel fact (tailscaled is not
kernel), otherwise `| grep -v tailscaled` before counting anything. Treat any
log-derived count taken over ssh as contaminated until one of those is applied.
An instrument that records the observer's question inside the observed data will
manufacture agreement with whatever the observer is looking for.

## 19. A graph count over ssh is warm-up-sensitive, so one read is not an observation

2026-08-23, a restarted worker's first five minutes. Three consecutive
`ros2 node list` calls over ssh against an unchanged Pi returned **5, then 18,
then 21**, and two further reads returned 21. Nothing started, nothing died, and
nothing was launched between them: the demo had been fully up the whole time.
Had the first read been reported, it would have said the standing demo was dead
— and it would have said so with no task_node and no web_console on the list,
which is exactly the shape of a real failure.

The mechanism is discovery, not flakiness. `ros2 node list` spawns a **brand-new
DDS participant** which must discover its peers before it can enumerate them,
and it prints whatever it has heard by the end of its short wait. Over ssh every
invocation is a fresh process on a fresh connection with no warm cache, so
**every read is a cold read** — the warm-up is paid again each time, and it is
paid in the same direction every time.

**That direction is the whole reason this is dangerous.** A cold read can only
under-report: it can miss nodes that exist, never invent nodes that do not. So
the error mode is a **false negative** — "the node is gone", "the demo is dead",
"the stack came down" — which is precisely the class of claim that gets acted on
urgently. And the instrument gives no hint: it exits 0 and prints a short,
plausible, well-formed list.

**Rules:**
1. **Repeat until stable.** A count is an observation only when at least two
   consecutive reads agree; on a graph this size, take three.
2. **Report how many reads agreed**, not just the number. "21 nodes, stable
   across 3 reads" is evidence; "21 nodes" is a sample.
3. **Never report an absence from a first read.** Absence is the failure mode
   this instrument has; re-read before telling anyone something is missing.
4. The same discipline covers the sibling CLIs that each spawn their own
   participant — `ros2 topic list`, `ros2 service list`, `ros2 node info`.

This is the fourth member of one family, and the family is the actual standard:
**the instrument the observer brings can distort the thing observed** — the
tailscaled echo of norm 18, a shielded run read as unshielded, a leaked
`ROS_DOMAIN_ID`, and now cold discovery. Three of the four produced a confident
wrong answer that someone had to withdraw. Before any count over ssh becomes a
finding, name what the act of measuring added or removed.

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
