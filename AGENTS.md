# Project agent instructions

## Cold-start sources of truth

- Canonical project status and next action live only in the Obsidian vault at
  `Projects/Sphero RVR ROS/Current Status.md`. Read it for grounding; do not edit it.
- Repository `STATUS.md` is a pointer, never a second status document.
- `docs/architecture_map.md` is the ownership/seam map. Consult it when you need to
  know a boundary. Update it ONLY when a change actually moves an ownership/seam
  boundary — routine, scoped fixes do not touch it. Phase documents are historical
  records, not competing current maps.

## Roles and governance

Updated 2026-08-09. **Codex is retired.** This section previously said Codex was the
sole committer and that agents must not edit the vault; both were stale and
contradicted `Current Status.md` and the Forward Plan's own frontmatter.

- **Claude implements, commits, and pushes directly to `main`.** No branch/PR
  ceremony is required. Scott decides, approves, and runs anything physical.
- **Claude owns the vault** (`Projects/Sphero RVR ROS/`) and keeps it current and
  lean. `Current Status.md` is the narrative; `Open Defects.md` is the checklist and
  is authoritative for what is broken; `01_planning/Forward Plan to SOTA.md` is the
  active roadmap. Requirements and scope remain Scott's call — do not rescope
  unilaterally.
- **Claude operates the Pi over SSH (`ssh sphero-pi-2`)** for deploys, bench work,
  and read-only inspection. **Actuate the rover ONLY when a goal explicitly
  authorizes a physical test**, and only attended (Scott present, power-cut
  reachable), bounded and low-speed. Otherwise code, bench, and tests only.
- Chassis runs are the project's scarce resource. Never spend one on something a
  bench test or an offline harness could have caught, and never start one without
  the run recorder capturing `/collision_stop/state` — an unrecorded failure has to
  be repeated.

## Working discipline (lean by default)

- One concern per goal. Make the smallest change that satisfies it.
- Opt into rigor; do not default to it. Unless a goal explicitly asks, do NOT add:
  scaffolding modules/classes, validation/`*_canonical`/`*_audit` harnesses,
  evidence artifacts, status docs, doc prose, or new tests for scaffolding you
  just built.
- Plan-gate: for any scoped fix, before writing code reply with the exact files,
  approximate line count, and any new symbols, then STOP and wait for explicit
  "go." A goal may waive this only if it says so.
- If a fix seems to need more files, new classes, or materially more lines than a
  goal implies, STOP and explain instead of expanding scope.

## Bounded test execution

- Never run bare `pytest` or `python -m pytest` for this repository.
- Use `python3 scripts/run_pytest_bounded.py --timeout 90 -- -vv` for the full suite.
- Use the same runner with `--timeout 60 -- -vv <focused paths>` for focused suites.
- Quiet mode is rejected because a timeout must identify the last test reached.
- Do not run concurrent suites against this repository.
- Exit `75` = another run owns the repository lock = **NO VERDICT**.
- Exit `124` = timeout = **NO VERDICT**.
- Exit `125` = leaked descendants / incomplete cleanup = **NO VERDICT**.
- Exit `129`, `130`, or `143` = interruption / signal cleanup = **NO VERDICT**.
- On timeout or interruption, verify the process group was reaped. A repeated
  bounded verbose timeout is a test-infrastructure finding, never PASS or FAIL.
- Report the command run and its result in chat. No formal handoff artifact,
  SHA/duration log, or evidence record is required unless a goal asks for one.
