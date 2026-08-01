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

- Codex is the sole committer to the repository. Push to a branch/PR; never merge
  your own PR. Scott merges after review.
- Do not edit the Obsidian vault. Report status and results in chat; the reviewer
  keeps the vault canonical.
- Never contact, deploy to, or actuate the rover. Scott runs every physical step,
  attended. Code and tests only.

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
