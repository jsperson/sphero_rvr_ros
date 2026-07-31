# Project agent instructions

## Cold-start sources of truth

- Canonical project status and next action live only in the Obsidian vault at
  `Projects/Sphero RVR ROS/Current Status.md`.
- Repository `STATUS.md` is a pointer, never a second status document.
- After the vault status, read `docs/architecture_map.md`; it is the single
  maintained ownership/seam map. Phase documents are historical design and
  evidence records, not competing current maps.

## Bounded test execution

- Never run bare `pytest` or `python -m pytest` for this repository.
- Use `python3 scripts/run_pytest_bounded.py --timeout 90 -- -vv` for the full suite.
- Use the same runner with `--timeout 60 -- -vv <focused paths>` for focused suites.
- Quiet mode is rejected because a timeout must identify the last test reached.
- Do not run concurrent suites against this repository.
- Exit `75` means another run owns the repository lock and is **NO VERDICT**.
- Exit `124` means timeout and is **NO VERDICT**.
- Exit `125` means leaked descendants or incomplete cleanup and is **NO VERDICT**.
- Exit `129`, `130`, or `143` means interruption/signal cleanup and is **NO VERDICT**.
- On timeout or interruption, verify the process group was reaped. A repeated bounded verbose timeout is a test-infrastructure finding, never PASS or a technical code-review FAIL.
- Record the exact SHA, shell-quoted command, duration/result, and cleanup in review handoffs.
