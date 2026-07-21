# Project agent instructions

## Bounded test execution

- Never run bare `pytest` or `python -m pytest` for this repository.
- Use `python3 scripts/run_pytest_bounded.py --timeout 90 -- -vv` for the full suite.
- Use the same runner with `--timeout 60` for focused suites.
- Do not run concurrent full suites against the same checkout.
- The runner enforces a repository-wide nonblocking lock; exit code `75` means another test run owns it and is **NO VERDICT**.
- Exit code `124`, `BOUNDED_PYTEST_TIMEOUT`, interruption, or external termination is **NO VERDICT**—never PASS or a technical code-review FAIL.
- After a timeout, verify the process group was reaped. Rerun once with `-vv` and the same hard deadline to identify the last test; repeated timeouts are test-infrastructure findings.
- Record the exact SHA, command, duration/result, and cleanup in review handoffs.
