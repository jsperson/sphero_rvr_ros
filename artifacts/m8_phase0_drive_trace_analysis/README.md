# Milestone 8 Phase 0A drive-trace analysis

This directory contains compact, reviewable derivatives of the private physical
trace. The raw JSONL stays on `sphero-pi-2` with mode `0600` and is deliberately
not committed.

- `context.json` records the goal geometry and semantic-finish evidence read
  from the Pi mission/binding databases. Event payload digests preserve the
  database bindings without copying those private databases.
- `report.json` is the deterministic output of
  `sphero_rvr_driver.drive_trace_analysis`. Its Phase 1 motion-evidence routing
  keeps geometry-ineligible and command-inconclusive trials distinct from the
  one result that can justify a Phase 0B breakaway sweep.

The source trace is mission
`m7-canonical-e90f7828e13843d981eab942b16751a4`, executable source
`c8cbff35d156332806f0fe8d16b47b23514eac6d`, 2,793,335 bytes, SHA-256
`54a6af978736c1f1dfac2fe405b20b7a0049429b175d5bcb6ee146172dbc49a0`.

Reproduce on a trusted host that has the private trace:

```bash
PYTHONPATH=src python3 -m sphero_rvr_driver.drive_trace_analysis \
  /path/to/drive-trace-m7-canonical-e90f7828e13843d981eab942b16751a4.jsonl \
  --context artifacts/m8_phase0_drive_trace_analysis/context.json \
  --output artifacts/m8_phase0_drive_trace_analysis/report.json
```

The analyzer rejects mixed mission/source provenance, a raw-trace digest that
does not match the context, invalid context event digests, missing trace
boundaries, and missing motion/completion events.
