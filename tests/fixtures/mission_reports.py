"""RECORDED mission reports, 2026-08-12 gauntlet reset, as fixtures.

Both reports lifted verbatim from the latched `/coverage_explorer/report` topic dumps
archived in the vault. Not summarised, not rounded, not reconstructed from the defect
register: these are the JSON payloads the rover actually filed.

  vault 03_validation/run_2026-08-12_gauntlet-reset_mission1/report_20260812_112919.txt
    sha256 ec4ca512cf97385bf62bb5a4f1270c54c190c6abead25fec9a6bac21644a9a26
  vault 03_validation/run_2026-08-12_gauntlet-reset_mission2/report_m2.txt
    sha256 0e8a4a7394e5191ad613f0dd1e7e470ff6267f6be09d5f1611650ad9c8fce50c

Provenance worth knowing before you trust these: both runs were pulled off the Pi on
2026-08-12 into a session scratchpad under /private/tmp and were not moved to the vault
until 2026-08-13, by which point a session had already searched 03_validation/ and
concluded they were never captured. The vault copies are byte-verified against the
scratchpad copies; the scratchpad-to-Pi link rests on the 08-12 session's log, because
the Pi was down when they were recovered and the originals could not be re-hashed.

**These are OLD-SCHEMA reports and that is the point.** They carry `freeze_marks`,
the pre-D35 field that held one entry per EVENT under a name that reads as places.
Nothing in the current code emits that key any more. Do not "modernise" this fixture:
its whole value is being the record the schema change was made in response to, and a
fixture edited to match the code it is testing has stopped being evidence.
"""

import json

# Mission 1 (run 112721) -- the PASSING run of the reset: honest-blocked, both halves
# of the room worked. Nine freeze entries, six places, `(-0.847,-1.094)` four times.
# This is D35's evidence.
MISSION_1_REPORT_JSON = (
    '{"outcome": "INCOMPLETE_BLOCKED_BY_UNSEEN_OBSTACLES", "complete": false, '
    '"covered_cells": 3473, "covered_area_m2": 8.683, "duration_s": 518.6, '
    '"goals": {"sent": 19, "succeeded": 8, "aborted": 7, "planner_rejections": 8}, '
    '"remaining_candidates": 3, '
    '"freeze_marks": [{"x": 1.466, "y": 0.537}, {"x": 0.37, "y": 0.507}, '
    '{"x": -1.367, "y": -0.25}, {"x": -0.994, "y": -0.409}, '
    '{"x": -0.797, "y": -1.366}, {"x": -0.847, "y": -1.094}, '
    '{"x": -0.847, "y": -1.094}, {"x": -0.847, "y": -1.094}, '
    '{"x": -0.847, "y": -1.094}], '
    '"map_files": ["/home/jsperson/.ros/missions/mission_20260812_113804.pgm", '
    '"/home/jsperson/.ros/missions/mission_20260812_113804.yaml"]}'
)

# Mission 2 (run 125305) -- the NO-COUNT run: died unplannable with every rung untried
# at the final pose, 96 planner rejections, zero aborts. D36's close-criterion source.
# Five freeze entries, five places: this report's count was NOT misleading, which makes
# it the control case for D35.
MISSION_2_REPORT_JSON = (
    '{"outcome": "INCOMPLETE_NO_PLANNABLE_TARGETS", "complete": false, '
    '"covered_cells": 2037, "covered_area_m2": 5.093, "duration_s": 160.9, '
    '"goals": {"sent": 3, "succeeded": 2, "aborted": 0, "planner_rejections": 96}, '
    '"remaining_candidates": 4, '
    '"freeze_marks": [{"x": -0.488, "y": 0.01}, {"x": -0.779, "y": -0.199}, '
    '{"x": -1.134, "y": 0.295}, {"x": -0.853, "y": 0.559}, '
    '{"x": -1.135, "y": 0.84}], '
    '"map_files": ["/home/jsperson/.ros/missions/mission_20260812_125612.pgm", '
    '"/home/jsperson/.ros/missions/mission_20260812_125612.yaml"]}'
)

MISSION_1 = json.loads(MISSION_1_REPORT_JSON)
MISSION_2 = json.loads(MISSION_2_REPORT_JSON)

# The freeze events as the explorer collected them, ready to replay into build_report.
MISSION_1_FREEZE_EVENTS = MISSION_1["freeze_marks"]
MISSION_2_FREEZE_EVENTS = MISSION_2["freeze_marks"]
