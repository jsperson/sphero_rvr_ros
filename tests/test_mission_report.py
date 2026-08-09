"""Tests for the mission report and map serialization."""

import json

import pytest

from sphero_rvr_core.mission_report import (
    OUTCOME_COMPLETE,
    OUTCOME_NO_PLANNABLE_TARGETS,
    OUTCOME_START_BLOCKED,
    PGM_FREE,
    PGM_OCCUPIED,
    PGM_UNKNOWN,
    build_report,
    map_yaml_text,
    occupancy_grid_to_pgm,
)


# --- report ---


def test_complete_report_says_complete_in_a_field_not_prose():
    r = build_report(OUTCOME_COMPLETE, covered_cells=1528, resolution=0.05,
                     duration_s=35.2, goals_sent=24, goals_succeeded=24)
    assert r["outcome"] == OUTCOME_COMPLETE
    assert r["complete"] is True
    assert r["covered_cells"] == 1528
    assert r["covered_area_m2"] == pytest.approx(3.82, abs=1e-3)
    assert json.loads(json.dumps(r)) == r  # serializable as-is


def test_out_of_targets_is_not_reported_as_complete():
    """The 2026-08-07 failure: a run that ran out of reachable targets was announced
    as COMPLETE. The two must never collapse into one outcome."""
    r = build_report(OUTCOME_NO_PLANNABLE_TARGETS, covered_cells=1066,
                     resolution=0.05, duration_s=240.0, remaining_candidates=12)
    assert r["complete"] is False
    assert r["remaining_candidates"] == 12


def test_start_blocked_is_its_own_outcome():
    r = build_report(OUTCOME_START_BLOCKED, covered_cells=0, resolution=0.05,
                     duration_s=4.0)
    assert r["complete"] is False
    assert r["outcome"] == OUTCOME_START_BLOCKED


def test_unknown_outcome_is_refused():
    with pytest.raises(ValueError):
        build_report("done", covered_cells=1, resolution=0.05, duration_s=1.0)


def test_goal_counts_and_map_files_round_trip():
    r = build_report(OUTCOME_COMPLETE, covered_cells=10, resolution=0.05,
                     duration_s=1.0, goals_sent=5, goals_succeeded=4,
                     goals_aborted=1, planner_rejections=7,
                     map_files=["/tmp/m.pgm", "/tmp/m.yaml"])
    assert r["goals"] == {"sent": 5, "succeeded": 4, "aborted": 1,
                          "planner_rejections": 7}
    assert r["map_files"] == ["/tmp/m.pgm", "/tmp/m.yaml"]


# --- map serialization ---


def test_pgm_header_and_size():
    pgm = occupancy_grid_to_pgm([0] * 6, width=3, height=2)
    assert pgm.startswith(b"P5\n3 2\n255\n")
    assert len(pgm) == len(b"P5\n3 2\n255\n") + 6


def test_pgm_maps_free_occupied_unknown():
    pgm = occupancy_grid_to_pgm([0, 100, -1], width=3, height=1)
    body = pgm.split(b"255\n", 1)[1]
    assert body == bytes([PGM_FREE, PGM_OCCUPIED, PGM_UNKNOWN])


def test_pgm_flips_rows_because_pgm_starts_at_the_top():
    # Row 0 of an OccupancyGrid is the BOTTOM. A PGM's first row is the top, so the
    # occupied cell in grid row 0 must appear in the LAST pgm row.
    grid = [100, 100] + [0, 0]     # row 0 occupied, row 1 free
    body = occupancy_grid_to_pgm(grid, width=2, height=2).split(b"255\n", 1)[1]
    assert body[:2] == bytes([PGM_FREE, PGM_FREE])          # top row = grid row 1
    assert body[2:] == bytes([PGM_OCCUPIED, PGM_OCCUPIED])  # bottom row = grid row 0


def test_pgm_midrange_costs_are_unknown_not_free():
    # A cell that is neither confidently free nor confidently occupied must not be
    # written as free -- that would invent open floor in the saved map.
    body = occupancy_grid_to_pgm([50], width=1, height=1).split(b"255\n", 1)[1]
    assert body == bytes([PGM_UNKNOWN])


def test_yaml_names_the_image_and_locates_it():
    y = map_yaml_text("mission.pgm", 0.05, -1.2, 3.4)
    assert "image: mission.pgm" in y
    assert "resolution: 0.05" in y
    assert "origin: [-1.2, 3.4, 0.0]" in y
