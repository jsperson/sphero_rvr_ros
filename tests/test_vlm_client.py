"""Tests for the VLM reply parsing + exploration geometry (pure parts)."""

import math

import pytest

from sphero_rvr_core.vlm_client import direction_to_goal, extract_json


def test_extract_plain_json():
    assert extract_json('{"turn_deg": 30, "go": true}') == {"turn_deg": 30, "go": True}


def test_extract_json_embedded_in_prose():
    text = 'Sure! I think you should go here: {"turn_deg": -20, "go": true, "reason": "doorway left"}. Good luck.'
    assert extract_json(text)["turn_deg"] == -20


def test_extract_json_in_code_fence():
    text = "```json\n{\"turn_deg\": 0, \"go\": false}\n```"
    assert extract_json(text) == {"turn_deg": 0, "go": False}


def test_extract_json_none_raises():
    with pytest.raises(ValueError):
        extract_json("I cannot see any openings, sorry.")


def test_direction_to_goal_straight():
    gx, gy, gyaw = direction_to_goal(0.0, 0.0, 0.0, turn_deg=0, distance_m=1.5)
    assert gx == pytest.approx(1.5) and gy == pytest.approx(0.0)


def test_direction_to_goal_right_is_negative_y():
    gx, gy, _ = direction_to_goal(0.0, 0.0, 0.0, turn_deg=90, distance_m=1.0)
    assert gx == pytest.approx(0.0, abs=1e-9) and gy == pytest.approx(-1.0)


def test_direction_to_goal_left_is_positive_y():
    gx, gy, _ = direction_to_goal(0.0, 0.0, 0.0, turn_deg=-90, distance_m=1.0)
    assert gy == pytest.approx(1.0)


def test_direction_to_goal_respects_robot_heading():
    # Robot facing +y (yaw 90deg), go straight -> point at (0, d).
    gx, gy, _ = direction_to_goal(0.0, 0.0, math.pi / 2, turn_deg=0, distance_m=2.0)
    assert gx == pytest.approx(0.0, abs=1e-9) and gy == pytest.approx(2.0)
