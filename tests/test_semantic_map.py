"""Tests for the semantic object memory (Stage C)."""

import pytest

from sphero_rvr_core.semantic_map import SemanticMap, SemanticObject


def test_first_observation_creates_object():
    m = SemanticMap()
    o = m.observe("chair", 1.0, 2.0, confidence=0.8, stamp=10.0)
    assert len(m) == 1 and o.label == "chair" and o.count == 1


def test_nearby_same_label_merges_not_duplicates():
    m = SemanticMap(merge_radius_m=0.6)
    m.observe("chair", 1.0, 2.0, 0.5, 1.0)
    m.observe("chair", 1.2, 2.1, 0.5, 2.0)
    assert len(m) == 1
    assert m.objects()[0].count == 2


def test_far_same_label_is_a_separate_object():
    m = SemanticMap(merge_radius_m=0.6)
    m.observe("chair", 0.0, 0.0, 0.5, 1.0)
    m.observe("chair", 3.0, 0.0, 0.5, 2.0)
    assert len(m) == 2


def test_different_labels_never_merge():
    m = SemanticMap(merge_radius_m=1.0)
    m.observe("chair", 0.0, 0.0, 0.5, 1.0)
    m.observe("table", 0.1, 0.0, 0.5, 2.0)
    assert len(m) == 2


def test_position_is_confidence_weighted_mean():
    m = SemanticMap(merge_radius_m=2.0)  # wide enough that the two sightings merge
    m.observe("mug", 0.0, 0.0, confidence=0.1, stamp=1.0)
    m.observe("mug", 1.0, 0.0, confidence=0.9, stamp=2.0)
    assert len(m) == 1
    # the confident sighting dominates the merged position
    assert m.objects()[0].x == pytest.approx(0.9, abs=0.02)


def test_repeat_sightings_raise_confidence_but_not_to_certainty():
    m = SemanticMap()
    for i in range(20):
        m.observe("door", 0.0, 0.0, 0.5, float(i))
    o = m.objects()[0]
    assert o.confidence > 0.5 and o.confidence <= 0.99


def test_low_confidence_observation_is_rejected():
    m = SemanticMap(min_confidence=0.4)
    assert m.observe("ghost", 0.0, 0.0, confidence=0.1) is None
    assert len(m) == 0


def test_query_by_label_substring_case_insensitive():
    m = SemanticMap()
    m.observe("office chair", 1.0, 0.0, 0.8, 1.0)
    m.observe("table", 2.0, 0.0, 0.8, 1.0)
    assert [o.label for o in m.query(label="CHAIR")] == ["office chair"]


def test_query_near_sorts_by_distance():
    m = SemanticMap()
    m.observe("box", 5.0, 0.0, 0.8, 1.0)
    m.observe("box", 1.0, 0.0, 0.8, 1.0)
    near = m.query(near=(0.0, 0.0), radius_m=10.0)
    assert near[0].x == pytest.approx(1.0)


def test_query_radius_excludes_far_objects():
    m = SemanticMap()
    m.observe("box", 9.0, 0.0, 0.8, 1.0)
    assert m.query(near=(0.0, 0.0), radius_m=1.0) == []


def test_query_min_count_filters_one_off_sightings():
    m = SemanticMap()
    m.observe("real", 0.0, 0.0, 0.5, 1.0)
    m.observe("real", 0.05, 0.0, 0.5, 2.0)
    m.observe("fluke", 5.0, 0.0, 0.5, 1.0)
    labels = [o.label for o in m.query(min_count=2)]
    assert labels == ["real"]


def test_forget_stale_drops_old_objects():
    m = SemanticMap()
    m.observe("old", 0.0, 0.0, 0.5, stamp=1.0)
    m.observe("new", 3.0, 0.0, 0.5, stamp=100.0)
    assert m.forget_stale(50.0) == 1
    assert [o.label for o in m.objects()] == ["new"]


def test_json_round_trip_preserves_objects():
    m = SemanticMap()
    m.observe("chair", 1.0, 2.0, 0.7, 5.0)
    m.observe("chair", 1.1, 2.0, 0.7, 6.0)
    m.observe("lamp", 4.0, 1.0, 0.6, 7.0)
    back = SemanticMap.from_json(m.to_json())
    assert len(back) == 2
    labels = sorted(o.label for o in back.objects())
    assert labels == ["chair", "lamp"]
    chair = back.query(label="chair")[0]
    assert chair.count == 2 and chair.x == pytest.approx(1.05, abs=0.02)


def test_reload_then_observe_still_merges():
    m = SemanticMap(merge_radius_m=0.6)
    m.observe("chair", 1.0, 2.0, 0.7, 5.0)
    back = SemanticMap.from_json(m.to_json(), merge_radius_m=0.6)
    back.observe("chair", 1.1, 2.05, 0.7, 8.0)
    assert len(back) == 1 and back.objects()[0].count == 2


def test_object_dict_round_trip():
    o = SemanticObject("cup", 1.0, 2.0, 0.9, 3.0)
    back = SemanticObject.from_dict(o.to_dict())
    assert back.label == "cup" and back.x == pytest.approx(1.0)
