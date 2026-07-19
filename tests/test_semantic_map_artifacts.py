from __future__ import annotations

import json
from pathlib import Path

from sphero_rvr_driver.semantic_map_artifacts import (
    CoverageRegion,
    SemanticMapInputs,
    SemanticObject,
    SemanticPose,
    generate_semantic_map_artifacts,
)


def _write_fixture_map(tmp_path: Path) -> tuple[Path, Path]:
    pgm_path = tmp_path / "fixture_map.pgm"
    pgm_path.write_text(
        "P2\n"
        "# semantic artifact test map\n"
        "6 4\n"
        "255\n"
        "0 0 0 0 0 0\n"
        "0 254 254 254 205 0\n"
        "0 254 254 205 205 0\n"
        "0 0 0 0 0 0\n"
    )
    yaml_path = tmp_path / "fixture_map.yaml"
    yaml_path.write_text(
        "image: fixture_map.pgm\n"
        "mode: trinary\n"
        "resolution: 0.5\n"
        "origin: [1.0, 2.0, 0.0]\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.25\n"
    )
    return yaml_path, pgm_path


def _write_vs05_observations(tmp_path: Path) -> Path:
    observations_path = tmp_path / "shoe_map_observations.json"
    observations_path.write_text(
        json.dumps(
            {
                "source_schema": "vs04_shoe_detector_evaluation",
                "frame": "map",
                "dedup_radius_m": 0.25,
                "track_count": 1,
                "observation_count": 2,
                "tracks": [
                    {
                        "track_id": "shoe_track_0001",
                        "label": "shoe",
                        "status": "review",
                        "confidence": 0.61,
                        "frame": "map",
                        "position": {"x": 1.75, "y": 2.75, "timestamp_ns": 200},
                        "observation_count": 2,
                        "evidence": [
                            {"frame_id": "frame_a_100", "path": "evidence/a.png", "timestamp_ns": 100},
                            {"frame_id": "frame_b_200", "path": "evidence/b.png", "timestamp_ns": 200},
                        ],
                    }
                ],
                "observations": [
                    {
                        "observation_id": "shoe_obs_0001",
                        "track_id": "shoe_track_0001",
                        "label": "shoe",
                        "status": "review",
                        "confidence": 0.57,
                        "frame": "map",
                        "position": {"x": 1.70, "y": 2.75, "yaw": 0.1, "timestamp_ns": 100},
                        "evidence": {"frame_id": "frame_a_100", "path": "evidence/a.png", "timestamp_ns": 100},
                        "uncertainty_limits": ["ground-plane assumption", "occlusion", "calibration error"],
                        "detector_reasons": ["review-level detector confidence"],
                    },
                    {
                        "observation_id": "shoe_obs_0002",
                        "track_id": "shoe_track_0001",
                        "label": "shoe",
                        "status": "review",
                        "confidence": 0.61,
                        "frame": "map",
                        "position": {"x": 1.80, "y": 2.76, "yaw": 0.1, "timestamp_ns": 200},
                        "evidence": {"frame_id": "frame_b_200", "path": "evidence/b.png", "timestamp_ns": 200},
                        "uncertainty_limits": ["ground-plane assumption", "pose drift", "calibration error"],
                        "detector_reasons": ["review-level detector confidence"],
                    },
                ],
                "uncertainty_limits": ["ground-plane assumption", "occlusion", "pose drift", "calibration error"],
            },
            indent=2,
        )
        + "\n"
    )
    return observations_path


def _write_vs04_detector_report(tmp_path: Path) -> Path:
    detector_path = tmp_path / "shoe_detector_evaluation.json"
    detector_path.write_text(
        json.dumps(
            {
                "detector": "floor_dark_blob_shoe_baseline_v1",
                "accepted_count": 0,
                "candidate_count": 12,
                "coverage_statement": "Replay evaluation is limited to sampled frames; do not infer every-shoe recall.",
            }
        )
        + "\n"
    )
    return detector_path


def test_semantic_object_serialization_preserves_sources_pose_evidence_and_uncertainty() -> None:
    obj = SemanticObject(
        object_id="shoe_track_0001",
        object_class="shoe",
        status="review",
        confidence=0.6123,
        detector_confidence=0.61,
        projection_confidence=0.72,
        pose=SemanticPose(x=1.25, y=2.5, yaw=0.0, frame="map", timestamp_ns=200),
        observation_count=2,
        first_observed_ns=100,
        last_observed_ns=200,
        evidence_frame_refs=("evidence/a.png", "evidence/b.png"),
        source_run_id="canonical_replay_run",
        source_map_id="fixture_map",
        uncertainty={"classification": "review", "projection": "ground-plane assumption"},
        coverage={"region_status": "observed", "coverage_confidence": 0.5},
    )

    data = obj.to_json_dict()
    feature = obj.to_geojson_feature()

    assert data["class"] == "shoe"
    assert data["map_pose"] == {"x": 1.25, "y": 2.5, "yaw": 0.0, "frame": "map", "timestamp_ns": 200}
    assert data["observation_count"] == 2
    assert data["timestamps"] == {"first_observed_ns": 100, "last_observed_ns": 200}
    assert data["evidence_frame_refs"] == ["evidence/a.png", "evidence/b.png"]
    assert data["source"] == {"run_id": "canonical_replay_run", "map_id": "fixture_map"}
    assert data["uncertainty"]["projection"] == "ground-plane assumption"
    assert data["coverage"]["region_status"] == "observed"
    assert feature["geometry"] == {"type": "Point", "coordinates": [1.25, 2.5]}
    assert feature["properties"]["object_id"] == "shoe_track_0001"


def test_generate_semantic_artifacts_json_geojson_report_and_annotated_map_are_consistent(tmp_path: Path) -> None:
    map_yaml, _ = _write_fixture_map(tmp_path)
    observation_path = _write_vs05_observations(tmp_path)
    detector_path = _write_vs04_detector_report(tmp_path)
    output_dir = tmp_path / "semantic_artifacts"

    result = generate_semantic_map_artifacts(
        SemanticMapInputs(
            map_yaml=map_yaml,
            detector_evaluation_json=detector_path,
            map_observations_json=observation_path,
            output_dir=output_dir,
            source_run_id="canonical_replay_run",
            source_map_id="fixture_map",
            coverage_regions=(
                CoverageRegion("observed", "Sampled camera/lidar field of view", 0.42),
                CoverageRegion("inaccessible", "Fixture border/walls and unmapped space", 0.0),
                CoverageRegion("occluded", "Line-of-sight blocked or unobserved behind obstacles", 0.0),
            ),
        )
    )

    semantic_json = json.loads(result.semantic_json.read_text())
    geojson = json.loads(result.geojson.read_text())
    report = result.coverage_report.read_text()
    summary = result.mission_summary.read_text()

    assert result.annotated_map_image.is_file()
    assert result.annotated_map_image.read_text().startswith("P3\n")
    assert semantic_json["schema"] == "sphero_rvr_semantic_map.v1"
    assert semantic_json["sources"]["vs02_map_yaml"] == str(map_yaml)
    assert semantic_json["sources"]["vs04_detector_evaluation_json"] == str(detector_path)
    assert semantic_json["sources"]["vs05_map_observations_json"] == str(observation_path)
    assert semantic_json["map"]["resolution"] == 0.5
    assert semantic_json["object_count"] == 1
    assert semantic_json["objects"][0]["object_id"] == "shoe_track_0001"
    assert semantic_json["objects"][0]["observation_count"] == 2
    assert semantic_json["objects"][0]["confidence"]["detector"] == 0.61
    assert semantic_json["objects"][0]["confidence"]["projection"] < 1.0
    assert semantic_json["objects"][0]["coverage"]["region_status"] == "observed"
    assert {region["status"] for region in semantic_json["coverage"]["regions"]} == {"observed", "inaccessible", "occluded"}
    assert geojson["type"] == "FeatureCollection"
    assert len(geojson["features"]) == semantic_json["object_count"]
    assert geojson["features"][0]["properties"]["evidence_frame_refs"] == ["evidence/a.png", "evidence/b.png"]
    assert "Observed coverage" in report
    assert "Inaccessible regions" in report
    assert "Occlusion" in report
    assert "uncertain detections" in report.lower()
    assert "detector confidence" in report.lower()
    assert "projection confidence" in report.lower()
    assert "does not claim every shoe" in report.lower()
    assert "every shoe was found" not in report.lower()
    assert "1 semantic shoe track" in summary


def test_semantic_generation_deduplicates_by_track_id_not_observation_count(tmp_path: Path) -> None:
    map_yaml, _ = _write_fixture_map(tmp_path)
    observation_path = _write_vs05_observations(tmp_path)
    detector_path = _write_vs04_detector_report(tmp_path)

    result = generate_semantic_map_artifacts(
        SemanticMapInputs(
            map_yaml=map_yaml,
            detector_evaluation_json=detector_path,
            map_observations_json=observation_path,
            output_dir=tmp_path / "out",
            source_run_id="run",
            source_map_id="map",
        )
    )

    semantic_json = json.loads(result.semantic_json.read_text())
    assert semantic_json["object_count"] == 1
    assert semantic_json["objects"][0]["object_id"] == "shoe_track_0001"
    assert semantic_json["objects"][0]["observation_count"] == 2
    assert semantic_json["observation_count"] == 2


def test_generate_semantic_artifacts_rejects_every_shoe_language_from_reports(tmp_path: Path) -> None:
    map_yaml, _ = _write_fixture_map(tmp_path)
    observation_path = _write_vs05_observations(tmp_path)
    detector_path = _write_vs04_detector_report(tmp_path)

    result = generate_semantic_map_artifacts(
        SemanticMapInputs(
            map_yaml=map_yaml,
            detector_evaluation_json=detector_path,
            map_observations_json=observation_path,
            output_dir=tmp_path / "out",
            source_run_id="run",
            source_map_id="map",
        )
    )

    combined = result.coverage_report.read_text() + result.mission_summary.read_text()
    assert "all shoes" not in combined.lower()
    assert "every shoe was found" not in combined.lower()
    assert "outside observed coverage" in combined.lower()


def test_out_of_extent_semantic_objects_are_still_annotated_as_clipped_markers(tmp_path: Path) -> None:
    map_yaml, _ = _write_fixture_map(tmp_path)
    observation_path = _write_vs05_observations(tmp_path)
    data = json.loads(observation_path.read_text())
    data["tracks"][0]["position"]["x"] = 99.0
    data["tracks"][0]["position"]["y"] = 99.0
    observation_path.write_text(json.dumps(data) + "\n")

    result = generate_semantic_map_artifacts(
        SemanticMapInputs(
            map_yaml=map_yaml,
            detector_evaluation_json=_write_vs04_detector_report(tmp_path),
            map_observations_json=observation_path,
            output_dir=tmp_path / "out",
            source_run_id="run",
            source_map_id="map",
        )
    )

    semantic_json = json.loads(result.semantic_json.read_text())
    ppm = result.annotated_map_image.read_text()
    assert semantic_json["objects"][0]["coverage"]["map_image_marker"] == "clamped_to_edge"
    assert "255 0 0" in ppm
