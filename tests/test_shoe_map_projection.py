from __future__ import annotations

import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from sphero_rvr_driver.shoe_detector import BoundingBox, ShoeDetection
from sphero_rvr_driver.shoe_map_projection import (
    CameraMount,
    EvidenceReference,
    Pose2D,
    PoseHistory,
    ProjectionLimits,
    ProjectedObservation,
    ShoeObservationTracker,
    detections_from_evaluation_report,
    main,
    project_detection_to_map,
    write_observation_report,
)


def _camera_info() -> SimpleNamespace:
    return SimpleNamespace(
        width=800,
        height=600,
        k=[500.0, 0.0, 400.0, 0.0, 500.0, 300.0, 0.0, 0.0, 1.0],
        d=[0.0, 0.0, 0.0, 0.0, 0.0],
        distortion_model="plumb_bob",
    )


def test_projection_rejects_empty_intrinsics_loudly() -> None:
    empty_info = SimpleNamespace(width=0, height=0, k=[0.0] * 9, d=[], distortion_model="")
    detection = ShoeDetection("shoe", 0.8, BoundingBox(380, 520, 40, 30), "accepted")

    with pytest.raises(ValueError, match="camera intrinsics are unconfigured"):
        project_detection_to_map(
            detection,
            camera_info=empty_info,
            camera_mount=CameraMount.measured_default(),
            pose=Pose2D(timestamp_ns=10, x=1.0, y=2.0, yaw=0.0),
            evidence=EvidenceReference(frame_id="frame", path="frame_evidence.png", timestamp_ns=10),
        )


def test_project_detection_bottom_center_to_map_ground_plane() -> None:
    detection = ShoeDetection("shoe", 0.82, BoundingBox(380, 520, 40, 30), "accepted", ("accepted by detector",))

    observation = project_detection_to_map(
        detection,
        camera_info=_camera_info(),
        camera_mount=CameraMount.measured_default(),
        pose=Pose2D(timestamp_ns=1784305462310414735, x=1.0, y=2.0, yaw=0.5),
        evidence=EvidenceReference(
            frame_id="bag_frame_0000_1784305462310414735",
            path="artifacts/vs04/evidence.png",
            timestamp_ns=1784305462310414735,
        ),
    )

    assert observation.frame == "map"
    assert observation.label == "shoe"
    assert observation.status == "accepted"
    assert observation.confidence == 0.82
    assert observation.position.x == pytest.approx(1.267, abs=0.003)
    assert observation.position.y == pytest.approx(2.112, abs=0.003)
    assert observation.evidence.frame_id == "bag_frame_0000_1784305462310414735"
    assert any("ground-plane assumption" in limit for limit in observation.uncertainty_limits)
    assert any("calibration error" in limit for limit in observation.uncertainty_limits)


def test_pose_history_interpolates_and_rejects_timestamp_boundaries() -> None:
    history = PoseHistory(
        (
            Pose2D(timestamp_ns=100, x=1.0, y=2.0, yaw=0.0),
            Pose2D(timestamp_ns=200, x=3.0, y=2.0, yaw=1.0),
        )
    )

    assert history.lookup(150) == Pose2D(timestamp_ns=150, x=2.0, y=2.0, yaw=0.5)
    assert history.lookup(100) == Pose2D(timestamp_ns=100, x=1.0, y=2.0, yaw=0.0)

    with pytest.raises(ValueError, match="before first pose"):
        history.lookup(99)
    with pytest.raises(ValueError, match="after last pose"):
        history.lookup(201)


def test_tracker_spatially_deduplicates_and_propagates_confidence() -> None:
    first = ProjectedObservation(
        observation_id="",
        track_id="",
        label="shoe",
        status="review",
        confidence=0.55,
        frame="map",
        position=Pose2D(timestamp_ns=100, x=1.0, y=2.0, yaw=0.0),
        evidence=EvidenceReference(frame_id="f1", path="f1.png", timestamp_ns=100),
    )
    repeat = ProjectedObservation(
        observation_id="",
        track_id="",
        label="shoe",
        status="accepted",
        confidence=0.83,
        frame="map",
        position=Pose2D(timestamp_ns=200, x=1.12, y=2.05, yaw=0.0),
        evidence=EvidenceReference(frame_id="f2", path="f2.png", timestamp_ns=200),
    )
    far = ProjectedObservation(
        observation_id="",
        track_id="",
        label="shoe",
        status="accepted",
        confidence=0.7,
        frame="map",
        position=Pose2D(timestamp_ns=300, x=2.5, y=2.0, yaw=0.0),
        evidence=EvidenceReference(frame_id="f3", path="f3.png", timestamp_ns=300),
    )

    tracker = ShoeObservationTracker(dedup_radius_m=0.25)
    tracked = tracker.add_many((first, repeat, far))
    report = tracker.to_report()

    assert tracked[0].track_id == tracked[1].track_id
    assert tracked[2].track_id != tracked[0].track_id
    assert report["observation_count"] == 3
    assert report["track_count"] == 2
    assert report["tracks"][0]["confidence"] == 0.83
    assert report["tracks"][0]["observation_count"] == 2
    assert report["uncertainty_limits"] == list(ProjectionLimits.DEFAULT)


def test_report_loader_projects_vs04_schema_and_writes_structured_json(tmp_path: Path) -> None:
    report_path = tmp_path / "shoe_detector_evaluation.json"
    report_path.write_text(
        json.dumps(
            {
                "detector": "floor_dark_blob_shoe_baseline_v1",
                "frames": [
                    {
                        "frame_id": "bag_frame_0000_1784305462310414735",
                        "source": "/tmp/frame.ppm",
                        "width": 800,
                        "height": 600,
                        "detections": [
                            {
                                "label": "shoe",
                                "confidence": 0.72,
                                "status": "accepted",
                                "bbox": {"x": 380, "y": 520, "width": 40, "height": 30},
                                "reasons": [],
                            },
                            {
                                "label": "shoe",
                                "confidence": 0.2,
                                "status": "rejected",
                                "bbox": {"x": 20, "y": 20, "width": 10, "height": 10},
                                "reasons": ["too small"],
                            },
                        ],
                    }
                ],
            }
        )
        + "\n"
    )

    frame_detections = detections_from_evaluation_report(report_path, evidence_dir=Path("evidence_frames"))
    assert len(frame_detections) == 1
    assert frame_detections[0].timestamp_ns == 1784305462310414735
    assert frame_detections[0].evidence.path == "evidence_frames/bag_frame_0000_1784305462310414735_evidence.png"
    assert [d.status for d in frame_detections[0].detections] == ["accepted"]

    history = PoseHistory((Pose2D(timestamp_ns=1784305462310414735, x=1.0, y=2.0, yaw=0.5),))
    tracker = ShoeObservationTracker(dedup_radius_m=0.25)
    for frame in frame_detections:
        pose = history.lookup(frame.timestamp_ns)
        tracker.add_many(
            project_detection_to_map(
                detection,
                camera_info=_camera_info(),
                camera_mount=CameraMount.measured_default(),
                pose=pose,
                evidence=frame.evidence,
            )
            for detection in frame.detections
        )

    output_path = write_observation_report(tracker.to_report(), tmp_path / "shoe_map_observations.json")
    written = json.loads(output_path.read_text())

    assert written["frame"] == "map"
    assert written["source_schema"] == "vs04_shoe_detector_evaluation"
    assert written["track_count"] == 1
    assert written["tracks"][0]["evidence"][0]["frame_id"] == "bag_frame_0000_1784305462310414735"


def test_cli_projects_evaluation_report_with_explicit_camera_info(tmp_path: Path, capsys) -> None:
    report_path = tmp_path / "shoe_detector_evaluation.json"
    report_path.write_text(
        json.dumps(
            {
                "frames": [
                    {
                        "frame_id": "bag_frame_0000_1784305462310414735",
                        "detections": [
                            {
                                "label": "shoe",
                                "confidence": 0.72,
                                "status": "accepted",
                                "bbox": [380, 520, 40, 30],
                            }
                        ],
                    }
                ]
            }
        )
        + "\n"
    )
    camera_info_path = tmp_path / "camera_info.json"
    camera_info_path.write_text(
        json.dumps({"width": 800, "height": 600, "k": _camera_info().k, "distortion_model": "plumb_bob"})
        + "\n"
    )
    output_path = tmp_path / "shoe_map_observations.json"

    rc = main(
        [
            str(report_path),
            "--camera-info-json",
            str(camera_info_path),
            "--pose",
            "1784305462310414735,1.0,2.0,0.5",
            "--evidence-dir",
            str(tmp_path / "evidence_frames"),
            "--output",
            str(output_path),
        ]
    )

    assert rc == 0
    assert str(output_path) in capsys.readouterr().out
    written = json.loads(output_path.read_text())
    assert written["track_count"] == 1
    assert written["observations"][0]["frame"] == "map"
