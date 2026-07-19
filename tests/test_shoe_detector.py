from __future__ import annotations

import json
from pathlib import Path

import pytest

from sphero_rvr_driver.shoe_detector import (
    BoundingBox,
    DetectorThresholds,
    ShoeDetection,
    classify_confidence,
    detect_shoes_in_rgb,
    evaluate_ppm_image,
    main,
    parse_detection_record,
    parse_thresholds,
    read_ppm,
    write_ppm,
)


def _blank(width: int, height: int, color: tuple[int, int, int] = (190, 190, 190)) -> bytearray:
    return bytearray(color * (width * height))


def _fill_rect(pixels: bytearray, width: int, box: BoundingBox, color: tuple[int, int, int]) -> None:
    for y in range(box.y, box.y + box.height):
        for x in range(box.x, box.x + box.width):
            idx = (y * width + x) * 3
            pixels[idx:idx + 3] = bytes(color)


def test_thresholds_require_review_not_above_accept() -> None:
    assert parse_thresholds(0.8, 0.4) == DetectorThresholds(accept=0.8, review=0.4)
    assert classify_confidence(0.8, DetectorThresholds(accept=0.75, review=0.5)) == "accepted"
    assert classify_confidence(0.6, DetectorThresholds(accept=0.75, review=0.5)) == "review"
    assert classify_confidence(0.2, DetectorThresholds(accept=0.75, review=0.5)) == "rejected"

    with pytest.raises(ValueError, match="review threshold must be <= accept threshold"):
        parse_thresholds(0.4, 0.5)
    with pytest.raises(ValueError, match="accept threshold"):
        parse_thresholds(1.1, 0.5)


def test_detection_record_schema_parses_bbox_dict_and_reclassifies_status() -> None:
    detection = parse_detection_record(
        {
            "label": "shoe",
            "confidence": 0.72,
            "bbox": {"x": 10, "y": 20, "width": 30, "height": 12},
            "status": "review",
            "reasons": ["sample"],
        },
        DetectorThresholds(accept=0.7, review=0.5),
    )

    assert detection == ShoeDetection(
        "shoe",
        0.72,
        BoundingBox(10, 20, 30, 12),
        "accepted",
        ("sample",),
    )
    assert detection.to_json_dict()["bbox"] == {"x": 10, "y": 20, "width": 30, "height": 12}


def test_detection_record_schema_rejects_invalid_confidence_and_status() -> None:
    with pytest.raises(ValueError, match="confidence"):
        parse_detection_record({"confidence": 1.2, "bbox": [0, 0, 10, 10], "status": "review"})
    with pytest.raises(ValueError, match="status"):
        parse_detection_record({"confidence": 0.5, "bbox": [0, 0, 10, 10], "status": "maybe"})


def test_ppm_roundtrip_and_detector_accepts_floor_adjacent_dark_blob(tmp_path: Path) -> None:
    width, height = 160, 120
    pixels = _blank(width, height)
    _fill_rect(pixels, width, BoundingBox(45, 78, 58, 22), (35, 35, 32))
    image = write_ppm(tmp_path / "shoe.ppm", width, height, bytes(pixels))

    read_width, read_height, read_pixels = read_ppm(image)
    detections = detect_shoes_in_rgb(read_width, read_height, read_pixels, DetectorThresholds(accept=0.55, review=0.35))

    assert (read_width, read_height) == (width, height)
    assert detections
    assert detections[0].status == "accepted"
    assert detections[0].bbox.x <= 48
    assert detections[0].bbox.y <= 80


def test_detector_marks_square_checker_candidate_below_acceptance_threshold() -> None:
    width, height = 160, 120
    pixels = _blank(width, height, (210, 210, 210))
    _fill_rect(pixels, width, BoundingBox(70, 65, 24, 24), (20, 20, 20))

    detections = detect_shoes_in_rgb(width, height, bytes(pixels), DetectorThresholds(accept=0.7, review=0.35))

    assert detections
    assert detections[0].status != "accepted"
    assert any("not elongated" in reason for reason in detections[0].reasons)


def test_evaluation_cli_writes_structured_results_and_evidence_frames(tmp_path: Path, capsys) -> None:
    width, height = 96, 72
    pixels = _blank(width, height)
    image = write_ppm(tmp_path / "empty.ppm", width, height, bytes(pixels))
    output_dir = tmp_path / "out"

    rc = main(["--image", str(image), "--output-dir", str(output_dir)])

    assert rc == 0
    output = capsys.readouterr().out
    assert "shoe_detector_evaluation.json" in output
    report = json.loads((output_dir / "shoe_detector_evaluation.json").read_text())
    assert report["detector"] == "floor_dark_blob_shoe_baseline_v1"
    assert report["frame_count"] == 1
    assert report["accepted_count"] == 0
    assert "coverage_statement" in report
    assert (output_dir / "evidence_frames" / "empty_evidence.ppm").is_file()


def test_evaluate_ppm_image_exposes_frame_schema(tmp_path: Path) -> None:
    image = write_ppm(tmp_path / "frame_001.ppm", 8, 8, bytes(_blank(8, 8)))

    frame = evaluate_ppm_image(image)

    assert frame.to_json_dict() == {
        "frame_id": "frame_001",
        "source": str(image),
        "width": 8,
        "height": 8,
        "detections": [],
        "accepted_count": 0,
    }
