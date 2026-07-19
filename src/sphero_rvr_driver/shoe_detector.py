"""Replay-first, dependency-light shoe detector evaluation helpers.

This is intentionally a narrow baseline, not a general ML perception platform. It
looks for dark, shoe-sized, floor-adjacent blobs in camera frames and records
explicit confidence limits so downstream semantic-map fusion can consume or
reject the observations without treating them as ground truth.
"""

from __future__ import annotations

import argparse
import json
import math
import tempfile
from collections import deque
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Iterable, Optional, Sequence

DEFAULT_ACCEPT_THRESHOLD = 0.70
DEFAULT_REVIEW_THRESHOLD = 0.45
DEFAULT_SAMPLE_COUNT = 5
DEFAULT_IMAGE_TOPIC = "/camera_node/image_raw"
KNOWN_FAILURE_MODES = (
    "no positive shoe examples in the current replay bag; absence of accepted detections is not proof that the detector recognizes shoes",
    "occluded, partially visible, very bright, or motion-blurred shoes can be missed",
    "dark floor tools, cables, wheels, and checkerboard calibration targets can become false candidates below the review threshold",
    "heuristic is tuned for floor-adjacent replay frames and should not be treated as a general object detector",
)


@dataclass(frozen=True)
class DetectorThresholds:
    accept: float = DEFAULT_ACCEPT_THRESHOLD
    review: float = DEFAULT_REVIEW_THRESHOLD

    def __post_init__(self) -> None:
        if not 0.0 <= self.review <= 1.0:
            raise ValueError("review threshold must be between 0 and 1")
        if not 0.0 <= self.accept <= 1.0:
            raise ValueError("accept threshold must be between 0 and 1")
        if self.review > self.accept:
            raise ValueError("review threshold must be <= accept threshold")


@dataclass(frozen=True)
class BoundingBox:
    x: int
    y: int
    width: int
    height: int

    @property
    def area(self) -> int:
        return self.width * self.height

    def as_xyxy(self) -> tuple[int, int, int, int]:
        return (self.x, self.y, self.x + self.width - 1, self.y + self.height - 1)


@dataclass(frozen=True)
class ShoeDetection:
    label: str
    confidence: float
    bbox: BoundingBox
    status: str
    reasons: tuple[str, ...] = ()

    @classmethod
    def from_mapping(cls, value: dict[str, Any]) -> "ShoeDetection":
        bbox_value = value.get("bbox", {})
        if isinstance(bbox_value, list):
            if len(bbox_value) != 4:
                raise ValueError("bbox list must be [x, y, width, height]")
            bbox = BoundingBox(*(int(item) for item in bbox_value))
        else:
            bbox = BoundingBox(
                int(bbox_value["x"]),
                int(bbox_value["y"]),
                int(bbox_value["width"]),
                int(bbox_value["height"]),
            )
        confidence = float(value["confidence"])
        if not 0.0 <= confidence <= 1.0:
            raise ValueError("confidence must be between 0 and 1")
        status = str(value.get("status", "candidate"))
        if status not in {"accepted", "review", "rejected"}:
            raise ValueError("status must be accepted, review, or rejected")
        return cls(
            label=str(value.get("label", "shoe")),
            confidence=confidence,
            bbox=bbox,
            status=status,
            reasons=tuple(str(item) for item in value.get("reasons", ())),
        )

    def to_json_dict(self) -> dict[str, Any]:
        data = asdict(self)
        data["bbox"] = asdict(self.bbox)
        data["reasons"] = list(self.reasons)
        data["confidence"] = round(self.confidence, 3)
        return data


@dataclass(frozen=True)
class FrameEvaluation:
    frame_id: str
    width: int
    height: int
    detections: tuple[ShoeDetection, ...]
    source: str = ""

    @property
    def accepted(self) -> tuple[ShoeDetection, ...]:
        return tuple(d for d in self.detections if d.status == "accepted")

    def to_json_dict(self) -> dict[str, Any]:
        return {
            "frame_id": self.frame_id,
            "source": self.source,
            "width": self.width,
            "height": self.height,
            "detections": [d.to_json_dict() for d in self.detections],
            "accepted_count": len(self.accepted),
        }


@dataclass(frozen=True)
class EvaluationReport:
    detector: str
    thresholds: DetectorThresholds
    frames: tuple[FrameEvaluation, ...]
    input_summary: dict[str, Any] = field(default_factory=dict)
    known_failure_modes: tuple[str, ...] = KNOWN_FAILURE_MODES

    def to_json_dict(self) -> dict[str, Any]:
        total_candidates = sum(len(frame.detections) for frame in self.frames)
        accepted = sum(len(frame.accepted) for frame in self.frames)
        review = sum(1 for frame in self.frames for detection in frame.detections if detection.status == "review")
        rejected = sum(1 for frame in self.frames for detection in frame.detections if detection.status == "rejected")
        return {
            "detector": self.detector,
            "thresholds": asdict(self.thresholds),
            "input_summary": self.input_summary,
            "frame_count": len(self.frames),
            "candidate_count": total_candidates,
            "accepted_count": accepted,
            "review_count": review,
            "rejected_count": rejected,
            "coverage_statement": (
                "Replay evaluation is limited to the sampled/available frames; do not infer every-shoe recall. "
                "This primary replay bag currently provides negative/no-positive coverage for shoes."
            ),
            "known_failure_modes": list(self.known_failure_modes),
            "frames": [frame.to_json_dict() for frame in self.frames],
        }


def classify_confidence(confidence: float, thresholds: DetectorThresholds) -> str:
    if confidence >= thresholds.accept:
        return "accepted"
    if confidence >= thresholds.review:
        return "review"
    return "rejected"


def parse_thresholds(accept: float = DEFAULT_ACCEPT_THRESHOLD, review: float = DEFAULT_REVIEW_THRESHOLD) -> DetectorThresholds:
    return DetectorThresholds(accept=float(accept), review=float(review))


def parse_detection_record(value: dict[str, Any], thresholds: Optional[DetectorThresholds] = None) -> ShoeDetection:
    detection = ShoeDetection.from_mapping(value)
    if thresholds is None:
        return detection
    status = classify_confidence(detection.confidence, thresholds)
    return ShoeDetection(detection.label, detection.confidence, detection.bbox, status, detection.reasons)


def _read_token(data: bytes, offset: int) -> tuple[bytes, int]:
    while offset < len(data) and data[offset] in b" \t\r\n":
        offset += 1
    if offset < len(data) and data[offset] == ord("#"):
        while offset < len(data) and data[offset] not in b"\r\n":
            offset += 1
        return _read_token(data, offset)
    start = offset
    while offset < len(data) and data[offset] not in b" \t\r\n":
        offset += 1
    return data[start:offset], offset


def read_ppm(path: Path) -> tuple[int, int, bytes]:
    data = path.read_bytes()
    magic, offset = _read_token(data, 0)
    if magic != b"P6":
        raise ValueError(f"{path}: expected binary PPM P6 image")
    width_token, offset = _read_token(data, offset)
    height_token, offset = _read_token(data, offset)
    max_token, offset = _read_token(data, offset)
    width = int(width_token)
    height = int(height_token)
    if int(max_token) != 255:
        raise ValueError(f"{path}: only max value 255 is supported")
    while offset < len(data) and data[offset] in b" \t\r\n":
        offset += 1
    pixels = data[offset:]
    expected = width * height * 3
    if len(pixels) != expected:
        raise ValueError(f"{path}: expected {expected} RGB bytes, found {len(pixels)}")
    return width, height, pixels


def write_ppm(path: Path, width: int, height: int, pixels: bytes) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(f"P6\n{width} {height}\n255\n".encode() + pixels)
    return path


def _candidate_mask(width: int, height: int, pixels: bytes, *, stride: int) -> tuple[set[tuple[int, int]], int, int]:
    grid_w = math.ceil(width / stride)
    grid_h = math.ceil(height / stride)
    min_y = int(height * 0.45)
    cells: set[tuple[int, int]] = set()
    for gy in range(grid_h):
        y0 = gy * stride
        if y0 < min_y:
            continue
        for gx in range(grid_w):
            x0 = gx * stride
            dark = 0
            total = 0
            for y in range(y0, min(y0 + stride, height)):
                row = y * width * 3
                for x in range(x0, min(x0 + stride, width)):
                    idx = row + x * 3
                    r, g, b = pixels[idx], pixels[idx + 1], pixels[idx + 2]
                    luma = 0.2126 * r + 0.7152 * g + 0.0722 * b
                    chroma = max(r, g, b) - min(r, g, b)
                    if luma < 72 and chroma < 90:
                        dark += 1
                    total += 1
            if total and dark / total >= 0.42:
                cells.add((gx, gy))
    return cells, grid_w, grid_h


def _components(cells: set[tuple[int, int]]) -> Iterable[set[tuple[int, int]]]:
    remaining = set(cells)
    while remaining:
        start = remaining.pop()
        component = {start}
        queue: deque[tuple[int, int]] = deque([start])
        while queue:
            gx, gy = queue.popleft()
            for nx, ny in ((gx + 1, gy), (gx - 1, gy), (gx, gy + 1), (gx, gy - 1)):
                if (nx, ny) in remaining:
                    remaining.remove((nx, ny))
                    component.add((nx, ny))
                    queue.append((nx, ny))
        yield component


def _score_component(box: BoundingBox, width: int, height: int, cell_count: int, stride: int) -> tuple[float, tuple[str, ...]]:
    frame_area = width * height
    area_ratio = box.area / frame_area
    aspect = box.width / max(box.height, 1)
    center_y = (box.y + box.height / 2) / height
    reasons: list[str] = []

    too_small = box.area < frame_area * 0.003
    too_large = area_ratio > 0.18
    not_elongated = aspect < 1.15
    too_thin = aspect > 7.0
    if too_small:
        reasons.append("too small for a reliable shoe candidate")
    if too_large:
        reasons.append("too large for a single shoe candidate")
    if not_elongated:
        reasons.append("not elongated enough for the baseline shoe shape prior")
    if too_thin:
        reasons.append("too thin/elongated; likely cable, rod, or edge artifact")
    if center_y < 0.52:
        reasons.append("not floor-adjacent enough for this replay heuristic")

    area_score = min(area_ratio / 0.05, 1.0)
    aspect_score = max(0.0, 1.0 - abs(aspect - 2.4) / 2.4)
    floor_score = min(max((center_y - 0.45) / 0.45, 0.0), 1.0)
    fill_score = min((cell_count * stride * stride) / max(box.area, 1), 1.0)
    confidence = 0.18 + 0.30 * area_score + 0.26 * aspect_score + 0.18 * floor_score + 0.08 * fill_score
    if too_large or too_thin:
        confidence = min(confidence, 0.40)
    elif too_small or not_elongated:
        confidence = min(confidence, 0.44)
    elif reasons:
        confidence = min(confidence, 0.58)
    return max(0.0, min(confidence, 0.95)), tuple(reasons)


def detect_shoes_in_rgb(
    width: int,
    height: int,
    pixels: bytes,
    thresholds: DetectorThresholds = DetectorThresholds(),
    *,
    stride: int = 8,
) -> tuple[ShoeDetection, ...]:
    if len(pixels) != width * height * 3:
        raise ValueError("pixels must be tightly packed RGB bytes")
    cells, _grid_w, _grid_h = _candidate_mask(width, height, pixels, stride=stride)
    detections: list[ShoeDetection] = []
    for component in _components(cells):
        if len(component) < 3:
            continue
        xs = [gx for gx, _gy in component]
        ys = [gy for _gx, gy in component]
        x0 = min(xs) * stride
        y0 = min(ys) * stride
        x1 = min(width, (max(xs) + 1) * stride)
        y1 = min(height, (max(ys) + 1) * stride)
        box = BoundingBox(x0, y0, x1 - x0, y1 - y0)
        confidence, reasons = _score_component(box, width, height, len(component), stride)
        status = classify_confidence(confidence, thresholds)
        if status == "rejected" and confidence < thresholds.review * 0.75:
            continue
        detections.append(ShoeDetection("shoe", confidence, box, status, reasons))
    detections.sort(key=lambda d: d.confidence, reverse=True)
    return tuple(detections[:12])


def evaluate_ppm_image(path: Path, thresholds: DetectorThresholds = DetectorThresholds()) -> FrameEvaluation:
    width, height, pixels = read_ppm(path)
    return FrameEvaluation(path.stem, width, height, detect_shoes_in_rgb(width, height, pixels, thresholds), source=str(path))


def evaluate_ppm_images(paths: Sequence[Path], thresholds: DetectorThresholds = DetectorThresholds()) -> EvaluationReport:
    frames = tuple(evaluate_ppm_image(path, thresholds) for path in paths)
    return EvaluationReport(
        detector="floor_dark_blob_shoe_baseline_v1",
        thresholds=thresholds,
        frames=frames,
        input_summary={"mode": "ppm", "images": [str(path) for path in paths]},
    )


def _draw_rect(pixels: bytearray, width: int, height: int, box: BoundingBox, color: tuple[int, int, int]) -> None:
    x0, y0, x1, y1 = box.as_xyxy()
    x0 = max(0, min(width - 1, x0)); x1 = max(0, min(width - 1, x1))
    y0 = max(0, min(height - 1, y0)); y1 = max(0, min(height - 1, y1))
    for x in range(x0, x1 + 1):
        for y in (y0, y1):
            idx = (y * width + x) * 3
            pixels[idx:idx + 3] = bytes(color)
    for y in range(y0, y1 + 1):
        for x in (x0, x1):
            idx = (y * width + x) * 3
            pixels[idx:idx + 3] = bytes(color)


def write_evidence_frame(source: Path, evaluation: FrameEvaluation, output_path: Path) -> Path:
    width, height, pixels = read_ppm(source)
    annotated = bytearray(pixels)
    for detection in evaluation.detections:
        if detection.status == "rejected":
            continue
        color = (255, 0, 0) if detection.status == "accepted" else (255, 191, 0)
        _draw_rect(annotated, width, height, detection.bbox, color)
    return write_ppm(output_path, width, height, bytes(annotated))


def write_report(report: EvaluationReport, output_path: Path) -> Path:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(report.to_json_dict(), indent=2, sort_keys=True) + "\n")
    return output_path


def _extract_bag_samples(bag_path: Path, output_dir: Path, topic: str, sample_count: int) -> list[Path]:
    try:
        from rclpy.serialization import deserialize_message  # type: ignore[import-not-found]
        from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions  # type: ignore[import-not-found]
        from sensor_msgs.msg import Image  # type: ignore[import-not-found]
    except Exception as exc:  # pragma: no cover - exercised on ROS hosts
        raise RuntimeError("ROS bag image extraction requires rosbag2_py, rclpy, and sensor_msgs") from exc

    output_dir.mkdir(parents=True, exist_ok=True)
    reader = SequentialReader()
    reader.open(StorageOptions(uri=str(bag_path), storage_id="mcap"), ConverterOptions("", ""))
    messages: list[tuple[int, Any]] = []
    while reader.has_next():
        msg_topic, data, timestamp = reader.read_next()
        if msg_topic == topic:
            messages.append((timestamp, deserialize_message(data, Image)))
    if not messages:
        raise RuntimeError(f"no {topic} images found in {bag_path}")
    if sample_count >= len(messages):
        indices = list(range(len(messages)))
    else:
        indices = sorted({round(i * (len(messages) - 1) / max(sample_count - 1, 1)) for i in range(sample_count)})
    paths: list[Path] = []
    for index in indices:
        timestamp, msg = messages[index]
        encoding = msg.encoding.lower()
        if encoding not in {"rgb8", "bgr8", "8uc3"}:
            raise RuntimeError(f"unsupported image encoding {msg.encoding!r}")
        raw = bytes(msg.data)
        pixels = bytearray()
        for y in range(msg.height):
            row = raw[y * msg.step:y * msg.step + msg.width * 3]
            if encoding == "bgr8":
                for x in range(0, len(row), 3):
                    pixels.extend((row[x + 2], row[x + 1], row[x]))
            else:
                pixels.extend(row)
        path = output_dir / f"bag_frame_{index:04d}_{timestamp}.ppm"
        write_ppm(path, int(msg.width), int(msg.height), bytes(pixels))
        paths.append(path)
    return paths


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Evaluate replay/sample frames with the baseline shoe detector")
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--image", action="append", type=Path, help="PPM P6 frame to evaluate; repeat for multiple frames")
    source.add_argument("--image-dir", type=Path, help="Directory of .ppm frames to evaluate")
    source.add_argument("--bag", type=Path, help="ROS 2 MCAP bag directory to sample and evaluate")
    parser.add_argument("--image-topic", default=DEFAULT_IMAGE_TOPIC)
    parser.add_argument("--sample-count", type=int, default=DEFAULT_SAMPLE_COUNT)
    parser.add_argument("--accept-threshold", type=float, default=DEFAULT_ACCEPT_THRESHOLD)
    parser.add_argument("--review-threshold", type=float, default=DEFAULT_REVIEW_THRESHOLD)
    parser.add_argument("--output-dir", type=Path, default=Path("artifacts/shoe_detector_eval"))
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = build_arg_parser()
    args = parser.parse_args(argv)
    thresholds = parse_thresholds(args.accept_threshold, args.review_threshold)
    if args.sample_count <= 0:
        parser.error("--sample-count must be greater than 0")

    output_dir: Path = args.output_dir
    output_dir.mkdir(parents=True, exist_ok=True)
    if args.image:
        images = args.image
        input_summary = {"mode": "ppm", "images": [str(path) for path in images]}
    elif args.image_dir:
        images = sorted(args.image_dir.glob("*.ppm"))
        input_summary = {"mode": "ppm_dir", "image_dir": str(args.image_dir), "images": [str(path) for path in images]}
    else:
        sample_dir = output_dir / "sampled_frames"
        try:
            images = _extract_bag_samples(args.bag, sample_dir, args.image_topic, args.sample_count)
        except RuntimeError as exc:
            parser.exit(2, f"error: {exc}\n")
        input_summary = {
            "mode": "rosbag",
            "bag": str(args.bag),
            "image_topic": args.image_topic,
            "sample_count": args.sample_count,
            "sampled_images": [str(path) for path in images],
        }
    if not images:
        parser.exit(2, "error: no input images found\n")

    frames = tuple(evaluate_ppm_image(path, thresholds) for path in images)
    report = EvaluationReport("floor_dark_blob_shoe_baseline_v1", thresholds, frames, input_summary=input_summary)
    evidence_dir = output_dir / "evidence_frames"
    for image, frame in zip(images, frames):
        write_evidence_frame(image, frame, evidence_dir / f"{frame.frame_id}_evidence.ppm")
    report_path = write_report(report, output_dir / "shoe_detector_evaluation.json")
    print(report_path)
    print(json.dumps({k: report.to_json_dict()[k] for k in ("frame_count", "candidate_count", "accepted_count")}, sort_keys=True))
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
