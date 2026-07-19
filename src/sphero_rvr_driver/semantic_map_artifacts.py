"""Generate final semantic map artifacts for the shoe-mapping vertical slice.

This module is intentionally ROS-free. It consumes the committed replay artifacts
from VS02/VS04/VS05 and produces deterministic JSON/GeoJSON/text/image outputs
that downstream API/UI/E2E checks can inspect without hardware.
"""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable, Optional, Sequence


SCHEMA = "sphero_rvr_semantic_map.v1"
DEFAULT_LIMITS = (
    "observed coverage only: semantic claims are bounded to sampled map/camera/lidar coverage",
    "inaccessible regions: walls, fixture borders, and unmapped space are not searched free space",
    "occlusion: line-of-sight blockage can hide shoes or shift visible shoe footpoints",
    "uncertain detections: review-level detector outputs remain candidates, not confirmed objects",
    "detector/projection confidence: object confidence combines detector score with map-frame projection limits",
)


@dataclass(frozen=True)
class SemanticPose:
    x: float
    y: float
    yaw: float
    frame: str
    timestamp_ns: int

    def to_json_dict(self) -> dict[str, Any]:
        return {
            "x": round(self.x, 4),
            "y": round(self.y, 4),
            "yaw": round(self.yaw, 6),
            "frame": self.frame,
            "timestamp_ns": self.timestamp_ns,
        }


@dataclass(frozen=True)
class CoverageRegion:
    status: str
    description: str
    coverage_confidence: float

    def to_json_dict(self) -> dict[str, Any]:
        return {
            "status": self.status,
            "description": self.description,
            "coverage_confidence": round(self.coverage_confidence, 3),
        }


@dataclass(frozen=True)
class SemanticObject:
    object_id: str
    object_class: str
    status: str
    confidence: float
    detector_confidence: float
    projection_confidence: float
    pose: SemanticPose
    observation_count: int
    first_observed_ns: int
    last_observed_ns: int
    evidence_frame_refs: tuple[str, ...]
    source_run_id: str
    source_map_id: str
    uncertainty: dict[str, Any]
    coverage: dict[str, Any]

    def to_json_dict(self) -> dict[str, Any]:
        return {
            "object_id": self.object_id,
            "class": self.object_class,
            "status": self.status,
            "confidence": {
                "combined": round(self.confidence, 3),
                "detector": round(self.detector_confidence, 3),
                "projection": round(self.projection_confidence, 3),
            },
            "map_pose": self.pose.to_json_dict(),
            "observation_count": self.observation_count,
            "timestamps": {
                "first_observed_ns": self.first_observed_ns,
                "last_observed_ns": self.last_observed_ns,
            },
            "evidence_frame_refs": list(self.evidence_frame_refs),
            "source": {"run_id": self.source_run_id, "map_id": self.source_map_id},
            "uncertainty": self.uncertainty,
            "coverage": self.coverage,
        }

    def to_geojson_feature(self) -> dict[str, Any]:
        properties = self.to_json_dict()
        properties.pop("map_pose")
        return {
            "type": "Feature",
            "id": self.object_id,
            "geometry": {"type": "Point", "coordinates": [round(self.pose.x, 4), round(self.pose.y, 4)]},
            "properties": properties,
        }


@dataclass(frozen=True)
class SemanticMapInputs:
    map_yaml: Path
    detector_evaluation_json: Path
    map_observations_json: Path
    output_dir: Path
    source_run_id: str
    source_map_id: str
    coverage_regions: tuple[CoverageRegion, ...] = (
        CoverageRegion("observed", "Map cells and sampled camera frames covered by the replay pipeline", 0.0),
        CoverageRegion("inaccessible", "Unknown, occupied, or unreachable map regions were not searched", 0.0),
        CoverageRegion("occluded", "Objects hidden by obstacles or outside line of sight remain unverified", 0.0),
    )


@dataclass(frozen=True)
class SemanticMapArtifacts:
    semantic_json: Path
    geojson: Path
    annotated_map_image: Path
    coverage_report: Path
    mission_summary: Path

    def to_json_dict(self) -> dict[str, str]:
        return {
            "semantic_json": str(self.semantic_json),
            "geojson": str(self.geojson),
            "annotated_map_image": str(self.annotated_map_image),
            "coverage_report": str(self.coverage_report),
            "mission_summary": str(self.mission_summary),
        }


def generate_semantic_map_artifacts(inputs: SemanticMapInputs) -> SemanticMapArtifacts:
    inputs.output_dir.mkdir(parents=True, exist_ok=True)
    map_info = _load_map_info(inputs.map_yaml)
    observation_report = json.loads(inputs.map_observations_json.read_text())
    detector_report = json.loads(inputs.detector_evaluation_json.read_text())
    objects = _semantic_objects_from_tracks(
        observation_report,
        source_run_id=inputs.source_run_id,
        source_map_id=inputs.source_map_id,
    )
    _annotate_object_map_extent(objects, map_info)
    coverage = _coverage_summary(inputs.coverage_regions, detector_report, observation_report, map_info)

    semantic_payload = {
        "schema": SCHEMA,
        "sources": {
            "run_id": inputs.source_run_id,
            "map_id": inputs.source_map_id,
            "vs02_map_yaml": str(inputs.map_yaml),
            "vs04_detector_evaluation_json": str(inputs.detector_evaluation_json),
            "vs05_map_observations_json": str(inputs.map_observations_json),
        },
        "map": map_info,
        "coverage": coverage,
        "uncertainty_limits": list(DEFAULT_LIMITS),
        "object_count": len(objects),
        "observation_count": int(observation_report.get("observation_count", 0)),
        "objects": [obj.to_json_dict() for obj in objects],
    }
    geojson_payload = {
        "type": "FeatureCollection",
        "name": "sphero_rvr_semantic_shoe_map",
        "schema": SCHEMA,
        "features": [obj.to_geojson_feature() for obj in objects],
    }

    semantic_json = inputs.output_dir / "semantic_map.json"
    geojson = inputs.output_dir / "semantic_map.geojson"
    annotated_map_image = inputs.output_dir / "annotated_semantic_map.ppm"
    coverage_report = inputs.output_dir / "coverage_uncertainty_report.md"
    mission_summary = inputs.output_dir / "mission_summary.md"

    _write_json(semantic_json, semantic_payload)
    _write_json(geojson, geojson_payload)
    _write_annotated_map(map_info, objects, annotated_map_image)
    coverage_report.write_text(_render_coverage_report(semantic_payload, detector_report), encoding="utf-8")
    mission_summary.write_text(_render_mission_summary(semantic_payload), encoding="utf-8")

    return SemanticMapArtifacts(
        semantic_json=semantic_json,
        geojson=geojson,
        annotated_map_image=annotated_map_image,
        coverage_report=coverage_report,
        mission_summary=mission_summary,
    )


def _semantic_objects_from_tracks(
    observation_report: dict[str, Any], *, source_run_id: str, source_map_id: str
) -> tuple[SemanticObject, ...]:
    observations_by_track: dict[str, list[dict[str, Any]]] = {}
    for observation in observation_report.get("observations", []):
        observations_by_track.setdefault(str(observation["track_id"]), []).append(observation)

    objects: list[SemanticObject] = []
    for track in observation_report.get("tracks", []):
        track_id = str(track["track_id"])
        observations = observations_by_track.get(track_id, [])
        evidence_refs = _track_evidence_refs(track, observations)
        timestamps = _observation_timestamps(track, observations)
        detector_confidence = float(track.get("confidence", 0.0))
        projection_confidence = _projection_confidence(observations, observation_report)
        combined = detector_confidence * projection_confidence
        position = track.get("position", {})
        timestamp_ns = int(position.get("timestamp_ns", timestamps[1]))
        status = str(track.get("status", "unknown"))
        objects.append(
            SemanticObject(
                object_id=track_id,
                object_class=str(track.get("label", "unknown")),
                status=status,
                confidence=combined,
                detector_confidence=detector_confidence,
                projection_confidence=projection_confidence,
                pose=SemanticPose(
                    x=float(position.get("x", 0.0)),
                    y=float(position.get("y", 0.0)),
                    yaw=float(position.get("yaw", 0.0)),
                    frame=str(track.get("frame", "map")),
                    timestamp_ns=timestamp_ns,
                ),
                observation_count=int(track.get("observation_count", len(observations))),
                first_observed_ns=timestamps[0],
                last_observed_ns=timestamps[1],
                evidence_frame_refs=evidence_refs,
                source_run_id=source_run_id,
                source_map_id=source_map_id,
                uncertainty={
                    "classification": _classification_uncertainty(status),
                    "projection": _projection_uncertainty(observations, observation_report),
                    "limits": sorted(_unique_limits(observations, observation_report)),
                },
                coverage={
                    "region_status": "observed",
                    "coverage_confidence": projection_confidence,
                    "scope": "inside observed replay/map-frame projection coverage only",
                },
            )
        )
    return tuple(objects)


def _track_evidence_refs(track: dict[str, Any], observations: Iterable[dict[str, Any]]) -> tuple[str, ...]:
    refs: list[str] = []
    for item in track.get("evidence", []):
        path = str(item.get("path", ""))
        if path and path not in refs:
            refs.append(path)
    for observation in observations:
        evidence = observation.get("evidence", {})
        path = str(evidence.get("path", ""))
        if path and path not in refs:
            refs.append(path)
    return tuple(refs)


def _observation_timestamps(track: dict[str, Any], observations: Sequence[dict[str, Any]]) -> tuple[int, int]:
    timestamps: list[int] = []
    for observation in observations:
        position = observation.get("position", {})
        if "timestamp_ns" in position:
            timestamps.append(int(position["timestamp_ns"]))
        evidence = observation.get("evidence", {})
        if "timestamp_ns" in evidence:
            timestamps.append(int(evidence["timestamp_ns"]))
    if not timestamps:
        timestamp = int(track.get("position", {}).get("timestamp_ns", 0))
        timestamps.append(timestamp)
    return min(timestamps), max(timestamps)


def _classification_uncertainty(status: str) -> str:
    if status == "accepted":
        return "accepted detector evidence, still bounded by replay coverage"
    if status == "review":
        return "review-level uncertain detection; needs operator/model confirmation before treating as a confirmed shoe"
    return "rejected or unknown detector status; not a confirmed semantic object"


def _projection_uncertainty(observations: Sequence[dict[str, Any]], observation_report: dict[str, Any]) -> str:
    limits = _unique_limits(observations, observation_report)
    if limits:
        return "; ".join(sorted(limits))
    return "map-frame projection inherits pose, camera calibration, ground-plane, and occlusion limits"


def _unique_limits(observations: Sequence[dict[str, Any]], observation_report: dict[str, Any]) -> set[str]:
    limits = {str(limit) for limit in observation_report.get("uncertainty_limits", [])}
    for observation in observations:
        limits.update(str(limit) for limit in observation.get("uncertainty_limits", []))
    return {limit for limit in limits if limit}


def _projection_confidence(observations: Sequence[dict[str, Any]], observation_report: dict[str, Any]) -> float:
    limits = _unique_limits(observations, observation_report)
    if not observations:
        return 0.0
    # Conservative deterministic score: every inherited uncertainty reduces the
    # projection confidence, but never below a visible review-floor for tracked observations.
    return max(0.35, min(0.95, 0.9 - 0.05 * len(limits)))


def _coverage_summary(
    regions: Sequence[CoverageRegion],
    detector_report: dict[str, Any],
    observation_report: dict[str, Any],
    map_info: dict[str, Any],
) -> dict[str, Any]:
    pixel_counts = map_info.get("pixel_counts", {})
    total = max(1, int(pixel_counts.get("total", 1)))
    free = int(pixel_counts.get("free", 0))
    occupied = int(pixel_counts.get("occupied", 0))
    unknown = int(pixel_counts.get("unknown", 0))
    observed_fraction = round(free / total, 3)
    explicit_regions = [region.to_json_dict() for region in regions]
    for region in explicit_regions:
        if region["status"] == "observed" and region["coverage_confidence"] == 0.0:
            region["coverage_confidence"] = observed_fraction
    return {
        "regions": explicit_regions,
        "map_cell_counts": {"free": free, "occupied": occupied, "unknown": unknown, "total": total},
        "observed_free_fraction": observed_fraction,
        "detector_sampled_frames": int(detector_report.get("frame_count", 0)),
        "detector_candidate_count": int(detector_report.get("candidate_count", 0)),
        "detector_accepted_count": int(detector_report.get("accepted_count", 0)),
        "semantic_observation_count": int(observation_report.get("observation_count", 0)),
        "semantic_track_count": int(observation_report.get("track_count", 0)),
        "claim_limit": "does not claim every shoe; no conclusions outside observed coverage",
    }


def _annotate_object_map_extent(objects: Sequence[SemanticObject], map_info: dict[str, Any]) -> None:
    width = int(map_info["width"])
    height = int(map_info["height"])
    resolution = float(map_info["resolution"])
    origin = map_info.get("origin", [0.0, 0.0, 0.0])
    origin_x = float(origin[0])
    origin_y = float(origin[1])
    for obj in objects:
        pixel_x = int(round((obj.pose.x - origin_x) / resolution))
        pixel_y_from_bottom = int(round((obj.pose.y - origin_y) / resolution))
        inside = 0 <= pixel_x < width and 0 <= pixel_y_from_bottom < height
        obj.coverage["map_image_marker"] = "in_extent" if inside else "clamped_to_edge"


def _load_map_info(map_yaml: Path) -> dict[str, Any]:
    yaml_data = _parse_simple_map_yaml(map_yaml)
    image_path = Path(str(yaml_data["image"]))
    if not image_path.is_absolute():
        image_path = map_yaml.parent / image_path
    pgm = _read_p2_pgm(image_path)
    return {
        "yaml": str(map_yaml),
        "image": str(image_path),
        "resolution": float(yaml_data.get("resolution", 0.0)),
        "origin": yaml_data.get("origin", [0.0, 0.0, 0.0]),
        "width": pgm["width"],
        "height": pgm["height"],
        "pixel_counts": _classify_map_pixels(pgm["pixels"], int(yaml_data.get("negate", 0))),
    }


def _parse_simple_map_yaml(path: Path) -> dict[str, Any]:
    data: dict[str, Any] = {}
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#") or ":" not in line:
            continue
        key, value = line.split(":", 1)
        data[key.strip()] = _parse_yaml_scalar(value.strip())
    if "image" not in data:
        raise ValueError(f"map YAML {path} is missing image")
    return data


def _parse_yaml_scalar(value: str) -> Any:
    if value.startswith("[") and value.endswith("]"):
        inner = value[1:-1].strip()
        if not inner:
            return []
        return [_parse_yaml_scalar(part.strip()) for part in inner.split(",")]
    try:
        if "." in value:
            return float(value)
        return int(value)
    except ValueError:
        return value.strip('"\'')


def _read_p2_pgm(path: Path) -> dict[str, Any]:
    tokens: list[str] = []
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.split("#", 1)[0].strip()
        if line:
            tokens.extend(line.split())
    if len(tokens) < 4 or tokens[0] != "P2":
        raise ValueError(f"expected ASCII P2 PGM map at {path}")
    width = int(tokens[1])
    height = int(tokens[2])
    max_value = int(tokens[3])
    values = [int(token) for token in tokens[4:]]
    if len(values) != width * height:
        raise ValueError(f"PGM pixel count mismatch for {path}: expected {width * height}, got {len(values)}")
    return {"width": width, "height": height, "max_value": max_value, "pixels": values}


def _classify_map_pixels(pixels: Sequence[int], negate: int) -> dict[str, int]:
    counts = {"free": 0, "occupied": 0, "unknown": 0, "total": len(pixels)}
    for value in pixels:
        normalized = 255 - value if negate else value
        if normalized <= 30:
            counts["occupied"] += 1
        elif normalized >= 240:
            counts["free"] += 1
        else:
            counts["unknown"] += 1
    return counts


def _write_annotated_map(map_info: dict[str, Any], objects: Sequence[SemanticObject], output_path: Path) -> None:
    pgm = _read_p2_pgm(Path(str(map_info["image"])))
    pixels = list(pgm["pixels"])
    rgb = [(_gray(value), _gray(value), _gray(value)) for value in pixels]
    width = int(pgm["width"])
    height = int(pgm["height"])
    resolution = float(map_info["resolution"])
    origin = map_info.get("origin", [0.0, 0.0, 0.0])
    origin_x = float(origin[0])
    origin_y = float(origin[1])
    for obj in objects:
        pixel_x = int(round((obj.pose.x - origin_x) / resolution))
        pixel_y_from_bottom = int(round((obj.pose.y - origin_y) / resolution))
        pixel_x = max(0, min(width - 1, pixel_x))
        pixel_y_from_bottom = max(0, min(height - 1, pixel_y_from_bottom))
        pixel_y = height - 1 - pixel_y_from_bottom
        _draw_cross(rgb, width, height, pixel_x, pixel_y, color=(255, 0, 0))
    lines = ["P3", "# annotated semantic map: red crosses mark semantic shoe tracks", f"{width} {height}", "255"]
    for row in range(height):
        items: list[str] = []
        for col in range(width):
            items.extend(str(channel) for channel in rgb[row * width + col])
        lines.append(" ".join(items))
    output_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _gray(value: int) -> int:
    return max(0, min(255, int(value)))


def _draw_cross(
    rgb: list[tuple[int, int, int]], width: int, height: int, x: int, y: int, *, color: tuple[int, int, int]
) -> None:
    for dx, dy in ((0, 0), (-1, 0), (1, 0), (0, -1), (0, 1)):
        px = x + dx
        py = y + dy
        if 0 <= px < width and 0 <= py < height:
            rgb[py * width + px] = color


def _render_coverage_report(semantic_payload: dict[str, Any], detector_report: dict[str, Any]) -> str:
    coverage = semantic_payload["coverage"]
    lines = [
        "# Coverage and uncertainty report",
        "",
        "Observed coverage: limited to replay-sampled map/camera/lidar evidence and free map cells; this does not claim every shoe.",
        "Inaccessible regions: occupied, unknown, or unreachable cells are excluded from semantic absence claims.",
        "Occlusion: shoes hidden behind obstacles, outside the camera view, or below reliable ground-plane projection remain unverified.",
        "Uncertain detections: review-level tracks are preserved as candidates with explicit status and evidence references.",
        "Detector confidence and projection confidence are reported separately; combined confidence is conservative.",
        "No bogus every-shoe claim: the artifacts avoid completeness claims outside observed coverage.",
        "",
        "## Counts",
        f"- Semantic tracks: {semantic_payload['object_count']}",
        f"- Semantic observations: {semantic_payload['observation_count']}",
        f"- Detector candidates: {coverage['detector_candidate_count']}",
        f"- Detector accepted count: {coverage['detector_accepted_count']}",
        f"- Detector coverage statement: {detector_report.get('coverage_statement', 'not provided')}",
        f"- Map cell counts: {coverage['map_cell_counts']}",
        "",
        "## Regions",
    ]
    for region in coverage["regions"]:
        lines.append(f"- {region['status']}: {region['description']} (confidence {region['coverage_confidence']})")
    lines.append("")
    return "\n".join(lines)


def _render_mission_summary(semantic_payload: dict[str, Any]) -> str:
    count = semantic_payload["object_count"]
    noun = "semantic shoe track" if count == 1 else "semantic shoe tracks"
    observations = semantic_payload["observation_count"]
    return (
        "# Mission summary\n\n"
        f"Generated {count} {noun} from {observations} map-frame observations. "
        "Results are bounded to observed coverage; the mission summary makes no all-shoes or absence claim outside observed coverage.\n"
    )


def _write_json(path: Path, payload: dict[str, Any]) -> None:
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Generate VS06 semantic map JSON/GeoJSON/report artifacts")
    parser.add_argument("--map-yaml", type=Path, required=True, help="VS02 map YAML")
    parser.add_argument("--detector-evaluation-json", type=Path, required=True, help="VS04 detector evaluation JSON")
    parser.add_argument("--map-observations-json", type=Path, required=True, help="VS05 map observations JSON")
    parser.add_argument("--output-dir", type=Path, default=Path("artifacts/vs06_semantic_map"))
    parser.add_argument("--source-run-id", default="canonical_replay_run")
    parser.add_argument("--source-map-id", default="vs02_replay_fixture_map")
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_arg_parser().parse_args(argv)
    artifacts = generate_semantic_map_artifacts(
        SemanticMapInputs(
            map_yaml=args.map_yaml,
            detector_evaluation_json=args.detector_evaluation_json,
            map_observations_json=args.map_observations_json,
            output_dir=args.output_dir,
            source_run_id=args.source_run_id,
            source_map_id=args.source_map_id,
        )
    )
    print(json.dumps(artifacts.to_json_dict(), sort_keys=True))
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
