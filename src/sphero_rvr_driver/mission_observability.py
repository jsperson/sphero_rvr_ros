"""Read-only mission observability JSON and static PWA surfaces.

The observability layer consumes the canonical Mission API control snapshot
shape and only publishes GET-able status views. It deliberately has no
start/cancel/motor command route so this surface can stay safe for replay demos
and phones on local Wi-Fi.
"""

from __future__ import annotations

import html
import json
from dataclasses import asdict, dataclass, field
from typing import Any, Iterable, Mapping, Sequence


class ReadOnlyRouteError(ValueError):
    """Raised when a caller asks the observability surface to mutate mission state."""


@dataclass(frozen=True)
class FreshnessStatus:
    fresh: bool
    age_s: float
    source: str
    warning_after_s: float

    def to_json_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class BatteryStatus:
    percentage: float
    voltage_v: float
    charging: bool = False

    def to_json_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class DiagnosticItem:
    name: str
    level: str
    message: str

    def to_json_dict(self) -> dict[str, str]:
        return asdict(self)


@dataclass(frozen=True)
class PosePreview:
    frame: str
    x_m: float
    y_m: float
    yaw_rad: float
    covariance_hint: str = "mock/replay pose only; live covariance not asserted"

    def to_json_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class MapPreview:
    frame: str
    occupancy_map_ref: str
    semantic_map_ref: str
    width_px: int
    height_px: int
    resolution_m: float

    def to_json_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class CameraPreview:
    stream_available: bool
    frame_ref: str
    frame_id: str
    note: str = "camera preview hook only; no live camera is started by this surface"

    def to_json_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class SemanticMarker:
    marker_id: str
    label: str
    status: str
    confidence: float
    frame: str
    x_m: float
    y_m: float
    evidence_ref: str

    def to_json_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class StaticPwaBundle:
    index_html: str
    manifest: Mapping[str, Any]
    service_worker_js: str


@dataclass(frozen=True)
class MissionObservabilitySnapshot:
    mission: Any
    robot_health: Mapping[str, Any]
    sensor_freshness: Mapping[str, FreshnessStatus]
    battery: BatteryStatus
    diagnostics: Sequence[DiagnosticItem]
    pose: PosePreview
    map_preview: MapPreview
    camera_preview: CameraPreview
    semantic_markers: Sequence[SemanticMarker]
    artifact_links: Mapping[str, str]
    read_only: bool = True
    allowed_methods: tuple[str, ...] = ("GET",)
    write_endpoints: tuple[str, ...] = field(default_factory=tuple)

    @property
    def api_version(self) -> str:
        return str(self.mission.to_json_dict()["api_version"])

    def to_json_dict(self) -> dict[str, Any]:
        mission_payload = self.mission.to_json_dict()
        return {
            "api_version": self.api_version,
            "read_only": self.read_only,
            "allowed_methods": list(self.allowed_methods),
            "write_endpoints": list(self.write_endpoints),
            "mission": mission_payload,
            "robot_health": dict(self.robot_health),
            "sensor_freshness": {
                name: status.to_json_dict() for name, status in self.sensor_freshness.items()
            },
            "battery": self.battery.to_json_dict(),
            "diagnostics": [item.to_json_dict() for item in self.diagnostics],
            "pose": self.pose.to_json_dict(),
            "map_preview": self.map_preview.to_json_dict(),
            "camera_preview": self.camera_preview.to_json_dict(),
            "semantic_markers": [marker.to_json_dict() for marker in self.semantic_markers],
            "mission_events": list(self.mission.event_log),
            "artifact_links": dict(self.artifact_links),
        }


def build_mock_observability_snapshot(mission: Any) -> MissionObservabilitySnapshot:
    """Build deterministic replay/mock observability from a canonical mission snapshot."""

    artifact_links = {
        "occupancy_map": "artifacts/vs02_slam_replay/shoe_room_map.yaml",
        "semantic_map": "artifacts/vs05_shoe_map_projection/shoe_map_observations.json",
        "shoe_detections": "artifacts/vs04_shoe_detector_replay/shoe_detector_evaluation.json",
    }
    mission_payload = mission.to_json_dict()
    telemetry = mission_payload.get("telemetry", {})
    return MissionObservabilitySnapshot(
        mission=mission,
        robot_health={
            "summary": "mock/replay",
            "mission_state": mission_payload["state"],
            "terminal": bool(telemetry.get("terminal", False)),
            "direct_write_controls": False,
        },
        sensor_freshness={
            "mission_state": FreshnessStatus(True, 0.0, "mission_api_snapshot", 5.0),
            "lidar_scan": FreshnessStatus(True, 0.08, "replay:/scan", 1.0),
            "odom_tf": FreshnessStatus(True, 0.12, "replay:odom->base_link", 1.0),
            "camera_frame": FreshnessStatus(False, 999.0, "hook:not_started", 1.0),
            "shoe_detections": FreshnessStatus(True, 0.25, "replay:shoe_detector", 5.0),
        },
        battery=BatteryStatus(percentage=0.76, voltage_v=7.34, charging=False),
        diagnostics=(
            DiagnosticItem("mission_api", "OK", "canonical snapshot serialized"),
            DiagnosticItem("observability", "OK", "read-only mock/replay surface"),
        ),
        pose=PosePreview(frame="map", x_m=1.0, y_m=2.0, yaw_rad=0.5),
        map_preview=MapPreview(
            frame="map",
            occupancy_map_ref=artifact_links["occupancy_map"],
            semantic_map_ref=artifact_links["semantic_map"],
            width_px=640,
            height_px=480,
            resolution_m=0.05,
        ),
        camera_preview=CameraPreview(
            stream_available=False,
            frame_ref="artifacts/vs04_shoe_detector_replay/evidence/latest_evidence.png",
            frame_id="camera_optical_frame",
        ),
        semantic_markers=(
            SemanticMarker(
                marker_id="shoe_track_0001",
                label="shoe",
                status="accepted",
                confidence=0.82,
                frame="map",
                x_m=1.27,
                y_m=2.11,
                evidence_ref="artifacts/vs04_shoe_detector_replay/evidence/bag_frame_0000_evidence.png",
            ),
        ),
        artifact_links=artifact_links,
    )


def iter_event_stream_frames(snapshots: Iterable[MissionObservabilitySnapshot]) -> Iterable[str]:
    """Yield Server-Sent Event frames carrying JSON observability snapshots."""

    for snapshot in snapshots:
        data = json.dumps(snapshot.to_json_dict(), sort_keys=True, separators=(",", ":"))
        yield f"event: mission_observability\ndata: {data}\n\n"


def build_static_pwa_bundle(*, app_name: str = "RVR Mission Observability") -> StaticPwaBundle:
    safe_name = html.escape(app_name, quote=True)
    manifest = {
        "name": app_name,
        "short_name": "RVR Watch",
        "start_url": ".",
        "display": "standalone",
        "background_color": "#0f172a",
        "theme_color": "#38bdf8",
        "icons": [],
    }
    index_html = f"""<!doctype html>
<html lang=\"en\">
<head>
  <meta charset=\"utf-8\">
  <meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">
  <meta name=\"theme-color\" content=\"#38bdf8\">
  <link rel=\"manifest\" href=\"/manifest.webmanifest\">
  <title>{safe_name}</title>
  <style>
    :root {{ color-scheme: dark; font-family: system-ui, -apple-system, Segoe UI, sans-serif; background: #020617; color: #e2e8f0; }}
    body {{ margin: 0; }}
    header {{ padding: 1rem; border-bottom: 1px solid #1e293b; background: #0f172a; }}
    main {{ display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); gap: 1rem; padding: 1rem; }}
    section {{ border: 1px solid #1e293b; border-radius: 0.8rem; padding: 1rem; background: #111827; min-height: 8rem; }}
    h1, h2 {{ margin: 0 0 0.5rem; }}
    .map {{ min-height: 14rem; border: 1px dashed #38bdf8; border-radius: 0.5rem; display: grid; place-items: center; }}
    .muted {{ color: #94a3b8; }}
    @media (max-width: 720px) {{ main {{ grid-template-columns: 1fr; padding: 0.75rem; }} section {{ min-height: 6rem; }} }}
  </style>
</head>
<body>
  <header>
    <h1>RVR Mission Observability</h1>
    <p class=\"muted\">Read-only mission watch surface for phone and laptop replay/static views.</p>
  </header>
  <main aria-live=\"polite\">
    <section id=\"robot-health\"><h2>Robot health</h2><pre data-bind=\"robot_health\"></pre></section>
    <section id=\"sensor-freshness\"><h2>Sensor freshness</h2><pre data-bind=\"sensor_freshness\"></pre></section>
    <section id=\"battery-diagnostics\"><h2>Battery & diagnostics</h2><pre data-bind=\"battery\"></pre><pre data-bind=\"diagnostics\"></pre></section>
    <section id=\"pose-map\"><h2>Pose / map preview</h2><div class=\"map\" data-bind=\"map_preview\">Map preview hook</div><pre data-bind=\"pose\"></pre></section>
    <section id=\"camera-preview\"><h2>Camera preview</h2><pre data-bind=\"camera_preview\"></pre></section>
    <section id=\"semantic-markers\"><h2>Semantic markers</h2><pre data-bind=\"semantic_markers\"></pre></section>
    <section id=\"mission-events\"><h2>Mission events</h2><pre data-bind=\"mission_events\"></pre></section>
    <section id=\"final-artifacts\"><h2>Final artifacts</h2><pre data-bind=\"artifact_links\"></pre></section>
  </main>
  <script>
    async function refresh() {{
      const response = await fetch('/api/observability');
      const payload = await response.json();
      for (const node of document.querySelectorAll('[data-bind]')) {{
        const key = node.getAttribute('data-bind');
        node.textContent = JSON.stringify(payload[key], null, 2);
      }}
    }}
    refresh().catch((error) => console.warn('observability refresh failed', error));
  </script>
</body>
</html>
"""
    service_worker_js = "self.addEventListener('install', () => self.skipWaiting());\n"
    return StaticPwaBundle(index_html=index_html, manifest=manifest, service_worker_js=service_worker_js)


def handle_observability_request(
    method: str,
    path: str,
    snapshot: MissionObservabilitySnapshot,
) -> tuple[int, str, str]:
    """Tiny dependency-free request router for tests, demos, or embedding."""

    normalized_method = method.upper()
    normalized_path = path.rstrip("/") or "/"
    if normalized_method != "GET":
        raise ReadOnlyRouteError("read-only observability surface only supports GET")
    if normalized_path in {"/api/mission/start", "/api/mission/cancel", "/api/motor", "/api/write"}:
        raise ReadOnlyRouteError(f"{normalized_path} is not exposed by the read-only observability surface")
    if normalized_path == "/api/observability":
        return 200, "application/json", json.dumps(snapshot.to_json_dict(), sort_keys=True)
    if normalized_path == "/api/events":
        return 200, "text/event-stream", "".join(iter_event_stream_frames((snapshot,)))
    if normalized_path == "/manifest.webmanifest":
        return 200, "application/manifest+json", json.dumps(build_static_pwa_bundle().manifest, sort_keys=True)
    if normalized_path == "/":
        return 200, "text/html; charset=utf-8", build_static_pwa_bundle().index_html
    raise ReadOnlyRouteError(f"{normalized_path} is not exposed by the read-only observability surface")
