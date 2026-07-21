# Read-only mission observability web/PWA surface

`src/sphero_rvr_driver/mission_observability.py` is the read-only observability layer over the canonical Mission API control snapshot contract. It is deliberately dependency-free and ROS-free so phone/laptop UI checks can run from static/mock/replay data without starting a motor-capable process.

## Scope

The surface exposes status only:

- robot health summary
- sensor freshness for mission state, lidar scan, odom/TF, camera hook, and shoe detections
- battery and diagnostics
- pose and map preview references
- camera preview hook metadata
- detection/semantic markers
- mission event log
- final artifact links for `occupancy_map`, `semantic_map`, and `shoe_detections`

It consumes `MissionControlSnapshot.to_json_dict()` semantics from the canonical `mission_api.v2` control layer and preserves Mission API state/event names for downstream authenticated controls.

## Read-only contract

The helper router only serves GET routes:

```text
GET /                    -> responsive static HTML shell
GET /api/observability   -> JSON snapshot
GET /api/events          -> Server-Sent Events frame(s)
GET /manifest.webmanifest -> PWA manifest
```

No start, cancel, motor, raw ROS, or write route is exposed by the observability surface. Unsupported route/method attempts raise `ReadOnlyRouteError`; start/cancel work belongs in the separate authenticated controls module.

## Static/PWA artifact

`build_static_pwa_bundle()` returns:

- `index_html`: responsive shell with sections for the accepted observability panes
- `manifest`: minimal standalone PWA manifest
- `service_worker_js`: install-only service worker stub for later offline shell caching

The HTML fetches `/api/observability` and binds JSON into each section. It intentionally contains no mission-control form.

## Mock/replay telemetry

`build_mock_observability_snapshot(mission_snapshot)` wraps a canonical mission control snapshot with deterministic mock/replay health data and artifact references. This gives a stable phone/laptop viewport target without starting hardware, camera, ROS launch files, or any motor-capable process.
