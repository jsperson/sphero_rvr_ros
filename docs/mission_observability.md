# Read-only mission observability web/PWA surface

`src/sphero_rvr_driver/mission_observability.py` is the VS08A read-only observability layer over the VS07 Mission API snapshot contract. It is deliberately dependency-free and ROS-free so phone/laptop UI checks can run from static/mock/replay data before any authenticated mission controls exist.

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

It consumes `MissionSnapshot.to_json_dict()` semantics from `mission_api.v1` and preserves the Mission API state names and event names for downstream authenticated controls.

## Read-only contract

The helper router only serves GET routes:

```text
GET /                    -> responsive static HTML shell
GET /api/observability   -> JSON snapshot
GET /api/events          -> Server-Sent Events frame(s)
GET /manifest.webmanifest -> PWA manifest
```

No start, cancel, motor, raw ROS, or write route is exposed by VS08A. Unsupported route/method attempts raise `ReadOnlyRouteError`; downstream start/cancel work should use a separate authenticated module instead of adding mutation to this one.

## Static/PWA artifact

`build_static_pwa_bundle()` returns:

- `index_html`: responsive shell with sections for the accepted observability panes
- `manifest`: minimal standalone PWA manifest
- `service_worker_js`: install-only service worker stub for later offline shell caching

The HTML fetches `/api/observability` and binds JSON into each section. It intentionally contains no mission-control form.

## Mock/replay telemetry

`build_mock_observability_snapshot(mission_snapshot)` wraps a VS07 mission snapshot with deterministic mock/replay health data and artifact references. This gives a stable phone/laptop viewport target without starting hardware, camera, ROS launch files, or any motor-capable process.
