"""Browser-owned M7.6 proposal, approval, and canonical-session lifecycle."""

from __future__ import annotations

import json
import math
import threading
import time
from typing import Any, Mapping, Optional, Protocol
import uuid

from .hierarchical_physical_binding import (
    ACCEPTED_DIRECTIONAL_ADDENDUM_SHA256,
    ACCEPTED_M7_3_EVIDENCE_SHA256,
    ACCEPTED_M7_4_EVIDENCE_SHA256,
    APPROVAL_SCHEMA,
    MISSION_LEASE_MAX_S,
    PHYSICAL_PROPOSAL_SCHEMA,
    PREFLIGHT_SCHEMA,
    HierarchicalPhysicalLimits,
    canonical_digest,
)
from .live_mission_service import LiveStateCache, snapshot_evidence
from .mission_api import MissionValidationError
from .mission_service import MissionService


CANONICAL_M7_OBJECTIVE = (
    "Explore this room, identify and map the shoes and any recognized people, "
    "inspect uncertain findings from another viewpoint, then return or stop safely."
)
CANONICAL_OBJECT_CLASSES = ("shoe", "person")
CANONICAL_APPROVAL_PREFIX = "APPROVE M7.6 CANONICAL MISSION "
CONTROLLER_SOURCE = "hierarchical_controller"
ADAPTER_SOURCE = "hierarchical_adapter"
PREFLIGHT_MAX_AGE_S = {
    # Stationary preflight only establishes that every required subsystem is
    # alive. Motion remains locked here; the tighter active limits below are
    # enforced before and throughout command authority.
    "lidar": 5.00,
    "camera": 5.00,
    "localization": 5.00,
    "semantic_map": 5.00,
}
ACTIVE_SENSOR_MAX_AGE_S = {
    "lidar": 0.50,
    "camera": 3.00,
    "localization": 0.500,
    "semantic_map": 3.00,
}
def _finite_mission_lease(value: Any) -> float:
    if isinstance(value, bool):
        raise MissionValidationError(
            "M7.6 mission lease must be a finite JSON number"
        )
    try:
        parsed = float(value)
    except (TypeError, ValueError) as exc:
        raise MissionValidationError(
            "M7.6 mission lease must be a finite JSON number"
        ) from exc
    if not math.isfinite(parsed):
        raise MissionValidationError(
            "M7.6 mission lease must be a finite JSON number"
        )
    return parsed


class HierarchicalSessionLifecycle(Protocol):
    activation_capable: bool

    def activate(
        self,
        *,
        proposal: Mapping[str, Any],
        approval: Mapping[str, Any],
        now_s: float,
        cancel_event: Optional[threading.Event] = None,
    ) -> Mapping[str, Any]: ...

    def deactivate(self, *, reason: str) -> Mapping[str, Any]: ...

    def status(self) -> Mapping[str, Any]: ...


class HierarchicalPhysicalMissionController:
    """Own exactly one canonical physical mission and its non-resumable lease."""

    def __init__(
        self,
        service: MissionService,
        cache: LiveStateCache,
        session_lifecycle: HierarchicalSessionLifecycle,
        *,
        execution_enabled: bool,
        clock_s: Any = time.time,
        monitor_period_s: float = 0.10,
    ) -> None:
        self.service = service
        self.cache = cache
        self.session_lifecycle = session_lifecycle
        self.execution_enabled = bool(execution_enabled)
        self._clock_s = clock_s
        self.monitor_period_s = float(monitor_period_s)
        if self.execution_enabled != bool(session_lifecycle.activation_capable):
            raise MissionValidationError(
                "hierarchical controller and session activation gates must agree"
            )
        if not 0.05 <= self.monitor_period_s <= 1.0:
            raise MissionValidationError(
                "hierarchical monitor period must be between 0.05 and 1.0 seconds"
            )
        self._lock = threading.RLock()
        self._threads: dict[str, threading.Thread] = {}
        self._activation_cancel: dict[str, threading.Event] = {}
        self._active_mission_id = ""
        self._closed = False

    def submit(
        self,
        prompt: str,
        *,
        session_id: str,
        source: str = "web",
        mission_id: Optional[str] = None,
        mission_lease_s: Optional[float] = None,
        operator: str = "",
        authentication_source: str = "",
    ) -> dict[str, Any]:
        del operator, authentication_source
        objective = str(prompt).strip()
        if objective != CANONICAL_M7_OBJECTIVE:
            raise MissionValidationError(
                "M7.6 accepts only the reviewed canonical semantic mission"
            )
        selected_lease_s = (
            MISSION_LEASE_MAX_S
            if mission_lease_s is None
            else _finite_mission_lease(mission_lease_s)
        )
        if (
            selected_lease_s <= 0.0
            or selected_lease_s > MISSION_LEASE_MAX_S
        ):
            raise MissionValidationError(
                "M7.6 mission lease must be greater than 0 and no more than "
                f"{MISSION_LEASE_MAX_S:g} seconds"
            )
        with self._lock:
            self._ensure_open()
            if self._active_mission_id or self._threads:
                raise MissionValidationError(
                    "another canonical physical mission is active or awaiting cleanup"
                )
            identifier = str(
                mission_id or f"m7-canonical-{uuid.uuid4().hex}"
            )
            snapshot = self.service.begin_prompt_mission(
                mission_id=identifier,
                session_id=session_id,
                prompt=objective,
                source=source,
                provider_call_started=False,
            )
            created_at_s = float(self._clock_s())
            payload = {
                "schema": PHYSICAL_PROPOSAL_SCHEMA,
                "mission_id": identifier,
                "objective": objective,
                "objective_revision": 1,
                "requested_object_classes": list(
                    CANONICAL_OBJECT_CLASSES
                ),
                "source_sha": self.service.source_sha,
                "created_at_s": created_at_s,
                "mission_lease_s": selected_lease_s,
            }
            proposal = {
                **payload,
                "proposal_digest": canonical_digest(payload),
            }
            return self.service.record_hierarchical_physical_proposal(
                identifier, proposal
            )

    def approve(
        self,
        mission_id: str,
        *,
        supplied_approval: str,
        operator: str,
        authentication_source: str = "",
        physical_room_confirmation: Optional[Mapping[str, Any]] = None,
    ) -> dict[str, Any]:
        if not self.execution_enabled:
            raise MissionValidationError(
                "M7.6 canonical physical activation is disabled"
            )
        if str(authentication_source).strip() != "tailscale-serve":
            raise MissionValidationError(
                "M7.6 requires an authenticated Tailscale browser request"
            )
        principal = str(operator).strip()
        if not principal:
            raise MissionValidationError(
                "M7.6 authenticated operator identity is required"
            )
        persisted = self.service.prompt_status(mission_id)
        proposal = persisted.get("proposal", {})
        if (
            persisted.get("status") != "proposed"
            or not isinstance(proposal, Mapping)
            or proposal.get("schema") != PHYSICAL_PROPOSAL_SCHEMA
        ):
            raise MissionValidationError(
                "a persisted canonical M7.6 proposal is required"
            )
        expected_phrase = (
            CANONICAL_APPROVAL_PREFIX
            + str(proposal.get("proposal_digest", ""))
        )
        if str(supplied_approval).strip() != expected_phrase:
            raise MissionValidationError(
                "M7.6 approval does not bind the current canonical proposal"
            )
        room = dict(physical_room_confirmation or {})
        expected_room = {
            "attended": True,
            "level_bounded": True,
            "stairs_ledges_dropoffs_absent": True,
            "negative_obstacle_sensing_available": False,
        }
        if room != expected_room:
            raise MissionValidationError(
                "M7.6 requires explicit attended, level, bounded, no-dropoff room confirmation"
            )
        now_s = float(self._clock_s())
        preflight = self._sensor_preflight(now_s)
        limits = HierarchicalPhysicalLimits().to_json_dict()
        selected_lease_s = _finite_mission_lease(
            proposal.get("mission_lease_s")
        )
        if (
            selected_lease_s <= 0.0
            or selected_lease_s > limits["mission_lease_max_s"]
        ):
            raise MissionValidationError(
                "persisted M7.6 proposal has an invalid mission lease"
            )
        payload = {
            "schema": APPROVAL_SCHEMA,
            "gate": "m7.6",
            "mission_id": str(mission_id),
            "operator": principal,
            "source_sha": self.service.source_sha,
            "deployed_sha": self.service.deployed_sha,
            "reviewed_sha": self.service.source_sha,
            "proposal_digest": str(proposal["proposal_digest"]),
            "approval_id": f"m7.6-{uuid.uuid4().hex}",
            "approved_at_s": now_s,
            "expires_at_s": now_s + selected_lease_s,
            "mission_lease_s": selected_lease_s,
            "m7_3_evidence_sha256": ACCEPTED_M7_3_EVIDENCE_SHA256,
            "directional_addendum_sha256": (
                ACCEPTED_DIRECTIONAL_ADDENDUM_SHA256
            ),
            "m7_4_evidence_sha256": ACCEPTED_M7_4_EVIDENCE_SHA256,
            "room": expected_room,
            "limits": limits,
        }
        approval = {
            **payload,
            "approval_digest": canonical_digest(payload),
        }
        with self._lock:
            self._ensure_open()
            if self._active_mission_id:
                raise MissionValidationError(
                    "another canonical physical mission already owns authority"
                )
            approved = self.service.approve_hierarchical_physical_mission(
                mission_id,
                approval=approval,
                preflight_evidence=preflight,
                now_s=now_s,
            )
            self._active_mission_id = str(mission_id)
            cancellation = threading.Event()
            self._activation_cancel[str(mission_id)] = cancellation
            self._start_thread(
                str(mission_id),
                proposal=dict(proposal),
                approval=approval,
                cancellation=cancellation,
            )
            return approved

    def _sensor_preflight(self, now_s: float) -> dict[str, Any]:
        snapshot = self.cache.snapshot(now_s=now_s)
        evidence: dict[str, Any] = {}
        values: dict[str, Mapping[str, Any]] = {}
        for source_name, max_age_s in PREFLIGHT_MAX_AGE_S.items():
            record = snapshot.source(source_name)
            if (
                not record.valid
                or record.received_at_s is None
                or record.source_timestamp_s is None
                or not math.isfinite(
                    float(record.source_timestamp_s)
                )
            ):
                raise MissionValidationError(
                    f"M7.6 sensor preflight requires valid {source_name} evidence"
                )
            age_s = now_s - float(record.received_at_s)
            source_age_s = now_s - float(
                record.source_timestamp_s
            )
            if (
                not math.isfinite(age_s)
                or age_s < 0.0
                or age_s > max_age_s
                or not math.isfinite(source_age_s)
                or source_age_s < 0.0
                or source_age_s > max_age_s
            ):
                raise MissionValidationError(
                    f"M7.6 sensor preflight {source_name} evidence is stale"
                )
            value = dict(record.value)
            if (
                value.get("motion_authority") is not False
                or value.get("physical_execution_enabled") is not False
            ):
                raise MissionValidationError(
                    f"M7.6 sensor preflight {source_name} is not no-motion evidence"
                )
            values[source_name] = value
            evidence[source_name] = {
                "received_at_s": float(record.received_at_s),
                "source_timestamp_s": record.source_timestamp_s,
                "age_s": age_s,
                "source_age_s": source_age_s,
                "max_age_s": max_age_s,
                "value_digest": canonical_digest(value),
            }

        lidar = values["lidar"]
        if (
            lidar.get("schema") != "sphero_rvr.live_lidar.v1"
            or not str(lidar.get("scan_id", "")).strip()
            or int(lidar.get("sample_count", 0)) <= 0
        ):
            raise MissionValidationError(
                "M7.6 sensor preflight lidar scan is invalid"
            )
        camera = values["camera"]
        if (
            camera.get("schema")
            != "sphero_rvr.live_camera_perception.v1"
            or camera.get("calibrated") is not True
            or not str(camera.get("frame_id", "")).strip()
            or int(camera.get("width", 0)) <= 0
            or int(camera.get("height", 0)) <= 0
        ):
            raise MissionValidationError(
                "M7.6 sensor preflight calibrated camera frame is invalid"
            )
        localization = values["localization"]
        pose = localization.get("pose")
        if (
            str(localization.get("state", "")).lower() != "valid"
            or not isinstance(pose, Mapping)
            or not str(localization.get("map_id", "")).strip()
            or localization.get("stationary_session") is not True
        ):
            raise MissionValidationError(
                "M7.6 sensor preflight stationary SLAM localization is invalid"
            )
        for coordinate in ("x_m", "y_m", "yaw_rad"):
            try:
                coordinate_value = float(pose[coordinate])
            except (KeyError, TypeError, ValueError):
                raise MissionValidationError(
                    "M7.6 sensor preflight localization pose is invalid"
                ) from None
            if not math.isfinite(coordinate_value):
                raise MissionValidationError(
                    "M7.6 sensor preflight localization pose is invalid"
                )
        semantic_map = values["semantic_map"]
        map_value = semantic_map.get("map")
        occupancy = semantic_map.get("occupancy")
        if (
            semantic_map.get("schema")
            != "sphero_rvr.live_semantic_map.v1"
            or not isinstance(map_value, Mapping)
            or map_value.get("stationary") is not True
            or map_value.get("occupancy_available") is not True
            or not isinstance(occupancy, Mapping)
            or not str(occupancy.get("map_id", "")).strip()
        ):
            raise MissionValidationError(
                "M7.6 sensor preflight live SLAM map is invalid"
            )
        evidence["lidar"]["summary"] = {
            "schema": lidar["schema"],
            "scan_id": str(lidar["scan_id"]),
            "sample_count": int(lidar["sample_count"]),
        }
        evidence["camera"]["summary"] = {
            "schema": camera["schema"],
            "frame_id": str(camera["frame_id"]),
            "width": int(camera["width"]),
            "height": int(camera["height"]),
            "calibrated": True,
        }
        evidence["localization"]["summary"] = {
            "state": str(localization["state"]).lower(),
            "source": str(localization.get("source", "")),
            "map_id": str(localization["map_id"]),
            "stationary_session": True,
            "pose": {
                coordinate: float(pose[coordinate])
                for coordinate in ("x_m", "y_m", "yaw_rad")
            },
        }
        evidence["semantic_map"]["summary"] = {
            "schema": semantic_map["schema"],
            "revision": int(semantic_map.get("revision", 0)),
            "map_id": str(occupancy["map_id"]),
            "stationary": True,
            "occupancy_available": bool(
                map_value.get("occupancy_available", False)
            ),
        }
        payload = {
            "schema": PREFLIGHT_SCHEMA,
            "observed_at_s": now_s,
            "motion_authority": False,
            "physical_execution_enabled": False,
            "sources": evidence,
        }
        return {
            **payload,
            "preflight_digest": canonical_digest(payload),
        }

    def status(self, mission_id: str) -> dict[str, Any]:
        return self.service.prompt_status(mission_id)

    def cancel(
        self,
        mission_id: str,
        *,
        reason: str = "operator cancelled canonical physical mission",
    ) -> dict[str, Any]:
        with self._lock:
            snapshot = self.service.prompt_status(mission_id)
            if snapshot["status"] in {
                "complete",
                "failed",
                "cancelled",
                "recovery_required",
                "timeout",
            }:
                return snapshot
            cancellation = self._activation_cancel.get(str(mission_id))
            if cancellation is not None:
                cancellation.set()
            relock_error = self._deactivate(
                "canonical session relocked after operator cancellation"
            )
            self._active_mission_id = ""
            if relock_error:
                return self.service.transition_prompt_mission(
                    mission_id,
                    "recovery_required",
                    reason="canonical session cleanup failed: " + relock_error,
                )
            return self.service.cancel_prompt_mission(
                mission_id, reason=reason
            )

    def confirm_no_contact(
        self,
        mission_id: str,
        *,
        operator: str,
        authentication_source: str = "",
    ) -> dict[str, Any]:
        if str(authentication_source).strip() != "tailscale-serve":
            raise MissionValidationError(
                "no-contact observation requires an authenticated Tailscale request"
            )
        principal = str(operator).strip()
        if not principal:
            raise MissionValidationError(
                "no-contact observation requires the authenticated operator"
            )
        payload = {
            "schema": (
                "sphero_rvr.hierarchical_no_contact_observation.v1"
            ),
            "mission_id": str(mission_id),
            "operator": principal,
            "authentication_source": "tailscale-serve",
            "no_contact": True,
            "observed_at_s": float(self._clock_s()),
            "source_sha": self.service.source_sha,
            "deployed_sha": self.service.deployed_sha,
        }
        return self.service.record_hierarchical_no_contact_observation(
            mission_id,
            {
                **payload,
                "observation_digest": canonical_digest(payload),
            },
        )

    def service_snapshot(self) -> dict[str, Any]:
        session = dict(self.session_lifecycle.status())
        active = bool(session.get("active", False))
        evidence = snapshot_evidence(
            self.cache.snapshot(), max_age_s=1.0
        )
        binding = dict(
            getattr(self.service, "hierarchical_physical_binding", {})
        )
        binding.update(
            {
                "state": "active" if active else "locked",
                "m7_6_execution_approved": active,
                "canonical_mission_approved": active,
                "motion_authority": active,
                "physical_execution_enabled": active,
            }
        )
        return {
            "api_version": "mission_api.v2",
            "mode": "live/hierarchical-canonical",
            "source_sha": self.service.source_sha,
            "deployed_sha": self.service.deployed_sha,
            "planning_enabled": True,
            "live_execution_enabled": active,
            "approval_activation_enabled": self.execution_enabled,
            "hierarchical_canonical_enabled": True,
            "hierarchical_physical_binding": binding,
            "physical_session": session,
            "motion_authority": active,
            "direct_ros_commands_allowed": False,
            "credentials_accepted_over_service": False,
            "canonical_objective": CANONICAL_M7_OBJECTIVE,
            "canonical_limits": HierarchicalPhysicalLimits().to_json_dict(),
            "canonical_risk_ledger": [
                "No negative-obstacle sensing: stairs, ledges, and drop-offs must be absent.",
                "Approval requires fresh no-motion lidar, calibrated camera, SLAM map, and localization evidence.",
                "Localization older than 0.500 seconds holds and replans.",
                "Known low-speed chassis stalls may pause progress.",
                "Real model latency can cause honest wait_planning pauses on short hops.",
                "Camera pitch and far floor projection remain handling-sensitive.",
            ],
            "hierarchical_live_evidence": {
                CONTROLLER_SOURCE: evidence.get(CONTROLLER_SOURCE, {}),
                ADAPTER_SOURCE: evidence.get(ADAPTER_SOURCE, {}),
            },
            "capabilities": self.service.capabilities(),
        }

    def close(self, *, timeout_s: float = 45.0) -> None:
        with self._lock:
            self._closed = True
            for cancellation in self._activation_cancel.values():
                cancellation.set()
            threads = tuple(self._threads.values())
            active_mission_id = self._active_mission_id
        if active_mission_id:
            try:
                status = str(
                    self.service.prompt_status(active_mission_id).get(
                        "status", ""
                    )
                )
            except MissionValidationError:
                status = ""
            if status not in {
                "complete",
                "failed",
                "cancelled",
                "recovery_required",
                "timeout",
            }:
                self._finish(
                    active_mission_id,
                    status="recovery_required",
                    reason=(
                        "canonical session relocked after mission service shutdown"
                    ),
                )
            else:
                self._deactivate(
                    "canonical session relocked after mission service shutdown"
                )
        for thread in threads:
            thread.join(timeout=timeout_s)
        if any(thread.is_alive() for thread in threads):
            raise RuntimeError(
                "canonical physical mission monitor did not stop"
            )
        with self._lock:
            self._active_mission_id = ""

    def _start_thread(
        self,
        mission_id: str,
        *,
        proposal: Mapping[str, Any],
        approval: Mapping[str, Any],
        cancellation: threading.Event,
    ) -> None:
        thread = threading.Thread(
            target=self._activate_and_monitor,
            args=(
                mission_id,
                dict(proposal),
                dict(approval),
                cancellation,
            ),
            name=f"hierarchical-canonical-{mission_id}",
            daemon=True,
        )
        self._threads[mission_id] = thread
        thread.start()

    def _activate_and_monitor(
        self,
        mission_id: str,
        proposal: Mapping[str, Any],
        approval: Mapping[str, Any],
        cancellation: threading.Event,
    ) -> None:
        try:
            if cancellation.is_set():
                return
            activation = self.session_lifecycle.activate(
                proposal=proposal,
                approval=approval,
                now_s=float(self._clock_s()),
                cancel_event=cancellation,
            )
            if cancellation.is_set():
                self._deactivate(
                    "canonical session relocked after activation cancellation"
                )
                return
            self.service.transition_prompt_mission(mission_id, "queued")
            self.service.transition_prompt_mission(mission_id, "running")
            active_graph_capture = activation.get(
                "active_graph_capture", {}
            )
            if isinstance(active_graph_capture, Mapping) and (
                active_graph_capture
            ):
                self.service.record_hierarchical_checkpoint(
                    mission_id,
                    {
                        "source": "active_graph_audit",
                        "value": dict(active_graph_capture),
                        "received_at_s": float(self._clock_s()),
                    },
                )
            expires_at_s = float(approval["expires_at_s"])
            last_checkpoints: dict[str, str] = {}
            run_evidence: dict[str, Any] = {
                "started_at_s": float(self._clock_s()),
                "ended_at_s": None,
                "odom_samples": 0,
                "nonzero_odom_samples": 0,
                "max_displacement_m": 0.0,
                "max_localization_age_s": 0.0,
                "localization_freshness_violations": 0,
                "max_required_sensor_age_s": {
                    source_name: 0.0
                    for source_name in ACTIVE_SENSOR_MAX_AGE_S
                },
                "max_required_sensor_source_age_s": {
                    source_name: 0.0
                    for source_name in ACTIVE_SENSOR_MAX_AGE_S
                },
                "required_sensor_freshness_violations": 0,
                "planning_sensor_freshness_observations": 0,
                "sensor_freshness_hold_observations": {
                    source_name: 0
                    for source_name in ACTIVE_SENSOR_MAX_AGE_S
                },
            }
            origin: Optional[tuple[float, float]] = None
            while not self._closed:
                if cancellation.is_set():
                    return
                now_s = float(self._clock_s())
                if now_s >= expires_at_s:
                    self._finish(
                        mission_id,
                        status="timeout",
                        reason="canonical M7.6 mission lease expired",
                        run_evidence=run_evidence,
                    )
                    return
                session_status = dict(self.session_lifecycle.status())
                if not bool(session_status.get("active", False)):
                    self._finish(
                        mission_id,
                        status="recovery_required",
                        reason=(
                            "canonical hierarchical graph exited while "
                            "the mission was active: "
                            + str(
                                session_status.get(
                                    "detail", "unit is not active"
                                )
                            )
                        ),
                        run_evidence=run_evidence,
                    )
                    return
                live_snapshot = self.cache.snapshot(now_s=now_s)
                odom = live_snapshot.source("odom")
                motion_observed = False
                if odom.valid and odom.received_at_s is not None:
                    try:
                        x_m = float(odom.value["x_m"])
                        y_m = float(odom.value["y_m"])
                        linear_mps = float(
                            odom.value.get("linear_mps", 0.0)
                        )
                        angular_rad_s = float(
                            odom.value.get("angular_rad_s", 0.0)
                        )
                        values = (
                            x_m,
                            y_m,
                            linear_mps,
                            angular_rad_s,
                        )
                        if all(math.isfinite(item) for item in values):
                            if origin is None:
                                origin = (x_m, y_m)
                            run_evidence["odom_samples"] += 1
                            if (
                                abs(linear_mps) > 0.005
                                or abs(angular_rad_s) > 0.01
                            ):
                                motion_observed = True
                                run_evidence[
                                    "nonzero_odom_samples"
                                ] += 1
                            run_evidence["max_displacement_m"] = max(
                                float(
                                    run_evidence[
                                        "max_displacement_m"
                                    ]
                                ),
                                math.hypot(
                                    x_m - origin[0],
                                    y_m - origin[1],
                                ),
                            )
                    except (KeyError, TypeError, ValueError):
                        pass
                controller = live_snapshot.source(CONTROLLER_SOURCE)
                adapter = live_snapshot.source(ADAPTER_SOURCE)
                adapter_goal_active = (
                    adapter.valid
                    and bool(adapter.value.get("goal_active", False))
                )
                if adapter_goal_active or motion_observed:
                    for source_name, max_age_s in (
                        ACTIVE_SENSOR_MAX_AGE_S.items()
                    ):
                        record = live_snapshot.source(source_name)
                        unavailable = (
                            not record.valid
                            or record.received_at_s is None
                        )
                        age_s = (
                            float("inf")
                            if unavailable
                            else now_s - float(record.received_at_s)
                        )
                        source_age_s = (
                            float("inf")
                            if unavailable
                            or record.source_timestamp_s is None
                            else now_s
                            - float(record.source_timestamp_s)
                        )
                        if math.isfinite(age_s):
                            maxima = run_evidence[
                                "max_required_sensor_age_s"
                            ]
                            maxima[source_name] = max(
                                float(maxima[source_name]),
                                max(0.0, age_s),
                            )
                            if source_name == "localization":
                                run_evidence[
                                    "max_localization_age_s"
                                ] = max(
                                    float(
                                        run_evidence[
                                            "max_localization_age_s"
                                        ]
                                    ),
                                    max(0.0, age_s),
                                )
                        if math.isfinite(source_age_s):
                            source_maxima = run_evidence[
                                "max_required_sensor_source_age_s"
                            ]
                            source_maxima[source_name] = max(
                                float(source_maxima[source_name]),
                                max(0.0, source_age_s),
                            )
                        value = (
                            {}
                            if unavailable
                            else dict(record.value)
                        )
                        invalid = (
                            unavailable
                            or not math.isfinite(age_s)
                            or age_s < 0.0
                            or age_s > max_age_s
                            or not math.isfinite(source_age_s)
                            or source_age_s < 0.0
                            or source_age_s > max_age_s
                            or value.get("motion_authority")
                            is not False
                            or value.get(
                                "physical_execution_enabled"
                            )
                            is not False
                        )
                        if not invalid:
                            continue
                        hold_counts = run_evidence[
                            "sensor_freshness_hold_observations"
                        ]
                        hold_counts[source_name] = (
                            int(hold_counts[source_name]) + 1
                        )
                        run_evidence[
                            "planning_sensor_freshness_observations"
                        ] += 1
                        if source_name == "localization":
                            run_evidence[
                                "localization_freshness_violations"
                            ] += 1
                        # This loop is an evidence observer, not a second
                        # motion supervisor.  The fixed collision supervisor
                        # already forces motor zero when scan receipts exceed
                        # 0.30 s. Planning/localization timing misses are
                        # observations, not a second route-cancellation
                        # authority. Record them without tearing down the
                        # graph and racing recovery.
                for source_name, record in (
                    (CONTROLLER_SOURCE, controller),
                    (ADAPTER_SOURCE, adapter),
                ):
                    if (
                        not record.valid
                        or record.received_at_s is None
                    ):
                        continue
                    value = dict(record.value)
                    checkpoint = canonical_digest(value)
                    if (
                        checkpoint
                        == last_checkpoints.get(source_name, "")
                    ):
                        continue
                    self.service.record_hierarchical_checkpoint(
                        mission_id,
                        {
                            "source": source_name,
                            "value": value,
                            "received_at_s": record.received_at_s,
                        },
                    )
                    last_checkpoints[source_name] = checkpoint
                if controller.valid and controller.received_at_s is not None:
                    value = dict(controller.value)
                    state = str(value.get("state", "")).lower()
                    if state == "complete":
                        self._finish(
                            mission_id,
                            status="complete",
                            reason=str(value.get("reason", "complete")),
                            run_evidence=run_evidence,
                        )
                        return
                    if state == "recovery_required":
                        self._finish(
                            mission_id,
                            status="recovery_required",
                            reason=str(
                                value.get(
                                    "reason",
                                    "hierarchical controller recovery required",
                                )
                            ),
                            run_evidence=run_evidence,
                        )
                        return
                time.sleep(self.monitor_period_s)
        except Exception as exc:
            self._finish(
                mission_id,
                status="recovery_required",
                reason=(
                    "canonical physical activation failed: "
                    f"{exc.__class__.__name__}: {exc}"
                ),
                run_evidence=locals().get("run_evidence"),
            )
        finally:
            with self._lock:
                self._threads.pop(mission_id, None)
                self._activation_cancel.pop(mission_id, None)
                if self._active_mission_id == mission_id:
                    self._active_mission_id = ""

    def _finish(
        self,
        mission_id: str,
        *,
        status: str,
        reason: str,
        run_evidence: Optional[Mapping[str, Any]] = None,
    ) -> None:
        relock_error = self._deactivate(
            f"canonical session relocked after {status}"
        )
        result = {
            "schema": "sphero_rvr.hierarchical_canonical_result.v1",
            "mission_id": mission_id,
            "status": status,
            "reason": reason,
            "source_sha": self.service.source_sha,
            "deployed_sha": self.service.deployed_sha,
            "cleanup_verified": relock_error == "",
            "run_evidence": {
                **dict(run_evidence or {}),
                "ended_at_s": float(self._clock_s()),
            },
            "hierarchical_live_evidence": {
                name: {
                    "value": dict(record.value),
                    "received_at_s": record.received_at_s,
                    "valid": record.valid,
                }
                for name, record in (
                    (
                        CONTROLLER_SOURCE,
                        self.cache.snapshot().source(
                            CONTROLLER_SOURCE
                        ),
                    ),
                    (
                        ADAPTER_SOURCE,
                        self.cache.snapshot().source(ADAPTER_SOURCE),
                    ),
                )
            },
            "motion_authority": False,
            "restart_resume_allowed": False,
        }
        terminal = status if not relock_error else "recovery_required"
        terminal_reason = (
            reason
            if not relock_error
            else f"{reason}; canonical cleanup failed: {relock_error}"
        )
        try:
            self.service.transition_prompt_mission(
                mission_id,
                terminal,
                reason=terminal_reason,
                result=result,
            )
        except MissionValidationError:
            pass

    def _deactivate(self, reason: str) -> str:
        try:
            self.session_lifecycle.deactivate(reason=reason)
        except Exception as exc:
            return f"{exc.__class__.__name__}: {exc}"
        return ""

    def _ensure_open(self) -> None:
        if self._closed:
            raise MissionValidationError(
                "canonical physical mission controller is closed"
            )
