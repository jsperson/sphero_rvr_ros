"""Disabled-by-default physical adaptive mission executor over the live-route seam.

This module is ROS-free.  It consumes the production node's receipt-time cache
and delegates one bounded movement intent at a time to ``RosLiveRouteExecutor``.
That transport publishes only a typed live-route request; ``live_route_runner``
owns ``/cmd_vel`` and the lidar collision supervisor remains the sole
``/cmd_vel_motor`` publisher.
"""

from __future__ import annotations

from concurrent.futures import Future, ThreadPoolExecutor
import math
import threading
import time
from typing import Any, Mapping, Optional, Protocol

from .live_mission_service import LiveStateCache, snapshot_evidence
from .live_route_runner import (
    LiveRouteRequest,
    RouteSegmentRequest,
)
from .mission_api import MissionValidationError
from .adaptive_mission_controller import (
    IntentExecutionResult,
    MovementDecision,
    ADAPTIVE_MISSION_SAFETY_POLICY,
    AdaptiveMissionIntent,
    AdaptiveMissionLimits,
    make_world_snapshot,
    validate_world_snapshot,
)

DEFAULT_MAX_TERMINAL_DISTANCE_ERROR_M = 0.03
DEFAULT_MAX_TERMINAL_ANGLE_ERROR_DEG = 10.0
DEFAULT_TRANSLATION_CLEARANCE_RESERVE_M = 0.40


class AdaptiveMissionRouteTransport(Protocol):
    def execute(self, request: LiveRouteRequest) -> Mapping[str, Any]: ...

    def cancel(self) -> bool: ...


def validate_physical_adaptive_mission_gate(
    *,
    enabled: bool,
    source_sha: str,
    deployed_sha: str,
    reviewed_sha: str,
    transport_available: bool,
) -> bool:
    """Require an exact reviewed deployment before constructing motion authority."""

    if not bool(enabled):
        return False
    source = str(source_sha).strip()
    deployed = str(deployed_sha).strip()
    reviewed = str(reviewed_sha).strip()
    if (
        not source
        or not deployed
        or not reviewed
        or source != deployed
        or source != reviewed
    ):
        raise MissionValidationError(
            "physical Adaptive mission requires identical source, deployed, and reviewed SHAs"
        )
    if not transport_available:
        raise MissionValidationError(
            "physical Adaptive mission requires the supervised live-route transport"
        )
    return True


class PhysicalAdaptiveMissionExecutor:
    """One-intent physical adapter with no direct ROS or motor command surface."""

    mode = "physical-supervised-live-route"
    motor_topic_publisher = "lidar_collision_stop_supervisor"

    def __init__(
        self,
        cache: LiveStateCache,
        *,
        source_sha: str,
        deployed_sha: str,
        reviewed_sha: str = "",
        execution_enabled: bool = False,
        transport: Optional[AdaptiveMissionRouteTransport] = None,
        limits: Optional[AdaptiveMissionLimits] = None,
        max_source_age_s: float = 0.30,
        max_perception_age_s: float = 1.0,
        cleanup_timeout_s: float = 3.0,
        max_terminal_distance_error_m: float = (
            DEFAULT_MAX_TERMINAL_DISTANCE_ERROR_M
        ),
        max_terminal_angle_error_deg: float = (
            DEFAULT_MAX_TERMINAL_ANGLE_ERROR_DEG
        ),
        translation_clearance_reserve_m: float = (
            DEFAULT_TRANSLATION_CLEARANCE_RESERVE_M
        ),
        now: Any = time.time,
    ) -> None:
        self.cache = cache
        self.source_sha = _required_sha(source_sha, "source_sha")
        self.deployed_sha = _required_sha(deployed_sha, "deployed_sha")
        self.transport = transport
        self.limits = limits or AdaptiveMissionLimits()
        self.max_source_age_s = float(max_source_age_s)
        self.max_perception_age_s = float(max_perception_age_s)
        self.cleanup_timeout_s = float(cleanup_timeout_s)
        self.max_terminal_distance_error_m = float(
            max_terminal_distance_error_m
        )
        self.max_terminal_angle_error_deg = float(
            max_terminal_angle_error_deg
        )
        self.translation_clearance_reserve_m = float(
            translation_clearance_reserve_m
        )
        self._now = now
        if (
            not math.isfinite(self.max_source_age_s)
            or self.max_source_age_s <= 0.0
        ):
            raise MissionValidationError(
                "physical adaptive mission source age must be positive and finite"
            )
        if (
            not math.isfinite(self.max_perception_age_s)
            or self.max_perception_age_s <= 0.0
        ):
            raise MissionValidationError(
                "physical Adaptive mission perception age must be positive and finite"
            )
        if (
            not math.isfinite(self.cleanup_timeout_s)
            or self.cleanup_timeout_s <= 0.0
        ):
            raise MissionValidationError(
                "physical Adaptive mission cleanup timeout must be positive and finite"
            )
        for name in (
            "max_terminal_distance_error_m",
            "max_terminal_angle_error_deg",
            "translation_clearance_reserve_m",
        ):
            value = float(getattr(self, name))
            if not math.isfinite(value) or value <= 0.0:
                raise MissionValidationError(
                    f"physical Adaptive mission {name} must be positive and finite"
                )
        self.execution_enabled = validate_physical_adaptive_mission_gate(
            enabled=execution_enabled,
            source_sha=self.source_sha,
            deployed_sha=self.deployed_sha,
            reviewed_sha=reviewed_sha,
            transport_available=transport is not None,
        )
        self._lock = threading.RLock()
        self._pool = ThreadPoolExecutor(
            max_workers=1, thread_name_prefix="adaptive-mission-live-route"
        )
        self._future: Optional[Future[Mapping[str, Any]]] = None
        self._mission_id = ""
        self._proposal_digest = ""
        self._approval_id = ""
        self._operator = ""
        self._intent_count = 0
        self._observation_count = 0
        self._cumulative_translation_m = 0.0
        self._cumulative_rotation_deg = 0.0
        self._last_execution: Optional[dict[str, Any]] = None

    @property
    def motion_authority(self) -> bool:
        return bool(self.execution_enabled and self._approval_id)

    def reset(self, mission_id: str) -> None:
        with self._lock:
            if self._future is not None and not self._future.done():
                raise MissionValidationError(
                    "cannot reset Adaptive mission while a physical intent is active"
                )
            self._mission_id = str(mission_id).strip()
            if not self._mission_id:
                raise MissionValidationError("physical adaptive mission id is required")
            self._proposal_digest = ""
            self._approval_id = ""
            self._operator = ""
            self._intent_count = 0
            self._observation_count = 0
            self._cumulative_translation_m = 0.0
            self._cumulative_rotation_deg = 0.0
            self._last_execution = None

    def bind_approval(
        self,
        *,
        proposal_digest: str,
        approval_id: str,
        operator: str,
    ) -> None:
        with self._lock:
            if not self.execution_enabled:
                raise MissionValidationError(
                    "physical Adaptive mission execution is disabled by deployment configuration"
                )
            digest = str(proposal_digest).strip().lower()
            approval = str(approval_id).strip()
            principal = str(operator).strip()
            if (
                len(digest) != 64
                or any(character not in "0123456789abcdef" for character in digest)
                or not approval
                or not principal
            ):
                raise MissionValidationError(
                    "physical adaptive mission approval binding is incomplete"
                )
            self._proposal_digest = digest
            self._approval_id = approval
            self._operator = principal

    def readiness(self) -> dict[str, Any]:
        snapshot = self.snapshot(self._mission_id or "adaptive-mission-readiness")
        evidence = snapshot["evidence"]
        safety = snapshot["safety"]
        observations = _mapping(snapshot.get("observations"))
        perception = _mapping(observations.get("perception"))
        source_receipts = _mapping(evidence.get("source_receipts"))
        planning_reasons = []
        if perception.get("camera_fresh") is not True:
            planning_reasons.append("camera")
        lidar = _mapping(source_receipts.get("lidar"))
        if lidar.get("fresh") is not True or lidar.get("valid") is not True:
            planning_reasons.append("lidar")
        if perception.get("localization_fresh") is not True:
            planning_reasons.append("localization")
        execution_reasons = []
        for field in ("scan_fresh", "transform_fresh", "odometry_fresh"):
            if evidence.get(field) is not True:
                execution_reasons.append(field)
        if str(safety.get("collision_state", "UNKNOWN")).upper() not in {
            "CLEAR",
            "SLOW",
        }:
            execution_reasons.append("collision_state")
        if safety.get("stop_active") is not False:
            execution_reasons.append("stop")
        if safety.get("estop_latched") is not False:
            execution_reasons.append("estop")
        reasons = list(planning_reasons)
        if self.execution_enabled:
            reasons.extend(execution_reasons)
        return {
            "ready": not reasons,
            "reasons": reasons,
            "planning_ready": not planning_reasons,
            "planning_reasons": planning_reasons,
            "execution_ready": not execution_reasons,
            "execution_reasons": execution_reasons,
            "execution_enabled": self.execution_enabled,
            "motion_authority": self.motion_authority,
            "source_sha": self.source_sha,
            "deployed_sha": self.deployed_sha,
            "command_path": [
                "AdaptiveMissionController",
                "RosLiveRouteExecutor",
                "live_route_runner",
                "/cmd_vel",
                "lidar_collision_stop_supervisor",
                "/cmd_vel_motor",
            ],
        }

    def snapshot(self, mission_id: str) -> Mapping[str, Any]:
        mission = str(mission_id).strip()
        if mission:
            self._mission_id = mission
        live = self.cache.snapshot(now_s=float(self._now()))
        evidence = snapshot_evidence(live, max_age_s=self.max_source_age_s)
        odom_record = evidence["odom"]
        collision_record = evidence["collision"]
        control_record = evidence["control"]
        odom = _mapping(odom_record.get("value"))
        collision = _mapping(collision_record.get("value"))
        control = _mapping(control_record.get("value"))
        scan_age = _optional_finite(collision.get("scan_age_s"))
        scan_fresh = bool(
            collision_record.get("fresh")
            and collision.get("scan_healthy") is True
            and scan_age is not None
            and scan_age <= self.max_source_age_s
        )
        transform_fresh = bool(
            collision_record.get("fresh")
            and collision.get("tf_available") is True
            and str(collision.get("tf_reason", "")).lower()
            in {
                "identity",
                "identity_same_frame",
                "available",
                "fresh",
                "ok",
                "",
            }
        )
        odometry_fresh = bool(odom_record.get("fresh"))
        collision_state = str(collision.get("state", "UNKNOWN")).upper()
        forward_clearance = _optional_finite(
            collision.get(
                "forward_corridor_clearance_m",
                collision.get("front_clearance_m"),
            )
        )
        rear_clearance = _optional_finite(
            collision.get("rear_clearance_m")
        )
        stop_active = _stop_active(control, collision_state)
        estop_latched = _estop_latched(control, collision_state)
        perception = _perception_observations(
            evidence,
            max_age_s=self.max_perception_age_s,
        )
        return make_world_snapshot(
            {
                "mission_id": self._mission_id or mission,
                "version": self._intent_count + 1,
                "observed_at_s": live.observed_at_s,
                "pose": {
                    "frame": str(odom.get("frame_id", "map")),
                    "x_m": odom.get("x_m"),
                    "y_m": odom.get("y_m"),
                    "yaw_deg": odom.get("heading_deg"),
                },
                "evidence": {
                    "scan_fresh": scan_fresh,
                    "transform_fresh": transform_fresh,
                    "odometry_fresh": odometry_fresh,
                    "localization_fresh": bool(
                        evidence["localization"].get("fresh", False)
                    ),
                    "scan_age_s": scan_age,
                    "odometry_age_s": odom_record.get("age_s"),
                    "transform_reason": collision.get(
                        "tf_reason", "unavailable"
                    ),
                    "source_receipts": {
                        name: {
                            "fresh": record.get("fresh", False),
                            "valid": record.get("valid", False),
                            "received_at_s": record.get("received_at_s"),
                        }
                        for name, record in evidence.items()
                        if isinstance(record, Mapping)
                    },
                    "drop_off_detection_available": False,
                },
                "safety": {
                    "collision_state": collision_state,
                    "stop_active": stop_active,
                    "estop_latched": estop_latched,
                    "supervisor": ADAPTIVE_MISSION_SAFETY_POLICY,
                    "control_state": control.get("state", "UNKNOWN"),
                },
                "execution": {
                    "mode": self.mode,
                    "motion_permitted": self.execution_enabled,
                    "physical_execution_enabled": self.execution_enabled,
                    "motion_authority": self.motion_authority,
                    "approval_bound": bool(self._approval_id),
                    "motor_topic_publisher": self.motor_topic_publisher,
                },
                "progress": {
                    "intent_count": self._intent_count,
                    "observation_count": self._observation_count,
                    "cumulative_translation_m": self._cumulative_translation_m,
                    "cumulative_rotation_deg": self._cumulative_rotation_deg,
                    "cumulative_limits": (
                        "unlimited within approved mission lease"
                    ),
                },
                "observations": {
                    "forward_clearance_m": forward_clearance,
                    "left_clearance_m": collision.get("left_clearance_m"),
                    "right_clearance_m": collision.get("right_clearance_m"),
                    "motion_clearance": {
                        "translation_reserve_m": (
                            self.translation_clearance_reserve_m
                        ),
                        "forward_usable_m": (
                            max(
                                0.0,
                                forward_clearance
                                - self.translation_clearance_reserve_m,
                            )
                            if forward_clearance is not None
                            else None
                        ),
                        "reverse_usable_m": (
                            max(
                                0.0,
                                rear_clearance
                                - self.translation_clearance_reserve_m,
                            )
                            if rear_clearance is not None
                            else None
                        ),
                    },
                    **perception,
                    "coverage_note": (
                        (
                            "Authoritative live geometry and fresh camera, "
                            "localization, and semantic-track evidence snapshot."
                            if perception["perception"]["available"]
                            else "Authoritative live geometry snapshot; semantic "
                            "recognition is unavailable or stale."
                        )
                        if scan_fresh and transform_fresh and odometry_fresh
                        else "Live evidence is incomplete or stale; movement is vetoed."
                    ),
                },
                "last_execution": self._last_execution,
            }
        )

    def execute(
        self, intent: AdaptiveMissionIntent, cancellation: threading.Event
    ) -> IntentExecutionResult:
        before = self.snapshot(self._mission_id)
        started = time.monotonic()
        deadline = started + intent.timeout_s
        if cancellation.is_set():
            return self._nonmotion_result(
                intent, "cancelled", "operator_cancelled", before
            )
        if intent.action == "observe":
            updated, reason = self._wait_for_updated_perception(
                before,
                cancellation,
                deadline=deadline,
            )
            if reason:
                return self._nonmotion_result(
                    intent,
                    "cancelled" if reason == "operator_cancelled" else "blocked",
                    reason,
                    updated,
                )
            self._observation_count += 1
            self._intent_count += 1
            return self._nonmotion_result(
                intent,
                "completed",
                "observation_completed",
                updated,
            )
        if intent.action == "stop":
            self._intent_count += 1
            return self._nonmotion_result(
                intent,
                "completed",
                "planner_stop",
                self.snapshot(self._mission_id),
            )
        if not self.motion_authority or self.transport is None:
            return self._nonmotion_result(
                intent, "blocked", "physical_authority_disabled", before
            )
        try:
            validate_world_snapshot(
                before,
                mission_id=self._mission_id,
                require_motion=True,
                allow_supervised_collision_escape=(
                    _is_supervised_collision_escape(intent)
                ),
            )
        except MissionValidationError as exc:
            return self._nonmotion_result(
                intent,
                "blocked",
                f"stale_or_unsafe_evidence: {exc}",
                before,
            )

        request = self._route_request(intent)
        future = self._pool.submit(self.transport.execute, request)
        with self._lock:
            self._future = future
        cancel_attempted = False
        cancel_reason = ""
        cleanup_deadline = math.inf
        try:
            while not future.done():
                now = time.monotonic()
                if cancellation.is_set() and not cancel_attempted:
                    cancel_attempted = True
                    cancel_reason = "operator_cancelled"
                    cleanup_deadline = now + self.cleanup_timeout_s
                    if not self.transport.cancel():
                        cancel_reason = "cleanup_uncertain"
                elif now >= deadline and not cancel_attempted:
                    cancel_attempted = True
                    cancel_reason = "intent_timeout"
                    cleanup_deadline = now + self.cleanup_timeout_s
                    if not self.transport.cancel():
                        cancel_reason = "cleanup_uncertain"
                if cancel_attempted and now >= cleanup_deadline:
                    return self._nonmotion_result(
                        intent,
                        "failed",
                        "cleanup_uncertain",
                        self.snapshot(self._mission_id),
                    )
                cancellation.wait(0.02)
            result = dict(future.result())
        except Exception as exc:
            return self._nonmotion_result(
                intent,
                "failed",
                f"cleanup_uncertain: route_transport_failure: {exc.__class__.__name__}",
                self.snapshot(self._mission_id),
            )
        finally:
            with self._lock:
                self._future = None
        terminal = self._terminal_result(intent, request, result, started)
        if cancel_reason == "operator_cancelled" and terminal.outcome == "cancelled":
            return terminal
        if cancel_reason == "intent_timeout" and terminal.outcome in {
            "cancelled",
            "timeout",
        }:
            return IntentExecutionResult(
                outcome="timeout",
                reason="intent_timeout",
                snapshot=terminal.snapshot,
                movement=terminal.movement,
                duration_s=terminal.duration_s,
            )
        if cancel_reason == "cleanup_uncertain":
            return self._nonmotion_result(
                intent,
                "failed",
                "cleanup_uncertain",
                terminal.snapshot,
            )
        if terminal.outcome == "completed":
            updated, reason = self._wait_for_updated_perception(
                before,
                cancellation,
                deadline=deadline,
            )
            if reason:
                return IntentExecutionResult(
                    outcome=(
                        "cancelled"
                        if reason == "operator_cancelled"
                        else "blocked"
                    ),
                    reason=reason,
                    snapshot=updated,
                    movement=terminal.movement,
                    duration_s=max(0.0, time.monotonic() - started),
                )
            return IntentExecutionResult(
                outcome=terminal.outcome,
                reason=terminal.reason,
                snapshot=updated,
                movement=terminal.movement,
                duration_s=max(0.0, time.monotonic() - started),
            )
        return terminal

    def _wait_for_updated_perception(
        self,
        previous: Mapping[str, Any],
        cancellation: threading.Event,
        *,
        deadline: float,
    ) -> tuple[Mapping[str, Any], str]:
        """Wait within the intent timeout for a newer typed sensor cycle."""

        previous_evidence = _mapping(previous.get("evidence"))
        previous_receipts = _mapping(
            previous_evidence.get("source_receipts")
        )
        latest = self.snapshot(self._mission_id)
        while True:
            latest_evidence = _mapping(latest.get("evidence"))
            latest_receipts = _mapping(
                latest_evidence.get("source_receipts")
            )
            observations = _mapping(latest.get("observations"))
            perception = _mapping(observations.get("perception"))
            updated = True
            for name in ("camera", "lidar", "localization"):
                prior = _mapping(previous_receipts.get(name))
                current = _mapping(latest_receipts.get(name))
                prior_at = _optional_finite(prior.get("received_at_s"))
                current_at = _optional_finite(current.get("received_at_s"))
                if (
                    current.get("valid") is not True
                    or (
                        name != "camera"
                        and current.get("fresh") is not True
                    )
                    or current_at is None
                    or (
                        prior_at is not None
                        and current_at <= prior_at
                    )
                ):
                    updated = False
                    break
            if (
                updated
                and perception.get("camera_fresh") is True
                and perception.get("localization_fresh") is True
            ):
                return latest, ""
            if cancellation.is_set():
                return latest, "operator_cancelled"
            if time.monotonic() >= deadline:
                return latest, "updated_perception_timeout"
            cancellation.wait(0.02)
            latest = self.snapshot(self._mission_id)

    def cancel(self) -> bool:
        with self._lock:
            active = self._future is not None and not self._future.done()
        if not active or self.transport is None:
            return False
        return bool(self.transport.cancel())

    def close(self) -> None:
        self.cancel()
        self._pool.shutdown(wait=False, cancel_futures=True)

    def map_projection(self) -> dict[str, Any]:
        snapshot = self.snapshot(self._mission_id or "adaptive-mission-map")
        pose = snapshot.get("pose", {})
        return {
            "available": all(
                _optional_finite(pose.get(name)) is not None
                for name in ("x_m", "y_m", "yaw_deg")
            ),
            "fixture_only": False,
            "source": "authoritative-live-adaptive-mission-snapshot",
            "frame": pose.get("frame", "map"),
            "rover": dict(pose),
            "goal_region": None,
            "proposed_route": [],
            "traveled_path": [],
            "obstacles": [],
            "objects": [],
            "localization": {
                "state": (
                    "valid"
                    if snapshot["evidence"].get("localization_fresh")
                    else "unavailable"
                ),
                "fresh": bool(
                    snapshot["evidence"].get("localization_fresh")
                ),
            },
        }

    def _route_request(self, intent: AdaptiveMissionIntent) -> LiveRouteRequest:
        correlation = (
            f"{self._mission_id}:adaptive-mission-intent:{intent.revision}"
        )
        if intent.action == "move_distance":
            arguments = {
                "distance_m": intent.distance_m,
                "speed_mps": self.limits.linear_speed_mps,
                "timeout_s": intent.timeout_s,
            }
        else:
            arguments = {
                "angle_deg": intent.angle_deg,
                "angular_speed_deg_s": math.degrees(
                    self.limits.angular_speed_rad_s
                ),
                "timeout_s": intent.timeout_s,
            }
        return LiveRouteRequest(
            route_id=correlation,
            segments=(
                RouteSegmentRequest(
                    correlation_id=correlation,
                    tool_id=intent.action,
                    arguments=arguments,
                ),
            ),
            max_runtime_s=intent.timeout_s,
            max_travel_m=max(
                abs(intent.distance_m),
                self.limits.max_translation_per_intent_m,
            ),
            source_sha=self.source_sha,
            approval_id=self._approval_id,
            supervised_collision_escape=(
                _is_supervised_collision_escape(intent)
            ),
        )

    def _terminal_result(
        self,
        intent: AdaptiveMissionIntent,
        request: LiveRouteRequest,
        result: Mapping[str, Any],
        started: float,
    ) -> IntentExecutionResult:
        reason = str(
            result.get("terminal_reason") or result.get("reason") or ""
        )
        status = str(result.get("status", "failed")).lower()
        supervision = _mapping(result.get("supervision"))
        movement = _movement_from_supervision(intent, supervision, self.limits)
        samples = _nonnegative_int(supervision.get("samples"))
        if status not in {
            "complete",
            "failed",
            "blocked",
            "cancelled",
            "stopped",
            "estopped",
            "timeout",
        }:
            status = "failed"
            reason = "terminal_status_invalid"
            movement = _zeroed_movement(movement, reason)
        if (
            str(result.get("route_id", "")) != request.route_id
            or str(result.get("source_sha", "")) != self.source_sha
        ):
            status = "failed"
            reason = "terminal_correlation_mismatch"
            movement = _zeroed_movement(
                movement, "terminal_correlation_mismatch"
            )
        if status in {
            "complete",
            "blocked",
            "cancelled",
            "stopped",
            "estopped",
            "timeout",
        } and (
            result.get("terminal_settled") is not True
            or samples < 1
        ):
            status = "failed"
            reason = "terminal_evidence_incomplete"
            movement = _zeroed_movement(
                movement, "terminal_evidence_incomplete"
            )
        if status == "failed" and result.get("terminal_settled") is not True:
            reason = "cleanup_uncertain"
            movement = _zeroed_movement(movement, reason)
        executed = result.get("executed_segments", [])
        if status == "complete":
            measured_distance = _optional_finite(
                result.get("measured_distance_m")
            )
            measured_rotation = _optional_finite(
                result.get("measured_angle_deg")
            )
            executed_segment = (
                _mapping(executed[0])
                if isinstance(executed, list) and len(executed) == 1
                else {}
            )
            if (
                not executed_segment
                or str(executed_segment.get("correlation_id", ""))
                != request.segments[0].correlation_id
                or str(executed_segment.get("status", "")).lower()
                != "complete"
                or measured_distance is None
                or measured_rotation is None
                or not self._terminal_motion_within_limits(
                    intent,
                    executed_segment,
                    measured_distance=measured_distance,
                    measured_rotation=measured_rotation,
                )
                or not _movement_within_limits(movement, self.limits)
            ):
                status = "failed"
                reason = "terminal_evidence_incomplete"
                movement = _zeroed_movement(
                    movement, "terminal_evidence_incomplete"
                )
        outcome = {
            "complete": "completed",
            "blocked": "blocked",
            "cancelled": "cancelled",
            "stopped": "cancelled",
            "estopped": "blocked",
            "timeout": "timeout",
        }.get(status, "failed")
        if outcome == "completed":
            measured_distance = abs(
                _optional_finite(result.get("measured_distance_m")) or 0.0
            )
            measured_rotation = abs(
                _optional_finite(result.get("measured_angle_deg")) or 0.0
            )
            self._cumulative_translation_m += measured_distance
            self._cumulative_rotation_deg += measured_rotation
            self._intent_count += 1
        self._last_execution = {
            "intent": intent.to_json_dict(),
            "route_terminal": dict(result),
            "movement": movement.to_json_dict(),
        }
        snapshot = self.snapshot(self._mission_id)
        return IntentExecutionResult(
            outcome=outcome,
            reason=reason or status,
            snapshot=snapshot,
            movement=movement,
            duration_s=max(0.0, time.monotonic() - started),
        )

    def _terminal_motion_within_limits(
        self,
        intent: AdaptiveMissionIntent,
        executed_segment: Mapping[str, Any],
        *,
        measured_distance: float,
        measured_rotation: float,
    ) -> bool:
        """Accept only settled motion within the runner's bounded stop tolerance."""

        epsilon = 1e-9
        if intent.action == "move_distance":
            terminal_error = _optional_finite(
                executed_segment.get("terminal_distance_error_m")
            )
            return bool(
                terminal_error is not None
                and terminal_error
                <= self.max_terminal_distance_error_m + epsilon
                and abs(measured_distance)
                <= (
                    self.limits.max_translation_per_intent_m
                    + self.max_terminal_distance_error_m
                    + epsilon
                )
                and abs(measured_rotation)
                <= self.limits.max_rotation_per_intent_deg + epsilon
            )
        terminal_error = _optional_finite(
            executed_segment.get("terminal_angle_error_deg")
        )
        return bool(
            terminal_error is not None
            and terminal_error
            <= self.max_terminal_angle_error_deg + epsilon
            and abs(measured_rotation)
            <= (
                self.limits.max_rotation_per_intent_deg
                + self.max_terminal_angle_error_deg
                + epsilon
            )
            and abs(measured_distance)
            <= self.limits.max_translation_per_intent_m + epsilon
        )

    def _nonmotion_result(
        self,
        intent: AdaptiveMissionIntent,
        outcome: str,
        reason: str,
        snapshot: Mapping[str, Any],
    ) -> IntentExecutionResult:
        movement = MovementDecision(
            outcome=(
                "allowed" if outcome == "completed" else outcome
            ),
            reason=reason,
            requested_linear_mps=0.0,
            requested_angular_rad_s=0.0,
            supervised_linear_mps=0.0,
            supervised_angular_rad_s=0.0,
            collision_state=str(
                snapshot.get("safety", {}).get(
                    "collision_state", "UNKNOWN"
                )
            ),
        )
        self._last_execution = {
            "intent": intent.to_json_dict(),
            "movement": movement.to_json_dict(),
        }
        updated_snapshot = self.snapshot(self._mission_id)
        return IntentExecutionResult(
            outcome=outcome,
            reason=reason,
            snapshot=updated_snapshot,
            movement=movement,
            duration_s=0.0,
        )


def _movement_from_supervision(
    intent: AdaptiveMissionIntent,
    supervision: Mapping[str, Any],
    limits: AdaptiveMissionLimits,
) -> MovementDecision:
    requested = _mapping(supervision.get("requested"))
    supervised = _mapping(supervision.get("supervised"))
    expected_linear = (
        math.copysign(limits.linear_speed_mps, intent.distance_m)
        if intent.action == "move_distance"
        else 0.0
    )
    expected_angular = (
        math.copysign(limits.angular_speed_rad_s, intent.angle_deg)
        if intent.action == "turn_angle"
        else 0.0
    )
    requested_linear = _optional_finite(
        requested.get("linear_mps", expected_linear)
    )
    requested_angular = _optional_finite(
        requested.get("angular_rad_s", expected_angular)
    )
    supervised_linear = _optional_finite(supervised.get("linear_mps", 0.0))
    supervised_angular = _optional_finite(
        supervised.get("angular_rad_s", 0.0)
    )
    return MovementDecision(
        outcome="allowed",
        reason="collision_supervised",
        requested_linear_mps=(
            expected_linear if requested_linear is None else requested_linear
        ),
        requested_angular_rad_s=(
            expected_angular if requested_angular is None else requested_angular
        ),
        supervised_linear_mps=(
            0.0 if supervised_linear is None else supervised_linear
        ),
        supervised_angular_rad_s=(
            0.0 if supervised_angular is None else supervised_angular
        ),
        collision_state=str(
            supervision.get("collision_state", "UNKNOWN")
        ),
    )


def _zeroed_movement(
    movement: MovementDecision, reason: str
) -> MovementDecision:
    return MovementDecision(
        outcome="blocked",
        reason=reason,
        requested_linear_mps=movement.requested_linear_mps,
        requested_angular_rad_s=movement.requested_angular_rad_s,
        supervised_linear_mps=0.0,
        supervised_angular_rad_s=0.0,
        collision_state=movement.collision_state,
    )


def _mapping(value: Any) -> dict[str, Any]:
    return dict(value) if isinstance(value, Mapping) else {}


def _optional_finite(value: Any) -> Optional[float]:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _nonnegative_int(value: Any) -> int:
    try:
        number = int(value)
    except (TypeError, ValueError, OverflowError):
        return 0
    return max(0, number)


def _movement_within_limits(
    movement: MovementDecision, limits: AdaptiveMissionLimits
) -> bool:
    epsilon = 1e-9
    return bool(
        abs(movement.requested_linear_mps)
        <= limits.linear_speed_mps + epsilon
        and abs(movement.supervised_linear_mps)
        <= limits.linear_speed_mps + epsilon
        and abs(movement.requested_angular_rad_s)
        <= limits.angular_speed_rad_s + epsilon
        and abs(movement.supervised_angular_rad_s)
        <= limits.angular_speed_rad_s + epsilon
    )


def _required_sha(value: str, name: str) -> str:
    result = str(value).strip()
    if not result or result.lower() == "unknown":
        raise MissionValidationError(f"physical Adaptive mission {name} is required")
    return result


def _stop_active(control: Mapping[str, Any], collision_state: str) -> bool:
    state = str(control.get("state", "")).upper()
    return bool(
        control.get("stop_active", False)
        or state in {"STOP", "STOPPED"}
        or collision_state in {"STOP", "STOPPED"}
    )


def _is_supervised_collision_escape(intent: AdaptiveMissionIntent) -> bool:
    return bool(
        intent.provider_id == "deterministic-supervised-recovery"
        and intent.action == "move_distance"
        and intent.distance_m < 0.0
    )


def _estop_latched(
    control: Mapping[str, Any], collision_state: str
) -> bool:
    state = str(control.get("state", "")).upper()
    return bool(
        control.get("estop_latched", False)
        or state in {"ESTOP", "ESTOPPED", "LATCHED"}
        or collision_state in {"ESTOP", "ESTOPPED", "LATCHED"}
    )


def _perception_observations(
    evidence: Mapping[str, Any],
    *,
    max_age_s: float,
) -> dict[str, Any]:
    """Project only fresh, localized semantic evidence into the LLM snapshot.

    Camera detections may be shown without localization, but map tracks are
    withheld unless camera, semantic-map, and localization receipts are all
    fresh.  Face names survive only with explicit enrollment evidence.
    """

    camera_record = _mapping(evidence.get("camera"))
    semantic_record = _mapping(evidence.get("semantic_map"))
    localization_record = _mapping(evidence.get("localization"))
    camera = _mapping(camera_record.get("value"))
    semantic = _mapping(semantic_record.get("value"))
    localization = _mapping(localization_record.get("value"))
    if isinstance(localization.get("localization"), Mapping):
        localization = _mapping(localization.get("localization"))
    localization_state = str(
        localization.get("state", "unknown")
    ).strip().lower()
    camera_fresh = _record_fresh(camera_record, max_age_s=max_age_s)
    semantic_fresh = _record_fresh(semantic_record, max_age_s=max_age_s)
    localization_fresh = bool(
        _record_fresh(localization_record, max_age_s=max_age_s)
        and localization_state in {"valid", "degraded"}
    )
    localized_semantics = bool(
        camera_fresh and semantic_fresh and localization_fresh
    )

    raw_detections = camera.get("detections", [])
    detections = (
        [
            normalized
            for item in raw_detections[:64]
            if isinstance(item, Mapping)
            for normalized in [_normalized_semantic_item(item)]
            if normalized
        ]
        if camera_fresh and isinstance(raw_detections, list)
        else []
    )
    raw_tracks = semantic.get("tracks", [])
    tracks = (
        [
            normalized
            for item in raw_tracks[:64]
            if isinstance(item, Mapping)
            and _semantic_item_fresh(
                item,
                observed_at_s=evidence.get("observed_at_s"),
                max_age_s=max_age_s,
            )
            for normalized in [_normalized_semantic_item(item)]
            if normalized
        ]
        if localized_semantics and isinstance(raw_tracks, list)
        else []
    )
    if localized_semantics and not tracks:
        raw_objects = semantic.get("objects")
        if not isinstance(raw_objects, list):
            semantic_map = _mapping(semantic.get("map"))
            raw_objects = semantic_map.get("objects", [])
        if isinstance(raw_objects, list):
            tracks = [
                normalized
                for item in raw_objects[:64]
                if isinstance(item, Mapping)
                and _semantic_item_fresh(
                    item,
                    observed_at_s=evidence.get("observed_at_s"),
                    max_age_s=max_age_s,
                )
                for normalized in [_normalized_semantic_item(item)]
                if normalized
            ]

    recognized_faces = [
        item
        for item in tracks
        if item.get("kind") == "face"
        and item.get("recognized_from_enrollment") is True
    ]
    unknown_faces = [
        item
        for item in tracks
        if item.get("kind") == "face"
        and item.get("recognized_from_enrollment") is not True
    ]
    object_tracks = [
        item for item in tracks if item.get("kind") != "face"
    ]
    return {
        "camera_detections": detections,
        "semantic_tracks": tracks,
        "recognized_objects": object_tracks,
        "recognized_faces": recognized_faces,
        "unknown_faces": unknown_faces,
        "perception": {
            "available": localized_semantics,
            "camera_fresh": camera_fresh,
            "semantic_map_fresh": semantic_fresh,
            "localization_fresh": localization_fresh,
            "localization_state": localization_state,
            "camera_frame_id": camera.get("frame_id"),
            "semantic_map_revision": semantic.get("revision"),
            "uncertain_track_id": str(
                camera.get("uncertain_track_id")
                or semantic.get("uncertain_track_id")
                or ""
            ),
            "identity_policy": (
                "face labels are authoritative only with explicit enrollment evidence"
            ),
        },
    }


def _normalized_semantic_item(item: Mapping[str, Any]) -> dict[str, Any]:
    kind = str(item.get("kind", "object")).strip().lower()
    if kind not in {"object", "face"}:
        kind = "object"
    track_id = str(
        item.get("track_id") or item.get("object_id") or item.get("detection_id") or ""
    ).strip()
    if not track_id:
        return {}
    label = str(item.get("label", "unknown")).strip() or "unknown"
    enrollment_ids = [
        str(value)
        for value in item.get("enrollment_evidence_ids", [])
        if str(value).strip()
    ] if isinstance(item.get("enrollment_evidence_ids", []), list) else []
    recognized = bool(
        kind == "face"
        and item.get("recognized_from_enrollment") is True
        and enrollment_ids
        and label.lower() != "unknown"
    )
    if kind == "face" and not recognized:
        label = "unknown"
        enrollment_ids = []
    normalized: dict[str, Any] = {
        "track_id": track_id,
        "kind": kind,
        "label": label,
        "recognized_from_enrollment": recognized,
        "enrollment_evidence_ids": enrollment_ids,
    }
    for name in (
        "confidence",
        "x_m",
        "y_m",
        "uncertainty_m",
        "last_seen_s",
    ):
        value = _optional_finite(item.get(name))
        if value is not None:
            normalized[name] = value
    for name in ("status", "evidence_ref"):
        value = str(item.get(name, "")).strip()
        if value:
            normalized[name] = value
    evidence_ids = item.get("evidence_ids", [])
    if isinstance(evidence_ids, list):
        normalized["evidence_ids"] = [
            str(value) for value in evidence_ids[-12:] if str(value).strip()
        ]
    try:
        observation_count = int(item.get("observation_count", 0))
    except (TypeError, ValueError):
        observation_count = 0
    if observation_count > 0:
        normalized["observation_count"] = observation_count
    return normalized


def _record_fresh(
    record: Mapping[str, Any],
    *,
    max_age_s: float,
) -> bool:
    age = _optional_finite(record.get("age_s"))
    return bool(
        record.get("valid")
        and age is not None
        and 0.0 <= age <= float(max_age_s)
    )


def _semantic_item_fresh(
    item: Mapping[str, Any],
    *,
    observed_at_s: Any,
    max_age_s: float,
) -> bool:
    observed = _optional_finite(observed_at_s)
    last_seen = _optional_finite(item.get("last_seen_s"))
    return bool(
        observed is not None
        and last_seen is not None
        and 0.0 <= observed - last_seen <= float(max_age_s)
    )
