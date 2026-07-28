"""Fail-closed authority and Nav2 binding for Milestone 7 Phase 4.

The model-facing semantic-goal controller remains ROS-free.  This module is
the deterministic boundary that converts already-resolved, revalidated goals
into ``NavigateThroughPoses`` requests and supplies the short-lived authority
heartbeat consumed by ``live_route_runner``.  It deliberately contains no
Twist publisher, serial access, or hardware API.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import hashlib
import json
import math
from pathlib import Path
import re
import sqlite3
import threading
import time
from typing import Any, Mapping, Optional, Sequence

from .hierarchical_goal_selection import (
    AsyncSemanticGoalController,
    DeterministicGoalResolver,
    ResolvedSemanticGoal,
    SemanticGoalDecision,
    revalidate_resolved_goal,
)
from .mission_api import MissionValidationError


AUTHORITY_SCHEMA = "sphero_rvr.hierarchical_physical_authority.v1"
APPROVAL_SCHEMA = "sphero_rvr.hierarchical_m7_6_approval.v1"
JOURNAL_SCHEMA = "sphero_rvr.hierarchical_physical_journal.v1"
NAV2_BATCH_SCHEMA = "sphero_rvr.hierarchical_nav2_goal_batch.v1"
GOAL_DISPATCH_SCHEMA = "sphero_rvr.hierarchical_goal_dispatch.v1"

AUTHORITY_TOPIC = "/mission_api/v2/hierarchical/authority"
GOAL_DISPATCH_TOPIC = "/mission_api/v2/hierarchical/goal_dispatch"
NAV2_ACTION = "/navigate_through_poses"
PRIVATE_NAV2_CMD_TOPIC = "/nav2_cmd_vel_request"
SUPERVISOR_REQUEST_TOPIC = "/cmd_vel"
MOTOR_TOPIC = "/cmd_vel_motor"

MAX_LINEAR_MPS = 0.10
MAX_ANGULAR_RAD_S = 0.4
COMMAND_LEASE_S = 0.50
AUTHORITY_HEARTBEAT_MAX_AGE_S = 0.30
LOCALIZATION_MAX_AGE_S = 0.30
MISSION_LEASE_MAX_S = 900.0

ACCEPTED_M7_3_EVIDENCE_SHA256 = (
    "7e2636f100ffad724477f1e6287458d0708057c3ee93f26d5dd6f52432281f55"
)
ACCEPTED_DIRECTIONAL_ADDENDUM_SHA256 = (
    "638abb8f293781adcf3827a486cf700b96693f1172d6fb058c40b79a8b8f4130"
)
ACCEPTED_M7_4_EVIDENCE_SHA256 = (
    "35e9c25d06b113d335775a54f16211a5887b8c15f4527d9cca975fdcc79012da"
)

_SHA1_RE = re.compile(r"^[0-9a-f]{40}$")
_SHA256_RE = re.compile(r"^[0-9a-f]{64}$")


def canonical_digest(value: Mapping[str, Any]) -> str:
    rendered = json.dumps(
        dict(value),
        sort_keys=True,
        separators=(",", ":"),
        allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(rendered).hexdigest()


def _sha1(value: Any, name: str) -> str:
    parsed = str(value).strip()
    if not _SHA1_RE.fullmatch(parsed):
        raise MissionValidationError(f"{name} must be an exact lowercase Git SHA")
    return parsed


def _sha256(value: Any, name: str) -> str:
    parsed = str(value).strip()
    if not _SHA256_RE.fullmatch(parsed):
        raise MissionValidationError(f"{name} must be an exact SHA-256 digest")
    return parsed


def _finite(value: Any, name: str) -> float:
    try:
        parsed = float(value)
    except (TypeError, ValueError) as exc:
        raise MissionValidationError(f"{name} must be finite") from exc
    if not math.isfinite(parsed):
        raise MissionValidationError(f"{name} must be finite")
    return parsed


@dataclass(frozen=True)
class HierarchicalPhysicalLimits:
    max_linear_mps: float = MAX_LINEAR_MPS
    max_angular_rad_s: float = MAX_ANGULAR_RAD_S
    command_lease_s: float = COMMAND_LEASE_S
    localization_max_age_s: float = LOCALIZATION_MAX_AGE_S
    mission_lease_max_s: float = MISSION_LEASE_MAX_S

    def __post_init__(self) -> None:
        exact = {
            "max_linear_mps": MAX_LINEAR_MPS,
            "max_angular_rad_s": MAX_ANGULAR_RAD_S,
            "command_lease_s": COMMAND_LEASE_S,
            "localization_max_age_s": LOCALIZATION_MAX_AGE_S,
            "mission_lease_max_s": MISSION_LEASE_MAX_S,
        }
        for name, expected in exact.items():
            if _finite(getattr(self, name), name) != expected:
                raise MissionValidationError(
                    f"hierarchical physical {name} must remain {expected}"
                )

    def to_json_dict(self) -> dict[str, float]:
        return asdict(self)


@dataclass(frozen=True)
class HierarchicalPhysicalApproval:
    mission_id: str
    operator: str
    source_sha: str
    deployed_sha: str
    reviewed_sha: str
    proposal_digest: str
    approval_id: str
    approval_digest: str
    approved_at_s: float
    expires_at_s: float
    m7_3_evidence_sha256: str
    directional_addendum_sha256: str
    m7_4_evidence_sha256: str
    attended: bool
    level_bounded: bool
    stairs_ledges_dropoffs_absent: bool
    negative_obstacle_sensing_available: bool
    limits: HierarchicalPhysicalLimits

    @classmethod
    def validated(
        cls,
        raw: Mapping[str, Any],
        *,
        now_s: float,
        source_sha: str,
        deployed_sha: str,
        reviewed_sha: str,
    ) -> "HierarchicalPhysicalApproval":
        payload = dict(raw)
        supplied_digest = _sha256(
            payload.pop("approval_digest", ""), "approval digest"
        )
        if payload.get("schema") != APPROVAL_SCHEMA:
            raise MissionValidationError("hierarchical approval schema is invalid")
        if str(payload.get("gate", "")).lower() != "m7.6":
            raise MissionValidationError(
                "physical hierarchical execution requires a separate M7.6 approval"
            )
        expected_keys = {
            "schema",
            "gate",
            "mission_id",
            "operator",
            "source_sha",
            "deployed_sha",
            "reviewed_sha",
            "proposal_digest",
            "approval_id",
            "approved_at_s",
            "expires_at_s",
            "m7_3_evidence_sha256",
            "directional_addendum_sha256",
            "m7_4_evidence_sha256",
            "room",
            "limits",
        }
        if set(payload) != expected_keys:
            raise MissionValidationError(
                "hierarchical approval contains missing or unreviewed fields"
            )
        if canonical_digest(payload) != supplied_digest:
            raise MissionValidationError("hierarchical approval digest is invalid")
        exact_source = _sha1(source_sha, "runtime source SHA")
        exact_deployed = _sha1(deployed_sha, "runtime deployed SHA")
        exact_reviewed = _sha1(reviewed_sha, "runtime reviewed SHA")
        approval_source = _sha1(payload["source_sha"], "approval source SHA")
        approval_deployed = _sha1(
            payload["deployed_sha"], "approval deployed SHA"
        )
        approval_reviewed = _sha1(
            payload["reviewed_sha"], "approval reviewed SHA"
        )
        if not (
            exact_source
            == exact_deployed
            == exact_reviewed
            == approval_source
            == approval_deployed
            == approval_reviewed
        ):
            raise MissionValidationError(
                "hierarchical approval must bind the exact source, deployed, and reviewed SHA"
            )
        mission_id = str(payload["mission_id"]).strip()
        operator = str(payload["operator"]).strip()
        approval_id = str(payload["approval_id"]).strip()
        if not mission_id or not operator or not approval_id:
            raise MissionValidationError(
                "hierarchical approval identity binding is incomplete"
            )
        proposal_digest = _sha256(
            payload["proposal_digest"], "proposal digest"
        )
        m7_3 = _sha256(
            payload["m7_3_evidence_sha256"], "M7.3 evidence digest"
        )
        directional = _sha256(
            payload["directional_addendum_sha256"],
            "directional evidence digest",
        )
        m7_4 = _sha256(
            payload["m7_4_evidence_sha256"], "M7.4 evidence digest"
        )
        if (
            m7_3 != ACCEPTED_M7_3_EVIDENCE_SHA256
            or directional != ACCEPTED_DIRECTIONAL_ADDENDUM_SHA256
            or m7_4 != ACCEPTED_M7_4_EVIDENCE_SHA256
        ):
            raise MissionValidationError(
                "hierarchical approval does not bind all accepted M7.3/M7.4 evidence"
            )
        room = payload["room"]
        if not isinstance(room, Mapping) or set(room) != {
            "attended",
            "level_bounded",
            "stairs_ledges_dropoffs_absent",
            "negative_obstacle_sensing_available",
        }:
            raise MissionValidationError("hierarchical approval room binding is invalid")
        if (
            room.get("attended") is not True
            or room.get("level_bounded") is not True
            or room.get("stairs_ledges_dropoffs_absent") is not True
            or room.get("negative_obstacle_sensing_available") is not False
        ):
            raise MissionValidationError(
                "hierarchical execution is restricted to an attended level bounded room without drop-offs"
            )
        raw_limits = payload["limits"]
        if not isinstance(raw_limits, Mapping):
            raise MissionValidationError("hierarchical approval limits are invalid")
        limits = HierarchicalPhysicalLimits(**dict(raw_limits))
        approved_at_s = _finite(payload["approved_at_s"], "approved time")
        expires_at_s = _finite(payload["expires_at_s"], "approval expiry")
        now = _finite(now_s, "approval validation time")
        if (
            approved_at_s > now
            or expires_at_s <= now
            or expires_at_s - approved_at_s > limits.mission_lease_max_s
        ):
            raise MissionValidationError(
                "hierarchical approval is future-dated, expired, or exceeds the mission lease"
            )
        return cls(
            mission_id=mission_id,
            operator=operator,
            source_sha=approval_source,
            deployed_sha=approval_deployed,
            reviewed_sha=approval_reviewed,
            proposal_digest=proposal_digest,
            approval_id=approval_id,
            approval_digest=supplied_digest,
            approved_at_s=approved_at_s,
            expires_at_s=expires_at_s,
            m7_3_evidence_sha256=m7_3,
            directional_addendum_sha256=directional,
            m7_4_evidence_sha256=m7_4,
            attended=True,
            level_bounded=True,
            stairs_ledges_dropoffs_absent=True,
            negative_obstacle_sensing_available=False,
            limits=limits,
        )


class HierarchicalBindingJournal:
    """Append-only durable authority and handoff evidence."""

    def __init__(self, path: str | Path) -> None:
        self.path = Path(path).expanduser()
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._lock = threading.RLock()
        self._connection = sqlite3.connect(
            str(self.path), check_same_thread=False
        )
        self._connection.execute(
            """
            CREATE TABLE IF NOT EXISTS hierarchical_binding_events (
                event_index INTEGER PRIMARY KEY AUTOINCREMENT,
                mission_id TEXT NOT NULL,
                kind TEXT NOT NULL,
                recorded_at_s REAL NOT NULL,
                payload_json TEXT NOT NULL,
                payload_sha256 TEXT NOT NULL
            )
            """
        )
        self._connection.commit()

    def append(
        self,
        mission_id: str,
        kind: str,
        payload: Mapping[str, Any],
        *,
        recorded_at_s: Optional[float] = None,
    ) -> dict[str, Any]:
        mission = str(mission_id).strip()
        event_kind = str(kind).strip()
        if not mission or not event_kind:
            raise MissionValidationError("binding journal event identity is required")
        record = json.loads(json.dumps(dict(payload), allow_nan=False))
        digest = canonical_digest(record)
        when = _finite(
            time.time() if recorded_at_s is None else recorded_at_s,
            "journal time",
        )
        rendered = json.dumps(
            record, sort_keys=True, separators=(",", ":"), allow_nan=False
        )
        with self._lock:
            cursor = self._connection.execute(
                """
                INSERT INTO hierarchical_binding_events
                    (mission_id, kind, recorded_at_s, payload_json, payload_sha256)
                VALUES (?, ?, ?, ?, ?)
                """,
                (mission, event_kind, when, rendered, digest),
            )
            self._connection.commit()
            index = int(cursor.lastrowid)
        return {
            "event_index": index,
            "mission_id": mission,
            "kind": event_kind,
            "recorded_at_s": when,
            "payload": record,
            "payload_sha256": digest,
        }

    def events(self, mission_id: Optional[str] = None) -> list[dict[str, Any]]:
        query = (
            """
            SELECT event_index, mission_id, kind, recorded_at_s,
                   payload_json, payload_sha256
            FROM hierarchical_binding_events
            """
        )
        arguments: tuple[Any, ...] = ()
        if mission_id is not None:
            query += " WHERE mission_id = ?"
            arguments = (str(mission_id),)
        query += " ORDER BY event_index"
        with self._lock:
            rows = self._connection.execute(query, arguments).fetchall()
        return [
            {
                "event_index": int(row[0]),
                "mission_id": str(row[1]),
                "kind": str(row[2]),
                "recorded_at_s": float(row[3]),
                "payload": json.loads(str(row[4])),
                "payload_sha256": str(row[5]),
            }
            for row in rows
        ]

    def active_mission_at_shutdown(self) -> Optional[str]:
        events = self.events()
        state_by_mission: dict[str, str] = {}
        for event in events:
            if event["kind"] in {"authority_activated", "authority_relocked"}:
                state_by_mission[event["mission_id"]] = event["kind"]
        active = [
            mission
            for mission, kind in state_by_mission.items()
            if kind == "authority_activated"
        ]
        return active[-1] if active else None

    def used_approval_digests(self) -> set[str]:
        return {
            str(event["payload"].get("approval_digest", ""))
            for event in self.events()
            if event["kind"] == "authority_activated"
        }

    def close(self) -> None:
        with self._lock:
            self._connection.close()


class HierarchicalPhysicalAuthorityOwner:
    """Single-process owner for one non-resumable physical mission lease."""

    def __init__(
        self,
        *,
        enabled: bool,
        source_sha: str,
        deployed_sha: str,
        reviewed_sha: str,
        journal: HierarchicalBindingJournal,
        boot_nonce: str,
    ) -> None:
        self.enabled = bool(enabled)
        self.source_sha = _sha1(source_sha, "source SHA")
        self.deployed_sha = _sha1(deployed_sha, "deployed SHA")
        self.reviewed_sha = (
            _sha1(reviewed_sha, "reviewed SHA")
            if str(reviewed_sha).strip()
            else ""
        )
        if self.enabled and not (
            self.source_sha == self.deployed_sha == self.reviewed_sha
        ):
            raise MissionValidationError(
                "enabled hierarchical authority requires matching exact SHAs"
            )
        self.journal = journal
        self.boot_nonce = str(boot_nonce).strip()
        if not self.boot_nonce:
            raise MissionValidationError("hierarchical boot nonce is required")
        interrupted = self.journal.active_mission_at_shutdown()
        self.state = "recovery_required" if interrupted else "locked"
        self.recovery_mission_id = interrupted or ""
        self._approval: Optional[HierarchicalPhysicalApproval] = None
        self._token = ""
        self._activated_once = False

    def activate(
        self, raw_approval: Mapping[str, Any], *, now_s: float
    ) -> Mapping[str, Any]:
        if not self.enabled:
            raise MissionValidationError(
                "physical hierarchical authority is disabled by default"
            )
        if self.state == "recovery_required":
            raise MissionValidationError(
                "interrupted physical mission requires explicit recovery; restart never resumes"
            )
        if self.state != "locked" or self._activated_once:
            raise MissionValidationError(
                "hierarchical authority owner permits one activation per process"
            )
        approval = HierarchicalPhysicalApproval.validated(
            raw_approval,
            now_s=now_s,
            source_sha=self.source_sha,
            deployed_sha=self.deployed_sha,
            reviewed_sha=self.reviewed_sha,
        )
        if approval.approval_digest in self.journal.used_approval_digests():
            raise MissionValidationError(
                "hierarchical approval digest was already consumed; approval replay is forbidden"
            )
        token_payload = {
            "schema": AUTHORITY_SCHEMA,
            "mission_id": approval.mission_id,
            "approval_digest": approval.approval_digest,
            "source_sha": self.source_sha,
            "boot_nonce": self.boot_nonce,
        }
        self._token = canonical_digest(token_payload)
        self._approval = approval
        self._activated_once = True
        self.state = "active"
        self.journal.append(
            approval.mission_id,
            "authority_activated",
            {
                **token_payload,
                "authority_token_sha256": self._token,
                "expires_at_s": approval.expires_at_s,
                "motion_authority": True,
                "twist_publisher": False,
            },
            recorded_at_s=now_s,
        )
        return self.heartbeat(now_s=now_s)

    def heartbeat(self, *, now_s: float) -> dict[str, Any]:
        now = _finite(now_s, "authority heartbeat time")
        approval = self._approval
        active = bool(
            self.state == "active"
            and approval is not None
            and now < approval.expires_at_s
        )
        if self.state == "active" and not active:
            self.relock(reason="mission_lease_expired", now_s=now)
            approval = None
        return {
            "schema": AUTHORITY_SCHEMA,
            "state": self.state,
            "active": active,
            "mission_lease_valid": active,
            "mission_id": approval.mission_id if active and approval else "",
            "approval_id": approval.approval_id if active and approval else "",
            "approval_digest": (
                approval.approval_digest if active and approval else ""
            ),
            "proposal_digest": (
                approval.proposal_digest if active and approval else ""
            ),
            "authority_token_sha256": self._token if active else "",
            "source_sha": self.source_sha,
            "deployed_sha": self.deployed_sha,
            "reviewed_sha": self.reviewed_sha,
            "issued_at_s": now,
            "expires_at_s": approval.expires_at_s if active and approval else 0.0,
            "limits": HierarchicalPhysicalLimits().to_json_dict(),
            "topics": {
                "nav2_action": NAV2_ACTION,
                "private_nav2_cmd": PRIVATE_NAV2_CMD_TOPIC,
                "bridge_output": SUPERVISOR_REQUEST_TOPIC,
                "motor_output": MOTOR_TOPIC,
            },
            "motion_authority": active,
            "physical_execution_enabled": active,
            "direct_twist_publisher": False,
            "restart_resume_allowed": False,
            "drop_off_detection_available": False,
            "room_restriction": (
                "attended_level_bounded_no_stairs_ledges_or_dropoffs"
            ),
        }

    def relock(self, *, reason: str, now_s: float) -> Mapping[str, Any]:
        mission_id = (
            self._approval.mission_id if self._approval is not None else "none"
        )
        prior_state = self.state
        self.state = "locked"
        self._approval = None
        self._token = ""
        self.journal.append(
            mission_id,
            "authority_relocked",
            {
                "reason": str(reason).strip() or "relocked",
                "prior_state": prior_state,
                "restart_resume_allowed": False,
            },
            recorded_at_s=now_s,
        )
        return self.heartbeat(now_s=now_s)


def validate_authority_heartbeat(
    raw: Mapping[str, Any],
    *,
    now_s: float,
    received_at_s: float,
    source_sha: str,
    deployed_sha: str,
    reviewed_sha: str,
    max_age_s: float = AUTHORITY_HEARTBEAT_MAX_AGE_S,
) -> tuple[bool, str]:
    """Validate a live authority heartbeat for the private command bridge."""

    try:
        payload = dict(raw)
        if payload.get("schema") != AUTHORITY_SCHEMA:
            return False, "authority_schema_invalid"
        source = _sha1(source_sha, "runtime source SHA")
        deployed = _sha1(deployed_sha, "runtime deployed SHA")
        reviewed = _sha1(reviewed_sha, "runtime reviewed SHA")
        if not (
            source
            == deployed
            == reviewed
            == _sha1(payload.get("source_sha"), "heartbeat source SHA")
            == _sha1(payload.get("deployed_sha"), "heartbeat deployed SHA")
            == _sha1(payload.get("reviewed_sha"), "heartbeat reviewed SHA")
        ):
            return False, "authority_sha_mismatch"
        now = _finite(now_s, "authority validation time")
        received = _finite(received_at_s, "authority receipt time")
        issued = _finite(payload.get("issued_at_s"), "authority issue time")
        expires = _finite(payload.get("expires_at_s"), "authority expiry")
        age_limit = _finite(max_age_s, "authority heartbeat max age")
        if (
            age_limit <= 0.0
            or received > now
            or issued > now
            or now - received > age_limit
            or now - issued > age_limit
        ):
            return False, "authority_heartbeat_stale"
        if expires <= now:
            return False, "mission_lease_expired"
        if (
            payload.get("state") != "active"
            or payload.get("active") is not True
            or payload.get("mission_lease_valid") is not True
            or payload.get("motion_authority") is not True
            or payload.get("physical_execution_enabled") is not True
            or payload.get("direct_twist_publisher") is not False
            or payload.get("restart_resume_allowed") is not False
            or payload.get("drop_off_detection_available") is not False
        ):
            return False, "authority_not_active"
        if not str(payload.get("mission_id", "")).strip():
            return False, "authority_mission_missing"
        _sha256(payload.get("approval_digest"), "approval digest")
        _sha256(payload.get("proposal_digest"), "proposal digest")
        _sha256(
            payload.get("authority_token_sha256"), "authority token digest"
        )
        limits = payload.get("limits")
        if not isinstance(limits, Mapping):
            return False, "authority_limits_invalid"
        HierarchicalPhysicalLimits(**dict(limits))
        return True, "active"
    except (MissionValidationError, TypeError, ValueError):
        return False, "authority_malformed"


@dataclass(frozen=True)
class Nav2Pose:
    generation: int
    target_id: str
    target_signature: str
    map_id: str
    map_revision: str
    x_m: float
    y_m: float
    yaw_rad: float

    def to_json_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class Nav2GoalBatch:
    mission_id: str
    source_sha: str
    approval_digest: str
    controller_session: int
    poses: tuple[Nav2Pose, ...]
    created_at_s: float
    reason: str

    def __post_init__(self) -> None:
        if not self.mission_id or self.controller_session < 1 or not self.poses:
            raise MissionValidationError("Nav2 goal batch identity is incomplete")
        _sha1(self.source_sha, "Nav2 batch source SHA")
        _sha256(self.approval_digest, "Nav2 batch approval digest")
        if len(self.poses) > 3:
            raise MissionValidationError("Nav2 goal batch exceeds lookahead depth 3")
        generations = [pose.generation for pose in self.poses]
        if generations != sorted(generations) or len(set(generations)) != len(
            generations
        ):
            raise MissionValidationError(
                "Nav2 goal batch generations must increase uniquely"
            )

    def to_json_dict(self) -> dict[str, Any]:
        payload = {
            "schema": NAV2_BATCH_SCHEMA,
            "mission_id": self.mission_id,
            "source_sha": self.source_sha,
            "approval_digest": self.approval_digest,
            "controller_session": self.controller_session,
            "poses": [pose.to_json_dict() for pose in self.poses],
            "created_at_s": self.created_at_s,
            "reason": self.reason,
            "action": NAV2_ACTION,
            "private_command_topic": PRIVATE_NAV2_CMD_TOPIC,
            "twist_publisher": False,
        }
        payload["batch_digest"] = canonical_digest(payload)
        return payload


def build_nav2_goal_batch(
    goals: Sequence[
        tuple[
            ResolvedSemanticGoal,
            Mapping[str, Any],
            Mapping[str, Any],
        ]
    ],
    *,
    mission_id: str,
    source_sha: str,
    approval_digest: str,
    controller_session: int,
    now_s: float,
    reason: str,
) -> Nav2GoalBatch:
    """Convert only server-resolved and currently revalidated goals."""

    poses: list[Nav2Pose] = []
    for goal, captured_snapshot, current_snapshot in tuple(goals)[:3]:
        localization = current_snapshot.get("localization", {})
        if not isinstance(localization, Mapping):
            raise MissionValidationError(
                "Nav2 batch requires current localization evidence"
            )
        localization_age_s = _finite(
            localization.get("age_s"), "localization age"
        )
        if (
            localization_age_s < 0.0
            or localization_age_s > LOCALIZATION_MAX_AGE_S
        ):
            raise MissionValidationError(
                "Nav2 batch localization exceeds the fixed 0.300 s freshness gate"
            )
        result = revalidate_resolved_goal(
            goal,
            captured_snapshot=captured_snapshot,
            current_snapshot=current_snapshot,
        )
        if not result.accepted:
            raise MissionValidationError(
                "Nav2 batch contains a stale or invalid semantic goal: "
                + ",".join(result.reasons)
            )
        if goal.kind != "motion" or goal.x_m is None or goal.y_m is None:
            raise MissionValidationError(
                "only server-resolved motion goals may enter Nav2"
            )
        yaw = (
            math.atan2(
                goal.y_m - float(current_snapshot["localization"]["y_m"]),
                goal.x_m - float(current_snapshot["localization"]["x_m"]),
            )
            if goal.yaw_rad is None
            else goal.yaw_rad
        )
        poses.append(
            Nav2Pose(
                generation=goal.decision.decision_generation,
                target_id=goal.target_id,
                target_signature=goal.target_signature,
                map_id=goal.map_id,
                map_revision=goal.map_revision,
                x_m=_finite(goal.x_m, "Nav2 goal x"),
                y_m=_finite(goal.y_m, "Nav2 goal y"),
                yaw_rad=_finite(yaw, "Nav2 goal yaw"),
            )
        )
    return Nav2GoalBatch(
        mission_id=str(mission_id).strip(),
        source_sha=_sha1(source_sha, "Nav2 batch source SHA"),
        approval_digest=_sha256(
            approval_digest, "Nav2 batch approval digest"
        ),
        controller_session=int(controller_session),
        poses=tuple(poses),
        created_at_s=_finite(now_s, "Nav2 batch time"),
        reason=str(reason).strip() or "semantic_goal_revalidated",
    )


def resolve_goal_dispatch(
    raw: Mapping[str, Any],
    *,
    authority: Mapping[str, Any],
    now_s: float,
) -> Nav2GoalBatch:
    """Resolve a mission-service dispatch without accepting model geometry."""

    payload = dict(raw)
    expected_keys = {
        "schema",
        "mission_id",
        "source_sha",
        "approval_digest",
        "controller_session",
        "reason",
        "goals",
        "dispatch_digest",
    }
    if set(payload) != expected_keys or payload.get("schema") != GOAL_DISPATCH_SCHEMA:
        raise MissionValidationError(
            "hierarchical goal dispatch has missing or unreviewed fields"
        )
    supplied_digest = _sha256(
        payload.pop("dispatch_digest"), "goal dispatch digest"
    )
    if canonical_digest(payload) != supplied_digest:
        raise MissionValidationError("hierarchical goal dispatch digest is invalid")
    mission_id = str(payload["mission_id"]).strip()
    source_sha = _sha1(payload["source_sha"], "goal dispatch source SHA")
    approval_digest = _sha256(
        payload["approval_digest"], "goal dispatch approval digest"
    )
    if (
        mission_id != str(authority.get("mission_id", "")).strip()
        or source_sha != str(authority.get("source_sha", "")).strip()
        or approval_digest != str(authority.get("approval_digest", "")).strip()
    ):
        raise MissionValidationError(
            "goal dispatch does not bind the active mission authority"
        )
    raw_goals = payload["goals"]
    if (
        not isinstance(raw_goals, list)
        or not 1 <= len(raw_goals) <= 3
    ):
        raise MissionValidationError(
            "hierarchical dispatch must contain one to three semantic goals"
        )
    resolver = DeterministicGoalResolver()
    resolved: list[
        tuple[ResolvedSemanticGoal, Mapping[str, Any], Mapping[str, Any]]
    ] = []
    for item in raw_goals:
        if not isinstance(item, Mapping) or set(item) != {
            "decision",
            "captured_snapshot",
            "current_snapshot",
        }:
            raise MissionValidationError(
                "hierarchical dispatch goal structure is invalid"
            )
        captured = item["captured_snapshot"]
        current = item["current_snapshot"]
        decision_raw = item["decision"]
        if (
            not isinstance(captured, Mapping)
            or not isinstance(current, Mapping)
            or not isinstance(decision_raw, Mapping)
        ):
            raise MissionValidationError(
                "hierarchical dispatch snapshots and decision must be objects"
            )
        expected_generation = int(
            captured.get("decision_generation", 0)
        )
        decision = SemanticGoalDecision.validated(
            decision_raw,
            snapshot=captured,
            expected_generation=expected_generation,
            provider_id=str(
                decision_raw.get(
                    "provider_id", "mission-service-semantic-provider"
                )
            ),
            model_id=str(
                decision_raw.get("model_id", "mission-service-semantic-model")
            ),
        )
        goal = resolver.resolve(decision, captured, ready_at_s=now_s)
        resolved.append((goal, captured, current))
    return build_nav2_goal_batch(
        resolved,
        mission_id=mission_id,
        source_sha=source_sha,
        approval_digest=approval_digest,
        controller_session=int(payload["controller_session"]),
        now_s=now_s,
        reason=str(payload["reason"]),
    )


def build_goal_dispatch(
    controller: AsyncSemanticGoalController,
    *,
    authority: Mapping[str, Any],
    current_snapshot: Mapping[str, Any],
    controller_session: int,
    reason: str,
) -> dict[str, Any]:
    """Export the live M6 controller queue as a bound semantic dispatch."""

    goals = []
    for goal, captured in controller.resolved_motion_goals():
        goals.append(
            {
                # Remove provider metadata that is evidence, not part of the
                # strict model response schema revalidated by the adapter.
                "decision": {
                    key: value
                    for key, value in goal.decision.to_json_dict().items()
                    if key not in {"provider_id", "model_id"}
                },
                "captured_snapshot": json.loads(
                    json.dumps(dict(captured), allow_nan=False)
                ),
                "current_snapshot": json.loads(
                    json.dumps(dict(current_snapshot), allow_nan=False)
                ),
            }
        )
    if not goals:
        raise MissionValidationError(
            "live hierarchical controller has no resolved Nav2 goal"
        )
    payload = {
        "schema": GOAL_DISPATCH_SCHEMA,
        "mission_id": str(authority.get("mission_id", "")).strip(),
        "source_sha": _sha1(
            authority.get("source_sha"), "authority source SHA"
        ),
        "approval_digest": _sha256(
            authority.get("approval_digest"), "authority approval digest"
        ),
        "controller_session": int(controller_session),
        "reason": str(reason).strip() or "controller_queue_updated",
        "goals": goals,
    }
    payload["dispatch_digest"] = canonical_digest(payload)
    # Exercise the same deterministic resolution/revalidation that the ROS
    # consumer will run before this dispatch can leave the process.
    resolve_goal_dispatch(
        payload,
        authority=authority,
        now_s=_finite(
            current_snapshot.get("observed_at_s", time.time()),
            "current snapshot observation time",
        ),
    )
    return payload
