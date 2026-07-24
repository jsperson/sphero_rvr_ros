import json
import math
from pathlib import Path
from typing import Optional

import pytest

from sphero_rvr_driver.perception_navigation import (
    GoalRegion,
    HorizonKind,
    HorizonObservation,
    LocalizationEstimate,
    LocalizationState,
    NavigationOutcome,
    PerceptionGuidedNavigator,
    Pose2D,
    REPLAY_SCHEMA,
    run_navigation_replay,
)


def localization(
    stamp_s: float,
    x_m: float,
    y_m: float,
    heading_deg: float,
    *,
    state: LocalizationState = LocalizationState.VALID,
    quality: float = 0.92,
    source: str = "lidar_scan_match",
    translation_disagreement_m: float = 0.01,
    heading_disagreement_deg: float = 1.0,
) -> LocalizationEstimate:
    return LocalizationEstimate(
        state=state,
        pose=None
        if state is LocalizationState.LOST
        else Pose2D(
            stamp_s=stamp_s,
            frame_id="map",
            x_m=x_m,
            y_m=y_m,
            yaw_rad=math.radians(heading_deg),
        ),
        source=source,
        quality=quality,
        covariance_xy_m2=0.0025,
        covariance_yaw_rad2=math.radians(2.0) ** 2,
        odom_translation_disagreement_m=translation_disagreement_m,
        odom_heading_disagreement_rad=math.radians(heading_disagreement_deg),
    )


def goal(
    *,
    x_m: float = 0.30,
    y_m: float = 0.0,
    radius_m: float = 0.05,
    heading_min_deg: Optional[float] = None,
    heading_max_deg: Optional[float] = None,
) -> GoalRegion:
    return GoalRegion(
        frame_id="map",
        x_m=x_m,
        y_m=y_m,
        radius_m=radius_m,
        minimum_clearance_m=0.20,
        max_runtime_s=20.0,
        max_cumulative_translation_m=1.0,
        heading_min_rad=None if heading_min_deg is None else math.radians(heading_min_deg),
        heading_max_rad=None if heading_max_deg is None else math.radians(heading_max_deg),
    )


def observe(
    estimate: LocalizationEstimate,
    *,
    left: float = 0.10,
    right: float = 0.10,
    **kwargs,
) -> HorizonObservation:
    return HorizonObservation(
        localization=estimate,
        now_s=estimate.pose.stamp_s if estimate.pose is not None else 2.0,
        left_track_delta_m=left,
        right_track_delta_m=right,
        **kwargs,
    )


def test_nominal_replay_recomputes_a_short_horizon_then_reaches_goal_region() -> None:
    navigator = PerceptionGuidedNavigator()

    first = navigator.start(goal(), localization(1.0, 0.0, 0.0, 0.0), now_s=1.0)
    second = navigator.observe(observe(localization(2.0, 0.14, 0.0, 0.0), left=0.14, right=0.14))
    terminal = navigator.observe(observe(localization(3.0, 0.26, 0.0, 0.0), left=0.12, right=0.12))

    assert first.next_horizon is not None
    assert first.next_horizon.kind is HorizonKind.TRANSLATE
    assert first.next_horizon.distance_m == pytest.approx(0.15)
    assert second.next_horizon is not None
    assert second.next_horizon.horizon_id == 2
    assert second.next_horizon.distance_m == pytest.approx(0.11)
    assert terminal.outcome is NavigationOutcome.REACHED
    assert terminal.terminal_reason == "goal_region_reached"
    assert terminal.next_horizon is None
    assert terminal.zero_output_required is True
    assert [round(item.x_m, 2) for item in terminal.path] == [0.0, 0.14, 0.26]


def test_heading_overshoot_changes_the_next_horizon_direction() -> None:
    navigator = PerceptionGuidedNavigator()
    heading_goal = goal(
        x_m=0.0,
        y_m=0.0,
        radius_m=0.05,
        heading_min_deg=34.0,
        heading_max_deg=36.0,
    )

    first = navigator.start(
        heading_goal,
        localization(1.0, 0.0, 0.0, 0.0),
        now_s=1.0,
    )
    correction = navigator.observe(
        observe(
            localization(2.0, 0.0, 0.0, 50.0),
            left=-0.03,
            right=0.03,
        )
    )
    terminal = navigator.observe(
        observe(
            localization(3.0, 0.0, 0.0, 35.0),
            left=0.03,
            right=-0.03,
        )
    )

    assert first.next_horizon is not None
    assert first.next_horizon.angle_rad > 0.0
    assert correction.next_horizon is not None
    assert correction.next_horizon.kind is HorizonKind.ROTATE
    assert correction.next_horizon.angle_rad < 0.0
    assert correction.events[-1].kind == "correction"
    assert terminal.outcome is NavigationOutcome.REACHED


def test_obstacle_evidence_selects_the_clearer_bounded_alternate_horizon() -> None:
    navigator = PerceptionGuidedNavigator()
    navigator.start(goal(x_m=0.50), localization(1.0, 0.0, 0.0, 0.0), now_s=1.0)

    decision = navigator.observe(
        observe(
            localization(2.0, 0.05, 0.0, 0.0),
            left=0.05,
            right=0.05,
            obstacle_blocked=True,
            left_clearance_m=0.55,
            right_clearance_m=0.24,
        )
    )

    assert decision.outcome is NavigationOutcome.RUNNING
    assert decision.next_horizon is not None
    assert decision.next_horizon.kind is HorizonKind.ROTATE
    assert decision.next_horizon.angle_rad > 0.0
    assert decision.next_horizon.angle_rad == pytest.approx(math.radians(15.0))
    assert "alternate_left" in decision.next_horizon.reason
    assert decision.events[-1].kind == "alternate_horizon"


def test_obstacle_with_no_clear_alternate_stops_blocked() -> None:
    navigator = PerceptionGuidedNavigator()
    navigator.start(goal(x_m=0.50), localization(1.0, 0.0, 0.0, 0.0), now_s=1.0)

    decision = navigator.observe(
        observe(
            localization(2.0, 0.05, 0.0, 0.0),
            left=0.05,
            right=0.05,
            obstacle_blocked=True,
            left_clearance_m=0.12,
            right_clearance_m=0.18,
        )
    )

    assert decision.outcome is NavigationOutcome.BLOCKED
    assert decision.terminal_reason == "no_clear_alternate_horizon"
    assert decision.next_horizon is None


@pytest.mark.parametrize(
    ("state", "quality", "now_s", "reason"),
    [
        (LocalizationState.LOST, 0.0, 2.0, "localization_lost"),
        (LocalizationState.STALE, 0.8, 2.0, "localization_stale"),
        (LocalizationState.DEGRADED, 0.4, 2.0, "localization_quality_below_minimum"),
        (LocalizationState.VALID, 0.9, 3.0, "localization_stale"),
    ],
)
def test_stale_lost_or_low_quality_localization_stops_without_another_horizon(
    state: LocalizationState,
    quality: float,
    now_s: float,
    reason: str,
) -> None:
    navigator = PerceptionGuidedNavigator()
    navigator.start(goal(), localization(1.0, 0.0, 0.0, 0.0), now_s=1.0)
    estimate = localization(2.0, 0.05, 0.0, 0.0, state=state, quality=quality)

    decision = navigator.observe(
        HorizonObservation(
            localization=estimate,
            now_s=now_s,
            left_track_delta_m=0.05,
            right_track_delta_m=0.05,
        )
    )

    assert decision.outcome is NavigationOutcome.LOCALIZATION_LOST
    assert decision.terminal_reason == reason
    assert decision.next_horizon is None
    assert decision.zero_output_required is True


def test_odom_disagreement_is_preserved_and_fails_localization_closed() -> None:
    navigator = PerceptionGuidedNavigator()
    navigator.start(goal(), localization(1.0, 0.0, 0.0, 0.0), now_s=1.0)

    decision = navigator.observe(
        observe(
            localization(
                2.0,
                0.05,
                0.0,
                0.0,
                translation_disagreement_m=0.20,
            ),
            left=0.05,
            right=0.05,
        )
    )

    assert decision.outcome is NavigationOutcome.LOCALIZATION_LOST
    assert decision.terminal_reason == "localization_odom_translation_disagreement"
    payload = decision.to_json_dict()
    assert payload["localization"]["odom_translation_disagreement_m"] == pytest.approx(0.20)
    assert payload["motion_authority"] is False


def test_recorded_attempt_5_track_asymmetry_stops_before_correction() -> None:
    replay_path = (
        Path(__file__).parents[1]
        / "artifacts"
        / "perception_navigation_replay"
        / "turn_attempt_5_replay.json"
    )
    payload = json.loads(replay_path.read_text())

    result = run_navigation_replay(payload)

    assert payload["schema"] == REPLAY_SCHEMA
    assert result["terminal"]["outcome"] == NavigationOutcome.PROGRESS_FAILED.value
    assert result["terminal"]["terminal_reason"] == "severe_tread_asymmetry"
    assert result["terminal"]["next_horizon"] is None
    assert result["terminal"]["zero_output_required"] is True
    assert result["motion_authority"] is False
    assert result["physical_execution_enabled"] is False
    assert result["source_evidence"]["source_sha"].startswith("c13cc51")


@pytest.mark.parametrize(
    ("flag", "outcome", "reason"),
    [
        ("cancel", NavigationOutcome.CANCELLED, "cancelled"),
        ("stop", NavigationOutcome.STOPPED, "operator_stop"),
        ("estop", NavigationOutcome.ESTOPPED, "estop"),
    ],
)
def test_cancel_stop_and_estop_latch_terminal_without_another_horizon(
    flag: str,
    outcome: NavigationOutcome,
    reason: str,
) -> None:
    navigator = PerceptionGuidedNavigator()
    navigator.start(goal(), localization(1.0, 0.0, 0.0, 0.0), now_s=1.0)
    kwargs = {flag: True}

    decision = navigator.observe(
        observe(
            localization(2.0, 0.01, 0.0, 0.0),
            left=0.01,
            right=0.01,
            **kwargs,
        )
    )

    assert decision.outcome is outcome
    assert decision.terminal_reason == reason
    assert decision.next_horizon is None
    after_latch = navigator.observe(
        observe(localization(3.0, 0.20, 0.0, 0.0), left=0.19, right=0.19)
    )
    assert after_latch is decision


def test_contract_rejects_encoder_only_localization_as_authoritative() -> None:
    navigator = PerceptionGuidedNavigator()

    decision = navigator.start(
        goal(),
        localization(1.0, 0.0, 0.0, 0.0, source="encoder_odometry"),
        now_s=1.0,
    )

    assert decision.outcome is NavigationOutcome.LOCALIZATION_LOST
    assert decision.terminal_reason == "localization_source_not_lidar_authoritative"


def test_goal_heading_range_accepts_wrapped_negative_angles() -> None:
    parsed = GoalRegion.from_mapping(
        {
            "frame_id": "map",
            "x_m": 0.0,
            "y_m": 0.0,
            "radius_m": 0.05,
            "minimum_clearance_m": 0.2,
            "max_runtime_s": 20.0,
            "max_cumulative_translation_m": 0.5,
            "heading_min_rad": math.radians(-10.0),
            "heading_max_rad": math.radians(10.0),
        }
    )

    assert parsed.heading_min_rad == pytest.approx(math.radians(-10.0))
