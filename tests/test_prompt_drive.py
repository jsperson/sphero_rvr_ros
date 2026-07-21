from __future__ import annotations

from dataclasses import replace
import io
import json
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace

import pytest

from sphero_rvr_driver.mission_api import MissionValidationError
from sphero_rvr_driver.prompt_drive import (
    OpenAIPromptDriveProvider,
    PromptDriveDecision,
    PromptDriveLimits,
    PromptDrivePlanner,
    PromptDriveProviderResponse,
    PromptDriveSegment,
    approval_phrase,
    approved_live_route,
    build_prompt_drive_manifest,
    build_prompt_drive_payload,
    parse_prompt_drive_response,
)
from sphero_rvr_driver import prompt_drive_cli
from sphero_rvr_driver.prompt_drive_ros import RosLiveRouteExecutor


REPO_ROOT = Path(__file__).resolve().parents[1]


class _FakeProvider:
    provider_id = "fake-openai"
    model_id = "gpt-test"
    reasoning_effort = "high"

    def __init__(self, response: PromptDriveProviderResponse):
        self.response = response
        self.prompts: list[str] = []

    def propose(self, prompt: str, limits: PromptDriveLimits) -> PromptDriveProviderResponse:
        self.prompts.append(prompt)
        assert limits.max_motion_calls == 3
        return self.response


def _provider_response() -> PromptDriveProviderResponse:
    return PromptDriveProviderResponse(
        PromptDriveDecision.PROPOSE,
        "Move forward 20 cm and turn left 90 degrees.",
        (
            {"tool_name": "move_distance", "value": 0.2},
            {"tool_name": "turn_angle", "value": 90.0},
        ),
    )


def _proposal():
    return PromptDrivePlanner(_FakeProvider(_provider_response()), source_sha="test-sha").propose(
        "Move forward 20 centimeters, turn left 90 degrees, then stop."
    )


def test_openai_payload_exposes_only_one_bounded_route_tool_and_typed_rejection() -> None:
    payload = build_prompt_drive_payload("Move forward 10 centimeters.", PromptDriveLimits(), reasoning_effort="high")

    assert payload["model"] == "gpt-5.6"
    assert payload["reasoning"] == {"effort": "high"}
    assert payload["tool_choice"] == "required"
    assert payload["parallel_tool_calls"] is False
    assert [tool["name"] for tool in payload["tools"]] == ["propose_rover_route", "reject_rover_route"]
    schema_text = json.dumps(payload["tools"], sort_keys=True)
    assert "move_distance" in schema_text
    assert "turn_angle" in schema_text
    assert "move_to_clearance" not in schema_text
    assert "speed_mps" not in schema_text
    assert "/cmd_vel" not in schema_text
    assert json.loads(payload["input"])["execution_mode"] == "proposal_only_until_local_operator_approval"


def test_response_parser_requires_exactly_one_typed_decision() -> None:
    body = {
        "output": [
            {
                "type": "function_call",
                "name": "propose_rover_route",
                "arguments": json.dumps(
                    {
                        "summary": "Turn right.",
                        "segments": [{"tool_name": "turn_angle", "value": -45.0}],
                    }
                ),
            }
        ]
    }

    response = parse_prompt_drive_response(body)

    assert response.decision is PromptDriveDecision.PROPOSE
    assert response.segments == ({"tool_name": "turn_angle", "value": -45.0},)
    with pytest.raises(MissionValidationError, match="exactly one"):
        parse_prompt_drive_response({"output": body["output"] * 2})


def test_planner_applies_fixed_executor_parameters_and_stable_digest() -> None:
    first = _proposal()
    second = _proposal()

    assert first.proposal_digest == second.proposal_digest
    assert [segment.tool_id for segment in first.segments] == ["move_distance", "turn_angle"]
    assert first.segments[0].arguments["speed_mps"] == 0.08
    assert first.segments[1].arguments["angular_speed_deg_s"] == 30.0
    assert first.segments[0].arguments["distance_m"] == 0.2
    assert first.segments[1].arguments["angle_deg"] == 90.0


def test_exact_digest_approval_builds_correlated_live_route() -> None:
    proposal = _proposal()

    route = approved_live_route(proposal, approval_phrase(proposal), operator="operator:scott")

    assert route.route_id == f"prompt-drive-{proposal.proposal_digest[:12]}"
    assert route.approval_id == f"operator:scott:{proposal.proposal_digest}"
    assert route.source_sha == "test-sha"
    assert [segment.tool_id for segment in route.segments] == ["move_distance", "turn_angle"]
    assert route.max_travel_m == 0.5


def test_approval_refuses_wrong_phrase_and_mutated_proposal() -> None:
    proposal = _proposal()
    with pytest.raises(MissionValidationError, match="does not match"):
        approved_live_route(proposal, "APPROVE wrong")

    changed_segment = PromptDriveSegment(
        proposal.segments[0].correlation_id,
        proposal.segments[0].tool_id,
        {**proposal.segments[0].arguments, "distance_m": 0.3},
    )
    changed = replace(proposal, segments=(changed_segment, *proposal.segments[1:]))
    with pytest.raises(MissionValidationError, match="changed after"):
        approved_live_route(changed, approval_phrase(changed))


@pytest.mark.parametrize(
    ("segments", "message"),
    (
        (({"tool_name": "capture_observation", "value": 1.0},), "non-MVP tool"),
        (({"tool_name": "move_distance", "value": -0.1},), "positive/forward"),
        (({"tool_name": "turn_angle", "value": 181.0},), "turn_angle exceeds"),
        (
            (
                {"tool_name": "move_distance", "value": 0.3},
                {"tool_name": "move_distance", "value": 0.3},
            ),
            "cumulative translation",
        ),
        (
            (
                {"tool_name": "turn_angle", "value": 10.0},
                {"tool_name": "turn_angle", "value": 20.0},
                {"tool_name": "turn_angle", "value": 30.0},
                {"tool_name": "turn_angle", "value": 40.0},
            ),
            "max_motion_calls",
        ),
    ),
)
def test_planner_fails_closed_on_model_calls_outside_the_mvp_envelope(segments, message) -> None:
    provider = _FakeProvider(PromptDriveProviderResponse(PromptDriveDecision.PROPOSE, "bad route", segments))

    with pytest.raises(MissionValidationError, match=message):
        PromptDrivePlanner(provider, source_sha="test-sha").propose("Move somehow.")


def test_model_rejection_cannot_carry_motion() -> None:
    safe = PromptDrivePlanner(
        _FakeProvider(PromptDriveProviderResponse(PromptDriveDecision.REJECT, "Reverse is unsupported.")),
        source_sha="test-sha",
    ).propose("Drive backward.")
    assert not safe.executable
    assert safe.segments == ()

    unsafe = _FakeProvider(
        PromptDriveProviderResponse(
            PromptDriveDecision.REJECT,
            "No.",
            ({"tool_name": "move_distance", "value": 0.1},),
        )
    )
    with pytest.raises(MissionValidationError, match="cannot include motion"):
        PromptDrivePlanner(unsafe, source_sha="test-sha").propose("Drive backward.")


def test_openai_provider_posts_first_party_response_and_does_not_expose_key(monkeypatch) -> None:
    monkeypatch.setenv("OPENAI_API_KEY", "super-secret-test-key")
    requests = []

    class _Response:
        def __enter__(self):
            return self

        def __exit__(self, *args):
            return False

        def read(self):
            return json.dumps(
                {
                    "output": [
                        {
                            "type": "function_call",
                            "name": "propose_rover_route",
                            "arguments": json.dumps(
                                {
                                    "summary": "Move forward 10 cm.",
                                    "segments": [{"tool_name": "move_distance", "value": 0.1}],
                                }
                            ),
                        }
                    ]
                }
            ).encode()

    def _urlopen(request, timeout):
        requests.append((request, timeout))
        return _Response()

    monkeypatch.setattr("urllib.request.urlopen", _urlopen)
    provider = OpenAIPromptDriveProvider(model="gpt-5.6", reasoning_effort="high", max_retries=0)

    response = provider.propose("Move forward 10 centimeters.", PromptDriveLimits())

    assert response.decision is PromptDriveDecision.PROPOSE
    request, timeout = requests[0]
    assert request.full_url == "https://api.openai.com/v1/responses"
    assert timeout == 45.0
    assert request.headers["Authorization"] == "Bearer super-secret-test-key"
    assert b"super-secret-test-key" not in request.data


def test_proposal_only_cli_never_imports_ros_and_writes_safe_manifest(monkeypatch, tmp_path, capsys) -> None:
    monkeypatch.setattr(prompt_drive_cli, "OpenAIPromptDriveProvider", lambda **kwargs: _FakeProvider(_provider_response()))
    sys.modules.pop("sphero_rvr_driver.prompt_drive_ros", None)

    result = prompt_drive_cli.main(
        ["Move forward 20 centimeters, turn left 90 degrees, then stop.", "--manifest-dir", str(tmp_path)]
    )

    assert result == 0
    assert "sphero_rvr_driver.prompt_drive_ros" not in sys.modules
    output = capsys.readouterr().out
    assert "No route was published" in output
    manifest = json.loads(next(tmp_path.glob("*.json")).read_text())
    assert manifest["status"] == "proposal_only"
    assert manifest["credential_material_recorded"] is False
    assert "OPENAI_API_KEY" not in json.dumps(manifest)


def test_execute_cli_requires_interactive_terminal_before_ros_import(monkeypatch, tmp_path) -> None:
    monkeypatch.setattr(prompt_drive_cli, "OpenAIPromptDriveProvider", lambda **kwargs: _FakeProvider(_provider_response()))
    monkeypatch.setattr(sys, "stdin", io.StringIO())
    sys.modules.pop("sphero_rvr_driver.prompt_drive_ros", None)

    result = prompt_drive_cli.main(
        ["Move forward 20 centimeters.", "--execute", "--manifest-dir", str(tmp_path)]
    )

    assert result == 3
    assert "sphero_rvr_driver.prompt_drive_ros" not in sys.modules
    assert json.loads(next(tmp_path.glob("*.json")).read_text())["status"] == "approval_refused"


def test_ros_executor_source_owns_only_route_request_and_never_velocity_or_motor_topics() -> None:
    source = (REPO_ROOT / "src" / "sphero_rvr_driver" / "prompt_drive_ros.py").read_text()
    setup = (REPO_ROOT / "setup.py").read_text()

    assert "rvr_prompt_drive = sphero_rvr_driver.prompt_drive_cli:main" in setup
    assert 'request_topic: str = "/mission_api/v2/live_route/request"' in source
    assert "create_publisher(String, self.request_topic, 10)" in source
    assert "from geometry_msgs" not in source
    assert "Twist" not in source
    assert "/cmd_vel_motor" not in source
    assert "Serial" not in source


def test_ros_executor_waits_for_both_route_topics_and_returns_correlated_terminal_status(monkeypatch) -> None:
    proposal = _proposal()
    route = approved_live_route(proposal, approval_phrase(proposal))
    published = []
    state = {"ok": False}

    class _String:
        def __init__(self):
            self.data = ""

    class _Trigger:
        class Request:
            pass

    class _Subscription:
        def get_publisher_count(self):
            return 1

    class _Publisher:
        def __init__(self, node):
            self.node = node

        def get_subscription_count(self):
            return 1

        def publish(self, message):
            published.append(json.loads(message.data))
            status = _String()
            status.data = json.dumps(
                {
                    "route_id": route.route_id,
                    "status": "complete",
                    "terminal_reason": "complete",
                    "measured_distance_m": 0.2,
                }
            )
            self.node.callback(status)

    class _Client:
        def service_is_ready(self):
            return True

        def call_async(self, request):
            return SimpleNamespace(done=lambda: True, result=lambda: SimpleNamespace(success=True))

    class _Node:
        def __init__(self, name):
            self.callback = None

        def create_publisher(self, message_type, topic, depth):
            return _Publisher(self)

        def create_subscription(self, message_type, topic, callback, depth):
            self.callback = callback
            return _Subscription()

        def create_client(self, service_type, name):
            return _Client()

        def destroy_node(self):
            pass

    rclpy = ModuleType("rclpy")
    rclpy.ok = lambda: state["ok"]
    rclpy.init = lambda args=None: state.update(ok=True)
    rclpy.spin_once = lambda node, timeout_sec=0.1: None
    rclpy.try_shutdown = lambda: state.update(ok=False)
    node_module = ModuleType("rclpy.node")
    node_module.Node = _Node
    std_msgs = ModuleType("std_msgs")
    std_msgs_msg = ModuleType("std_msgs.msg")
    std_msgs_msg.String = _String
    std_srvs = ModuleType("std_srvs")
    std_srvs_srv = ModuleType("std_srvs.srv")
    std_srvs_srv.Trigger = _Trigger
    for name, module in {
        "rclpy": rclpy,
        "rclpy.node": node_module,
        "std_msgs": std_msgs,
        "std_msgs.msg": std_msgs_msg,
        "std_srvs": std_srvs,
        "std_srvs.srv": std_srvs_srv,
    }.items():
        monkeypatch.setitem(sys.modules, name, module)

    result = RosLiveRouteExecutor().execute(route)

    assert result["status"] == "complete"
    assert result["measured_distance_m"] == 0.2
    assert published == [route.to_json_dict()]
    assert state["ok"] is False


def test_safe_manifest_records_approval_binding_without_credentials() -> None:
    proposal = _proposal()
    route = approved_live_route(proposal, approval_phrase(proposal), operator="operator:scott")

    manifest = build_prompt_drive_manifest(
        proposal,
        status="complete",
        operator="operator:scott",
        approved=True,
        route=route,
        execution_result={"status": "complete", "measured_distance_m": 0.2},
    )

    text = json.dumps(manifest)
    assert manifest["approval"]["proposal_digest"] == proposal.proposal_digest
    assert manifest["route_request"]["approval_id"].endswith(proposal.proposal_digest)
    assert manifest["execution_result"]["measured_distance_m"] == 0.2
    assert "OPENAI_API_KEY" not in text
    assert "Authorization" not in text
