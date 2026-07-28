"""ROS publisher for the non-resumable hierarchical physical authority lease."""

from __future__ import annotations

import json
import os
from pathlib import Path
import time
import uuid

from .hierarchical_physical_binding import (
    AUTHORITY_TOPIC,
    HierarchicalBindingJournal,
    HierarchicalPhysicalAuthorityOwner,
)


def main(args=None):
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from std_msgs.msg import String

    class HierarchicalAuthorityNode(Node):
        def __init__(self):
            super().__init__("hierarchical_physical_authority")
            for name, default in {
                "enabled": False,
                "source_sha": "",
                "deployed_sha": "",
                "reviewed_sha": "",
                "approval_file": "",
                "journal_path": (
                    "~/.local/state/sphero_rvr/"
                    "hierarchical-physical-evidence.sqlite3"
                ),
                "authority_topic": AUTHORITY_TOPIC,
                "heartbeat_period_s": 0.10,
            }.items():
                self.declare_parameter(name, default)
            enabled = bool(self.get_parameter("enabled").value)
            if not enabled:
                raise ValueError(
                    "hierarchical authority node is default-off; enabled must be explicit"
                )
            topic = str(self.get_parameter("authority_topic").value)
            if topic != AUTHORITY_TOPIC:
                raise ValueError(
                    f"hierarchical authority topic must remain {AUTHORITY_TOPIC}"
                )
            source_sha = self._required_provenance(
                "source_sha", "RVR_SOURCE_SHA"
            )
            deployed_sha = self._required_provenance(
                "deployed_sha", "RVR_DEPLOYED_SHA"
            )
            reviewed_sha = self._required_provenance(
                "reviewed_sha", "RVR_HIERARCHICAL_REVIEWED_SHA"
            )
            approval_file = Path(
                str(self.get_parameter("approval_file").value)
            ).expanduser()
            if not approval_file.is_file():
                raise ValueError(
                    "hierarchical authority requires a pre-existing M7.6 approval file"
                )
            try:
                approval = json.loads(
                    approval_file.read_text(encoding="utf-8")
                )
            except (OSError, ValueError, json.JSONDecodeError) as exc:
                raise ValueError(
                    "hierarchical M7.6 approval file is unreadable or malformed"
                ) from exc
            if not isinstance(approval, dict):
                raise ValueError("hierarchical M7.6 approval must be an object")
            self._journal = HierarchicalBindingJournal(
                str(self.get_parameter("journal_path").value)
            )
            self._owner = HierarchicalPhysicalAuthorityOwner(
                enabled=True,
                source_sha=source_sha,
                deployed_sha=deployed_sha,
                reviewed_sha=reviewed_sha,
                journal=self._journal,
                boot_nonce=uuid.uuid4().hex,
            )
            self._owner.activate(approval, now_s=time.time())
            self._publisher = self.create_publisher(String, AUTHORITY_TOPIC, 10)
            period = float(self.get_parameter("heartbeat_period_s").value)
            if not 0.05 <= period <= 0.20:
                raise ValueError(
                    "hierarchical authority heartbeat period must be between 0.05 and 0.20 seconds"
                )
            self.create_timer(period, self._publish)
            self._publish()

        def _required_provenance(
            self, parameter: str, environment: str
        ) -> str:
            value = (
                str(self.get_parameter(parameter).value).strip()
                or os.environ.get(environment, "").strip()
            )
            if not value:
                raise ValueError(
                    f"{parameter} must come from reviewed deployment provenance"
                )
            return value

        def _publish(self) -> None:
            payload = self._owner.heartbeat(now_s=time.time())
            message = String()
            message.data = json.dumps(payload, sort_keys=True)
            self._publisher.publish(message)

        def relock(self, reason: str) -> None:
            self._owner.relock(reason=reason, now_s=time.time())
            self._publish()
            self._journal.close()

    rclpy.init(args=args)
    node = HierarchicalAuthorityNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.relock("authority_node_shutdown")
        finally:
            node.destroy_node()
            rclpy.try_shutdown()


if __name__ == "__main__":
    main()
