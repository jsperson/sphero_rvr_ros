import sys
from types import ModuleType, SimpleNamespace

import pytest

from sphero_rvr_driver.tui_ros import (
    RVRStatus,
    RVRROSClient,
    format_status_lines,
    update_battery_status,
    update_odom_status,
    update_scan_status,
)


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


class FakeNode:
    def __init__(self):
        self.created_publishers = []
        self.destroyed_publishers = []
        self.topics = {"/cmd_vel"}

    def create_publisher(self, msg_type, topic, qos):
        publisher = FakePublisher()
        self.created_publishers.append((msg_type, topic, qos, publisher))
        return publisher

    def destroy_publisher(self, publisher):
        self.destroyed_publishers.append(publisher)

    def create_subscription(self, *args):
        return SimpleNamespace(args=args)

    def create_client(self, _srv_type, name):
        return SimpleNamespace(srv_name=name, service_is_ready=lambda: False)

    def get_topic_names_and_types(self):
        return [(topic, []) for topic in self.topics]

    def count_publishers(self, topic):
        if topic != "/cmd_vel":
            return 0
        return len(self.created_publishers) - len(self.destroyed_publishers)


class FakeRclpy(ModuleType):
    def __init__(self, node):
        super().__init__("rclpy")
        self._node = node
        self.initialized = False
        self.time = SimpleNamespace(Time=lambda: None)

    def ok(self):
        return self.initialized

    def init(self, args=None):
        self.initialized = True

    def create_node(self, name):
        return self._node

    def shutdown(self):
        self.initialized = False

    def spin_once(self, node, timeout_sec=0.1):
        return None


class DelayedTriggerFuture:
    def __init__(self):
        self.done_calls = 0

    def done(self):
        self.done_calls += 1
        return self.done_calls >= 2

    def result(self):
        return SimpleNamespace(success=True, message="service ok")


def install_fake_ros_modules(monkeypatch):
    node = FakeNode()
    fake_rclpy = FakeRclpy(node)
    monkeypatch.setitem(sys.modules, "rclpy", fake_rclpy)
    monkeypatch.setitem(sys.modules, "rclpy.node", SimpleNamespace(Node=FakeNode))
    message_modules = {
        "diagnostic_msgs.msg": {"DiagnosticArray": type("DiagnosticArray", (), {})},
        "geometry_msgs.msg": {"Twist": _fake_twist_type()},
        "nav_msgs.msg": {"Odometry": type("Odometry", (), {})},
        "sensor_msgs.msg": {
            "BatteryState": type("BatteryState", (), {}),
            "LaserScan": type("LaserScan", (), {}),
        },
        "std_srvs.srv": {"Trigger": type("Trigger", (), {"Request": type("Request", (), {})})},
    }
    for module_name, attrs in message_modules.items():
        module = ModuleType(module_name)
        for attr, value in attrs.items():
            setattr(module, attr, value)
        monkeypatch.setitem(sys.modules, module_name, module)
        package_name = module_name.rsplit(".", 1)[0]
        monkeypatch.setitem(sys.modules, package_name, ModuleType(package_name))
    return node


def _fake_twist_type():
    class Twist:
        def __init__(self):
            self.linear = SimpleNamespace(x=0.0)
            self.angular = SimpleNamespace(z=0.0)

    return Twist


def test_ros_client_does_not_create_cmd_vel_publisher_during_init(monkeypatch):
    node = install_fake_ros_modules(monkeypatch)

    client = RVRROSClient()

    assert node.created_publishers == []
    assert client._cmd_pub is None
    assert client.status.cmd_vel_available is False
    assert client.status.cmd_vel_publisher_count == 0


def test_ros_client_enables_velocity_publisher_once_before_publishing(monkeypatch):
    node = install_fake_ros_modules(monkeypatch)
    client = RVRROSClient()

    client.publish_velocity(0.0, 0.0)
    assert client.status.diagnostic_message == "cmd_vel publisher not enabled; zero velocity skipped"
    assert node.created_publishers == []

    with pytest.raises(RuntimeError, match="cmd_vel publisher is not enabled"):
        client.publish_velocity(0.1, 0.0)

    publisher = client.enable_velocity_publisher()
    assert client.enable_velocity_publisher() is publisher
    assert len(node.created_publishers) == 1
    assert node.created_publishers[0][1:3] == ("cmd_vel", 10)

    client.publish_velocity(0.1, -0.2)

    assert len(publisher.messages) == 1
    assert publisher.messages[0].linear.x == pytest.approx(0.1)
    assert publisher.messages[0].angular.z == pytest.approx(-0.2)


def test_status_lines_render_graph_readiness_and_sensor_summaries():
    status = RVRStatus(
        connected=True,
        diagnostic_message="driver ready",
        battery_percentage=0.82,
        battery_voltage=7.31,
        battery_received_at=10.0,
        odom_received_at=9.5,
        odom_x=1.2,
        odom_y=-0.4,
        odom_yaw=0.75,
        odom_distance_m=1.26,
        scan_received_at=9.0,
        scan_range_count=720,
        scan_valid_count=700,
        scan_min_range=0.18,
        scan_max_range=6.50,
        cmd_vel_available=True,
        cmd_vel_publisher_count=1,
        service_available={"/stop": True, "/estop": False, "/clear_estop": True},
        tf_available={"odom->base_link": True, "base_link->laser": False, "map->odom": None},
    )

    lines = format_status_lines(status, armed=False, speed=0.1, turn=0.4, now=10.5)

    assert lines[:2] == ["RVR Control Console", "────────────────────────────────────────"]
    assert "RVR driver: present    /cmd_vel: available (publishers=1)" in lines
    assert "Battery: 82% / 7.31 V (fresh 0.5s)" in lines
    assert "Odom: fresh 1.0s pose=(1.20, -0.40, yaw=0.75) distance=1.26 m" in lines
    assert "Scan: fresh 1.5s ranges=720 valid=700 min=0.18 m max=6.50 m" in lines
    assert "Services: /stop ok  /estop missing  /clear_estop ok" in lines
    assert "TF: odom->base_link ok  base_link->laser missing  map->odom waiting" in lines
    assert "Diagnostics: driver ready" in lines


def test_status_lines_render_waiting_and_stale_values():
    status = RVRStatus(
        battery_percentage=0.5,
        battery_voltage=7.0,
        battery_received_at=0.0,
        odom_received_at=None,
        scan_received_at=1.0,
        scan_range_count=0,
        cmd_vel_available=False,
        service_available={},
        tf_available={},
    )

    lines = format_status_lines(status, armed=True, speed=0.2, turn=0.5, now=6.5)

    assert "RVR driver: missing    /cmd_vel: not-exposed (publishers=0)" in lines
    assert "Battery: 50% / 7.00 V (stale 6.5s)" in lines
    assert "Odom: waiting" in lines
    assert "Scan: stale 5.5s ranges=0 valid=0" in lines
    assert "Services: /stop missing  /estop missing  /clear_estop missing" in lines
    assert "Armed: True    Estop: False" in lines


def test_status_lines_render_stale_sensor_timestamps():
    status = RVRStatus(
        odom_received_at=1.0,
        odom_x=0.0,
        odom_y=0.0,
        odom_yaw=0.0,
        odom_distance_m=0.0,
        scan_received_at=1.0,
        scan_range_count=3,
        scan_valid_count=3,
    )

    lines = format_status_lines(status, armed=False, speed=0.1, turn=0.4, now=5.0)

    assert "Odom: stale 4.0s pose=(0.00, 0.00, yaw=0.00) distance=0.00 m" in lines
    assert "Scan: stale 4.0s ranges=3 valid=3" in lines


def test_battery_odom_and_scan_updates_record_freshness_and_summaries():
    status = RVRStatus()
    battery = SimpleNamespace(percentage=0.25, voltage=7.4)
    odom = SimpleNamespace(
        pose=SimpleNamespace(
            pose=SimpleNamespace(
                position=SimpleNamespace(x=3.0, y=4.0),
                orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
            )
        )
    )
    scan = SimpleNamespace(ranges=[float("inf"), 2.5, float("nan"), 0.2], range_min=0.1, range_max=8.0)

    update_battery_status(status, battery, now=1.0)
    update_odom_status(status, odom, now=2.0)
    update_scan_status(status, scan, now=3.0)

    assert status.battery_percentage == 0.25
    assert status.battery_voltage == 7.4
    assert status.battery_received_at == 1.0
    assert status.odom_received_at == 2.0
    assert status.odom_x == pytest.approx(3.0)
    assert status.odom_y == pytest.approx(4.0)
    assert status.odom_yaw == pytest.approx(0.0)
    assert status.odom_distance_m == pytest.approx(5.0)
    assert status.scan_received_at == 3.0
    assert status.scan_range_count == 4
    assert status.scan_valid_count == 2
    assert status.scan_min_range == pytest.approx(0.2)
    assert status.scan_max_range == pytest.approx(2.5)


def test_live_ros_client_defers_cmd_vel_publisher_until_enabled(monkeypatch):
    node = install_fake_ros_modules(monkeypatch)

    client = RVRROSClient()

    assert node.created_publishers == []
    assert client.velocity_publisher_enabled is False
    with pytest.raises(RuntimeError, match="velocity publisher is not enabled"):
        client.publish_velocity(0.1, 0.0)

    client.enable_velocity_publisher()
    client.enable_velocity_publisher()
    client.publish_velocity(0.2, -0.3)

    assert len(node.created_publishers) == 1
    _msg_type, topic, qos, publisher = node.created_publishers[0]
    assert (topic, qos) == ("cmd_vel", 10)
    assert publisher.messages[-1].linear.x == pytest.approx(0.2)
    assert publisher.messages[-1].angular.z == pytest.approx(-0.3)


def test_live_ros_client_status_does_not_expose_cmd_vel_before_enablement(monkeypatch):
    node = install_fake_ros_modules(monkeypatch)
    client = RVRROSClient()

    client._refresh_graph_status()

    assert "/cmd_vel" in node.topics
    assert client.status.cmd_vel_available is False
    assert client.status.cmd_vel_publisher_count == 0

    client.enable_velocity_publisher()
    client._refresh_graph_status()

    assert client.status.cmd_vel_available is True
    assert client.status.cmd_vel_publisher_count == 1


def test_trigger_service_wait_does_not_spin_node_already_spun_by_background_thread(monkeypatch):
    install_fake_ros_modules(monkeypatch)
    client = RVRROSClient()
    future = DelayedTriggerFuture()
    trigger_client = SimpleNamespace(
        srv_name="stop",
        wait_for_service=lambda timeout_sec: True,
        call_async=lambda request: future,
    )

    def fail_if_nested_spin(node, timeout_sec=0.1):
        raise RuntimeError("Executor is already spinning")

    client._rclpy.spin_once = fail_if_nested_spin
    monkeypatch.setattr("sphero_rvr_driver.tui_ros.time.sleep", lambda _seconds: None)

    assert client._call_trigger(trigger_client, timeout_sec=0.2) == "service ok"
