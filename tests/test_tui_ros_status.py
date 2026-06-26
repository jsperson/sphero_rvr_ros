from types import SimpleNamespace

import pytest

from sphero_rvr_driver.tui_ros import (
    RVRStatus,
    format_status_lines,
    update_battery_status,
    update_odom_status,
    update_scan_status,
)


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
