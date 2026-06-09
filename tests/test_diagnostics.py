from sphero_rvr_driver.diagnostics import BatterySnapshot, battery_state_fields, summarize_state
from sphero_rvr_core.state import RVRState


def test_battery_state_fields_convert_percent_and_voltage_for_ros_message():
    fields = battery_state_fields(BatterySnapshot(percentage=40, voltage=7.403))

    assert fields.percentage == 0.40
    assert fields.voltage == 7.403
    assert fields.present is True


def test_battery_state_fields_clamp_percentage_to_ros_fraction():
    assert battery_state_fields(BatterySnapshot(percentage=150)).percentage == 1.0
    assert battery_state_fields(BatterySnapshot(percentage=-10)).percentage == 0.0


def test_diagnostics_report_emergency_stop_before_connected_state():
    summary = summarize_state(RVRState(connected=False, emergency_stopped=True))

    assert summary.level == "ERROR"
    assert "emergency stop" in summary.message
