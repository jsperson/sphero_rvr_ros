import struct

from sphero_rvr_core.commands import RVRCommands
from sphero_rvr_core.packet import DID_DRIVE, DID_IR, DID_POWER, DID_SENSOR, DID_SYSTEM_INFO, TARGET_BT, TARGET_MCU


def assert_request_response(packet):
    assert packet.flags & 0x02


def test_extended_battery_query_builders():
    commands = RVRCommands()

    voltage = commands.get_battery_voltage(1)
    state = commands.get_battery_voltage_state(2)
    thresholds = commands.get_battery_thresholds(3)

    assert voltage.device_id == DID_POWER
    assert voltage.command_id == RVRCommands.CID_GET_BATTERY_VOLTAGE
    assert voltage.target == TARGET_BT
    assert voltage.payload == bytes([0])
    assert_request_response(voltage)

    assert state.command_id == RVRCommands.CID_GET_BATTERY_VOLTAGE_STATE
    assert thresholds.command_id == RVRCommands.CID_GET_BATTERY_THRESHOLDS
    assert_request_response(state)
    assert_request_response(thresholds)


def test_color_and_light_sensor_builders():
    commands = RVRCommands()

    rgbc = commands.get_rgbc_sensor_values(4)
    enable = commands.enable_color_detection(5, enabled=True)
    notify = commands.enable_color_detection_notify(6, enabled=False, interval_ms=250, confidence=7)
    current = commands.get_current_detected_color(7)

    assert rgbc.device_id == DID_SENSOR
    assert rgbc.command_id == RVRCommands.CID_GET_RGBC_SENSOR
    assert rgbc.target == TARGET_BT
    assert_request_response(rgbc)

    assert enable.payload == bytes([1])
    assert enable.command_id == RVRCommands.CID_ENABLE_COLOR_DETECTION
    assert_request_response(enable)

    assert notify.payload == struct.pack(">BHB", 0, 250, 7)
    assert notify.command_id == RVRCommands.CID_ENABLE_COLOR_DETECTION_NOTIFY

    assert current.command_id == RVRCommands.CID_GET_CURRENT_DETECTED_COLOR
    assert_request_response(current)


def test_temperature_motion_sensor_and_motor_protection_builders():
    commands = RVRCommands()

    temp = commands.get_temperature(8)
    thermal = commands.get_thermal_protection_status(9)
    encoders = commands.get_encoder_counts(10)
    mag = commands.get_magnetometer(11)
    calibrate = commands.calibrate_magnetometer(12)
    motor_fault = commands.get_motor_fault_state(13)
    stall_notify = commands.enable_motor_stall_notify(14, enabled=False)
    fault_notify = commands.enable_motor_fault_notify(15, enabled=True)

    assert temp.device_id == DID_SENSOR
    assert temp.target == TARGET_MCU
    assert temp.payload == bytes([4, 5])
    assert_request_response(temp)

    assert thermal.command_id == RVRCommands.CID_GET_THERMAL_PROTECTION
    assert encoders.command_id == RVRCommands.CID_GET_ENCODER_COUNTS
    assert mag.command_id == RVRCommands.CID_GET_MAGNETOMETER
    assert calibrate.command_id == RVRCommands.CID_CALIBRATE_MAGNETOMETER

    assert motor_fault.device_id == DID_DRIVE
    assert motor_fault.command_id == RVRCommands.CID_GET_MOTOR_FAULT
    assert_request_response(motor_fault)
    assert stall_notify.payload == bytes([0])
    assert fault_notify.payload == bytes([1])


def test_drive_to_position_si_uses_simplified_error_only_packet_shape():
    packet = RVRCommands().drive_to_position_si(
        sequence_id=29,
        yaw_angle=90.0,
        x=1.25,
        y=-0.5,
        linear_speed=0.75,
        flags=2,
    )

    assert packet.device_id == DID_DRIVE
    assert packet.command_id == RVRCommands.CID_DRIVE_TO_POSITION_SI
    assert packet.sequence_id == 29
    assert packet.target is None
    assert packet.flags == 0x06
    assert packet.payload == struct.pack(">ffffB", 90.0, 1.25, -0.5, 0.75, 2)


def test_system_info_query_builders():
    commands = RVRCommands()

    version = commands.get_main_app_version(16, target=TARGET_MCU)
    board = commands.get_board_revision(17)
    processor = commands.get_processor_name(18)
    sku = commands.get_sku(19)
    uptime = commands.get_core_uptime(20, target=TARGET_MCU)

    assert version.device_id == DID_SYSTEM_INFO
    assert version.command_id == RVRCommands.CID_GET_MAIN_APP_VERSION
    assert version.target == TARGET_MCU
    assert_request_response(version)
    assert board.command_id == RVRCommands.CID_GET_BOARD_REVISION
    assert processor.command_id == RVRCommands.CID_GET_PROCESSOR_NAME
    assert sku.command_id == RVRCommands.CID_GET_SKU
    assert uptime.command_id == RVRCommands.CID_GET_CORE_UPTIME


def test_ir_builders_validate_and_pack_payloads():
    commands = RVRCommands()

    send = commands.send_ir_message(21, code=3, strength=32)
    start_broadcast = commands.start_ir_broadcast(22, far_code=1, near_code=2)
    stop_broadcast = commands.stop_ir_broadcast(23)
    follow = commands.start_ir_following(24, far_code=4, near_code=5)
    stop_follow = commands.stop_ir_following(25)
    evade = commands.start_ir_evading(26, far_code=6, near_code=7)
    stop_evade = commands.stop_ir_evading(27)
    readings = commands.get_ir_readings(28)

    assert send.device_id == DID_IR
    assert send.command_id == RVRCommands.CID_SEND_IR_MESSAGE
    assert send.payload == bytes([3, 32])
    assert start_broadcast.payload == bytes([1, 2])
    assert stop_broadcast.command_id == RVRCommands.CID_STOP_IR_BROADCAST
    assert follow.payload == bytes([4, 5])
    assert stop_follow.command_id == RVRCommands.CID_STOP_IR_FOLLOWING
    assert evade.payload == bytes([6, 7])
    assert stop_evade.command_id == RVRCommands.CID_STOP_IR_EVADING
    assert readings.device_id == DID_SENSOR
    assert_request_response(readings)
