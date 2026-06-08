from sphero_rvr_core.commands import RVRCommands


def test_drive_rc_maps_forward_velocity_to_raw_motors():
    packet = RVRCommands().drive_rc(sequence_id=1, linear_mps=0.5, angular_rad_s=0.0)

    assert packet.command_id == RVRCommands.CID_RAW_MOTORS
    assert packet.payload == bytes([1, 127, 1, 127])


def test_drive_rc_mixes_linear_and_angular_velocity_into_tank_tracks():
    packet = RVRCommands().drive_rc(sequence_id=2, linear_mps=0.5, angular_rad_s=0.25)

    assert packet.command_id == RVRCommands.CID_RAW_MOTORS
    assert packet.payload == bytes([1, 191, 1, 63])


def test_drive_rc_normalizes_tracks_and_encodes_reverse_mode():
    packet = RVRCommands().drive_rc(sequence_id=3, linear_mps=-0.25, angular_rad_s=1.0)

    assert packet.command_id == RVRCommands.CID_RAW_MOTORS
    assert packet.payload == bytes([1, 153, 2, 255])
