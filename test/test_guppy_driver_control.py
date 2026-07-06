"""Tests for the Guppy MCU serial protocol and command adapters."""

from geometry_msgs.msg import Twist

from guppy.guppy_driver_control import (
    checksum,
    CmdVelAdapter,
    command_from_values,
    CommandState,
    GuppySerialProtocol,
    parse_received_line,
)


class FakeTime:
    """Small stand-in for ROS Time in timeout tests."""

    def __init__(self, nanoseconds):
        self.nanoseconds = nanoseconds

    def __sub__(self, other):
        return FakeTime(self.nanoseconds - other.nanoseconds)


def packet_line(packet):
    """Convert a packet byte list into the MCU's ASCII hex line format."""
    return ' '.join(f'{value:02X}' for value in packet).encode()


def test_encode_command_preserves_wire_format():
    """Commands are encoded with the original header, order, and checksum."""
    msg = command_from_values(
        surge=-200,
        sway=200,
        yaw=-1,
        heave=0,
        aux=[1, 2, 3, 4, 5, 6],
    )

    encoded = list(GuppySerialProtocol.encode_command(msg))
    expected_without_checksum = [
        0xAB,
        0xCD,
        0xFF,
        0x38,
        0x00,
        0xC8,
        0xFF,
        0xFF,
        0x00,
        0x00,
        0x01,
        0x02,
        0x03,
        0x04,
        0x05,
        0x06,
    ]

    assert encoded[:-1] == expected_without_checksum
    assert encoded[-1] == checksum(expected_without_checksum)


def test_parse_received_line_and_convert_to_message():
    """Valid telemetry packets are parsed and converted into ROS messages."""
    packet = [
        0xDC,
        0xBA,
        0xFF,
        0xFF,
        0x00,
        0x02,
        0xFF,
        0x38,
        0x03,
        0x04,
        0x05,
        0xFF,
        0xF6,
    ]
    packet.append(checksum(packet))

    parsed = parse_received_line(packet_line(packet))
    msg = GuppySerialProtocol.received_packet_to_msg(parsed)

    assert msg.roll == -1
    assert msg.pitch == 2
    assert msg.yaw == -200
    assert msg.depth == 3
    assert msg.water == 4
    assert msg.volt == 5
    assert msg.amp == -10


def test_parse_received_line_rejects_bad_checksum():
    """Telemetry packets with bad checksums are rejected."""
    packet = [
        0xDC,
        0xBA,
        0x00,
        0x01,
        0x00,
        0x02,
        0x00,
        0x03,
        0x04,
        0x05,
        0x06,
        0x00,
        0x07,
        0x00,
    ]

    try:
        parse_received_line(packet_line(packet))
    except ValueError as exc:
        assert 'bad checksum' in str(exc)
    else:
        raise AssertionError('bad checksum packet was accepted')


def test_cmd_vel_adapter_clips_and_scales_auv_axes():
    """Normalized Twist axes map to Guppy surge, sway, heave, and yaw."""
    twist = Twist()
    twist.linear.x = 0.5
    twist.linear.y = -2.0
    twist.linear.z = 1.0
    twist.angular.z = -0.25

    msg = CmdVelAdapter(max_output=200).to_guppy_command(twist)

    assert msg.surge == 100
    assert msg.sway == -200
    assert msg.heave == 200
    assert msg.yaw == -50
    assert msg.s1 == 0
    assert msg.o4 == 0


def test_command_state_requests_one_timeout_stop():
    """Timeout safety requests exactly one stop until a new command arrives."""
    state = CommandState(timeout_sec=0.5)
    state.mark_command(FakeTime(0))

    assert state.should_stop(FakeTime(400_000_000)) is False
    assert state.should_stop(FakeTime(600_000_000)) is True
    assert state.should_stop(FakeTime(700_000_000)) is False

    state.mark_command(FakeTime(800_000_000))

    assert state.should_stop(FakeTime(1_400_000_000)) is True
