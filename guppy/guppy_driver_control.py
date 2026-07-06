"""ROS 2 serial driver for the Guppy ROV MCU."""

from dataclasses import dataclass
from typing import Iterable, Sequence

from geometry_msgs.msg import Twist
from guppy_interface.msg import GuppyDriverReceived
from guppy_interface.msg import GuppyDriverTransmitted
import rclpy
from rclpy.node import Node
import serial


@dataclass
class CommandState:
    """Track command timeout state for the active MCU command stream."""

    timeout_sec: float
    last_command_time: object = None
    timed_out: bool = True

    def mark_command(self, now):
        """Remember that a fresh command was sent."""
        self.last_command_time = now
        self.timed_out = False

    def should_stop(self, now):
        """Return true once when the command stream exceeds the timeout."""
        if self.timeout_sec <= 0.0 or self.last_command_time is None:
            return False

        age = (now - self.last_command_time).nanoseconds / 1_000_000_000.0
        if age <= self.timeout_sec or self.timed_out:
            return False

        self.timed_out = True
        return True


class GuppySerialProtocol:
    """Encode MCU commands and decode MCU telemetry packets."""

    transmitted_header = [0xAB, 0xCD]
    received_header = [0xDC, 0xBA]
    received_packet_size = 14
    command_min = -400
    command_max = 400

    @staticmethod
    def hex_byte(value):
        """Parse a byte from an int, bytes, or hex string field."""
        if isinstance(value, int):
            return value & 0xFF
        if isinstance(value, bytes):
            value = value.replace(b'\x00', b'').strip()
        elif isinstance(value, str):
            value = value.replace('\x00', '').strip()

        parsed = int(value, 16)
        if not 0 <= parsed <= 0xFF:
            raise ValueError(f'hex byte out of range: {value!r}')

        return parsed

    @staticmethod
    def int16_to_bytes(value):
        """Convert a signed integer into two big-endian two's-complement bytes."""
        value = int(value)
        if value < 0:
            value += 1 << 16

        return [(value >> 8) & 0xFF, value & 0xFF]

    @classmethod
    def bytes_to_int16(cls, values):
        """Convert two big-endian two's-complement bytes into a signed integer."""
        value = (cls.hex_byte(values[0]) << 8) + cls.hex_byte(values[1])
        if value & (1 << 15):
            value -= 1 << 16

        return value

    @classmethod
    def encode_command(cls, msg):
        """Encode a Guppy command message for the MCU serial protocol."""
        payload = (
            cls.int16_to_bytes(cls.motion_command_value(msg.surge))
            + cls.int16_to_bytes(cls.motion_command_value(msg.sway))
            + cls.int16_to_bytes(cls.motion_command_value(msg.yaw))
            + cls.int16_to_bytes(cls.motion_command_value(msg.heave))
            + cls.uint8_fields([msg.s1, msg.s2, msg.o1, msg.o2, msg.o3, msg.o4])
        )
        packet = cls.transmitted_header + payload
        packet.append(sum(packet) & 0xFF)
        return bytes(packet)

    @classmethod
    def motion_command_value(cls, value):
        """Clamp a motion command to the MCU-supported command range."""
        return max(cls.command_min, min(cls.command_max, int(value)))

    @staticmethod
    def uint8_fields(values):
        """Convert command aux fields into unsigned bytes."""
        return [int(value) & 0xFF for value in values]

    @classmethod
    def parse_received_line(cls, line):
        """Parse and validate one newline-delimited MCU telemetry line."""
        fields = line.split()
        if len(fields) < cls.received_packet_size:
            raise ValueError(f'short packet with {len(fields)} fields: {line!r}')

        packet = [cls.hex_byte(field) for field in fields[:cls.received_packet_size]]
        if packet[:2] != cls.received_header:
            raise ValueError(f'invalid header: {packet[:2]!r}')

        calculated_sum = sum(packet[:13]) & 0xFF
        if calculated_sum != packet[13]:
            raise ValueError(
                f'bad checksum: calculated=0x{calculated_sum:02x} '
                f'received=0x{packet[13]:02x}')

        return packet

    @classmethod
    def received_packet_to_msg(cls, packet):
        """Convert a validated MCU telemetry packet into a ROS message."""
        msg = GuppyDriverReceived()
        msg.roll = cls.bytes_to_int16(packet[2:4])
        msg.pitch = cls.bytes_to_int16(packet[4:6])
        msg.yaw = cls.bytes_to_int16(packet[6:8])
        msg.depth = packet[8]
        msg.water = packet[9]
        msg.volt = packet[10]
        msg.amp = cls.bytes_to_int16(packet[11:13])
        return msg

    @staticmethod
    def zero_command():
        """Build a neutral command message."""
        return GuppyDriverTransmitted()


class CmdVelAdapter:
    """Convert normalized Twist commands into Guppy MCU command messages."""

    def __init__(self, max_output):
        self.max_output = GuppySerialProtocol.motion_command_value(abs(int(max_output)))

    def to_guppy_command(self, twist):
        """Map AUV-style Twist axes into the Guppy command message."""
        msg = GuppyDriverTransmitted()
        msg.surge = self.scale_axis(twist.linear.x)
        msg.sway = self.scale_axis(twist.linear.y)
        msg.heave = self.scale_axis(twist.linear.z)
        msg.yaw = self.scale_axis(twist.angular.z)
        return msg

    def scale_axis(self, value):
        """Clip a normalized axis and scale it to the configured command range."""
        clipped = max(-1.0, min(1.0, float(value)))
        return int(round(clipped * self.max_output))


class GuppyDriverNode(Node):
    """Bridge ROS command and telemetry topics to the Guppy MCU serial protocol."""

    def __init__(self):
        super().__init__('guppy_driver_control')
        self.declare_parameter('port', '/dev/ttyS0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('rtscts', True)
        self.declare_parameter('debug_serial', False)
        self.declare_parameter('cmd_vel_max_output', GuppySerialProtocol.command_max)
        self.declare_parameter('cmd_timeout_sec', 0.5)
        self.declare_parameter('enable_legacy_topics', True)
        self.declare_parameter('enable_cmd_vel', True)

        port = self.get_parameter('port').get_parameter_value().string_value
        baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        rtscts = self.get_parameter('rtscts').get_parameter_value().bool_value
        max_output = self.get_parameter(
            'cmd_vel_max_output').get_parameter_value().integer_value
        timeout_sec = self.get_parameter(
            'cmd_timeout_sec').get_parameter_value().double_value
        self.debug_serial = self.get_parameter(
            'debug_serial').get_parameter_value().bool_value
        enable_legacy_topics = self.get_parameter(
            'enable_legacy_topics').get_parameter_value().bool_value
        enable_cmd_vel = self.get_parameter(
            'enable_cmd_vel').get_parameter_value().bool_value

        self.protocol = GuppySerialProtocol()
        self.cmd_vel_adapter = CmdVelAdapter(max_output)
        self.command_state = CommandState(timeout_sec)
        self.rx_buffer = bytearray()
        self.ser = serial.Serial(port, baud_rate, rtscts=rtscts, timeout=1)
        self.get_logger().info(f'Opened serial: {port} @ {baud_rate} rtscts={rtscts}')

        self.legacy_telemetry_pub = self.create_publisher(
            GuppyDriverReceived, 'Guppy_Received', 10)
        self.telemetry_pub = self.create_publisher(
            GuppyDriverReceived, '/guppy/telemetry', 10)

        self.timer = self.create_timer(0.05, self.received_callback)
        self.timeout_timer = self.create_timer(0.05, self.timeout_callback)

        self.cmd_raw_subscription = self.create_subscription(
            GuppyDriverTransmitted,
            '/guppy/cmd_raw',
            self.command_callback,
            10)

        self.legacy_subscription = None
        if enable_legacy_topics:
            self.legacy_subscription = self.create_subscription(
                GuppyDriverTransmitted,
                'Guppy_Transmitted',
                self.command_callback,
                10)

        self.cmd_vel_subscription = None
        if enable_cmd_vel:
            self.cmd_vel_subscription = self.create_subscription(
                Twist,
                '/guppy/cmd_vel',
                self.cmd_vel_callback,
                10)

    def cmd_vel_callback(self, msg):
        """Convert a velocity command and send it to the MCU."""
        self.send_command(self.cmd_vel_adapter.to_guppy_command(msg))

    def command_callback(self, msg):
        """Send a raw Guppy command message to the MCU."""
        self.send_command(msg)

    def send_command(self, msg):
        """Encode, write, and track one command for timeout safety."""
        self.ser.write(self.protocol.encode_command(msg))
        self.command_state.mark_command(self.get_clock().now())

    def timeout_callback(self):
        """Send one neutral command when active commands stop arriving."""
        if not self.command_state.should_stop(self.get_clock().now()):
            return

        self.send_command(self.protocol.zero_command())
        self.command_state.timed_out = True
        self.get_logger().warn('Command timeout; sent neutral command')

    def received_callback(self):
        """Read serial telemetry lines and publish valid packets."""
        waiting = self.ser.in_waiting
        if waiting == 0:
            return

        self.rx_buffer.extend(self.ser.read(waiting))

        parsed_count = 0
        last_msg = None
        while b'\n' in self.rx_buffer:
            line, _, rest = self.rx_buffer.partition(b'\n')
            self.rx_buffer = bytearray(rest)
            line = line.strip()
            if not line:
                continue

            try:
                packet = self.protocol.parse_received_line(line)
            except ValueError as exc:
                self.get_logger().warn(
                    f'Dropping serial line: {exc}',
                    throttle_duration_sec=1.0)
                continue

            last_msg = self.protocol.received_packet_to_msg(packet)
            self.legacy_telemetry_pub.publish(last_msg)
            self.telemetry_pub.publish(last_msg)
            parsed_count += 1

        if self.debug_serial and parsed_count:
            self.get_logger().info(
                f'Published {parsed_count} packet(s); latest: {last_msg}',
                throttle_duration_sec=1.0)


def hex_byte(value):
    """Compatibility wrapper for byte parsing."""
    return GuppySerialProtocol.hex_byte(value)


def twos_complement_transmitted(value, bit_width=16):
    """Compatibility wrapper for signed command byte conversion."""
    if bit_width != 16:
        raise ValueError('only 16-bit conversion is supported')

    return GuppySerialProtocol.int16_to_bytes(value)


def twos_complement_received(values, bit_width=16):
    """Compatibility wrapper for signed telemetry conversion."""
    if bit_width != 16:
        raise ValueError('only 16-bit conversion is supported')

    return GuppySerialProtocol.bytes_to_int16(values)


def received_packet_to_msg(packet):
    """Compatibility wrapper for telemetry message conversion."""
    return GuppySerialProtocol.received_packet_to_msg(packet)


def parse_received_line(line):
    """Compatibility wrapper for telemetry line parsing."""
    return GuppySerialProtocol.parse_received_line(line)


def command_from_values(
    surge=0,
    sway=0,
    yaw=0,
    heave=0,
    aux: Iterable[int] = (),
):
    """Build a Guppy command message from scalar values."""
    aux_values = list(aux) + [0] * 6
    msg = GuppyDriverTransmitted()
    msg.surge = int(surge)
    msg.sway = int(sway)
    msg.yaw = int(yaw)
    msg.heave = int(heave)
    msg.s1 = int(aux_values[0])
    msg.s2 = int(aux_values[1])
    msg.o1 = int(aux_values[2])
    msg.o2 = int(aux_values[3])
    msg.o3 = int(aux_values[4])
    msg.o4 = int(aux_values[5])
    return msg


def checksum(values: Sequence[int]):
    """Return the low byte checksum used by the MCU serial protocol."""
    return sum(values) & 0xFF


def main(args=None):
    """Run the Guppy ROS 2 serial driver node."""
    rclpy.init(args=args)
    guppy_driver = GuppyDriverNode()

    try:
        rclpy.spin(guppy_driver)
    finally:
        guppy_driver.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
