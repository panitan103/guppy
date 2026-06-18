import rclpy
from rclpy.node import Node
import serial

from guppy_interface.msg import GuppyDriverTransmitted,GuppyDriverReceived

RECEIVED_HEADER = [0xDC, 0xBA]
RECEIVED_PACKET_SIZE = 14


def hex_byte(value):
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
                       
def twos_complement_transmitted(value, bit_width=16):

    """Compute the 2's complement of int value with bit_width bits."""
    if value < 0:
        # If sign bit is set, compute negative value
        value += 1 << bit_width
    out=[((value>>8&0xFF)),((value&0xFF))]
    return out
def twos_complement_received(input, bit_width=16):
    value=(hex_byte(input[0])<<8)+hex_byte(input[1])
    """Compute the 2's complement of int value with bit_width bits."""
    if value & (1 << (bit_width - 1)):
        # If sign bit is set, compute negative value
        value -= 1 << bit_width
    return value


def received_packet_to_msg(packet):
    msg = GuppyDriverReceived()
    msg.roll = twos_complement_received(packet[2:4])
    msg.pitch = twos_complement_received(packet[4:6])
    msg.yaw = twos_complement_received(packet[6:8])
    msg.depth = packet[8]
    msg.water = packet[9]
    msg.volt = packet[10]
    msg.amp = twos_complement_received(packet[11:13])
    return msg


def parse_received_line(line):
    fields = line.split()
    if len(fields) < RECEIVED_PACKET_SIZE:
        raise ValueError(f'short packet with {len(fields)} fields: {line!r}')

    packet = [hex_byte(field) for field in fields[:RECEIVED_PACKET_SIZE]]
    if packet[:2] != RECEIVED_HEADER:
        raise ValueError(f'invalid header: {packet[:2]!r}')

    cal_sum = sum(packet[:13]) & 0xff
    if cal_sum != packet[13]:
        raise ValueError(
            f'bad checksum: calculated=0x{cal_sum:02x} received=0x{packet[13]:02x}')

    return packet

class GuppyTransmitted(Node):

    def __init__(self):
        super().__init__('GuppyTransmitted')
        self.declare_parameter('port', '/dev/ttyS0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('rtscts', True)
        self.declare_parameter('debug_serial', False)
        
        port = self.get_parameter('port').get_parameter_value().string_value
        baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        rtscts = self.get_parameter('rtscts').get_parameter_value().bool_value
        self.debug_serial = self.get_parameter('debug_serial').get_parameter_value().bool_value
        self.ser = serial.Serial(port, baud_rate, rtscts=rtscts, timeout=1)
        self.rx_buffer = bytearray()
        self.get_logger().info(f'Opened serial: {port} @ {baud_rate} rtscts={rtscts}')

        self.publisher_ = self.create_publisher(GuppyDriverReceived, 'Guppy_Received', 10)
        self.timer = self.create_timer(0.05, self.received_callback)

        self.subscription = self.create_subscription(
            GuppyDriverTransmitted,
            'Guppy_Transmitted',
            self.transmitted_callback,
            10)
        self.subscription  # prevent unused variable warningGuppyDriverReceived

    def transmitted_callback(self, msg):
        # self.get_logger().info('I heard: "%s"' % msg.data)

        Surge=twos_complement_transmitted(int(msg.surge),16)
        Sway=twos_complement_transmitted(int(msg.sway),16)
        Yaw=twos_complement_transmitted(int(msg.yaw),16)
        Heave=twos_complement_transmitted(int(msg.heave),16)
        S1=[int(msg.s1)&0xFF]
        S2=[int(msg.s2)&0xFF]
        O1=[int(msg.o1)&0xFF]
        O2=[int(msg.o2)&0xFF]
        O3=[int(msg.o3)&0xFF]
        O4=[int(msg.o4)&0xFF]

        cal_sum_sent=[(sum(value for value in [int('AB',16),int('CD',16)]+Surge+Sway+Yaw+Heave+S1+S2+O1+O2+O3+O4)&0xff)]

        serial_out=([int('AB',16),int('CD',16)]+Surge+Sway+Yaw+Heave+S1+S2+O1+O2+O3+O4+cal_sum_sent)
        self.ser.write(bytes(serial_out))
        print(serial_out)

    def received_callback(self):
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
                packet = parse_received_line(line)
            except ValueError as exc:
                self.get_logger().warn(
                    f'Dropping serial line: {exc}',
                    throttle_duration_sec=1.0)
                continue

            last_msg = received_packet_to_msg(packet)
            self.publisher_.publish(last_msg)
            parsed_count += 1

        if self.debug_serial and parsed_count:
            self.get_logger().info(
                f'Published {parsed_count} packet(s); latest: {last_msg}',
                throttle_duration_sec=1.0)



def main(args=None):
    rclpy.init(args=args)

    guppy_transmitted = GuppyTransmitted()

    rclpy.spin(guppy_transmitted)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    guppy_transmitted.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
