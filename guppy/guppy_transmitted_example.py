from guppy_interface.msg import GuppyDriverTransmitted
import rclpy
from rclpy.node import Node


class GuppyTransmittedExample(Node):
    """Publish a simple oscillating Guppy command for driver testing."""

    def __init__(self):
        super().__init__('guppy_transmitted_example')
        self.declare_parameter('rate_hz', 5.0)
        self.declare_parameter('step', 10)

        rate_hz = self.get_parameter('rate_hz').get_parameter_value().double_value
        self.step = self.get_parameter('step').get_parameter_value().integer_value
        self.value = -200
        self.direction = 1

        self.publisher = self.create_publisher(
            GuppyDriverTransmitted,
            'Guppy_Transmitted',
            10)
        self.timer = self.create_timer(1.0 / rate_hz, self.publish_command)

    def publish_command(self):
        """Publish the next sample command."""
        msg = GuppyDriverTransmitted()
        msg.surge = self.value
        msg.sway = self.value
        msg.yaw = self.value
        msg.heave = self.value
        msg.s1 = 0
        msg.s2 = 0
        msg.o1 = 0
        msg.o2 = 0
        msg.o3 = 0
        msg.o4 = 0

        self.publisher.publish(msg)
        self.get_logger().info(
            f'Published surge/sway/yaw/heave={self.value}',
            throttle_duration_sec=1.0)

        self.value += self.step * self.direction
        if self.value >= 200:
            self.value = 200
            self.direction = -1
        elif self.value <= -200:
            self.value = -200
            self.direction = 1


def main(args=None):
    """Run the example command publisher."""
    rclpy.init(args=args)
    node = GuppyTransmittedExample()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
