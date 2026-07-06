from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port = LaunchConfiguration('port')
    baud_rate = LaunchConfiguration('baud_rate')
    rtscts = LaunchConfiguration('rtscts')
    debug_serial = LaunchConfiguration('debug_serial')
    cmd_vel_max_output = LaunchConfiguration('cmd_vel_max_output')
    cmd_timeout_sec = LaunchConfiguration('cmd_timeout_sec')
    enable_legacy_topics = LaunchConfiguration('enable_legacy_topics')
    enable_cmd_vel = LaunchConfiguration('enable_cmd_vel')

    return LaunchDescription([
        DeclareLaunchArgument(
            'port',
            default_value='/dev/ttyS0',
            description='Serial port used by the Guppy driver.'),
        DeclareLaunchArgument(
            'baud_rate',
            default_value='115200',
            description='Serial baud rate used by the Guppy driver.'),
        DeclareLaunchArgument(
            'rtscts',
            default_value='true',
            description='Enable RTS/CTS serial flow control.'),
        DeclareLaunchArgument(
            'debug_serial',
            default_value='false',
            description='Enable serial packet debug logging.'),
        DeclareLaunchArgument(
            'cmd_vel_max_output',
            default_value='400',
            description='Maximum Guppy command generated from normalized cmd_vel.'),
        DeclareLaunchArgument(
            'cmd_timeout_sec',
            default_value='0.5',
            description='Seconds before a missing command stream is stopped.'),
        DeclareLaunchArgument(
            'enable_legacy_topics',
            default_value='true',
            description='Enable legacy Guppy_Transmitted topic subscription.'),
        DeclareLaunchArgument(
            'enable_cmd_vel',
            default_value='true',
            description='Enable /guppy/cmd_vel Twist subscription.'),
        Node(
            package='guppy',
            executable='guppy_driver_control',
            name='guppy_driver_control',
            output='screen',
            parameters=[{
                'port': port,
                'baud_rate': baud_rate,
                'rtscts': rtscts,
                'debug_serial': debug_serial,
                'cmd_vel_max_output': cmd_vel_max_output,
                'cmd_timeout_sec': cmd_timeout_sec,
                'enable_legacy_topics': enable_legacy_topics,
                'enable_cmd_vel': enable_cmd_vel,
            }],
        ),
    ])
