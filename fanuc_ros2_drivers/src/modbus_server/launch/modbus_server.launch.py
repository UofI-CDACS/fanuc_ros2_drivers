from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('host',         default_value='0.0.0.0',
                              description='Bind address (0.0.0.0 = all interfaces)'),
        DeclareLaunchArgument('port',         default_value='1502'),
        DeclareLaunchArgument('log_interval', default_value='10.0'),

        Node(
            package='modbus_server',
            executable='modbus_server_node',
            name='modbus_server_node',
            output='screen',
            parameters=[{
                'host':         LaunchConfiguration('host'),
                'port':         LaunchConfiguration('port'),
                'log_interval': LaunchConfiguration('log_interval'),
            }],
        ),
    ])
