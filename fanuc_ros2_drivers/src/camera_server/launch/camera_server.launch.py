import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Inject fanuc_ros2_drivers into PYTHONPATH so mvsdk and pip_test are
# importable without sourcing setup_ws.bash.  Assumes launch is run from
# the workspace root (same directory as the .env file).
_fanuc_root = os.path.join(os.getcwd(), 'fanuc_ros2_drivers')
_extra_pythonpath = _fanuc_root + ':' + os.environ.get('PYTHONPATH', '')


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('modbus_host',     default_value='localhost',
                              description='Modbus server hostname or IP'),
        DeclareLaunchArgument('modbus_port',     default_value='1502'),
        DeclareLaunchArgument('poll_interval',   default_value='0.1'),
        DeclareLaunchArgument('camera_index',    default_value='0'),
        DeclareLaunchArgument('capture_timeout', default_value='5.0'),

        Node(
            package='camera_server',
            executable='camera_server_node',
            name='camera_server_node',
            output='screen',
            parameters=[{
                'modbus_host':     LaunchConfiguration('modbus_host'),
                'modbus_port':     LaunchConfiguration('modbus_port'),
                'poll_interval':   LaunchConfiguration('poll_interval'),
                'camera_index':    LaunchConfiguration('camera_index'),
                'capture_timeout': LaunchConfiguration('capture_timeout'),
            }],
            additional_env={'PYTHONPATH': _extra_pythonpath},
        ),
    ])
