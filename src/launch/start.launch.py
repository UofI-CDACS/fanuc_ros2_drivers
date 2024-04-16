from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution
import sys

name = sys.argv[1]
ip = sys.argv[2]

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('action_servers'),
                    'launch',
                    'action_servers.launch.py'
                ])
            ]),
            launch_arguments={
                'robot_name': name,
                'robot_ip': ip,
            }.items()
        )
    ])