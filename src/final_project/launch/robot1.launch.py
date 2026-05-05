from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='final_project',
            executable='camera_server',
            name='camera_server',
            output='screen',
        ),
        Node(
            package='final_project',
            executable='robot1_master',
            name='robot1_master',
            output='screen',
        ),
    ])
