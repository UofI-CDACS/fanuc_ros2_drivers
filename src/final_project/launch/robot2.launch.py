from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='final_project',
            executable='robot2_master',
            name='robot2_master',
            output='screen',
        ),
    ])
