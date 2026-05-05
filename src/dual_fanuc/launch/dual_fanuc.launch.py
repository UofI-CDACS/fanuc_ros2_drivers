from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dual_fanuc',
            executable='mv_camera_node',
            name='mv_camera_node',
            output='screen',
        ),
    ])
