"""
Launch file for the dice inspection nodes.

Usage:
  ros2 launch dice_inspection dice_inspection.launch.py robot_name:=fanuc

The robot_name must match what was used to launch the action_servers.
Run the existing drivers first:
  ros2 launch launch/start.launch.py robot_name:=fanuc robot_ip:=<IP>

Then launch this file in a separate terminal.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='fanuc',
        description='Robot name — must match the action_servers launch argument'
    )

    camera_node = Node(
        package='dice_inspection',
        executable='camera_node',
        name='camera_node',
        output='screen',
    )

    master_node = Node(
        package='dice_inspection',
        executable='master_node',
        name='master_node',
        output='screen',
        parameters=[{
            'robot_name': LaunchConfiguration('robot_name'),
        }],
    )

    return LaunchDescription([
        robot_name_arg,
        camera_node,
        master_node,
    ])
