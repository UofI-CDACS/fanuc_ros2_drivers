"""
robot2.launch.py  —  runs on Robot 2's machine

Starts:
  robot2_ctrl  (dice_game)  — Robot 2's state machine

Note: Robot 2's conveyor action servers are expected to already be running
via the standard start.launch.py for Robot 2's machine.  The conveyor server
names (FRONT_CONV_NAME / BACK_CONV_NAME) must match what is set in
robot2_controller.py.

Usage:
  ros2 launch dice_game robot2.launch.py robot_name:=<name> robot_ip:=<ip>
"""

import sys
from launch import LaunchDescription
from launch_ros.actions import Node

robot_name = 'Robot2'
robot_ip   = '0.0.0.0'   # ← set your actual IP here or pass on CLI

for arg in sys.argv:
    if arg.startswith('robot_name:='):
        robot_name = arg.split(':=')[1]
    elif arg.startswith('robot_ip:='):
        robot_ip = arg.split(':=')[1]


def generate_launch_description():
    robot2_ctrl = Node(
        package='dice_game',
        executable='robot2_ctrl',
        name='robot2_controller',
        output='screen',
        parameters=[
            {'robot_name': robot_name},
            {'robot_ip':   robot_ip},
        ],
    )

    return LaunchDescription([robot2_ctrl])
