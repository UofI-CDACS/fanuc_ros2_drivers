"""
robot1.launch.py  —  runs on Beaker's machine

Starts:
  camera_server  (dice_game)  — only one allowed on the whole network
  robot1_ctrl    (dice_game)  — Beaker's state machine

Usage:
  ros2 launch dice_game robot1.launch.py robot_name:=Beaker robot_ip:=<ip> modbus_ip:=<bunsen_ip>
"""

import sys
from launch import LaunchDescription
from launch_ros.actions import Node

robot_name = 'Beaker'
robot_ip   = '0.0.0.0'
modbus_ip  = '0.0.0.0'   # Bunsen's IP for Modbus TCP

for arg in sys.argv:
    if arg.startswith('robot_name:='):
        robot_name = arg.split(':=')[1]
    elif arg.startswith('robot_ip:='):
        robot_ip = arg.split(':=')[1]
    elif arg.startswith('modbus_ip:='):
        modbus_ip = arg.split(':=')[1]


def generate_launch_description():
    camera_server = Node(
        package='dice_game',
        executable='camera_server',
        name='dice_camera_server',
        output='screen',
    )

    robot1_ctrl = Node(
        package='dice_game',
        executable='robot1_ctrl',
        name='robot1_controller',
        output='screen',
        parameters=[
            {'robot_name': robot_name},
            {'robot_ip':   robot_ip},
            {'modbus_ip':  modbus_ip},
        ],
    )

    return LaunchDescription([camera_server, robot1_ctrl])
