"""
dice_game.launch.py

Launches the three game nodes that run on the laptop:
  - camera_server  : captures images and counts pips
  - dj_control     : DJ (Robot 1, LEFT)  state machine
  - bill_control   : BILL (Robot 2, RIGHT) state machine

The FANUC driver nodes (action_servers + msg_publishers) for each robot
are started separately in their own terminals, e.g.:

  Terminal 1 (DJ drivers):
    ros2 launch action_servers action_servers.launch.py \
      robot_name:=DJ robot_ip:=10.8.4.16

  Terminal 2 (DJ publishers):
    ros2 launch msg_publishers message_publishers.launch.py \
      robot_name:=DJ robot_ip:=10.8.4.16

  Terminal 3 (BILL drivers):
    ros2 launch action_servers action_servers.launch.py \
      robot_name:=BILL robot_ip:=10.8.4.6

  Terminal 4 (BILL publishers):
    ros2 launch msg_publishers message_publishers.launch.py \
      robot_name:=BILL robot_ip:=10.8.4.6

  Terminal 5 (game):
    ros2 launch dice_game dice_game.launch.py
"""

import launch
from launch_ros.actions import Node


def generate_launch_description():
    dj_node = Node(
        package='dice_game',
        executable='dj_control',
        name='dj_control',
        output='screen',
    )

    bill_node = Node(
        package='dice_game',
        executable='bill_control',
        name='bill_control',
        output='screen',
    )

    return launch.LaunchDescription([
        dj_node,
        bill_node,
    ])
