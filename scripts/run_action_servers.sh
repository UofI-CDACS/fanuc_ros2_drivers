#!/bin/bash
source /opt/ros/jazzy/setup.bash
source "$(dirname "$0")/../../install/setup.bash"
ros2 launch action_servers action_servers.launch.py robot_name:=fanuc robot_ip:=10.8.4.16
