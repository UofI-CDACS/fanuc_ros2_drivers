#!/bin/bash
source /opt/ros/jazzy/setup.bash
source "$(dirname "$0")/../../install/setup.bash"
python3 "$(dirname "$0")/dice_inspection_system.py" --ros-args -p robot_name:=fanuc -p robot_ip:=10.8.4.16 --params-file "$(dirname "$0")/../../config/robot_poses.yaml"
