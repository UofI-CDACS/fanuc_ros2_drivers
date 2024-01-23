#!/bin/bash
# This is a *temporary* 'launch' file to initalize all ROS nodes for a robot. Requires IP address.
# To run in the terminal: ./startRobot.sh XXX.XXX.X.XXX YYY.YYY.Y.YYY # Y and beyond is optional 
# Replace X's (and optional Y's) with Robot's IP(s)


if [ $# -eq 0 ];
then
    echo "Need at least 1 IP address..."
    exit 1
fi
# source the overlay    
source install/setup.sh

# For each IP that was given
for i in "$@"
do
# These will all run in the background
    # Actions
    python3 src/action_servers/action_servers/cart_pose_server.py $i &
    python3 src/action_servers/action_servers/convey_server.py $i &
    python3 src/action_servers/action_servers/joint_pose_server.py $i &
    python3 src/action_servers/action_servers/schunk_server.py $i &
    python3 src/action_servers/action_servers/single_joint_server.py $i &
    # Topics
    python3 src/msg_publishers/current_grip.py $i &
    python3 src/msg_publishers/current_cart.py $i &
    python3 src/msg_publishers/current_joint.py $i &
    python3 src/msg_publishers/move_check.py $i &
    python3 src/msg_publishers/prox_check.py $i &
    python3 src/msg_publishers/speed_check.py $i &
    # Services
    python3 src/srv_services/home_position.py $i &
    python3 src/srv_services/mount_position.py $i &
    python3 src/srv_services/set_speed.py $i &
  
done

# When the terminal closes, these processes *should* be killed.
