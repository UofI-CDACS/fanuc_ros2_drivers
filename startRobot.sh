#!/bin/bash
# This is a *"temporary"* (slowly becoming more permanent) 'launch' file to initalize all ROS nodes for a robot. Requires IP address.
# To run in the terminal: ./startRobot.sh XXX.XXX.X.XXX ROBOT_NAME
# Replace X's and ROBOT_NAME with Robot's IP and name, respectively.

# Value checking
if [ $# -eq 0 ]
then
    echo "Need an IP and Name!"
    exit 1
fi
if [ $# -eq 1 ]
then
    echo "Need a Name!"
    exit 1
fi

# source the overlay    
source install/setup.sh

declare -A pid_array # This will hold all of our process PID's
declare -A commands # This will hold all of the commands for re-running in the background if needed
process_count=0 # Track number of processes, key for arrays
command_count=0


commands[$((command_count++))]='python3 src/action_servers/action_servers/cart_pose_server.py '$1' '$2
commands[$((command_count++))]='python3 src/action_servers/action_servers/convey_server.py '$1' '$2
commands[$((command_count++))]='python3 src/action_servers/action_servers/joint_pose_server.py '$1' '$2
commands[$((command_count++))]='python3 src/action_servers/action_servers/schunk_server.py '$1' '$2
commands[$((command_count++))]='python3 src/action_servers/action_servers/single_joint_server.py '$1' '$2
commands[$((command_count++))]='python3 src/msg_publishers/current_grip.py '$1' '$2
commands[$((command_count++))]='python3 src/msg_publishers/current_cart.py '$1' '$2
commands[$((command_count++))]='python3 src/msg_publishers/current_joint.py '$1' '$2
commands[$((command_count++))]='python3 src/msg_publishers/move_check.py '$1' '$2
commands[$((command_count++))]='python3 src/msg_publishers/prox_check.py '$1' '$2
commands[$((command_count++))]='python3 src/msg_publishers/speed_check.py '$1' '$2
commands[$((command_count++))]='python3 src/srv_services/mount_position.py '$1' '$2
commands[$((command_count))]='python3 src/srv_services/set_speed.py '$1' '$2
# These will all run in the background
for cmd in "${commands[@]}" 
do
    echo $cmd
    $cmd &
    
    pid_array[$((process_count++))]=$! # Get proccess ID
done

echo "Number of commands: $((command_count+1))"
echo "Number started: " $process_count
process_count=$((process_count - 1)) # For array management

while true
do
    for i in $(seq 0 $process_count)
    do
        if ! ps -p ${pid_array[$i]} > /dev/null 
        then # If the process isn't running anymore, restart it
            echo "Restarting "${commands[$i]}
            ${commands[$i]} &
            pid_array[$i]=$! # Get new proccess ID      
        fi
    done
done
