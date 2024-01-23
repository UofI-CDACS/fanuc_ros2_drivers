#!/bin/bash
# This is a *"temporary"* (slowly becoming more permanent) 'launch' file to initalize all ROS nodes for a robot. Requires IP address.
# To run in the terminal: ./startRobot.sh XXX.XXX.X.XXX YYY.YYY.Y.YYY # Y and beyond is optional 
# Replace X's (and optional Y's) with Robot's IP(s)


if [ $# -eq 0 ];
then
    echo "Need at least 1 IP address..."
    exit 1
fi
# source the overlay    
source install/setup.sh

declare -A pid_array # This will hold all of our process PID's
declare -A commands # This will hold all of the commands for re-running in the background if needed
process_count=0 # Track number of processes, key for arrays
command_count=0

# For each IP that was given
for i in "$@"
do
    commands[$((command_count++))]='python3 src/action_servers/action_servers/cart_pose_server.py '$i 
    commands[$((command_count++))]='python3 src/action_servers/action_servers/convey_server.py '$i 
    commands[$((command_count++))]='python3 src/action_servers/action_servers/joint_pose_server.py '$i 
    commands[$((command_count++))]='python3 src/action_servers/action_servers/schunk_server.py '$i 
    commands[$((command_count++))]='python3 src/action_servers/action_servers/single_joint_server.py '$i 
    commands[$((command_count++))]='python3 src/msg_publishers/current_grip.py '$i 
    commands[$((command_count++))]='python3 src/msg_publishers/current_cart.py '$i 
    commands[$((command_count++))]='python3 src/msg_publishers/current_joint.py '$i 
    commands[$((command_count++))]='python3 src/msg_publishers/move_check.py '$i 
    commands[$((command_count++))]='python3 src/msg_publishers/prox_check.py '$i 
    commands[$((command_count++))]='python3 src/msg_publishers/speed_check.py '$i 
    commands[$((command_count++))]='python3 src/srv_services/home_position.py '$i 
    commands[$((command_count++))]='python3 src/srv_services/mount_position.py '$i 
    commands[$((command_count))]='python3 src/srv_services/set_speed.py '$i 
    # These will all run in the background
    for cmd in "${commands[@]}" 
    do
        echo $cmd
        $cmd &
        
        pid_array[$((process_count++))]=$! # Get proccess ID
    done

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
