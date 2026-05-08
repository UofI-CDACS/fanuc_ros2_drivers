# Before running (Do once): chmod +x topicsTest.sh
# To run: ./topicsTest.sh ROBOT_NAME # Robot name to be replaced with name

# This script will read off the most current message from each topic
if [ $# -eq 0 ];
then
    echo "Need at least 1 name..."
    exit 1
fi

# for each name given
for i in "$@"
do
    ros2 topic echo --once /$i/cur_cartesian
    ros2 topic echo --once /$i/cur_joints
    #ros2 topic echo --once /$i/grip_status # This one currently does not publish anything
    ros2 topic echo --once /$i/is_moving
    ros2 topic echo --once /$i/prox_readings
    ros2 topic echo --once /$i/speed
done