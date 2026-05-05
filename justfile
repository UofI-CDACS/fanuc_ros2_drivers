set dotenv-load := true

ros_setup  := "/opt/ros/jazzy/setup.bash"
ws_setup   := "install/setup.bash"
venv_pkgs  := justfile_directory() + "/.venv/lib/python3.12/site-packages"
robot1     := env('ROBOT_1_NAME', 'robot1')
robot1_ip  := env('ROBOT_1_IP', '0.0.0.0')
robot2     := env('ROBOT_2_NAME', 'robot2')
robot2_ip  := env('ROBOT_2_IP', '0.0.0.0')

# One-time: create .venv, install Python deps, vendor mvsdk.py into the venv
setup:
    python3.12 -m venv .venv
    .venv/bin/pip install --upgrade pip
    .venv/bin/pip install -r requirements.txt
    cp third_party/mvsdk.py {{venv_pkgs}}/

# Print commands to source workspace + venv overlay in current shell
source:
    @echo "Run the following in your shell:"
    @echo "  source /opt/ros/jazzy/setup.bash && source install/setup.bash"
    @echo "  export PYTHONPATH={{venv_pkgs}}:\${PYTHONPATH:-}"

# Build the workspace
build:
    bash -c "source {{ros_setup}} && PYTHONPATH={{venv_pkgs}}:${PYTHONPATH:-} colcon build"

# Launch both robot server stacks + camera node (Ctrl+C stops all)
launch:
    bash -c "source {{ros_setup}} && source {{ws_setup}} && \
        PYTHONPATH={{venv_pkgs}}:${PYTHONPATH:-} \
        ros2 launch launch/start.launch.py robot_name:={{robot1}} robot_ip:={{robot1_ip}} & \
        PYTHONPATH={{venv_pkgs}}:${PYTHONPATH:-} \
        ros2 launch launch/start.launch.py robot_name:={{robot2}} robot_ip:={{robot2_ip}} & \
        PYTHONPATH={{venv_pkgs}}:${PYTHONPATH:-} \
        ros2 run dual_fanuc mv_camera_node & \
        wait"

# Launch Robot 1 server stack + camera node only
launch1:
    bash -c "source {{ros_setup}} && source {{ws_setup}} && \
        PYTHONPATH={{venv_pkgs}}:${PYTHONPATH:-} \
        ros2 launch launch/start.launch.py robot_name:={{robot1}} robot_ip:={{robot1_ip}} & \
        PYTHONPATH={{venv_pkgs}}:${PYTHONPATH:-} \
        ros2 run dual_fanuc mv_camera_node & \
        wait"

# Launch Robot 2 server stack only
launch2:
    bash -c "source {{ros_setup}} && source {{ws_setup}} && \
        PYTHONPATH={{venv_pkgs}}:${PYTHONPATH:-} \
        ros2 launch launch/start.launch.py robot_name:={{robot2}} robot_ip:={{robot2_ip}} & \
        wait"

# Grab a single frame via ROS2 camera node (node must be running) → /tmp/grab.bmp
grab:
    bash -c "source {{ros_setup}} && source {{ws_setup}} && PYTHONPATH={{venv_pkgs}}:${PYTHONPATH:-} python3 scripts/grab.py"

# Grab a single frame directly via SDK (camera node must NOT be running) → /tmp/grab.bmp
grab-sdk:
    bash -c "PYTHONPATH={{venv_pkgs}}:${PYTHONPATH:-} python3 scripts/grab_sdk.py"

# Open Robot 1 Schunk gripper (launch1 must be running)
open-schunk:
    bash -c "source {{ros_setup}} && source {{ws_setup}} && PYTHONPATH={{venv_pkgs}}:${PYTHONPATH:-} python3 scripts/test_gripper.py schunk open {{robot1}}"

# Close Robot 1 Schunk gripper (launch1 must be running)
close-schunk:
    bash -c "source {{ros_setup}} && source {{ws_setup}} && PYTHONPATH={{venv_pkgs}}:${PYTHONPATH:-} python3 scripts/test_gripper.py schunk close {{robot1}}"

# Open Robot 2 OnRobot gripper (launch2 must be running)
open-onrobot:
    bash -c "source {{ros_setup}} && source {{ws_setup}} && PYTHONPATH={{venv_pkgs}}:${PYTHONPATH:-} python3 scripts/test_gripper.py onrobot open {{robot2}}"

# Close Robot 2 OnRobot gripper (launch2 must be running)
close-onrobot:
    bash -c "source {{ros_setup}} && source {{ws_setup}} && PYTHONPATH={{venv_pkgs}}:${PYTHONPATH:-} python3 scripts/test_gripper.py onrobot close {{robot2}}"

# Run Robot 1 — pick/present/analyse/place loop
run:
    bash -c "source {{ros_setup}} && source {{ws_setup}} && PYTHONPATH={{venv_pkgs}}:${PYTHONPATH:-} ros2 run dual_fanuc robot1"

# Run Robot 2 — waits for pip count from Robot 1, then acts
run2:
    bash -c "source {{ros_setup}} && source {{ws_setup}} && PYTHONPATH={{venv_pkgs}}:${PYTHONPATH:-} ros2 run dual_fanuc robot2"

# Robot 1: pick die from DICE_PICK, place at DICE_PLACE, run all five rotation primitives
rotate1:
    bash -c "source {{ros_setup}} && source {{ws_setup}} && PYTHONPATH={{venv_pkgs}}:${PYTHONPATH:-} ros2 run dual_fanuc rotate1"

# Robot 2: assume die is at DICE_PLACE, run all five rotation primitives
rotate2:
    bash -c "source {{ros_setup}} && source {{ws_setup}} && PYTHONPATH={{venv_pkgs}}:${PYTHONPATH:-} ros2 run dual_fanuc rotate2"

# Run front conveyor (Robot 2) forward until Ctrl+C
conveyer-front:
    bash -c "source {{ros_setup}} && source {{ws_setup}} && PYTHONPATH={{venv_pkgs}}:${PYTHONPATH:-} python3 scripts/test_conveyor.py front {{robot2}}"

# Run back conveyor (Robot 1) forward until Ctrl+C
conveyer-back:
    bash -c "source {{ros_setup}} && source {{ws_setup}} && PYTHONPATH={{venv_pkgs}}:${PYTHONPATH:-} python3 scripts/test_conveyor.py back {{robot1}}"

# Tune HSV crop parameters interactively (run 'just grab' first to get an image)
calibrate-hsv:
    bash -c "PYTHONPATH={{venv_pkgs}}:${PYTHONPATH:-} python3 scripts/calibrate_hsv.py"

# Tune HoughCircles parameters interactively (run 'just calibrate-hsv' first to get a cropped image)
calibrate-hough:
    bash -c "PYTHONPATH={{venv_pkgs}}:${PYTHONPATH:-} python3 scripts/calibrate_hough.py"

# Force-kill all ROS2 nodes and related processes
kill:
    #!/usr/bin/env bash
    pkill -9 -f "ros2"          || true
    pkill -9 -f "robot_controller" || true
    pkill -9 -f "action_servers"   || true
    pkill -9 -f "msg_publishers"   || true
    pkill -9 -f "srv_services"     || true
    pkill -9 -f "dual_fanuc"       || true
    echo "All ROS2 processes killed."
