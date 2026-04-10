set shell := ["bash", "-c"]
set dotenv-load := true

robot_name   := env('ROBOT_NAME', 'my_robot')
robot_ip     := env('ROBOT_IP',   '0.0.0.0')
venv_pkgs    := `pwd` + "/.venv/lib/python3.12/site-packages"

# Source the workspace into your current shell
# NOTE: 'just source' cannot modify your current shell's environment — that is
# a shell limitation, not a just limitation. Run the printed commands directly.
source:
    @echo ""
    @echo "  Run these in your terminal to source the workspace:"
    @echo ""
    @echo "      source install/setup.bash"
    @echo "      export PYTHONPATH={{venv_pkgs}}:\$PYTHONPATH"
    @echo ""

# Build all packages
build:
    source /opt/ros/jazzy/setup.bash && \
    PYTHONPATH={{venv_pkgs}}:$PYTHONPATH \
    colcon build --symlink-install

# Launch all nodes (FANUC servers + MindVision camera)
launch:
    source install/setup.bash && \
    PYTHONPATH={{venv_pkgs}}:$PYTHONPATH \
    ros2 launch launch/start.launch.py robot_name:={{robot_name}} robot_ip:={{robot_ip}}

# Run the dice roller task
run:
    source install/setup.bash && \
    PYTHONPATH={{venv_pkgs}}:$PYTHONPATH \
    ros2 run dice_task dice_roller

# Quick camera test — grabs one frame and saves it to /tmp/grab.bmp (nodes must be running)
grab:
    source install/setup.bash && \
    PYTHONPATH={{venv_pkgs}}:$PYTHONPATH \
    python3 scripts/grab.py

# Tune HoughCircles parameters interactively (run 'just grab' first to get an image)
calibrate:
    PYTHONPATH={{venv_pkgs}}:$PYTHONPATH \
    python3 scripts/calibrate_hough.py

# Tune HSV crop parameters interactively (run 'just grab' first to get an image)
calibrate-hsv:
    PYTHONPATH={{venv_pkgs}}:$PYTHONPATH \
    python3 scripts/calibrate_hsv.py
