# Source this file to set up the full workspace environment:
#   source setup_ws.bash

VENV_ROOT=~/ros2_ws/fanuc_ros2_drivers/ros_env

source "${VENV_ROOT}/bin/activate"
source /opt/ros/jazzy/setup.bash
source "$(dirname "${BASH_SOURCE[0]}")/install/setup.bash"

# Expose venv site-packages to nodes launched by ROS2 (they use the system
# Python shebang and cannot see the venv otherwise).
export PYTHONPATH="${VENV_ROOT}/lib/python3.12/site-packages:${PYTHONPATH}"

# Make dice_vision importable — the installed base_node.py path arithmetic
# doesn't reach the workspace root, so we add it explicitly.
_WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export PYTHONPATH="${_WS_ROOT}:${PYTHONPATH}"
