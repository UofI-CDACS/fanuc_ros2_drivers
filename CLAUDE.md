# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

ROS2 drivers for FANUC CRX10 robots (30-Series controllers) communicating over Ethernet/IP. Targets ROS2 Jazzy. Provides publishers for robot state, action servers for motion commands, and services for utility operations.

## Build & Run

```bash
# Source ROS2
source /opt/ros/jazzy/setup.bash

# Install dependencies (from workspace root)
rosdep install -i --from-path src --rosdistro jazzy -y
pip3 install pycomm3

# Build
colcon build

# Source install
source install/setup.bash

# Launch all nodes (publishers + action servers + services)
ros2 launch launch/start.launch.py robot_name:=<NAME> robot_ip:=<IP>
```

## Testing

```bash
# Run ament linting tests for a package
colcon test --packages-select <package_name>
colcon test-result --verbose

# Manual integration tests (requires live robot connection)
python3 tests/actionsTest.py
python3 tests/servicesTest.py
bash tests/topicsTest.sh
```

## Architecture

The repo has 4 ROS2 packages under `src/`:

- **`fanuc_interfaces`** — Pure interface definitions (CMake). Defines all custom `.msg`, `.action`, and `.srv` types. No executable code.
- **`msg_publishers`** — 6 nodes that poll and publish robot state at 2Hz (joints, Cartesian pose, gripper status, speed, movement, proximity sensors).
- **`action_servers`** — 6 action server nodes for motion commands: `CartPose`, `JointPose`, `SJointPose`, `SchunkGripper`, `OnRobotGripper`, `Conveyor`.
- **`srv_services`** — 2 service nodes: `SetSpeed` (0–300 mm/s) and `Mount`.

All topics are namespaced under `/{robot_name}/` (e.g., `/ROBOT/cur_joints`).

## Shared Driver Layer

Each Python package (`action_servers`, `msg_publishers`, `srv_services`) contains a `dependencies/` directory with identical copies of:

- **`robot_controller.py`** — High-level `robot` class. All ROS2 nodes instantiate this class and call its methods. Key methods: `read_current_joint_position()`, `write_joint_pose()`, `write_cartesian_position()`, `set_speed()`, `is_moving()`, `schunk_gripper()`, `onRobot_gripper()`, `conveyor()`.
- **`FANUCethernetipDriver.py`** — Low-level EthernetIP/CIP driver wrapping `pycomm3`. Reads/writes FANUC position registers (PR[]) and integer registers (R[]) directly.

If you modify either driver file, update it in all three `dependencies/` directories.

## Key Register Conventions

FANUC robot registers used by the driver:
- `PR[1]` — Cartesian/joint position target
- `R[1]` — Robot connection bit
- `R[2]` — Motion sync trigger
- `R[3–5]` — Speed, gripper/conveyor control
- `R[20–23]` — Schunk gripper
- `R[30–31]` — Proximity sensors
- `R[35, 39–40]` — OnRobot gripper

## Robot-Side Requirements

The robot must have two TP programs loaded:
- `ros2_eip_back.tp` — Run in **background** (continuous state sync)
- `ros2_eip_mainv2.tp` — Run in **foreground** (active motion execution)

Source files are in `FANUC_TP_Program/`.
