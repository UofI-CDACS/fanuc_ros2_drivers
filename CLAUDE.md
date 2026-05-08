# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ROS2 middleware for controlling FANUC CRX10 industrial robots over Ethernet/IP. Maintained by the University of Idaho CDACS lab. Current ROS2 target: **Jazzy** (also supports Humble).

## Build & Run

```bash
# Initial setup
rosdep install -i --from-path src --rosdistro jazzy -y
pip3 install pycomm3
colcon build
source install/setup.sh

# Selective rebuild (faster for iteration)
colcon build --packages-select fanuc_interfaces     # After changing .action/.msg/.srv files
colcon build --packages-select action_servers       # After changing Python node files
colcon build --symlink-install                       # Avoids rebuilds for pure Python changes

# Launch all nodes
ros2 launch launch/start.launch.py robot_name:=ROBOT_NAME robot_ip:=192.168.1.X

# Run test clients (requires server to be running)
python3 tests/actionsTest.py
python3 tests/servicesTest.py
```

## Architecture

### Communication Stack

```
ROS2 Clients / Applications
        │  Topics / Actions / Services
ROS2 Node Layer (14 nodes total)
  ├─ action_servers/   – 6 action servers (movement, grippers, conveyor)
  ├─ msg_publishers/   – 6 publishers polling robot state at 2 Hz
  └─ srv_services/     – 2 service servers (mount position, speed)
        │  Python API
dependencies/robot_controller.py       ← high-level abstraction
dependencies/FANUCethernetipDriver.py  ← Ethernet/IP (CIP) protocol
        │  TCP/IP (Ethernet/IP)
FANUC Robot Controller
  ├─ ros2_eip_back.tp   (background program – must always be running)
  └─ ros2_eip_mainv2.tp (foreground program – run during ROS control)
```

### Package Layout

| Package | Build type | Role |
|---|---|---|
| `src/fanuc_interfaces/` | `ament_cmake` | `.action`, `.msg`, `.srv` definitions |
| `src/action_servers/` | `ament_python` | Action server nodes |
| `src/msg_publishers/` | `ament_python` | Publisher nodes |
| `src/srv_services/` | `ament_python` | Service server nodes |

### Shared Driver Code (`dependencies/`)

Each Python package contains a `dependencies/` subdirectory (duplicated across packages) with:
- `FANUCethernetipDriver.py` — uses `pycomm3.CIPDriver` to read/write position registers via CIP. Reads CURPOS (class `0x7D`, instance `0x01`) and writes to `PR[N]` registers.
- `robot_controller.py` — `robot` class that wraps the driver. Constructor: `robot(robotIP, DEBUG=False)`. Methods cover joint/cartesian movement, gripper control, speed, and mount position.

### Interfaces (`fanuc_interfaces`)

**Actions:** `CartPose`, `JointPose`, `SJointPose`, `SchunkGripper`, `OnRobotGripper`, `Conveyor`

**Messages:** `CurCartesian`, `CurJoints`, `CurGripper`, `CurSpeed`, `IsMoving`, `ProxReadings`

**Services:** `Mount`, `SetSpeed`

### Node Conventions

- All nodes declare `robot_ip` and `robot_name` parameters; topics are namespaced by `robot_name`
- All nodes launch with `respawn=True`, `respawn_delay=4`
- Angle values are in degrees, range `[-179.0, 179.0]`; WPR value `200.0` means "maintain current orientation"
- Action servers return `distance_left` feedback during movement

## Key Constraints

- `pycomm3` is installed via `pip3`, **not** via `rosdep` (it was removed as a rosdep dependency intentionally — see commit `5fd2eb3`)
- The robot controller TP programs must be running on the FANUC controller before launching ROS nodes
- When adding new interfaces, both `CMakeLists.txt` and the `.action`/`.msg`/`.srv` file must be updated, then rebuild `fanuc_interfaces` before dependent packages
