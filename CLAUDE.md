# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Environment Setup

Every new terminal session requires sourcing ROS2 and the workspace overlay:

```bash
source /opt/ros/jazzy/setup.bash   # or humble
source install/setup.bash          # after building
```

First-time dependency install (pycomm3 must be installed separately — it was removed from rosdep):

```bash
sudo rosdep init && rosdep update
rosdep install -i --from-path src --rosdistro jazzy -y
pip3 install pycomm3
```

## Build & Test

```bash
colcon build                          # build all packages
colcon test                           # run linter tests (flake8, pep257)
colcon test-result --verbose          # see test output
```

Manual integration tests (robot driver must be running first):

```bash
python3 tests/actionsTest.py          # interactive action server tests (press S)
python3 tests/servicesTest.py         # interactive service tests (press S)
./tests/topicsTest.sh ROBOT_NAME      # echo all state topics for a robot
```

## Running the Driver

Each robot requires its own terminal. Parameters **must** use `:=` syntax:

```bash
ros2 launch launch/start.launch.py robot_name:=bunsen robot_ip:=10.0.0.1
```

This starts 14 nodes: 6 action servers, 6 message publishers, and 2 service servers — all namespaced under `/{robot_name}/`.

## Architecture

### Communication Stack

```
FANUC Controller (Ethernet/IP)
    └─ FANUCethernetipDriver.py   (pycomm3 wrapper — low-level register reads/writes)
        └─ robot_controller.py    (motion/gripper abstraction)
            └─ Node implementations (action servers, publishers, service servers)
                └─ ROS2 Topics / Actions / Services
```

`robot_controller.py` and `FANUCethernetipDriver.py` are duplicated inside each package's `dependencies/` folder. If you fix a bug in one, fix it in all three.

### Four Packages

| Package | Type | Role |
|---|---|---|
| `fanuc_interfaces` | ament_cmake | All custom `.action`, `.msg`, `.srv` definitions |
| `action_servers` | ament_python | Action servers for long-running moves |
| `msg_publishers` | ament_python | Timer-based state publishers (0.5 s period) |
| `srv_services` | ament_python | One-shot service servers (mount, speed) |

### Interfaces (`fanuc_interfaces`)

**Actions** — all use `/{robot_name}/` prefix:
- `CartPose` — x, y, z, w, p, r (WPR = yaw/pitch/roll in degrees; pass 200.0 to keep current)
- `JointPose` — joint1–joint6 in degrees
- `SJointPose` — single joint index + angle
- `SchunkGripper` — `command` string: `"open"` or `"close"`
- `OnRobotGripper` — width (0–160 mm), force (0–120 N)
- `Conveyor` — `command` string: `"forward"`, `"reverse"`, or `"stop"`

**State topics** published by `msg_publishers`:
- `/{robot_name}/cur_cartesian` (`CurCartesian`) — X, Y, Z, W, P, R
- `/{robot_name}/cur_joints` (`CurJoints`) — joint1–joint6
- `/{robot_name}/is_moving` (`IsMoving`) — bool
- `/{robot_name}/prox_readings` (`ProxReadings`) — left, right bools
- `/{robot_name}/cur_gripper` (`CurGripper`) — open bool
- `/{robot_name}/speed` (`CurSpeed`) — float

**Services**:
- `/{robot_name}/go_mount` (`Mount`) — move to mount position
- `/{robot_name}/set_speed` (`SetSpeed`) — speed value 0–300

### Node Parameters

Every node (action servers, publishers, services) declares two ROS2 parameters passed from the launch file:
- `robot_ip` — IP address of the FANUC controller (default: `172.29.208.0`)
- `robot_name` — used for topic/action/service namespacing (default: `noNAME`)

### Hardware Prerequisites

Two FANUC TP programs must be loaded on the controller before running:
- `ros2_eip_back.tp` — runs continuously in the background
- `ros2_eip_mainv2.tp` — run in the foreground when ROS2 control is needed

Both are in `FANUC_TP_Program/`.
