# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a FANUC ROS2 robotic dice inspection and sorting system. Two FANUC robot arms pick up dice from a conveyor, inspect pip count via a MindVision USB camera, rotate the dice to a target face (odd or even), and drop them on the appropriate outfeed conveyor. All code lives under `fanuc_ros2_drivers/`.

## Build & Run Commands

All commands assume working directory `fanuc_ros2_drivers/` unless noted.

**Build (ROS2 workspace — run from repo root):**
```bash
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.sh
```

**Start all nodes for one robot:**
```bash
ros2 launch launch/start.launch.py robot_name:=bunsen robot_ip:=192.168.0.100
# Robot 2: robot_name:=beaker robot_ip:=192.168.0.101
```

**Standalone scripts (no ROS2 needed):**
```bash
python3 find_pip.py              # Interactive CLI: type a target pip, robot executes
python3 robot1_dice_control.py   # Example: even-pip target via DiceController API
python3 robot1_odd.py            # Example: odd-pip sequence (1→3→5)
```

**Vision tools (no robot needed):**
```bash
python3 pip_test.py              # Live camera + pip detection with 4 debug windows
python3 hsv_picker.py            # Interactive HSV calibration for yellow dice
```

**Tests:**
```bash
source install/setup.sh
python3 tests/servicesTest.py    # ROS2 service tests (mount, set_speed)
python3 tests/actionsTest.py     # ROS2 action tests (motion, gripper, conveyor)
```

**Test dice_vision library without robot or camera:**
```python
from dice_vision import DiceController, NullSchema
ctrl = DiceController(schema=NullSchema(verbose=True))
ctrl.set_state_directly(top=3, front=1, right=5)
result = ctrl.run(target='even')
print(result['rotation_sequence'])
```

## Architecture

### Top-Level Layers

```
Standalone scripts (find_pip.py, robot1_dice_control.py, robot1_odd.py)
        │
        ▼
dice_vision/              ← Pure Python library: orientation math + BFS planner
        │
        ▼
rotations.py              ← Physical robot poses + rotation sequence definitions
        │
        ▼
src/msg_publishers/dependencies/robot_controller.py  ← EtherNet/IP to FANUC
```

```
ROS2 Task Nodes (src/robot_task/)
        │  reads/writes
        ▼
Modbus TCP Server (src/modbus_server/)   ← inter-robot coordination
        ▲
        │  reads capture requests, writes pip results
src/camera_server/                       ← MindVision camera + pip detection
```

### `dice_vision/` — Core Library

Self-contained; no robot or camera dependency. Five modules:

- **`dice_model.py`** — `DiceState(top, front, right)` immutable tuple. Bottom/back/left are derived via the opposite-faces-sum-to-7 rule. Enumerates all 24 orientations at import time. Six rotation primitives: `roll_forward`, `roll_backward`, `roll_left`, `roll_right`, `spin_cw`, `spin_ccw`.
- **`rotation_planner.py`** — BFS over the 24-orientation graph. `plan_rotation_sequence(state, predicate, allowed_moves)` returns the shortest move list. `choose_discovery_rotation` picks the single rotation that maximally disambiguates unknown state.
- **`rotation_schema.py`** — `RotationSchema` abstract base class. Implement `_execute(rotation_name: str)` to wire in a real robot. `NullSchema` is a no-op for testing.
- **`dice_controller.py`** — `DiceController` orchestrates the full pipeline: state tracking, discovery roll, BFS planning, per-rotation execution, and camera verification.
- **`__init__.py`** — Public API exports.

### `rotations.py` — Physical Rotation Definitions

Each rotation is a list of steps:
1. `'move'` — Cartesian waypoints for collision-free travel
2. `'place'` — lower, release, lift clear
3. `'pick'` — reposition with new WPR, descend, grip, lift

The new wrist orientation (WPR) at pick time is what physically re-orients the die. **Edit only this file to adapt the system to a different robot cell.**

Robot connection (`ROBOT_IP = '10.8.4.6'`) and timing constants (`HANDSHAKE_TIMEOUT_S`, `GRIPPER_TIMEOUT_S`) are defined at the top.

### `src/` — ROS2 Packages

| Package | Role |
|---|---|
| `action_servers/` | Joint, Cartesian, and gripper motion; conveyor control |
| `msg_publishers/` | Publish current joint/Cartesian/gripper state at ~10 Hz |
| `srv_services/` | Mount position and speed-setting services |
| `camera_server/` | Capture frames, detect pips, write results to Modbus |
| `modbus_server/` | Modbus TCP server + register map (shared state) |
| `robot_task/` | State machine orchestration for each robot |

`robot_controller.py` (in `msg_publishers/dependencies/`) is the low-level EtherNet/IP layer used by all motion nodes and standalone scripts.

### Modbus Coordination Protocol

The Modbus server (`modbus_server/modbus_server/register_map.py`) owns 4 holding registers and 12 coils. Camera handshake:

1. Robot sets `COIL_ROBOT_x_CAPTURE_REQUEST`
2. Camera server captures image, writes pip count to `REG_ROBOT_x_PIP_RESULT`
3. Camera server clears `COIL_ROBOT_x_CAPTURE_REQUEST`
4. Robot polls until coil is cleared, then reads the result register

Inter-robot exclusion uses `COIL_ROBOT_x_CAMERA_ACTIVE`. Per-pip completion tracking uses `COIL_PIP_1_DONE` through `COIL_PIP_6_DONE`.

### State Machine

`src/robot_task/robot_task/states.py` defines 50+ states. The full flow is diagrammed in `state_diagram.md` (Mermaid format). Key paths:

- **Fast path:** top face already matches target → 0 rotations
- **One-correction path:** top after discovery roll matches → 1 rotation
- **BFS path:** 1 discovery roll + up to 3 BFS-planned rotations

`robot1_node.py` handles odd pips (1→3→5); `robot2_node.py` handles even pips (2→4→6).

## Hardware Prerequisites

- FANUC controller with `ros2_eip_back.tp` running in background and `ros2_eip_mainv2.tp` available in foreground
- MindVision USB 3.0 camera
- OnRobot gripper (controlled via Modbus register R[3] on the FANUC side)
- `pip3 install pycomm3` (EtherNet/IP library, may need manual install outside rosdep)

## Key File Locations

| What | Where |
|---|---|
| Robot IP & name env vars | `fanuc_ros2_drivers/.env` (copy from `.env.example`) |
| Physical poses & rotation sequences | `fanuc_ros2_drivers/rotations.py` |
| Modbus address map | `src/modbus_server/modbus_server/register_map.py` |
| Camera HSV thresholds | `fanuc_ros2_drivers/pip_test.py` (`LOWER_YELLOW`, `UPPER_YELLOW`) |
| State machine states enum | `src/robot_task/robot_task/states.py` |
| Full state diagram | `state_diagram.md` |
