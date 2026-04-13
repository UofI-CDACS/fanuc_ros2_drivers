
# Interaction Log — ROS2 Dice Project

---

## Session 1 — 2026-04-08

### User Prompt
> You are helping me build a ROS2 robotics project.
>
> PROJECT GOAL:
> - Pick up a dice with a robot
> - Move it to a camera
> - Capture image
> - Count pips
> - Repeat 3 times
> - Store individual and total pip count
>
> TECH STACK: ROS2 (Python), OpenCV
>
> TASK:
> 1. Help me set up folder structure and ROS2 workspace
> 2. Suggest ROS2 packages and nodes
> 3. Generate clean, modular Python code

### Assistant Response


**Architecture Decision:** 4-package modular design.

| Package | Role |
|---|---|
| `dice_interfaces` | Custom msgs, srvs, actions |
| `dice_vision` | Camera capture + OpenCV pip counting |
| `dice_robot_control` | FANUC robot motion wrapper |
| `dice_orchestrator` | State machine — coordinates 3 dice cycles |

**Nodes:**
- `orchestrator_node` — drives the 3-dice inspection loop
- `camera_node` — captures images via OpenCV
- `pip_counter_node` — counts pips using Hough circle detection
- `robot_control_node` — sends CartPose/JointPose actions to FANUC driver

**Generated files:** full workspace scaffold with all package.xml, setup.py, launch files, and Python nodes.

### Tool Calls / Commands
- Listed `/home/nishan-thapa/Downloads/Robotics/ROS2_dice_project/`
- Created workspace scaffold under `src/`

---

## Session 2 — 2026-04-12

### Summary

Continued development and integration of the ROS2 dice inspection project. Key work done this session:

#### 1. Robot Control Node — Pose Type Fix
- Identified that `pose_drop` and `pose_move_up` were labelled as `(joint)` in `poses.yaml` but their values were Cartesian (mm), not joint angles (degrees)
- Updated `poses.yaml` comments and `robot_control_node.py` docstring to correctly label them as `(cart)`
- Moved the parameter declarations into the correct "Cartesian poses" group
- Confirmed `robot_control_node.py` already calls `_cart()` for both steps — code was already correct

#### 2. Coordinate Sync
- User updated `config/robot_poses.yaml` with calibrated real-world coordinates
- Verified `src/dice_robot_control/config/poses.yaml` (the file actually used at runtime) was already in sync

#### 3. Running the System
- Diagnosed and fixed `PackageNotFoundError` — caused by missing `source install/setup.bash`
- Fixed `dice_system.launch.py` being called as a package (it's not installed) — switched to absolute file path
- Diagnosed robot connection `TimeoutError` to `172.29.208.1:44818` — network/IP issue, not a code issue
- User updated robot IP directly in `dice_system.launch.py`
- Added `source install/setup.bash` to `~/.bashrc` to persist across terminals

#### 4. Orchestrator Architecture Clarification
- User requested the robot repeat 3 times — initially added `num_cycles` loop to `robot_control_node.py`
- Discovered the `orchestrator_node` already handles the 3-dice repetition by sending separate goals for dice 1, 2, 3
- Reverted `num_cycles` change to avoid 9-cycle conflict and broken `at_camera` phase detection

#### 5. Real-Time Display Window
Added live video display with pip count overlay:

| Change | File |
|---|---|
| Added continuous 10 Hz frame publisher (`camera/image_raw`) | `camera_node.py` |
| Added debug image publisher (`dice_vision/debug_image`) after each pip count | `pip_counter_node.py` |
| Created new `display_node.py` — OpenCV window with live feed, per-dice results panel, running total | `display_node.py` (new) |
| Registered `display_node` entry point | `setup.py` |
| Added `display_node` to launch | `dice_vision.launch.py` |
| Added `q` key to close the display window | `display_node.py` |

#### Final System Flow
```
Launch dice_system.launch.py
  │
  ├── FANUC drivers      →  robot motion (CartPose / JointPose / SchunkGripper)
  ├── camera_node        →  /capture_image service + /camera/image_raw stream
  ├── pip_counter_node   →  /count_pips service + /dice_vision/debug_image topic
  ├── display_node       →  OpenCV window (live feed + results panel, press Q to close)
  ├── robot_control_node →  /inspect_dice action server
  └── orchestrator_node  →  drives 3-dice cycle, saves JSON to /tmp/dice_results/
```

---
