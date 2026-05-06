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

---

## Session Log — 2026-05-04

### What Was Built

This session added the full `dice_game` package and wired up Modbus state reporting for Robot 1 (Beaker). The project was also pushed to GitHub and shared with a partner.

### Files Changed / Created

| File | What changed |
|---|---|
| `src/dice_game/dice_game/robot1_controller.py` | Main file — see details below |
| `src/dice_game/launch/robot1.launch.py` | Added `modbus_ip` CLI parameter |
| `src/fanuc_interfaces/CMakeLists.txt` | Added `CaptureImage.srv` to build |
| `src/fanuc_interfaces/srv/CaptureImage.srv` | New service definition for camera capture |

---

### robot1_controller.py — Changes in Detail

#### 1. Wrist Rotation to Find Target Pip

Previously the robot picked the die, took one photo, and if the pip count was wrong it put the die back down and re-picked (relying on random re-orientation). Now it actively rotates.

**New constant:**
```python
CAMERA_ROTATION_STEPS = [0, 60, 120, 180, -120, -60]  # degrees offset on CAMERA_POSE['r']
```

**New method `_capture_count_at(label)`** — captures and counts pips at the current robot position without moving first. Refactored out of `_capture_and_count`.

**New method `_find_pip_rotating(target, label)`** — moves to camera pose, then steps through each 60° wrist rotation, taking a photo at each position. Returns as soon as the target pip count is found (robot stays at that rotation). Returns `0` if target is not found on any face. Used in both Phase 1 (find pip=1) and Phase 2 (sequential 1→6).

The die is only put back down and re-picked if the target pip is not visible on any of the 6 rotational positions.

#### 2. Modbus State Reporting

Beaker connects as a **Modbus TCP client** to Bunsen's machine (Robot 2) and writes its state throughout the game so Bunsen can coordinate.

**Modbus registers/coils:**
| Name | Type | Address | Values |
|---|---|---|---|
| State | Holding register | 0 | 1–9 (see states below) |
| Pip count | Holding register | 1 | 0–6 |
| Ready | Coil | 0 | True/False |

**States (`State` IntEnum):**
| Value | Name | When set |
|---|---|---|
| 1 | SETUP | On startup, waiting for servers |
| 2 | WAIT | After go_home, waiting for die return |
| 3 | GRAB_DIE | Before pick_dice() |
| 4 | PIP_COUNT | First rotation position (r_offset=0°) |
| 5 | ROTATE_PIP | Each subsequent rotation step |
| 6 | PLACE_DIE | Before place_on_conveyor() |
| 7 | FINISH | After all 6 targets delivered |
| 8 | RECOVER | When target pip not found, re-picking |
| 9 | FAULT | On timeout or unhandled exception |

**New helpers:** `_modbus_connect(ip)`, `_set_state(state)`, `_set_pip(pips)`, `_set_ready(ready)` — all silently do nothing if Modbus is not connected, so the robot still works without a Modbus server.

**New dependency:** `pip3 install pymodbus`

#### 3. Launch File

`modbus_ip` added as a CLI parameter:
```bash
ros2 launch dice_game robot1.launch.py robot_name:=Beaker robot_ip:=10.8.4.16 modbus_ip:=<bunsen_ip>
```

---

### Running Robot 1 Only (Beaker)

You need **three** terminals:

**Terminal 1 — FANUC driver nodes** (action servers, publishers, services):
```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch launch/start.launch.py robot_name:=Beaker robot_ip:=10.8.4.16
```

**Terminal 2 — Game controller + camera**:
```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch dice_game robot1.launch.py robot_name:=Beaker robot_ip:=10.8.4.16 modbus_ip:=<bunsen_ip>
```

Also requires the FANUC TP programs running on the controller before starting.

---

### GitHub Setup

- Repo: **https://github.com/ColinClang/Dice_game** (private)
- Branch: `v1.1`
- Collaborator: `JonPal7756` (write access)

**Daily workflow:**
```bash
git add <files>
git commit -m "description"
git push

git pull   # to get partner's changes
```

---

## Session Log — 2026-05-05 (Bunsen side)

### What Was Merged

Pulled partner's `dice_game/v1.1` commits into `final_project`. Applied fixes and improvements to `robot1_master.py` and `camera.py`.

### Files Changed

| File | What changed |
|---|---|
| `src/final_project/final_project/robot1_master.py` | Conveyor bug fixes, camera fallback, updated positions |
| `src/final_project/final_project/camera.py` | Fixed trigger mode 0 → 1 (software trigger) |
| `test_conveyor.py` | Copied from partner — standalone belt timing test |

### robot1_master.py — Changes in Detail

#### 1. `receive_from_bunsen()` conveyor bug fixed

Was incorrectly calling `_run_conveyor('forward')` (Beaker's rear belt) during a receive operation. Beaker does not control the front conveyor — Bunsen does. Beaker only writes `CONV_FRONT_RUNNING` as a Modbus signal; Bunsen starts its own belt. Removed all `_run_conveyor` calls from `receive_from_bunsen()`.

#### 2. `send_to_bunsen()` now runs rear belt

Was dropping die on belt but never running it. Added:
```python
self._run_conveyor('forward')
time.sleep(REAR_CONVEYOR_TRAVEL_SECS)
self._run_conveyor('stop')
```
New constant `REAR_CONVEYOR_TRAVEL_SECS = 5.0` (tune to match physical belt length).

#### 3. Camera fallback

`wait_for_service()` was blocking forever. Now uses `timeout_sec=5.0` — if camera not found, robot still runs through all motions and assumes correct pip (for testing without camera). Added `_camera_ok` flag.

#### 4. Conveyor positions updated

Replaced placeholder `CONV_ABOVE`/`CONV_DROP`/`FRONT_CONV_ABOVE`/`FRONT_CONV_PICKUP` with partner's calibrated coordinates:
- `CONV_REAR_ABV`, `CONV_REAR_DRP` — calibrated rear conveyor drop positions
- `CONV_FRNT_ABV`, `CONV_FRNT_DWN` — front conveyor pickup positions (still need physical verify)

#### 5. `receive_from_bunsen()` now takes camera token back

After `CONV_DIE_ON_FRONT` confirmed, Beaker writes `COIL_CAMERA_CLIENT=False` to reclaim camera before picking up the die.

### camera.py — Trigger Mode Fix

`CameraSetTriggerMode(hCamera, 0)` → `CameraSetTriggerMode(hCamera, 1)` (software trigger mode required for single-frame capture).
