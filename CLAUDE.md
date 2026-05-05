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

**NOTE: Register map below was superseded later in this session — see Session Log update at bottom.**

**Modbus registers/coils (original, since replaced):**
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

## Session Log — 2026-05-04 (continued)

### What Was Built

After pulling partner's (`JonPal7756`) `final_project` package, `robot1_controller.py` was fully retrofitted to speak the same Modbus protocol as `modbus_server.py` (Bunsen's Modbus TCP server). The simple register writes were replaced with a proper conveyor handshake state machine so both robots coordinate cleanly.

### Files Changed

| File | What changed |
|---|---|
| `src/dice_game/dice_game/robot1_controller.py` | Full Modbus retrofit — see details below |

---

### robot1_controller.py — Modbus Retrofit Details

#### Updated Modbus Register Map (matches `src/final_project/final_project/modbus_server.py`)

| Name | Type | Address | Description |
|---|---|---|---|
| STATE | Holding register | 0 | Current Beaker state (1–9) |
| PIP_PROGRESS | Holding register | 1 | Current target pip number (1–6) |
| CONV_CMD | Holding register | 2 | Conveyor handshake state machine (0–8) |
| RETRIES | Holding register | 3 | Bunsen's cumulative retry count (read at end) |
| READY | Coil | 0 | Bunsen idle flag (not written by Beaker) |
| CAMERA_CLIENT | Coil | 1 | 1 = Bunsen holds camera token, 0 = Beaker holds it |

Port changed from `502` → `5020`.

#### Conveyor Handshake State Machine (REG_CONV_CMD)

Two transfer sequences coordinated via a single shared register:

**Beaker → Bunsen (rear conveyor):**
```
IDLE(0) → BEAKER_WANTS_SEND(1) → REAR_RUNNING(2) → DIE_ON_REAR(3) → BUNSEN_HAS_DIE(4) → IDLE(0)
```

**Bunsen → Beaker (front conveyor):**
```
IDLE(0) → BUNSEN_WANTS_SEND(5) → FRONT_RUNNING(6) → DIE_ON_FRONT(7) → BEAKER_HAS_DIE(8) → IDLE(0)
```

#### New / Replaced Methods

| Old | New | Why |
|---|---|---|
| `place_on_conveyor(which)` | `send_to_bunsen()` | Full handshake instead of fire-and-forget |
| `_wait_for_return()` | `receive_from_bunsen()` | Polls CONV_CMD instead of ROS2 topic |
| `_set_pip(pips)` | `_set_pip_progress(pip)` | Writes target pip to REG_PIP_PROGRESS |
| — | `_mb_read(addr)` | Reads a holding register |
| — | `_mb_write(addr, val)` | Writes a holding register |
| — | `_mb_write_coil(addr, val)` | Writes a coil |
| — | `_wait_conv(target, timeout)` | Polls CONV_CMD until target value |
| — | `_run_conveyor(command)` | Conveyor ROS2 action client call |

**Camera token:** When Beaker places die on rear belt, it sets `COIL_CAMERA_CLIENT = True` so Bunsen knows it can call the camera service for pip verification. Beaker takes the token back (`False`) before picking the die up from the front conveyor.

#### Positions Still Needing Calibration

```python
CONV_FRNT_ABV = dict(x=0.0, ...)   # front conveyor pickup — above
CONV_FRNT_DWN = dict(x=0.0, ...)   # front conveyor pickup — down
```

These are in `robot1_controller.py` at the top and need real coordinates once the physical front conveyor pickup position is known.

#### Running (updated)

```bash
# Terminal 1 — Bunsen's Modbus server (run on Bunsen's machine)
python3 src/final_project/final_project/modbus_server.py

# Terminal 2 — Beaker FANUC driver nodes
ros2 launch launch/start.launch.py robot_name:=Beaker robot_ip:=10.8.4.16

# Terminal 3 — Beaker game controller + camera
ros2 launch dice_game robot1.launch.py robot_name:=Beaker robot_ip:=10.8.4.16 modbus_ip:=<bunsen_ip>
```
