# Project Changelog — Robotics Final Project

Two-robot Fanuc system: picks dice, inspects pip count via MindVision
GigE camera, rotates to target pip using BFS orientation planner, and
drops onto odd/even conveyor belts.

---

## 2026-04-23

### `61a3375` — Initial commit: Robotics Final Project
- Initial project structure with Fanuc EtherNet/IP drivers, action servers,
  message publishers, and service servers for robot control

### `3c2de2c` — Add modbus_server and robot_task packages
- Added modbus_server (shared state store for both robots) and robot_task
  (state machine controlling pick/inspect/rotate/drop cycle)

### `e3906b1` — Add camera_server and wire Modbus capture handshake
- Added camera_server ROS2 package; implemented Modbus coil handshake
  so task nodes can request pip captures without direct camera access

### `ab0e7da` — Redesign robot task for two-robot alternating pip sequence
- Rewrote task logic for coordinated two-robot operation: robots alternate
  picking up dice and delivering even/odd pip values to respective conveyors

---

## 2026-04-24

### `a2cc1a5` — Add .env file support for robot name and IP
- Launch files now load ROBOT_NAME and ROBOT_IP from a .env file

### `d1107eb` — Split task_node into states/base/robot1/robot2 modules
- Refactored monolithic task_node.py into separate modules for state
  definitions, shared base logic, and per-robot subclasses

### `11eb9a0` — Use Schunk gripper for Robot 2
- `task_config.yaml` / robot2_node: switched Robot 2 to use Schunk gripper
  action server instead of OnRobot

### `0993b4f` — Redesign for two conveyors and physical sensor coordination
- Major redesign: added second conveyor belt, proximity sensor handshake
  between robots, and alternating pip assignment logic

### `5b5c4eb` — Fix two flow bugs found in spec evaluation
- Fixed state transition errors discovered during spec review

### `8035322` — Fix modbus server crash and port conflict
- Resolved port binding conflict and crash on startup in modbus_server_node

### `d62e310` — Fix modbus IndexError, wrong robot class selection, and R2 calibration
- Fixed off-by-one in Modbus register reads; corrected robot class
  instantiation logic; updated Robot 2 calibration values

### `738d229` — Add dice vision system and robot1 die-inspection control
- Added `dice_vision` package: DiceState model, BFS rotation planner,
  DiceController, and RotationSchema interface
- Wired dice inspection into Robot 1 task flow

---

## 2026-04-26

### `695bb78` — Add threading-based wait utility and gripper sync to rotations
- Added thread-safe wait helpers and gripper synchronization steps to
  rotation execution

### `e7c2d44` — Add multi-pass camera inspection and BFS rotation planning
- `base_node.py`: implemented two-pass camera inspection (face_1 + discovery
  roll → face_2) to reconstruct full die orientation, then BFS-plan minimum
  rotations to target pip

### `5f6c619` — Wire BFS orientation planner, per-robot rotation schemas, and modular config
- Integrated dice_vision BFS planner into the task state machine; added
  per-robot RotationSchema classes; loaded task parameters from YAML

### `4065e74` — Move rotation sequences to per-robot YAML configs, load in base_node
- Extracted all rotation waypoint data into `rotations_r1.yaml` and
  `rotations_r2.yaml`; base_node loads the correct file based on robot index

### `7b151a4` — Add setup_ws.bash to fix Python environment for ROS2 node launch
- Added shell script to configure PYTHONPATH and source the workspace
  before launching nodes

---

## 2026-04-28

### `0369574` — Fix dice_vision import to work from installed location
- Corrected relative import paths in dice_vision package so it works
  from both source and installed locations

### `89e9071` — Inject fanuc_ros2_drivers into PYTHONPATH so dice_vision is importable
- `task.launch.py`: added PYTHONPATH injection for the task node

### `98fe969` — Fix _peek_robot_index always returning 1 when launched via launch file
- `task_node.py`: corrected argument parsing so robot_index is read
  correctly from ROS2 launch arguments

### `44ab946` — Clamp all joint and orientation degree values to [-179.9, 179.9]
- `FANUCethernetipDriver.py`: added clamping to prevent out-of-range
  degree values from being sent to the robot controller

### `4e37f41` — Add CLI arg support to camera_server and modbus_server launch files
- Launch files now accept robot_name, robot_ip, and other parameters
  as command-line arguments

### `c5d24db` — Implement pip detection in camera_server using pip_test.py pipeline
- `camera_server_node.py`: wired up the full MindVision camera capture
  and pip detection pipeline (open_camera, grab_frame, detect_pips)

### `1bbb834` — Inject fanuc_ros2_drivers into PYTHONPATH in camera_server launch file
- `camera_server.launch.py`: added fanuc_ros2_drivers root to PYTHONPATH
  so mvsdk and pip_test.py are importable from the installed node

### `0030c11` — Skip joint home during rotation — go directly to ROTATE_DICE
- `base_node.py`: removed the joint-home move between rotations to
  reduce cycle time

### `3700717` — Re-pick from roll-area rest position after each rotation
- Rotation sequences: added a pick step at the roll-area rest position
  after each sub-rotation so the robot re-grips before the next move

### `802e70b` / `a2c5014` — Rotation edits
- Manual calibration adjustments to rotation waypoint coordinates

### `3030280` — Add per-step logging to rotation execution for debugging
- `base_node.py`: added detailed log messages at each rotation step to
  help diagnose sequencing issues

### `b81949c` — Remove camera approach/ascend — move directly to camera pose
- `base_node.py`: simplified camera inspection by driving directly to the
  camera pose instead of using intermediate approach/ascend waypoints

### `2fe80f6` — Retry pip capture up to 3 times on zero result
- `base_node.py` (INSPECT_AT_CAMERA state): added retry loop that
  re-requests a camera capture if the result is 0

### `9670853` — Fix prox sensor namespace and bunsen drop direction
- Corrected proximity sensor ROS2 topic namespace and Robot 1 conveyor
  drop direction parameter

### `5306916` — Update task_config.yaml with calibrated conveyor pickup/drop poses for R1
- Filled in Robot 1 conveyor pickup and drop Cartesian coordinates from
  physical calibration

---

## 2026-04-30

### `b4becdd` — Switch camera to software trigger mode (trigger mode 1)
- `pip_test.py`: changed from free-run to software trigger mode so the
  camera only captures on explicit request, preventing stale frames

### `84f54b3` — Fix AE convergence in software trigger mode
- `pip_test.py`: corrected auto-exposure convergence loop for software
  trigger mode (was not triggering frames during warmup)

### `f4e60cf` — Lock exposure after AE warmup in software trigger mode
- `pip_test.py`: disable auto-exposure after warmup frames to stabilize
  brightness before pip detection

---

## 2026-05-01

### `38de413` — Sync hsv_picker camera init with pip_test (trigger mode + AE lock)
- `hsv_picker.py`: matched camera initialization sequence to pip_test.py
  so HSV calibration runs under identical conditions

### `6473574` — Update HSV thresholds for current lighting conditions
- `pip_test.py`: recalibrated LOWER_YELLOW and UPPER_YELLOW HSV bounds
  for the actual lab lighting

### `0db2eff` — Remove camera warmup delays and unused time import
- `camera_server_node.py`: removed unnecessary sleep delays that were
  added for camera warmup but are no longer needed with software trigger mode

### `ea4088f` — Fix bunsen drop conveyor command: backward → reverse
- `task_config.yaml`: corrected the conveyor direction string for
  Robot 1 (bunsen) drop action

### `56e0bcd` — Swap drop/pickup conveyor namespaces for both robots
- `task_config.yaml`: corrected which belt each robot drops onto vs.
  picks up from (odd/even were reversed)

### `663dbf5` — Update task_config.yaml conveyor namespaces and commands
- `task_config.yaml`: corrected conveyor ROS2 action namespace paths and
  start/stop command strings

### `245f979` — Add 2s settle delay after prox sensor triggers before stopping conveyor
- `base_node.py`: added a 2-second settle wait after the proximity sensor
  fires to let the die fully come to rest before the conveyor stops

### `6764914` — Update pip_test.py tuning constants (MIN_AREA 80→1000)
- `pip_test.py`: raised MIN_AREA from 80 to 1000 to filter out noise
  contours that were being counted as pips

### `8908af9` — Add calibrated rotation positions for Robot 2 (beaker)
- `rotations_r2.yaml`: populated with physically calibrated coordinates
  for all rotation sequences on Robot 2

### `84fe2a4` — Add try/except to conveyor execute_callback for proper error logging
- `convey_server.py`: wrapped the action callback in try/except so
  EtherNet/IP errors are logged and the goal is properly aborted

### `552f0d7` — Fix phantom roll_forward in roll_right compound sequence
- `rotations_r2.yaml`: removed a duplicated roll_forward sub-sequence
  that was causing an extra unwanted rotation

### `c0be2c6` — Reduce MIN_AREA threshold from 1000 to 900
- `pip_test.py`: lowered minimum pip blob area to catch smaller pip
  contours that were being filtered out

### `699da29` — Trim pick/carry steps from compound sequences (roll_right, roll_left, roll_backward)
- Rotation YAML: removed intermediate pick/carry steps from compound
  rotation sequences to shorten move count

### `1663634` — Update spin/roll waypoints to use roll-area X/Y for both robots
- Rotation YAML configs: unified spin and roll waypoints to share the
  roll-area X/Y coordinates for consistency

---

## 2026-05-02

### `fa5103a` — Remove joint home moves before camera, before conveyor drop, and after conveyor pickup
- `base_node.py`: eliminated unnecessary joint-home waypoints to reduce
  cycle time

### `66a86d8` — Remove final rest-place from rotation sequences; drop re-pick in rotate handler
- Simplified rotation sequences by removing the final rest-place step;
  updated rotate handler to not attempt a re-pick after the last rotation

### `a1d4810` — Remove final place step from compound sequence roll_forward portions
- Rotation YAML: removed redundant final place at end of the roll_forward
  sub-sequence embedded in compound rotations

### `d16d745` — Reduce post-sensor settle wait from 2.0s to 1.5s
- `base_node.py`: first reduction of the post-sensor settle delay

### `a7afc8d` — Update spin area Y coordinate to -363.909 in roll_right and roll_left
- `rotations_r2.yaml`: corrected Y coordinate for the spin/roll area

### `3743200` — Reduce post-sensor settle wait from 1.5s to 1.25s
- `base_node.py`: shortened the delay after proximity sensor triggers before
  stopping the conveyor

### `e8c15b8` — Update R2 conveyor pickup and drop poses with calibrated positions
- `task_config.yaml`: filled in Robot 2 conveyor pickup and drop coordinates
  from physical calibration runs

### `d87582f` — Update rotation sequences: travel_z 100mm, simplified waypoints, recalibrated WPR values
- Rotation YAML configs: set travel_z to 100mm clearance; simplified
  waypoint counts; updated W/P/R orientation values from calibration

### `1bf8537` — Move gripper settle sleep out of ROS2 executor callback
- Moved blocking time.sleep() calls for gripper settle out of the ROS2
  executor callback thread to prevent blocking the executor

---

## 2026-05-05

### `47a8fe5` — Show machine IP on modbus server startup
- `modbus_server_node.py`: logs the machine's IP address at startup so
  operators know what address to point clients at

### `d34e6b9` — Fix machine IP detection to show actual network interface IP
- `modbus_server_node.py`: replaced hostname-based IP lookup with UDP socket
  trick (connect to 8.8.8.8) to get the actual outbound network interface IP

### `a57cbbd` — Add retry logic to camera open to recover from stale GigE connections
- `camera_server_node.py`: _open_camera() now retries up to 5 times with 5s
  delays when CameraInit fails, recovering from stale GigE sessions left by
  previously crashed nodes

### `528caeb` — Fix action server bugs: polling loop rate, abort on error, goal validation
- `cart_pose_server.py`: added 50ms sleep in is_moving() loop to prevent
  busy-spinning that flooded EtherNet/IP reads; changed goal_handle.canceled()
  to abort() on exception so clients receive the correct terminal state
- `onrobot_server.py`: fixed bitwise | operators to logical or in width/force
  validation (| has lower precedence than comparisons, so validation was never
  triggering)

### `7c8b96b` — Fix camera/modbus shutdown: add SIGTERM handler and GigE recovery
- `camera_server_node.py`: added SIGTERM handler so destroy_node() fires on
  ros2 launch shutdown, preventing the GigE firmware lock (err:-14) left by
  unkilled processes; extracted _release_camera() helper; _grab_frame() now
  releases and reopens camera on err:-37 (network send error)
- `modbus_server_node.py`: added SIGTERM handler so the asyncio event loop
  stops cleanly, preventing port 1502 from entering TIME_WAIT and blocking
  the next launch for ~60 seconds

### `7a6bafa` — Add manual pip input fallback when camera is unavailable
- `find_pip.py`: added prompt_pip() helper; both face_1 and face_2 reads now
  fall back to operator keyboard entry instead of aborting when the camera fails
- `base_node.py`: after all 3 camera capture retries return 0, the task node
  prompts the operator for a manual pip count instead of flowing into State.ERROR

### `a0da98e` — Fix manual pip prompt to use /dev/tty instead of input()
- `base_node.py`: ros2 launch redirects child process stdin to /dev/null so
  input() returned EOF immediately; switched to opening /dev/tty directly so
  the operator prompt reaches the controlling terminal

### `5d993df` — Update R2 rotation sequences and R1 conveyor pickup calibration
- `rotations_r2.yaml`: travel_z reduced 100mm → 80mm; removed initial approach
  move from roll_forward; added final pick step to roll_right, roll_left,
  roll_backward compound sequences
- `task_config.yaml`: R1 conveyor_pickup_y → -633.358

### `ef52eac` — Update R2 compound rotation sequences and R1 conveyor pose calibration
- `rotations_r2.yaml`: reordered pick/place steps in roll_right and roll_left;
  added place+pick re-grip steps between spin_ccw passes in roll_backward
- `task_config.yaml`: R1 conveyor_pickup_y → -673.358, conveyor_drop_x → -210.577

### `ebb06ca` — Add retry logic to robot_controller init for slow robot connections
- `robot_controller.py` (all three copies — action_servers, msg_publishers,
  srv_services): constructor now retries the initial EtherNet/IP reads up to
  10 times with 3s delays (~30s total) before giving up, so nodes survive a
  slow robot boot instead of crashing immediately

### `918893c` — Increase MIN_CIRC from 0.40 to 0.50 for stricter pip filtering
- `pip_test.py`: raised minimum pip circularity threshold to reduce false
  positives from gripper edges and reflection artifacts

### `58d2e50` — Update R1 conveyor_drop_x calibration to -204.577
- `task_config.yaml`: adjusted Robot 1 conveyor drop X position from -210.577
  to -204.577
