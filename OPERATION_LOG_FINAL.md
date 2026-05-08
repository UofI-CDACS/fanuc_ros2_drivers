# Operation Log — Final Assignment: Two-Robot Dice Coordination
**Course: Final Assignment | FANUC CRX10 Robot | ROS2 Jazzy**
**Author:** Michael Habermann
**Partner:** TBD
**AI Assistance:** Claude Code (claude.ai/code) — claude-sonnet-4-6
**Branch:** Michael-Habermann-finalassignment
**Date Started:** 2026-04-24

---

## Overview

Two-robot FANUC CRX10 coordination task:
- Robot 1 (DJ): picks up dice, presents to camera, counts pips, routes to conveyor
- Robot 2 (Bill): receives dice from conveyor, tracks pips, places at end
- Sequential pip counting 1→6; start state = pip 1
- Even pip → front conveyor (Bill), Odd pip → back conveyor (DJ)
- Track retries per robot and combined
- Communication via Modbus TCP/IP (server on DJ's machine, port 5020)

---

## Session 1 — 2026-04-24

### Prompt 1
> "hello I am starting a new assignment... I need you to create a new branch/pull request based on what we previously did..."

**Action (Claude):**
- Read existing `ModbusStuff/` files and `dice_pipeline` package for context
- Created branch `Michael-Habermann-finalassignment` off `v1.1`
- Created `OPERATION_LOG_FINAL.md`
- Initialized Claude memory system

---

### Prompt 2
> "I can worry about rotation... DJ is Schunk you have no need to worry about Bill I control my own conveyor. Now I need a test server for modbus and a test client to see if it works"

**Clarifications captured:**
- User handles all wrist rotation orientations manually
- Front face = face toward camera at pickup; after wrist flip, bottom face visible
- Die placed on conveyor in correct orientation; flip happens on belt
- DJ = Schunk gripper; only DJ's conveyor logic needed
- Modbus server on DJ's machine, port 5020, slave=1
- Namespaces: `dj` and `bill`

**Action (Claude):**
- Created `ModbusStuff/modbus_test_server.py` — async pymodbus 3.x TCP server
- Created `ModbusStuff/modbus_test_client.py` — reads/writes every register, prints PASS/FAIL
- Fixed pymodbus off-by-one: added `zero_mode=True` to `ModbusSlaveContext` (pymodbus 3.x defaults to 1-based addressing, causing last register in each block to be unreachable)
- Smoke tested locally: all 13 register tests passed

**To run:**
```bash
# Terminal 1
python3 ModbusStuff/modbus_test_server.py

# Terminal 2 (local)
python3 ModbusStuff/modbus_test_client.py

# Terminal 2 (against remote machine)
python3 ModbusStuff/modbus_test_client.py --host <IP> --port 5020
```

---

<!-- New entries appended below each session -->

## Session 2 — Camera pipeline + HSV tuning

**Topic:** Get the camera node publishing reliably; tune HSV detection so pip
counts are accurate.

**Prompts (summarised):**
- "where are my photos taken" / "how do I recompile / how do I run it / how can
  I see if it is presenting the images"
- Reported camera node printed "ready" but `/camera/image_raw` was silent.
- Wanted live HSV tuning for the dice colour mask.
- Requested camera node to also publish pip count at 1 Hz.

**Actions (Claude):**
- Identified bug in `dice_pipeline/dice_pipeline/camera_node.py`:
  `mvsdk.CameraStop()` was being called after every `_publish_frame()`. First
  frame published, camera stopped, no further frames. Removed the call.
- Walked through `rqt_image_view` / `ros2 topic hz` for verifying the live
  feed, plus `colcon build --symlink-install` so future Python edits don't
  need a rebuild.
- Created `src/final_assignment/final_assignment/hsv_tuner_node.py`:
  subscribes to `/camera/image_raw`, OpenCV trackbars for H/S/V min/max +
  die-area / pip-area thresholds, publishes `Int32` to `/camera/pip_count`
  once per second. Registered as `hsv_tuner` console script in the
  `final_assignment` package's `setup.py`.
- After empirical tuning, accepted final detection parameters:
  `HSV_LOWER=[7, 141, 53]`, `HSV_UPPER=[18, 255, 135]`,
  `DIE_SIZE_LOWER=5000`, `PIP_SIZE_LOWER=340`, `PIP_SIZE_UPPER=4039`.
- Modified `camera_node.py` to also publish `/camera/pip_count` (Int32) at
  1 Hz from a timer (alongside the existing image stream + count_pips
  service).

---

## Session 3 — Position recording helper

**Topic:** A simpler way to capture joint + cartesian poses from the live arm
than editing constants by hand.

**Prompts (summarised):**
- "make me a program that constantly reads my positions, when I click enter
  I am prompted to enter the name of the position and it prints out the
  cartesian and joint position for it."
- Follow-up: "if I click `o` it opens the gripper and if I click `c` it
  closes it."

**Actions (Claude):**
- Created `ModbusStuff/record_positions.py`:
  - Subscribes to `<robot_name>/cur_cartesian` and `<robot_name>/cur_joints`
  - Live-prints the current pose on a single overwriting line
  - On Enter, prompts for a name, then prints **and appends** a snapshot
    block to `ModbusStuff/recorded_positions.txt`
  - Single-key shortcuts in the prompt: `o` → open Schunk gripper, `c` →
    close, `q` → quit
- Added a SchunkGripper `ActionClient` with bounded waits; pause-during-prompt
  via `threading.Event` so the live-line and the input prompt don't fight.

---

## Session 4 — Dice cycle test (find face 1, place it up)

**Topic:** Sequence the pickup → camera → rotate → place flow into one
deterministic script.

**Key prompts:**
- "create me a test program that can run through this sequence until it can
  successfully place the dice on the conveyor after it has read a 1"
- "by knowing two locations on the dice you should be able to know the whole
  dice so think of this algorithmically"
- Several iterations on placement geometry: "I need it to pick it up from the
  flipped position", "place_on_conveyor_flipped should be about 20 cm
  forward — actually 25 mm", "move my z up just a little like 10mm".

**Actions (Claude):**
- Created `ModbusStuff/dice_cycle_test.py` with:
  - All recorded joint waypoints in `POSITIONS` and a parallel
    `CART_POSITIONS` for poses we wanted to nudge in cartesian space
  - `_all_orientations()` enumerates the 24 rotational orientations of a
    Western die. From the (cam1, cam2) pair the unique orientation is found
    and the location of face 1 in the gripper frame is read off.
  - Smart planner (`plan_for_target`): given the inferred orientation, picks
    the minimum-cost action — direct flip placement, double-flip, basic
    placement (gripped face up), or N rotations + flip placement.
  - `rotate_on_conveyor()` does a real 90° die rotation by drop → lift →
    swap wrist → descend → grip on the conveyor.
  - `flip_and_regrab()` for the case where face 1 sits on the cam-2 axis —
    drops with the tilted flip wrist, regrabs from the top with the
    vertical wrist (so the previously-gripped face is now exposed).
  - `verify_via_repickup()` confirms the placed-up face by re-picking with a
    different wrist orientation.
- Added a 0.5 s sleep after every gripper action so the jaws finish moving
  before the next motion starts.
- Optimised camera read: if camera 1 already shows the target face, skip the
  camera 2 move/read entirely.
- Added two waypoints to `CART_POSITIONS` driven by user feedback —
  `place_on_conveyor_flipped_approach` (current saved spot) and an updated
  `place_on_conveyor_flipped` (+25 mm in +Y). Y on the rotate / flip-above
  poses bumped +20 mm; Z on both flip poses lifted 10 mm so the gripper
  doesn't collide on descent.

---

## Session 5 — Final program: multi-cycle 1→6 with Modbus + conveyor handoff

**Topic:** Productionise `dice_cycle_test.py` into the final assignment
program. Sequential pip counting 1, 2, 3, … with Bill via shared Modbus
state and the physical conveyor handoff.

**Key prompts:**
- "I want a program that is almost identical to dice_cycle_test … BUT
  afterwards it sends it on the conveyor … sets DJ_Has_Dice to false … move
  the dice using the conveyor while waiting for ready_for_pickup to be true
  once it passes the sensor fully …"
- "report Total_Pip_Count to increment by that pip_count every time it
  scans, increment DJ_Retries after every retry cycle, and report
  Last_Known_Pip — the upper face placed on the conveyor."
- Follow-up: "use the last known pip position to know what the next target
  is (until last known pip is 6 where I will just place on the table again
  and go home)"
- Late iteration: gripper should pause briefly after open/close.
- "It shouldn't have went home after moving above the dice — should have
  been above the second conveyor" (after I left a `go_home` between the
  cycle and the conveyor handoff).

**Actions (Claude):**
- Created `ModbusStuff/final_assignment_dj.py`:
  - Reuses the dice-cycle logic, but parameterised by `target` so we can
    find any face value (renamed `plan_for_one` → `plan_for_target` and
    threaded `target` through `show_to_camera`, `_place_after_flip`,
    `run_dice_cycle`).
  - Modbus client (`pymodbus.ModbusTcpClient`) — host/port via CLI flags.
  - Subscribes to `<robot_name>/prox_readings` and adds a `Conveyor` action
    client at `<robot_name>/conveyor`.
  - On startup writes: `Total_Pip_Count=0`, `DJ_Retries=0`,
    `Last_Known_Pip=0`, `Cycle_Active=True`, `DJ_Has_Dice=True`,
    `Ready_For_Pickup=False`.
  - Every camera scan increments `Total_Pip_Count` by the pip count read.
  - Every retry (rotation or flip) increments `DJ_Retries`.
  - After each successful placement, writes `Last_Known_Pip = target`.
- Conveyor handoff: drives the conveyor forward, waits for the right prox
  sensor to go ON then OFF (die fully past), stops the conveyor, sets
  `DJ_Has_Dice=False` and `Ready_For_Pickup=True`.
- Wait for Bill: two-stage so it doesn't trigger trivially — waits for
  `Bill_Has_Dice` to first become True, then for `Bill_Has_Dice=False AND
  Ready_For_Pickup=True`.
- Receive from second conveyor: starts the conveyor, parks the arm at
  `second_conveyor_pickup_above`, waits for the right sensor to go ON,
  stops the conveyor, descends to `second_conveyor_pickup`, closes gripper,
  lifts. Joints captured live by the user.
- Driver loop: target = 1 first, then `target = Last_Known_Pip + 1` after
  each receive. If `Last_Known_Pip == 6` on receive, `place_on_table()`
  and end. If DJ itself targets 6, hand off and end (Bill finalises).
- Replaced the post-cycle `go_home` with a move to
  `second_conveyor_pickup_above` so the arm is already parked over the
  receive zone while Bill works.

---

## Session 6 — Cleanup for public publication

**Topic:** Make the repo safe to push as a public PR — strip identifiable
network/host information.

**Prompts (summarised):**
- "I cannot have identifiable information such as IP addresses in my github
  commit. can you scan the folder and make it friendly to that?"
- "sure" to scrubbing local paths from the operation log if any existed.

**Actions (Claude):**
- Searched the staged-and-untracked tree for IPv4 patterns and known
  identifiers. Found four hard-coded lab IPs in legacy ModbusStuff scripts.
- Replaced each with a `os.environ.get(...)` lookup so the scripts still
  run when the IP is supplied via environment, but the literal addresses
  never appear in source:
  - `ModbusStuff/A.py` and `ModbusStuff/handoff_robotA.py`:
    `ModbusTcpClient(...)` host now comes from `PARTNER_IP` (default
    `localhost`).
  - `ModbusStuff/DJ_ASN6_mqtt_fanuc driver.py`: `broker` from
    `MQTT_BROKER` (default `localhost`); `dj_ip` from `ROBOT_IP` (default
    `0.0.0.0`).
- Extended `.gitignore`:
  ```
  .env
  .env.local
  __pycache__/
  *.py[cod]
  *.egg-info/
  ```
  `.env` was already untracked; this prevents accidental adds from this
  point on. `MVSDK_PATH`, robot IP, and camera IP all live in `.env` only.
- No paths under `/home/<user>/...` were present in
  `OPERATION_LOG_FINAL.md`; nothing else to scrub there.

**Confirmed clean:** post-scrub grep across all to-be-committed files finds
zero lab IPs (`10.8.x`, etc.). Upstream defaults like `172.29.208.x` and
camera-vendor defaults like `192.168.0.x` were left untouched — they were
already in the upstream tracked code or are factory defaults, not lab
identifiers.

---

## Submission checklist

- [x] Source-code IPs scrubbed; `.env` ignored
- [x] `.gitignore` updated
- [x] Operation log present (this file)
- [ ] Push branch `Michael-Habermann-finalassignment` to personal fork
- [ ] Open PR against upstream `v1.1`
- [ ] Canvas: operation log, video, GitHub username
