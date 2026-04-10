# dice_task

A ROS2 package that uses a FANUC robot arm with a Schunk gripper and an overhead MindVision GigE camera to pick up three dice, photograph each one, count the pips using OpenCV, and report a running total.

---

## Hardware

| Component | Details |
|---|---|
| Robot | FANUC arm via EtherNet/IP |
| Gripper | Schunk (parallel jaw) |
| Camera | MindVision GigE (MV-GE134GC-IT, overhead mount) |

The existing `fanuc_ros2_drivers` action servers and the MindVision SDK must be available before running this package.

---

## Package Structure

```
src/dice_task/
├── dice_task/
│   ├── dice_roller.py      # Main task node — pick, photo, count, total
│   └── mv_camera_node.py   # MindVision camera driver node
├── launch/
│   └── dice_task.launch.py # Starts mv_camera_node (included by start.launch.py)
├── scripts/
│   ├── grab.py             # One-shot frame capture to /tmp/grab.bmp
│   ├── calibrate_hsv.py    # Interactive HSV crop tuner
│   └── calibrate_hough.py  # Interactive HoughCircles pip-detection tuner
├── package.xml
├── setup.py
└── .env.example            # Template — copy to .env and fill in real values
```

---

## Dependencies

### Python packages

| Package | Purpose | Install |
|---|---|---|
| `pycomm3` | EtherNet/IP robot comms (via `.venv`) | `pip install pycomm3` |
| `mvsdk` | MindVision SDK Python bindings | Copy from SDK: `demo/python_demo/mvsdk.py` |
| `opencv-python` | Image processing | `pip install opencv-python` |
| `cv_bridge` | ROS2 ↔ OpenCV image conversion | via `rosdep` |

### MindVision SDK

Download the Linux SDK from MindVision, run `install.sh` to place `libMVSDK.so` in `/lib`, then copy `demo/python_demo/mvsdk.py` into your Python environment.

The `justfile` handles `PYTHONPATH` automatically — see [Building & Running](#building--running).

---

## Configuration

### Environment variables

Copy `.env.example` to `.env` in the repo root and fill in your values:

```
ROBOT_NAME=your_robot_name
ROBOT_IP=your_robot_ip
```

`.env` is gitignored so credentials are never committed. `just` loads it automatically via `set dotenv-load := true`.

### Position constants (dice_roller.py)

All tunable values are at the top of `dice_roller.py`. **You must set these to match your physical setup before running.**

| Constant | Default | Description |
|---|---|---|
| `DICE_WIDTH` | `80.0` mm | Center-to-center spacing between dice in +x |
| `APPROACH_OFFSET_Z` | `2 × DICE_WIDTH` | Height above pick position for approach/retreat |
| `_d1` | `(469.0, -15.4, -178.5, w=179.9, p=0, r=30)` | Die 1 pick position (Cartesian, mm/deg) |
| `CAMERA_PRESENT_JOINTS` | `(60, 33, 20, -57, -34, 80)` | Joint angles for camera presentation (deg) |
| Home joints | `(0, 0, 0, 0, -90, 30)` | Hard-coded in `go_home()` |

Die 2 and Die 3 positions are computed automatically as `_d1.x + i × DICE_WIDTH`.

### Vision constants (dice_roller.py)

Tune these with the calibration scripts rather than editing by hand.

| Constant | Purpose |
|---|---|
| `HSV_H/S/V_LOW/HIGH` | HSV range for isolating the yellow die face |
| `HOUGH_DP/MIN_DIST/PARAM1/PARAM2/MIN_R/MAX_R` | HoughCircles pip detection parameters |

---

## Building & Running

This project uses [`just`](https://github.com/casey/just) as a command runner.

```bash
# 1. Build all packages
just build

# 2. Terminal 1 — launch all nodes (FANUC servers + camera)
just launch

# 3. Terminal 2 — run the dice task
just run
```

`just build` and `just launch` automatically prepend the `.venv` packages to `PYTHONPATH`.

### Other recipes

```bash
just grab           # Capture one frame → /tmp/grab.bmp  (nodes must be running)
just calibrate-hsv  # Tune HSV crop interactively        (run grab first)
just calibrate      # Tune HoughCircles interactively     (run calibrate-hsv first)
```

---

## Calibration Workflow

Run this once after any change to the camera position, lighting, or die colour:

```
just grab  →  just calibrate-hsv  →  just calibrate  →  just run
```

1. **`just grab`** — saves a raw frame to `/tmp/grab.bmp`
2. **`just calibrate-hsv`** — opens a split-panel window with HSV sliders; adjust until only the die face is highlighted; **quit (q)** to save `/tmp/grab_cropped.bmp` and write `HSV_*` constants back into `dice_roller.py`
3. **`just calibrate`** — opens the cropped image with HoughCircles sliders; adjust until all pips are detected cleanly; **quit (q)** to write `HOUGH_*` constants back into `dice_roller.py`
4. **`just run`** — full task using the saved constants

---

## Task Sequence

For each of the three dice:

1. Open gripper
2. Move to approach height above die (`pick_z + APPROACH_OFFSET_Z`)
3. Descend to pick position
4. Close gripper
5. Lift die back to approach height
6. Move to camera present joint position
7. Trigger `/mv_camera/capture` service → receive frame via `/mv_camera/image_raw`
8. HSV-mask crop to isolate the die face
9. HoughCircles pip count
10. Return die to its original position and release
11. Lift away before moving to next die

After all three dice: report final total and return to home.

Debug images (raw crop + annotated circles) are saved to `/tmp/die_N_raw.png` and `/tmp/die_N_debug.png` after each die.

---

## ROS2 Interface

### Nodes

| Node | Executable | Description |
|---|---|---|
| `mv_camera_node` | `mv_camera_node` | Streams frames from MindVision camera |
| `dice_roller` | `dice_roller` | Runs the pick-photo-count task |

### Topics

| Topic | Type | Direction |
|---|---|---|
| `/mv_camera/image_raw` | `sensor_msgs/Image` | Published by `mv_camera_node` |
| `/<robot_name>/cartesian_pose` | `fanuc_interfaces/CartPose` | Action — used by `dice_roller` |
| `/<robot_name>/joint_pose` | `fanuc_interfaces/JointPose` | Action — used by `dice_roller` |
| `/<robot_name>/schunk_gripper` | `fanuc_interfaces/SchunkGripper` | Action — used by `dice_roller` |

### Services

| Service | Type | Description |
|---|---|---|
| `/mv_camera/capture` | `std_srvs/Trigger` | Grab and publish one fresh frame |
