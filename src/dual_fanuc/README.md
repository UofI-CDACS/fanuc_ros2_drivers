<a name="readme-top"></a>

[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)

# ClaudeDualFanucAssignment

A two-robot FANUC dice-orientation pipeline. Robot 1 (Schunk gripper) picks a die from a bin, scans it with an overhead MindVision GigE camera, flips it so a target pip lands on top, and hands it to Robot 2 (OnRobot gripper) over a back-and-forth conveyor pair. The two robots iterate through pips 1–6 and end the run with the final die placed at a fixed termination pose.

This is a fork of an upstream FANUC ROS2 EtherNet/IP driver (`v1.1`). The upstream driver, action servers, message publishers, and TP programs are kept; everything above the driver is new.

<details>
<summary>Table of contents</summary>

- [What this fork adds](#what-this-fork-adds)
- [Prerequisites](#prerequisites)
- [Setup](#setup)
- [Running it](#running-it)
- [Calibration](#calibration)
- [License](#license)

</details>


## What this fork adds

| Addition | Where | Purpose |
|---|---|---|
| `dual_fanuc` ROS2 package | `src/dual_fanuc/` | Two-robot orchestration: `robot1.py` (pick + present + sort) and `robot2.py` (receive + reorient + return). Shared lookup table in `pip_lookup.py` maps `(top, adjacent)` scan pairs to the single rotation primitive that brings each target pip to the top. |
| MindVision camera node | `src/dual_fanuc/dual_fanuc/mv_camera_node.py` | Wraps the vendor `mvsdk` Python bindings as a ROS2 node. Publishes `/mv_camera/image_raw` and exposes a `/mv_camera/capture` trigger service for synchronous frames. Auto-selects the routable host NIC when multiple host IPs reach the camera. |
| HSV + HoughCircles pip detector | `_crop_die` / `_count_pips` in `robot1.py` and `robot2.py` | Crops the die out of the scene by ROI → HSV mask → contour fill → bitwise-and, then counts pips on the masked crop. Tunable from `scripts/calibrate_hsv.py` and `scripts/calibrate_hough.py` (both write to both robot files). |
| OnRobot gripper action server | `src/action_servers/action_servers/onrobot_server.py` | Width + force action interface for the OnRobot gripper on Robot 2. Robot 1 keeps the upstream Schunk action server. |
| Rotation primitives + lookup-driven orient | `_rotate_x_pos_90`, `_rotate_x_neg_90`, `_rotate_y_neg_90`, `_rotate_y_pos_90`, `_flip_x_180`, `_orient_pip` / `_scan_orient_place` | Per-robot pure-motion primitives composed from a single x-flip primitive. Robot 1's single primitive is `_rotate_x_pos_90`, Robot 2's is `_rotate_x_neg_90` (the robots are mounted in mirrored orientations). The `pip_lookup` action labels are interpreted in each robot's local frame. |
| `just` task runner | `justfile` | One-line recipes for every common workflow. `just --list` for the full set. |
| Local-only env file | `.env` (gitignored) + `.env.example` | Robot names and IPs are read from `.env`; nothing real ships in the repo. |
| Vendored Python SDK | `third_party/mvsdk.py`, copied into the venv by `just setup` | Avoids touching system site-packages. |
| Rotation testers | `just rotate1`, `just rotate2` | Drive each robot through all five rotation primitives in isolation; useful when the camera or other robot is unavailable. |

The upstream pieces — `fanuc_interfaces`, `msg_publishers`, `srv_services`, the cartesian/joint/Schunk/conveyor action servers, and the `FANUC_TP_Program/*.tp` files — are left as they were.


## Prerequisites

- **Linux** (developed on Ubuntu 24.04 with kernel 6.x).
- **ROS2 Jazzy.** Humble may also work but is untested in this fork.
- **Python 3.12** with `python3.12-venv`.
- **`just`** ([install](https://github.com/casey/just?tab=readme-ov-file#installation)).
- **MindVision Linux SDK** for the GigE camera. The vendor archive is included as `linuxSDK_V2.1.0.49(202602041120)/` locally but is gitignored — download it yourself from MindVision and run `sudo ./install.sh` from the extracted folder. This installs `/usr/lib/libMVSDK.so` and udev rules.
- **Camera networking.** The MindVision camera ships in DHCP mode and falls back to a `169.254.x.x` link-local address with no DHCP server. Add a `169.254.0.1/16` IPv4 alias on the NIC the camera is plugged into so the host can route to the camera regardless of subnet (this is the typical fix when `just grab-sdk` returns MindVision error `-14`).
- **FANUC TP programs.** Load both files from `FANUC_TP_Program/` onto each controller:
  - `ros2_eip_back.tp` — must be running in the **background** on each controller.
  - `ros2_eip_mainv2.tp` — runs in the **foreground** while you're using ROS.
- **`numpy<2`.** ROS Jazzy's `cv_bridge` is compiled against the NumPy 1.x ABI. The pin is in `requirements.txt`; do not bump it without rebuilding `cv_bridge`.


## Setup

```sh
# 1. Clone and configure environment
git clone <your-fork-url> dual_fanuc_ws
cd dual_fanuc_ws
cp .env.example .env       # then edit ROBOT_1_NAME/IP and ROBOT_2_NAME/IP

# 2. One-time tooling
sudo apt install python3.12-venv python3-rosdep
sudo rosdep init && rosdep update          # first time on this machine only
rosdep install -i --from-path src --rosdistro jazzy -y

# 3. Install MindVision SDK (download archive from vendor first)
cd linuxSDK_V*/ && sudo ./install.sh && cd ..

# 4. Python venv + workspace build
just setup                  # creates .venv, installs requirements.txt, vendors mvsdk.py
source /opt/ros/jazzy/setup.bash
just build                  # colcon build
```

Every new shell after that needs the workspace overlay sourced before running ROS commands:

```sh
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export PYTHONPATH=$PWD/.venv/lib/python3.12/site-packages:$PYTHONPATH
```

`just source` prints the same three lines if you forget.


## Running it

`just --list` shows every recipe. The most common ones:

| Recipe | What it does |
|---|---|
| `just launch` | Launch both robot server stacks + the camera node in one terminal. |
| `just launch1` / `just launch2` | Launch one robot's server stack only. `launch1` also starts the camera node. |
| `just run` | Start the Robot 1 orchestration node (the main pipeline). |
| `just run2` | Start the Robot 2 orchestration node. |
| `just rotate1` / `just rotate2` | Drive one robot through all five rotation primitives. Doesn't need the camera or the other robot. |
| `just rotation-test` | Step through `_rotate_x_neg_90` then `_rotate_y_neg_90` interactively (Robot 1). |
| `just open-schunk` / `just close-schunk` | Toggle Robot 1's gripper (needs `launch1`). |
| `just open-onrobot` / `just close-onrobot` | Toggle Robot 2's gripper (needs `launch2`). |
| `just conveyer-back` / `just conveyer-front` | Run a conveyor forward until Ctrl-C. |
| `just kill` | Force-kill any orphaned ROS nodes. |

Typical full-pipeline sequence: in one terminal `just launch`, then in two more terminals `just run` and `just run2`. Place a die in the bin in front of Robot 1 and the run starts.


## Calibration

Pip detection has three knobs that need tuning per camera/lighting setup. Both calibration scripts edit `robot1.py` and `robot2.py` in place so the tuned values survive re-builds.

```sh
just grab            # capture one frame via the running camera node
just grab-sdk        # capture one frame directly via the SDK (camera node must NOT be running)
just calibrate-hsv   # tune ROI + HSV thresholds + minimum contour area
just calibrate-hough # tune HoughCircles (dp, minDist, param1, param2, min/max radius)
```

The calibration UI draws a sidebar inside the image with each parameter labelled and color-grouped — opencv-python's Linux Qt build mangles the native trackbar labels, so the sidebar is the source of truth.


## License

GPL v3 — same as upstream. See [`LICENSE.txt`](LICENSE.txt).

<p align="right">(<a href="#readme-top">back to top</a>)</p>
