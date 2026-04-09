# FANUC ROS2 Dice Inspection — Session Log
**Date:** 2026-04-09
**File:** `Controlling_robots_using_claude.py`

---

## Summary

Developed and debugged an autonomous FANUC dice inspection routine that picks up a yellow dice, presents it to an overhead camera, counts the pips using HSV colour masking, and repeats for multiple rolls. The robot rotates the dice by changing the wrist orientation at drop time so each pick-up shows a new face.

---

## Issues Fixed

### 1. Pip count was wrong (original HoughCircles approach)
The original `count_pips` used `cv2.HoughCircles` with fixed parameters. It had no way to distinguish the dice from the background, causing false positives and misses. Replaced entirely — see Vision section below.

### 2. No wait for dice orientation change
After completing a roll, the robot immediately moved to pick up the next dice without giving time to reorient. Added a prompt (`input()`) before `pick_dice()` on rolls 2+ so the user could place/orient the dice first. This was later replaced by the automatic rotation approach (see below).

### 3. Robot didn't return home after completion
`go_home()` was never called after the results report. Added call at the end of `run()`.

---

## Vision Pipeline — HSV Colour Masking

Replaced HoughCircles (and a brief attempt at Claude API vision) with a local HSV masking pipeline. No model files or API keys required.

### Why HSV works here
- Dice: bright yellow, very high saturation (S≈235)
- Pips: near-black (V≈22–25)
- Table: natural wood — similar hue to dice but much lower saturation (S≈30–100)
- Camera position is fixed overhead, lighting is consistent

### Pipeline steps
1. **Yellow mask** — `cv2.inRange` with `YELLOW_LO`/`YELLOW_HI` isolates the dice face
2. **Morphological cleanup** — CLOSE then OPEN with 9×9 ellipse kernel to fill gaps and remove noise
3. **Fill solid** — find the largest yellow contour and draw it filled. This is critical: without it, the pip holes in the yellow mask cancel out the pip pixels in the AND step
4. **Black mask** — anything with HSV Value ≤ `BLACK_V_MAX` inside the filled dice region
5. **Contour filtering** — keep blobs between `PIP_AREA_MIN` and `PIP_AREA_MAX` px²
6. **Count and clamp** — `min(len(pip_contours), 6)`
7. **Save debug image** — annotated PNG saved to `~/Desktop/dice_roll_N.png`

### Tuned HSV values (measured with debug_colors.py)
| Constant | Value | Notes |
|---|---|---|
| `YELLOW_LO` | `(18, 180, 150)` | Tight S≥180 excludes wood table |
| `YELLOW_HI` | `(24, 255, 255)` | |
| `BLACK_V_MAX` | `60` | Pip measured at V≈22–25, plenty of margin |
| `PIP_AREA_MIN` | `30` | Smallest pip on table ≈42 px² |
| `PIP_AREA_MAX` | `2000` | Largest pip at camera height ≈315 px² |

### Key bug: pip holes blocking detection
The yellow mask had holes exactly where the pips are (pips are dark, not yellow). ANDing black_mask with a holey yellow_mask produced nothing. Fixed by filling the largest yellow contour solid before the AND.

---

## debug_colors.py

Created a standalone HSV colour picker tool (`debug_colors.py`) to tune values without running the robot:

- **Hover** over any pixel → HSV value overlaid on frame
- **Click** → locks sample and prints to terminal
- **Windows shown:** main annotated feed, yellow mask, black mask, pip mask
- **Terminal output:** blob areas (largest first) each frame — used to set `PIP_AREA_MIN`/`MAX`

Run with:
```bash
source install/setup.bash
python3 debug_colors.py
```

---

## Robot Motion Changes

### PICK_ABOVE before user prompt (intermediate approach)
For rolls 2+, the robot moved to `PICK_ABOVE` and opened the gripper before waiting for the user to place the dice, so the user could see exactly where to put it.

### Dice rotation — final approach
Instead of asking the user to manually rotate the dice, the robot rotates it automatically by changing the `r` (wrist rotation) component of `DROP_POSE` by +90° each roll:

```python
self.release_dice(extra_r=90.0 * (roll_num - 1))
```

- Roll 1 → drops at `r = 120°` (base)
- Roll 2 → drops at `r = 210°` (+90°)
- Roll 3 → drops at `r = 300°` (+180°)

The robot always picks up with the same fixed `PICK_DOWN` orientation, so the dice is effectively rotated 90° further each round.

### Joint 6 rotation approach (attempted, removed)
An alternative was tried: subscribing to `/{ROBOT_NAMESPACE}/cur_joints` (`fanuc_interfaces.msg.CurJoints`), reading the current joint 6 angle, and sending a new `JointPose` with joint 6 incremented by 90°. This worked but was more complex than needed. Removed in favour of the DROP_POSE `r` offset approach above.

---

## Final Configuration

```python
ROBOT_NAMESPACE  = 'Beaker'
NUM_ROLLS        = 3
CAMERA_IP        = 'Camera_IP'

PICK_ABOVE  = dict(x=470.0, y=-15.0, z=-18.0,   w=179.9, p=0.0,   r=30.0)
PICK_DOWN   = dict(x=470.0, y=-15.0, z=-185.0,  w=179.9, p=0.0,   r=30.0)
PICK_LIFT   = dict(x=470.0, y=-15.0, z=-18.0,   w=179.9, p=0.0,   r=30.0)
CAMERA_POSE = dict(x=490.0, y=890.0, z=881.0,   w=73.0,  p=-66.0, r=-170.0)
DROP_POSE   = dict(x=470.0, y=-15.0, z=-185.0,  w=179.9, p=0.0,   r=120.0)

YELLOW_LO    = (18, 180, 150)
YELLOW_HI    = (24, 255, 255)
BLACK_V_MAX  = 60
PIP_AREA_MIN = 30
PIP_AREA_MAX = 2000
```

---

## Run Instructions

```bash
# Terminal 1 — start robot driver
ros2 launch launch/start.launch.py robot_name:=Beaker robot_ip:=<ip>

# Terminal 2 — run inspection
source install/setup.bash
python3 Controlling_robots_using_claude.py

# Tuning only (no robot needed)
python3 debug_colors.py
```
