#!/usr/bin/env python3
"""
robot1_odd.py
=============
Robot 1 — finds pips 1, 3, 5 in order.

Callable from other files:
    from robot1_odd import go_to_pip
    state = go_to_pip(1)            # unknown start, does 1 discovery roll
    state = go_to_pip(3, state)     # continues from known state, no extra roll
    state = go_to_pip(5, state)

To run standalone:
    python3 robot1_odd.py
"""

import sys
sys.path.insert(0, '/home/astrum/Documents/Robots2Final/fanuc_ros2_drivers')

import mvsdk
from pip_test import detect_pips, open_camera, grab_frame
from src.msg_publishers.dependencies.robot_controller import robot
from dice_finder import find_pip, run_sequence, ODD_SEQUENCE


# ── Robot and camera setup ────────────────────────────────────────────────────

ROBOT_IP     = '172.29.208.0'            # ← your robot's IP
CAMERA_INDEX = 0                          # ← MindVision camera index

bot          = robot(ROBOT_IP)            # connect to robot over EtherNet/IP
h_cam, buf, is_color = open_camera(CAMERA_INDEX)   # open camera once


# ════════════════════════════════════════════════════════════════════════════
#  ROTATION DEFINITIONS  (fill in your joint values here)
#
#  Coordinate frame:
#    X axis — perpendicular to the table (front / back)
#    Y axis — parallel to the table      (left / right)
#    Z axis — vertical                   (up / down)
#
#  CW / CCW convention — viewed from the POSITIVE end of each axis:
#    Z: viewed from ABOVE   (most intuitive — same as looking down)
#    Y: viewed from the RIGHT side of the table
#    X: viewed from the FRONT of the table (camera side)
#
#  Each rotation is a list of steps run in order.
#  Step types:
#    ('joint_offset', joint_number, degrees)    — offset one joint by ±N degrees
#    ('joint_pose',   [j1,j2,j3,j4,j5,j6])     — move all joints to these angles
#    ('cartesian',    [x, y, z, w, p, r])       — move to a Cartesian position
#
#  Add as many steps as needed per rotation.  Mix types freely.
# ════════════════════════════════════════════════════════════════════════════

# ── Z axis (vertical) ────────────────────────────────────────────────────────
#    Spinning around Z leaves the TOP face unchanged; only the side faces rotate.
#    CW from above:  front → right → back → left → front
#    CCW from above: front → left  → back → right → front

ROTATE_Z_CW = [                       # spin die clockwise from above
    ('joint_offset', 6,  90.0),       # TODO: replace with your joint + degrees
]

ROTATE_Z_CCW = [                      # spin die counter-clockwise from above
    ('joint_offset', 6, -90.0),       # TODO
]

# ── Y axis (left / right, parallel to table) ─────────────────────────────────
#    CW from the right:  top tips toward the camera → BACK face becomes new top
#    CCW from the right: top tips away from camera  → FRONT face becomes new top

ROTATE_Y_CW = [                       # top tips toward camera, back face → top
    ('joint_offset', 5,  90.0),       # TODO
]

ROTATE_Y_CCW = [                      # top tips away from camera, front face → top
    ('joint_offset', 5, -90.0),       # TODO
]

# ── X axis (front / back, perpendicular to table) ────────────────────────────
#    CW from the front: top tips to the right → LEFT  face becomes new top
#    CCW from the front: top tips to the left → RIGHT face becomes new top

ROTATE_X_CW = [                       # top tips right, left face → top
    ('joint_offset', 4,  90.0),       # TODO
]

ROTATE_X_CCW = [                      # top tips left, right face → top
    ('joint_offset', 4, -90.0),       # TODO
]


# ════════════════════════════════════════════════════════════════════════════
#  MAPPING — connects your axis names to the planner's internal names
#
#  The planner always requests moves by names like 'roll_forward'.
#  This table is the only place you need to know that correspondence.
#  Do not rename the keys on the left — rename the values on the right instead.
#
#  What each planner name does to the die:
#    roll_forward  → front face becomes top   (Y-CCW from right)
#    roll_backward → back  face becomes top   (Y-CW  from right)
#    roll_right    → left  face becomes top   (X-CW  from front)
#    roll_left     → right face becomes top   (X-CCW from front)
#    spin_cw       → top unchanged, CW spin   (Z-CW  from above)
#    spin_ccw      → top unchanged, CCW spin  (Z-CCW from above)
# ════════════════════════════════════════════════════════════════════════════

ROTATIONS = {
    'roll_forward':  ROTATE_Y_CCW,    # Y-CCW → front face to top
    'roll_backward': ROTATE_Y_CW,     # Y-CW  → back  face to top
    'roll_right':    ROTATE_X_CW,     # X-CW  → left  face to top
    'roll_left':     ROTATE_X_CCW,    # X-CCW → right face to top
    'spin_cw':       ROTATE_Z_CW,     # Z-CW  → top stays, side faces rotate CW
    'spin_ccw':      ROTATE_Z_CCW,    # Z-CCW → top stays, side faces rotate CCW
}


# ── Step executor ─────────────────────────────────────────────────────────────

def execute(rotation_name):
    """Look up the axis rotation by planner name and run each step in order."""
    for step in ROTATIONS[rotation_name]:          # get the step list for this rotation
        step_type = step[0]                        # first element is always the type

        if step_type == 'joint_offset':            # ('joint_offset', joint, degrees)
            _, joint, degrees = step
            bot.write_joint_offset(joint=joint, value=degrees)   # move one joint

        elif step_type == 'joint_pose':            # ('joint_pose', [j1..j6])
            _, joints = step
            bot.write_joint_pose(joints)                          # set all joints

        elif step_type == 'cartesian':             # ('cartesian', [x,y,z,w,p,r])
            _, coords = step
            bot.write_cartesian_position(coords)                  # cartesian move


# ── Camera ────────────────────────────────────────────────────────────────────

def detect_top():
    """Photograph the die and return the pip count on top (1-6), or None."""
    frame = grab_frame(h_cam, buf, is_color)       # grab one frame from MindVision
    _, _, _, _, count = detect_pips(frame)         # HSV mask → warp → Otsu → count
    return count if count > 0 else None            # 0 means die not detected


# ── Public callable (import this from other files) ────────────────────────────

def go_to_pip(target_pip, current_state=None):
    """
    Rotate the die to show target_pip on top.

    First call  (current_state=None): takes one photo, does one discovery roll,
                takes another photo, then goes to the target with minimum moves.
    Later calls (current_state=state): skips discovery, goes straight to target.

    Returns the updated DiceState — pass it into the next call.

    Example from another file:
        from robot1_odd import go_to_pip

        state = go_to_pip(1)           # unknown start → 1 discovery roll + route to 1
        state = go_to_pip(3, state)    # known state   → routes directly to 3
        state = go_to_pip(5, state)    # known state   → routes directly to 5
    """
    return find_pip(target_pip, detect_top, execute, current_state)


# ── Standalone ────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    try:
        run_sequence(ODD_SEQUENCE, detect_top, execute)   # 1 → 3 → 5
    finally:
        mvsdk.CameraStop(h_cam)
        mvsdk.CameraUnInit(h_cam)
