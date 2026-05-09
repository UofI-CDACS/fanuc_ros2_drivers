#!/usr/bin/env python3
"""
robot1_dice_control.py
======================
Example robot control file that uses dice_vision to detect die pip count
and rotate it to show an EVEN number on top.

Copy this file for each robot and change:
  - ROBOT_IP
  - The joint offsets inside each rotation method
  - target='even' → target='odd' for the second robot

Run this file directly:
    python3 robot1_dice_control.py
"""

import sys
import os
from ctypes import addressof, c_ubyte

import numpy as np

# --- Path setup -----------------------------------------------------------
# These let Python find dice_vision/ and pip_test.py from this directory.
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

# dice_vision package (the five files we built)
from dice_vision import DiceController, RotationSchema

# Project's existing camera + detection code
from pip_test import detect_pips
import mvsdk

# Project's existing robot communication layer
from src.msg_publishers.dependencies.robot_controller import robot


# --------------------------------------------------------------------------
#  Configuration — change these values for your setup
# --------------------------------------------------------------------------

ROBOT_IP    = '172.29.208.0'   # IP address of this robot
CAMERA_IDX  = 0                # MindVision camera index
AE_TARGET   = 80               # Auto-exposure target (same as pip_test.py)


# --------------------------------------------------------------------------
#  Step 1 — Rotation schema for THIS robot
#
#  Each method maps an abstract rotation name to physical joint movements.
#  Use write_joint_offset(joint, degrees) to nudge one joint by ±N degrees
#  from its current position, OR write_joint_pose([j1,j2,...,j6]) to move
#  all joints to an absolute pose.
#
#  !! You must fill in the joint numbers and degree values that match how
#     your robot is grasping the die. !!
#  !! Use the FANUC teach pendant to find the right values experimentally. !!
#
#  Rotation reference (what each name means physically):
#    roll_forward  — die tips away from camera (front face drops down)
#    roll_backward — die tips toward camera    (front face rises up)
#    roll_right    — die tips to the robot's right
#    roll_left     — die tips to the robot's left
#    spin_cw       — die spins clockwise from above (Z-axis rotation)
#    spin_ccw      — die spins counter-clockwise from above
# --------------------------------------------------------------------------

class Robot1Schema(RotationSchema):
    """Rotation schema for Robot 1.  Holds the die with a Schunk gripper."""

    def __init__(self, bot: robot):
        self.bot = bot

    def _execute(self, rotation: str) -> bool:
        """Translate rotation name → actual joint movement.  Returns True on success."""

        if rotation == 'roll_forward':
            # TODO: replace with the joint + degrees that tip the die forward
            self.bot.write_joint_offset(joint=5, value=90.0)   # example

        elif rotation == 'roll_backward':
            # TODO: reverse of roll_forward
            self.bot.write_joint_offset(joint=5, value=-90.0)  # example

        elif rotation == 'roll_right':
            # TODO: joint that tips the die to the right
            self.bot.write_joint_offset(joint=4, value=90.0)   # example

        elif rotation == 'roll_left':
            # TODO: reverse of roll_right
            self.bot.write_joint_offset(joint=4, value=-90.0)  # example

        elif rotation == 'spin_cw':
            # TODO: joint that spins the die clockwise from above (usually J6)
            self.bot.write_joint_offset(joint=6, value=90.0)   # example

        elif rotation == 'spin_ccw':
            # TODO: reverse of spin_cw
            self.bot.write_joint_offset(joint=6, value=-90.0)  # example

        return True  # write_joint_offset blocks until the move completes


# --------------------------------------------------------------------------
#  Step 2 — Camera helpers (thin wrappers around pip_test.py)
# --------------------------------------------------------------------------

def open_mindvision_camera(index: int, ae_target: int):
    """Open a MindVision camera and return (hCamera, buf, is_color)."""
    dev_list = mvsdk.CameraEnumerateDevice()
    if len(dev_list) <= index:
        raise RuntimeError(f'No MindVision camera at index {index}.')

    h       = mvsdk.CameraInit(dev_list[index])
    max_w, max_h, b_color = mvsdk.CameraGetCapabilityEx2(h)
    is_color = (b_color != 0)
    ch       = 3 if is_color else 1

    if is_color:
        mvsdk.CameraSetIspOutFormat(h, mvsdk.CAMERA_MEDIA_TYPE_BGR8)
    else:
        mvsdk.CameraSetIspOutFormat(h, mvsdk.CAMERA_MEDIA_TYPE_MONO8)

    buf = (c_ubyte * (max_w * max_h * ch))()
    mvsdk.CameraSetAeState(h, True)
    mvsdk.CameraSetAeTarget(h, ae_target)
    mvsdk.CameraPlay(h)

    print('Warming up camera...')
    for _ in range(10):
        try:
            raw, head = mvsdk.CameraGetImageBuffer(h, 1000)
            mvsdk.CameraReleaseImageBuffer(h, raw)
        except mvsdk.CameraException:
            pass

    return h, buf, is_color, ch


def grab_frame_bgr(h, buf, is_color, ch) -> np.ndarray:
    """Grab one frame and return a BGR numpy array."""
    raw, head = mvsdk.CameraGetImageBuffer(h, 2000)
    mvsdk.CameraImageProcess(h, raw, addressof(buf), head)
    mvsdk.CameraReleaseImageBuffer(h, raw)
    n    = head.iWidth * head.iHeight * ch
    view = (c_ubyte * n).from_address(addressof(buf))
    img  = np.frombuffer(view, dtype=np.uint8).reshape(
               (head.iHeight, head.iWidth, ch)).copy()
    if not is_color:
        import cv2
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    return img


# --------------------------------------------------------------------------
#  Step 3 — Main routine
# --------------------------------------------------------------------------

def run_dice_rotation(target: str = 'even'):
    """
    Full pipeline for a randomly placed die with unknown orientation:
      1. Open camera + connect to robot
      2. Read top face from camera                   (knows top + bottom)
      3. Choose optimal discovery rotation            (minimises worst-case total moves)
      4. Execute discovery rotation; read new top     (reveals one side face → full state known)
      5. Plan minimum remaining rotations to target
      6. Execute them with per-step camera verification
    """

    # -- Connect to robot ---------------------------------------------------
    print(f'Connecting to robot at {ROBOT_IP}...')
    bot    = robot(ROBOT_IP)
    schema = Robot1Schema(bot)

    # -- Open camera --------------------------------------------------------
    print(f'Opening camera {CAMERA_IDX}...')
    h_cam, buf, is_color, ch = open_mindvision_camera(CAMERA_IDX, AE_TARGET)

    # Convenience closure: capture one frame and return pip count (1-6) or None
    def detect_top() -> 'int | None':
        frame = grab_frame_bgr(h_cam, buf, is_color, ch)
        _, _, _, _, pip_count = detect_pips(frame)
        return pip_count if pip_count > 0 else None

    try:
        ctrl   = DiceController(schema=schema)

        # discover_and_rotate handles the full unknown-orientation pipeline:
        #   - reads top face, picks best discovery rotation, executes it,
        #     reconstructs full die state, then plans+executes minimum remaining moves.
        result = ctrl.discover_and_rotate(
            target=target,
            top_verifier=detect_top,
        )

        # -- Report -----------------------------------------------------------
        if result['success']:
            print(f'\nDone!')
            print(f'  Initial top face : {result["initial_top"]}')
            print(f'  Initial die state: {result["initial_state"]}')
            print(f'  Rotation sequence: {result["rotation_sequence"]}')
            print(f'  Final top face   : {result["final_state"].top}')
        else:
            print('\nFailed — see log above for details.')

        if result['verification_log']:
            print('\nVerification log:')
            for entry in result['verification_log']:
                match_str = entry.get('match')
                status    = ('OK' if match_str else 'MISMATCH') if match_str is not None else '—'
                print(f'  {entry["rotation"]:15s}  detected={entry["detected_top"]}  [{status}]')

    finally:
        mvsdk.CameraStop(h_cam)
        mvsdk.CameraUnInit(h_cam)


# --------------------------------------------------------------------------
if __name__ == '__main__':
    # Change 'even' to 'odd' for Robot 2
    run_dice_rotation(target='even')
