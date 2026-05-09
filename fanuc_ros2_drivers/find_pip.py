#!/usr/bin/env python3
"""
find_pip.py
===========
Type a pip number — the robot finds it.

Every robot motion is written out as an explicit line here.
To insert a waypoint, add a bot.write_cartesian_position() line.
To change the order of steps, move the lines.

Rotations are called with execute('name') — sequences live in rotations.py.
The camera, orientation math, and BFS planner are imported and called directly.
"""

import os
import sys
import time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))          # finds mvsdk.py next to this file
sys.path.insert(0, '/home/astrum/Documents/Robots2Final/fanuc_ros2_drivers')

import mvsdk
from pip_test import open_camera, grab_frame, detect_pips               # camera functions
from dice_vision.dice_model import from_visible_faces                   # orientation reconstructor
from dice_finder import rotate_to_pip                                    # BFS planner + executor

from rotations import (
    bot,                  # robot connection — lazy, connects on first use
    GRIPPER_OPEN_MM,      # jaw width when open  (currently 120 mm)
    GRIPPER_CLOSE_MM,     # jaw width when gripping (currently 70 mm)
    GRIPPER_FORCE_N,      # grip force in newtons
    HOME_POSE,            # safe resting position
    PICK_POSE,            # die on the table
    PRE_PICK_POSE,        # directly above PICK_POSE
    CAMERA_POSE,          # die in front of camera
    execute,              # execute a named rotation (place → reposition → pick)
)


# ── Camera ────────────────────────────────────────────────────────────────────

CAMERA_INDEX = 0


def _reset_camera(index):
    """Stop and release any lingering camera session so we always start clean.
    Safe to call even if no session is currently open — errors are silently ignored."""
    try:
        dev_list = mvsdk.CameraEnumerateDevice()
        if len(dev_list) > index:
            h = mvsdk.CameraInit(dev_list[index])   # grab whatever handle exists
            mvsdk.CameraStop(h)                      # stop any running capture
            mvsdk.CameraUnInit(h)                    # fully release the device
    except Exception:
        pass   # camera wasn't open — nothing to clean up


_reset_camera(CAMERA_INDEX)                          # ensure clean state before opening
h_cam, buf, is_color = open_camera(CAMERA_INDEX)     # open once; stays open for all grabs


def detect_top(retries=10):
    """Grab a frame and return the pip count visible to the camera (1-6), or None.
    On exception, reinitializes the camera before retrying.
    Also retries if the frame is valid but no pip was detected."""
    global h_cam, buf, is_color
    for attempt in range(1, retries + 1):
        try:
            frame = grab_frame(h_cam, buf, is_color)           # grab one frame
            _, _, _, _, count = detect_pips(frame)             # count pips in frame
            if count > 0:
                return count                                    # success
            print(f'  Camera attempt {attempt}/{retries}: no pip detected in frame')
        except Exception as e:
            print(f'  Camera attempt {attempt}/{retries} failed: {e}')
            try:                                                # reinitialize camera to clear stuck state
                mvsdk.CameraStop(h_cam)
                mvsdk.CameraUnInit(h_cam)
            except Exception:
                pass
            time.sleep(1.0)                                    # let hardware settle after release
            try:
                h_cam, buf, is_color = open_camera(CAMERA_INDEX)   # reopen fresh
            except Exception as re:
                print(f'  Camera reinit failed: {re}')
        if attempt < retries:
            time.sleep(1.0)                                    # fixed pause before next attempt
    print('  Camera failed after all retries.')
    return None


def prompt_pip(label: str) -> int:
    """Ask the operator to manually enter the upward-facing pip count."""
    while True:
        try:
            val = int(input(f'\n  Camera unavailable. Enter pip count for {label} (1-6): '))
            if 1 <= val <= 6:
                return val
            print('  Value must be 1-6.')
        except ValueError:
            print('  Please enter a number between 1 and 6.')


# ── Helpers ───────────────────────────────────────────────────────────────────

def _place_and_go_home():
    """Return die safely to the table, then drive arm to home.
    Always lowers to the table before releasing so the die does not drop from height."""
    print('\nPlacing die and returning home...')
    bot.write_cartesian_position(HOME_POSE)                  # come down to a safe intermediate height
    bot.write_cartesian_position(PRE_PICK_POSE)              # descend to directly above table pick spot
    bot.write_cartesian_position(PICK_POSE)                  # lower to table surface
    bot.onRobot_gripper(GRIPPER_OPEN_MM, GRIPPER_FORCE_N)   # release die onto table
    time.sleep(1.5)                                          # wait for gripper to open
    bot.write_cartesian_position(PRE_PICK_POSE)              # lift clear of die
    bot.write_cartesian_position(HOME_POSE)                  # return to safe home


# ── Main sequence ─────────────────────────────────────────────────────────────

if __name__ == '__main__':
    try:
        target = int(input('Which pip do you want on top? (1-6): '))

        # ── Open gripper wide before any movement ─────────────────────────────
        print('\nOpening gripper...')
        bot.onRobot_gripper(GRIPPER_OPEN_MM, GRIPPER_FORCE_N)   # open wide while arm is clear
        time.sleep(1.5)                                          # wait for gripper to finish opening

        # ── Pick up die ───────────────────────────────────────────────────────
        print('\nPicking up die...')
        bot.write_cartesian_position(PRE_PICK_POSE)              # move to above die
        bot.write_cartesian_position(PICK_POSE)                  # lower to die on table
        bot.onRobot_gripper(GRIPPER_CLOSE_MM, GRIPPER_FORCE_N)  # grip die
        time.sleep(1.5)                                          # wait for gripper to finish closing
        bot.write_cartesian_position(PRE_PICK_POSE)              # lift back to travel height

        # ── First camera read ─────────────────────────────────────────────────
        print('\nMoving to camera...')
        bot.write_cartesian_position(CAMERA_POSE)               # carry die to camera
        time.sleep(0.8)                                          # let arm settle before taking photo
        face_1 = detect_top()                                    # read face visible at camera pose
        if face_1 is None:
            face_1 = prompt_pip('face 1 (current top)')
        print(f'  Camera reads: {face_1}')

        # ── Discovery rotation ────────────────────────────────────────────────
        # Rotate once so the second camera read reveals a different face.
        # Together the two readings uniquely identify the full die orientation.
        print('\nDiscovery rotation...')
        execute('roll_forward')                                  # place die, pick up in new orientation

        # ── Return to camera and read again ───────────────────────────────────
        print('\nReturning to camera...')
        bot.write_cartesian_position(CAMERA_POSE)               # bring die back to camera
        time.sleep(0.8)                                          # let arm settle before taking photo
        face_2 = detect_top()                                    # read face after the roll
        if face_2 is None:
            face_2 = prompt_pip('face 2 (after discovery roll)')
        print(f'  Camera reads: {face_2}')

        # ── Reconstruct full die orientation ──────────────────────────────────
        print('\nComputing orientation...')
        initial_state = from_visible_faces(top=face_1, front=face_2)   # state just before the roll
        if initial_state is None:
            print(f'  face_1={face_1} + face_2={face_2} is not a valid die orientation — aborting.')
            _place_and_go_home()
            sys.exit(1)
        state = initial_state.apply('roll_forward')             # apply the roll we already did
        print(f'  top:{state.top}  front:{state.front}  right:{state.right}  '
              f'bottom:{state.bottom}  back:{state.back}  left:{state.left}')

        # ── Rotate to target pip ──────────────────────────────────────────────
        # Plan the minimum rotations, then execute them straight through.
        # No camera check between steps — just rotate and place.
        print(f'\nRotating to pip {target}...')
        if state.top == target:
            print('  Already on top — no rotation needed.')
        else:
            rotate_to_pip(target, state, execute)   # BFS plan → execute each rotation in sequence
        print(f'  Pip {target} is on top.')

        # ── Place die and go home ─────────────────────────────────────────────
        # After the last rotation the arm is at _above(pick), ~60 mm above the
        # table surface.  Travel home first for a clean approach angle, then
        # lower to the table before releasing so the die does not drop.
        print('\nPlacing die...')
        bot.write_cartesian_position(HOME_POSE)                  # travel to safe intermediate height
        bot.write_cartesian_position(PRE_PICK_POSE)              # descend to directly above table pick spot
        bot.write_cartesian_position(PICK_POSE)                  # lower to table surface
        bot.onRobot_gripper(GRIPPER_OPEN_MM, GRIPPER_FORCE_N)   # release die onto table
        time.sleep(1.5)                                          # wait for gripper to open
        bot.write_cartesian_position(PRE_PICK_POSE)              # lift clear of die
        bot.write_cartesian_position(HOME_POSE)                  # return to safe home

    finally:
        mvsdk.CameraStop(h_cam)                                  # stop camera stream
        mvsdk.CameraUnInit(h_cam)                                # release camera hardware
