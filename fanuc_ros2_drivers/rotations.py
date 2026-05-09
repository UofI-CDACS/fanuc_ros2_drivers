#!/usr/bin/env python3
"""
rotations.py
============
All robot positions and rotation sequences for the die-inspection system.

Rotation strategy
-----------------
Each rotation is a list of steps the robot follows in order:

  1. 'move'  steps — move to a Cartesian pose while holding the die
                     (add as many as you need for collision-free travel)
  2. 'place' step  — lower to the position, release the die, lift clear
  3. 'pick'  step  — reposition above die, descend, grip die, lift back up

Because the pick position has a different WPR (wrist orientation) than the
place position, the die is in a new orientation after the pick.

To edit a rotation:  change the pose values or add/remove 'move' waypoints.
To add a new rotation:  copy a block, change the name and poses, add it to ROTATIONS.

Edit only this file to match your robot cell.
Import from any other script:
    from rotations import pick_up_die, present_to_camera, return_die, execute
"""

import os
import sys
import time
import threading

# ── Path setup ────────────────────────────────────────────────────────────────
# Add fanuc_ros2_drivers root to sys.path so src.msg_publishers... is importable
# regardless of which directory the script is run from.
_FANUC_ROOT = os.path.dirname(os.path.abspath(__file__))
if _FANUC_ROOT not in sys.path:
    sys.path.insert(0, _FANUC_ROOT)

# ── .env loading ──────────────────────────────────────────────────────────────
# Reads KEY=VALUE pairs from the workspace-root .env file into os.environ.
# Existing env vars take precedence (os.environ.setdefault never overwrites).
def _load_dotenv(path: str):
    if not os.path.isfile(path):
        return
    with open(path) as _f:
        for _line in _f:
            _line = _line.strip()
            if not _line or _line.startswith('#') or '=' not in _line:
                continue
            _key, _, _val = _line.partition('=')
            os.environ.setdefault(_key.strip(), _val.strip())

_load_dotenv(os.path.join(os.path.dirname(_FANUC_ROOT), '.env'))

from src.msg_publishers.dependencies.robot_controller import robot   # EtherNet/IP robot link
from src.msg_publishers.dependencies import FANUCethernetipDriver    # needed to refresh config bytes


# ── Robot connection ──────────────────────────────────────────────────────────

ROBOT_IP = os.environ.get('ROBOT_IP', '')   # set in .env or export ROBOT_IP=<ip>

HANDSHAKE_POLL_S    = 0.05   # seconds between R[1] polls while waiting for move to finish
HANDSHAKE_TIMEOUT_S = 30     # seconds before giving up and raising an error
POLL_INTERVAL_S     = 0.02   # background-thread poll rate for _wait_until
POSITION_TOL_MM     = 5.0    # Cartesian distance threshold for "position reached"
GRIPPER_TIMEOUT_S   = 10.0   # max seconds to wait for the gripper to complete
GRIPPER_SYNC_REG    = 3      # R[3]: OnRobot gripper command/sync register (TP clears it when done)


def _wait_until(condition_fn, timeout_s):
    """
    Block the calling thread until condition_fn() returns True.

    All polling runs in a daemon thread; this thread blocks on threading.Event.wait()
    so there is no time.sleep() in the calling thread.  Raises TimeoutError if
    condition_fn() has not returned True after timeout_s seconds.
    """
    done = threading.Event()
    exc  = [None]

    def _poll():
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            try:
                if condition_fn():
                    done.set()
                    return
            except Exception as e:
                exc[0] = e
                done.set()
                return
            time.sleep(POLL_INTERVAL_S)
        exc[0] = TimeoutError(f"Condition not met within {timeout_s}s — check robot/gripper state")
        done.set()

    threading.Thread(target=_poll, daemon=True).start()
    done.wait()
    if exc[0]:
        raise exc[0]


class _LazyRobot:
    """Defers the EtherNet/IP connection until the first robot call.
    All motion methods use R[1] handshake polling instead of is_moving() to
    avoid the race condition where fast moves finish before Python can detect them."""

    def __init__(self, ip):
        self._ip  = ip
        self._bot = None

    def _connect(self):
        if self._bot is None:
            self._bot = robot(self._ip)   # open connection on first use

    def _wait_for_move(self):
        """Two-phase R[1] handshake: wait for TP to acknowledge (R[1]→1), then complete (R[1]→0).

        Phase 1 catches the race condition where _bot.write_* returns before the TP
        has set R[1]=1.  Without it, Phase 2's while loop never enters and the next
        command fires while the arm is still moving.
        If Phase 1 times out it means R[1] rose and fell faster than one poll interval
        (instantaneous move or very fast TP) — skip straight to Phase 2 which will
        return immediately because R[1] is already 0.
        """
        # Phase 1: wait for TP to set R[1]=1 (acknowledges the command)
        deadline_start = time.time() + 1.0
        while self._bot.read_robot_start_register() == 0:
            if time.time() > deadline_start:
                break   # TP already cleared R[1] before we could catch it
            time.sleep(HANDSHAKE_POLL_S)
        # Phase 2: wait for TP to clear R[1]=0 (move complete)
        deadline = time.time() + HANDSHAKE_TIMEOUT_S
        while self._bot.read_robot_start_register() == 1:
            if time.time() > deadline:
                raise TimeoutError(
                    f"Robot motion did not complete within {HANDSHAKE_TIMEOUT_S}s — check TP program"
                )
            time.sleep(HANDSHAKE_POLL_S)

    def _refresh_cartesian_config(self):
        """Read live Turn1/Turn2/Turn3/Bitflip from the robot and patch CurCartesianPosList[8:12].

        Indices 0-1 (UTOOL/UFRAME) are irrelevant — writeCartesianPositionRegister always
        sends 0x0000 for those regardless.  Indices 8-11 are what the IK solver actually
        uses to choose a joint solution: Turn1/2/3 (how many full rotations each joint has
        made) and Bitflip (wrist-flip / elbow-up / shoulder-front flags).  robot_controller
        reads these once at __init__ and never refreshes them; after spin sequences change
        the wrist configuration the stale values send the IK solver to the wrong joint
        solution, which makes the arm swing to what looks like 'home'."""
        fresh = FANUCethernetipDriver.returnCartesianCurrentPostion(self._ip)
        self._bot.CurCartesianPosList[8]  = fresh[8]   # Turn1
        self._bot.CurCartesianPosList[9]  = fresh[9]   # Turn2
        self._bot.CurCartesianPosList[10] = fresh[10]  # Turn3
        self._bot.CurCartesianPosList[11] = fresh[11]  # Bitflip

    # ── Completion checks — used by _wait_until callbacks ────────────────────────

    def _position_reached(self, coords):
        """True when the robot's XYZ is within POSITION_TOL_MM of the target."""
        cur  = FANUCethernetipDriver.returnCartesianCurrentPostion(self._ip)
        dist = sum((cur[i + 2] - coords[i]) ** 2 for i in range(3)) ** 0.5
        return dist < POSITION_TOL_MM

    def _gripper_done(self):
        """True when the TP has cleared R[GRIPPER_SYNC_REG] back to 0 (gripper motion complete)."""
        return FANUCethernetipDriver.readR_Register(self._ip, GRIPPER_SYNC_REG) == 0

    # ── Motion methods — send command non-blocking, then wait on R[1] handshake ──

    def write_cartesian_position(self, coords, blocking=True):
        self._connect()
        self._refresh_cartesian_config()                             # always use live config bytes
        self._bot.write_cartesian_position(coords, blocking=False)  # send without broken is_moving check
        self._wait_for_move()                                        # R[1] handshake: TP signals move done
        _wait_until(lambda: self._position_reached(coords), HANDSHAKE_TIMEOUT_S)  # confirm arm arrived

    def write_joint_offset(self, joint, value, blocking=True):
        self._connect()
        self._bot.write_joint_offset(joint=joint, value=value, blocking=False)
        self._wait_for_move()

    def write_joint_pose(self, joints, blocking=True):
        self._connect()
        self._bot.write_joint_pose(joints, blocking=False)
        self._wait_for_move()

    def onRobot_gripper(self, width_mm, force_n):
        """Send gripper command and block until TP clears R[GRIPPER_SYNC_REG] (gripper done)."""
        self._connect()
        self._bot.onRobot_gripper(width_mm, force_n)
        _wait_until(self._gripper_done, GRIPPER_TIMEOUT_S)

    def __getattr__(self, name):   # all other calls (reads, etc.) go straight through
        self._connect()
        return getattr(self._bot, name)


bot = _LazyRobot(ROBOT_IP)       # connection is made only when the first move/gripper call happens


# ── OnRobot gripper settings ──────────────────────────────────────────────────

GRIPPER_OPEN_MM  = 120   # jaw width when open — must be wider than the die
GRIPPER_CLOSE_MM = 70    # jaw width when gripping — adjust until die is held without slipping
GRIPPER_FORCE_N  = 40    # grip force in newtons (0 – 120 N); increase if die slips


# ── Travel offset ─────────────────────────────────────────────────────────────

TRAVEL_Z = 60.0          # mm above any place/pick position to move before descending


# ════════════════════════════════════════════════════════════════════════════
#  ROBOT POSITIONS    [X, Y, Z, W, P, R]   — mm for XYZ, degrees for WPR
#
#  How to get these values:
#    1. Jog the robot to the position using the teach pendant
#    2. Read the Cartesian readout (POSN > USER on the pendant)
#    3. Paste the six numbers here
# ════════════════════════════════════════════════════════════════════════════

HOME_POSE  = [447.854,  -6.335,   282.0,   179.605,  1.089, 1.409]   # safe resting position
PICK_POSE  = [447.854,  -6.335,  -122.356, 179.605,  1.089, 1.409]   # die sitting on the table
CAMERA_POSE = [497.894, -938.334,  728.0,   149.150, 89.3,  58.349]  # position in front of camera

PRE_PICK_POSE = list(PICK_POSE)   # copy of PICK_POSE...
PRE_PICK_POSE[2] += 80.0         # ...raised 80 mm so arm clears the die on approach


# ════════════════════════════════════════════════════════════════════════════
#  ROTATION SEQUENCES
#
#  Each sequence is a list of steps run in order.
#
#  Step types:
#    ('move',         [x, y, z, w, p, r])  — move here while holding the die
#    ('joint_offset', joint, degrees)      — rotate one joint by ±N° in place (e.g. J6 for Z-spin)
#    ('place',        [x, y, z, w, p, r])  — auto: approach above, lower, release die, lift clear
#    ('pick',         [x, y, z, w, p, r])  — auto: approach above, lower, grip die, lift up
#
#  Notes:
#    - Add as many 'move' waypoints as needed for collision-free travel
#    - Sequences can chain multiple place+pick pairs (e.g. spin then roll)
#    - 'place' and 'pick' share XYZ but use different WPR —
#      the new wrist angle determines which face ends up on top after the pick
#    - TRAVEL_Z is automatically added above every 'place' and 'pick' position
#
#  Which base rotations are physically reachable on this robot:
#    roll_forward  — direct                         (Y-axis rotation)
#    roll_backward — NOT direct; use spin_cw×2 + roll_forward
#    roll_right    — NOT direct; use spin_cw  + roll_forward  (left  face → top)
#    roll_left     — NOT direct; use spin_ccw + roll_forward  (right face → top)
#    spin_cw       — direct 90° Z rotation
#    spin_ccw      — direct 90° Z rotation (opposite)
# ════════════════════════════════════════════════════════════════════════════

# ── Direct rotations (define these first — reused by compound rotations below) ──

# ── Uniform rotation flow ─────────────────────────────────────────────────────
#
#  Every rotation sequence follows the same pattern:
#    1. pick  from spin area (die resting there, neutral wrist)
#    2. rotate (carry, tip, reorient — whatever the rotation requires)
#    3. place  back at spin area (neutral wrist, consistent rest position)
#
#  This means:
#   - The die is always at the spin area between operations.
#   - No rotation can "negate" a previous one by resetting to neutral mid-sequence
#     while still holding the die — the gripper only goes neutral during the final
#     pick (after the die has already been released and re-gripped in new orientation).
#   - ROLL_RIGHT/ROLL_LEFT chain correctly: SPIN ends with place-at-spin,
#     ROLL_FORWARD starts with pick-from-spin.
# ─────────────────────────────────────────────────────────────────────────────

ROLL_FORWARD = [
    # Robot arrives already holding the die with camera-pose WPR (W=156.453, P=89.329, R=65.890)
    ('move',  [669.417, -260.224,  -26.122,  156.453,  89.329,  65.890]),   # approach roll area, maintain camera-pose WPR
    ('place', [669.417, -260.224, -142.992,  156.453,  89.329,  65.890]),   # set die at roll area — camera WPR tips die; die rolls to pick location
    ('move',  [669.417, -260.224,  -26.122,  156.453,  89.329,  65.890]),   # lift above roll area before moving sideways
    ('move',  [703.334, -365.535,  43.832,  -177.805,  -1.434,  87.403]),   # move above rolled-die location, rotate wrist to neutral
    ('pick',  [703.334, -365.535, -60.608,  -177.805,  -1.434,  87.403]),   # pick with neutral wrist — die now in rolled orientation
    ('move',  [703.334, -365.535,  33.000,  -177.805,  -1.434,  87.403]),   # lift above pick location before moving
    ('move',  [669.417, -260.224,  -26.122,  156.453,  89.329,  65.890]),   # return to roll area approach height (camera WPR)
    ('place', [669.417, -260.224, -142.992,  156.453,  89.329,  65.890]),   # set die back at roll area
]

# Spin sequence — each step explained:
#   1. pick  — grip die from spin area (neutral wrist)
#   2. move  — reorient wrist ±90° while holding die — die physically spins with it
#   3. place — lower, release die, lift clear  (wrist still at spun angle)
#   4. move  — intermediate position (arm clears die, wrist returns to neutral)
#   5. pick  — descend and grip die with neutral wrist — die is now spun relative to gripper
#   6. place — set die at spin area (consistent rest position for next operation)
# For step 3 WPR must match step 2 so the robot doesn't re-spin the die during descent.

SPIN_CCW = [
    ('pick',  [667.497, -348.564,  -77.243, -177.384,  -1.816,  89.122]),   # pick die from spin area
    ('move',  [659.287, -359.247,   54.268,  178.150,   2.760, -177.932]),  # rotate wrist CCW while holding die (R: 89 → -178) — die spins with gripper
    ('place', [667.497, -348.564,  -77.243, -179.744,   -.037, -171.821]),  # place at CCW-spun WPR (R≈-172, matches step above)
    ('move',  [659.287, -359.247,   54.268, -177.384,  -1.816,  89.122]),   # return to neutral R=89 via -99° wrapping path (avoids joint limit)
    ('pick',  [667.497, -348.564,  -77.243, -177.384,  -1.816,  89.122]),   # pick with neutral wrist — die is now spun CCW
    ('place', [667.497, -348.564,  -77.243, -177.384,  -1.816,  89.122]),   # set die at spin area
]

SPIN_CW = [
    ('pick',  [667.497, -348.564,  -77.243, -177.384,  -1.816,  89.122]),   # pick die from spin area
    ('move',  [659.287, -359.247,   54.268, -178.054,   2.526,   2.029]),   # rotate wrist CW while holding die (R: 89 → 2) — die spins with gripper
    ('place', [667.497, -348.564,  -77.243, -179.744,   -.037,   3.775]),   # place at CW-spun WPR (R≈2, matches step above)
    ('move',  [659.287, -359.247,   54.268, -178.054,   2.526,   2.029]),   # lift arm clear of die
    ('pick',  [667.497, -348.564,  -77.243, -177.384,  -1.816,  89.122]),   # pick with neutral wrist — die is now spun CW
    ('place', [667.497, -348.564,  -77.243, -177.384,  -1.816,  89.122]),   # set die at spin area
]

# ── Compound rotations ────────────────────────────────────────────────────────
#
#  ROLL_RIGHT: SPIN_CCW + ROLL_FORWARD
#    SPIN_CCW ends with place-at-spin → ROLL_FORWARD starts with pick-from-spin ✓
#  ROLL_LEFT:  SPIN_CW  + ROLL_FORWARD  (same chain logic)

ROLL_BACKWARD = (
    # ── spin 180° around Z, then roll forward ────────────────────────────────
    SPIN_CCW + SPIN_CCW + ROLL_FORWARD   # two CCW spins = 180° Z rotation, then roll
)

ROLL_RIGHT = SPIN_CCW + ROLL_FORWARD   # SPIN_CCW brings left  face to front → roll → left  face on top
ROLL_LEFT  = SPIN_CW  + ROLL_FORWARD   # SPIN_CW  brings right face to front → roll → right face on top

# ── Camera presentation helpers ───────────────────────────────────────────────
#  Used by dice_finder.py to bring the die to CAMERA_POSE for reading/verification.
#  Always call 'return_to_spin' after camera_fn() to put the die back for next rotation.

PICK_FROM_ROLL_TO_CAMERA = [
    ('pick', [669.417, -260.224, -142.992,  156.453,  89.329,  65.890]),   # grip die from roll area (camera WPR)
    ('move', CAMERA_POSE),                                                   # carry to camera
]

PICK_FROM_SPIN_TO_CAMERA = [
    ('pick', [667.497, -348.564,  -77.243, -177.384,  -1.816,  89.122]),   # grip die from spin area (neutral WPR)
    ('move', CAMERA_POSE),                                                   # carry to camera
]

RETURN_TO_SPIN = [
    ('place', [667.497, -348.564,  -77.243, -177.384,  -1.816,  89.122]),  # set die at spin area
]


# ── Rotation dispatcher ───────────────────────────────────────────────────────
#  Maps planner names to the sequences above.
#  Never rename the keys on the left — only swap the values on the right.

ROTATIONS = {
    'roll_forward':          ROLL_FORWARD,
    'roll_backward':         ROLL_BACKWARD,
    'roll_right':            ROLL_RIGHT,
    'roll_left':             ROLL_LEFT,
    'spin_cw':               SPIN_CW,
    'spin_ccw':              SPIN_CCW,
    'pick_roll_to_camera':   PICK_FROM_ROLL_TO_CAMERA,
    'pick_spin_to_camera':   PICK_FROM_SPIN_TO_CAMERA,
    'return_to_spin':        RETURN_TO_SPIN,
}


# ── Step executor ─────────────────────────────────────────────────────────────

def _above(pose):
    """Return a copy of pose with Z raised by TRAVEL_Z — used for safe approach and depart."""
    raised = list(pose)        # copy so the original list is never modified
    raised[2] += TRAVEL_Z     # raise Z only; X, Y, W, P, R stay the same
    return raised


def execute(rotation_name):
    """
    Run through every step in the named rotation's sequence in order.

    'move'         — move to a Cartesian waypoint while holding the die
    'joint_offset' — rotate one joint by ±N degrees (used for J6 Z-spins)
    'place'        — approach from above, lower, release die, lift clear
    'pick'         — approach from above, lower, grip die, lift up
    """
    for step in ROTATIONS[rotation_name]:           # loop through each step in the list
        kind = step[0]                              # first element is always the step type

        if kind == 'move':
            bot.write_cartesian_position(step[1])           # move to Cartesian waypoint holding die

        elif kind == 'joint_offset':
            _, joint, degrees = step                        # unpack joint number and offset amount
            bot.write_joint_offset(joint=joint, value=degrees)   # rotate that joint by ±N degrees

        elif kind == 'place':
            bot.write_cartesian_position(_above(step[1]))            # move to safe height above place position
            bot.write_cartesian_position(step[1])                    # lower straight down to place position
            bot.onRobot_gripper(GRIPPER_OPEN_MM, GRIPPER_FORCE_N)   # open gripper — blocks until R[3] cleared
            bot.write_cartesian_position(_above(step[1]))            # lift gripper clear of the die

        elif kind == 'pick':
            bot.write_cartesian_position(_above(step[1]))            # move to safe height above pick position
            bot.write_cartesian_position(step[1])                    # lower to grip position
            bot.onRobot_gripper(GRIPPER_CLOSE_MM, GRIPPER_FORCE_N)  # close gripper — blocks until R[3] cleared
            bot.write_cartesian_position(_above(step[1]))            # lift die to travel height


# ════════════════════════════════════════════════════════════════════════════
#  PICK, PRESENT, AND RETURN HELPERS
# ════════════════════════════════════════════════════════════════════════════

def open_gripper():
    """Open the gripper to its full width — call once at startup before any movement."""
    bot.onRobot_gripper(GRIPPER_OPEN_MM, GRIPPER_FORCE_N)


def pick_up_die():
    """Approach above the die, descend, grip it, and lift to travel height."""
    bot.onRobot_gripper(GRIPPER_OPEN_MM, GRIPPER_FORCE_N)   # open gripper FIRST while arm is clear
    bot.write_cartesian_position(PRE_PICK_POSE)              # move to above die (gripper already open)
    bot.write_cartesian_position(PICK_POSE)                  # lower to die on table
    bot.onRobot_gripper(GRIPPER_CLOSE_MM, GRIPPER_FORCE_N)  # grip die
    bot.write_cartesian_position(PRE_PICK_POSE)              # lift back to travel height


def present_to_camera():
    """Carry the held die to the camera inspection position."""
    bot.write_cartesian_position(CAMERA_POSE)                # move die in front of camera


def return_die():
    """Lower the die back to the table, release it, and return to home."""
    bot.write_cartesian_position(PRE_PICK_POSE)              # travel to above drop spot
    bot.write_cartesian_position(PICK_POSE)                  # lower to table
    bot.onRobot_gripper(GRIPPER_OPEN_MM, GRIPPER_FORCE_N)   # release die
    bot.write_cartesian_position(PRE_PICK_POSE)              # lift clear
    bot.write_cartesian_position(HOME_POSE)                  # go to safe home position
