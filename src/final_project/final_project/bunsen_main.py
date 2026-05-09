#!/usr/bin/env python3
"""
bunsen_main.py — Bunsen (Robot 2) production game controller.

Prerequisites — run in separate terminals before this:
  Terminal 1: ros2 launch launch/start.launch.py robot_name:=Bunsen robot_ip:=10.8.4.6
  Terminal 2: python3 fanuc_ros2_drivers/src/final_project/final_project/modbus_server.py

Bunsen handles EVEN pip targets: 2, 4, 6.
Beaker handles ODD pip targets:  1, 3, 5.

Flow per round:
  Wait  →  GrabDie (rear conveyor)  →  PipCount  →  PlaceDie (front conveyor, back 9.9 s)
  └── if pip == 6: Finish (place die in front of robot, exit)

Env vars (all optional):
  BUNSEN_NAME       robot namespace    (default: Bunsen)
  MODBUS_HOST       Modbus server IP   (default: 127.0.0.1)
  MODBUS_PORT       Modbus server port (default: 5020)
  CAMERA_IP         GigE camera IP     (default: 10.8.4.19)
"""

import os
import signal
import sys
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)

# Auto-source ROS2 environment if not already done.
if 'ROS_DISTRO' not in os.environ:
    _ws = os.path.normpath(os.path.join(_HERE, '..', '..', '..', 'install', 'setup.bash'))
    _cmd = 'source /opt/ros/jazzy/setup.bash'
    if os.path.exists(_ws):
        _cmd += f' && source {_ws}'
    _cmd += f' && exec python3 {os.path.abspath(__file__)}'
    os.execvp('bash', ['bash', '-c', _cmd])

from modbus_server import (
    REG_BUNSEN_STATE, REG_PIP_PROGRESS, REG_CONV_CMD, REG_RETRIES, REG_BEAKER_STATE,
    COIL_BUNSEN_READY, COIL_CAMERA_CLIENT, COIL_BEAKER_READY,
    STATE_SETUP, STATE_WAIT, STATE_GRAB_DIE, STATE_PIP_COUNT,
    STATE_PLACE_DIE, STATE_FINISH, STATE_FAULT,
    STATE_NAMES,
    CONV_IDLE, CONV_BEAKER_WANTS_SEND, CONV_REAR_RUNNING, CONV_DIE_ON_REAR,
    CONV_BUNSEN_HAS_DIE, CONV_BUNSEN_WANTS_SEND, CONV_FRONT_RUNNING,
    CONV_DIE_ON_FRONT, CONV_BEAKER_HAS_DIE,
)

# ---------------------------------------------------------------------------
# Tuning constants — adjust before each run
# ---------------------------------------------------------------------------
GRIPPER_OPEN_WIDTH  = 120   # mm  (OnRobot)
GRIPPER_OPEN_FORCE  = 30    # N
GRIPPER_CLOSE_WIDTH = 70    # mm
GRIPPER_CLOSE_FORCE = 50    # N

REAR_CONV_TRAVEL_SECS   = 5.0   # seconds after DIE_ON_REAR before die reaches pickup spot (unused — Beaker pre-travels)
CAMERA_SETTLE_SECS      = 0.4   # pause after moving before capturing
GRIPPER_SETTLE_SECS     = 1.5   # pause after gripper open/close before next move
POLL_INTERVAL           = 0.2   # Modbus polling rate (seconds)
CONV_TIMEOUT            = 60.0  # max wait for a conveyor handshake step
MAX_PIP_RETRIES         = 20    # give up pip search after this many put-down + re-picks

# ---------------------------------------------------------------------------
# Robot poses — CALIBRATE all zeroed entries before first run
# ---------------------------------------------------------------------------
HOME_JOINTS = dict(joint1=0.0, joint2=0.0, joint3=0.0, joint4=0.0, joint5=-90.0, joint6=0.0)

# Where Bunsen parks while waiting for Beaker to deliver die
CONVEYOR_WAIT_POSE = dict(joint1=-77.477, joint2=-12.035, joint3=-2.185, joint4=-1.566, joint5=-88.860, joint6=-12.605)

# Rear conveyor — Beaker delivers die here
REAR_CONV_ABOVE  = dict(joint1=-95.107, joint2=12.370, joint3=-10.552, joint4=-1.190, joint5=-80.923, joint6=5.182)
REAR_CONV_PICKUP = dict(joint1=-95.108, joint2=18.648, joint3=-32.133, joint4=-1.366, joint5=-59.347, joint6=5.692)

# Central safe hold location — robot moves here after picking die
SAFE_HOLD_POSE = dict(joint1=0.0, joint2=0.0, joint3=0.0, joint4=0.0, joint5=-90.0, joint6=0.0)    # CALIBRATE

# Camera positions — joint angles at which Bunsen presents die to overhead camera
CAM_POSE_1 = dict(joint1=-69.532, joint2=23.453, joint3=9.053, joint4=4.350, joint5=-11.355, joint6=-4.712)
CAM_POSE_2 = dict(joint1=-67.702, joint2=41.934, joint3=0.924, joint4=5.952, joint5=70.592, joint6=-4.709)

# Table spot for die re-orientation — put die down here between pip-count attempts
TABLE_PLACE_ABOVE = dict(joint1=-42.010, joint2=20.282, joint3=.918, joint4=-1.881, joint5=-90.859, joint6=-48.146)
TABLE_PLACE_DOWN  = dict(joint1=-42.015, joint2=31.966, joint3=-39.884, joint4=-2.453, joint5=-50.084, joint6=-46.540)

# Re-grip position — approach die on table from a different angle to expose other faces
TABLE_REPOS_1       = dict(joint1=-42.142, joint2=31.965, joint3=-39.884, joint4=-3.262, joint5=-49.758, joint6=-133.223)
TABLE_REPOS_1_ABOVE = dict(joint1=0.0, joint2=0.0, joint3=0.0, joint4=0.0, joint5=0.0, joint6=0.0)    # CALIBRATE — lift after REPOS_1 grip before rotating away
TABLE_REPOS_2 = dict(joint1=-28.210, joint2=69.225, joint3=-47.417, joint4=32.772, joint5=-119.359, joint6=-156.490)
TABLE_REPOS_3 = dict(joint1=-28.898, joint2=61.730, joint3=-43.216, joint4=35.709, joint5=-121.764, joint6=-145.216)
TABLE_REPOS_PICK = dict(joint1=-42.015, joint2=31.304, joint3=-38.739, joint4=-2.413, joint5=-51.228, joint6=39.646)

# Front conveyor — Bunsen places die here to send back to Beaker
FRONT_CONV_ABOVE  = dict(joint1=-62.667, joint2=13.088, joint3=-25.034, joint4=-1.655, joint5=-65.618, joint6=-26.181)
FRONT_CONV_PLACE  = dict(joint1=-64.747, joint2=15.950, joint3=-33.644, joint4=-1.767, joint5=-57.065, joint6=-23.823)

# Final placement — pip 6, placed in front of Bunsen for display
FINAL_PLACE_ABOVE = dict(joint1=18.885, joint2=-3.830,  joint3=-27.348, joint4=0.106,  joint5=-62.685,  joint6=-18.934)
FINAL_PLACE_DOWN  = dict(joint1=18.885, joint2=16.454,  joint3=-64.706, joint4=0.221,  joint5=-25.327,  joint6=-19.085)

# Chirality reposition poses (used by _face_rotate) — same XY as FINAL_PLACE_DOWN, only joint6 differs
# FACE_ROTATE_MID: wrist CW 90° at table level (joint6: -19°→-106°). left face comes to front.
FACE_ROTATE_MID   = dict(joint1=18.061, joint2=16.056,  joint3=-65.049, joint4=0.217,  joint5=-24.269,  joint6=-106.040)
# FACE_ROTATE: arm forward and angled down — gripper approaches die from its front face to tip it.
FACE_ROTATE       = dict(joint1=9.743,  joint2=64.697,  joint3=-61.031, joint4=-10.561, joint5=-116.591, joint6=-5.378)

# ---------------------------------------------------------------------------
# Standard western die chirality
# (top_face, front_face) → right_face  — all 24 valid orientations
# Opposite faces always sum to 7: 1↔6, 2↔5, 3↔4
# ---------------------------------------------------------------------------
_DIE_CHIRALITY = {
    (1,2):3,(1,3):5,(1,5):4,(1,4):2,
    (2,6):3,(2,3):1,(2,1):4,(2,4):6,
    (3,2):6,(3,6):5,(3,5):1,(3,1):2,
    (4,2):1,(4,1):5,(4,5):6,(4,6):2,
    (5,1):3,(5,3):6,(5,6):4,(5,4):1,
    (6,5):3,(6,3):2,(6,2):4,(6,4):5,
}


def _chirality_j6_steps(face_pose1: int, face_pose2: int, target: int):
    """
    Given face_pose1 (up at CAM_POSE_1) and face_pose2 (up at CAM_POSE_2, which is
    CAM_POSE_1 with joint6 offset by J6_DELTA), return how many J6_DELTA steps to
    apply to the pickup joint6 to put target face up at CAM_POSE_1.

    Returns 0–3, or None if target is on the front/back axis (unreachable by j6 alone).
    """
    roll = [face_pose1, face_pose2, 7 - face_pose1, 7 - face_pose2]
    return roll.index(target) if target in roll else None


def _all_faces_from_two(top: int, front: int) -> set:
    """
    Given the top face and one visible side face, use the chirality table to
    derive all six face values and their positions: top, bottom, front, back,
    right, left.  Returns a set of all six pip values (should always be {1..6}).
    """
    right   = _DIE_CHIRALITY.get((top, front))
    if right is None:
        return set()
    bottom  = 7 - top
    back    = 7 - front
    left    = 7 - right
    return {top, bottom, front, back, right, left}


# ===========================================================================
# BunsenMain — ROS2 node, full state machine
# ===========================================================================

def _make_node():
    import rclpy
    from rclpy.node import Node
    from rclpy.action import ActionClient
    from fanuc_interfaces.action import CartPose, JointPose, OnRobotGripper, Conveyor
    from fanuc_interfaces.msg import CurGripper
    from pymodbus.client import ModbusTcpClient
    from camera import Camera
    from pip_counter import count_pips

    class BunsenMain(Node):

        def __init__(self):
            ns      = os.environ.get('BUNSEN_NAME',  'Bunsen')
            mb_host = os.environ.get('MODBUS_HOST',  '127.0.0.1')
            mb_port = int(os.environ.get('MODBUS_PORT', '5020'))
            self._cam_ip = os.environ.get('CAMERA_IP', '10.8.4.19')

            Node.__init__(self, 'bunsen_main')

            self._cart    = ActionClient(self, CartPose,       f'/{ns}/cartesian_pose')
            self._joint   = ActionClient(self, JointPose,      f'/{ns}/joint_pose')
            self._gripper = ActionClient(self, OnRobotGripper, f'/{ns}/onrobot_gripper')
            self._conv    = ActionClient(self, Conveyor,       f'/{ns}/conveyor')

            self.mb = ModbusTcpClient(mb_host, port=mb_port)
            if not self.mb.connect():
                raise RuntimeError(f'Cannot connect to Modbus at {mb_host}:{mb_port}')
            self.get_logger().info(f'Modbus connected to {mb_host}:{mb_port}')

            self._camera = None
            self._bunsen_retries = 0
            self._step_mode = False

        # ── Modbus helpers ────────────────────────────────────────────────────

        def _mb_read(self, addr):
            return self.mb.read_holding_registers(addr, 1).registers[0]

        def _mb_write(self, addr, val):
            self.mb.write_register(addr, val)

        def _mb_read_coil(self, addr):
            return bool(self.mb.read_coils(addr, 1).bits[0])

        def _mb_write_coil(self, addr, val):
            self.mb.write_coil(addr, bool(val))

        def _set_state(self, state):
            self._mb_write(REG_BUNSEN_STATE, state)
            self.get_logger().info(f'→ {STATE_NAMES.get(state, state)}')

        def _wait_mb(self, reg_or_coil, target, is_coil=False, timeout=CONV_TIMEOUT):
            deadline = time.time() + timeout
            while time.time() < deadline:
                val = self._mb_read_coil(reg_or_coil) if is_coil \
                      else self._mb_read(reg_or_coil)
                if val == target:
                    return True
                time.sleep(POLL_INTERVAL)
            return False

        # ── Action helpers ────────────────────────────────────────────────────

        def _send_cart(self, **kw) -> bool:
            if not self._cart.wait_for_server(timeout_sec=5.0):
                self.get_logger().error('CartPose server timeout')
                return False
            goal = CartPose.Goal()
            for k, v in kw.items():
                setattr(goal, k, float(v))
            fut = self._cart.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, fut)
            gh = fut.result()
            if not gh.accepted: return False
            res = gh.get_result_async()
            rclpy.spin_until_future_complete(self, res)
            return res.result().result.success

        def _send_joint(self, **kw) -> bool:
            if not self._joint.wait_for_server(timeout_sec=5.0):
                self.get_logger().error('JointPose server timeout')
                return False
            goal = JointPose.Goal()
            for k, v in kw.items():
                setattr(goal, k, float(v))
            fut = self._joint.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, fut)
            gh = fut.result()
            if not gh.accepted: return False
            res = gh.get_result_async()
            rclpy.spin_until_future_complete(self, res)
            return res.result().result.success

        def _open_gripper(self) -> bool:
            if not self._gripper.wait_for_server(timeout_sec=5.0):
                self.get_logger().error('Gripper server timeout')
                return False
            goal = OnRobotGripper.Goal()
            goal.width = GRIPPER_OPEN_WIDTH
            goal.force = GRIPPER_OPEN_FORCE
            fut = self._gripper.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, fut)
            gh = fut.result()
            if not gh.accepted: return False
            res = gh.get_result_async()
            rclpy.spin_until_future_complete(self, res)
            return res.result().result.success

        def _close_gripper(self) -> bool:
            if not self._gripper.wait_for_server(timeout_sec=5.0):
                self.get_logger().error('Gripper server timeout')
                return False
            goal = OnRobotGripper.Goal()
            goal.width = GRIPPER_CLOSE_WIDTH
            goal.force = GRIPPER_CLOSE_FORCE
            fut = self._gripper.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, fut)
            gh = fut.result()
            if not gh.accepted: return False
            res = gh.get_result_async()
            rclpy.spin_until_future_complete(self, res)
            return res.result().result.success

        def _move(self, retries: int = 3, **kw) -> bool:
            """_send_joint with automatic retry on rejection."""
            for attempt in range(retries):
                if attempt > 0:
                    self.get_logger().warn(f'Joint move rejected — retry {attempt}/{retries - 1}')
                    time.sleep(GRIPPER_SETTLE_SECS)
                if self._send_joint(**kw):
                    return True
            self.get_logger().error('Joint move failed after all retries')
            return False

        def _run_conveyor(self, command: str) -> bool:
            if not self._conv.wait_for_server(timeout_sec=5.0):
                self.get_logger().error('Conveyor server timeout')
                return False
            goal = Conveyor.Goal()
            goal.command = command
            fut = self._conv.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, fut)
            gh = fut.result()
            if not gh.accepted: return False
            res = gh.get_result_async()
            rclpy.spin_until_future_complete(self, res)
            return res.result().result.success

        def _gripper_is_open(self) -> bool:
            """Read current gripper state from topic; returns True if open."""
            ns = os.environ.get('BUNSEN_NAME', 'Bunsen')
            result = [None]
            def _cb(msg):
                result[0] = msg.open
            sub = self.create_subscription(CurGripper, f'/{ns}/cur_gripper', _cb, 1)
            deadline = time.time() + 2.0
            while result[0] is None and time.time() < deadline:
                rclpy.spin_once(self, timeout_sec=0.1)
            self.destroy_subscription(sub)
            return result[0] if result[0] is not None else True  # assume open on timeout

        def _ask_pip_manual(self, position: str) -> int:
            """Prompt the operator to type the pip count on the up-facing die face."""
            while True:
                try:
                    raw = input(f'  [Manual] {position} — pips on UP face (1-6): ').strip()
                    val = int(raw)
                    if 1 <= val <= 6:
                        return val
                except (ValueError, EOFError):
                    pass
                print('  Enter a number 1-6.')

        def _capture_pip(self, position: str = 'position') -> int:
            """Grab a frame and count pips; retries on 0, falls back to manual."""
            if self._camera is None or self._camera.hCamera is None:
                return self._ask_pip_manual(position)
            for attempt in range(3):
                try:
                    frame = self._camera.getFrame()
                    pips = count_pips(frame)
                    if pips > 0:
                        self.get_logger().info(f'Camera ({position}): {pips} pip(s)')
                        return pips
                    self.get_logger().warn(
                        f'Camera ({position}): 0 pips detected (attempt {attempt + 1}/3)')
                    time.sleep(0.3)
                except Exception as e:
                    self.get_logger().warn(f'Camera error at {position}: {e}')
                    break
            self.get_logger().warn(f'Camera returned 0 at {position} after retries — manual input')
            return self._ask_pip_manual(position)

        # ====================================================================
        # Chirality-guided die reposition
        # ====================================================================

        def _face_rotate(self, where: str):
            """Minimum-move reposition bringing target face to a camera-visible position.

            Call with robot at FINAL_PLACE_ABOVE holding die.
            Returns with robot at FINAL_PLACE_ABOVE, gripper open, die on table.

            FRM: lower to FINAL_PLACE_DOWN while gripping → rotate wrist CW 90°
                 (FACE_ROTATE_MID, joint6: -19→-106°) at table level → release.
                 Effect: left face → front face.

            FR:  move to FACE_ROTATE (arm forward/low, gripper at die's front face) →
                 close gripper → drag arm to FINAL_PLACE_DOWN (tips die: front→top,
                 back→bottom) → release.
                 Gripper closes ONLY here, never in FRM or place.

            Sequences per target face:
              left  → FRM            (1 step)  left→front          visible at VIEW 1
              back  → FR             (1 step)  back→bottom         visible at VIEW 2
              right → FRM + FR       (2 steps) right→back→bottom   visible at VIEW 2
              top   → FR + FR        (2 steps) top→back→bottom     visible at VIEW 2
            """
            def _above():
                # lift arm to safe height above final place, arm pointing straight down
                self._move(**FINAL_PLACE_ABOVE)
                time.sleep(GRIPPER_SETTLE_SECS)

            def _frm():
                # lower onto die while gripping, rotate wrist CW 90° at table level, release
                print('  [face_rotate] FINAL_PLACE_DOWN')
                self._move(**FINAL_PLACE_DOWN)    # lower arm straight down onto die, still gripping
                time.sleep(GRIPPER_SETTLE_SECS)
                print('  [face_rotate] FACE_ROTATE_MID — wrist CW 90° (joint6: -19→-106°)')
                self._move(**FACE_ROTATE_MID)     # same XYZ as FPD, just joint6 rotated — die rotates on table
                time.sleep(GRIPPER_SETTLE_SECS)
                print('  [face_rotate] open gripper')
                self._open_gripper()              # release die at table level in rotated orientation
                time.sleep(GRIPPER_SETTLE_SECS)
                _above()                          # lift arm up; die stays on table

            def _place():
                # place held die on table without changing its orientation (prep for _fr)
                print('  [face_rotate] FINAL_PLACE_DOWN (place)')
                self._move(**FINAL_PLACE_DOWN)    # lower arm straight down to table level
                time.sleep(GRIPPER_SETTLE_SECS)
                print('  [face_rotate] open gripper')
                self._open_gripper()              # release die — orientation unchanged
                time.sleep(GRIPPER_SETTLE_SECS)
                _above()                          # retract straight up

            def _fr():
                # grip die from front angle, drag arm to vertical — tips die (back→bottom)
                print('  [face_rotate] FACE_ROTATE — arm forward/low, gripper at die front face')
                self._move(**FACE_ROTATE)         # arm swings forward and angled down; gripper above die's front face
                time.sleep(GRIPPER_SETTLE_SECS)
                print('  [face_rotate] close gripper')
                self._close_gripper()             # grip die from front-face angle while on table
                time.sleep(GRIPPER_SETTLE_SECS)
                print('  [face_rotate] FINAL_PLACE_DOWN (tip die)')
                self._move(**FINAL_PLACE_DOWN)    # drag arm back to vertical — levers die forward (front→top, back→bottom)
                time.sleep(GRIPPER_SETTLE_SECS)
                print('  [face_rotate] open gripper')
                self._open_gripper()              # release die at table level in tipped orientation
                time.sleep(GRIPPER_SETTLE_SECS)
                _above()                          # lift arm up

            print(f'  [face_rotate] target on {where}')
            if where == 'left':
                _frm()                  # left → front (VIEW 1 visible)
            elif where == 'back':
                _place(); _fr()         # back → bottom (VIEW 2 visible)
            elif where == 'right':
                _frm(); _fr()           # right → back → bottom (VIEW 2 visible)
            elif where == 'top':
                _place(); _fr(); _fr()  # top → back → bottom (VIEW 2 visible)
            else:
                self.get_logger().warn(f'face_rotate: unknown face "{where}" — FRM+FR fallback')
                _frm(); _fr()

        # ====================================================================
        # State: SETUP
        # ====================================================================

        def _state_setup(self):
            print('\n' + '=' * 55)
            print('  BUNSEN starting up — clearing Modbus registers...')
            print('=' * 55)

            # Clear everything
            self._mb_write(REG_BUNSEN_STATE, STATE_SETUP)
            self._mb_write(REG_BEAKER_STATE, 0)
            self._mb_write(REG_PIP_PROGRESS, 1)      # pip 1 = Beaker's first target
            self._mb_write(REG_CONV_CMD,     CONV_IDLE)
            self._mb_write(REG_RETRIES,      0)
            self._mb_write_coil(COIL_BUNSEN_READY,  False)
            self._mb_write_coil(COIL_CAMERA_CLIENT, False)
            self._mb_write_coil(COIL_BEAKER_READY,  False)

            print('  Waiting for ROS2 action servers...')
            self._cart.wait_for_server()
            self._joint.wait_for_server()
            self._gripper.wait_for_server()
            self._conv.wait_for_server()
            print('  Action servers ready.')

            print(f'  Opening camera at {self._cam_ip}...')
            self._camera = Camera(camera_ip=self._cam_ip)
            if self._camera.hCamera is not None:
                print('  Camera ready.')
            else:
                print('  [WARN] Camera not found — captures will return -1.')

            self._run_conveyor('stop')

            if not self._gripper_is_open():
                print('  [Setup] Gripper is closed — placing held die at final location...')
                self._send_joint(**FINAL_PLACE_ABOVE)
                self._send_joint(**FINAL_PLACE_DOWN)
                self._open_gripper()
                self._send_joint(**FINAL_PLACE_ABOVE)
            else:
                self._open_gripper()

            self._send_joint(**HOME_JOINTS)
            self._set_state(STATE_WAIT)
            print('  Setup complete. pip_progress=1 (Beaker targets pip 1 first).\n')

        # ====================================================================
        # State: WAIT
        # ====================================================================

        def _state_wait(self):
            self._set_state(STATE_WAIT)
            self._mb_write_coil(COIL_BUNSEN_READY, False)

            pip = self._mb_read(REG_PIP_PROGRESS)
            print(f'  [Wait] Moved to conveyor wait pose.  '
                  f'Expecting Beaker to deliver pip {pip + 1 if pip % 2 == 1 else pip}...')

            self._send_joint(**CONVEYOR_WAIT_POSE)

            print('  [Wait] Polling for CONV_CMD=DIE_ON_REAR (Beaker placed die on belt)...')
            deadline = time.time() + (5.0 if self._step_mode else float('inf'))
            while True:
                conv = self._mb_read(REG_CONV_CMD)
                if conv == CONV_DIE_ON_REAR:
                    break
                if self._step_mode and time.time() >= deadline:
                    print('  [Wait] Step mode: 5 s elapsed — simulating Beaker placed die.')
                    self._mb_write(REG_CONV_CMD, CONV_DIE_ON_REAR)
                    break
                time.sleep(POLL_INTERVAL)

            print('  [Wait] Beaker ready — moving to GrabDie.')
            self._set_state(STATE_GRAB_DIE)

        # ====================================================================
        # State: GRAB_DIE
        # ====================================================================

        def _state_grab_die(self):
            self._set_state(STATE_GRAB_DIE)
            print('  [GrabDie] Moving to rear conveyor...')

            self._open_gripper()
            self._send_joint(**REAR_CONV_ABOVE)
            self._send_joint(**REAR_CONV_PICKUP)
            self._close_gripper()
            time.sleep(2.0)
            self._send_joint(**REAR_CONV_ABOVE)

            # Acknowledge pickup — Beaker resets to IDLE on seeing this
            self._mb_write(REG_CONV_CMD, CONV_BUNSEN_HAS_DIE)
            print('  [GrabDie] Die grabbed — signalled BUNSEN_HAS_DIE.')

            self._send_joint(**CONVEYOR_WAIT_POSE)
            self._set_state(STATE_PIP_COUNT)

        # ====================================================================
        # State: PIP_COUNT
        # ====================================================================

        def _state_pip_count(self):
            self._set_state(STATE_PIP_COUNT)
            target = self._mb_read(REG_PIP_PROGRESS)
            print(f'  [PipCount] Target pip: {target}')

            self._mb_write_coil(COIL_CAMERA_CLIENT, True)

            def _two_views():
                # VIEW 1 (CAM_POSE_1): front face visible to camera
                self._move(**CAM_POSE_1)
                time.sleep(CAMERA_SETTLE_SECS)
                v1 = self._capture_pip('VIEW 1')
                print(f'  [PipCount] VIEW 1 (front) → {v1}')

                # VIEW 2 (CAM_POSE_2): bottom face visible to camera → top = 7 - v2
                self._move(**CAM_POSE_2)
                time.sleep(CAMERA_SETTLE_SECS)
                v2 = self._capture_pip('VIEW 2')
                print(f'  [PipCount] VIEW 2 (bottom) → {v2}  (top: {7 - v2})')

                self._move(**CAM_POSE_1)
                return v1, v2

            def _place_die():
                self._move(**TABLE_PLACE_ABOVE)
                time.sleep(GRIPPER_SETTLE_SECS)
                self._move(**TABLE_PLACE_DOWN)
                time.sleep(GRIPPER_SETTLE_SECS)
                self._open_gripper()
                time.sleep(GRIPPER_SETTLE_SECS)
                self._move(**TABLE_PLACE_ABOVE)
                time.sleep(GRIPPER_SETTLE_SECS)

            def _regrip():
                # Die is on table at TABLE_PLACE_ABOVE; reorient and repick
                self._open_gripper()
                time.sleep(GRIPPER_SETTLE_SECS)
                self._move(**TABLE_REPOS_1)
                time.sleep(GRIPPER_SETTLE_SECS)
                self._close_gripper()
                time.sleep(GRIPPER_SETTLE_SECS)
                self._move(**TABLE_REPOS_1_ABOVE)
                time.sleep(GRIPPER_SETTLE_SECS)
                self._move(**TABLE_PLACE_ABOVE)
                time.sleep(GRIPPER_SETTLE_SECS)
                self._move(**TABLE_REPOS_3)
                time.sleep(3.0)
                self._move(**TABLE_REPOS_2)
                time.sleep(GRIPPER_SETTLE_SECS)
                self._open_gripper()
                time.sleep(GRIPPER_SETTLE_SECS)
                self._move(**TABLE_REPOS_PICK)
                time.sleep(GRIPPER_SETTLE_SECS)
                self._close_gripper()
                time.sleep(GRIPPER_SETTLE_SECS)
                self._move(**TABLE_PLACE_ABOVE)
                time.sleep(GRIPPER_SETTLE_SECS)

            # Chirality loop — same strategy as Beaker's _find_pip_rotating
            attempt = 0
            while True:
                attempt += 1
                print(f'  [PipCount] chirality attempt {attempt}')

                front_pip, bottom_pip = _two_views()
                top_pip = 7 - bottom_pip

                # Direct hit
                if target in (front_pip, bottom_pip):
                    print(f'  [PipCount] pip {target} visible — done.')
                    break

                # Derive all six faces from chirality table
                right_pip = _DIE_CHIRALITY.get((top_pip, front_pip))
                if right_pip is not None:
                    back_pip, left_pip = 7 - front_pip, 7 - right_pip
                    face_map = {
                        'front':  front_pip,  'back':   back_pip,
                        'top':    top_pip,    'bottom': bottom_pip,
                        'right':  right_pip,  'left':   left_pip,
                    }
                    target_face = next((f for f, v in face_map.items() if v == target), None)
                    print(f'  [PipCount] All faces: {face_map}')
                    print(f'  [PipCount] pip {target} on {target_face} face')
                else:
                    print(f'  [PipCount] ({top_pip},{front_pip}) invalid orientation')
                    target_face = None

                if attempt >= MAX_PIP_RETRIES:
                    print(f'  [PipCount] pip {target} not found after {attempt} attempts — sending anyway.')
                    self._bunsen_retries += 1
                    self._mb_write(REG_RETRIES, self._bunsen_retries)
                    break

                # Odd attempts: chirality-guided face_rotate (minimum moves).
                # Even attempts: brute-force place + regrip (random reorient).
                if attempt % 2 == 1 and target_face not in (None, 'front', 'bottom'):
                    print(f'  [PipCount] face_rotate: pip {target} is on {target_face} face')
                    # Transit from cam area (CAM_POSE_1) to final position holding die
                    self._move(**CONVEYOR_WAIT_POSE)
                    time.sleep(GRIPPER_SETTLE_SECS)
                    self._move(**FINAL_PLACE_ABOVE)   # position above reposition spot, still holding die
                    time.sleep(GRIPPER_SETTLE_SECS)
                    self._face_rotate(target_face)    # reorient; ends at FINAL_PLACE_ABOVE, gripper open, die on table
                    # Pick die back up from FINAL_PLACE in its new orientation
                    print('  [PipCount] re-picking die after face_rotate')
                    self._move(**FINAL_PLACE_DOWN)    # lower onto die
                    time.sleep(GRIPPER_SETTLE_SECS)
                    self._close_gripper()             # grip die in new orientation
                    time.sleep(GRIPPER_SETTLE_SECS)
                    self._move(**FINAL_PLACE_ABOVE)   # lift up
                    time.sleep(GRIPPER_SETTLE_SECS)
                    self._move(**CONVEYOR_WAIT_POSE)  # waypoint back toward camera area
                    time.sleep(GRIPPER_SETTLE_SECS)
                else:
                    print(f'  [PipCount] regrip (pip {target} on {target_face})...')
                    _place_die()
                    _regrip()

            self._mb_write_coil(COIL_CAMERA_CLIENT, False)
            self._move(**CONVEYOR_WAIT_POSE)
            self._set_state(STATE_FINISH if target == 6 else STATE_PLACE_DIE)

        # ====================================================================
        # State: PLACE_DIE
        # ====================================================================

        def _state_place_die(self):
            self._set_state(STATE_PLACE_DIE)
            target = self._mb_read(REG_PIP_PROGRESS)
            print(f'  [PlaceDie] Placing pip {target} on front conveyor...')

            self._move(**FRONT_CONV_ABOVE)
            self._move(**FRONT_CONV_PLACE)
            self._open_gripper()
            time.sleep(GRIPPER_SETTLE_SECS)
            self._move(**FRONT_CONV_ABOVE)

            print('  [PlaceDie] Running conveyor 9.9 s...')
            self._run_conveyor('reverse')
            time.sleep(9.9)
            self._run_conveyor('stop')

            print('  [PlaceDie] Waiting 3 s then signalling Beaker ready...')
            time.sleep(3.0)
            self._mb_write_coil(COIL_BUNSEN_READY, True)

            self._mb_write(REG_PIP_PROGRESS, target + 1)
            print(f'  [PlaceDie] Done. pip_progress → {target + 1}.')
            self._set_state(STATE_WAIT)

        # ====================================================================
        # State: FINISH
        # ====================================================================

        def _state_finish(self):
            self._set_state(STATE_FINISH)
            print('\n  [Finish] Pip 6 confirmed — placing die in front of Bunsen...')

            self._send_joint(**FINAL_PLACE_ABOVE)
            self._send_joint(**FINAL_PLACE_DOWN)
            self._open_gripper()
            self._send_joint(**FINAL_PLACE_ABOVE)
            self._send_joint(**HOME_JOINTS)

            self._mb_write_coil(COIL_BUNSEN_READY,  False)
            self._mb_write_coil(COIL_CAMERA_CLIENT, False)

            retries = self._mb_read(REG_RETRIES)
            print('\n' + '=' * 55)
            print('         BUNSEN DONE — pip 6 placed!')
            print(f'         Bunsen repositions : {retries}')
            print('=' * 55 + '\n')

        # ====================================================================
        # Step-mode prompt
        # ====================================================================

        _STATE_MAP = {
            '1': STATE_SETUP,
            '2': STATE_WAIT,
            '3': STATE_GRAB_DIE,
            '4': STATE_PIP_COUNT,
            '5': STATE_PLACE_DIE,
            '6': STATE_FINISH,
            '8': STATE_FAULT,
        }

        def _step_prompt(self, state: int) -> int:
            """
            In step mode, pause before running a state.
            Returns the state to run (may be overridden by user).
            Returns -1 to quit the loop.
            """
            name = STATE_NAMES.get(state, str(state))
            print(f'\n{"─" * 50}')
            print(f'  STEP MODE — about to run: {name}')
            print(f'  States: 1=Setup  2=Wait  3=GrabDie  4=PipCount')
            print(f'          5=PlaceDie  6=Finish  8=Fault')
            print(f'  Enter=run  |  1-8=jump to state  |  q=quit')
            print(f'{"─" * 50}')
            while True:
                try:
                    raw = input('  > ').strip().lower()
                except EOFError:
                    return -1
                if raw == '':
                    return state
                if raw == 'q':
                    return -1
                if raw in self._STATE_MAP:
                    new = self._STATE_MAP[raw]
                    self._mb_write(REG_BUNSEN_STATE, new)
                    print(f'  Jumping to {STATE_NAMES.get(new, new)}')
                    return new
                print('  Invalid — press Enter, type a state number, or q.')

        # ====================================================================
        # Main loop
        # ====================================================================

        def run(self, step_mode: bool = False):
            self._step_mode = step_mode
            if not step_mode:
                self._state_setup()

            while True:
                state = self._mb_read(REG_BUNSEN_STATE)

                if step_mode:
                    state = self._step_prompt(state)
                    if state == -1:
                        print('  Step mode: quit.')
                        break

                if state == STATE_SETUP:
                    self._state_setup()

                elif state == STATE_WAIT:
                    self._state_wait()

                elif state == STATE_GRAB_DIE:
                    self._state_grab_die()

                elif state == STATE_PIP_COUNT:
                    self._state_pip_count()

                elif state == STATE_PLACE_DIE:
                    self._state_place_die()

                elif state == STATE_FINISH:
                    self._state_finish()
                    break

                elif state == STATE_FAULT:
                    print('\n  [FAULT] Entering safe stop.')
                    self._run_conveyor('stop')
                    self._open_gripper()
                    self._send_joint(**HOME_JOINTS)
                    self._mb_write_coil(COIL_BUNSEN_READY, False)
                    break

                else:
                    print(f'  [WARN] Unknown state {state} — resetting to Wait.')
                    self._set_state(STATE_WAIT)

        def destroy_node(self):
            if self._camera is not None:
                self._camera.disable()
            self.mb.close()
            super().destroy_node()

    return BunsenMain


def main():
    import argparse
    import rclpy

    parser = argparse.ArgumentParser(description='Bunsen game controller')
    parser.add_argument('--step', action='store_true',
                        help='Step mode: pause before each state for manual confirmation / jump')
    args, _ = parser.parse_known_args()

    rclpy.init()
    # Override SIGINT: let Ctrl+C raise KeyboardInterrupt without destroying context.
    signal.signal(signal.SIGINT, lambda _s, _f: (_ for _ in ()).throw(KeyboardInterrupt()))

    BunsenMain = _make_node()
    node = BunsenMain()
    if args.step:
        print('\n  *** STEP MODE enabled — confirm each state before it runs ***\n')
    try:
        node.run(step_mode=args.step)
    except KeyboardInterrupt:
        print('\nInterrupted — shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
