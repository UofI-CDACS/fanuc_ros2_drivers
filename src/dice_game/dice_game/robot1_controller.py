"""
robot1_controller.py  —  Beaker (Robot 1)

State machine:
  SEARCHING_START  Pick die, rotate wrist at camera until pip == 1 found.
  SEQUENTIAL       For targets 1-6: find pip by wrist rotation, send to Bunsen
                   via rear conveyor handshake, receive back via front conveyor.
  DONE             Print results and return home.

Modbus registers (matches modbus_server.py on Bunsen):
  HR 0  STATE         current state (1-9, see State enum)
  HR 1  PIP_PROGRESS  current target pip (1-6)
  HR 2  CONV_CMD      conveyor handshake state machine
  HR 3  RETRIES       Bunsen's cumulative retry count (read at end)

  Coil 0  READY         not used by Beaker
  Coil 1  CAMERA_CLIENT 1 = Bunsen holds camera token, 0 = Beaker holds it

ROS2 topics (monitoring / backup, under /dice_game/):
  Publishes:
    pip_count        (Int32)  — pip just found
    conveyor_select  (String) — 'rear' when sending to Bunsen
    dice_ready       (Bool)   — True when die placed on conveyor
"""

import time
from enum import IntEnum

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from fanuc_interfaces.action import CartPose, JointPose, SchunkGripper, Conveyor
from fanuc_interfaces.srv import CaptureImage
from std_msgs.msg import Int32, String, Bool
from pymodbus.client import ModbusTcpClient

from dice_game.pip_counter import count_pips, save_debug_image

# ── Robot 1 positions (Beaker) ────────────────────────────────────────────────
# Cartesian: x, y, z in mm  |  w, p, r in degrees
HOME_JOINTS   = (1.1, 1.5, -2.0, -1.7, -88.6, -30.0)
PICK_ABOVE    = dict(x=470.0, y=-15.0,  z=-18.0,  w=179.9, p=0.0,   r=30.0)
PICK_DOWN     = dict(x=470.0, y=-15.0,  z=-185.0, w=179.9, p=0.0,   r=30.0)
# Re-orient drop: same position as pick but wrist rotated 90° (r=120° vs r=30°).
# Die is set down in a new orientation, then picked back up at r=30° so the next
# camera view sees a fresh face.  Follows DROP_POSE from Controlling_robots_using_claude.py.
REORIENT_ABV  = dict(x=470.0, y=-15.0,  z=-18.0,  w=179.9, p=0.0,   r=120.0)  # CALIBRATE
REORIENT_DWN  = dict(x=470.0, y=-15.0,  z=-185.0, w=179.9, p=0.0,   r=120.0)  # CALIBRATE
CAMERA_POSE   = dict(x=490.0, y=890.0,  z=881.0,  w=73.0,  p=-66.0, r=-170.0)
# Second camera view — joint pose that rotates J5 ~90° so the die face that was
# resting on the table (the "bottom" at pick) points toward the camera.
# This lets the code read both front and top with the camera (or two manual prompts)
# and then use the chirality table to locate any target pip.
# Fill in your calibrated joint angles and remove the "None" assignment.
CAMERA_JOINT_2 = None   # CALIBRATE: replace with (j1, j2, j3, j4, j5, j6) degrees
#                          e.g. (1.1, 1.5, -2.0, -1.7, 1.4, -30.0)
CONV_REAR_ABV = dict(x=-194.112, y=617.369,  z=200.840,  w=179.9, p=0.0,   r=120.0)
CONV_REAR_DRP = dict(x=-194.112, y=617.369,  z=8.840,  w=179.9, p=0.0,   r=120.0)
# Front conveyor — Bunsen sends die back here; needs physical calibration
CONV_FRNT_ABV = dict(x=142.579, y=617.369, z=200.168, w=179.9, p=0.0, r=120.0)   # CALIBRATE
CONV_FRNT_DWN = dict(x=142.579, y=617.369, z=8.168, w=179.9, p=0.0, r=120.0)   # CALIBRATE

# Wrist roll offsets applied to CAMERA_POSE['r'] — 60° steps cover all 6 faces
CAMERA_ROTATION_STEPS = [0, 60, 120, 180, -120, -60]

DEBUG_IMG_DIR        = '/home/colin/Desktop'
CONV_TIMEOUT         = 60.0   # seconds to wait for conveyor handshake steps
CONVEYOR_TRAVEL_SECS = 5.0    # time for front belt (Bunsen-side, receiving die back)
POLL_INTERVAL        = 0.2

# ── Edit this to tune how long the rear belt runs to deliver die to Bunsen ───
REAR_CONVEYOR_TRAVEL_SECS = 5.0

# ── Modbus — mirrors modbus_server.py running on Bunsen ──────────────────────
MODBUS_PORT = 5020

# Holding register addresses
REG_STATE        = 0
REG_PIP_PROGRESS = 1
REG_CONV_CMD     = 2
REG_RETRIES      = 3

# Coil addresses
COIL_READY         = 0
COIL_CAMERA_CLIENT = 1   # 1 = Bunsen holds camera token

# Conveyor state machine values
CONV_IDLE              = 0
CONV_BEAKER_WANTS_SEND = 1   # Beaker: die ready, ask Bunsen to start rear belt
CONV_REAR_RUNNING      = 2   # Bunsen: rear belt running, Beaker may place die
CONV_DIE_ON_REAR       = 3   # Beaker: die placed on rear belt
CONV_BUNSEN_HAS_DIE    = 4   # Bunsen: die picked up
CONV_BUNSEN_WANTS_SEND = 5   # Bunsen: die ready, ask Beaker to start front belt
CONV_FRONT_RUNNING     = 6   # Beaker: front belt running, Bunsen may place die
CONV_DIE_ON_FRONT      = 7   # Bunsen: die placed on front belt
CONV_BEAKER_HAS_DIE    = 8   # Beaker: die picked up

# ── Standard western die chirality ────────────────────────────────────────────
# (top_pip, front_pip) → right_pip   (all 24 valid orientations)
# Opposite faces always sum to 7: 1-6, 2-5, 3-4.
# Right-handed rule: when 1 is up and 2 faces you, 3 is to your right.
_DIE_RIGHT = {
    (1, 2): 3, (1, 3): 5, (1, 5): 4, (1, 4): 2,
    (2, 6): 3, (2, 3): 1, (2, 1): 4, (2, 4): 6,
    (3, 2): 6, (3, 6): 5, (3, 5): 1, (3, 1): 2,
    (4, 2): 1, (4, 1): 5, (4, 5): 6, (4, 6): 2,
    (5, 1): 3, (5, 3): 6, (5, 6): 4, (5, 4): 1,
    (6, 5): 3, (6, 3): 2, (6, 2): 4, (6, 4): 5,
}

# Best rotation step index (into CAMERA_ROTATION_STEPS) for each die face.
# Assumes increasing wrist roll brings the RIGHT face toward the camera.
# If the robot moves to the opposite face, swap 'right' and 'left' indices (2↔5).
# top/bottom are not reachable by wrist roll alone — re-pick needed if target is there.
_FACE_STEP = {'front': 0, 'right': 2, 'back': 3, 'left': 5}


class State(IntEnum):
    SETUP      = 1
    WAIT       = 2
    GRAB_DIE   = 3
    PIP_COUNT  = 4
    ROTATE_PIP = 5
    PLACE_DIE  = 6
    FINISH     = 7
    RECOVER    = 8
    FAULT      = 9


class Robot1Controller(Node):

    def __init__(self):
        super().__init__('robot1_controller')

        self.declare_parameters('', [
            ('robot_name', 'Beaker'),
            ('robot_ip',   '10.8.4.16'),
            ('modbus_ip',  '0.0.0.0'),     # Bunsen's IP — set via launch file
        ])
        self._name = self.get_parameter('robot_name').value
        self._mb: ModbusTcpClient | None = None

        # ── Action clients ────────────────────────────────────────────────────
        self._cart    = ActionClient(self, CartPose,      f'/{self._name}/cartesian_pose')
        self._joint   = ActionClient(self, JointPose,     f'/{self._name}/joint_pose')
        self._gripper = ActionClient(self, SchunkGripper, f'/{self._name}/schunk_gripper')
        self._conv    = ActionClient(self, Conveyor,      f'/{self._name}/conveyor')

        # ── Camera service client ─────────────────────────────────────────────
        self._cam_cli = self.create_client(CaptureImage, '/dice_game/capture_image')

        # ── ROS2 monitoring publishers ────────────────────────────────────────
        self._pub_pip      = self.create_publisher(Int32,  '/dice_game/pip_count',       10)
        self._pub_conveyor = self.create_publisher(String, '/dice_game/conveyor_select',  10)
        self._pub_ready    = self.create_publisher(Bool,   '/dice_game/dice_ready',       10)

        # ── Counters ──────────────────────────────────────────────────────────
        self.r1_retries   = 0
        self._total_counts = []   # (target, actual_pip, round_retries)
        self._camera_ok   = False

    # ── Modbus helpers ────────────────────────────────────────────────────────

    def _modbus_connect(self, ip: str):
        if ip == '0.0.0.0':
            self.get_logger().warn('modbus_ip not set — Modbus disabled')
            return
        self._mb = ModbusTcpClient(host=ip, port=MODBUS_PORT)
        if self._mb.connect():
            self.get_logger().info(f'Modbus connected to {ip}:{MODBUS_PORT}')
        else:
            self.get_logger().warn(f'Modbus connection to {ip} failed — continuing without Modbus')
            self._mb = None

    def _mb_read(self, addr: int) -> int:
        if self._mb:
            return self._mb.read_holding_registers(addr, 1).registers[0]
        return 0

    def _mb_write(self, addr: int, val: int):
        if self._mb:
            self._mb.write_register(addr, val)

    def _mb_write_coil(self, addr: int, val: bool):
        if self._mb:
            self._mb.write_coil(addr, bool(val))

    def _set_state(self, state: State):
        self.get_logger().info(f'State → {state.name}')
        self._mb_write(REG_STATE, int(state))

    def _set_pip_progress(self, pip: int):
        self._mb_write(REG_PIP_PROGRESS, pip)

    def _wait_conv(self, target: int, timeout: float = CONV_TIMEOUT) -> bool:
        """Poll CONV_CMD register until it equals target or timeout."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self._mb_read(REG_CONV_CMD) == target:
                return True
            time.sleep(POLL_INTERVAL)
        return False

    # ── Action helpers ────────────────────────────────────────────────────────

    def _send_cart(self, x, y, z, w=200.0, p=200.0, r=200.0) -> bool:
        self._cart.wait_for_server()
        goal = CartPose.Goal()
        goal.x, goal.y, goal.z = float(x), float(y), float(z)
        goal.w, goal.p, goal.r = float(w), float(p), float(r)
        future = self._cart.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        gh = future.result()
        if not gh.accepted:
            return False
        res = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res)
        return res.result().result.success

    def _send_joint(self, j1, j2, j3, j4, j5, j6) -> bool:
        self._joint.wait_for_server()
        goal = JointPose.Goal()
        goal.joint1, goal.joint2, goal.joint3 = float(j1), float(j2), float(j3)
        goal.joint4, goal.joint5, goal.joint6 = float(j4), float(j5), float(j6)
        future = self._joint.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        gh = future.result()
        if not gh.accepted:
            return False
        res = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res)
        return res.result().result.success

    def _send_gripper(self, command: str) -> bool:
        self._gripper.wait_for_server()
        goal = SchunkGripper.Goal()
        goal.command = command
        future = self._gripper.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        gh = future.result()
        if not gh.accepted:
            return False
        res = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res)
        return res.result().result.success

    def _run_conveyor(self, command: str) -> bool:
        self._conv.wait_for_server()
        goal = Conveyor.Goal()
        goal.command = command
        future = self._conv.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        gh = future.result()
        if not gh.accepted:
            return False
        res = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res)
        return res.result().result.success

    # ── Camera ────────────────────────────────────────────────────────────────

    def _capture_count_at(self, label: str) -> int:
        """Capture image at current position and count pips (no movement)."""
        if not self._camera_ok:
            time.sleep(0.4)
            return -1
        time.sleep(0.4)
        req = CaptureImage.Request()
        future = self._cam_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        resp = future.result()

        if not resp.success:
            self.get_logger().error(f'Camera capture failed: {resp.message}')
            return 0

        arr  = np.array(resp.image_data, dtype=np.uint8)
        img  = arr.reshape((resp.height, resp.width, resp.channels))
        pips = count_pips(img)

        path = f'{DEBUG_IMG_DIR}/r1_{label}.png'
        save_debug_image(img, pips, path)
        self.get_logger().info(f'Robot 1 counted {pips} pip(s) — saved {path}')
        return pips

    def _find_pip_rotating(self, target: int, label: str) -> int:
        """
        Move to camera pose and rotate wrist in 60° steps (6 positions = full turn)
        looking for target pip count. Returns pip count when found, or 0 if the target
        face is not seen in any orientation. Robot stays at the matching rotation.

        When camera is unavailable, falls back to _find_pip_manual() which prompts
        the user to type the pip count they see at each rotation position.
        """
        self.get_logger().info(f'Scanning for pip={target} by rotating wrist...')
        self._send_cart(**CAMERA_POSE)

        if not self._camera_ok:
            return self._find_pip_manual(target)

        for i, r_offset in enumerate(CAMERA_ROTATION_STEPS):
            self._set_state(State.PIP_COUNT if i == 0 else State.ROTATE_PIP)

            if r_offset != 0:
                self._send_cart(**{**CAMERA_POSE, 'r': CAMERA_POSE['r'] + r_offset})

            pips = self._capture_count_at(f'{label}_rot{i}')
            self.get_logger().info(f'  r_offset={r_offset:+.0f}°: {pips} pip(s)')

            if pips == target:
                return pips

        return 0

    def _find_pip_manual(self, target: int) -> int:
        """
        Manual camera fallback — called when camera service is unavailable.

        Robot is already at CAMERA_POSE (called from _find_pip_rotating).

        1. Prompts for the front face (camera-facing) and top face (2 numbers).
        2. Uses the standard die chirality table to compute all 6 faces.
        3. Jumps directly to the rotation step most likely to show the target pip.
        4. Prompts for confirmation at that step; if wrong, sweeps remaining steps.

        Falls back to a full step-by-step scan if the (top, front) combination is
        not a valid standard-die orientation.

        Returns the target pip count when found, or 0 if not found on any face.
        """
        print(f'\n{"=" * 56}')
        print(f'  MANUAL MODE  —  looking for pip {target}')
        print(f'{"=" * 56}')

        # ── View 1: front face at CAMERA_POSE ────────────────────────────────
        print('\n  VIEW 1  —  die at camera position.')
        front_pip = self._prompt_face('Front face (what you see facing the camera)')

        # ── View 2: top face at CAMERA_JOINT_2 ───────────────────────────────
        if CAMERA_JOINT_2 is not None:
            print('\n  Moving to second view (J5 rotated ~90°)...')
            self._send_joint(*CAMERA_JOINT_2)
            print('  VIEW 2  —  bottom-of-table face now points at camera.')
            top_pip = self._prompt_face('Top face  (what you see now)              ')
            # Return to camera base before the rotation sweep
            self._send_cart(**CAMERA_POSE)
        else:
            # CAMERA_JOINT_2 not calibrated yet — ask user to read top face in place
            print('\n  VIEW 2  —  (CAMERA_JOINT_2 not set; look at the top of the die)')
            top_pip = self._prompt_face('Top face  (what is pointing up)           ')

        right_pip = _DIE_RIGHT.get((top_pip, front_pip))
        if right_pip is None:
            print(f'\n  ! ({top_pip}, {front_pip}) is not a valid standard-die orientation.')
            print(f'  Falling back to step-by-step scan...')
            return self._scan_all_steps(target)

        back_pip   = 7 - front_pip
        left_pip   = 7 - right_pip
        bottom_pip = 7 - top_pip

        print(f'\n  Die layout:')
        print(f'    front={front_pip}  right={right_pip}  back={back_pip}'
              f'  left={left_pip}  top={top_pip}  bottom={bottom_pip}')

        face_map = {
            'front': front_pip, 'right': right_pip,
            'back':  back_pip,  'left':  left_pip,
            'top':   top_pip,   'bottom': bottom_pip,
        }
        target_face = next((f for f, v in face_map.items() if v == target), None)
        print(f'  Pip {target} is on the {target_face} face.')

        if target_face in ('top', 'bottom'):
            print(f'  Cannot reach the {target_face} face by wrist rotation — re-pick needed.')
            return 0

        # Best-guess step first, then sweep any remaining steps
        best = _FACE_STEP[target_face]
        step_order = [best] + [i for i in range(len(CAMERA_ROTATION_STEPS)) if i != best]

        for i in step_order:
            r_offset = CAMERA_ROTATION_STEPS[i]
            self._set_state(State.PIP_COUNT if i == 0 else State.ROTATE_PIP)
            if r_offset != 0:
                self._send_cart(**{**CAMERA_POSE, 'r': CAMERA_POSE['r'] + r_offset})

            pips = self._prompt_pip(i, r_offset)
            self.get_logger().info(f'  Position {i + 1}/6  r={r_offset:+.0f}°: {pips} pip(s)')
            self._pub_pip.publish(Int32(data=pips))

            if pips == target:
                print(f'  >> Pip {target} found — continuing.\n')
                return pips

        print(f'  Pip {target} not found at any position.\n')
        return 0

    def _scan_all_steps(self, target: int) -> int:
        """Fallback: step through all 6 rotation positions in order, prompting at each."""
        for i, r_offset in enumerate(CAMERA_ROTATION_STEPS):
            self._set_state(State.PIP_COUNT if i == 0 else State.ROTATE_PIP)
            if r_offset != 0:
                self._send_cart(**{**CAMERA_POSE, 'r': CAMERA_POSE['r'] + r_offset})

            pips = self._prompt_pip(i, r_offset)
            self.get_logger().info(f'  Position {i + 1}/6  r={r_offset:+.0f}°: {pips} pip(s)')
            self._pub_pip.publish(Int32(data=pips))

            if pips == target:
                print(f'  >> Pip {target} found — continuing.\n')
                return pips

        print(f'  Pip {target} not found at any position.\n')
        return 0

    @staticmethod
    def _prompt_face(label: str) -> int:
        """Prompt user for a die face pip count (1-6)."""
        while True:
            try:
                val = int(input(f'  {label} [1-6]: ').strip())
                if 1 <= val <= 6:
                    return val
            except (ValueError, EOFError):
                pass
            print('    Please enter a number from 1 to 6.')

    @staticmethod
    def _prompt_pip(step: int, r_offset: float) -> int:
        """Prompt user for the pip count visible at the current rotation position."""
        while True:
            try:
                raw = input(
                    f'\n  Position {step + 1}/6  (wrist {r_offset:+.0f}°)'
                    f'  —  pip count you see [1-6]: '
                ).strip()
                val = int(raw)
                if 1 <= val <= 6:
                    return val
            except (ValueError, EOFError):
                pass
            print('    Please enter a whole number from 1 to 6.')

    # ── Robot moves ───────────────────────────────────────────────────────────

    def go_home(self):
        self.get_logger().info('Going home...')
        self._send_joint(*HOME_JOINTS)

    def pick_dice(self):
        self._send_gripper('open')
        self._send_cart(**PICK_ABOVE)
        self._send_cart(**PICK_DOWN)
        self._send_gripper('close')
        self._send_cart(**PICK_ABOVE)

    def reorient_and_repick(self):
        """
        Drop die at REORIENT_DWN (wrist 90° offset from pick) then pick it back
        up at PICK_DOWN (standard orientation).  The 90° offset means the gripper
        releases a different die axis each time, so the next camera view sees a
        fresh face.  Matches the DROP_POSE pattern in Controlling_robots_using_claude.py.
        Calibrate REORIENT_ABV / REORIENT_DWN for the physical drop position.
        """
        self.get_logger().info('Re-orienting die — dropping at 90° wrist offset...')
        self._send_cart(**REORIENT_ABV)
        self._send_cart(**REORIENT_DWN)
        self._send_gripper('open')
        self._send_cart(**REORIENT_ABV)

        self.get_logger().info('Re-picking die in standard orientation...')
        self._send_cart(**PICK_ABOVE)
        self._send_cart(**PICK_DOWN)
        self._send_gripper('close')
        self._send_cart(**PICK_ABOVE)

    # ── Conveyor handshake ────────────────────────────────────────────────────

    def send_to_bunsen(self) -> bool:
        """
        Place die on rear conveyor and hand off to Bunsen.
        Handshake: BEAKER_WANTS_SEND → wait REAR_RUNNING → place die
                   → DIE_ON_REAR (give camera token) → wait BUNSEN_HAS_DIE → IDLE
        """
        self.get_logger().info('Sending die to Bunsen via rear conveyor...')

        # Tell Bunsen we want to send — it will start the rear belt
        self._mb_write(REG_CONV_CMD, CONV_BEAKER_WANTS_SEND)

        if not self._wait_conv(CONV_REAR_RUNNING):
            self.get_logger().error('Timeout: Bunsen did not start rear conveyor')
            self._mb_write(REG_CONV_CMD, CONV_IDLE)
            return False

        # Drop die onto rear belt then run it toward Bunsen
        self._send_cart(**CONV_REAR_ABV)
        self._send_cart(**CONV_REAR_DRP)
        self._send_gripper('open')
        self._send_cart(**CONV_REAR_ABV)

        self._run_conveyor('forward')
        self.get_logger().info(f'Rear belt running for {REAR_CONVEYOR_TRAVEL_SECS}s...')
        time.sleep(REAR_CONVEYOR_TRAVEL_SECS)
        self._run_conveyor('stop')

        # Signal die is on belt and hand camera token to Bunsen for verification
        self._mb_write(REG_CONV_CMD, CONV_DIE_ON_REAR)
        self._mb_write_coil(COIL_CAMERA_CLIENT, True)

        # ROS2 monitoring
        self._pub_conveyor.publish(String(data='rear'))
        self._pub_ready.publish(Bool(data=True))

        if not self._wait_conv(CONV_BUNSEN_HAS_DIE):
            self.get_logger().error('Timeout: Bunsen did not confirm die pickup')
            return False

        self._mb_write(REG_CONV_CMD, CONV_IDLE)
        return True

    def receive_from_bunsen(self) -> bool:
        """
        Receive die back from Bunsen via front conveyor (Bunsen-controlled).
        Handshake: wait BUNSEN_WANTS_SEND → FRONT_RUNNING (signal Bunsen to start belt)
                   → wait DIE_ON_FRONT → travel delay → pick
                   → BEAKER_HAS_DIE → wait IDLE
        """
        self.get_logger().info('Waiting for Bunsen to send die back...')

        if not self._wait_conv(CONV_BUNSEN_WANTS_SEND):
            self.get_logger().error('Timeout: Bunsen did not signal WANTS_SEND')
            return False

        # Signal Bunsen it may start the front belt and place the die
        self._mb_write(REG_CONV_CMD, CONV_FRONT_RUNNING)

        if not self._wait_conv(CONV_DIE_ON_FRONT):
            self.get_logger().error('Timeout: Bunsen did not place die on front conveyor')
            return False

        # Wait for die to travel to pickup point (Bunsen controls the front belt)
        time.sleep(CONVEYOR_TRAVEL_SECS)

        # Take camera token back before picking (Beaker needs camera for next round)
        self._mb_write_coil(COIL_CAMERA_CLIENT, False)

        # Pick die from front conveyor
        self._send_gripper('open')
        self._send_cart(**CONV_FRNT_ABV)
        self._send_cart(**CONV_FRNT_DWN)
        self._send_gripper('close')
        self._send_cart(**CONV_FRNT_ABV)

        self._mb_write(REG_CONV_CMD, CONV_BEAKER_HAS_DIE)

        # Wait for Bunsen to acknowledge and reset to IDLE
        self._wait_conv(CONV_IDLE, timeout=10.0)

        self._pub_ready.publish(Bool(data=False))
        return True

    # ── Main game loop ────────────────────────────────────────────────────────

    def run(self):
        self._set_state(State.SETUP)
        self.get_logger().info('Waiting for action servers and camera service...')
        self._cart.wait_for_server()
        self._joint.wait_for_server()
        self._gripper.wait_for_server()
        self._conv.wait_for_server()

        self._camera_ok = self._cam_cli.wait_for_service(timeout_sec=5.0)
        if self._camera_ok:
            self.get_logger().info('Camera service found.')
        else:
            self.get_logger().warn('Camera service not found — running without camera, pip counts will be skipped')

        modbus_ip = self.get_parameter('modbus_ip').value
        self._modbus_connect(modbus_ip)
        self._set_state(State.SETUP)

        self.get_logger().info('All servers ready. Starting game.')

        self._set_state(State.WAIT)
        self.go_home()

        try:
            # ── Phase 1: find pip = 1 ────────────────────────────────────────
            self.get_logger().info('\n=== PHASE 1: Searching for pip = 1 ===')
            self._set_pip_progress(0)
            self._set_state(State.GRAB_DIE)
            self.pick_dice()

            while True:
                pips = self._find_pip_rotating(1, 'search')
                self._pub_pip.publish(Int32(data=pips))

                if pips == 1:
                    self.get_logger().info('pip = 1 found — START STATE reached!')
                    break

                self.get_logger().info('pip = 1 not on any face — re-orienting die...')
                self._set_state(State.RECOVER)
                self.reorient_and_repick()
                self.r1_retries += 1

            # ── Phase 2: sequential 1 → 6 ────────────────────────────────────
            self.get_logger().info('\n=== PHASE 2: Sequential 1 → 6 ===')

            # Already holding pip=1 from Phase 1
            holding_die = True

            for target in range(1, 7):
                round_retries = 0
                self.get_logger().info(f'\n--- Target pip: {target} ---')
                self._set_pip_progress(target)

                while True:
                    if not holding_die:
                        self._set_state(State.GRAB_DIE)
                        self.pick_dice()
                    holding_die = False

                    pips = self._find_pip_rotating(target, f'seq_t{target}_r{round_retries}')
                    self._pub_pip.publish(Int32(data=pips))

                    if pips == target:
                        self.get_logger().info(f'Correct pip ({pips}) — sending to Bunsen')
                        break

                    self.get_logger().info(f'pip={target} not on any face — re-orienting die...')
                    self._set_state(State.RECOVER)
                    self.reorient_and_repick()
                    holding_die = True
                    round_retries += 1
                    self.r1_retries += 1

                self._total_counts.append((target, pips, round_retries))

                self._set_state(State.PLACE_DIE)
                if not self.send_to_bunsen():
                    self._set_state(State.FAULT)
                    break

                if target == 6:
                    break   # Bunsen places pip=6 — game over

                # Wait for Bunsen to verify and return the die
                self._set_state(State.WAIT)
                if not self.receive_from_bunsen():
                    self.get_logger().error('Failed to receive die back from Bunsen')
                    self._set_state(State.FAULT)
                    break

                holding_die = True
                self.get_logger().info('Die received — moving to next target')

            # ── Results ───────────────────────────────────────────────────────
            self._set_state(State.FINISH)
            self.go_home()
            self._print_results()

        except Exception as e:
            self.get_logger().error(f'Unhandled exception: {e}')
            self._set_state(State.FAULT)
            raise

        finally:
            if self._mb:
                self._mb.close()

    def _print_results(self):
        bunsen_retries = self._mb_read(REG_RETRIES)
        sep = '=' * 50
        print(f'\n{sep}')
        print('         DICE GAME  —  RESULTS')
        print(sep)
        print(f'  {"Target":>6}  {"Got":>4}  {"R1 Retries":>10}')
        print(f'  {"-"*6}  {"-"*4}  {"-"*10}')
        for target, pips, retries in self._total_counts:
            print(f'  {target:>6}  {pips:>4}  {retries:>10}')
        print(f'  {"-"*35}')
        print(f'  Beaker (R1) retries : {self.r1_retries}')
        print(f'  Bunsen (R2) retries : {bunsen_retries}')
        print(f'  Combined            : {self.r1_retries + bunsen_retries}')
        print(sep + '\n')


def main(args=None):
    rclpy.init(args=args)
    node = Robot1Controller()
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
