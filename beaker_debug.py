#!/usr/bin/env python3
"""
beaker_debug.py — Beaker (Robot 1) backup / debug controller.

Mirrors the step-mode structure from bunsen_main.py so you can walk through
each action manually and confirm before the robot moves.

Usage:
    python3 beaker_debug.py           # normal run, no Modbus
    python3 beaker_debug.py --step    # pause before every action
    python3 beaker_debug.py --step --modbus 10.8.4.X   # with Modbus

Requires FANUC driver running:
    ros2 launch launch/start.launch.py robot_name:=Beaker robot_ip:=10.8.4.16

Camera server is optional — falls back to manual pip entry if unavailable:
    ros2 launch dice_game robot1.launch.py robot_name:=Beaker robot_ip:=10.8.4.16 modbus_ip:=0.0.0.0
"""

import argparse
import signal
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from fanuc_interfaces.action import CartPose, JointPose, SchunkGripper, Conveyor
from fanuc_interfaces.srv import CaptureImage

# ── Pip counter (same as main controller) ────────────────────────────────────
try:
    from dice_game.pip_counter import count_pips, save_debug_image
    _PIP_COUNTER_OK = True
except ImportError:
    _PIP_COUNTER_OK = False

# ── Positions — keep in sync with robot1_controller.py ───────────────────────
ROBOT_NAME  = 'Beaker'
RUN_SECONDS = 9.9

HOME_JOINTS   = (1.1, 1.5, -2.0, -1.7, -88.6, -30.0)
PICK_ABOVE    = dict(x=470.0,    y=-15.0,   z=-18.0,   w=179.9, p=0.0,   r=30.0)
PICK_DOWN     = dict(x=470.0,    y=-15.0,   z=-185.0,  w=179.9, p=0.0,   r=30.0)
CAMERA_POSE   = dict(x=490.0,    y=890.0,   z=881.0,   w=73.0,  p=-66.0, r=-170.0)
CONV_REAR_ABV  = dict(x=-194.112, y=617.369, z=200.840, w=179.9, p=0.0,   r=120.0)
CONV_REAR_DRP  = dict(x=-194.112, y=617.369, z=8.840,   w=179.9, p=0.0,   r=120.0)
CONV_REAR_JNT  = (102.382, 51.116, -128.327, 164.504, -126.179, -159.536)
CONV_FRNT_ABV  = dict(x=142.579,  y=617.369, z=200.168, w=179.9, p=0.0,   r=120.0)  # CALIBRATE
CONV_FRNT_DWN  = dict(x=142.579,  y=617.369, z=8.168,   w=179.9, p=0.0,   r=120.0)  # CALIBRATE

# Second camera view — joint angles that tilt the die so its top face points at camera
CAMERA_JOINT_2 = (50.731, 31.588, -14.992, 173.365, -103.358, 125.27)

# Wrist roll offsets applied to CAMERA_POSE['r'] — 60° steps, 6 faces
CAMERA_ROTATION_STEPS = [0, 60, 120, 180, -120, -60]

# Standard western die chirality: (top_pip, front_pip) → right_pip (all 24 orientations)
_DIE_RIGHT = {
    (1, 2): 3, (1, 3): 5, (1, 5): 4, (1, 4): 2,
    (2, 6): 3, (2, 3): 1, (2, 1): 4, (2, 4): 6,
    (3, 2): 6, (3, 6): 5, (3, 5): 1, (3, 1): 2,
    (4, 2): 1, (4, 1): 5, (4, 5): 6, (4, 6): 2,
    (5, 1): 3, (5, 3): 6, (5, 6): 4, (5, 4): 1,
    (6, 5): 3, (6, 3): 2, (6, 2): 4, (6, 4): 5,
}
# Best CAMERA_ROTATION_STEPS index to bring each face toward the camera
_FACE_STEP = {'front': 0, 'right': 2, 'back': 3, 'left': 5}

DEBUG_IMG_DIR = '/home/colin/Desktop'
# ─────────────────────────────────────────────────────────────────────────────


class BeakerDebug(Node):

    def __init__(self):
        super().__init__('beaker_debug')
        self._cart    = ActionClient(self, CartPose,      f'/{ROBOT_NAME}/cartesian_pose')
        self._joint   = ActionClient(self, JointPose,     f'/{ROBOT_NAME}/joint_pose')
        self._gripper = ActionClient(self, SchunkGripper, f'/{ROBOT_NAME}/schunk_gripper')
        self._conv    = ActionClient(self, Conveyor,      f'/{ROBOT_NAME}/conveyor')
        self._cam     = self.create_client(CaptureImage,  '/dice_game/capture_image')

        self._camera_ok = False
        self._step_mode = False
        self._retries   = 0

    # ── Step-mode prompt ──────────────────────────────────────────────────────

    def _step_pause(self, label: str) -> bool:
        """
        Pause before running an action. Press Enter to proceed, q to quit.
        No-op and returns True when step mode is off.
        """
        if not self._step_mode:
            return True
        print(f'\n{"─" * 54}')
        print(f'  STEP MODE — next: {label}')
        print(f'  Enter = run  |  q = quit')
        print(f'{"─" * 54}')
        while True:
            try:
                raw = input('  > ').strip().lower()
            except EOFError:
                return False
            if raw == '':
                return True
            if raw == 'q':
                return False
            print('  Press Enter to run or q to quit.')

    # ── Action helpers ────────────────────────────────────────────────────────

    def _send_cart(self, **kwargs) -> bool:
        self._cart.wait_for_server()
        goal = CartPose.Goal()
        for k, v in kwargs.items():
            setattr(goal, k, float(v))
        fut = self._cart.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut)
        gh = fut.result()
        if not gh.accepted:
            return False
        res = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res)
        return res.result().result.success

    def _send_joint(self, *args) -> bool:
        self._joint.wait_for_server()
        goal = JointPose.Goal()
        goal.joint1, goal.joint2, goal.joint3 = float(args[0]), float(args[1]), float(args[2])
        goal.joint4, goal.joint5, goal.joint6 = float(args[3]), float(args[4]), float(args[5])
        fut = self._joint.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut)
        gh = fut.result()
        if not gh.accepted:
            return False
        res = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res)
        return res.result().result.success

    def _send_gripper(self, command: str) -> bool:
        self._gripper.wait_for_server()
        goal = SchunkGripper.Goal()
        goal.command = command
        fut = self._gripper.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut)
        gh = fut.result()
        if not gh.accepted:
            return False
        res = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res)
        return res.result().result.success

    def _send_conveyor(self, command: str) -> bool:
        self._conv.wait_for_server()
        goal = Conveyor.Goal()
        goal.command = command
        fut = self._conv.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut)
        gh = fut.result()
        if not gh.accepted:
            return False
        res = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res)
        return res.result().result.success

    # ── Camera / pip helpers ──────────────────────────────────────────────────

    def _ask_pip_manual(self, position: str = 'front face') -> int:
        """Prompt operator to type the pip count — mirrors bunsen_main._ask_pip_manual."""
        while True:
            try:
                raw = input(f'  [Manual] {position} — pips you see (1-6): ').strip()
                val = int(raw)
                if 1 <= val <= 6:
                    return val
            except (ValueError, EOFError):
                pass
            print('  Enter a number 1-6.')

    def _capture_pip(self, label: str = 'front') -> int:
        """Capture and count pips; falls back to manual prompt if camera unavailable."""
        if not self._camera_ok or not _PIP_COUNTER_OK:
            return self._ask_pip_manual(label)
        time.sleep(0.4)
        fut = self._cam.call_async(CaptureImage.Request())
        rclpy.spin_until_future_complete(self, fut)
        resp = fut.result()
        if not resp.success:
            self.get_logger().warn(f'Capture failed: {resp.message} — manual input')
            return self._ask_pip_manual(label)
        arr  = np.array(resp.image_data, dtype=np.uint8)
        img  = arr.reshape((resp.height, resp.width, resp.channels))
        pips = count_pips(img)
        path = f'{DEBUG_IMG_DIR}/beaker_debug_{label}.png'
        save_debug_image(img, pips, path)
        self.get_logger().info(f'Camera ({label}): {pips} pip(s) — saved {path}')
        return pips

    # ── Motion primitives ─────────────────────────────────────────────────────

    def _pick_die(self):
        self._send_gripper('open')
        self._send_cart(**PICK_ABOVE)
        self._send_cart(**PICK_DOWN)
        self._send_gripper('close')
        self._send_cart(**PICK_ABOVE)

    def _reorient(self):
        """Release die at PICK_DOWN and immediately repick."""
        self._send_cart(**PICK_ABOVE)
        self._send_cart(**PICK_DOWN)
        self._send_gripper('open')
        self._send_cart(**PICK_ABOVE)
        self._send_cart(**PICK_DOWN)
        self._send_gripper('close')
        self._send_cart(**PICK_ABOVE)
        self._retries += 1

    def _place_on_conveyor(self):
        """Drop die at CONV_REAR_JNT (pip-face-up) and retreat to CONV_REAR_ABV."""
        self._send_joint(*CONV_REAR_JNT)
        self._send_gripper('open')
        self._send_cart(**CONV_REAR_ABV)

    def _find_pip_chirality(self, target: int) -> bool:
        """
        Two-view chirality approach — same logic as robot1_controller.py:
          VIEW 1 at CAMERA_POSE    → front face
          VIEW 2 at CAMERA_JOINT_2 → top face
          Chirality table          → locate target face
          Wrist rotation           → bring target face to front
          Confirm                  → return True if verified, False otherwise.
        Falls back to full 6-position scan if orientation is invalid.
        """
        self._send_cart(**CAMERA_POSE)

        # VIEW 1
        front_pip = self._capture_pip('front')
        self.get_logger().info(f'  VIEW 1 (front): {front_pip} pip(s)')

        # VIEW 2
        self._send_joint(*CAMERA_JOINT_2)
        top_pip = self._capture_pip('top')
        self.get_logger().info(f'  VIEW 2 (top):   {top_pip} pip(s)')
        self._send_cart(**CAMERA_POSE)

        # Chirality
        right_pip = _DIE_RIGHT.get((top_pip, front_pip))
        if right_pip is None:
            print(f'  ! ({top_pip},{front_pip}) not valid — scanning all positions...')
            return self._scan_all(target)

        back_pip = 7 - front_pip
        left_pip = 7 - right_pip
        face_map = {
            'front': front_pip, 'right': right_pip,
            'back':  back_pip,  'left':  left_pip,
            'top':   top_pip,   'bottom': 7 - top_pip,
        }
        target_face = next((f for f, v in face_map.items() if v == target), None)
        print(f'  Die: front={front_pip} right={right_pip} back={back_pip}'
              f' left={left_pip} top={top_pip}  →  pip {target} on {target_face}')

        if target_face in ('top', 'bottom'):
            print(f'  Cannot reach {target_face} by wrist rotation — re-orient needed.')
            return False

        # Rotate to target face
        r_offset = CAMERA_ROTATION_STEPS[_FACE_STEP[target_face]]
        if r_offset != 0:
            self._send_cart(**{**CAMERA_POSE, 'r': CAMERA_POSE['r'] + r_offset})

        # Confirm
        pips = self._capture_pip(f'confirm_r{r_offset:+.0f}')
        self.get_logger().info(f'  Confirm at r={r_offset:+.0f}°: {pips} pip(s)')

        if pips == target:
            print(f'  >> Pip {target} confirmed on front face!\n')
            return True

        print(f'  Chirality predicted {target} but saw {pips} — re-orient needed.\n')
        return False

    def _scan_all(self, target: int) -> bool:
        """Fallback: sweep all 6 wrist rotations."""
        for i, r_offset in enumerate(CAMERA_ROTATION_STEPS):
            if r_offset != 0:
                self._send_cart(**{**CAMERA_POSE, 'r': CAMERA_POSE['r'] + r_offset})
            pips = self._capture_pip(f'scan_{i}')
            self.get_logger().info(f'  Scan {i + 1}/6  r={r_offset:+.0f}°: {pips} pip(s)')
            if pips == target:
                print(f'  >> Pip {target} found at scan position {i + 1}!\n')
                return True
        print(f'  Pip {target} not found at any position.\n')
        return False

    # ── Phase 1: find pip 1 ───────────────────────────────────────────────────

    def _phase1(self) -> bool:
        print('\n' + '=' * 54)
        print('  PHASE 1 — find pip 1')
        print('=' * 54)

        if not self._step_pause('Pick die from table'):
            return False
        self.get_logger().info('Picking die...')
        self._pick_die()

        attempt = 0
        while True:
            attempt += 1
            if not self._step_pause(f'Camera check for pip 1 (attempt {attempt})'):
                return False
            if self._find_pip_chirality(1):
                print('  >> Pip 1 on front face — Phase 1 complete!\n')
                return True

            if not self._step_pause('Pip 1 not found — release and repick at PICK_DOWN'):
                return False
            self.get_logger().info('Re-orienting die...')
            self._reorient()

    # ── Phase 2: sequential 1 → 6 ────────────────────────────────────────────

    def _phase2(self) -> bool:
        print('\n' + '=' * 54)
        print('  PHASE 2 — sequential pip 1 → 6')
        print('=' * 54)

        holding_die = True   # carried over from Phase 1

        for target in range(1, 7):
            print(f'\n  ── Target pip: {target} ──')
            attempt = 0

            while True:
                attempt += 1

                if not holding_die:
                    if not self._step_pause(f'Pick die for pip {target}'):
                        return False
                    self.get_logger().info('Picking die...')
                    self._pick_die()
                holding_die = False

                if not self._step_pause(
                    f'Camera check for pip {target} (attempt {attempt})'
                ):
                    return False
                if self._find_pip_chirality(target):
                    print(f'  >> Pip {target} confirmed on front face!\n')
                    break

                if not self._step_pause(
                    f'Pip {target} not found — release and repick at PICK_DOWN'
                ):
                    return False
                self.get_logger().info('Re-orienting die...')
                self._reorient()
                holding_die = True

            if not self._step_pause(f'Drop pip {target} on rear conveyor → Bunsen'):
                return False
            self.get_logger().info(f'Placing pip {target} on conveyor...')
            self._place_on_conveyor()

            if not self._step_pause(f'Run rear belt for {RUN_SECONDS}s'):
                return False
            self._send_conveyor('forward')
            time.sleep(RUN_SECONDS)
            self._send_conveyor('stop')
            self.get_logger().info('Belt stopped.')

            if target == 6:
                break

            if not self._step_pause(f'Wait — Bunsen returns die after pip {target}'):
                return False
            print('  [Waiting] Press Enter once Bunsen has placed die on front conveyor...')
            input()

            if not self._step_pause('Pick die from front conveyor'):
                return False
            self._send_gripper('open')
            self._send_cart(**CONV_FRNT_ABV)
            self._send_cart(**CONV_FRNT_DWN)
            self._send_gripper('close')
            self._send_cart(**CONV_FRNT_ABV)
            holding_die = True
            self.get_logger().info('Die received from Bunsen.')

        return True

    # ── Main ─────────────────────────────────────────────────────────────────

    def run(self, step_mode: bool = False):
        self._step_mode = step_mode

        self.get_logger().info('Waiting for action servers...')
        if not self._cart.wait_for_server(timeout_sec=15.0):
            self.get_logger().error('Cartesian server not found after 15s.')
            return

        self._camera_ok = self._cam.wait_for_service(timeout_sec=5.0)
        if self._camera_ok and _PIP_COUNTER_OK:
            self.get_logger().info('Camera service found — using camera.')
        else:
            self.get_logger().warn('Camera unavailable — using manual pip input.')

        ok = self._phase1()
        if ok:
            ok = self._phase2()

        if ok:
            print('\n' + '=' * 54)
            print(f'  Done!  Total re-orients: {self._retries}')
            print('=' * 54 + '\n')
        else:
            self.get_logger().warn('Run aborted by user (step mode quit).')


def main():
    parser = argparse.ArgumentParser(description='Beaker debug / backup controller')
    parser.add_argument('--step', action='store_true',
                        help='Pause before every robot action for manual confirmation')
    parser.add_argument('--modbus', default='',
                        help='Bunsen Modbus IP (optional, not used in this debug script)')
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args or None)
    signal.signal(signal.SIGINT, lambda _s, _f: (_ for _ in ()).throw(KeyboardInterrupt()))

    node = BeakerDebug()
    if args.step:
        print('\n  *** STEP MODE — press Enter before each action, q to quit ***\n')
    try:
        node.run(step_mode=args.step)
    except KeyboardInterrupt:
        print('\nInterrupted — stopping conveyor.')
        node._send_conveyor('stop')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
