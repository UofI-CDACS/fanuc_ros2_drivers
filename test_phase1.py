#!/usr/bin/env python3
"""
test_phase1.py — test the full Phase 1 sequence without Modbus.

Sequence:
  1. Pick die from PICK_DOWN
  2. Move to CAMERA_POSE — check front face (camera or manual)
  3. If pip 1 on front face → go to CONV_REAR_JNT (joint drop) → open gripper
  4. If not → release at PICK_DOWN, repick, repeat from step 2
  5. Retreat to CONV_REAR_ABV → run belt for RUN_SECONDS → stop

No Modbus / no Bunsen handshake — pure motion test.

Usage:
    python3 test_phase1.py

Requires FANUC driver running:
    ros2 launch launch/start.launch.py robot_name:=Beaker robot_ip:=10.8.4.16

Also requires camera_server running (optional — falls back to manual input):
    ros2 launch dice_game robot1.launch.py robot_name:=Beaker robot_ip:=10.8.4.16 modbus_ip:=0.0.0.0
"""

import sys
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from fanuc_interfaces.action import CartPose, JointPose, SchunkGripper, Conveyor
from fanuc_interfaces.srv import CaptureImage
from dice_game.pip_counter import count_pips, save_debug_image

# ── Positions — must match robot1_controller.py ───────────────────────────────
ROBOT_NAME  = 'Beaker'
RUN_SECONDS = 9.9

HOME_JOINTS   = (1.1, 1.5, -2.0, -1.7, -88.6, -30.0)
PICK_ABOVE    = dict(x=470.0,    y=-15.0,   z=-18.0,   w=179.9, p=0.0,   r=30.0)
PICK_DOWN     = dict(x=470.0,    y=-15.0,   z=-185.0,  w=179.9, p=0.0,   r=30.0)
REORIENT_ABOVE = dict(x=470.0,    y=-15.0,   z=-18.0,   w=179.9, p=0.0,   r=120.0)
REORIENT_DOWN  = dict(x=470.0,    y=-15.0,   z=-185.0,  w=179.9, p=0.0,   r=120.0)
CAMERA_POSE   = dict(x=490.0,    y=890.0,   z=881.0,   w=73.0,  p=-66.0, r=-170.0)
CONV_REAR_ABV  = dict(x=-194.112, y=617.369, z=200.840, w=179.9, p=0.0,   r=120.0)
# Joint pose that drops the die with the front-face pip facing up on the belt
CONV_REAR_JNT  = (102.382, 51.116, -128.327, 164.504, -126.179, -159.536)
# Joint pose that brings the top face toward the camera when pip is on top
TOP_FACE_JNT   = (-34.6, 56.534, -70.0, -55.8, -99.7, 170.5)

# Second camera view — joint angles that tilt the die so its top face points at camera
CAMERA_JOINT_2 = (50.731, 31.588, -14.992, 173.365, -103.358, 125.27)

# Wrist roll offsets (degrees) applied to CAMERA_POSE['r'] — 60° steps, 6 faces
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


class Phase1Test(Node):

    def __init__(self):
        super().__init__('phase1_test')
        self._cart    = ActionClient(self, CartPose,      f'/{ROBOT_NAME}/cartesian_pose')
        self._joint   = ActionClient(self, JointPose,     f'/{ROBOT_NAME}/joint_pose')
        self._gripper = ActionClient(self, SchunkGripper, f'/{ROBOT_NAME}/schunk_gripper')
        self._conv    = ActionClient(self, Conveyor,      f'/{ROBOT_NAME}/conveyor')
        self._cam     = self.create_client(CaptureImage,  '/dice_game/capture_image')

        self._camera_ok = False

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

    # ── Camera ────────────────────────────────────────────────────────────────

    def _capture(self, label: str) -> int:
        """Capture image and count pips. Returns pip count or 0 on failure."""
        time.sleep(0.4)
        fut = self._cam.call_async(CaptureImage.Request())
        rclpy.spin_until_future_complete(self, fut)
        resp = fut.result()
        if not resp.success:
            self.get_logger().error(f'Capture failed: {resp.message}')
            return 0
        arr  = np.array(resp.image_data, dtype=np.uint8)
        img  = arr.reshape((resp.height, resp.width, resp.channels))
        pips = count_pips(img)
        path = f'{DEBUG_IMG_DIR}/phase1_test_{label}.png'
        save_debug_image(img, pips, path)
        self.get_logger().info(f'Camera: {pips} pip(s) — saved {path}')
        return pips

    # ── Manual input helpers ──────────────────────────────────────────────────

    @staticmethod
    def _prompt_face(label: str) -> int:
        while True:
            try:
                val = int(input(f'  {label} [1-6]: ').strip())
                if 1 <= val <= 6:
                    return val
            except (ValueError, EOFError):
                pass
            print('    Enter a number from 1 to 6.')

    def _get_face(self, label: str) -> int:
        """Camera capture if available, else manual prompt."""
        if self._camera_ok:
            return self._capture(label)
        return self._prompt_face(label)

    # ── Find pip 1 ────────────────────────────────────────────────────────────

    def find_pip1(self) -> bool:
        """
        Chirality loop — repeats one physical move at a time until pip 1
        naturally lands on the front face. No wrist offsets used.

        Each iteration:
          1. Take VIEW 1 + VIEW 2 to know the full die layout via chirality.
          2a. Front face  → done, return True.
          2b. Top face    → HOME → TOP_FACE_JNT → HOME → repick → loop again.
          2c. Any other   → HOME → REORIENT release → HOME → repick → loop again.

        Goes home before every move to guarantee IK reachability.
        """
        attempt = 0
        while True:
            attempt += 1
            print(f'\n{"=" * 54}')
            print(f'  Finding pip 1 — attempt {attempt}')
            print(f'{"=" * 54}')

            # ── VIEW 1: front face ────────────────────────────────────────────
            self._send_cart(**CAMERA_POSE)
            print('\n  VIEW 1 — die at camera position.')
            front_pip = self._get_face('front')
            print(f'  Front face: {front_pip}')

            # ── VIEW 2: bottom face (CAMERA_JOINT_2 tilts so camera sees bottom)
            self._send_joint(*CAMERA_JOINT_2)
            print('\n  VIEW 2 — bottom face toward camera.')
            bottom_pip = self._get_face('bottom')
            top_pip = 7 - bottom_pip
            print(f'  Bottom: {bottom_pip}  Top: {top_pip}')
            self._send_cart(**CAMERA_POSE)

            # ── Chirality lookup ──────────────────────────────────────────────
            right_pip = _DIE_RIGHT.get((top_pip, front_pip))
            if right_pip is None:
                print(f'  ! ({top_pip},{front_pip}) invalid — reorienting and retrying.')
                self._send_joint(*HOME_JOINTS)
                self._send_cart(**PICK_ABOVE)
                self._send_cart(**REORIENT_ABOVE)
                self._send_cart(**REORIENT_DOWN)
                self._send_gripper('open')
                self._send_joint(*HOME_JOINTS)
                self._send_cart(**PICK_ABOVE)
                self._send_cart(**PICK_DOWN)
                self._send_gripper('close')
                self._send_cart(**PICK_ABOVE)
                continue

            back_pip = 7 - front_pip
            left_pip = 7 - right_pip
            print(f'  Layout: front={front_pip} right={right_pip} back={back_pip}'
                  f' left={left_pip} top={top_pip} bottom={bottom_pip}')

            face_map = {
                'front': front_pip, 'right': right_pip,
                'back':  back_pip,  'left':  left_pip,
                'top':   top_pip,   'bottom': bottom_pip,
            }
            target_face = next((f for f, v in face_map.items() if v == 1), None)
            print(f'  Pip 1 is on the {target_face} face.')

            # ── Front: done ───────────────────────────────────────────────────
            if target_face == 'front':
                print('  >> Pip 1 on front face — ready for drop.\n')
                return True

            # ── Top: flip via TOP_FACE_JNT ────────────────────────────────────
            if target_face == 'top':
                print('  Pip 1 on top → HOME → TOP_FACE_JNT → HOME → repick...')
                self._send_joint(*HOME_JOINTS)
                self._send_joint(*TOP_FACE_JNT)
                self._send_gripper('open')
                self._send_joint(*HOME_JOINTS)
                self._send_cart(**PICK_ABOVE)
                self._send_cart(**PICK_DOWN)
                self._send_gripper('close')
                self._send_cart(**PICK_ABOVE)
                continue   # re-check after flip

            # ── Bottom / right / back / left: REORIENT release ────────────────
            print(f'  Pip 1 on {target_face} → HOME → REORIENT → HOME → repick...')
            self._send_joint(*HOME_JOINTS)
            self._send_cart(**PICK_ABOVE)
            self._send_cart(**REORIENT_ABOVE)
            self._send_cart(**REORIENT_DOWN)
            self._send_gripper('open')
            self._send_joint(*HOME_JOINTS)
            self._send_cart(**PICK_ABOVE)
            self._send_cart(**PICK_DOWN)
            self._send_gripper('close')
            self._send_cart(**PICK_ABOVE)
            continue   # re-check after reorient

    def _scan_for_pip1(self) -> bool:
        """Fallback: step through all 6 wrist rotations looking for pip 1."""
        for i, r_offset in enumerate(CAMERA_ROTATION_STEPS):
            if r_offset != 0:
                self._send_cart(**{**CAMERA_POSE, 'r': CAMERA_POSE['r'] + r_offset})
            pips = self._get_face(f'scan_{i}')
            self.get_logger().info(f'  Scan pos {i + 1}/6  r={r_offset:+.0f}°: {pips} pip(s)')
            if pips == 1:
                print(f'  >> Pip 1 found at scan position {i + 1}!\n')
                return True
        print('  Pip 1 not found at any position.\n')
        return False

    # ── Main sequence ─────────────────────────────────────────────────────────

    def run(self):
        self.get_logger().info('Waiting for action servers...')
        if not self._cart.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Cartesian server not found.')
            return

        self._camera_ok = self._cam.wait_for_service(timeout_sec=5.0)
        if self._camera_ok:
            self.get_logger().info('Camera service found — using camera.')
        else:
            self.get_logger().warn('Camera service not found — using manual input.')

        # ── 1. Pick die ───────────────────────────────────────────────────────
        self.get_logger().info('Picking die...')
        self._send_gripper('open')
        self._send_cart(**PICK_ABOVE)
        self._send_cart(**PICK_DOWN)
        self._send_gripper('close')
        self._send_cart(**PICK_ABOVE)

        # ── 2. Find pip 1 (loops internally until positioned) ────────────────
        self.find_pip1()

        # ── 3. Final check then drop on rear conveyor ────────────────────────
        pips = self._get_face('pre_drop')
        self.get_logger().info(f'Pre-drop check: {pips} pip(s) facing camera.')

        self.get_logger().info('Moving above rear conveyor...')
        self._send_cart(**CONV_REAR_ABV)
        self.get_logger().info('Moving to conveyor drop position (pip-1 face up)...')
        self._send_joint(*CONV_REAR_JNT)
        self._send_gripper('open')
        self._send_cart(**CONV_REAR_ABV)

        self.get_logger().info('Done. Robot at CONV_REAR_ABV.')


def main():
    rclpy.init()
    node = Phase1Test()
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
