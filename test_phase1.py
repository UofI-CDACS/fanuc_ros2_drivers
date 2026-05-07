#!/usr/bin/env python3
"""
test_phase1.py — test the full Phase 1 sequence without Modbus.

Sequence:
  1. Pick die from PICK_DOWN
  2. Move to CAMERA_POSE — camera captures front face (or you type it)
  3. Move to CAMERA_JOINT_2 — camera captures top face (or you type it)
  4. Return to CAMERA_POSE — chirality table locates pip 1
  5. Rotate wrist to pip-1 position — camera or manual confirms
  6. Move to CONV_REAR_ABV → CONV_REAR_DRP — open gripper
  7. Lift back to CONV_REAR_ABV — run belt for RUN_SECONDS — stop

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

PICK_ABOVE    = dict(x=470.0,    y=-15.0,   z=-18.0,   w=179.9, p=0.0,   r=30.0)
PICK_DOWN     = dict(x=470.0,    y=-15.0,   z=-185.0,  w=179.9, p=0.0,   r=30.0)
CAMERA_POSE   = dict(x=490.0,    y=890.0,   z=881.0,   w=73.0,  p=-66.0, r=-170.0)
CAMERA_JOINT_2 = (50.731, 31.588, -14.992, 173.365, -103.358, -125.27)
CONV_REAR_ABV = dict(x=-194.112, y=617.369, z=200.840, w=179.9, p=0.0,   r=120.0)
CONV_REAR_DRP = dict(x=-194.112, y=617.369, z=8.840,   w=179.9, p=0.0,   r=120.0)

CAMERA_ROTATION_STEPS = [0, 60, 120, 180, -120, -60]
DEBUG_IMG_DIR = '/home/colin/Desktop'

_DIE_RIGHT = {
    (1, 2): 3, (1, 3): 5, (1, 5): 4, (1, 4): 2,
    (2, 6): 3, (2, 3): 1, (2, 1): 4, (2, 4): 6,
    (3, 2): 6, (3, 6): 5, (3, 5): 1, (3, 1): 2,
    (4, 2): 1, (4, 1): 5, (4, 5): 6, (4, 6): 2,
    (5, 1): 3, (5, 3): 6, (5, 6): 4, (5, 4): 1,
    (6, 5): 3, (6, 3): 2, (6, 2): 4, (6, 4): 5,
}
_FACE_STEP = {'front': 0, 'right': 2, 'back': 3, 'left': 5}
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

    @staticmethod
    def _prompt_pip(step: int, r_offset: float) -> int:
        while True:
            try:
                val = int(input(
                    f'\n  Position {step + 1}/6  (wrist {r_offset:+.0f}°)'
                    f'  —  pip count you see [1-6]: '
                ).strip())
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
        Two-view chirality approach:
          VIEW 1 at CAMERA_POSE      → front face pip
          VIEW 2 at CAMERA_JOINT_2   → top face pip
          Chirality table            → compute full layout
          Best rotation step         → confirm pip 1

        Returns True if pip 1 is found and robot is positioned at that rotation.
        Returns False if not found (caller should reorient and retry).
        """
        print(f'\n{"=" * 54}')
        print('  Finding pip 1')
        print(f'{"=" * 54}')

        # ── View 1: front face ────────────────────────────────────────────────
        self.get_logger().info('Moving to camera position (VIEW 1 — front face)...')
        self._send_cart(**CAMERA_POSE)
        print('\n  VIEW 1  —  die at camera position.')
        if not self._camera_ok:
            front_pip = self._prompt_face('Front face (what you see facing the camera)')
        else:
            front_pip = self._get_face('front')
            print(f'  Camera reads front face: {front_pip}')

        # ── View 2: top face ──────────────────────────────────────────────────
        self.get_logger().info('Moving to second view (VIEW 2 — top face)...')
        self._send_joint(*CAMERA_JOINT_2)
        print('\n  VIEW 2  —  J5 rotated, table-bottom face toward camera.')
        if not self._camera_ok:
            top_pip = self._prompt_face('Top face  (what you see now)              ')
        else:
            top_pip = self._get_face('top')
            print(f'  Camera reads top face: {top_pip}')

        # Return to camera base for rotation sweep
        self._send_cart(**CAMERA_POSE)

        # ── Chirality lookup ──────────────────────────────────────────────────
        right_pip = _DIE_RIGHT.get((top_pip, front_pip))
        if right_pip is None:
            print(f'\n  ! ({top_pip}, {front_pip}) is not a valid standard-die combination.')
            print('  Falling back to step-by-step scan...')
            return self._scan_for_pip1()

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
        target_face = next((f for f, v in face_map.items() if v == 1), None)
        print(f'  Pip 1 is on the {target_face} face.')

        if target_face in ('top', 'bottom'):
            print('  Cannot reach that face by wrist rotation — re-pick needed.')
            return False

        # ── Rotate to pip-1 position and confirm ─────────────────────────────
        best = _FACE_STEP[target_face]
        step_order = [best] + [i for i in range(len(CAMERA_ROTATION_STEPS)) if i != best]

        for i in step_order:
            r_offset = CAMERA_ROTATION_STEPS[i]
            if r_offset != 0:
                self._send_cart(**{**CAMERA_POSE, 'r': CAMERA_POSE['r'] + r_offset})

            if self._camera_ok:
                pips = self._capture(f'rot{i}')
            else:
                pips = self._prompt_pip(i, r_offset)

            self.get_logger().info(f'  Position {i + 1}/6  r={r_offset:+.0f}°: {pips} pip(s)')

            if pips == 1:
                print(f'  >> Pip 1 confirmed at position {i + 1}!\n')
                return True

        print('  Pip 1 not found at any rotation.\n')
        return False

    def _scan_for_pip1(self) -> bool:
        """Fallback: step through all 6 positions in order."""
        for i, r_offset in enumerate(CAMERA_ROTATION_STEPS):
            if r_offset != 0:
                self._send_cart(**{**CAMERA_POSE, 'r': CAMERA_POSE['r'] + r_offset})
            pips = self._prompt_pip(i, r_offset)
            if pips == 1:
                print(f'  >> Pip 1 found at position {i + 1}!\n')
                return True
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

        # ── 2. Find pip 1 (retry with reorient if needed) ────────────────────
        found = False
        retries = 0
        while not found:
            found = self.find_pip1()
            if not found:
                retries += 1
                print(f'  Re-orienting die (attempt {retries})...')
                # Drop at 90° offset, pick back up
                self._send_cart(**dict(x=470.0, y=-15.0, z=-18.0,  w=179.9, p=0.0, r=120.0))
                self._send_cart(**dict(x=470.0, y=-15.0, z=-185.0, w=179.9, p=0.0, r=120.0))
                self._send_gripper('open')
                self._send_cart(**dict(x=470.0, y=-15.0, z=-18.0,  w=179.9, p=0.0, r=120.0))
                self._send_cart(**PICK_ABOVE)
                self._send_cart(**PICK_DOWN)
                self._send_gripper('close')
                self._send_cart(**PICK_ABOVE)

        # ── 3. Drop on rear conveyor and run belt ─────────────────────────────
        self.get_logger().info('Moving to rear conveyor drop position...')
        self._send_cart(**CONV_REAR_ABV)
        self._send_cart(**CONV_REAR_DRP)
        self._send_gripper('open')
        self._send_cart(**CONV_REAR_ABV)

        self.get_logger().info(f'Running rear belt for {RUN_SECONDS}s...')
        self._send_conveyor('forward')
        time.sleep(RUN_SECONDS)
        self._send_conveyor('stop')

        self.get_logger().info(
            f'Done. Robot at CONV_REAR_ABV. Retries: {retries}.'
        )


def main():
    rclpy.init()
    node = Phase1Test()
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted — stopping conveyor.')
        node._send_conveyor('stop')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
