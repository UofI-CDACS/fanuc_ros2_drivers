#!/usr/bin/env python3
"""
Robot 1 master node — runs on Beaker's machine.

Beaker picks the die, rotates it at the camera to find the target pip, then
ships it to Bunsen via the rear conveyor.  Bunsen verifies orientation and
sends it back via the front conveyor.  Repeat until pip 6 — Bunsen keeps that
one and places it.

Positions are from Beaker's calibration (partner).
Conveyor coordination uses the Modbus CONV_CMD register (modbus_server.py).
"""

import os
import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from fanuc_interfaces.action import CartPose, JointPose, SchunkGripper, Conveyor
from fanuc_interfaces.srv import CaptureAndCount

from pymodbus.client import ModbusTcpClient

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from modbus_server import (
    REG_PIP_PROGRESS, REG_CONV_CMD, REG_RETRIES,
    COIL_READY, COIL_CAMERA_CLIENT,
    CONV_IDLE, CONV_BEAKER_WANTS_SEND, CONV_REAR_RUNNING,
    CONV_DIE_ON_REAR, CONV_BUNSEN_HAS_DIE,
    CONV_BUNSEN_WANTS_SEND, CONV_FRONT_RUNNING,
    CONV_DIE_ON_FRONT, CONV_BEAKER_HAS_DIE,
)

POLL_INTERVAL             = 0.2
CONV_TIMEOUT              = 60.0
REAR_CONVEYOR_TRAVEL_SECS = 5.0   # tune: time for die to travel rear belt to Bunsen
CONVEYOR_TRAVEL_SECS      = 5.0   # tune: time for die to travel front belt to Beaker

# ── Beaker positions (calibrated by partner) ──────────────────────────────────
HOME_JOINTS = (1.1, 1.5, -2.0, -1.7, -88.6, -30.0)

PICK_ABOVE    = dict(x=470.0,    y=-15.0,   z=-18.0,   w=179.9, p=0.0, r=30.0)
PICK_DOWN     = dict(x=470.0,    y=-15.0,   z=-185.0,  w=179.9, p=0.0, r=30.0)
CAMERA_POSE   = dict(x=490.0,    y=890.0,   z=881.0,   w=73.0,  p=-66.0, r=-170.0)
CONV_REAR_ABV = dict(x=470.0, y=-15.0, z=-18.0,  w=179.9, p=0.0, r=120.0)
CONV_REAR_DRP = dict(x=470.0, y=-15.0, z=-185.0, w=179.9, p=0.0, r=120.0)

# Front conveyor — Bunsen sends die back here after verifying pip
CONV_FRNT_ABV = dict(x=0.0, y=0.0, z=0.0, w=0.0, p=0.0, r=0.0)  # CALIBRATE
CONV_FRNT_DWN = dict(x=0.0, y=0.0, z=0.0, w=0.0, p=0.0, r=0.0)  # CALIBRATE

# 60° wrist rotation steps — covers all 6 die faces in one sweep
CAMERA_ROTATION_STEPS = [0, 60, 120, 180, -120, -60]


class Robot1Master(Node):

    def __init__(self):
        super().__init__('robot1_master')

        ns       = os.environ.get('ROBOT1_NAME', 'Beaker')
        mb_host  = os.environ.get('MODBUS_HOST',  '127.0.0.1')
        mb_port  = int(os.environ.get('MODBUS_PORT', '5020'))

        self._cart    = ActionClient(self, CartPose,      f'/{ns}/cartesian_pose')
        self._joint   = ActionClient(self, JointPose,     f'/{ns}/joint_pose')
        self._gripper = ActionClient(self, SchunkGripper, f'/{ns}/schunk_gripper')
        self._conv    = ActionClient(self, Conveyor,      f'/{ns}/conveyor')
        self._cam     = self.create_client(CaptureAndCount, '/camera/capture_and_count')

        self.mb = ModbusTcpClient(mb_host, port=mb_port)
        if not self.mb.connect():
            raise RuntimeError(f'Cannot connect to Modbus at {mb_host}:{mb_port}')
        self.get_logger().info(f'Modbus connected to {mb_host}:{mb_port}')

        self.r1_retries = 0
        self._camera_ok = False

    # ── Modbus helpers ────────────────────────────────────────────────────────

    def _mb_read(self, addr):
        return self.mb.read_holding_registers(addr, 1).registers[0]

    def _mb_write(self, addr, val):
        self.mb.write_register(addr, val)

    def _mb_write_coil(self, addr, val):
        self.mb.write_coil(addr, bool(val))

    def _wait_conv(self, target, timeout=CONV_TIMEOUT):
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self._mb_read(REG_CONV_CMD) == target:
                return True
            time.sleep(POLL_INTERVAL)
        return False

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

    def _send_gripper(self, command) -> bool:
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

    def _run_conveyor(self, command) -> bool:
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

    def _capture(self) -> int:
        """Call CaptureAndCount service; returns pip count or 0 on failure."""
        if not self._camera_ok:
            time.sleep(0.4)
            return -1
        time.sleep(0.4)
        fut = self._cam.call_async(CaptureAndCount.Request())
        rclpy.spin_until_future_complete(self, fut)
        resp = fut.result()
        if not resp.success:
            self.get_logger().error('Camera capture failed')
            return 0
        return resp.pip_count

    def _find_pip_rotating(self, target: int) -> int:
        """
        Move to camera pose and step through 6 wrist rotations (60° each),
        capturing at each position.  Returns pip count when target is found,
        or 0 if not found on any face.  Robot stays at the matching rotation.
        If camera is unavailable, steps through all rotations and returns target
        so the game keeps moving.
        """
        self.get_logger().info(f'Scanning for pip={target} (wrist rotation sweep)')
        self._send_cart(**CAMERA_POSE)

        if not self._camera_ok:
            self.get_logger().warn('No camera — stepping all rotations and assuming target found')
            for i, r_offset in enumerate(CAMERA_ROTATION_STEPS):
                if r_offset != 0:
                    self._send_cart(**{**CAMERA_POSE, 'r': CAMERA_POSE['r'] + r_offset})
                time.sleep(0.4)
            return target

        for i, r_offset in enumerate(CAMERA_ROTATION_STEPS):
            if r_offset != 0:
                self._send_cart(**{**CAMERA_POSE, 'r': CAMERA_POSE['r'] + r_offset})
            pips = self._capture()
            self.get_logger().info(f'  r_offset={r_offset:+.0f}°  →  {pips} pip(s)')
            if pips == target:
                return pips

        return 0

    # ── Robot motion sequences ────────────────────────────────────────────────

    def go_home(self):
        self.get_logger().info('Going home...')
        self._send_joint(*HOME_JOINTS)

    def pick_die(self):
        self._send_gripper('open')
        self._send_cart(**PICK_ABOVE)
        self._send_cart(**PICK_DOWN)
        self._send_gripper('close')
        self._send_cart(**PICK_ABOVE)

    def put_die_down(self):
        """Return die to pick spot (re-orient on re-pick)."""
        self._send_cart(**PICK_ABOVE)
        self._send_cart(**PICK_DOWN)
        self._send_gripper('open')
        self._send_cart(**PICK_ABOVE)

    # ── Conveyor coordination ─────────────────────────────────────────────────

    def send_to_bunsen(self):
        """
        Place die on rear conveyor and hand off to Bunsen.
        Sequence: BEAKER_WANTS_SEND → wait REAR_RUNNING → place → DIE_ON_REAR
                  (set camera token) → wait BUNSEN_HAS_DIE → IDLE
        """
        self.get_logger().info('Sending die to Bunsen via rear conveyor...')
        self._mb_write(REG_CONV_CMD, CONV_BEAKER_WANTS_SEND)

        if not self._wait_conv(CONV_REAR_RUNNING):
            self.get_logger().error('Timeout waiting for Bunsen to start rear conveyor')
            self._mb_write(REG_CONV_CMD, CONV_IDLE)
            return False

        self._send_cart(**CONV_REAR_ABV)
        self._send_cart(**CONV_REAR_DRP)
        self._send_gripper('open')
        self._send_cart(**CONV_REAR_ABV)

        # Run rear belt to deliver die to Bunsen, then signal die is on belt
        self._run_conveyor('forward')
        self.get_logger().info(f'Rear belt running for {REAR_CONVEYOR_TRAVEL_SECS}s...')
        time.sleep(REAR_CONVEYOR_TRAVEL_SECS)
        self._run_conveyor('stop')

        # Hand camera token to Bunsen so it can read pip count
        self._mb_write(REG_CONV_CMD, CONV_DIE_ON_REAR)
        self._mb_write_coil(COIL_CAMERA_CLIENT, True)

        if not self._wait_conv(CONV_BUNSEN_HAS_DIE):
            self.get_logger().error('Timeout waiting for Bunsen to confirm pickup')
            return False

        self._mb_write(REG_CONV_CMD, CONV_IDLE)
        return True

    def receive_from_bunsen(self):
        """
        Bunsen sends die back via front conveyor (Beaker-owned).
        Sequence: wait BUNSEN_WANTS_SEND → start front belt → FRONT_RUNNING
                  → wait DIE_ON_FRONT → travel → stop → pick → BEAKER_HAS_DIE
                  → wait IDLE
        """
        self.get_logger().info('Waiting for Bunsen to send die back...')
        if not self._wait_conv(CONV_BUNSEN_WANTS_SEND):
            self.get_logger().error('Timeout waiting for BUNSEN_WANTS_SEND')
            return False

        # Signal Bunsen to start the front belt — Bunsen controls that conveyor
        self._mb_write(REG_CONV_CMD, CONV_FRONT_RUNNING)

        if not self._wait_conv(CONV_DIE_ON_FRONT):
            self.get_logger().error('Timeout waiting for die on front conveyor')
            return False

        # Wait for die to travel to pickup point (Bunsen runs the belt)
        time.sleep(CONVEYOR_TRAVEL_SECS)

        # Take camera token back before picking (Beaker needs camera for next round)
        self._mb_write_coil(COIL_CAMERA_CLIENT, False)

        self._send_gripper('open')
        self._send_cart(**CONV_FRNT_ABV)
        self._send_cart(**CONV_FRNT_DWN)
        self._send_gripper('close')
        self._send_cart(**CONV_FRNT_ABV)

        self._mb_write(REG_CONV_CMD, CONV_BEAKER_HAS_DIE)
        self._wait_conv(CONV_IDLE, timeout=10.0)
        return True

    # ── Main game loop ────────────────────────────────────────────────────────

    def run(self):
        self.get_logger().info('Waiting for action servers...')
        self._cart.wait_for_server()
        self._joint.wait_for_server()
        self._gripper.wait_for_server()
        self._conv.wait_for_server()
        self._camera_ok = self._cam.wait_for_service(timeout_sec=5.0)
        if self._camera_ok:
            self.get_logger().info('Camera service found.')
        else:
            self.get_logger().warn('Camera service not found — running without camera (pip counts skipped)')
        self.get_logger().info('Ready.')

        self.go_home()

        # ── Phase 1: find pip 1 ───────────────────────────────────────────────
        self.get_logger().info('=== PHASE 1: searching for pip 1 ===')
        while True:
            self.pick_die()
            pips = self._find_pip_rotating(1)
            if pips == 1:
                self.get_logger().info('Pip 1 found — START confirmed!')
                break
            self.get_logger().info('Pip 1 not visible on any face — re-picking')
            self.put_die_down()
            self.r1_retries += 1

        self._mb_write(REG_PIP_PROGRESS, 1)
        self.send_to_bunsen()

        # ── Phase 2: pips 2 → 6 ──────────────────────────────────────────────
        self.get_logger().info('=== PHASE 2: sequential 2 → 6 ===')
        for target in range(2, 7):
            self.get_logger().info(f'--- Target pip: {target} ---')
            self._mb_write(REG_PIP_PROGRESS, target)

            # Receive die back from Bunsen (except first iteration — already have it)
            self.receive_from_bunsen()

            # Find target pip by wrist rotation
            while True:
                pips = self._find_pip_rotating(target)
                if pips == target:
                    self.get_logger().info(f'Pip {target} found!')
                    break
                self.get_logger().info(f'Pip {target} not visible — re-picking')
                self.put_die_down()
                self.pick_die()
                self.r1_retries += 1

            if target == 6:
                # Bunsen keeps pip 6 and places it — just send and we're done
                self.send_to_bunsen()
                break

            self.send_to_bunsen()

        # ── Finish ────────────────────────────────────────────────────────────
        self.get_logger().info('All pips delivered. Going home.')
        self.go_home()
        self._print_results()

    def _print_results(self):
        retries = self._mb_read(REG_RETRIES)
        print('\n' + '=' * 50)
        print('         DICE GAME  —  BEAKER RESULTS')
        print('=' * 50)
        print(f'  Beaker (R1) retries : {self.r1_retries}')
        print(f'  Bunsen (R2) retries : {retries}')
        print(f'  Combined            : {self.r1_retries + retries}')
        print('=' * 50 + '\n')

    def destroy_node(self):
        self.mb.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Robot1Master()
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
