#!/usr/bin/env python3
"""
Robot 1 master node — runs on Beaker's machine.

Responsibilities:
  - Find pip 1 (start state), then find pips 3 and 5 in sequence
  - Own and run the FRONT conveyor (used for even pips coming back from Robot 2)
  - Signal Robot 2 to run the BACK conveyor when sending dice over (odd pips)
  - Track retry count and report final results
"""
import os
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from fanuc_interfaces.action import CartPose, JointPose, SchunkGripper, Conveyor
from fanuc_interfaces.srv import CaptureAndCount

from pymodbus.client import ModbusTcpClient

# ---------------------------------------------------------------------------
# Modbus register addresses  (must match modbus_server.py)
# ---------------------------------------------------------------------------
REG_PIP       = 0
REG_TARGET    = 1
REG_CONV_CMD  = 2
REG_R1_RETRY  = 3
REG_R2_RETRY  = 4
REG_GAME      = 5

CONV_IDLE        = 0
CONV_REQ_FRONT   = 1
CONV_REQ_BACK    = 2
CONV_READY       = 3
CONV_DICE_PLACED = 4
CONV_DICE_RECVD  = 5

GAME_FINDING_START = 0
GAME_RUNNING       = 1
GAME_COMPLETE      = 2

# ---------------------------------------------------------------------------
# Robot poses  — CALIBRATE all values before first run
# ---------------------------------------------------------------------------
HOME_JOINTS = dict(joint1=0.0, joint2=0.0, joint3=0.0,
                   joint4=0.0, joint5=-90.0, joint6=0.0)

# Initial dice position (in front of Robot 1)
PICK_ABOVE  = dict(x=470.0, y=-15.0, z=-18.0,  w=179.9, p=0.0, r=30.0)
PICK_DOWN   = dict(x=470.0, y=-15.0, z=-185.0, w=179.9, p=0.0, r=30.0)

# Overhead camera position
CAMERA_POSE = dict(x=490.0, y=890.0, z=881.0,  w=73.0, p=-66.0, r=-170.0)

# Back conveyor — Robot 1 places dice here to send to Robot 2
BACK_CONV_ABOVE = dict(x=0.0, y=0.0, z=0.0, w=0.0, p=0.0, r=0.0)   # CALIBRATE
BACK_CONV_PLACE = dict(x=0.0, y=0.0, z=0.0, w=0.0, p=0.0, r=0.0)   # CALIBRATE

# Front conveyor — Robot 1 picks dice here when Robot 2 sends it back
FRONT_CONV_ABOVE  = dict(x=0.0, y=0.0, z=0.0, w=0.0, p=0.0, r=0.0)  # CALIBRATE
FRONT_CONV_PICKUP = dict(x=0.0, y=0.0, z=0.0, w=0.0, p=0.0, r=0.0)  # CALIBRATE

CONVEYOR_RUN_SECONDS = 5.0   # how long to run the belt before stopping


class Robot1Master(Node):

    def __init__(self):
        super().__init__('robot1_master')

        ns = os.environ.get('ROBOT1_NAME', 'Beaker')
        mb_host = os.environ.get('MODBUS_HOST', '127.0.0.1')
        mb_port = int(os.environ.get('MODBUS_PORT', '5020'))

        self.cart_ac    = ActionClient(self, CartPose,      f'/{ns}/cartesian_pose')
        self.joint_ac   = ActionClient(self, JointPose,     f'/{ns}/joint_pose')
        self.gripper_ac = ActionClient(self, SchunkGripper, f'/{ns}/schunk_gripper')
        self.conveyor_ac = ActionClient(self, Conveyor,     f'/{ns}/conveyor')
        self.cam_client = self.create_client(CaptureAndCount, '/camera/capture_and_count')

        self.mb = ModbusTcpClient(mb_host, port=mb_port)
        if not self.mb.connect():
            raise RuntimeError(f'Cannot connect to Modbus server at {mb_host}:{mb_port}')
        self.get_logger().info(f'Modbus connected to {mb_host}:{mb_port}')

    # -----------------------------------------------------------------------
    # Modbus helpers
    # -----------------------------------------------------------------------

    def mb_read(self, addr: int) -> int:
        result = self.mb.read_holding_registers(addr, 1)
        return result.registers[0]

    def mb_write(self, addr: int, value: int):
        self.mb.write_register(addr, value)

    # -----------------------------------------------------------------------
    # Action helpers (blocking)
    # -----------------------------------------------------------------------

    def _send_cart(self, **kwargs) -> bool:
        self.cart_ac.wait_for_server()
        goal = CartPose.Goal()
        for k, v in kwargs.items():
            setattr(goal, k, float(v))
        future = self.cart_ac.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        gh = future.result()
        if not gh.accepted:
            self.get_logger().error('Cart goal rejected')
            return False
        res_fut = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut)
        return res_fut.result().result.success

    def _send_joint(self, **kwargs) -> bool:
        self.joint_ac.wait_for_server()
        goal = JointPose.Goal()
        for k, v in kwargs.items():
            setattr(goal, k, float(v))
        future = self.joint_ac.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        gh = future.result()
        if not gh.accepted:
            self.get_logger().error('Joint goal rejected')
            return False
        res_fut = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut)
        return res_fut.result().result.success

    def _send_gripper(self, command: str) -> bool:
        self.gripper_ac.wait_for_server()
        goal = SchunkGripper.Goal()
        goal.command = command
        future = self.gripper_ac.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        gh = future.result()
        if not gh.accepted:
            self.get_logger().error(f'Gripper goal rejected: {command}')
            return False
        res_fut = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut)
        return res_fut.result().result.success

    def _run_conveyor(self, command: str) -> bool:
        self.conveyor_ac.wait_for_server()
        goal = Conveyor.Goal()
        goal.command = command
        future = self.conveyor_ac.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        gh = future.result()
        if not gh.accepted:
            return False
        res_fut = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut)
        return res_fut.result().result.success

    # -----------------------------------------------------------------------
    # Camera service call
    # -----------------------------------------------------------------------

    def capture_and_count(self) -> int:
        self.cam_client.wait_for_service()
        req = CaptureAndCount.Request()
        future = self.cam_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        resp = future.result()
        if not resp.success:
            self.get_logger().warn('Camera capture failed')
            return 0
        return resp.pip_count

    # -----------------------------------------------------------------------
    # Robot motion sequences
    # -----------------------------------------------------------------------

    def go_home(self):
        self.get_logger().info('Going home...')
        self._send_joint(**HOME_JOINTS)

    def pick_dice(self):
        """Pick dice from fixed starting position."""
        self._send_gripper('open')
        self._send_cart(**PICK_ABOVE)
        self._send_cart(**PICK_DOWN)
        self._send_gripper('close')
        self._send_cart(**PICK_ABOVE)

    def drop_dice(self):
        """Drop dice back at pick position (retry)."""
        self._send_cart(**PICK_ABOVE)
        self._send_cart(**PICK_DOWN)
        self._send_gripper('open')
        self._send_cart(**PICK_ABOVE)

    def present_to_camera(self):
        self._send_cart(**CAMERA_POSE)
        time.sleep(0.3)

    def pick_from_front_conveyor(self):
        """Pick dice that Robot 2 sent back via front conveyor."""
        self._send_gripper('open')
        self._send_cart(**FRONT_CONV_ABOVE)
        self._send_cart(**FRONT_CONV_PICKUP)
        self._send_gripper('close')
        self._send_cart(**FRONT_CONV_ABOVE)

    def place_on_back_conveyor(self):
        """Place dice onto the back conveyor to send to Robot 2."""
        self._send_cart(**BACK_CONV_ABOVE)
        self._send_cart(**BACK_CONV_PLACE)
        self._send_gripper('open')
        self._send_cart(**BACK_CONV_ABOVE)

    # -----------------------------------------------------------------------
    # Conveyor coordination helpers
    # -----------------------------------------------------------------------

    def _wait_conv_cmd(self, target: int, timeout: float = 30.0):
        """Poll CONV_CMD register until it equals target."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self.mb_read(REG_CONV_CMD) == target:
                return True
            time.sleep(0.2)
        raise TimeoutError(f'Timed out waiting for CONV_CMD={target}')

    def send_dice_via_back_conveyor(self):
        """
        Coordinate with Robot 2 to run the back conveyor and transfer dice.
        Robot 2 owns the back conveyor.
        """
        self.get_logger().info('Requesting back conveyor from Robot 2...')
        self.mb_write(REG_CONV_CMD, CONV_REQ_BACK)

        # Wait for Robot 2 to start its conveyor
        self._wait_conv_cmd(CONV_READY)
        self.get_logger().info('Back conveyor ready — placing dice...')

        self.place_on_back_conveyor()
        self.mb_write(REG_CONV_CMD, CONV_DICE_PLACED)

        # Wait for Robot 2 to confirm receipt
        self._wait_conv_cmd(CONV_DICE_RECVD)
        self.mb_write(REG_CONV_CMD, CONV_IDLE)
        self.get_logger().info('Dice received by Robot 2.')

    def receive_dice_via_front_conveyor(self):
        """
        Robot 2 requests front conveyor — run it, wait for dice, pick up.
        Robot 1 owns the front conveyor.
        """
        self.get_logger().info('Robot 2 requesting front conveyor — starting belt...')
        self._run_conveyor('forward')
        self.mb_write(REG_CONV_CMD, CONV_READY)

        # Wait for Robot 2 to place dice
        self._wait_conv_cmd(CONV_DICE_PLACED)
        self.get_logger().info('Dice placed on front conveyor — waiting for travel...')
        time.sleep(CONVEYOR_RUN_SECONDS)
        self._run_conveyor('stop')

        self.pick_from_front_conveyor()
        self.mb_write(REG_CONV_CMD, CONV_DICE_RECVD)
        self._wait_conv_cmd(CONV_IDLE)
        self.get_logger().info('Dice retrieved from front conveyor.')

    # -----------------------------------------------------------------------
    # Main game loop
    # -----------------------------------------------------------------------

    def _retry_until_pip(self, target: int, retry_reg: int,
                         pick_fn, present_fn) -> int:
        """
        Repeatedly pick → present → count until pip == target.
        Returns number of retries needed (0 = correct on first try).
        """
        attempts = 0
        while True:
            pick_fn()
            present_fn()
            pip = self.capture_and_count()
            self.mb_write(REG_PIP, pip)
            self.get_logger().info(f'Pip count: {pip}  (target: {target})')

            if pip == target:
                return attempts

            attempts += 1
            current = self.mb_read(retry_reg) + 1
            self.mb_write(retry_reg, current)
            self.get_logger().info(f'Wrong pip — retry #{current}')
            self.drop_dice()

    def run(self):
        self.get_logger().info('Robot 1 starting — waiting for action servers...')
        self.cart_ac.wait_for_server()
        self.gripper_ac.wait_for_server()
        self.get_logger().info('Ready.')

        self.go_home()

        # ------------------------------------------------------------------
        # Phase 1: find pip 1 (start state)
        # ------------------------------------------------------------------
        self.get_logger().info('=== PHASE 1: Searching for pip 1 (start) ===')
        self._retry_until_pip(1, REG_R1_RETRY, self.pick_dice, self.present_to_camera)
        self.get_logger().info('Pip 1 found — game started!')
        self.mb_write(REG_GAME, GAME_RUNNING)
        self.mb_write(REG_TARGET, 2)

        # pip 1 is odd → back conveyor
        self.send_dice_via_back_conveyor()

        # ------------------------------------------------------------------
        # Phase 2: find pips 3 and 5 (Robot 1 handles odd pips)
        # ------------------------------------------------------------------
        for target in [3, 5]:
            # Wait for Robot 2 to request the front conveyor (sending dice back)
            self.get_logger().info(f'Waiting for Robot 2 to send dice back (front conveyor)...')
            self._wait_conv_cmd(CONV_REQ_FRONT)
            self.receive_dice_via_front_conveyor()

            self.get_logger().info(f'=== Searching for pip {target} ===')

            def _pick():
                self.present_to_camera()

            self._retry_until_pip(
                target, REG_R1_RETRY,
                lambda: None,           # dice is already in hand after conveyor pick
                self.present_to_camera
            )
            self.get_logger().info(f'Pip {target} found!')
            self.mb_write(REG_TARGET, target + 1)

            # 3 and 5 are odd → back conveyor
            self.send_dice_via_back_conveyor()

        # ------------------------------------------------------------------
        # Wait for Robot 2 to complete (pip 6)
        # ------------------------------------------------------------------
        self.get_logger().info('Waiting for Robot 2 to finish...')
        while self.mb_read(REG_GAME) != GAME_COMPLETE:
            time.sleep(0.5)

        self.go_home()
        self._print_results()

    def _print_results(self):
        r1 = self.mb_read(REG_R1_RETRY)
        r2 = self.mb_read(REG_R2_RETRY)
        print('\n' + '=' * 50)
        print('           FINAL RESULTS')
        print('=' * 50)
        print(f'  Robot 1 retries : {r1}')
        print(f'  Robot 2 retries : {r2}')
        print(f'  Combined retries: {r1 + r2}')
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
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
