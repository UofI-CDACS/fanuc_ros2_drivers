"""
robot1_controller.py  —  Beaker (Robot 1)

State machine:
  SEARCHING_START  Pick dice, count pips, retry until pip == 1.
  SEQUENTIAL       For target 1-6: pick, count, retry until correct pip,
                   then deliver to Robot 2 via the appropriate conveyor.
  DONE             Print final stats and return home.

Inter-robot communication (all topics under /dice_game/):
  Publishes:
    pip_count        (Int32)   — pip just counted by Robot 1
    conveyor_select  (String)  — "front" (even) or "back" (odd)
    dice_ready       (Bool)    — True when die is placed on conveyor
  Subscribes:
    dice_returned    (Bool)    — Robot 2 signals die is back at pickup
"""

import time
from enum import IntEnum

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from fanuc_interfaces.action import CartPose, JointPose, SchunkGripper
from fanuc_interfaces.srv import CaptureImage
from std_msgs.msg import Int32, String, Bool
from pymodbus.client import ModbusTcpClient

from dice_game.pip_counter import count_pips, save_debug_image

# ── Robot 1 positions (Beaker) ────────────────────────────────────────────────
# Cartesian: x, y, z in mm  |  w, p, r in degrees
HOME_JOINTS   = (1.1, 1.5, -2.0, -1.7, -88.6, -30.0)
PICK_ABOVE    = dict(x=470.0, y=-15.0,  z=-18.0,  w=179.9, p=0.0,   r=30.0)
PICK_DOWN     = dict(x=470.0, y=-15.0,  z=-185.0, w=179.9, p=0.0,   r=30.0)
CAMERA_POSE   = dict(x=490.0, y=890.0,  z=881.0,  w=73.0,  p=-66.0, r=-170.0)
CONVEYOR_DROP = dict(x=470.0, y=-15.0,  z=-185.0, w=179.9, p=0.0,   r=120.0)
CONVEYOR_ABV  = dict(x=470.0, y=-15.0,  z=-18.0,  w=179.9, p=0.0,   r=120.0)

# Wrist roll offsets (degrees) applied to CAMERA_POSE['r'] to expose all die faces.
# 60° steps give 6 positions covering a full rotation.
CAMERA_ROTATION_STEPS = [0, 60, 120, 180, -120, -60]

DEBUG_IMG_DIR = '/home/colin/Desktop'

# ── Modbus ────────────────────────────────────────────────────────────────────
MODBUS_PORT       = 502
MODBUS_REG_STATE  = 0   # holding register 0: current state (1–9)
MODBUS_REG_PIP    = 1   # holding register 1: pip count (0–6)
MODBUS_COIL_READY = 0   # coil 0: die ready on conveyor


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

        # ── Camera service client ─────────────────────────────────────────────
        self._cam_cli = self.create_client(CaptureImage, '/dice_game/capture_image')

        # ── Inter-robot communication ─────────────────────────────────────────
        self._pub_pip      = self.create_publisher(Int32,  '/dice_game/pip_count',       10)
        self._pub_conveyor = self.create_publisher(String, '/dice_game/conveyor_select',  10)
        self._pub_ready    = self.create_publisher(Bool,   '/dice_game/dice_ready',       10)

        self._dice_returned = False
        self.create_subscription(Bool, '/dice_game/dice_returned', self._on_dice_returned, 10)

        # ── Retry counters ────────────────────────────────────────────────────
        self.r1_retries = 0   # failed picks (wrong pip)
        self._total_counts = []  # (target, actual, retries_for_that_round)

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _on_dice_returned(self, msg: Bool):
        if msg.data:
            self._dice_returned = True

    # ── Modbus helpers ────────────────────────────────────────────────────────

    def _modbus_connect(self, ip: str):
        if ip == '0.0.0.0':
            self.get_logger().warn('modbus_ip not set — Modbus state reporting disabled')
            return
        self._mb = ModbusTcpClient(host=ip, port=MODBUS_PORT)
        if self._mb.connect():
            self.get_logger().info(f'Modbus connected to {ip}:{MODBUS_PORT}')
        else:
            self.get_logger().warn(f'Modbus connection to {ip} failed — state reporting disabled')
            self._mb = None

    def _set_state(self, state: State):
        self.get_logger().info(f'State → {state.name}')
        if self._mb:
            self._mb.write_register(MODBUS_REG_STATE, int(state))

    def _set_pip(self, pips: int):
        if self._mb:
            self._mb.write_register(MODBUS_REG_PIP, pips)

    def _set_ready(self, ready: bool):
        if self._mb:
            self._mb.write_coil(MODBUS_COIL_READY, ready)

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

    # ── Camera ────────────────────────────────────────────────────────────────

    def _capture_count_at(self, label: str) -> int:
        """Capture image at current position and count pips (no movement)."""
        time.sleep(0.4)
        self._cam_cli.wait_for_service()
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

    def _capture_and_count(self, label: str) -> int:
        """Move to camera pose, capture, count pips, return count (0 = no face found)."""
        self.get_logger().info('Moving to camera position...')
        self._send_cart(**CAMERA_POSE)
        return self._capture_count_at(label)

    def _find_pip_rotating(self, target: int, label: str) -> int:
        """
        Move to camera pose and rotate the wrist in 60° steps (6 positions = full turn)
        looking for target pip count. Returns the pip count when found, or 0 if the
        target face is not seen in any orientation. Robot stays at the matching rotation
        so the die is correctly oriented when this returns.
        """
        self.get_logger().info(f'Scanning for pip={target} by rotating wrist...')
        self._send_cart(**CAMERA_POSE)

        for i, r_offset in enumerate(CAMERA_ROTATION_STEPS):
            self._set_state(State.PIP_COUNT if i == 0 else State.ROTATE_PIP)

            if r_offset != 0:
                self._send_cart(**{**CAMERA_POSE, 'r': CAMERA_POSE['r'] + r_offset})

            pips = self._capture_count_at(f'{label}_rot{i}')
            self._set_pip(pips)
            self.get_logger().info(f'  r_offset={r_offset:+.0f}°: {pips} pip(s)')

            if pips == target:
                return pips

        return 0

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

    def put_dice_down(self):
        """Return dice to pickup spot without gripping off — lets die re-orient."""
        self._send_cart(**PICK_ABOVE)
        self._send_cart(**PICK_DOWN)
        self._send_gripper('open')
        self._send_cart(**PICK_ABOVE)

    def place_on_conveyor(self, which: str):
        """
        Notify Robot 2 which conveyor to use, then drop die on it.
        which: 'front' (even pip) or 'back' (odd pip)
        """
        self.get_logger().info(f'Delivering to {which} conveyor...')
        sel_msg = String()
        sel_msg.data = which
        self._pub_conveyor.publish(sel_msg)

        self._send_cart(**CONVEYOR_ABV)
        self._send_cart(**CONVEYOR_DROP)
        self._send_gripper('open')
        self._send_cart(**CONVEYOR_ABV)

        # Signal Robot 2 the die is on the belt
        ready_msg = Bool()
        ready_msg.data = True
        self._pub_ready.publish(ready_msg)

    def _wait_for_return(self, timeout: float = 120.0) -> bool:
        self._dice_returned = False
        deadline = time.time() + timeout
        self.get_logger().info('Waiting for Robot 2 to return the die...')
        while not self._dice_returned and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self._dice_returned

    # ── Main game loop ────────────────────────────────────────────────────────

    def run(self):
        self._set_state(State.SETUP)
        self.get_logger().info('Waiting for action servers and camera service...')
        self._cart.wait_for_server()
        self._joint.wait_for_server()
        self._gripper.wait_for_server()
        self._cam_cli.wait_for_service()

        modbus_ip = self.get_parameter('modbus_ip').value
        self._modbus_connect(modbus_ip)
        self._set_state(State.SETUP)

        self.get_logger().info('All servers ready. Starting game.')

        self._set_state(State.WAIT)
        self.go_home()

        try:
            # ── Phase 1: find pip = 1 (start state) ──────────────────────────
            self.get_logger().info('\n=== PHASE 1: Searching for pip = 1 ===')
            phase1_retries = 0
            while True:
                self._set_state(State.GRAB_DIE)
                self.pick_dice()

                pips = self._find_pip_rotating(1, 'search')

                pip_msg = Int32()
                pip_msg.data = pips
                self._pub_pip.publish(pip_msg)

                if pips == 1:
                    self.get_logger().info('pip = 1 found — START STATE reached!')
                    break

                self.get_logger().info('pip = 1 not on any face — putting down and re-picking...')
                self._set_state(State.RECOVER)
                self.put_dice_down()
                phase1_retries += 1
                self.r1_retries += 1

            # ── Phase 2: sequential 1 → 6 ────────────────────────────────────
            self.get_logger().info('\n=== PHASE 2: Sequential 1 → 6 ===')

            # Robot 1 already holds the die showing pip=1, so skip pick for target=1
            skip_pick = True

            for target in range(1, 7):
                round_retries = 0
                self.get_logger().info(f'\n--- Target pip: {target} ---')

                while True:
                    if not skip_pick:
                        self._set_state(State.GRAB_DIE)
                        self.pick_dice()

                    skip_pick = False
                    pips = self._find_pip_rotating(target, f'seq_t{target}_r{round_retries}')

                    pip_msg = Int32()
                    pip_msg.data = pips
                    self._pub_pip.publish(pip_msg)

                    if pips == target:
                        self.get_logger().info(f'Correct pip ({pips}) — delivering to Robot 2')
                        break

                    self.get_logger().info(f'pip={target} not on any face — putting down and re-picking...')
                    self._set_state(State.RECOVER)
                    self.put_dice_down()
                    round_retries += 1
                    self.r1_retries += 1

                self._total_counts.append((target, pips, round_retries))

                self._set_state(State.PLACE_DIE)
                conveyor = 'front' if target % 2 == 0 else 'back'
                self.place_on_conveyor(conveyor)
                self._set_ready(True)

                if target == 6:
                    break   # Robot 2 places die — game over

                # Wait for Robot 2 to verify and return the die
                self._set_state(State.WAIT)
                if not self._wait_for_return():
                    self.get_logger().error('Timeout waiting for die return — aborting')
                    self._set_state(State.FAULT)
                    break

                self._set_ready(False)
                self.get_logger().info('Die returned — picking up for next target')

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
        sep = '=' * 50
        print(f'\n{sep}')
        print('         DICE GAME  —  ROBOT 1 RESULTS')
        print(sep)
        print(f'  {"Target":>6}  {"Got":>4}  {"R1 Retries":>10}')
        print(f'  {"-"*6}  {"-"*4}  {"-"*10}')
        for target, pips, retries in self._total_counts:
            print(f'  {target:>6}  {pips:>4}  {retries:>10}')
        print(f'  {"-"*35}')
        print(f'  Total Robot 1 retries: {self.r1_retries}')
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
