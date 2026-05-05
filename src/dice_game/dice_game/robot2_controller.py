"""
robot2_controller.py  —  Robot 2

Waits for Robot 1 to deliver the die, receives it from the conveyor,
presents it to the overhead camera (via the camera server on Robot 1's machine),
verifies the pip count, then either returns the die to Robot 1 OR places it
down in front of itself (pip == 6 = end of game).

Robot 2 controls BOTH conveyors:
  front conveyor action: /<FRONT_CONV_NAME>/conveyor   (even pip counts)
  back  conveyor action: /<BACK_CONV_NAME>/conveyor    (odd pip counts)

  *** Set FRONT_CONV_NAME / BACK_CONV_NAME below to match your launch. ***

Inter-robot communication (all topics under /dice_game/):
  Subscribes:
    pip_count        (Int32)  — pip Robot 1 just counted
    conveyor_select  (String) — "front" or "back"
    dice_ready       (Bool)   — die is on the conveyor belt
  Publishes:
    dice_returned    (Bool)   — die is back at Robot 1's pickup spot
    r2_pip_count     (Int32)  — Robot 2's own pip count (verification)

Retry counting:
  r2_retries increments when Robot 2's pip count != Robot 1's reported pip.
  This catches cases where the die shifted on the conveyor.
"""

import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from fanuc_interfaces.action import CartPose, JointPose, SchunkGripper, Conveyor
from fanuc_interfaces.srv import CaptureImage
from std_msgs.msg import Int32, String, Bool

from dice_game.pip_counter import count_pips, save_debug_image

# ── Conveyor action server names ──────────────────────────────────────────────
# Update these to match the robot_name used when launching the conveyor servers.
FRONT_CONV_NAME = 'front_conveyor'   # /<name>/conveyor  (even pips)
BACK_CONV_NAME  = 'back_conveyor'    # /<name>/conveyor  (odd pips)

# ── Robot 2 positions  (FILL THESE IN for your physical setup) ────────────────
HOME_JOINTS      = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)     # TODO
CONV_FRONT_PICK  = dict(x=0.0, y=0.0, z=0.0, w=0.0, p=0.0, r=0.0)   # TODO
CONV_BACK_PICK   = dict(x=0.0, y=0.0, z=0.0, w=0.0, p=0.0, r=0.0)   # TODO
CAMERA_POSE      = dict(x=0.0, y=0.0, z=0.0, w=0.0, p=0.0, r=0.0)   # TODO
RETURN_DROP      = dict(x=0.0, y=0.0, z=0.0, w=0.0, p=0.0, r=0.0)   # TODO — Robot 1 pickup spot
FINAL_PLACE      = dict(x=0.0, y=0.0, z=0.0, w=0.0, p=0.0, r=0.0)   # TODO — in front of Robot 2

DEBUG_IMG_DIR = '/home/colin/Desktop'


class Robot2Controller(Node):

    def __init__(self):
        super().__init__('robot2_controller')

        self.declare_parameters('', [
            ('robot_name', 'Robot2'),
            ('robot_ip',   '0.0.0.0'),
        ])
        self._name = self.get_parameter('robot_name').value

        # ── Action clients — Robot 2's own arm ───────────────────────────────
        self._cart    = ActionClient(self, CartPose,      f'/{self._name}/cartesian_pose')
        self._joint   = ActionClient(self, JointPose,     f'/{self._name}/joint_pose')
        self._gripper = ActionClient(self, SchunkGripper, f'/{self._name}/schunk_gripper')

        # ── Conveyor action clients ───────────────────────────────────────────
        self._front_conv = ActionClient(self, Conveyor, f'/{FRONT_CONV_NAME}/conveyor')
        self._back_conv  = ActionClient(self, Conveyor, f'/{BACK_CONV_NAME}/conveyor')

        # ── Camera service client (server runs on Robot 1's machine) ─────────
        self._cam_cli = self.create_client(CaptureImage, '/dice_game/capture_image')

        # ── Inter-robot communication ─────────────────────────────────────────
        self._pub_returned  = self.create_publisher(Bool,  '/dice_game/dice_returned', 10)
        self._pub_r2_pip    = self.create_publisher(Int32, '/dice_game/r2_pip_count',  10)

        self._r1_pip       = 0
        self._conveyor_sel = ''
        self._dice_ready   = False

        self.create_subscription(Int32,  '/dice_game/pip_count',      self._on_pip,      10)
        self.create_subscription(String, '/dice_game/conveyor_select', self._on_conveyor, 10)
        self.create_subscription(Bool,   '/dice_game/dice_ready',      self._on_ready,    10)

        # ── Retry counter ─────────────────────────────────────────────────────
        self.r2_retries = 0

    # ── Topic callbacks ───────────────────────────────────────────────────────

    def _on_pip(self, msg: Int32):
        self._r1_pip = msg.data

    def _on_conveyor(self, msg: String):
        self._conveyor_sel = msg.data

    def _on_ready(self, msg: Bool):
        if msg.data:
            self._dice_ready = True

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

    def _send_conveyor(self, which: str, command: str) -> bool:
        client = self._front_conv if which == 'front' else self._back_conv
        client.wait_for_server()
        goal = Conveyor.Goal()
        goal.command = command
        future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        gh = future.result()
        if not gh.accepted:
            return False
        res = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res)
        return res.result().result.success

    # ── Camera ────────────────────────────────────────────────────────────────

    def _capture_and_count(self, label: str) -> int:
        self.get_logger().info('Moving to camera position...')
        self._send_cart(**CAMERA_POSE)
        time.sleep(0.4)

        self._cam_cli.wait_for_service()
        future = self._cam_cli.call_async(CaptureImage.Request())
        rclpy.spin_until_future_complete(self, future)
        resp = future.result()

        if not resp.success:
            self.get_logger().error(f'Camera capture failed: {resp.message}')
            return 0

        arr  = np.array(resp.image_data, dtype=np.uint8)
        img  = arr.reshape((resp.height, resp.width, resp.channels))
        pips = count_pips(img)

        path = f'{DEBUG_IMG_DIR}/r2_{label}.png'
        save_debug_image(img, pips, path)
        self.get_logger().info(f'Robot 2 counted {pips} pip(s) — saved {path}')

        pip_msg = Int32()
        pip_msg.data = pips
        self._pub_r2_pip.publish(pip_msg)
        return pips

    # ── Robot moves ───────────────────────────────────────────────────────────

    def go_home(self):
        self.get_logger().info('Going home...')
        self._send_joint(*HOME_JOINTS)

    def _wait_for_dice_ready(self, timeout: float = 120.0) -> bool:
        self._dice_ready = False
        deadline = time.time() + timeout
        while not self._dice_ready and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self._dice_ready

    def _receive_from_conveyor(self, which: str):
        """Start conveyor, wait for die to arrive, pick it up, stop conveyor."""
        pick_pose = CONV_FRONT_PICK if which == 'front' else CONV_BACK_PICK

        self.get_logger().info(f'Starting {which} conveyor...')
        self._send_conveyor(which, 'forward')

        # Move to receive position while conveyor runs
        self._send_cart(**pick_pose)

        # Brief wait for die to reach pickup end, then grab
        time.sleep(2.0)   # TODO: tune to conveyor travel time
        self._send_gripper('close')
        self._send_conveyor(which, 'stop')

    def _return_die_to_robot1(self, which: str):
        """Reverse the conveyor briefly to return die to Robot 1's side."""
        self.get_logger().info('Returning die to Robot 1 via conveyor reverse...')
        self._send_gripper('open')
        self._send_conveyor(which, 'reverse')
        time.sleep(3.0)   # TODO: tune to conveyor travel time
        self._send_conveyor(which, 'stop')

        returned_msg = Bool()
        returned_msg.data = True
        self._pub_returned.publish(returned_msg)

    # ── Main game loop ────────────────────────────────────────────────────────

    def run(self):
        self.get_logger().info('Waiting for action servers and camera service...')
        self._cart.wait_for_server()
        self._joint.wait_for_server()
        self._gripper.wait_for_server()
        self._cam_cli.wait_for_service()
        self.get_logger().info('Robot 2 ready. Waiting for Robot 1...')

        self.go_home()

        r2_results = []  # (round, r1_pip, r2_pip, mismatch)

        for target in range(1, 7):
            self.get_logger().info(f'\n--- Robot 2 waiting for pip {target} ---')

            # Wait for Robot 1 to place die on conveyor
            if not self._wait_for_dice_ready():
                self.get_logger().error('Timeout waiting for die — aborting')
                break

            which = self._conveyor_sel   # 'front' or 'back' set by Robot 1
            r1_reported = self._r1_pip

            self._receive_from_conveyor(which)

            # Present to camera and verify
            r2_pip = self._capture_and_count(f'verify_t{target}')

            mismatch = (r2_pip != r1_reported)
            if mismatch:
                self.get_logger().warn(
                    f'Pip mismatch! Robot 1 said {r1_reported}, Robot 2 sees {r2_pip}')
                self.r2_retries += 1
            else:
                self.get_logger().info(f'Verified pip = {r2_pip} ✓')

            r2_results.append((target, r1_reported, r2_pip, mismatch))

            if target == 6:
                # Final placement in front of Robot 2
                self.get_logger().info('pip = 6 — placing die in front of Robot 2')
                self._send_cart(**FINAL_PLACE)
                self._send_gripper('open')
                self._send_cart(**dict(FINAL_PLACE, z=FINAL_PLACE['z'] + 50))  # lift clear
                break

            # Return die to Robot 1 for next round
            self._return_die_to_robot1(which)

        self.go_home()
        self._print_results(r2_results)

    def _print_results(self, results):
        sep = '=' * 50
        print(f'\n{sep}')
        print('         DICE GAME  —  ROBOT 2 RESULTS')
        print(sep)
        print(f'  {"Pip":>4}  {"R1 Said":>7}  {"R2 Saw":>6}  {"Match?":>6}')
        print(f'  {"-"*4}  {"-"*7}  {"-"*6}  {"-"*6}')
        for pip, r1, r2, mismatch in results:
            match_str = 'NO' if mismatch else 'yes'
            print(f'  {pip:>4}  {r1:>7}  {r2:>6}  {match_str:>6}')
        print(f'  {"-"*35}')
        print(f'  Total Robot 2 retries (mismatches): {self.r2_retries}')
        print(sep + '\n')


def main(args=None):
    rclpy.init(args=args)
    node = Robot2Controller()
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
