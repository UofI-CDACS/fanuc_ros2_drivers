#!/usr/bin/env python3
"""
test_move.py — Basic movement test: home → atCamera (wait 5s) → home
Usage: python3 test_move.py <robot_name>
"""
import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from fanuc_interfaces.action import CartPose, JointPose

# ── HOME — joint angles (degrees) — fill in before running ──────────────────
HOME = dict(joint1=1.1, joint2=1.5, joint3=-2.0, joint4=-1.7, joint5=-88.6, joint6=-30.0)

# ── AT_CAMERA — Cartesian (mm / degrees) ────────────────────────────────────
AT_CAMERA = dict(x=490.0, y=890.0, z=881.0, w=73.0, p=-66.0, r=-170.0)
# ────────────────────────────────────────────────────────────────────────────


class TestMove(Node):
    def __init__(self, robot_name: str):
        super().__init__('test_move')
        self.joint_ac = ActionClient(self, JointPose, f'/{robot_name}/joint_pose')
        self.cart_ac  = ActionClient(self, CartPose,  f'/{robot_name}/cartesian_pose')

    def _send_joint(self, pos: dict) -> bool:
        self.joint_ac.wait_for_server()
        goal = JointPose.Goal()
        goal.joint1 = pos['joint1']
        goal.joint2 = pos['joint2']
        goal.joint3 = pos['joint3']
        goal.joint4 = pos['joint4']
        goal.joint5 = pos['joint5']
        goal.joint6 = pos['joint6']
        future = self.joint_ac.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        if not goal_handle.accepted:
            return False
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return result_future.result().result.success

    def _send_cart(self, pos: dict) -> bool:
        self.cart_ac.wait_for_server()
        goal = CartPose.Goal()
        goal.x = pos['x']
        goal.y = pos['y']
        goal.z = pos['z']
        goal.w = pos['w']
        goal.p = pos['p']
        goal.r = pos['r']
        future = self.cart_ac.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        if not goal_handle.accepted:
            return False
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return result_future.result().result.success

    def run(self):
        print('Moving to HOME (joint)...')
        if not self._send_joint(HOME):
            print('ERROR: failed to reach HOME')
            return

        print('Moving to AT_CAMERA (cartesian)...')
        if not self._send_cart(AT_CAMERA):
            print('ERROR: failed to reach AT_CAMERA')
            return

        print('Waiting 5 seconds...')
        time.sleep(5)

        print('Returning to HOME (joint)...')
        if not self._send_joint(HOME):
            print('ERROR: failed to return HOME')
            return

        print('Done.')


def main():
    if len(sys.argv) < 2:
        print('Usage: python3 test_move.py <robot_name>')
        sys.exit(1)

    rclpy.init()
    node = TestMove(sys.argv[1])
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
