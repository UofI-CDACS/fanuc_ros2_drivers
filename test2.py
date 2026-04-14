#!/usr/bin/env python3
"""
test2.py — Basic movement test: home → atCamera (wait 5s) → home
Usage: python3 test2.py <robot_name>
"""
import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from fanuc_interfaces.action import CartPose, JointPose, SchunkGripper

# ── HOME — joint angles (degrees) — fill in before running ──────────────────
HOME = dict(joint1=1.1, joint2=1.5, joint3=-2.0, joint4=-1.7, joint5=-88.6, joint6=-30.0)

# ── AT_CAMERA — Cartesian (mm / degrees) — fill in before running ────────────
AT_CAMERA = dict(x=490.0, y=890.0, z=881.0, w=73.0, p=-66.0, r=-170.0)

# ── ABOVE_DIE — Cartesian (mm / degrees) — fill in before running ────────────
ABOVE_DIE = dict(x=465.0, y=-15.0, z=-145.0, w=179.9, p=0.0, r=30.0)

# ── DIE_HOME — Cartesian (mm / degrees) — fill in before running ─────────────
DIE_HOME = dict(x=465.0, y=-15.0, z=-185.0, w=179.9, p=0.0, r=30.0)
# ────────────────────────────────────────────────────────────────────────────


class TestMove(Node):
    def __init__(self, robot_name: str):
        super().__init__('test_move')
        self.joint_ac   = ActionClient(self, JointPose,     f'/{robot_name}/joint_pose')
        self.cart_ac    = ActionClient(self, CartPose,      f'/{robot_name}/cartesian_pose')
        self.gripper_ac = ActionClient(self, SchunkGripper, f'/{robot_name}/schunk_gripper')

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

    def _send_gripper(self, command: str) -> bool:
        self.gripper_ac.wait_for_server()
        goal = SchunkGripper.Goal()
        goal.command = command
        future = self.gripper_ac.send_goal_async(goal)
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
            print('ERROR: failed to reach HOME'); return

        print('Opening gripper...')
        if not self._send_gripper('open'):
            print('ERROR: failed to open gripper'); return

        print('Moving to ABOVE_DIE...')
        if not self._send_cart(ABOVE_DIE):
            print('ERROR: failed to reach ABOVE_DIE'); return

        print('Moving to DIE_HOME...')
        if not self._send_cart(DIE_HOME):
            print('ERROR: failed to reach DIE_HOME'); return

        print('Closing gripper...')
        if not self._send_gripper('close'):
            print('ERROR: failed to close gripper'); return

        print('Moving to HOME (joint)...')
        if not self._send_joint(HOME):
            print('ERROR: failed to reach HOME'); return

        print('Moving to AT_CAMERA...')
        if not self._send_cart(AT_CAMERA):
            print('ERROR: failed to reach AT_CAMERA'); return

        print('Waiting 5 seconds...')
        time.sleep(5)

        print('Moving to ABOVE_DIE...')
        if not self._send_cart(ABOVE_DIE):
            print('ERROR: failed to reach ABOVE_DIE'); return

        print('Moving to DIE_HOME...')
        if not self._send_cart(DIE_HOME):
            print('ERROR: failed to reach DIE_HOME'); return

        print('Opening gripper...')
        if not self._send_gripper('open'):
            print('ERROR: failed to open gripper'); return

        print('Moving to HOME (joint)...')
        if not self._send_joint(HOME):
            print('ERROR: failed to return HOME'); return

        print('Done.')


def main():
    if len(sys.argv) < 2:
        print('Usage: python3 test2.py <robot_name>')
        sys.exit(1)

    rclpy.init()
    node = TestMove(sys.argv[1])
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
