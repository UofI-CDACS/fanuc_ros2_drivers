#!/usr/bin/env python3
"""
Master / control node for FANUC dice inspection assignment.

Sequence (repeated 3 times per run):
  1. Open gripper
  2. Move to approach position (above die)
  3. Move to pick position (grasp die)
  4. Close gripper
  5. Move back to approach position
  6. Move to present position (hold die up to overhead camera)
  7. Call capture_and_count service -> record pip count
  8. Move to set-down position
  9. Open gripper
 10. Move to approach
 11. Move home (joint)

After 3 rounds: print individual counts and total.

Configuration via ROS2 parameters (set in launch file / .env):
  - robot_name  : matches the name used when launching action_servers
  - robot_ip    : not used directly here; servers handle comms

Positions are defined as constants below. Adjust after physical testing.
"""

import os
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.task import Future

from action_msgs.msg import GoalStatus
from fanuc_interfaces.action import CartPose, JointPose, OnRobotGripper
from std_srvs.srv import Trigger


# -----------------------------------------------------------------------
# Robot positions  (all Cartesian in mm/deg, FANUC User Frame 1 Tool 1)
# -----------------------------------------------------------------------

# Pick position — arm vertical, gripper reaching down to die
PICK = dict(x=517.953, y=-859.577, z=95.431,
            w=-177.637, p=3.802,   r=-134.627)

# Approach — same orientation as pick but raised by 100 mm on Z
# Adjust APPROACH_Z_OFFSET if needed after live testing
APPROACH_Z_OFFSET = 100.0
APPROACH = dict(x=PICK['x'], y=PICK['y'],
                z=PICK['z'] + APPROACH_Z_OFFSET,
                w=PICK['w'], p=PICK['p'], r=PICK['r'])

# Set-down position — arm horizontal, die resting on surface
SETDOWN = dict(x=526.943, y=-593.427, z=-135.913,
               w=92.306,  p=48.264,   r=4.772)

# Present-to-camera — same x/y/w/p/r as set-down, raised 300 mm
PRESENT = dict(x=SETDOWN['x'], y=SETDOWN['y'],
               z=SETDOWN['z'] + 300.0,
               w=SETDOWN['w'], p=SETDOWN['p'], r=SETDOWN['r'])

# Home joint angles (safe resting position — adjust as needed)
HOME_JOINTS = dict(joint1=0.0, joint2=20.0, joint3=-20.0,
                   joint4=0.0, joint5=-20.0, joint6=0.0)

# -----------------------------------------------------------------------
# Gripper parameters (OnRobot 2-finger)
# -----------------------------------------------------------------------
GRIPPER_OPEN_WIDTH  = 100   # mm  — wide enough to clear the die
GRIPPER_OPEN_FORCE  = 20    # N
GRIPPER_CLOSE_WIDTH = 55    # mm  — gripping die (~50 mm wide)
GRIPPER_CLOSE_FORCE = 40    # N

# Number of pip-count rounds
NUM_ROUNDS = 3


class MasterNode(Node):
    def __init__(self):
        super().__init__('master_node')

        self.declare_parameter('robot_name', 'fanuc')
        robot_name = self.get_parameter('robot_name').value

        # Action clients
        self._cart_client  = ActionClient(self, CartPose,
                                          f'/{robot_name}/cartesian_pose')
        self._joint_client = ActionClient(self, JointPose,
                                          f'{robot_name}/joint_pose')
        self._grip_client  = ActionClient(self, OnRobotGripper,
                                          f'{robot_name}/onrobot_gripper')

        # Camera service client
        self._camera_client = self.create_client(Trigger, 'capture_and_count')

        self.get_logger().info('Master node initialised. Starting dice inspection...')

    # -----------------------------------------------------------------------
    # Helpers — blocking action/service calls
    # -----------------------------------------------------------------------
    def _send_cart(self, pos: dict) -> bool:
        self._cart_client.wait_for_server()
        goal = CartPose.Goal()
        goal.x = float(pos['x'])
        goal.y = float(pos['y'])
        goal.z = float(pos['z'])
        goal.w = float(pos['w'])
        goal.p = float(pos['p'])
        goal.r = float(pos['r'])
        self.get_logger().info(
            f'Moving to Cart ({goal.x:.1f}, {goal.y:.1f}, {goal.z:.1f})'
        )
        future = self._cart_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Cart goal rejected!')
            return False
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        success = result_future.result().result.success
        if not success:
            self.get_logger().error('Cart move failed.')
        return success

    def _send_joint(self, joints: dict) -> bool:
        self._joint_client.wait_for_server()
        goal = JointPose.Goal()
        goal.joint1 = float(joints['joint1'])
        goal.joint2 = float(joints['joint2'])
        goal.joint3 = float(joints['joint3'])
        goal.joint4 = float(joints['joint4'])
        goal.joint5 = float(joints['joint5'])
        goal.joint6 = float(joints['joint6'])
        self.get_logger().info(
            f'Moving to joint pose J1={goal.joint1} J2={goal.joint2} ...'
        )
        future = self._joint_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Joint goal rejected!')
            return False
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        success = result_future.result().result.success
        if not success:
            self.get_logger().error('Joint move failed.')
        return success

    def _send_gripper(self, width: int, force: int) -> bool:
        self._grip_client.wait_for_server()
        goal = OnRobotGripper.Goal()
        goal.width = width
        goal.force = force
        action = 'open' if width >= GRIPPER_OPEN_WIDTH else 'close'
        self.get_logger().info(f'Gripper {action} (width={width} mm, force={force} N)')
        future = self._grip_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Gripper goal rejected!')
            return False
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return result_future.result().result.success

    def _capture_and_count(self) -> int:
        self._camera_client.wait_for_service()
        future = self._camera_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response.success:
            count = int(response.message)
            self.get_logger().info(f'Pip count this round: {count}')
            return count
        else:
            self.get_logger().error('Camera service failed.')
            return -1

    # -----------------------------------------------------------------------
    # Main inspection sequence
    # -----------------------------------------------------------------------
    def run(self):
        # Go to home
        self.get_logger().info('--- Step 0: Go home ---')
        self._send_joint(HOME_JOINTS)

        # Open gripper
        self.get_logger().info('--- Step 1: Open gripper ---')
        self._send_gripper(GRIPPER_OPEN_WIDTH, GRIPPER_OPEN_FORCE)

        # Approach above die
        self.get_logger().info('--- Step 2: Approach (above pick) ---')
        self._send_cart(APPROACH)

        # Lower to pick
        self.get_logger().info('--- Step 3: Move to pick ---')
        self._send_cart(PICK)

        # Close gripper — grip die
        self.get_logger().info('--- Step 4: Close gripper ---')
        self._send_gripper(GRIPPER_CLOSE_WIDTH, GRIPPER_CLOSE_FORCE)

        # Lift back to approach height
        self.get_logger().info('--- Step 5: Lift to approach ---')
        self._send_cart(APPROACH)

        # Move to present position
        self.get_logger().info('--- Step 6: Move to present position ---')
        self._send_cart(PRESENT)

        # ---- 3 rounds of pip counting ----
        counts = []
        for i in range(NUM_ROUNDS):
            self.get_logger().info(f'--- Step 7.{i+1}: Capture and count (round {i+1}/{NUM_ROUNDS}) ---')
            count = self._capture_and_count()
            counts.append(count)

        # ---- Report results ----
        self._report(counts)

        # Move to set-down position
        self.get_logger().info('--- Step 8: Move to set-down position ---')
        self._send_cart(SETDOWN)

        # Release die
        self.get_logger().info('--- Step 9: Open gripper ---')
        self._send_gripper(GRIPPER_OPEN_WIDTH, GRIPPER_OPEN_FORCE)

        # Lift to approach
        self.get_logger().info('--- Step 10: Lift from set-down ---')
        self._send_cart(APPROACH)

        # Return home
        self.get_logger().info('--- Step 11: Return home ---')
        self._send_joint(HOME_JOINTS)

        self.get_logger().info('Dice inspection sequence complete.')

    def _report(self, counts: list):
        total = sum(c for c in counts if c >= 0)
        self.get_logger().info('=' * 45)
        self.get_logger().info('         DICE INSPECTION RESULTS')
        self.get_logger().info('=' * 45)
        for i, c in enumerate(counts, start=1):
            status = str(c) if c >= 0 else 'ERROR'
            self.get_logger().info(f'  Round {i}: {status} pip(s)')
        self.get_logger().info(f'  Total pip count across {NUM_ROUNDS} rounds: {total}')
        self.get_logger().info('=' * 45)

        # Also print to stdout for easy capture in terminal
        print('\n' + '=' * 45)
        print('         DICE INSPECTION RESULTS')
        print('=' * 45)
        for i, c in enumerate(counts, start=1):
            status = str(c) if c >= 0 else 'ERROR'
            print(f'  Round {i}: {status} pip(s)')
        print(f'  Total pip count across {NUM_ROUNDS} rounds: {total}')
        print('=' * 45 + '\n')


def main(args=None):
    rclpy.init(args=args)
    node = MasterNode()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
