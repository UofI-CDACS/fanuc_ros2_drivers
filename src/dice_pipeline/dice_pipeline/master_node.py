#!/usr/bin/env python3
"""
Master / control node for the dice pip counting pipeline.

Sequence (repeated NUM_ROLLS times):
  1. Move to home joint position
  2. Open gripper
  3. Approach dice (Cartesian, above)
  4. Lower to grasp height
  5. Close gripper
  6. Lift back up
  7. Move to camera presentation position
  8. Call /camera/count_pips service → record result
  9. Return to place-down position above table
 10. Lower and release die
 11. Lift back up

After all rolls, logs:
  • Individual pip count per roll
  • Total pip count across all rolls
  • Frequency table: how many times each face value (1-6) appeared

IMPORTANT: Update the TODO positions below to match your actual robot setup.
           Jog the robot to each key pose and record the coordinates.
"""

import threading

import rclpy
from fanuc_interfaces.action import CartPose, JointPose, SchunkGripper
from fanuc_interfaces.srv import CountPips
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

NUM_ROLLS = 3

# ── Robot positions ───────────────────────────────────────────────────────────
# Joint home position [J1..J6] in degrees -- a safe, out-of-the-way pose.
HOME_JOINTS    = [0.0,  0.0, 0.0, 0.0, -90.0, 30.0]
# J1 rotated 90° from home -- arm swings to face the camera table.
# Only J1 moves between HOME_JOINTS and CAMERA_JOINTS, so this is fast.
CAMERA_JOINTS  = [90.0, 0.0, 0.0, 0.0, -90.0, 30.0]

# Cartesian poses as dicts with keys x, y, z (mm) and w, p, r (degrees).
# w=200, p=200, r=200 is a sentinel that tells cart_pose_server to keep the
# current WPR value unchanged (see cart_pose_server.py goal_callback).
PICK_ABOVE  = dict(x=637.73, y=-9.84,  z=106.67, w=179.9, p=0.0,   r=30.0)
PICK_GRASP  = dict(x=637.73, y=-9.84,  z=63.28,  w=179.9, p=0.0,   r=30.0)
# Lift straight up before retracting to HOME so the arm clears the side table.
TRANSIT_UP    = dict(x=637.73,  y=-9.84,  z=400.0,   w=179.9,  p=0.0,   r=30.0)
# Rotation position: robot holds die at an angle near the table so that when
# the gripper opens the die tips over and lands on a new face.
ROTATION_POS   = dict(x=903.648, y=-1.89,  z=-208.587, w=-92.131, p=59.676, r=89.319)
# After releasing: step forward in x to clear die front, then lift clear.
ROTATION_CLEAR = dict(x=980.0,   y=-1.89,  z=-208.587, w=-92.131, p=59.676, r=89.319)
CAMERA_POS    = dict(x=183.91,  y=638.96, z=747.54,  w=91.89,  p=-62.45, r=174.91)
PLACE_ABOVE = dict(x=637.73, y=-9.84,  z=106.67, w=179.9, p=0.0,   r=30.0)
PLACE_DOWN  = dict(x=637.73, y=-9.84,  z=63.28,  w=179.9, p=0.0,   r=30.0)
# ─────────────────────────────────────────────────────────────────────────────


class MasterNode(Node):
    def __init__(self, robot_name: str):
        super().__init__('master_node')
        self.cb_group = ReentrantCallbackGroup()

        ns = f'/{robot_name}'
        self.cart_ac = ActionClient(
            self, CartPose, f'{ns}/cartesian_pose',
            callback_group=self.cb_group,
        )
        self.joint_ac = ActionClient(
            self, JointPose, f'{ns}/joint_pose',
            callback_group=self.cb_group,
        )
        self.schunk_ac = ActionClient(
            self, SchunkGripper, f'{ns}/schunk_gripper',
            callback_group=self.cb_group,
        )
        self.count_client = self.create_client(
            CountPips, '/camera/count_pips',
            callback_group=self.cb_group,
        )

    # ── Low-level helpers ─────────────────────────────────────────────────────

    def _wait(self, future) -> None:
        """Block the pipeline thread until a future completes.

        The MultiThreadedExecutor is already spinning in the main thread and
        will process all callbacks -- we just need to wait here without trying
        to spin again (which would raise 'Executor is already spinning').
        """
        event = threading.Event()
        future.add_done_callback(lambda _: event.set())
        event.wait()

    def _send_cart(self, **pose) -> bool:
        """Block until a CartPose action completes. Returns success flag."""
        goal = CartPose.Goal()
        goal.x, goal.y, goal.z = pose['x'], pose['y'], pose['z']
        goal.w, goal.p, goal.r = pose['w'], pose['p'], pose['r']

        self.cart_ac.wait_for_server()
        gh_fut = self.cart_ac.send_goal_async(goal)
        self._wait(gh_fut)
        goal_handle = gh_fut.result()
        if not goal_handle.accepted:
            self.get_logger().error('CartPose goal rejected.')
            return False
        res_fut = goal_handle.get_result_async()
        self._wait(res_fut)
        return res_fut.result().result.success

    def _send_joints(self, joints: list) -> bool:
        """Block until a JointPose action completes. Returns success flag."""
        goal = JointPose.Goal()
        (goal.joint1, goal.joint2, goal.joint3,
         goal.joint4, goal.joint5, goal.joint6) = joints

        self.joint_ac.wait_for_server()
        gh_fut = self.joint_ac.send_goal_async(goal)
        self._wait(gh_fut)
        goal_handle = gh_fut.result()
        if not goal_handle.accepted:
            self.get_logger().error('JointPose goal rejected.')
            return False
        res_fut = goal_handle.get_result_async()
        self._wait(res_fut)
        return res_fut.result().result.success

    def _schunk(self, command: str) -> bool:
        """Open or close the Schunk gripper. command: 'open' | 'close'."""
        goal = SchunkGripper.Goal()
        goal.command = command

        self.schunk_ac.wait_for_server()
        gh_fut = self.schunk_ac.send_goal_async(goal)
        self._wait(gh_fut)
        goal_handle = gh_fut.result()
        if not goal_handle.accepted:
            self.get_logger().error(f'SchunkGripper goal "{command}" rejected.')
            return False
        res_fut = goal_handle.get_result_async()
        self._wait(res_fut)
        return res_fut.result().result.success

    def _count_pips(self) -> int:
        """Call the camera node's count_pips service. Returns pip count (-1 on error)."""
        if not self.count_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('/camera/count_pips service not available.')
            return -1
        fut = self.count_client.call_async(CountPips.Request())
        self._wait(fut)
        resp = fut.result()
        if not resp.success:
            self.get_logger().error(f'Count pips failed: {resp.message}')
            return -1
        return resp.pip_count

    # ── Main pipeline sequence ────────────────────────────────────────────────

    def _do_rotation(self):
        """Pick up die, tilt to new face, release, retract. Die stays on table rotated."""
        self.get_logger().info('Rotating die for next roll...')
        self._send_cart(**PICK_ABOVE)
        self._send_cart(**PICK_GRASP)
        self._schunk('close')
        self._send_cart(**PICK_ABOVE)
        self._send_cart(**TRANSIT_UP)
        self._send_cart(**ROTATION_POS)
        self._schunk('open')
        self._send_cart(**ROTATION_CLEAR)
        self._send_cart(**TRANSIT_UP)
        self._send_cart(**PICK_ABOVE)

    def run_pipeline(self):
        roll_counts = []

        for roll in range(1, NUM_ROLLS + 1):
            self.get_logger().info(f'=== Roll {roll}/{NUM_ROLLS} ===')

            # 1. Pick up die.
            # Roll 1: start fresh from home.
            # Rolls 2+: rotation left us at PICK_ABOVE with gripper open — go straight down.
            if roll == 1:
                self.get_logger().info('Moving to home...')
                self._send_joints(HOME_JOINTS)
                self._schunk('open')
                self._send_cart(**PICK_ABOVE)
            self.get_logger().info('Picking up die...')
            self._send_cart(**PICK_GRASP)
            self._schunk('close')

            # 2. Transit to camera
            self.get_logger().info('Transiting to camera...')
            self._send_cart(**TRANSIT_UP)
            self._send_joints(HOME_JOINTS)
            self._send_joints(CAMERA_JOINTS)
            self._send_cart(**CAMERA_POS)

            # 3. Count pips
            self.get_logger().info('Counting pips...')
            count = self._count_pips()

            # 4. Return die to table
            self.get_logger().info('Returning die to table...')
            self._send_joints(CAMERA_JOINTS)
            self._send_joints(HOME_JOINTS)
            self._send_cart(**PLACE_ABOVE)
            self._send_cart(**PLACE_DOWN)
            self._schunk('open')
            self._send_cart(**PLACE_ABOVE)

            if count < 0:
                self.get_logger().warn(f'Roll {roll}: pip detection failed, skipping.')
            else:
                roll_counts.append(count)
                self.get_logger().info(f'Roll {roll} result: {count} pip(s)')

            # 5. Rotate die for next roll (skip after last roll)
            if roll < NUM_ROLLS:
                self._do_rotation()

        # Return home when done
        self._send_joints(HOME_JOINTS)

        # ── Report results ────────────────────────────────────────────────────
        self._report(roll_counts)

    def _report(self, roll_counts: list):
        sep = '=' * 40
        self.get_logger().info(sep)
        self.get_logger().info('DICE COUNTING RESULTS')
        self.get_logger().info(sep)

        for i, c in enumerate(roll_counts, 1):
            self.get_logger().info(f'  Roll {i}: {c} pip(s)')

        if roll_counts:
            total = sum(roll_counts)
            self.get_logger().info(f'  Total pips across {len(roll_counts)} roll(s): {total}')

            # Frequency table: how many times each face value appeared
            freq = {}
            for c in roll_counts:
                freq[c] = freq.get(c, 0) + 1
            self.get_logger().info('  Face frequency:')
            for face in sorted(freq):
                self.get_logger().info(f'    {face} pip(s): {freq[face]} time(s)')
        else:
            self.get_logger().warn('  No valid rolls recorded.')

        self.get_logger().info(sep)


def main(args=None):
    rclpy.init(args=args)

    # Read robot_name from ROS2 parameter (set via launch file or CLI).
    temp = rclpy.create_node('_param_reader')
    temp.declare_parameter('robot_name', 'noNAME')
    robot_name = temp.get_parameter('robot_name').value
    temp.destroy_node()

    node = MasterNode(robot_name)
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    # Run the pipeline sequence in a background thread so the executor can
    # process action/service callbacks while we "wait" sequentially.
    pipeline_thread = threading.Thread(
        target=node.run_pipeline, daemon=True
    )
    pipeline_thread.start()

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        pipeline_thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
