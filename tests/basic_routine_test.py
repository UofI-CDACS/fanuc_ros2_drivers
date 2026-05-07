#!/usr/bin/env python3
"""
basic_routine_test.py  —  BILL routine test (non-interactive)

Runs BILL through a pick-from-conveyor and set-on-conveyor sequence.
Conveyor runs as a background monitor: starts when right sensor trips,
stops when left sensor trips (reverse direction).

Run:
    python3 tests/basic_routine_test.py
"""

import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from fanuc_interfaces.action import JointPose, OnRobotGripper, Conveyor
from fanuc_interfaces.msg import ProxReadings, IsMoving

# ── Joint positions ───────────────────────────────────────────────────────────
BILL_HOME                   = [0.0,    0.0,    0.0,   0.0,   -90.0,  -45.0]
BILL_PRESENT_TO_CAMERA      = [-65.17, 26.40,  -5.36, -78.83, 27.21,  31.95]
BILL_SET_ON_CONVEYOR        = [-89.51, 22.82, -27.22,  1.25, -61.61, -48.54]
BILL_PRE_SET_ON_CONVEYOR    = [-89.51, 18.89, -12.84,  1.14, -75.99, -48.22]  # NOTE: j5 was written as -61.61-75.99 — using -75.99
BILL_PRE_PICK_FROM_CONVEYOR = [-109.77,  31.81,  -7.24,  0.633, -81.27, -27.79]
BILL_PICK_FROM_CONVEYOR     = [-109.768, 34.16, -18.87,  0.668, -69.65, -27.93]

# ── Gripper ───────────────────────────────────────────────────────────────────
GRIPPER_OPEN_WIDTH  = 90   # mm
GRIPPER_CLOSE_WIDTH = 50   # mm
GRIPPER_FORCE       = 10   # N

# ── Conveyor sensors (BILL front conveyor) ────────────────────────────────────
CONVEYOR_START_SENSOR   = 'right'   # trip → start conveyor
CONVEYOR_STOP_SENSOR    = 'left'    # trip → stop conveyor
CONVEYOR_DIRECTION      = 'reverse'
CONVEYOR_SENSOR_TIMEOUT = 200.0     # seconds per sensor

# How long to wait for the robot to start moving after a command (seconds)
MOTION_START_TIMEOUT    = 3.0
# How long to wait for a motion to complete (seconds)
MOTION_COMPLETE_TIMEOUT = 60.0


class BillRoutineTest(Node):
    def __init__(self):
        super().__init__('bill_routine_test')
        cb = ReentrantCallbackGroup()

        self._joints  = ActionClient(self, JointPose,      'BILL/joint_pose',      callback_group=cb)
        self._gripper = ActionClient(self, OnRobotGripper, 'BILL/onrobot_gripper', callback_group=cb)
        self._convey  = ActionClient(self, Conveyor,       'BILL/conveyor',        callback_group=cb)

        self._prox   = {'left': 0, 'right': 0}
        self._moving = False

        self.create_subscription(ProxReadings, 'BILL/prox_readings',
            lambda msg: self._prox.update({'left': int(msg.left), 'right': int(msg.right)}), 10,
            callback_group=cb)

        self.create_subscription(IsMoving, 'BILL/is_moving',
            lambda msg: setattr(self, '_moving', bool(msg.moving)), 10,
            callback_group=cb)

        # Conveyor monitor runs in background thread
        self._conveyor_monitor_active = False
        self._monitor_thread = threading.Thread(target=self._conveyor_monitor, daemon=True)
        self._monitor_thread.start()

    # ── Helpers ───────────────────────────────────────────────────────────────
    def _wait(self, future, timeout=30.0):
        deadline = time.time() + timeout
        while not future.done():
            if time.time() > deadline:
                raise TimeoutError('Future timed out')
            time.sleep(0.02)
        return future.result()

    def _wait_for_move(self):
        """Block until BILL/is_moving goes True then False (motion starts and finishes)."""
        # Wait for motion to start
        deadline = time.time() + MOTION_START_TIMEOUT
        while not self._moving:
            if time.time() > deadline:
                self.get_logger().warn('Robot did not register as moving — may have already arrived')
                break
            time.sleep(0.05)

        # Wait for motion to finish
        deadline = time.time() + MOTION_COMPLETE_TIMEOUT
        while self._moving:
            if time.time() > deadline:
                raise TimeoutError('Motion did not complete within timeout')
            time.sleep(0.05)

    def _move(self, joints: list):
        self.get_logger().info(f'Move → {joints}')
        goal = JointPose.Goal()
        goal.joint1, goal.joint2, goal.joint3 = joints[0], joints[1], joints[2]
        goal.joint4, goal.joint5, goal.joint6 = joints[3], joints[4], joints[5]
        self._joints.wait_for_server()
        gh = self._wait(self._joints.send_goal_async(goal))
        if not gh.accepted:
            raise RuntimeError('Joint move rejected')
        self._wait(gh.get_result_async(), timeout=MOTION_COMPLETE_TIMEOUT)

    def _set_gripper(self, width: int):
        goal = OnRobotGripper.Goal()
        goal.width = width
        goal.force = GRIPPER_FORCE
        self._gripper.wait_for_server()
        gh = self._wait(self._gripper.send_goal_async(goal))
        if not gh.accepted:
            raise RuntimeError('Gripper goal rejected')
        self._wait(gh.get_result_async())
        state = 'OPEN' if width >= GRIPPER_OPEN_WIDTH else 'CLOSE'
        self.get_logger().info(f'Gripper {state}')
        time.sleep(2.0)

    def _conveyor_cmd(self, cmd: str):
        goal = Conveyor.Goal()
        goal.command = cmd
        self._convey.wait_for_server()
        gh = self._wait(self._convey.send_goal_async(goal))
        if not gh.accepted:
            raise RuntimeError('Conveyor goal rejected')
        self._wait(gh.get_result_async())

    # ── Conveyor background monitor ───────────────────────────────────────────
    def _conveyor_monitor(self):
        """
        Runs one cycle in background:
          Idle until activated → wait for start sensor (right) → start conveyor
          → wait for stop sensor (left) → stop conveyor → signal done and exit.
        """
        # Wait until activated by run()
        while not self._conveyor_monitor_active:
            if not rclpy.ok():
                return
            time.sleep(0.05)

        # Wait for start sensor
        self.get_logger().info('Conveyor monitor: waiting for start sensor...')
        deadline = time.time() + CONVEYOR_SENSOR_TIMEOUT
        while not self._prox[CONVEYOR_START_SENSOR]:
            if time.time() > deadline:
                self.get_logger().warn('Conveyor monitor: start sensor timeout')
                self._conveyor_monitor_active = False
                return
            time.sleep(0.05)

        self.get_logger().info('Conveyor monitor: start sensor tripped → running')
        self._conveyor_cmd(CONVEYOR_DIRECTION)

        # Wait for stop sensor
        deadline = time.time() + CONVEYOR_SENSOR_TIMEOUT
        while not self._prox[CONVEYOR_STOP_SENSOR]:
            if time.time() > deadline:
                self.get_logger().warn('Conveyor monitor: stop sensor timeout')
                break
            time.sleep(0.05)

        self._conveyor_cmd('stop')
        self.get_logger().info('Conveyor monitor: stop sensor tripped → stopped')
        self._conveyor_monitor_active = False   # signal run() to exit

    # ── Main routine ──────────────────────────────────────────────────────────
    def run(self):
        self.get_logger().info('=== BILL routine test starting ===')

        self._conveyor_monitor_active = True

        self._move(BILL_HOME)
        self._set_gripper(GRIPPER_OPEN_WIDTH)
        self._move(BILL_PRE_PICK_FROM_CONVEYOR)
        self._move(BILL_PICK_FROM_CONVEYOR)
        self._set_gripper(GRIPPER_CLOSE_WIDTH)
        self._move(BILL_PRE_PICK_FROM_CONVEYOR)
        self._move(BILL_PRE_SET_ON_CONVEYOR)
        self._move(BILL_SET_ON_CONVEYOR)
        self._set_gripper(GRIPPER_OPEN_WIDTH)   # die placed — conveyor monitor takes over
        self._move(BILL_PRE_SET_ON_CONVEYOR)
        self._move(BILL_HOME)

        # Block until conveyor monitor completes one cycle and exits
        self.get_logger().info('Waiting for conveyor to complete...')
        self._monitor_thread.join()

        self.get_logger().info('=== Routine complete ===')


def main(args=None):
    rclpy.init(args=args)
    node = BillRoutineTest()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
