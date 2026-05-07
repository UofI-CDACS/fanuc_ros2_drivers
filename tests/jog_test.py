#!/usr/bin/env python3
"""
jog_test.py

Interactive test for both robots. Run from the workspace root after sourcing:
    python3 tests/jog_test.py

Requires driver nodes to be running:
    ros2 launch launch/start.launch.py robot_name:=DJ   robot_ip:=10.8.4.16
    ros2 launch launch/start.launch.py robot_name:=BILL robot_ip:=10.8.4.6
"""

import sys
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from fanuc_interfaces.action import JointPose, SchunkGripper, OnRobotGripper, Conveyor
from fanuc_interfaces.msg import ProxReadings


class RobotTester(Node):
    def __init__(self):
        super().__init__('jog_test')
        cb = ReentrantCallbackGroup()

        # DJ clients
        self.dj_joints  = ActionClient(self, JointPose,    'DJ/joint_pose',       callback_group=cb)
        self.dj_schunk  = ActionClient(self, SchunkGripper,'DJ/schunk_gripper',   callback_group=cb)
        self.dj_convey  = ActionClient(self, Conveyor,     'DJ/conveyor',         callback_group=cb)

        # BILL clients
        self.bill_joints   = ActionClient(self, JointPose,      'BILL/joint_pose',      callback_group=cb)
        self.bill_onrobot  = ActionClient(self, OnRobotGripper, 'BILL/onrobot_gripper', callback_group=cb)
        self.bill_convey   = ActionClient(self, Conveyor,       'BILL/conveyor',        callback_group=cb)

        # Prox sensor cache
        self._prox = {'DJ': {'left': 0, 'right': 0}, 'BILL': {'left': 0, 'right': 0}}
        self.create_subscription(ProxReadings, 'DJ/prox_readings',
            lambda msg: self._prox['DJ'].update({'left': int(msg.left), 'right': int(msg.right)}), 10)
        self.create_subscription(ProxReadings, 'BILL/prox_readings',
            lambda msg: self._prox['BILL'].update({'left': int(msg.left), 'right': int(msg.right)}), 10)

    # ------------------------------------------------------------------
    def _wait(self, future, timeout=10.0):
        deadline = time.time() + timeout
        while not future.done():
            if time.time() > deadline:
                raise TimeoutError('Action timed out')
            time.sleep(0.02)
        return future.result()

    def _send_joint(self, client, j1, j2, j3, j4, j5, j6):
        goal = JointPose.Goal()
        goal.joint1, goal.joint2, goal.joint3 = j1, j2, j3
        goal.joint4, goal.joint5, goal.joint6 = j4, j5, j6
        client.wait_for_server(timeout_sec=5.0)
        gh = self._wait(client.send_goal_async(goal))
        if not gh.accepted:
            print('  Goal rejected.')
            return False
        result = self._wait(gh.get_result_async(), timeout=30.0)
        print(f'  Done. success={result.result.success}')
        return result.result.success

    def _send_conveyor(self, client, cmd):
        goal = Conveyor.Goal()
        goal.command = cmd
        client.wait_for_server(timeout_sec=5.0)
        gh = self._wait(client.send_goal_async(goal))
        if not gh.accepted:
            print('  Goal rejected.')
            return
        self._wait(gh.get_result_async())
        print(f'  Conveyor {cmd}.')

    def _send_schunk(self, cmd):
        goal = SchunkGripper.Goal()
        goal.command = cmd
        self.dj_schunk.wait_for_server(timeout_sec=5.0)
        gh = self._wait(self.dj_schunk.send_goal_async(goal))
        if not gh.accepted:
            print('  Goal rejected.')
            return
        self._wait(gh.get_result_async())
        print(f'  Schunk {cmd}.')

    def _run_conveyor_sensors(self, robot: str, client, start_sensor: str, stop_sensor: str,
                              direction: str = 'forward', timeout: float = 20.0):
        """Wait for start_sensor to trip, run conveyor, stop when stop_sensor trips.
        Each sensor gets its own timeout window."""
        prox = self._prox[robot]

        print(f'  [{robot}] Waiting for start sensor ({start_sensor}) to trip — block it now...')
        deadline = time.time() + timeout
        while not prox[start_sensor]:
            if time.time() > deadline:
                print('  Timed out waiting for start sensor.')
                return
            time.sleep(0.05)
        print(f'  [{robot}] Start sensor tripped — conveyor ON ({direction}).')
        self._send_conveyor(client, direction)

        print(f'  [{robot}] Waiting for stop sensor ({stop_sensor}) to trip...')
        deadline = time.time() + timeout
        while not prox[stop_sensor]:
            if time.time() > deadline:
                print('  Timed out waiting for stop sensor.')
                self._send_conveyor(client, 'stop')
                return
            time.sleep(0.05)
        self._send_conveyor(client, 'stop')
        print(f'  [{robot}] Stop sensor tripped — conveyor OFF.')

    def _send_onrobot(self, width, force):
        goal = OnRobotGripper.Goal()
        goal.width = width
        goal.force = force
        self.bill_onrobot.wait_for_server(timeout_sec=5.0)
        gh = self._wait(self.bill_onrobot.send_goal_async(goal))
        if not gh.accepted:
            print('  Goal rejected.')
            return
        self._wait(gh.get_result_async())
        print(f'  OnRobot width={width}mm force={force}N.')

    # ------------------------------------------------------------------
    def run_menu(self):
        menu = """
========================================
  Robot Jog Test
========================================
  DJ joints     : dj j1 j2 j3 j4 j5 j6
  BILL joints   : bill j1 j2 j3 j4 j5 j6

  DJ conveyor   : djc              (sensor: start=left  stop=right, forward)
                  djc [forward|reverse|stop]   (manual override)
  BILL conveyor : bc               (sensor: start=right stop=left,  reverse)
                  bc  [forward|reverse|stop]   (manual override)

  DJ schunk     : schunk [open|close]
  BILL onrobot  : onrobot [open|close]
                  onrobot width force

  quit          : q
========================================
"""
        print(menu)
        while True:
            try:
                raw = input('> ').strip()
            except (EOFError, KeyboardInterrupt):
                break

            if not raw:
                continue

            parts = raw.split()
            cmd = parts[0].lower()

            try:
                if cmd == 'q':
                    break

                elif cmd == 'dj':
                    if len(parts) != 7:
                        print('Usage: dj j1 j2 j3 j4 j5 j6')
                        continue
                    joints = [float(x) for x in parts[1:]]
                    print(f'  Moving DJ to {joints}')
                    self._send_joint(self.dj_joints, *joints)

                elif cmd == 'bill':
                    if len(parts) != 7:
                        print('Usage: bill j1 j2 j3 j4 j5 j6')
                        continue
                    joints = [float(x) for x in parts[1:]]
                    print(f'  Moving BILL to {joints}')
                    self._send_joint(self.bill_joints, *joints)

                elif cmd == 'djc':
                    if len(parts) == 1:
                        print('DJ sensor-controlled conveyor (start=left, stop=right, forward)')
                        self._run_conveyor_sensors('DJ', self.dj_convey, 'left', 'right')
                    elif len(parts) == 2 and parts[1] in ('forward', 'reverse', 'stop'):
                        self._send_conveyor(self.dj_convey, parts[1])
                    else:
                        print('Usage: djc              (sensor-controlled)')
                        print('       djc [forward|reverse|stop]   (manual)')

                elif cmd == 'bc':
                    if len(parts) == 1:
                        print('BILL sensor-controlled conveyor (start=right, stop=left, reverse)')
                        self._run_conveyor_sensors('BILL', self.bill_convey, 'right', 'left', direction='reverse')
                    elif len(parts) == 2 and parts[1] in ('forward', 'reverse', 'stop'):
                        self._send_conveyor(self.bill_convey, parts[1])
                    else:
                        print('Usage: bc              (sensor-controlled)')
                        print('       bc  [forward|reverse|stop]   (manual)')

                elif cmd == 'djcs':
                    print('DJ sensor-controlled conveyor test (start=left, stop=right)')
                    self._run_conveyor_sensors('DJ', self.dj_convey, 'left', 'right')

                elif cmd == 'bcs':
                    print('BILL sensor-controlled conveyor test (start=right, stop=left)')
                    self._run_conveyor_sensors('BILL', self.bill_convey, 'right', 'left', direction='reverse')

                elif cmd == 'schunk':
                    if len(parts) != 2 or parts[1] not in ('open', 'close'):
                        print('Usage: schunk [open|close]')
                        continue
                    self._send_schunk(parts[1])

                elif cmd == 'onrobot':
                    if len(parts) == 2:
                        width = 80 if parts[1] == 'open' else 35
                        force = 40
                    elif len(parts) == 3:
                        width, force = int(parts[1]), int(parts[2])
                    else:
                        print('Usage: onrobot [open|close]  OR  onrobot width force')
                        continue
                    self._send_onrobot(width, force)

                else:
                    print(f'Unknown command: {cmd}')

            except TimeoutError as e:
                print(f'  Timeout: {e}')
            except Exception as e:
                print(f'  Error: {e}')


def main():
    rclpy.init()
    node = RobotTester()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    node.run_menu()

    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
