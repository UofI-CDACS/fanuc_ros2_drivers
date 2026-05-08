"""
Position recorder.

Subscribes to <robot_name>/cur_cartesian and <robot_name>/cur_joints
(published by the action_servers / msg_publishers launch), shows the live
pose on one terminal line, and on each Enter prompts for a name and prints
+ appends a snapshot to recorded_positions.txt.

Usage:
    python3 ModbusStuff/record_positions.py [robot_name]

robot_name defaults to "dj". Make sure the launch is running first:
    ros2 launch launch/start.launch.py robot_name:=dj robot_ip:=<IP>

Controls:
  Enter        — capture snapshot, prompt for name
  o + Enter    — open gripper
  c + Enter    — close gripper
  Ctrl-C / q   — quit
"""

import os
import sys
import threading
import time
from datetime import datetime

import rclpy
from fanuc_interfaces.action import SchunkGripper
from fanuc_interfaces.msg import CurCartesian, CurJoints
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

OUTFILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       'recorded_positions.txt')


class PoseListener(Node):

    def __init__(self, robot_name: str):
        super().__init__('position_recorder')

        self._cart = None
        self._joints = None
        self._lock = threading.Lock()

        self.create_subscription(
            CurCartesian, f'{robot_name}/cur_cartesian',
            self._cart_cb, 10,
        )
        self.create_subscription(
            CurJoints, f'{robot_name}/cur_joints',
            self._joint_cb, 10,
        )

        self._gripper_ac = ActionClient(
            self, SchunkGripper, f'{robot_name}/schunk_gripper'
        )

    def _cart_cb(self, msg: CurCartesian):
        with self._lock:
            self._cart = list(msg.pose)

    def _joint_cb(self, msg: CurJoints):
        with self._lock:
            self._joints = list(msg.joints)

    def snapshot(self):
        with self._lock:
            return (
                list(self._cart) if self._cart is not None else None,
                list(self._joints) if self._joints is not None else None,
            )

    def gripper(self, command: str, timeout: float = 5.0) -> bool:
        if not self._gripper_ac.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('Schunk gripper action server not available')
            return False
        goal = SchunkGripper.Goal()
        goal.command = command
        send_fut = self._gripper_ac.send_goal_async(goal)
        deadline = time.time() + timeout
        while not send_fut.done() and time.time() < deadline:
            time.sleep(0.02)
        if not send_fut.done():
            return False
        gh = send_fut.result()
        if gh is None or not gh.accepted:
            return False
        result_fut = gh.get_result_async()
        while not result_fut.done() and time.time() < deadline:
            time.sleep(0.02)
        return result_fut.done()


def _fmt(vals):
    if vals is None:
        return 'waiting...'
    return '[' + ', '.join(f'{v:8.2f}' for v in vals) + ']'


def live_loop(node: PoseListener, pause_evt: threading.Event):
    while rclpy.ok():
        if not pause_evt.is_set():
            cart, joints = node.snapshot()
            line = f'  cart={_fmt(cart)}  joints={_fmt(joints)}'
            sys.stdout.write('\r\033[K' + line)
            sys.stdout.flush()
        time.sleep(0.2)


def prompt_loop(node: PoseListener, pause_evt: threading.Event):
    print(f'Recording to {OUTFILE}')
    print('Enter=capture, o=open gripper, c=close gripper, q=quit.\n')

    while rclpy.ok():
        try:
            user = input()
        except EOFError:
            return

        pause_evt.set()
        sys.stdout.write('\r\033[K')

        cmd = user.strip().lower()

        if cmd in ('q', 'quit', 'exit'):
            return

        if cmd == 'o':
            print('  Opening gripper...')
            ok = node.gripper('open')
            print('  Gripper opened.\n' if ok else '  Gripper open FAILED.\n')
            pause_evt.clear()
            continue

        if cmd == 'c':
            print('  Closing gripper...')
            ok = node.gripper('close')
            print('  Gripper closed.\n' if ok else '  Gripper close FAILED.\n')
            pause_evt.clear()
            continue

        cart, joints = node.snapshot()
        if cart is None or joints is None:
            print('No pose received yet — is the launch running? '
                  'Check the robot_name matches.\n')
            pause_evt.clear()
            continue

        try:
            name = input('  Name this position: ').strip()
        except EOFError:
            return
        if not name:
            print('  (empty name — skipped)\n')
            pause_evt.clear()
            continue

        ts = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        block = (
            f'\n# {name}    ({ts})\n'
            f'{name}_cart   = {cart}\n'
            f'{name}_joints = {joints}\n'
        )
        print(block)

        with open(OUTFILE, 'a') as f:
            f.write(block)

        pause_evt.clear()


def main():
    robot_name = sys.argv[1] if len(sys.argv) > 1 else 'dj'

    rclpy.init()
    node = PoseListener(robot_name)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    pause_evt = threading.Event()
    live_thread = threading.Thread(
        target=live_loop, args=(node, pause_evt), daemon=True,
    )
    live_thread.start()

    print(f'Subscribed to {robot_name}/cur_cartesian and '
          f'{robot_name}/cur_joints')

    try:
        prompt_loop(node, pause_evt)
    except KeyboardInterrupt:
        pass
    finally:
        sys.stdout.write('\r\033[K')
        sys.stdout.flush()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        print('\nBye.')


if __name__ == '__main__':
    main()
