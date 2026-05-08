"""
Conveyor handoff test.

Runs the conveyor forward and monitors the RIGHT proximity sensor.
Stop logic:
  1. Wait for right sensor to go True  (dice arrives at sensor)
  2. Wait for right sensor to go False (dice passes sensor)
  3. Stop conveyor — dice is now in partner's pickup zone

Prints live sensor state every cycle so you can see what's happening.

Usage:
    python3 conveyor_test.py

Requires ROS2 action servers + msg publishers to be running:
    ros2 launch launch/start.launch.py robot_name:=dj robot_ip:=<IP>
"""

import threading
import time

import rclpy
from fanuc_interfaces.action import Conveyor
from fanuc_interfaces.msg import ProxReadings
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

NAMESPACE = 'dj'


class ConveyorTest(Node):

    def __init__(self):
        super().__init__('conveyor_test')
        self.conveyor_ac = ActionClient(self, Conveyor, f'/{NAMESPACE}/conveyor')

        self._left  = False
        self._right = False
        self._lock  = threading.Lock()

        self.create_subscription(
            ProxReadings, f'/{NAMESPACE}/prox_readings', self._prox_cb, 10
        )

    def _prox_cb(self, msg: ProxReadings):
        with self._lock:
            self._left  = msg.left
            self._right = msg.right

    def _sensors(self):
        with self._lock:
            return self._left, self._right

    # ── Conveyor helpers ──────────────────────────────────────────────────────

    def _wait(self, fut):
        while not fut.done():
            time.sleep(0.02)

    def conveyor_cmd(self, command: str) -> bool:
        self.conveyor_ac.wait_for_server()
        goal = Conveyor.Goal()
        goal.command = command
        send = self.conveyor_ac.send_goal_async(goal)
        self._wait(send)
        gh = send.result()
        if not gh or not gh.accepted:
            self.get_logger().error(f'Conveyor goal rejected ({command})')
            return False
        self._wait(gh.get_result_async())
        return True

    # ── Main test ─────────────────────────────────────────────────────────────

    def run(self):
        print(f"\n{'='*55}")
        print(f"  Conveyor handoff test  —  namespace: /{NAMESPACE}")
        print(f"{'='*55}")
        print("  Logic: forward → wait for right sensor ON → wait for right sensor OFF → stop")
        print("  Place dice on conveyor, then press Enter to start.\n")

        try:
            input("  Press Enter to start conveyor ... ")
        except (EOFError, KeyboardInterrupt):
            return

        # ── Step 1: start conveyor forward ────────────────────────────────────
        print("\n  Starting conveyor FORWARD ...")
        if not self.conveyor_cmd('forward'):
            print("  Failed to start conveyor."); return
        print("  Conveyor running.\n")

        # ── Step 2: wait for right sensor to go True ─────────────────────────
        print("  Waiting for dice to reach right sensor ...")
        try:
            while True:
                left, right = self._sensors()
                print(f"\r    sensors — left: {'ON ' if left else 'off'}  right: {'ON ' if right else 'off'}   ", end='', flush=True)
                if right:
                    print(f"\n  Right sensor ON — dice detected!")
                    break
                time.sleep(0.05)
        except KeyboardInterrupt:
            print("\n  Aborted — stopping conveyor.")
            self.conveyor_cmd('stop'); return

        # ── Step 3: wait for right sensor to go False ────────────────────────
        print("  Waiting for dice to pass right sensor ...")
        try:
            while True:
                left, right = self._sensors()
                print(f"\r    sensors — left: {'ON ' if left else 'off'}  right: {'ON ' if right else 'off'}   ", end='', flush=True)
                if not right:
                    print(f"\n  Right sensor OFF — dice has passed!")
                    break
                #time.sleep(0.05)
        except KeyboardInterrupt:
            print("\n  Aborted — stopping conveyor.")
            self.conveyor_cmd('stop'); return

        # ── Step 4: stop conveyor ─────────────────────────────────────────────
        print("\n  Stopping conveyor ...")
        self.conveyor_cmd('stop')
        print("  Conveyor stopped.")

        left, right = self._sensors()
        print(f"\n{'='*55}")
        print(f"  DONE — dice is in partner pickup zone.")
        print(f"  Final sensor state — left: {'ON' if left else 'off'}  right: {'ON' if right else 'off'}")
        print(f"{'='*55}\n")


def main():
    rclpy.init()
    node = ConveyorTest()

    executor = MultiThreadedExecutor()
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
