#!/usr/bin/env python3
"""
Master / Control node for the dice experiment.

Sequence (repeated NUM_ROLLS times)
-------------------------------------
1. Open gripper
2. Move to approach position above dice
3. Descend to pick position
4. Close gripper (grab dice)
5. Lift back to approach height
6. Move to camera presentation position
7. Call camera capture service → get pip count
8. Return to home
9. Open gripper (release dice)

After all rolls, a summary table is printed to the ROS logger.

Robot positions
---------------
All Cartesian positions are [X, Y, Z, W, P, R] in mm / degrees.
!!! IMPORTANT: These must be calibrated to your physical robot setup !!!
Set them via ROS parameters (see parameter declarations below) or edit
the DEFAULTS dict directly.

How to run
----------
    ros2 run dice_controller master_node --ros-args \
        -p robot_name:=bunsen \
        -p pick_x:=110.0 -p pick_y:=640.0 -p pick_z:=-100.0 ...
"""

import threading
import time

import rclpy
from fanuc_interfaces.action import CartPose, SchunkGripper
from rclpy.action import ActionClient
from rclpy.node import Node
from std_srvs.srv import Trigger

NUM_ROLLS = 3

# ──────────────────────────────────────────────────────────────────────────────
# Default robot positions — MUST be calibrated to your setup (mm / degrees)
# ──────────────────────────────────────────────────────────────────────────────
DEFAULTS = {
    # Home / safe resting position
    'home_x': 110.0,  'home_y': 600.0,  'home_z': 200.0,
    'home_w': 170.0,  'home_p':   0.0,  'home_r':  30.0,

    # Approach — directly above the dice at a safe height
    'approach_x': 110.0,  'approach_y': 640.0,  'approach_z': -50.0,
    'approach_w': 170.0,  'approach_p':   0.0,  'approach_r':  30.0,

    # Pick — at dice level (gripper contacts dice here)
    'pick_x': 110.0,  'pick_y': 640.0,  'pick_z': -100.0,
    'pick_w': 170.0,  'pick_p':   0.0,  'pick_r':   30.0,

    # Camera — position dice under the overhead camera for imaging
    'camera_x': 200.0,  'camera_y': 500.0,  'camera_z': 50.0,
    'camera_w': 170.0,  'camera_p':   0.0,  'camera_r': 30.0,
}
# ──────────────────────────────────────────────────────────────────────────────


def _poll_future(future, poll_interval: float = 0.05):
    """
    Block the calling thread until an rclpy Future completes.

    This is safe to call from the main thread while a separate thread is
    spinning the executor (which resolves the future via callbacks).
    """
    while not future.done():
        time.sleep(poll_interval)
    return future.result()


class DiceMasterNode(Node):
    def __init__(self):
        super().__init__('dice_master')

        # ── ROS parameters ──────────────────────────────────────────────────
        param_decls = [('robot_name', 'noNAME')]
        for key, val in DEFAULTS.items():
            param_decls.append((key, val))
        self.declare_parameters(namespace='', parameters=param_decls)

        robot_name = self.get_parameter('robot_name').value

        # ── Action clients ──────────────────────────────────────────────────
        self.cart_ac   = ActionClient(self, CartPose,      f'/{robot_name}/cartesian_pose')
        self.schunk_ac = ActionClient(self, SchunkGripper, f'/{robot_name}/schunk_gripper')

        # ── Camera service client ────────────────────────────────────────────
        self.camera_cli = self.create_client(Trigger, f'/{robot_name}/camera/capture')

        self.get_logger().info(f'Dice master node ready (robot: {robot_name})')

    # ── position helpers ──────────────────────────────────────────────────────

    def _pose(self, prefix: str) -> list:
        """Build a 6-element pose list from parameters named <prefix>_x/y/z/w/p/r."""
        return [self.get_parameter(f'{prefix}_{ax}').value for ax in ('x', 'y', 'z', 'w', 'p', 'r')]

    # ── action / service wrappers ─────────────────────────────────────────────

    def _move_cartesian(self, pose: list) -> bool:
        """
        Send a CartPose goal and block until the robot finishes moving.

        Parameters
        ----------
        pose : [X, Y, Z, W, P, R]

        Returns
        -------
        bool  True on success
        """
        goal = CartPose.Goal()
        goal.x, goal.y, goal.z = pose[0], pose[1], pose[2]
        goal.w, goal.p, goal.r = pose[3], pose[4], pose[5]

        self.get_logger().info(f'Moving to [{", ".join(f"{v:.1f}" for v in pose)}]')
        self.cart_ac.wait_for_server()

        goal_future   = self.cart_ac.send_goal_async(goal)
        handle        = _poll_future(goal_future)

        if not handle.accepted:
            self.get_logger().error('CartPose goal rejected by server')
            return False

        result = _poll_future(handle.get_result_async()).result
        if not result.success:
            self.get_logger().error('CartPose action returned failure')
        return result.success

    def _gripper(self, command: str) -> bool:
        """
        Send a SchunkGripper goal ('open' or 'close') and wait for completion.

        Returns
        -------
        bool  True on success
        """
        goal = SchunkGripper.Goal()
        goal.command = command

        self.get_logger().info(f'Gripper: {command}')
        self.schunk_ac.wait_for_server()

        goal_future = self.schunk_ac.send_goal_async(goal)
        handle      = _poll_future(goal_future)

        if not handle.accepted:
            self.get_logger().error(f'SchunkGripper goal "{command}" rejected')
            return False

        result = _poll_future(handle.get_result_async()).result
        return result.success

    def _capture_and_count(self) -> tuple[int, str]:
        """
        Call the camera capture service and parse the response.

        Returns
        -------
        (pip_count, image_path)  pip_count is 0 on failure.
        """
        self.camera_cli.wait_for_service()
        response = _poll_future(self.camera_cli.call_async(Trigger.Request()))

        if not response.success:
            self.get_logger().error('Camera capture service reported failure')
            return 0, ''

        pip_count_str, image_path = response.message.split('|', 1)
        return int(pip_count_str), image_path

    # ── main sequence ─────────────────────────────────────────────────────────

    def run_dice_sequence(self):
        """
        Execute the full dice-inspection sequence NUM_ROLLS times, then print
        a results summary.
        """
        home_pose     = self._pose('home')
        approach_pose = self._pose('approach')
        pick_pose     = self._pose('pick')
        camera_pose   = self._pose('camera')

        results: list[tuple[int, int, str]] = []  # (roll_num, pip_count, img_path)
        face_counts: dict[int, int] = {}           # {dice_value: times_seen}

        # Move to known-good starting position
        self._move_cartesian(home_pose)

        for roll in range(1, NUM_ROLLS + 1):
            self.get_logger().info(f'══ Roll {roll}/{NUM_ROLLS} ══')

            # 1. Open gripper so we can grab the dice
            self._gripper('open')

            # 2. Move above dice (approach height — safe clearance)
            self._move_cartesian(approach_pose)

            # 3. Descend to dice level
            self._move_cartesian(pick_pose)

            # 4. Close gripper to pick up the dice
            self._gripper('close')

            # 5. Lift back to approach height (dice now held)
            self._move_cartesian(approach_pose)

            # 6. Move to camera presentation position
            self._move_cartesian(camera_pose)

            # 7. Capture image and count pips
            pip_count, img_path = self._capture_and_count()
            self.get_logger().info(f'Roll {roll} result: {pip_count} pip(s)  image → {img_path}')

            results.append((roll, pip_count, img_path))
            face_counts[pip_count] = face_counts.get(pip_count, 0) + 1

            # 8. Return home with dice still in gripper
            self._move_cartesian(home_pose)

            # 9. Release dice
            self._gripper('open')

        self._print_summary(results, face_counts)

    # ── reporting ─────────────────────────────────────────────────────────────

    def _print_summary(
        self,
        results: list[tuple[int, int, str]],
        face_counts: dict[int, int],
    ):
        total = sum(r[1] for r in results)
        W = 44  # table width

        lines = ['', '=' * W]
        lines.append(f'{"DICE ROLL SUMMARY":^{W}}')
        lines.append('=' * W)

        # Per-roll breakdown
        lines.append(f'{"Roll":<6} {"Pips":>4}  {"Face":<6}  Image')
        lines.append('-' * W)
        for roll_num, pips, path in results:
            fname = path.split('/')[-1] if path else '(none)'
            pip_bar = '●' * pips
            lines.append(f'{roll_num:<6} {pips:>4}  {pip_bar:<6}  {fname}')

        # Totals
        lines.append('-' * W)
        lines.append(f'{"Total pips:":<20} {total}')
        lines.append(f'{"Rolls completed:":<20} {len(results)}')
        lines.append(f'{"Average pips/roll:":<20} {total / len(results):.2f}')

        # Histogram by dice face (pip count)
        lines.append('')
        lines.append('Count per dice face value:')
        for val in sorted(face_counts):
            bar   = '█' * face_counts[val]
            count = face_counts[val]
            label = f'  [{val} pip{"s" if val != 1 else " "}]'
            lines.append(f'{label:<14} {bar} ({count}x)')

        lines.append('=' * W)
        self.get_logger().info('\n'.join(lines))


def main(args=None):
    rclpy.init(args=args)
    node = DiceMasterNode()

    # Spin the executor in a background daemon thread so ROS callbacks
    # (action results, service responses) are processed while the main
    # thread runs the sequential dice sequence.
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        node.run_dice_sequence()
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted — shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
