"""
Robot 2 Task Node — pips 2 → 4 → 6
====================================
R2 always picks up from the ODD conveyor (pickup_conveyor_ns) and drops on
the EVEN conveyor (drop_conveyor_ns).  On the final cycle (pip 6) the dice
is placed at dice_pose instead of on the even conveyor.

Full sequence:
  [cycle 0] HOME → wait odd sensor → stop odd → pick up → camera loop (pip 2)
            → drop on even → start even → HOME → wait odd sensor → stop odd → pick up
  [cycle 1] camera loop (pip 4) → drop on even → start even
            → HOME → wait odd sensor → stop odd → pick up
  [cycle 2] camera loop (pip 6) → place at dice_pose → FINAL_HOME

Gripper
-------
R2 uses the Schunk gripper ('open'/'close') — _create_action_clients replaces
the base OnRobot client with a SchunkGripper client on /{robot_name}/schunk_gripper.

Config (task_config.yaml, robot_index=2):
  drop_conveyor_ns:   <even conveyor robot_name>
  pickup_conveyor_ns: <odd conveyor robot_name>
  prox_sensor_side:   <whichever side detects the odd conveyor>
"""

import asyncio
import time

from rclpy.action import ActionClient
from fanuc_interfaces.action import SchunkGripper

from robot_task.base_node import RobotTaskBase
from robot_task.states import State
from modbus_server.register_map import ROBOT_STATE_TO_CONVEYOR  # used in _handle_safe_home_to_final


class Robot2TaskNode(RobotTaskBase):

    _PIP_SEQUENCE           = [2, 4, 6]
    _FIRST_STATE_AFTER_HOME = State.WAIT_OWN_SENSOR

    def __init__(self):
        super().__init__('robot2_task_node')

    # ── Schunk gripper ────────────────────────────────────────────────

    def _create_action_clients(self):
        super()._create_action_clients()
        robot_ns = self.get_parameter('robot_name').value
        self._gripper_client = ActionClient(
            self, SchunkGripper, f'/{robot_ns}/schunk_gripper'
        )

    async def _send_gripper(self, width, force):
        command = 'open' if width >= self.gripper_open_width else 'close'
        self.get_logger().info(f"  SchunkGripper  command={command}")
        goal = SchunkGripper.Goal()
        goal.command = command
        goal_handle = await self._await_future(
            self._gripper_client.send_goal_async(goal)
        )
        if not goal_handle.accepted:
            raise RuntimeError(f"SchunkGripper goal '{command}' rejected")
        result = await self._await_future(goal_handle.get_result_async())
        if not result.result.success:
            raise RuntimeError(f"SchunkGripper '{command}' failed")
        loop = asyncio.get_running_loop()
        await loop.run_in_executor(None, time.sleep, self.gripper_settle_time)

    # ── Rotation setdown / re-pick routing ───────────────────────────

    @property
    def _setdown_pose(self) -> dict:
        """R2 always sets down at its dedicated dice surface, not the conveyor."""
        return self.dice_pose

    def _next_state_after_pip_found(self) -> State:
        if self._cycle == len(self._PIP_SEQUENCE) - 1:
            return State.SAFE_HOME_TO_FINAL
        return State.SAFE_HOME_TO_CV_DROP

    # ── Final placement at dice_pose (cycle 2 only) ───────────────────
    # Per spec: "set the dice down at the dice position" after finding pip 6.
    # dice_pose is also R2's rotation setdown surface — the same stable spot.

    async def _handle_safe_home_to_final(self):
        self.get_logger().info("[SAFE_HOME_TO_FINAL]")
        loop = asyncio.get_running_loop()
        await loop.run_in_executor(None, self._mb_set_state, ROBOT_STATE_TO_CONVEYOR)
        await self._send_joint_pose(self.home_joints)
        return State.APPROACH_FINAL_HOME

    async def _handle_approach_final_home(self):
        self.get_logger().info("[APPROACH_FINAL_HOME]")
        await self._send_cart_pose(**self._approach(self.dice_pose))
        return State.DESCEND_FINAL_HOME

    async def _handle_descend_final_home(self):
        self.get_logger().info("[DESCEND_FINAL_HOME]")
        await self._send_cart_pose(**self.dice_pose)
        return State.RELEASE_FINAL

    async def _handle_release_final(self):
        self.get_logger().info("[RELEASE_FINAL] placing dice at dice_pose (final rest)")
        await self._send_gripper(self.gripper_open_width, force=30)
        return State.ASCEND_FINAL_HOME

    async def _handle_ascend_final_home(self):
        self.get_logger().info("[ASCEND_FINAL_HOME]")
        await self._send_cart_pose(**self._approach(self.dice_pose))
        return State.FINAL_HOME

    async def execute_task(self):
        handlers = {
            **self._base_handlers(),
            State.SAFE_HOME_TO_FINAL:  self._handle_safe_home_to_final,
            State.APPROACH_FINAL_HOME: self._handle_approach_final_home,
            State.DESCEND_FINAL_HOME:  self._handle_descend_final_home,
            State.RELEASE_FINAL:       self._handle_release_final,
            State.ASCEND_FINAL_HOME:   self._handle_ascend_final_home,
        }
        await self._run(handlers)
