"""
Robot 1 Task Node — pips 1 → 3 → 5
====================================
R1 starts with the dice and always drops on the ODD conveyor (drop_conveyor_ns).
It picks up from the EVEN conveyor (pickup_conveyor_ns) between cycles.

Full sequence:
  [cycle 0] HOME → pick dice → camera loop (find pip 1) → drop on odd conveyor
            → start odd → HOME → wait even sensor → stop even → pick up
  [cycle 1] camera loop (find pip 3) → drop on odd → start odd
            → HOME → wait even sensor → stop even → pick up
  [cycle 2] camera loop (find pip 5) → drop on odd → start odd → FINAL_HOME

Config (task_config.yaml, robot_index=1):
  drop_conveyor_ns:   <odd conveyor robot_name>
  pickup_conveyor_ns: <even conveyor robot_name>
  prox_sensor_side:   <whichever side detects the even conveyor>
"""

from robot_task.base_node import RobotTaskBase
from robot_task.states import State


class Robot1TaskNode(RobotTaskBase):

    _PIP_SEQUENCE           = [1, 3, 5]
    _FIRST_STATE_AFTER_HOME = State.OPEN_GRIPPER

    def __init__(self):
        super().__init__('robot1_task_node')

    async def execute_task(self):
        await self._run(self._base_handlers())
