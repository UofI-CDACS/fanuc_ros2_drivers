"""
Abstract interface between the rotation planner and robot-specific motion.

The planner produces a list of abstract rotation names
(roll_forward, roll_backward, roll_left, roll_right, spin_cw, spin_ccw).
A RotationSchema translates each name into the actual robot commands for a
particular gripper configuration, robot model, or grasping strategy.

How to add a new robot
----------------------
1. Subclass RotationSchema.
2. Implement _execute(rotation_name) to call your robot's motion API.
3. Pass an instance to DiceController.

Example skeleton for a FANUC robot:
    from dice_vision import RotationSchema

    class FanucRobot1Schema(RotationSchema):
        def __init__(self, robot_ip: str):
            from robot_controller import robot
            self.bot = robot(robot_ip)

        def _execute(self, rotation: str) -> bool:
            # map rotation name → joint deltas or Cartesian moves
            if rotation == 'roll_forward':
                ...   # e.g. self.bot.write_joint_position(...)
            elif rotation == 'roll_backward':
                ...
            ...
            return True
"""

from __future__ import annotations

from abc import ABC, abstractmethod

ROTATION_NAMES: tuple[str, ...] = (
    'roll_forward', 'roll_backward',
    'roll_left',    'roll_right',
    'spin_cw',      'spin_ccw',
)


class RotationSchema(ABC):
    """
    Abstract mapping from rotation names to physical robot movements.

    Each execute call should block until the robot finishes moving so the
    controller can reliably track state after each step.
    """

    def execute(self, rotation: str) -> bool:
        """
        Execute one named rotation.

        Parameters
        ----------
        rotation:
            One of the six ROTATION_NAMES strings.

        Returns
        -------
        True if the robot completed the move successfully, False on failure.
        """
        if rotation not in ROTATION_NAMES:
            raise ValueError(
                f"Unknown rotation '{rotation}'. Must be one of {ROTATION_NAMES}"
            )
        return self._execute(rotation)

    @abstractmethod
    def _execute(self, rotation: str) -> bool:
        """Override this in each robot-specific subclass."""
        ...

    def execute_sequence(self, rotations: list[str]) -> bool:
        """
        Execute a list of rotations in order.

        Stops immediately and returns False if any single rotation fails.
        """
        for rot in rotations:
            if not self.execute(rot):
                return False
        return True


class NullSchema(RotationSchema):
    """
    No-op schema for unit tests and dry runs.

    Logs each rotation name without sending any commands to a robot.
    Inspect .history to verify the planned sequence.
    """

    def __init__(self, verbose: bool = True):
        self.verbose = verbose
        self.history: list[str] = []

    def _execute(self, rotation: str) -> bool:
        self.history.append(rotation)
        if self.verbose:
            print(f'[NullSchema] execute: {rotation}')
        return True

    def reset(self) -> None:
        """Clear the execution history."""
        self.history.clear()
