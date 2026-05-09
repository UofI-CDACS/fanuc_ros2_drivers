"""
dice_vision — die orientation, rotation planning, and robot execution interface.

Quick start
-----------
    from dice_vision import DiceController, DiceState, NullSchema

    # 1. Set initial die state (top, front, right face pip counts)
    ctrl = DiceController(schema=NullSchema(verbose=True))
    ctrl.set_state_directly(top=3, front=1, right=5)

    # 2. Plan & execute rotations so the top face becomes even
    result = ctrl.run(target='even')
    print(result['rotation_sequence'])   # e.g. ['roll_forward']
    print(result['final_state'].top)     # 4, 2, or 6

See dice_controller.py for the full API and rotation_schema.py to wire in
a real robot.
"""

from .dice_model import (
    DiceState,
    from_visible_faces,
    ALL_ORIENTATIONS,
    CANONICAL,
    ROTATION_NAMES,
)
from .rotation_planner import (
    plan_rotation_sequence,
    plan_to_even,
    plan_to_odd,
    plan_to_value,
    choose_discovery_rotation,
)
from .rotation_schema import RotationSchema, NullSchema
from .dice_controller import DiceController

__all__ = [
    # Model
    'DiceState', 'from_visible_faces', 'ALL_ORIENTATIONS', 'CANONICAL',
    'ROTATION_NAMES',
    # Planner
    'plan_rotation_sequence', 'plan_to_even', 'plan_to_odd', 'plan_to_value',
    'choose_discovery_rotation',
    # Schema
    'RotationSchema', 'NullSchema',
    # Controller
    'DiceController',
]
