"""
BFS-based rotation sequence planner.

Given a DiceState and a goal predicate, finds the shortest sequence of rotation
names that satisfies the goal.  Because there are only 24 possible die orientations
the BFS is O(1) in practice (≤ 5 rotations to reach any target from any state).

The ``allowed_rotations`` parameter lets you restrict which moves the robot can
physically perform.  If the goal is unreachable with the permitted moves, the
function returns None.
"""

from __future__ import annotations

from collections import deque
from typing import Callable, Optional

from .dice_model import DiceState, ROTATION_NAMES


def plan_rotation_sequence(
    current:           DiceState,
    target:            Callable[[DiceState], bool],
    allowed_rotations: tuple[str, ...] = ROTATION_NAMES,
) -> Optional[list[str]]:
    """
    Find the shortest rotation sequence from *current* to any state satisfying *target*.

    Parameters
    ----------
    current:
        Die orientation at the start of planning.
    target:
        Predicate; receives a DiceState, returns True when the goal is reached.
    allowed_rotations:
        Subset of ROTATION_NAMES the robot can execute.  Defaults to all six.

    Returns
    -------
    List of rotation name strings (empty list if already at target), or None if
    the target is unreachable using the allowed rotations.
    """
    if target(current):
        return []

    queue:   deque[tuple[DiceState, list[str]]] = deque([(current, [])])
    visited: set[DiceState]                     = {current}

    while queue:
        state, path = queue.popleft()
        for rot_name in allowed_rotations:
            nxt = state.apply(rot_name)
            if nxt in visited:
                continue
            next_path = path + [rot_name]
            if target(nxt):
                return next_path
            visited.add(nxt)
            queue.append((nxt, next_path))

    return None   # target unreachable with allowed_rotations


# ---------------------------------------------------------------------------
#  Convenience wrappers
# ---------------------------------------------------------------------------

def plan_to_even(
    current:           DiceState,
    allowed_rotations: tuple[str, ...] = ROTATION_NAMES,
) -> Optional[list[str]]:
    """Shortest sequence to place an even number (2, 4, or 6) on top."""
    return plan_rotation_sequence(current, lambda s: s.top % 2 == 0, allowed_rotations)


def plan_to_odd(
    current:           DiceState,
    allowed_rotations: tuple[str, ...] = ROTATION_NAMES,
) -> Optional[list[str]]:
    """Shortest sequence to place an odd number (1, 3, or 5) on top."""
    return plan_rotation_sequence(current, lambda s: s.top % 2 == 1, allowed_rotations)


def plan_to_value(
    current:           DiceState,
    value:             int,
    allowed_rotations: tuple[str, ...] = ROTATION_NAMES,
) -> Optional[list[str]]:
    """Shortest sequence to place a specific pip value (1–6) on top."""
    if not 1 <= value <= 6:
        raise ValueError(f'value must be 1–6, got {value}')
    return plan_rotation_sequence(current, lambda s: s.top == value, allowed_rotations)


# ---------------------------------------------------------------------------
#  Discovery-phase helpers (for unknown initial orientation)
# ---------------------------------------------------------------------------

def choose_discovery_rotation(
    current_top:       int,
    target:            Callable[[DiceState], bool],
    allowed_rotations: tuple[str, ...] = ROTATION_NAMES,
) -> str:
    """
    Given only the top face pip count, select the single discovery rotation
    that minimises the *worst-case* total rotation count (1 discovery move +
    remaining plan-to-target moves) across all 4 possible initial orientations
    that share that top face.

    Only roll_forward / roll_backward / roll_left / roll_right are candidates;
    spin moves leave the top unchanged and reveal nothing new.

    Parameters
    ----------
    current_top:
        Pip count detected on the top face before any move.
    target:
        Same predicate used by plan_rotation_sequence — returns True when the
        goal orientation is reached.
    allowed_rotations:
        Subset of rotations the robot can physically perform.

    Returns
    -------
    The rotation name string that minimises worst-case total moves.
    """
    from .dice_model import ALL_ORIENTATIONS

    candidates = [
        r for r in ('roll_forward', 'roll_backward', 'roll_left', 'roll_right')
        if r in allowed_rotations
    ]
    if not candidates:
        raise ValueError('No valid discovery rotations in allowed_rotations.')

    # The four possible initial states that share current_top
    possible = [s for s in ALL_ORIENTATIONS if s.top == current_top]

    best_rotation  = candidates[0]
    best_worst     = float('inf')
    best_avg       = float('inf')

    for disc_rot in candidates:
        totals = []
        for s in possible:
            after    = s.apply(disc_rot)
            # If the discovery rotation itself lands on target, 0 more moves needed
            if target(after):
                totals.append(1)
            else:
                remaining = plan_rotation_sequence(after, target, allowed_rotations)
                totals.append(1 + (len(remaining) if remaining is not None else 999))

        worst = max(totals)
        avg   = sum(totals) / len(totals)

        # Primary sort: worst-case; tie-break: average
        if (worst, avg) < (best_worst, best_avg):
            best_worst    = worst
            best_avg      = avg
            best_rotation = disc_rot

    return best_rotation
