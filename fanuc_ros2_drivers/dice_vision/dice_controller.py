"""
DiceController — orchestrates image analysis, orientation tracking, and robot execution.

Workflow
--------
1. Tell the controller the die's initial orientation (set_state_directly or
   set_state_from_faces when three faces are visible simultaneously).
2. Call run(target) — it plans the shortest rotation path and executes each
   step via the injected RotationSchema, updating the tracked state as it goes.
3. Optionally pass a top_verifier callback to confirm each step with a camera.

Integrating with pip_test.py
-----------------------------
The project's pip detection lives in pip_test.py (MindVision camera + HSV/Otsu
pipeline).  Pass a verifier built from that pipeline to run():

    import sys, os
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
    from pip_test import detect_pips, open_camera, grab_frame

    h, buf, is_color = open_camera(0)

    def grab_top():
        frame = grab_frame(h, buf, is_color)
        _, _, _, _, count = detect_pips(frame)
        return count if count > 0 else None

    ctrl = DiceController(schema=MyRobotSchema(...))
    ctrl.set_state_directly(top=3, front=1, right=5)
    result = ctrl.run(target='even', top_verifier=grab_top)
"""

from __future__ import annotations

from typing import Callable, Optional, Union

from .dice_model import (
    DiceState, from_visible_faces, ROTATION_NAMES,
    ALL_ORIENTATIONS, DISCOVERY_FACE_REVEALED,
)
from .rotation_planner import (
    plan_rotation_sequence,
    plan_to_even,
    plan_to_odd,
    plan_to_value,
    choose_discovery_rotation,
)
from .rotation_schema import RotationSchema, NullSchema


class DiceController:
    """
    High-level controller for detecting die orientation and rotating to a target.

    Parameters
    ----------
    schema:
        Robot-specific RotationSchema.  Defaults to NullSchema (dry run / test).
    """

    def __init__(self, schema: Optional[RotationSchema] = None):
        self.schema: RotationSchema         = schema or NullSchema()
        self._state: Optional[DiceState]    = None

    # -------------------------------------------------------------------------
    #  State management
    # -------------------------------------------------------------------------

    @property
    def current_state(self) -> Optional[DiceState]:
        """The currently tracked die orientation, or None if not yet set."""
        return self._state

    def set_state_directly(self, top: int, front: int, right: int) -> DiceState:
        """
        Set the die orientation explicitly.

        Use this when the initial position is known (e.g. the robot always places
        the die the same way, or you have read the three visible faces manually).

        Parameters
        ----------
        top, front, right:
            Pip counts on the corresponding faces (1–6).
        """
        self._state = DiceState(top=top, front=front, right=right)
        return self._state

    def set_state_from_faces(self, **known_faces: int) -> DiceState:
        """
        Resolve die orientation from two or more known face pip counts.

        Keyword arguments are face names ('top', 'bottom', 'front', 'back',
        'left', 'right') mapped to their pip counts.  Three *adjacent* faces
        (e.g. top + front + right) uniquely determine the orientation; two
        opposite faces do not.

        Raises
        ------
        ValueError if the faces are ambiguous or contradict a standard die.

        Example
        -------
            # Camera sees top, front, and right simultaneously
            ctrl.set_state_from_faces(top=3, front=1, right=5)
        """
        state = from_visible_faces(**known_faces)
        if state is None:
            raise ValueError(
                f'Could not uniquely determine die orientation from faces: {known_faces}. '
                'Provide three adjacent faces (top + front + right, or similar).'
            )
        self._state = state
        return state

    # -------------------------------------------------------------------------
    #  Planning
    # -------------------------------------------------------------------------

    def plan(
        self,
        target:            Union[str, int, Callable[[DiceState], bool]],
        allowed_rotations: Optional[tuple[str, ...]] = None,
    ) -> Optional[list[str]]:
        """
        Plan the shortest rotation sequence from the current state to the target.

        Parameters
        ----------
        target:
            'even'   — any even number (2, 4, 6) on top
            'odd'    — any odd number  (1, 3, 5) on top
            int      — that specific pip value on top
            callable — custom predicate: receives DiceState, returns bool

        allowed_rotations:
            Limit the planner to a subset of rotations the robot can perform.
            Defaults to all six.

        Returns
        -------
        List of rotation name strings (empty list if already at goal), or None
        if the target cannot be reached with the allowed rotations.

        Raises
        ------
        RuntimeError if set_state_* has not been called yet.
        """
        if self._state is None:
            raise RuntimeError(
                'Die state is unknown. '
                'Call set_state_directly() or set_state_from_faces() first.'
            )
        ar = tuple(allowed_rotations) if allowed_rotations else ROTATION_NAMES

        target_fn = self._resolve_target_fn(target)
        return plan_rotation_sequence(self._state, target_fn, ar)

    # -------------------------------------------------------------------------
    #  Internal helpers
    # -------------------------------------------------------------------------

    def _resolve_target_fn(
        self, target: Union[str, int, Callable[[DiceState], bool]]
    ) -> Callable[[DiceState], bool]:
        """Convert a target string/int/callable into a DiceState predicate."""
        if target == 'even':
            return lambda s: s.top % 2 == 0
        elif target == 'odd':
            return lambda s: s.top % 2 == 1
        elif isinstance(target, int):
            if not 1 <= target <= 6:
                raise ValueError(f'Target value must be 1–6, got {target}')
            return lambda s, v=target: s.top == v
        elif callable(target):
            return target
        else:
            raise ValueError(
                f"Unrecognised target {target!r}. "
                "Use 'even', 'odd', an int 1–6, or a callable predicate."
            )

    # -------------------------------------------------------------------------
    #  Execution
    # -------------------------------------------------------------------------

    def execute_rotation(self, rotation_name: str) -> bool:
        """
        Execute a single rotation via the schema and update the tracked state.

        Returns True on success.  On failure the tracked state is NOT updated
        so it reflects the last confirmed position.
        """
        if self._state is None:
            raise RuntimeError('Die state unknown; call set_state_* first.')
        ok = self.schema.execute(rotation_name)
        if ok:
            self._state = self._state.apply(rotation_name)
        return ok

    # -------------------------------------------------------------------------
    #  Full pipeline
    # -------------------------------------------------------------------------

    def run(
        self,
        target:            Union[str, int, Callable[[DiceState], bool]],
        allowed_rotations: Optional[tuple[str, ...]] = None,
        top_verifier:      Optional[Callable[[], Optional[int]]] = None,
        execute:           bool = True,
    ) -> dict:
        """
        Plan and (optionally) execute rotations to reach the target.

        Parameters
        ----------
        target:
            See plan() for accepted values.
        allowed_rotations:
            Restrict which rotations the robot may perform.
        top_verifier:
            Optional zero-argument callable that captures a camera frame and
            returns the current top-face pip count (or None on detection failure).
            Called after each rotation to confirm the physical die matches the
            tracked state.  Detection mismatches are logged to the result dict
            but do not halt execution.
        execute:
            If False, plan only — do not move the robot.  Useful for previewing
            the planned sequence before committing.

        Returns
        -------
        dict with keys:
            'initial_state'     — DiceState before any rotation
            'rotation_sequence' — list of rotation names (None if no path found)
            'final_state'       — predicted DiceState after all rotations
            'success'           — True if plan exists and all moves executed OK
            'verification_log'  — list of (rotation, expected_top, detected_top)
                                  entries when top_verifier is provided
        """
        if self._state is None:
            raise RuntimeError('Die state unknown; call set_state_* first.')

        initial = self._state
        sequence = self.plan(target, allowed_rotations)

        result: dict = {
            'initial_state':     initial,
            'rotation_sequence': sequence,
            'final_state':       None,
            'success':           False,
            'verification_log':  [],
        }

        if sequence is None:
            return result   # no path exists

        if not execute:
            # Simulate without moving the robot
            sim = initial
            for rot in sequence:
                sim = sim.apply(rot)
            result['final_state'] = sim
            result['success']     = True
            return result

        # Execute step by step, optionally verifying with camera
        for rot in sequence:
            ok = self.execute_rotation(rot)
            if not ok:
                return result   # robot reported failure; stop here

            if top_verifier is not None:
                detected  = top_verifier()
                expected  = self._state.top
                mismatch  = (detected is not None) and (detected != expected)
                result['verification_log'].append({
                    'rotation':     rot,
                    'expected_top': expected,
                    'detected_top': detected,
                    'match':        not mismatch,
                })
                if mismatch:
                    print(
                        f'[DiceController] WARNING: after {rot!r} expected top={expected}, '
                        f'camera saw top={detected}. State may be out of sync.'
                    )

        result['final_state'] = self._state
        result['success']     = True
        return result

    # -------------------------------------------------------------------------
    #  Unknown-orientation pipeline
    # -------------------------------------------------------------------------

    def discover_and_rotate(
        self,
        target:            Union[str, int, Callable[[DiceState], bool]],
        top_verifier:      Callable[[], Optional[int]],
        allowed_rotations: Optional[tuple[str, ...]] = None,
    ) -> dict:
        """
        Full pipeline for a randomly placed die whose orientation is unknown.

        Algorithm
        ---------
        1. Read top face T1.  Know top + bottom (= 7 - T1) for free.
        2. If T1 already satisfies the target across ALL possible orientations
           with that top value, return immediately (0 rotations).
        3. Select the single best *discovery rotation* — the roll that
           minimises worst-case total moves across the 4 unknown orientations.
        4. Execute the discovery rotation; read new top T2.
           T2 was a hidden side face of the initial state — together with T1,
           top + one side face uniquely determines all 6 faces on a standard die.
        5. Reconstruct the full initial orientation, then compute the current
           state (initial state after the discovery rotation).
        6. Plan the minimum remaining rotations to the target.
        7. Execute them.

        Maximum total rotations:
            even/odd target  → at most 2  (1 discovery + 1 correction)
            specific value   → at most 4  (1 discovery + up to 3)

        Parameters
        ----------
        target:
            'even', 'odd', an int 1–6, or a callable predicate.
        top_verifier:
            Zero-argument callable; captures a camera frame and returns the
            top-face pip count (1–6), or None on detection failure.
            Must work both before and after each robot move.
        allowed_rotations:
            Restrict which rotations the robot can perform.

        Returns
        -------
        dict with keys:
            'initial_top'        — pip count seen before any movement
            'discovery_rotation' — rotation used to reveal a side face (or None)
            'initial_state'      — full DiceState before any movement
            'rotation_sequence'  — every rotation executed (including discovery)
            'final_state'        — DiceState after all rotations
            'success'            — True if target reached without error
            'verification_log'   — camera checks after each rotation
        """
        ar        = tuple(allowed_rotations) if allowed_rotations else ROTATION_NAMES
        target_fn = self._resolve_target_fn(target)

        result: dict = {
            'initial_top':        None,
            'discovery_rotation': None,
            'initial_state':      None,
            'rotation_sequence':  [],
            'final_state':        None,
            'success':            False,
            'verification_log':   [],
        }

        # ---- Step 1: detect top face ----------------------------------------
        T1 = top_verifier()
        if T1 is None:
            print('[DiceController] ERROR: could not detect top face.')
            return result
        result['initial_top'] = T1
        print(f'[DiceController] Detected top face: {T1}')

        # ---- Step 2: early exit if already at target ------------------------
        # A target that only depends on .top (even/odd/specific value) is
        # satisfied regardless of the unknown orientation if T1 satisfies it.
        possible_initial = [s for s in ALL_ORIENTATIONS if s.top == T1]
        if all(target_fn(s) for s in possible_initial):
            print(f'[DiceController] Top={T1} already satisfies target. Done.')
            result['success'] = True
            return result

        # ---- Step 3: choose optimal discovery rotation ----------------------
        disc_rot = choose_discovery_rotation(T1, target_fn, ar)
        result['discovery_rotation'] = disc_rot
        print(f'[DiceController] Discovery rotation chosen: {disc_rot}')

        # ---- Step 4: execute discovery rotation and read new top ------------
        ok = self.schema.execute(disc_rot)
        if not ok:
            print(f'[DiceController] ERROR: discovery rotation {disc_rot!r} failed.')
            return result

        T2 = top_verifier()
        if T2 is None:
            print('[DiceController] ERROR: could not detect top face after discovery rotation.')
            return result
        print(f'[DiceController] After {disc_rot}: new top = {T2}  '
              f'(= initial {DISCOVERY_FACE_REVEALED[disc_rot]} face)')

        result['verification_log'].append({
            'rotation':     disc_rot,
            'detected_top': T2,
        })

        # ---- Step 5: reconstruct full orientation ---------------------------
        revealed_face = DISCOVERY_FACE_REVEALED[disc_rot]
        initial_state = from_visible_faces(top=T1, **{revealed_face: T2})

        if initial_state is None:
            print(
                f'[DiceController] ERROR: top={T1}, {revealed_face}={T2} does not match '
                'any valid die orientation. Check camera calibration or die chirality.'
            )
            return result

        result['initial_state'] = initial_state
        self._state = initial_state.apply(disc_rot)   # current state after discovery
        print(f'[DiceController] Initial state: {initial_state}')
        print(f'[DiceController] Current state: {self._state}')

        # ---- Step 6: plan remaining rotations --------------------------------
        remaining = plan_rotation_sequence(self._state, target_fn, ar)
        if remaining is None:
            print('[DiceController] ERROR: target unreachable from current state.')
            return result

        full_sequence = [disc_rot] + remaining
        result['rotation_sequence'] = full_sequence
        print(f'[DiceController] Full rotation sequence: {full_sequence}')

        # ---- Step 7: execute remaining rotations ----------------------------
        for rot in remaining:
            ok = self.execute_rotation(rot)
            if not ok:
                print(f'[DiceController] ERROR: rotation {rot!r} failed.')
                return result

            detected = top_verifier()
            expected = self._state.top
            mismatch = (detected is not None) and (detected != expected)
            result['verification_log'].append({
                'rotation':     rot,
                'expected_top': expected,
                'detected_top': detected,
                'match':        not mismatch,
            })
            if mismatch:
                print(
                    f'[DiceController] WARNING: after {rot!r} expected top={expected}, '
                    f'camera saw top={detected}.'
                )

        result['final_state'] = self._state
        result['success']     = True
        print(f'[DiceController] Done. Final top: {self._state.top}')
        return result
