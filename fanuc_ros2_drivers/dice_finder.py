"""
dice_finder.py
==============
Simple functions to identify a die's orientation and rotate it to target pip values.

How it works in three steps
----------------------------
1. Look at the top face.  The bottom is always 7 - top, so two faces are known for free.
2. Roll once.  The face that comes up was a hidden side face.
   Top + any one side face uniquely identifies all 6 faces on a standard die.
3. Plan the shortest path to each target pip using BFS over the 24 possible orientations.

You don't call the BFS or model directly — just call run_sequence() from your robot file.
"""

# Die orientation math (24 orientations, 6 rotation names)
from dice_vision.dice_model import from_visible_faces

# BFS planner — finds shortest rotation list to reach a target pip
from dice_vision.rotation_planner import plan_rotation_sequence


# ── Pip sequences ────────────────────────────────────────────────────────────

ODD_SEQUENCE  = [1, 3, 5]   # Robot 1 finds pips in this order
EVEN_SEQUENCE = [2, 4, 6]   # Robot 2 finds pips in this order


# ── Core functions ───────────────────────────────────────────────────────────

def discover_orientation(initial_top, camera_fn, execute_fn):
    """
    Roll once to reveal a hidden side face, then reconstruct the full die state.

    We already know:  top = initial_top,  bottom = 7 - initial_top
    After rolling forward:  the old FRONT face becomes the new top.
    top + front uniquely identifies every face on a standard die.

    Returns the DiceState AFTER the discovery roll (i.e. our current position).

    Parameters
    ----------
    initial_top : int
        Pip count seen on top BEFORE any movement.
    camera_fn : callable
        Returns the current top pip count (1-6), called after the roll.
    execute_fn : callable
        Takes a rotation name string and moves the robot.
    """
    execute_fn('roll_forward')        # roll the die — front face comes to top

    new_top = camera_fn()             # what's on top now = what was on the front
    if new_top is None:
        raise RuntimeError("Camera failed to detect pip after discovery roll.")

    # Reconstruct the die's orientation just before the roll
    initial_state = from_visible_faces(top=initial_top, front=new_top)
    if initial_state is None:
        raise RuntimeError(
            f"top={initial_top} + front={new_top} doesn't match any valid die. "
            "Check that the camera is reading the correct face."
        )

    # Apply the roll we already did to get the current state
    current_state = initial_state.apply('roll_forward')
    return current_state


def rotate_to_pip(target_pip, current_state, execute_fn, camera_fn=None):
    """
    Rotate the die to put target_pip on top using the minimum number of moves.

    Uses BFS to find the shortest sequence, then executes each rotation
    one at a time, updating the tracked state after each step.

    Parameters
    ----------
    target_pip : int
        The pip value (1-6) to place on top.
    current_state : DiceState
        The die's current orientation (kept in sync with the physical die).
    execute_fn : callable
        Takes a rotation name string and moves the robot.
    camera_fn : callable, optional
        If provided, reads the top pip after each move for a sanity check.

    Returns the updated DiceState after all rotations.
    """
    # BFS: find the shortest list of rotation names that puts target_pip on top
    moves = plan_rotation_sequence(
        current_state,
        lambda state: state.top == target_pip   # goal: target pip on top
    )

    if moves is None:
        raise RuntimeError(f"Pip {target_pip} is unreachable. Die model may be out of sync.")

    print(f"  Planned moves: {moves}")

    for move in moves:
        execute_fn(move)                             # physically rotate the die
        current_state = current_state.apply(move)   # update our model to match

        if camera_fn is not None:                   # optional camera check
            seen = camera_fn()
            ok   = "OK" if seen == current_state.top else "MISMATCH"
            print(f"  After '{move}': model says top={current_state.top}, camera says {seen} [{ok}]")

    return current_state


def run_sequence(pip_sequence, camera_fn, execute_fn):
    """
    Work through pip_sequence in order from a random starting position.

    Automatically discovers the die's orientation with one roll, then
    plans the minimum rotations for each pip in the list.

    Add your per-pip action (gripper, signal, etc.) in the marked spot below.

    Parameters
    ----------
    pip_sequence : list[int]
        Pip values to find in order — e.g. ODD_SEQUENCE or EVEN_SEQUENCE.
    camera_fn : callable
        Returns top pip count (1-6), or None on detection failure.
    execute_fn : callable
        Takes a rotation name ('roll_forward', 'roll_right', etc.) and moves the robot.
    """

    # ── Step 1: read the starting top face ───────────────────────────────────
    print("Reading die...")
    initial_top = camera_fn()
    if initial_top is None:
        raise RuntimeError("Could not detect top pip. Check camera and lighting.")
    print(f"  Top: {initial_top}   Bottom (free): {7 - initial_top}")

    # ── Step 2: one roll reveals everything ──────────────────────────────────
    print("Discovering orientation (1 roll)...")
    state = discover_orientation(initial_top, camera_fn, execute_fn)
    print(f"  Full state — top:{state.top}  front:{state.front}  right:{state.right}  "
          f"bottom:{state.bottom}  back:{state.back}  left:{state.left}")

    # ── Step 3: find each target pip in order ────────────────────────────────
    for target in pip_sequence:
        print(f"\n{'─'*40}")
        print(f"Target pip: {target}")

        if state.top == target:
            # The gripper's return from camera pose rotates the die so what the model
            # tracks as 'top' is physically at the front.  Roll_forward (front→top)
            # corrects the drift.  After the roll we re-check and route from there.
            print("  Model says already on top; rolling forward to correct camera-pose drift.")
            execute_fn('roll_forward')
            state = state.apply('roll_forward')

        if state.top != target:
            state = rotate_to_pip(target, state, execute_fn, camera_fn)

        print(f"  Pip {target} is on top.")

        # ── ADD YOUR ACTION HERE ─────────────────────────────────────────────
        # This runs every time the correct pip is on top.
        # Examples:
        #   bot.schunk_gripper('close')
        #   send_signal_to_next_robot()
        #   input("Press Enter to continue...")   ← uncomment to step manually
        # ────────────────────────────────────────────────────────────────────

    print("\nSequence complete.")


def find_pip(target_pip, camera_fn, execute_fn, current_state=None):
    """
    Rotate the die to show target_pip on top.  Importable and callable from any file.

    If current_state is None  →  assumes unknown starting position, does one
                                 discovery roll to figure out the full orientation first.
    If current_state is given →  skips discovery and goes directly to the target.
                                 Use this when chaining multiple calls so you don't
                                 waste a roll re-discovering an already-known state.

    Returns the DiceState after the move (pass it into the next call).

    Examples
    --------
    # From an unknown starting position:
    state = find_pip(1, detect_top, execute)

    # Chain calls — no extra discovery rolls:
    state = find_pip(1, detect_top, execute)
    state = find_pip(3, detect_top, execute, current_state=state)
    state = find_pip(5, detect_top, execute, current_state=state)
    """
    if current_state is None:
        initial_top  = camera_fn()                                  # read the top face
        current_state = discover_orientation(initial_top, camera_fn, execute_fn)

    if target_pip == 7 - current_state.top:
        # target is the bottom face — opposite faces sum to 7, so no need to
        # inspect a second side; two forward rolls always bring bottom to top
        print(f"  Target {target_pip} is opposite top {current_state.top} — rolling twice.")
        execute_fn('roll_forward')
        execute_fn('roll_forward')
        return current_state.apply('roll_forward').apply('roll_forward')

    return rotate_to_pip(target_pip, current_state, execute_fn, camera_fn)
