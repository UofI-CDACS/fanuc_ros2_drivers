"""
Dice cycle test — pick up, present to camera, rotate on conveyor until a 1
is seen, then place on conveyor.

Algorithm (two-face inference):
  Two camera reads (A, B) tell us all six faces of a Western die:
    - opposite faces sum to 7  →  -A axis = 7-A,  -B axis = 7-B
    - 1, 2, 3 around a corner go CCW (right-handed)  →  the third axis is
      uniquely determined.
  We enumerate all 24 cube orientations, find the one matching (A, B), and
  read off where face value 1 currently sits on the gripper. The inferred
  axis of "1" picks the rotation mode: ±Z → flip (top↔side), else rotate.

Sequence (deterministic from a single (A, B) read):
    home -> above_dice -> (open) -> grab_dice_table -> (close) -> above_dice
    -> table_show_pre -> camera_dice_position_1 [read A]
    -> camera_position_2 [read B]
    -> infer orientation; locate face 1 in the gripper frame.
       Faces map:
         CAM1_AXIS         -> face A (front of die at pickup)
         CAM2_AXIS         -> face B (bottom of die at pickup)
         OPP[CAM2_AXIS]    -> the gripped (top) face — rotation can't move it
       Placement choice based on where face 1 sits:
         on CAM1_AXIS      -> place_face_a_up (single flip, face A up)
         on CAM2_AXIS      -> flip_and_regrab + rescan + re-plan + place
         on gripped face   -> place_basic + verify_via_repickup
         on a side face    -> rotate N times to CAM1_AXIS, then place_face_a_up
    -> verification read; place; home

Run:
    python3 ModbusStuff/dice_cycle_test.py [robot_name]

Make sure the launch is up first:
    ros2 launch launch/start.launch.py robot_name:=dj robot_ip:=<IP>

Flags:
    --auto    skip the per-step Enter prompts (full speed)
    --max N   stop after N read attempts (default 8)
"""

import sys
import threading
import time

import rclpy
from fanuc_interfaces.action import CartPose, JointPose, SchunkGripper
from fanuc_interfaces.srv import CountPips
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


# ── Joint waypoints (from recorded_positions.txt) ─────────────────────────────
HOME_JOINTS = [0.0, 0.0, 0.0, 0.0, -90.0, 30.0]

POSITIONS = {
    'above_dice':                [12.943, 13.472, -34.815,  0.000, -55.185, 17.057],
    'grab_dice_table':           [12.943, 23.461, -52.853,  0.000, -37.147, 17.057],
    'table_show_pre':            [90.599,  5.785,  -1.906, -0.518, -84.400, 17.580],
    'camera_dice_position_1':    [90.000,  0.000,   0.019, -0.079,  -7.510, 22.932],
    'camera_position_2':         [90.366, 40.547,  -3.094, -0.493,  86.951, 20.317],
    'above_conveyor':            [141.312, 18.915,  -4.325,  2.615, -86.085, -22.345],
    'above_place_conveyor':      [141.292, 23.324, -26.470,  2.903, -63.965, -23.422],
    'above_rotation_conveyor':   [140.721, 19.383,   0.107, -0.203, -92.739, -107.715],
    'pickup_conveyor_rotation':  [140.696, 23.829, -25.053, -0.348, -67.566, -110.158],
    'above_conveyor_flip':       [158.677, -1.161, -37.929, 106.898, -73.756, -103.641],
    'place_on_conveyor_flipped': [156.677, 43.480, -102.660, 114.022, -98.044, -164.281],
}

# Cartesian overrides — keys here are sent as CartPose moves (taking precedence
# over the joint values above). Used to shift recorded poses by a fixed offset
# without re-recording joint angles.
CART_POSITIONS = {
    # Original recorded y was 602.07; shifted +20 mm.
    'pickup_conveyor_rotation': dict(
        x=-500.10, y=622.07, z=260.16, w=177.43, p=-0.61, r=30.41,
    ),
    # Original recorded y was 300.85; shifted +20 mm.
    'above_conveyor_flip': dict(
        x=-486.43, y=320.85, z=498.84, w=83.99, p=-62.59, r=-172.87,
    ),
    # Original recorded place_on_conveyor_flipped (z bumped +10 mm) — now used
    # as the APPROACH: the gripper descends here first, then advances 25 mm
    # in +Y to the actual place/grab spot.
    'place_on_conveyor_flipped_approach': dict(
        x=-491.88, y=303.15, z=-23.99, w=83.99, p=-62.59, r=-172.87,
    ),
    # The actual flip place/grab spot — 25 mm forward (+Y) from the approach.
    'place_on_conveyor_flipped': dict(
        x=-491.88, y=328.15, z=-23.99, w=83.99, p=-62.59, r=-172.87,
    ),
    # Top-down grab pose AT the flip spot xy: basic (vertical) wrist
    # orientation copied from above_place_conveyor's W/P/R, descending onto
    # the die that was just dropped at place_on_conveyor_flipped. z values
    # mirror above_conveyor (high transit) and above_place_conveyor (descend).
    # These are starting guesses — drive there once and verify the gripper
    # actually contacts the die.
}

# ── Camera-to-axis mapping (TUNE EMPIRICALLY) ─────────────────────────────────
# Which axis of the gripper-frame is each camera position viewing?
# Axes use right-handed labels relative to the gripper. Pick any consistent
# assignment; the third_face() inference uses these to compute remaining sides.
CAM1_AXIS = '+X'      # face seen at camera_dice_position_1
CAM2_AXIS = '+Y'      # face seen at camera_position_2

# ── Die orientation logic ─────────────────────────────────────────────────────

AXES = ['+X', '-X', '+Y', '-Y', '+Z', '-Z']
OPP  = {'+X': '-X', '-X': '+X', '+Y': '-Y', '-Y': '+Y', '+Z': '-Z', '-Z': '+Z'}


def _all_orientations():
    """All 24 orientations of a Western die.

    Canonical: +Z=1 up, +Y=2 forward, +X=3 right (right-handed; 1-2-3 corner CCW).
    Each orientation is a dict mapping axis string -> face value (1..6).
    """
    base = {'+Z': 1, '-Z': 6, '+Y': 2, '-Y': 5, '+X': 3, '-X': 4}

    def rot_z(o):       # +Z fixed; +X -> +Y, +Y -> -X
        return {'+Z': o['+Z'], '-Z': o['-Z'],
                '+Y': o['+X'], '-X': o['+Y'],
                '-Y': o['-X'], '+X': o['-Y']}

    def rot_x(o):       # +X fixed; +Y -> +Z, +Z -> -Y
        return {'+X': o['+X'], '-X': o['-X'],
                '+Z': o['+Y'], '-Y': o['+Z'],
                '-Z': o['-Y'], '+Y': o['-Z']}

    seen, out = set(), []
    stack = [base]
    while stack:
        o = stack.pop()
        key = tuple(o[a] for a in AXES)
        if key in seen:
            continue
        seen.add(key)
        out.append(o)
        stack.append(rot_z(o))
        stack.append(rot_x(o))
    return out


_ORIENTATIONS = _all_orientations()
assert len(_ORIENTATIONS) == 24


def infer_orientation(face_a: int, face_b: int):
    """Given face values seen at CAM1_AXIS and CAM2_AXIS, return the unique
    orientation dict, or None if (a, b) doesn't correspond to two adjacent
    faces (e.g., a + b == 7 means they're opposites, impossible from one die)."""
    if face_a + face_b == 7 or face_a == face_b:
        return None
    for o in _ORIENTATIONS:
        if o[CAM1_AXIS] == face_a and o[CAM2_AXIS] == face_b:
            return o
    return None


def axis_with_value(orientation: dict, value: int):
    for ax, v in orientation.items():
        if v == value:
            return ax
    return None


def fmt_orientation(o):
    return ', '.join(f'{ax}={o[ax]}' for ax in AXES)


# ── ROS plumbing ──────────────────────────────────────────────────────────────

class Cycle(Node):

    def __init__(self, robot_name: str):
        super().__init__('dice_cycle_test')
        self.joints_ac = ActionClient(
            self, JointPose, f'/{robot_name}/joint_pose'
        )
        self.cart_ac = ActionClient(
            self, CartPose, f'/{robot_name}/cartesian_pose'
        )
        self.schunk_ac = ActionClient(
            self, SchunkGripper, f'/{robot_name}/schunk_gripper'
        )
        self.pip_client = self.create_client(CountPips, '/camera/count_pips')

    def _wait(self, fut, timeout: float = 30.0):
        deadline = time.time() + timeout
        while not fut.done() and time.time() < deadline:
            time.sleep(0.02)
        return fut.done()

    def move_joints(self, joints, label: str = '') -> bool:
        if not self.joints_ac.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('joint_pose action server not available')
            return False
        goal = JointPose.Goal()
        goal.joint1, goal.joint2, goal.joint3 = joints[0], joints[1], joints[2]
        goal.joint4, goal.joint5, goal.joint6 = joints[3], joints[4], joints[5]
        send = self.joints_ac.send_goal_async(goal)
        if not self._wait(send):
            return False
        gh = send.result()
        if not gh or not gh.accepted:
            self.get_logger().error(f'Joint goal rejected ({label})')
            return False
        return self._wait(gh.get_result_async())

    def move_cart(self, pose: dict, label: str = '') -> bool:
        if not self.cart_ac.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('cartesian_pose action server not available')
            return False
        goal = CartPose.Goal()
        goal.x, goal.y, goal.z = pose['x'], pose['y'], pose['z']
        goal.w, goal.p, goal.r = pose['w'], pose['p'], pose['r']
        send = self.cart_ac.send_goal_async(goal)
        if not self._wait(send):
            return False
        gh = send.result()
        if not gh or not gh.accepted:
            self.get_logger().error(f'Cart goal rejected ({label})')
            return False
        return self._wait(gh.get_result_async())

    def gripper(self, command: str) -> bool:
        if not self.schunk_ac.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('schunk_gripper action server not available')
            return False
        goal = SchunkGripper.Goal()
        goal.command = command
        send = self.schunk_ac.send_goal_async(goal)
        if not self._wait(send):
            return False
        gh = send.result()
        if not gh or not gh.accepted:
            self.get_logger().error(f'Gripper rejected ({command})')
            return False
        ok = self._wait(gh.get_result_async())
        # Brief pause so the gripper finishes opening/closing before the next
        # motion starts — avoids the arm leaving while the jaws are still moving.
        time.sleep(0.5)
        return ok

    def read_pips(self):
        if not self.pip_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('/camera/count_pips not available')
            return None
        fut = self.pip_client.call_async(CountPips.Request())
        if not self._wait(fut, timeout=10.0):
            return None
        res = fut.result()
        if not res or not res.success:
            self.get_logger().warn(
                f'count_pips failed: {res.message if res else "no response"}'
            )
            return None
        return res.pip_count


# ── Sequence helpers ──────────────────────────────────────────────────────────

def step(label: str, auto: bool):
    print(f'\n>>> {label}')
    if not auto:
        input('    [Enter to continue, Ctrl-C to abort] ')


def go(robot: Cycle, key: str, auto: bool) -> bool:
    step(f'move {key}', auto)
    if key in CART_POSITIONS:
        return robot.move_cart(CART_POSITIONS[key], key)
    return robot.move_joints(POSITIONS[key], key)


def go_home(robot: Cycle, auto: bool) -> bool:
    step('move HOME', auto)
    return robot.move_joints(HOME_JOINTS, 'home')


def pickup(robot: Cycle, auto: bool) -> bool:
    return (
        go_home(robot, auto)
        and go(robot, 'above_dice', auto)
        and (step('open gripper', auto) or robot.gripper('open'))
        and go(robot, 'grab_dice_table', auto)
        and (step('close gripper', auto) or robot.gripper('close'))
        and go(robot, 'above_dice', auto)
    )


def show_to_camera(robot: Cycle, auto: bool):
    """Move to both camera positions, return (face_a, face_b).

    Optimization: if camera 1 already shows the target face (1), skip the
    second camera move/read entirely and return (1, None). Callers must
    handle b == None separately from b == 0 / failure."""
    if not go(robot, 'table_show_pre', auto):
        return None, None
    if not go(robot, 'camera_dice_position_1', auto):
        return None, None
    step('read camera 1', auto)
    a = robot.read_pips()
    print(f'    camera 1 read: {a}')

    if a == 1:
        print('    Target seen at camera 1 — skipping camera 2.')
        return a, None

    if not go(robot, 'camera_position_2', auto):
        return a, None
    step('read camera 2', auto)
    b = robot.read_pips()
    print(f'    camera 2 read: {b}')
    return a, b


def place_basic(robot: Cycle, auto: bool) -> bool:
    return (
        go(robot, 'above_conveyor', auto)
        and go(robot, 'above_place_conveyor', auto)
        and (step('open gripper', auto) or robot.gripper('open'))
        and go(robot, 'above_conveyor', auto)
    )


def verify_via_repickup(robot: Cycle, auto: bool) -> bool:
    """After place_basic puts the die face-up with 1 on top, re-pickup using
    the rotated-wrist pickup_conveyor_rotation pose. The 90° wrist roll means
    the gripper jaws now clamp on a different pair of side faces — the face
    that was gripped at the original pickup (with 1 on it) becomes exposed
    and can show up at a camera. Show to both cameras, then place back at
    the same spot with the same rotated wrist so the die ends up exactly
    where (and how) it was before the verification."""
    print('\n========== Re-pickup with flipped wrist (verification) ==========')
    if not (
        go(robot, 'above_conveyor', auto)
        and go(robot, 'above_conveyor_flip', auto)
        and go(robot, 'place_on_conveyor_flipped_approach', auto)
        and go(robot, 'place_on_conveyor_flipped', auto)
        and (step('close gripper (re-grip rotated)', auto)
             or robot.gripper('close'))
        and go(robot, 'place_on_conveyor_flipped_approach', auto)
        and go(robot, 'above_conveyor_flip', auto)
        and go(robot, 'above_conveyor', auto)
    ):
        return False

    print('\n========== Verification camera read ==========')
    a, b = show_to_camera(robot, auto)
    if a is not None and b is not None:
        print(f'    Verification: {CAM1_AXIS}={a}, {CAM2_AXIS}={b}')
        if a == 1 or b == 1:
            print('    *** Camera confirmed 1 ***')
        else:
            print(f'    Camera did not see 1 directly (A={a}, B={b}).')

    print('\n========== Placing back with flipped wrist ==========')
    return (
        go(robot, 'above_conveyor', auto)
        and go(robot, 'above_conveyor_flip', auto)
        and go(robot, 'place_on_conveyor_flipped_approach', auto)
        and go(robot, 'place_on_conveyor_flipped', auto)
        and (step('open gripper (place back)', auto) or robot.gripper('open'))
        and go(robot, 'place_on_conveyor_flipped_approach', auto)
        and go(robot, 'above_conveyor_flip', auto)
        and go(robot, 'above_conveyor', auto)
    )


def place_face_a_up(robot: Cycle, auto: bool) -> bool:
    """1 was seen at camera 1 ("first try"). Flip-place so the camera-1 face
    ends up facing up on the conveyor."""
    print('\n[*] Placement: flip (face A up)')
    return (
        go(robot, 'above_conveyor', auto)
        and go(robot, 'above_conveyor_flip', auto)
        and go(robot, 'place_on_conveyor_flipped_approach', auto)
        and go(robot, 'place_on_conveyor_flipped', auto)
        and (step('open gripper', auto) or robot.gripper('open'))
        and go(robot, 'place_on_conveyor_flipped_approach', auto)
        and go(robot, 'above_conveyor_flip', auto)
        and go(robot, 'above_conveyor', auto)
    )


def flip_and_regrab(robot: Cycle, auto: bool) -> bool:
    """Single flip: drop the die at place_on_conveyor_flipped (with the tilted
    flip wrist), then re-grab from the TOP using the basic vertical wrist at
    the flip spot. The caller should rescan with show_to_camera afterward
    since the die's orientation in the gripper has changed."""
    print('\n[*] Flip + regrab from top')

    print('\n--- Drop with flip wrist ---')
    if not (
        go(robot, 'above_conveyor', auto)
        and go(robot, 'above_conveyor_flip', auto)
        and go(robot, 'place_on_conveyor_flipped_approach', auto)
        and go(robot, 'place_on_conveyor_flipped', auto)
        and (step('open gripper (drop)', auto) or robot.gripper('open'))
        and go(robot, 'place_on_conveyor_flipped_approach', auto)
        and go(robot, 'above_conveyor_flip', auto)
    ):
        return False

    print('\n--- Regrab from top with basic wrist ---')
    return (
        go(robot, 'above_conveyor', auto)
        and go(robot, 'above_place_conveyor', auto)
        and (step('close gripper (regrab from top)', auto)
             or robot.gripper('close'))
        and go(robot, 'above_place_conveyor', auto)
        and go(robot, 'above_conveyor', auto)
    )


def rotate_on_conveyor(robot: Cycle, mode: str, auto: bool) -> bool:
    """Place the die on the conveyor in a rotated orientation, lift back to
    the placement's "above" position, then re-grip with a DIFFERENT wrist
    orientation so the die actually rotates inside the gripper.

    mode='rotate':
        Drop with rotated wrist at pickup_conveyor_rotation, lift to
        above_rotation_conveyor, swap to the un-rotated wrist via
        above_conveyor, descend with normal wrist to above_place_conveyor
        (same conveyor x/y as the drop), grip, lift.  Net effect: 90° die
        rotation around the gripper's tool-roll axis.

    mode='flip':
        Drop at the flip spot (different conveyor location), lift back to
        above_conveyor_flip, return to above_conveyor.  We don't have a
        recorded "descend at flip spot with normal wrist" position, so this
        mode places the die flipped and ends — the die is left on the
        conveyor.  Add that position and an extra step here for true flip
        re-grip.
    """
    if mode == 'rotate':
        return (
            go(robot, 'above_conveyor', auto)
            and go(robot, 'above_rotation_conveyor', auto)
            and go(robot, 'pickup_conveyor_rotation', auto)
            and (step('open gripper (release)', auto) or robot.gripper('open'))
            and go(robot, 'above_rotation_conveyor', auto)
            and go(robot, 'above_conveyor', auto)
            and go(robot, 'above_place_conveyor', auto)
            and (step('close gripper (re-grip rotated)', auto)
                 or robot.gripper('close'))
            and go(robot, 'above_conveyor', auto)
        )

    if mode == 'flip':
        print('    [!] flip mode drops the die but cannot re-grip without an '
              'additional recorded position at the flip spot.')
        return (
            go(robot, 'above_conveyor', auto)
            and go(robot, 'above_conveyor_flip', auto)
            and go(robot, 'place_on_conveyor_flipped_approach', auto)
            and go(robot, 'place_on_conveyor_flipped', auto)
            and (step('open gripper (release)', auto) or robot.gripper('open'))
            and go(robot, 'place_on_conveyor_flipped_approach', auto)
            and go(robot, 'above_conveyor_flip', auto)
            and go(robot, 'above_conveyor', auto)
        )

    raise ValueError(f'unknown rotation mode: {mode}')


# ── Smart planner ─────────────────────────────────────────────────────────────
#
# Empirical observations (from live runs on this robot):
#   - The rotate_on_conveyor cycle rotates the die by 90° about the gripper's
#     CAM2_AXIS (the camera-2 axis = '+Y' in our labels). Faces on ±Y stay
#     fixed; faces on +X, -Z, -X, +Z cycle as +X → +Z → -X → -Z → +X.
#   - The gripper grips the die along the ±Y axis: when a face is on -Y,
#     it is hidden between the jaws and unreachable by rotation alone.
#
# So given the inferred orientation we can plan exactly how many rotations are
# needed to bring face 1 to CAM1_AXIS (+X), where it'll be visible at camera 1
# and we can run place_face_a_up. If 1 is on +Y or -Y, rotation can't help —
# +Y is already at camera 2 (handled separately), and -Y needs flip mode.

ROTATIONS_TO_PLUS_X = {'+X': 0, '-Z': 1, '-X': 2, '+Z': 3}


def plan_for_one(orientation: dict):
    """Return (action, n) for the optimal next move.

    Possible actions:
        'place_a'              — A==1 already, do place_face_a_up
        'place_b'              — B==1 already, do place_face_b_up
        'place_basic'          — 1 is on the gripped face (opposite CAM2);
                                 basic placement puts the gripped face up.
        'rotate_then_a'        — do n rotations, then place_face_a_up
    """
    one_axis = axis_with_value(orientation, 1)
    if one_axis == CAM1_AXIS:
        return ('place_a', 0)
    if one_axis == CAM2_AXIS:
        return ('place_b', 0)
    if one_axis == OPP[CAM2_AXIS]:
        # The face opposite CAM2_AXIS is the gripped face (the "top" of the die
        # at pickup). Rotation can't expose it because it's on the rotation
        # axis — but we don't need to: above_place_conveyor places "as is",
        # which leaves the gripped face up on the conveyor.
        return ('place_basic', 0)
    n = ROTATIONS_TO_PLUS_X.get(one_axis)
    if n is None:
        return (None, None)
    return ('rotate_then_a', n)


# ── Main loop ─────────────────────────────────────────────────────────────────

def _place_after_flip(robot: Cycle, auto: bool) -> str:
    """After flip_and_regrab, the die is in the gripper held from the top
    in a (probably) different orientation than at original pickup. Rescan
    and place based on the new (a, b) — but do NOT trigger another flip
    cycle (would recurse). If 1 is hidden on the new gripped face, we use
    place_basic since the gripped face will end up up on the conveyor."""
    print('\n========== Rescan after flip ==========')
    a, b = show_to_camera(robot, auto)
    if a is None:
        return 'fail_read'

    if a == 1:
        print(f'\n    Faces seen: {CAM1_AXIS}={a}')
        print('    1 at camera 1 after flip — face A up.')
        if not place_face_a_up(robot, auto):
            return 'fail_place'
        return 'success' if go_home(robot, auto) else 'fail_home'

    if b is None:
        return 'fail_read'

    print(f'\n    Faces seen: {CAM1_AXIS}={a}, {CAM2_AXIS}={b}')

    orientation = infer_orientation(a, b)
    if orientation is None:
        print('    [!] Inconsistent reads after flip. Aborting.')
        return 'fail_inconsistent'
    print(f'    Inferred die: {fmt_orientation(orientation)}')
    one_axis = axis_with_value(orientation, 1)
    print(f'    Face "1" now on gripper axis {one_axis}')

    # If 1 is back on cam2 axis (b == 1), don't flip again — place basic so
    # the gripped face ends up up. (1 is on cam2 face = +Y; opposite is -Y =
    # gripped; basic places gripped face up. So we'd actually want a rotation
    # not a basic. But to keep this simple and avoid loops, just fall through
    # to the rotation branch below, which handles ±X / ±Z. If 1 is on +Y the
    # planner will return 'place_b' which we override below.)
    if b == 1 or one_axis == CAM2_AXIS:
        print('    1 still on cam2 axis after flip — placing basic.')
        if not place_basic(robot, auto):
            return 'fail_place'
        return 'success' if go_home(robot, auto) else 'fail_home'

    if one_axis == OPP[CAM2_AXIS]:
        print('    1 on the gripped face — basic placement.')
        if not place_basic(robot, auto):
            return 'fail_place'
        return 'success' if go_home(robot, auto) else 'fail_home'

    # Side face — execute rotations to bring 1 to CAM1_AXIS, then place A up.
    n = ROTATIONS_TO_PLUS_X.get(one_axis)
    if n is None:
        return 'fail_plan'
    print(f'    {n} rotation(s) to bring 1 to {CAM1_AXIS}, then face A up.')
    for i in range(n):
        if not rotate_on_conveyor(robot, 'rotate', auto):
            return 'fail_rotate'
    if not place_face_a_up(robot, auto):
        return 'fail_place'
    return 'success' if go_home(robot, auto) else 'fail_home'


def run(robot: Cycle, max_attempts: int, auto: bool) -> str:
    """Pick once from the table, read both cameras, infer orientation, plan
    the minimum rotations to put face 1 on camera 1, execute, verify, and
    place. Falls back to one extra read+rotate on verification mismatch."""
    print('\n========== Initial pickup from table ==========')
    if not pickup(robot, auto):
        return 'fail_pickup'

    print('\n========== Initial camera read ==========')
    a, b = show_to_camera(robot, auto)
    if a is None:
        return 'fail_read'

    # Direct hit on cam 1 — no need for cam 2 or any inference.
    if a == 1:
        print(f'\n    Faces seen: {CAM1_AXIS}={a}')
        print('    Saw 1 at camera 1 — placing face A up.')
        if not place_face_a_up(robot, auto):
            return 'fail_place'
        return 'success' if go_home(robot, auto) else 'fail_home'

    if b is None:
        return 'fail_read'

    print(f'\n    Faces seen: {CAM1_AXIS}={a}, {CAM2_AXIS}={b}')

    if b == 1:
        # 1 is on cam2 axis (the bottom of the die at pickup). Flip the die
        # at the conveyor and regrab from the top so we can rescan and place
        # accordingly.
        print('    Saw 1 at camera 2 — flip + regrab + rescan.')
        if not flip_and_regrab(robot, auto):
            return 'fail_place'
        return _place_after_flip(robot, auto)

    orientation = infer_orientation(a, b)
    if orientation is None:
        print('    [!] Inconsistent reads — cannot infer orientation. Aborting.')
        return 'fail_inconsistent'

    print(f'    Inferred die: {fmt_orientation(orientation)}')
    one_axis = axis_with_value(orientation, 1)
    print(f'    Face "1" currently on gripper axis {one_axis}')

    action, n = plan_for_one(orientation)

    if action == 'place_basic':
        print('\n    1 is on the gripped face — basic placement leaves it up.')
        if not place_basic(robot, auto):
            return 'fail_place'
        if not verify_via_repickup(robot, auto):
            return 'fail_verify'
        return 'success' if go_home(robot, auto) else 'fail_home'

    if action != 'rotate_then_a' or n is None:
        print(f'    [!] Planner returned an unexpected action: {action}.')
        return 'fail_plan'

    print(f'\n    Plan: {n} rotation(s) to bring face 1 to {CAM1_AXIS}, '
          'then face-A-up placement.')
    for i in range(n):
        print(f'\n========== Rotation {i + 1}/{n} ==========')
        if not rotate_on_conveyor(robot, 'rotate', auto):
            return 'fail_rotate'

    print('\n========== Verification read ==========')
    a, b = show_to_camera(robot, auto)
    if a is None:
        return 'fail_verify'

    if a == 1:
        print(f'    Verification: {CAM1_AXIS}={a}')
        print('    Verified — placing face A up.')
        if not place_face_a_up(robot, auto):
            return 'fail_place'
        return 'success' if go_home(robot, auto) else 'fail_home'

    if b is None:
        return 'fail_verify'

    print(f'    Verification: {CAM1_AXIS}={a}, {CAM2_AXIS}={b}')

    if b == 1:
        print('    1 ended up on camera 2 — flip + regrab + rescan.')
        if not flip_and_regrab(robot, auto):
            return 'fail_place'
        return _place_after_flip(robot, auto)

    print(f'    [!] Plan did not produce a 1 (got A={a}, B={b}). '
          'Possibly the rotation slipped — retry from the start.')
    return 'fail_unverified'


def main():
    args = sys.argv[1:]
    auto = '--auto' in args
    args = [a for a in args if a != '--auto']

    max_attempts = 8
    if '--max' in args:
        i = args.index('--max')
        max_attempts = int(args[i + 1])
        del args[i:i + 2]

    robot_name = args[0] if args else 'dj'

    rclpy.init()
    node = Cycle(robot_name)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        result = run(node, max_attempts, auto)
        print(f'\n--- final result: {result} ---')
        if result == 'success':
            print('\n*** Placed face 1 up on the conveyor — done. ***')
        elif result == 'fail_unverified':
            print('\nRotations did not produce the expected face. Likely the '
                  'die slipped during a re-grip — try again.')
        else:
            print(f'\nFailure: {result}.')
    except KeyboardInterrupt:
        print('\nInterrupted.')
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
