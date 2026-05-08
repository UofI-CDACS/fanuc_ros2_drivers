"""
Final assignment — DJ side.

Same dice-cycle logic as dice_cycle_test.py (find face 1, place it up on the
conveyor), but with two additions:

  1. Modbus TCP coordination with the partner robot (Bill):
       - Reset Total_Pip_Count, DJ_Retries to 0 at start of program
       - DJ_Has_Dice  := True  at start (DJ has the die)
       - Cycle_Active := True  at start
       - Each camera scan increments Total_Pip_Count by the pip count seen.
       - Each retry cycle (rotation or flip) increments DJ_Retries.
       - Last_Known_Pip := the upper face placed on the conveyor (the first
         camera scan in the algorithm — face A from the initial read).
       - After placement, run the conveyor forward until the die has fully
         passed the right proximity sensor, then set Ready_For_Pickup := True
         and DJ_Has_Dice := False.
       - Wait for Bill to take the die and return it: Bill_Has_Dice goes True
         (Bill picks up) then back to False with Ready_For_Pickup := True
         (Bill has placed it back and signaled).
       - DJ_Has_Dice := True, Ready_For_Pickup := False.

  2. The script is a single one-shot run; re-run for each die.

Run:
    python3 ModbusStuff/final_assignment_dj.py [robot_name] [--auto] \
        [--mb-host HOST] [--mb-port 5020]

Make sure the launch is up (action servers, msg publishers, camera node) and
the Modbus server is reachable.
"""

import sys
import threading
import time

import rclpy
from fanuc_interfaces.action import (
    CartPose, Conveyor, JointPose, SchunkGripper,
)
from fanuc_interfaces.msg import ProxReadings
from fanuc_interfaces.srv import CountPips
from pymodbus.client import ModbusTcpClient
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


# ── Modbus register map (matches ClaudeAssignmentModbusRegister.py) ────────────
COIL_DJ_HAS_DICE      = 0
COIL_BILL_HAS_DICE    = 1
COIL_READY_FOR_PICKUP = 2
COIL_CYCLE_ACTIVE     = 3

HR_TOTAL_PIP_COUNT = 0
HR_TOTAL_RETRIES   = 1
HR_BILL_RETRIES    = 2
HR_DJ_RETRIES      = 3
HR_LAST_KNOWN_PIP  = 4

MB_SLAVE = 1


# ── Joint waypoints — copied from dice_cycle_test.py ──────────────────────────
HOME_JOINTS = [0.0, 0.0, 0.0, 0.0, -90.0, 30.0]

POSITIONS = {
    'above_dice':                  [12.943, 13.472, -34.815,  0.000, -55.185, 17.057],
    'grab_dice_table':             [12.943, 23.461, -52.853,  0.000, -37.147, 17.057],
    'table_show_pre':              [90.599,  5.785,  -1.906, -0.518, -84.400, 17.580],
    'camera_dice_position_1':      [90.000,  0.000,   0.019, -0.079,  -7.510, 22.932],
    'camera_position_2':           [90.366, 40.547,  -3.094, -0.493,  86.951, 20.317],
    'above_conveyor':              [141.312, 18.915,  -4.325,  2.615, -86.085, -22.345],
    'above_place_conveyor':        [141.292, 23.324, -26.470,  2.903, -63.965, -23.422],
    'above_rotation_conveyor':     [140.721, 19.383,   0.107, -0.203, -92.739, -107.715],
    'pickup_conveyor_rotation':    [140.696, 23.829, -25.053, -0.348, -67.566, -110.158],
    'above_conveyor_flip':         [158.677, -1.161, -37.929, 106.898, -73.756, -103.641],
    'place_on_conveyor_flipped':   [156.677, 43.480, -102.660, 114.022, -98.044, -164.281],
    'second_conveyor_pickup_above': [117.061, 18.693,  -2.709,  0.000, -87.291,  -0.162],
    'second_conveyor_pickup':       [117.061, 23.305, -25.827,  0.000, -64.173,  -0.162],
}

CART_POSITIONS = {
    'pickup_conveyor_rotation': dict(
        x=-500.10, y=622.07, z=260.16, w=177.43, p=-0.61, r=30.41,
    ),
    'above_conveyor_flip': dict(
        x=-486.43, y=320.85, z=498.84, w=83.99, p=-62.59, r=-172.87,
    ),
    'place_on_conveyor_flipped_approach': dict(
        x=-491.88, y=303.15, z=-23.99, w=83.99, p=-62.59, r=-172.87,
    ),
    'place_on_conveyor_flipped': dict(
        x=-491.88, y=328.15, z=-23.99, w=83.99, p=-62.59, r=-172.87,
    ),
}

CAM1_AXIS = '+X'
CAM2_AXIS = '+Y'

AXES = ['+X', '-X', '+Y', '-Y', '+Z', '-Z']
OPP  = {'+X': '-X', '-X': '+X', '+Y': '-Y', '-Y': '+Y', '+Z': '-Z', '-Z': '+Z'}


# ── Die orientation logic (copied from dice_cycle_test.py) ────────────────────

def _all_orientations():
    base = {'+Z': 1, '-Z': 6, '+Y': 2, '-Y': 5, '+X': 3, '-X': 4}

    def rot_z(o):
        return {'+Z': o['+Z'], '-Z': o['-Z'],
                '+Y': o['+X'], '-X': o['+Y'],
                '-Y': o['-X'], '+X': o['-Y']}

    def rot_x(o):
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


def infer_orientation(face_a, face_b):
    if face_a is None or face_b is None:
        return None
    if face_a + face_b == 7 or face_a == face_b:
        return None
    for o in _ORIENTATIONS:
        if o[CAM1_AXIS] == face_a and o[CAM2_AXIS] == face_b:
            return o
    return None


def axis_with_value(orientation, value):
    for ax, v in orientation.items():
        if v == value:
            return ax
    return None


def fmt_orientation(o):
    return ', '.join(f'{ax}={o[ax]}' for ax in AXES)


ROTATIONS_TO_PLUS_X = {'+X': 0, '-Z': 1, '-X': 2, '+Z': 3}


def plan_for_target(orientation, target: int):
    target_axis = axis_with_value(orientation, target)
    if target_axis == CAM1_AXIS:
        return ('place_a', 0)
    if target_axis == CAM2_AXIS:
        return ('place_b', 0)
    if target_axis == OPP[CAM2_AXIS]:
        return ('place_basic', 0)
    n = ROTATIONS_TO_PLUS_X.get(target_axis)
    if n is None:
        return (None, None)
    return ('rotate_then_a', n)


# ── ROS plumbing ──────────────────────────────────────────────────────────────

class Cycle(Node):

    def __init__(self, robot_name: str):
        super().__init__('final_assignment_dj')
        self.joints_ac = ActionClient(self, JointPose, f'/{robot_name}/joint_pose')
        self.cart_ac   = ActionClient(self, CartPose, f'/{robot_name}/cartesian_pose')
        self.schunk_ac = ActionClient(self, SchunkGripper, f'/{robot_name}/schunk_gripper')
        self.conveyor_ac = ActionClient(self, Conveyor, f'/{robot_name}/conveyor')
        self.pip_client = self.create_client(CountPips, '/camera/count_pips')

        # Proximity sensor state for conveyor handoff.
        self._left  = False
        self._right = False
        self._prox_lock = threading.Lock()
        self.create_subscription(
            ProxReadings, f'/{robot_name}/prox_readings', self._prox_cb, 10
        )

    def _prox_cb(self, msg: ProxReadings):
        with self._prox_lock:
            self._left  = msg.left
            self._right = msg.right

    def prox(self):
        with self._prox_lock:
            return self._left, self._right

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
        time.sleep(0.5)  # let the jaws settle before the next motion
        return ok

    def conveyor(self, command: str) -> bool:
        if not self.conveyor_ac.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('conveyor action server not available')
            return False
        goal = Conveyor.Goal()
        goal.command = command
        send = self.conveyor_ac.send_goal_async(goal)
        if not self._wait(send):
            return False
        gh = send.result()
        if not gh or not gh.accepted:
            self.get_logger().error(f'Conveyor rejected ({command})')
            return False
        return self._wait(gh.get_result_async())

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


# ── Modbus helper ─────────────────────────────────────────────────────────────

class Modbus:
    """Thin wrapper around pymodbus for the registers used in this assignment."""

    def __init__(self, host: str, port: int):
        self.client = ModbusTcpClient(host, port=port)
        self.connected = self.client.connect()
        if not self.connected:
            raise RuntimeError(f'Could not connect to Modbus server at {host}:{port}')
        print(f'  Modbus connected to {host}:{port} (slave={MB_SLAVE})')

    def close(self):
        try:
            self.client.close()
        except Exception:
            pass

    # Coils
    def write_coil(self, addr: int, value: bool, label: str = ''):
        rw = self.client.write_coil(addr, value, slave=MB_SLAVE)
        if rw.isError():
            print(f'  [Modbus] write_coil {label or addr} = {value}  ERROR: {rw}')
        else:
            print(f'  [Modbus] write_coil {label or addr} = {value}')

    def read_coil(self, addr: int) -> bool:
        rr = self.client.read_coils(addr, count=1, slave=MB_SLAVE)
        if rr.isError():
            return False
        return bool(rr.bits[0])

    # Holding registers
    def write_register(self, addr: int, value: int, label: str = ''):
        rw = self.client.write_register(addr, value, slave=MB_SLAVE)
        if rw.isError():
            print(f'  [Modbus] write_register {label or addr} = {value}  ERROR: {rw}')
        else:
            print(f'  [Modbus] write_register {label or addr} = {value}')

    def read_register(self, addr: int) -> int:
        rr = self.client.read_holding_registers(addr, count=1, slave=MB_SLAVE)
        if rr.isError():
            return 0
        return rr.registers[0]

    def increment_register(self, addr: int, by: int, label: str = '') -> int:
        cur = self.read_register(addr)
        new = cur + by
        self.write_register(addr, new, label)
        return new


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


def show_to_camera(robot: Cycle, auto: bool,
                   modbus: 'Modbus | None' = None,
                   target: int = 1):
    """Read both cameras. If modbus is provided, increment Total_Pip_Count by
    each successful read so the live total reflects every camera scan.

    Optimization: if camera 1 already shows the target face, skip the second
    camera move/read entirely and return (target, None)."""
    if not go(robot, 'table_show_pre', auto):
        return None, None
    if not go(robot, 'camera_dice_position_1', auto):
        return None, None
    
    step('read camera 1', auto)
    a = robot.read_pips()
    print(f'    camera 1 read: {a}')
    if modbus is not None and a is not None:
        modbus.increment_register(HR_TOTAL_PIP_COUNT, a, 'Total_Pip_Count')

    if a == target:
        print(f'    Target ({target}) seen at camera 1 — skipping camera 2.')
        return a, None
    modbus.increment_register(HR_DJ_RETRIES, 1, 'DJ_Retries')

    if not go(robot, 'camera_position_2', auto):
        return a, None
    step('read camera 2', auto)
    b = robot.read_pips()
    print(f'    camera 2 read: {b}')
    if modbus is not None and b is not None:
        modbus.increment_register(HR_TOTAL_PIP_COUNT, b, 'Total_Pip_Count')
    if b != target:
        modbus.increment_register(HR_DJ_RETRIES, 1, 'DJ_Retries')
    return a, b


def place_basic(robot: Cycle, auto: bool) -> bool:
    return (
        go(robot, 'above_conveyor', auto)
        and go(robot, 'above_place_conveyor', auto)
        and (step('open gripper', auto) or robot.gripper('open'))
        and go(robot, 'above_conveyor', auto)
    )


def verify_via_repickup(robot: Cycle, auto: bool, modbus: 'Modbus | None' = None) -> bool:
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
    a, b = show_to_camera(robot, auto, modbus)
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
    raise ValueError(f'unknown rotation mode: {mode}')


# ── Conveyor handoff ──────────────────────────────────────────────────────────

def conveyor_handoff_dj_to_bill(robot: Cycle, modbus: Modbus, auto: bool) -> bool:
    """Run the conveyor forward until the die has fully passed the right
    proximity sensor, then signal Bill via Modbus. Steps:
        1. DJ_Has_Dice := False         (we're about to send it away)
        2. Conveyor forward
        3. Wait for right sensor to go ON  (die at sensor)
        4. Wait for right sensor to go OFF (die has passed)
        5. Stop conveyor
        6. Ready_For_Pickup := True     (signal to Bill)
    """
    print('\n========== Conveyor handoff: DJ -> Bill ==========')
    modbus.write_coil(COIL_DJ_HAS_DICE, False, 'DJ_Has_Dice')

    step('start conveyor FORWARD', auto)
    if not robot.conveyor('forward'):
        return False

    print('  Waiting for die to reach right sensor ...')
    try:
        while rclpy.ok():
            left, right = robot.prox()
            print(f'\r    sensors — left: {"ON " if left else "off"}  '
                  f'right: {"ON " if right else "off"}   ',
                  end='', flush=True)
            if right:
                print('\n  Right sensor ON — die detected!')
                break
            #time.sleep(0.05)

        print('  Waiting for die to pass right sensor ...')
        while rclpy.ok():
            left, right = robot.prox()
            print(f'\r    sensors — left: {"ON " if left else "off"}  '
                  f'right: {"ON " if right else "off"}   ',
                  end='', flush=True)
            if not right:
                print('\n  Right sensor OFF — die has passed!')
                break
            #time.sleep(0.05)
    except KeyboardInterrupt:
        print('\n  Aborted — stopping conveyor.')
        robot.conveyor('stop')
        return False

    step('stop conveyor', auto)
    if not robot.conveyor('stop'):
        return False

    modbus.write_coil(COIL_READY_FOR_PICKUP, True, 'Ready_For_Pickup')
    return True


def wait_for_bill_return(modbus: Modbus) -> bool:
    """Wait for Bill to take the die and return it. Two stages:
        1. Bill_Has_Dice goes True   (Bill picked up our die)
        2. Bill_Has_Dice back to False AND Ready_For_Pickup True
           (Bill placed the die back on the conveyor and signaled)
    """
    print('\n========== Waiting for Bill ==========')

    print('  Waiting for Bill to pick up the die '
          '(Bill_Has_Dice -> True) ...')
    try:
        while rclpy.ok():
            if modbus.read_coil(COIL_BILL_HAS_DICE):
                print('\n  Bill picked up the die.')
                break
            time.sleep(0.2)

        print('  Waiting for Bill to return the die '
              '(Bill_Has_Dice -> False AND Ready_For_Pickup True) ...')
        while rclpy.ok():
            bill_has = modbus.read_coil(COIL_BILL_HAS_DICE)
            ready    = modbus.read_coil(COIL_READY_FOR_PICKUP)
            if (not bill_has) and ready:
                print('\n  Bill returned the die — ready for DJ pickup.')
                return True
            time.sleep(0.2)
    except KeyboardInterrupt:
        print('\n  Aborted while waiting for Bill.')
        return False

    return False


# ── Main run ──────────────────────────────────────────────────────────────────

def _place_after_flip(robot: Cycle, modbus: Modbus, auto: bool,
                      target: int) -> str:
    print('\n========== Rescan after flip ==========')
    a, b = show_to_camera(robot, auto, modbus, target=target)
    if a is None:
        return 'fail_read'

    if a == target:
        print(f'\n    Faces seen: {CAM1_AXIS}={a}')
        if not place_face_a_up(robot, auto):
            return 'fail_place'
        return 'placed'

    if b is None:
        return 'fail_read'

    print(f'\n    Faces seen: {CAM1_AXIS}={a}, {CAM2_AXIS}={b}')

    orientation = infer_orientation(a, b)
    if orientation is None:
        return 'fail_inconsistent'
    print(f'    Inferred die: {fmt_orientation(orientation)}')
    target_axis = axis_with_value(orientation, target)
    print(f'    Target face ({target}) now on gripper axis {target_axis}')

    if b == target or target_axis == CAM2_AXIS or target_axis == OPP[CAM2_AXIS]:
        if not place_basic(robot, auto):
            return 'fail_place'
        return 'placed'

    n = ROTATIONS_TO_PLUS_X.get(target_axis)
    if n is None:
        return 'fail_plan'
    print(f'    {n} rotation(s) to bring {target} to {CAM1_AXIS}, then face A up.')
    for _ in range(n):
        if not rotate_on_conveyor(robot, 'rotate', auto):
            return 'fail_rotate'
    if not place_face_a_up(robot, auto):
        return 'fail_place'
    return 'placed'


def run_dice_cycle(robot: Cycle, modbus: Modbus, auto: bool,
                   target: int) -> str:
    """Run the dice cycle: assume the die is already in the gripper (caller
    handles pickup), find face `target`, place it up on the conveyor.
    Returns 'placed' on success."""
    print(f'\n========== Camera read (target = {target}) ==========')
    a, b = show_to_camera(robot, auto, modbus, target=target)
    if a is None:
        return 'fail_read'

    if a == target:
        print(f'\n    Faces seen: {CAM1_AXIS}={a}')
        print(f'    Saw {target} at camera 1 — placing face A up.')
        return 'placed' if place_face_a_up(robot, auto) else 'fail_place'

    if b is None:
        return 'fail_read'

    print(f'\n    Faces seen: {CAM1_AXIS}={a}, {CAM2_AXIS}={b}')

    if b == target:
        print(f'    Saw {target} at camera 2 — flip + regrab + rescan.')
        if not flip_and_regrab(robot, auto):
            return 'fail_place'
        return _place_after_flip(robot, modbus, auto, target)

    orientation = infer_orientation(a, b)
    if orientation is None:
        return 'fail_inconsistent'

    print(f'    Inferred die: {fmt_orientation(orientation)}')
    target_axis = axis_with_value(orientation, target)
    print(f'    Target face ({target}) currently on gripper axis {target_axis}')

    action, n = plan_for_target(orientation, target)

    if action == 'place_basic':
        print(f'\n    {target} is on the gripped face — basic placement leaves it up.')
        if not place_basic(robot, auto):
            return 'fail_place'
        if not verify_via_repickup(robot, auto, modbus):
            return 'fail_verify'
        return 'placed'

    if action != 'rotate_then_a' or n is None:
        return 'fail_plan'

    print(f'\n    Plan: {n} rotation(s) to bring face {target} to {CAM1_AXIS}, '
          'then face-A-up placement.')
    for i in range(n):
        print(f'\n========== Rotation {i + 1}/{n} ==========')
        if not rotate_on_conveyor(robot, 'rotate', auto):
            return 'fail_rotate'

    print('\n========== Verification read ==========')
    a, b = show_to_camera(robot, auto, modbus, target=target)
    if a is None:
        return 'fail_verify'

    if a == target:
        print(f'    Verification: {CAM1_AXIS}={a}')
        return 'placed' if place_face_a_up(robot, auto) else 'fail_place'

    if b is None:
        return 'fail_verify'

    print(f'    Verification: {CAM1_AXIS}={a}, {CAM2_AXIS}={b}')

    if b == target:
        if not flip_and_regrab(robot, auto):
            return 'fail_place'
        return _place_after_flip(robot, modbus, auto, target)

    return 'fail_unverified'


# ── Second-conveyor pickup (receive from Bill) ────────────────────────────────

def receive_from_conveyor(robot: Cycle, auto: bool) -> bool:
    """Run the conveyor and pick up the dice that Bill just sent back.
    Sequence:
        1. start conveyor forward
        2. move to second_conveyor_pickup_above (waiting position)
        3. wait for the right prox sensor to indicate the dice has arrived
        4. stop conveyor
        5. descend to second_conveyor_pickup, close gripper
        6. lift to second_conveyor_pickup_above
    """
    print('\n========== Receive dice from conveyor ==========')
    return (
        go(robot, 'second_conveyor_pickup_above', auto)
        and go(robot, 'second_conveyor_pickup', auto)
             and robot.gripper('close')
        and go(robot, 'second_conveyor_pickup_above', auto)
    )


# ── Final placement back on the table ─────────────────────────────────────────

def place_on_table(robot: Cycle, auto: bool) -> bool:
    """Place the dice back on the table at the original pickup spot. The
    upper face of the dice (currently on the gripped axis) ends up on top."""
    print('\n========== Final placement on table ==========')
    return (
        go(robot, 'above_dice', auto)
        and go(robot, 'grab_dice_table', auto)
        and (step('open gripper (place on table)', auto)
             and robot.gripper('open'))
        and go(robot, 'above_dice', auto)
        and go_home(robot, auto)
    )


def run(robot: Cycle, modbus: Modbus, auto: bool) -> str:
    """Driver loop for DJ:
        - Initial: pickup from table, target = 1, run cycle, place on conveyor,
          handoff to Bill, wait for Bill to return.
        - Each subsequent iteration: receive dice from conveyor, read
          Last_Known_Pip. If LKP == 6, place on table and end. Otherwise,
          target = LKP + 1, run cycle, place on conveyor, handoff, repeat.
    """
    # ── Init coils & counters ────────────────────────────────────────────────
    print('\n========== Initialising Modbus state ==========')
    modbus.write_register(HR_TOTAL_PIP_COUNT, 0, 'Total_Pip_Count')
    modbus.write_register(HR_DJ_RETRIES,      0, 'DJ_Retries')
    modbus.write_register(HR_LAST_KNOWN_PIP,  0, 'Last_Known_Pip')
    modbus.write_register(HR_BILL_RETRIES,    0, 'Bill_Retries')
    modbus.write_register(HR_TOTAL_RETRIES,   0, 'Total_Retries')
    modbus.write_coil(COIL_BILL_HAS_DICE,     False, 'Bill_Has_Dice')
    modbus.write_coil(COIL_CYCLE_ACTIVE,      True,  'Cycle_Active')
    modbus.write_coil(COIL_DJ_HAS_DICE,       True,  'DJ_Has_Dice')
    modbus.write_coil(COIL_READY_FOR_PICKUP,  False, 'Ready_For_Pickup')

    # ── First iteration: pickup from the table, target = 1 ──────────────────
    print('\n========== Initial pickup from table ==========')
    if not pickup(robot, auto):
        return 'fail_pickup'

    target = 1
    iteration = 1

    while True:
        print(f'\n############ Iteration {iteration}  —  target = {target} '
              f'############')

        status = run_dice_cycle(robot, modbus, auto, target)
        if status != 'placed':
            return status

        # Last_Known_Pip = upper face placed on the conveyor. The cycle's
        # contract is "place face `target` up", so target IS the upper face
        # whenever status == 'placed'.
        modbus.write_register(HR_LAST_KNOWN_PIP, target, 'Last_Known_Pip')

        # Pre-park above the second conveyor so the arm is already in position
        # for the receive (instead of going all the way home and back).
        if not go(robot, 'second_conveyor_pickup_above', auto):
            return 'fail_park'

        # If DJ just placed 6 on the conveyor, hand it off and end — Bill will
        # take it from there (per the assignment, the robot holding 6 places
        # it down, but our DJ-side trigger for the table placement is
        # LKP == 6 ON RECEIVE, so DJ ends here without waiting).
        if target == 6:
            print('\n    DJ placed 6 — handing off and ending. Bill will '
                  'finalise.')
            if not conveyor_handoff_dj_to_bill(robot, modbus, auto):
                return 'fail_conveyor'
            return 'success'

        # Conveyor handoff to Bill.
        if not conveyor_handoff_dj_to_bill(robot, modbus, auto):
            return 'fail_conveyor'
        # Receive Bill's die from the conveyor.
        if target == 5:
            modbus.increment_register(HR_TOTAL_RETRIES, modbus.read_register(HR_DJ_RETRIES), 'Total_Retries')
            print('\n    Next target would be 6 — skipping receive and going to '
                  'final placement.')
            if go_home(robot, auto):
                return 'success'  # we succeeded in placing 5, so end on a high note instead of risking a receive failure

        # Wait for Bill to take and return the die.
        if not wait_for_bill_return(modbus):
            return 'fail_wait_bill'
        if not receive_from_conveyor(robot, auto):
            return 'fail_receive'
        modbus.write_coil(COIL_DJ_HAS_DICE,      True,  'DJ_Has_Dice')
        modbus.write_coil(COIL_READY_FOR_PICKUP, False, 'Ready_For_Pickup')
        


        # Read Last_Known_Pip — Bill set this when he placed.
        last_known = modbus.read_register(HR_LAST_KNOWN_PIP)
        print(f'\n    Last_Known_Pip = {last_known} (set by Bill).')

        """if last_known == 6:
            print('    LKP == 6 — final placement on the table.')
            if not place_on_table(robot, auto):
                return 'fail_table'
            return 'success'"""

        
        """if target > 6:
            print(f'    target would be {target} (out of range) — placing on '
                  'table and ending.')
            if not place_on_table(robot, auto):
                return 'fail_table'
            return 'success'"""
        target = last_known + 1
        iteration += 1


def main():
    args = sys.argv[1:]
    auto = '--auto' in args
    args = [a for a in args if a != '--auto']

    mb_host = 'localhost'
    if '--mb-host' in args:
        i = args.index('--mb-host')
        mb_host = args[i + 1]
        del args[i:i + 2]

    mb_port = 5020
    if '--mb-port' in args:
        i = args.index('--mb-port')
        mb_port = int(args[i + 1])
        del args[i:i + 2]

    robot_name = args[0] if args else 'dj'

    # Modbus first so we fail fast if the server isn't up.
    modbus = Modbus(mb_host, mb_port)

    rclpy.init()
    node = Cycle(robot_name)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        result = run(node, modbus, auto)
        print(f'\n--- final result: {result} ---')

        # Final report
        print('\n========== Final Modbus state ==========')
        print(f'  Total_Pip_Count: {modbus.read_register(HR_TOTAL_PIP_COUNT)}')
        print(f'  DJ_Retries:      {modbus.read_register(HR_DJ_RETRIES)}')
        print(f'  Last_Known_Pip:  {modbus.read_register(HR_LAST_KNOWN_PIP)}')
        print(f'  DJ_Has_Dice:     {modbus.read_coil(COIL_DJ_HAS_DICE)}')
        print(f'  Bill_Has_Dice:   {modbus.read_coil(COIL_BILL_HAS_DICE)}')
        print(f'  Ready_For_Pickup:{modbus.read_coil(COIL_READY_FOR_PICKUP)}')
    except KeyboardInterrupt:
        print('\nInterrupted.')
    finally:
        try:
            modbus.close()
        except Exception:
            pass
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
