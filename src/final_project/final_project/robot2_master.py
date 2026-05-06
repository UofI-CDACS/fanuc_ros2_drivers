#!/usr/bin/env python3
"""
Bunsen master node — Robot 2, IP 10.8.4.6.

BunsenBase  — pure Python state machine, no ROS2.  Importable standalone.
BunsenMaster(BunsenBase, Node) — wires in ROS2 action clients.

State machine:
  Setup → Wait → GrabDie → PipCount → PositionPip
       → PlaceDie → Wait  (pips 2, 4)
       → Finish          (pip 6)
  Any state → Recover → retry, or Fault → safe exit.
"""

import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from pymodbus.client import ModbusTcpClient

from modbus_server import (
    REG_STATE, REG_PIP_PROGRESS, REG_CONV_CMD, REG_RETRIES,
    COIL_READY, COIL_CAMERA_CLIENT,
    STATE_SETUP, STATE_WAIT, STATE_GRAB_DIE, STATE_PIP_COUNT,
    STATE_POSITION_PIP, STATE_PLACE_DIE, STATE_FINISH,
    STATE_RECOVER, STATE_FAULT, STATE_NAMES,
    CONV_IDLE, CONV_BEAKER_WANTS_SEND, CONV_REAR_RUNNING,
    CONV_DIE_ON_REAR, CONV_BUNSEN_HAS_DIE,
    CONV_BUNSEN_WANTS_SEND, CONV_FRONT_RUNNING,
    CONV_DIE_ON_FRONT, CONV_BEAKER_HAS_DIE,
)

# ---------------------------------------------------------------------------
# Tuning
# ---------------------------------------------------------------------------
MAX_RECOVER_ATTEMPTS = 3
MAX_POSITION_RETRIES = 5
CONVEYOR_TRAVEL_SECS = 5.0
CAMERA_SETTLE_SECS   = 0.3
CONV_TIMEOUT_SECS    = 60.0
POLL_INTERVAL        = 0.2

# ---------------------------------------------------------------------------
# Robot poses — CALIBRATE all values before first run
# ---------------------------------------------------------------------------
HOME_JOINTS = dict(joint1=0.0, joint2=0.0, joint3=0.0,
                   joint4=0.0, joint5=-90.0, joint6=0.0)

CONVEYOR_WAIT_POSE = dict(x=-72.719, y=-404.0, z=352.581, w=-175.773, p=0.668, r=-89.634)

REAR_CONV_ABOVE  = dict(x=0.0, y=0.0, z=0.0, w=0.0, p=0.0, r=0.0)     # CALIBRATE
REAR_CONV_PICKUP = dict(x=0.0, y=0.0, z=0.0, w=0.0, p=0.0, r=0.0)     # CALIBRATE

FRONT_CONV_ABOVE  = dict(joint1=-62.667, joint2=13.088, joint3=-25.034, joint4=-1.655, joint5=-65.618, joint6=-26.181)
FRONT_CONV_PLACE  = dict(joint1=-64.747, joint2=15.950, joint3=-33.644, joint4=-1.767, joint5=-57.065, joint6=-23.823)
FRONT_CONV_PICKUP = FRONT_CONV_PLACE  # alias

CAM_POSE_1 = dict(joint1=-49.388, joint2=36.993, joint3=24.010, joint4=67.703, joint5=-51.149, joint6=119.961)
CAM_POSE_2 = dict(joint1=-70.999, joint2=51.903, joint3=8.141, joint4=174.404, joint5=-79.550, joint6=23.121)

ROT_TILT_AWAY   = dict(x=0.0, y=0.0, z=0.0, w=0.0, p=0.0, r=0.0)     # front→top  CALIBRATE
ROT_TILT_TOWARD = dict(x=0.0, y=0.0, z=0.0, w=0.0, p=0.0, r=0.0)     # back→top   CALIBRATE
ROT_ROLL_LEFT   = dict(x=0.0, y=0.0, z=0.0, w=0.0, p=0.0, r=0.0)     # right→top  CALIBRATE
ROT_ROLL_RIGHT  = dict(x=0.0, y=0.0, z=0.0, w=0.0, p=0.0, r=0.0)     # left→top   CALIBRATE
ROT_FLIP_1      = dict(x=0.0, y=0.0, z=0.0, w=0.0, p=0.0, r=0.0)     # flip step1 CALIBRATE
ROT_FLIP_2      = dict(x=0.0, y=0.0, z=0.0, w=0.0, p=0.0, r=0.0)     # flip step2 CALIBRATE

FINAL_PLACE_ABOVE = dict(joint1=18.885, joint2=-3.830, joint3=-27.348, joint4=0.106, joint5=-62.685, joint6=-18.934)
FINAL_PLACE_DOWN  = dict(joint1=18.885, joint2=16.454, joint3=-64.706, joint4=0.221, joint5=-25.327, joint6=-19.085)

# ---------------------------------------------------------------------------
# Die orientation math
# ---------------------------------------------------------------------------
def _build_die_chirality():
    visited = {}
    queue = [(1, 2, 3)]
    while queue:
        top, front, right = queue.pop(0)
        if (top, front) in visited:
            continue
        visited[(top, front)] = right
        for new in [
            (7 - front, top,     right),
            (front,     7 - top, right),
            (7 - right, front,   top),
            (right,     front,   7 - top),
        ]:
            if (new[0], new[1]) not in visited:
                queue.append(new)
    return visited

_CHIRALITY = _build_die_chirality()

def _rotation_for_target(face1, face2, target):
    right = _CHIRALITY.get((face1, face2))
    if right is None:
        return None
    if target == face1:       return 'none'
    if target == 7 - face1:   return 'flip'
    if target == face2:       return 'tilt_away'
    if target == 7 - face2:   return 'tilt_toward'
    if target == right:       return 'roll_left'
    return 'roll_right'


# ===========================================================================
# BunsenBase — state machine with no ROS2 dependency
# ===========================================================================

class BunsenBase:
    """
    Self-contained state machine.  Subclasses implement the five abstract
    robot-action methods; everything else (Modbus, state logic, die math)
    lives here.
    """

    def __init__(self, mb_host='127.0.0.1', mb_port=5020):
        self.mb = ModbusTcpClient(mb_host, port=mb_port)
        if not self.mb.connect():
            raise RuntimeError(f'Cannot connect to Modbus at {mb_host}:{mb_port}')

        self.state            = STATE_SETUP
        self.prev_state       = STATE_SETUP
        self.failed_action    = ''
        self.recover_attempts = 0
        self.position_retries = 0
        self.face1            = None
        self.face2            = None

    # ---- logging (override in subclass for ROS2 logger) --------------------

    def _log_info(self, msg):  print(f'[INFO]  {msg}')
    def _log_warn(self, msg):  print(f'[WARN]  {msg}')
    def _log_error(self, msg): print(f'[ERROR] {msg}')

    # ---- hook called once before the state machine starts ------------------

    def _wait_for_servers(self):
        pass  # ROS2 subclass overrides this

    # ---- abstract robot actions (subclass must implement) ------------------

    def _send_cart(self, **kwargs) -> bool:
        raise NotImplementedError

    def _send_joint(self, **kwargs) -> bool:
        raise NotImplementedError

    def _send_gripper(self, command: str) -> bool:
        raise NotImplementedError

    def _run_conveyor(self, command: str) -> bool:
        raise NotImplementedError

    def _capture_pip(self) -> int:
        raise NotImplementedError

    # ---- Modbus helpers ----------------------------------------------------

    def mb_read_reg(self, addr):
        return self.mb.read_holding_registers(addr, 1).registers[0]

    def mb_write_reg(self, addr, value):
        self.mb.write_register(addr, value)

    def mb_read_coil(self, addr):
        return bool(self.mb.read_coils(addr, 1).bits[0])

    def mb_write_coil(self, addr, value):
        self.mb.write_coil(addr, bool(value))

    def _set_state(self, new_state):
        self.state = new_state
        self.mb_write_reg(REG_STATE, new_state)
        if new_state not in (STATE_RECOVER, STATE_FAULT):
            self.recover_attempts = 0
        self._log_info(f'→ {STATE_NAMES[new_state]}')

    def _fault(self, action):
        self._log_error(f'Failed: "{action}" in {STATE_NAMES[self.state]}')
        self.prev_state    = self.state
        self.failed_action = action
        self.recover_attempts += 1
        if self.recover_attempts <= MAX_RECOVER_ATTEMPTS:
            self._set_state(STATE_RECOVER)
        else:
            self._set_state(STATE_FAULT)

    def _wait_conv(self, target, timeout=CONV_TIMEOUT_SECS):
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self.mb_read_reg(REG_CONV_CMD) == target:
                return True
            time.sleep(POLL_INTERVAL)
        return False

    def _apply_rotation(self, rotation):
        if rotation == 'none':        return True
        if rotation == 'tilt_away':   return self._send_cart(**ROT_TILT_AWAY)
        if rotation == 'tilt_toward': return self._send_cart(**ROT_TILT_TOWARD)
        if rotation == 'roll_left':   return self._send_cart(**ROT_ROLL_LEFT)
        if rotation == 'roll_right':  return self._send_cart(**ROT_ROLL_RIGHT)
        if rotation == 'flip':
            return self._send_cart(**ROT_FLIP_1) and self._send_cart(**ROT_FLIP_2)
        return False

    # =========================================================================
    # States
    # =========================================================================

    def _state_setup(self):
        self._log_info('Setup: initializing...')
        self.mb_write_reg(REG_STATE,        STATE_SETUP)
        self.mb_write_reg(REG_PIP_PROGRESS, 0)
        self.mb_write_reg(REG_CONV_CMD,     CONV_IDLE)
        self.mb_write_reg(REG_RETRIES,      0)
        self.mb_write_coil(COIL_READY,         False)
        self.mb_write_coil(COIL_CAMERA_CLIENT, False)

        self._wait_for_servers()

        if not self._send_joint(**HOME_JOINTS):
            self._fault('home_on_setup')
            return

        self.mb_write_coil(COIL_READY, True)
        self._set_state(STATE_WAIT)

    def _state_wait(self):
        target = self.mb_read_reg(REG_PIP_PROGRESS)
        self._log_info(f'Wait: staged at rear conveyor, pip progress = {target}')
        self.mb_write_coil(COIL_READY, True)

        if not self._send_cart(**CONVEYOR_WAIT_POSE):
            self._fault('conveyor_wait_pose')
            return

        self._log_info('Wait: polling for BEAKER_WANTS_SEND...')
        while self.mb_read_reg(REG_CONV_CMD) != CONV_BEAKER_WANTS_SEND:
            time.sleep(POLL_INTERVAL)

        self.mb_write_coil(COIL_READY, False)
        self._set_state(STATE_GRAB_DIE)

    def _state_grab_die(self):
        self._log_info('GrabDie: starting rear conveyor...')

        if not self._run_conveyor('forward'):
            self._fault('rear_conveyor_start')
            return
        self.mb_write_reg(REG_CONV_CMD, CONV_REAR_RUNNING)

        self._log_info('GrabDie: waiting for die on rear conveyor...')
        if not self._wait_conv(CONV_DIE_ON_REAR):
            self._run_conveyor('stop')
            self._fault('wait_die_on_rear')
            return

        time.sleep(CONVEYOR_TRAVEL_SECS)
        self._run_conveyor('stop')

        if not self._send_gripper('open'):
            self._fault('gripper_open_before_pick')
            return
        if not self._send_cart(**REAR_CONV_ABOVE):
            self._fault('approach_rear_conv')
            return
        if not self._send_cart(**REAR_CONV_PICKUP):
            self._fault('descend_rear_conv')
            return
        if not self._send_gripper('close'):
            self._fault('gripper_close_on_die')
            return
        if not self._send_cart(**REAR_CONV_ABOVE):
            self._fault('retract_rear_conv')
            return

        self.mb_write_reg(REG_CONV_CMD, CONV_BUNSEN_HAS_DIE)
        if not self._wait_conv(CONV_IDLE, timeout=10.0):
            self.mb_write_reg(REG_CONV_CMD, CONV_IDLE)

        self._set_state(STATE_PIP_COUNT)

    def _state_pip_count(self):
        target = self.mb_read_reg(REG_PIP_PROGRESS)
        self._log_info(f'PipCount: reading two faces (target pip = {target})')
        self.position_retries = 0
        self.face1 = None
        self.face2 = None

        if not self._send_cart(**CAM_POSE_1):
            self._fault('move_cam_pose_1')
            return
        time.sleep(CAMERA_SETTLE_SECS)
        f1 = self._capture_pip()
        if f1 < 1:
            self._fault('capture_cam_pose_1')
            return

        if not self._send_cart(**CAM_POSE_2):
            self._fault('move_cam_pose_2')
            return
        time.sleep(CAMERA_SETTLE_SECS)
        f2 = self._capture_pip()
        if f2 < 1:
            self._fault('capture_cam_pose_2')
            return

        if f1 == f2:
            self._log_warn('face1 == face2 — cam pose 2 did not expose a new face')
            self._fault('pip_count_same_face')
            return

        self.face1 = f1
        self.face2 = f2
        self._log_info(f'PipCount: face1={f1}  face2={f2}  target={target}')

        if not self._send_cart(**CAM_POSE_1):
            self._fault('return_cam_pose_1')
            return

        self._set_state(STATE_POSITION_PIP)

    def _state_position_pip(self):
        target = self.mb_read_reg(REG_PIP_PROGRESS)
        self._log_info(
            f'PositionPip: face1={self.face1} face2={self.face2} '
            f'target={target} attempt={self.position_retries + 1}')

        rotation = _rotation_for_target(self.face1, self.face2, target)
        if rotation is None:
            self._log_error(f'({self.face1},{self.face2}) is not a valid adjacent-face pair')
            self._fault('invalid_die_orientation')
            return

        self._log_info(f'PositionPip: applying rotation "{rotation}"')
        if not self._apply_rotation(rotation):
            self._fault(f'rotation_{rotation}')
            return

        if not self._send_cart(**CAM_POSE_1):
            self._fault('move_cam_pose_1_verify')
            return
        time.sleep(CAMERA_SETTLE_SECS)
        confirmed = self._capture_pip()

        if confirmed == target:
            self._log_info(f'PositionPip: pip {target} confirmed!')
            self.mb_write_reg(REG_PIP_PROGRESS, target + 1 if target < 6 else 0)
            self.mb_write_coil(COIL_CAMERA_CLIENT, False)
            self._set_state(STATE_FINISH if target == 6 else STATE_PLACE_DIE)
            return

        self._log_warn(f'PositionPip: got {confirmed}, expected {target} — re-reading')
        self.position_retries += 1
        self.mb_write_reg(REG_RETRIES, self.mb_read_reg(REG_RETRIES) + 1)

        if self.position_retries >= MAX_POSITION_RETRIES:
            self._fault('position_pip_max_retries')
            return

        if not self._send_cart(**CAM_POSE_2):
            self._fault('move_cam_pose_2_reread')
            return
        time.sleep(CAMERA_SETTLE_SECS)
        new_f2 = self._capture_pip()
        if new_f2 < 1:
            self._fault('capture_cam_pose_2_reread')
            return

        self.face1 = confirmed
        self.face2 = new_f2
        if not self._send_cart(**CAM_POSE_1):
            self._fault('return_cam_pose_1_reread')
            return
        # stays in STATE_POSITION_PIP; run() will call again

    def _state_place_die(self):
        self._log_info('PlaceDie: requesting front conveyor from Beaker...')
        self.mb_write_reg(REG_CONV_CMD, CONV_BUNSEN_WANTS_SEND)

        if not self._wait_conv(CONV_FRONT_RUNNING):
            self.mb_write_reg(REG_CONV_CMD, CONV_IDLE)
            self._fault('wait_front_conv_running')
            return

        if not self._send_cart(**FRONT_CONV_ABOVE):
            self._fault('approach_front_conv')
            return
        if not self._send_cart(**FRONT_CONV_PLACE):
            self._fault('descend_front_conv')
            return
        if not self._send_gripper('open'):
            self._fault('gripper_open_place')
            return
        if not self._send_cart(**FRONT_CONV_ABOVE):
            self._fault('retract_front_conv')
            return

        self.mb_write_reg(REG_CONV_CMD, CONV_DIE_ON_FRONT)

        if not self._wait_conv(CONV_BEAKER_HAS_DIE):
            self._fault('wait_beaker_has_die')
            return
        self.mb_write_reg(REG_CONV_CMD, CONV_IDLE)

        self._log_info('PlaceDie: die transferred. Returning to Wait.')
        self._set_state(STATE_WAIT)

    def _state_finish(self):
        self._log_info('Finish: placing pip-6 die in front of Bunsen.')

        if not self._send_cart(**FINAL_PLACE_ABOVE):
            self._fault('approach_final')
            return
        if not self._send_cart(**FINAL_PLACE_DOWN):
            self._fault('descend_final')
            return
        if not self._send_gripper('open'):
            self._fault('gripper_open_final')
            return
        if not self._send_cart(**FINAL_PLACE_ABOVE):
            self._fault('retract_final')
            return

        self._send_joint(**HOME_JOINTS)
        self.mb_write_coil(COIL_READY,         False)
        self.mb_write_coil(COIL_CAMERA_CLIENT, False)

        retries = self.mb_read_reg(REG_RETRIES)
        print('\n' + '=' * 50)
        print('    BUNSEN DONE — pip 6 found and placed!')
        print(f'    Bunsen repositions : {retries}')
        print('=' * 50 + '\n')

    def _state_recover(self):
        self._log_warn(
            f'Recover ({self.recover_attempts}/{MAX_RECOVER_ATTEMPTS}): '
            f'"{self.failed_action}" in {STATE_NAMES[self.prev_state]}')

        if self.prev_state == STATE_SETUP:
            time.sleep(2.0)
            self._set_state(STATE_SETUP)

        elif self.prev_state == STATE_WAIT:
            time.sleep(1.0)
            self._set_state(STATE_WAIT)

        elif self.prev_state == STATE_GRAB_DIE:
            self._run_conveyor('stop')
            self._send_gripper('open')
            self.mb_write_reg(REG_CONV_CMD, CONV_IDLE)
            time.sleep(1.0)
            self._send_joint(**HOME_JOINTS)
            self._set_state(STATE_WAIT)

        elif self.prev_state in (STATE_PIP_COUNT, STATE_POSITION_PIP):
            self.face1 = None
            self.face2 = None
            self.position_retries = 0
            self._send_cart(**CAM_POSE_1)
            self._set_state(STATE_PIP_COUNT)

        elif self.prev_state == STATE_PLACE_DIE:
            self._run_conveyor('stop')
            self.mb_write_reg(REG_CONV_CMD, CONV_IDLE)
            time.sleep(1.0)
            self._set_state(STATE_PLACE_DIE)

        elif self.prev_state == STATE_FINISH:
            self._set_state(STATE_FINISH)

        else:
            self._log_error('No recovery path — going to Fault')
            self.recover_attempts = MAX_RECOVER_ATTEMPTS + 1
            self._set_state(STATE_FAULT)

    def _state_fault(self):
        print('\n' + '=' * 50)
        print('  !! BUNSEN FAULT — program stopped safely !!')
        print(f'  Failed state : {STATE_NAMES.get(self.prev_state)}')
        print(f'  Failed action: {self.failed_action}')
        print(f'  Recovery tries: {self.recover_attempts}')
        print('=' * 50 + '\n')

        self.mb_write_coil(COIL_READY,         False)
        self.mb_write_coil(COIL_CAMERA_CLIENT, False)
        self._run_conveyor('stop')
        self._send_gripper('open')
        try:
            self._send_joint(**HOME_JOINTS)
        except Exception:
            pass

    # =========================================================================
    # Main loop
    # =========================================================================

    def run(self):
        self._wait_for_servers()
        self._set_state(STATE_SETUP)

        while self.state not in (STATE_FINISH, STATE_FAULT):
            if self.state == STATE_SETUP:         self._state_setup()
            elif self.state == STATE_WAIT:         self._state_wait()
            elif self.state == STATE_GRAB_DIE:     self._state_grab_die()
            elif self.state == STATE_PIP_COUNT:    self._state_pip_count()
            elif self.state == STATE_POSITION_PIP: self._state_position_pip()
            elif self.state == STATE_PLACE_DIE:    self._state_place_die()
            elif self.state == STATE_RECOVER:      self._state_recover()

        if self.state == STATE_FINISH:  self._state_finish()
        elif self.state == STATE_FAULT: self._state_fault()

    def close(self):
        self.mb.close()


# ===========================================================================
# BunsenMaster — adds ROS2 action clients on top of BunsenBase
# ===========================================================================

def _make_ros2_master():
    """
    Deferred import so the module can be imported without ROS2 available.
    Returns the BunsenMaster class.
    """
    import rclpy
    from rclpy.node import Node
    from rclpy.action import ActionClient
    from fanuc_interfaces.action import CartPose, JointPose, SchunkGripper, Conveyor
    from fanuc_interfaces.srv import CaptureAndCount

    class BunsenMaster(BunsenBase, Node):

        def __init__(self):
            ns      = os.environ.get('BUNSEN_NAME', 'Bunsen')
            mb_host = os.environ.get('MODBUS_HOST', '127.0.0.1')
            mb_port = int(os.environ.get('MODBUS_PORT', '5020'))
            cam_svc = os.environ.get('CAMERA_SERVICE', '/camera/capture_and_count')

            Node.__init__(self, 'bunsen_master')
            BunsenBase.__init__(self, mb_host, mb_port)

            self.cart_ac     = ActionClient(self, CartPose,      f'/{ns}/cartesian_pose')
            self.joint_ac    = ActionClient(self, JointPose,     f'/{ns}/joint_pose')
            self.gripper_ac  = ActionClient(self, SchunkGripper, f'/{ns}/schunk_gripper')
            self.conveyor_ac = ActionClient(self, Conveyor,      f'/{ns}/conveyor')
            self.cam_client  = self.create_client(CaptureAndCount, cam_svc)

        def _log_info(self, msg):  self.get_logger().info(msg)
        def _log_warn(self, msg):  self.get_logger().warn(msg)
        def _log_error(self, msg): self.get_logger().error(msg)

        def _wait_for_servers(self):
            self._log_info('Waiting for action servers...')
            self.cart_ac.wait_for_server()
            self.joint_ac.wait_for_server()
            self.gripper_ac.wait_for_server()
            self.conveyor_ac.wait_for_server()
            self._log_info('Action servers ready.')

        def _send_cart(self, **kwargs):
            if not self.cart_ac.wait_for_server(timeout_sec=5.0):
                self._log_error('CartPose server not available (timeout 5 s)')
                return False
            goal = CartPose.Goal()
            for k, v in kwargs.items():
                setattr(goal, k, float(v))
            fut = self.cart_ac.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, fut)
            gh = fut.result()
            if not gh.accepted: return False
            res = gh.get_result_async()
            rclpy.spin_until_future_complete(self, res)
            return res.result().result.success

        def _send_joint(self, **kwargs):
            if not self.joint_ac.wait_for_server(timeout_sec=5.0):
                self._log_error('JointPose server not available (timeout 5 s)')
                return False
            goal = JointPose.Goal()
            for k, v in kwargs.items():
                setattr(goal, k, float(v))
            fut = self.joint_ac.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, fut)
            gh = fut.result()
            if not gh.accepted: return False
            res = gh.get_result_async()
            rclpy.spin_until_future_complete(self, res)
            return res.result().result.success

        def _send_gripper(self, command):
            if not self.gripper_ac.wait_for_server(timeout_sec=5.0):
                self._log_error('SchunkGripper server not available (timeout 5 s)')
                return False
            goal = SchunkGripper.Goal()
            goal.command = command
            fut = self.gripper_ac.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, fut)
            gh = fut.result()
            if not gh.accepted: return False
            res = gh.get_result_async()
            rclpy.spin_until_future_complete(self, res)
            return res.result().result.success

        def _run_conveyor(self, command):
            if not self.conveyor_ac.wait_for_server(timeout_sec=5.0):
                self._log_error('Conveyor server not available (timeout 5 s)')
                return False
            goal = Conveyor.Goal()
            goal.command = command
            fut = self.conveyor_ac.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, fut)
            gh = fut.result()
            if not gh.accepted: return False
            res = gh.get_result_async()
            rclpy.spin_until_future_complete(self, res)
            return res.result().result.success

        def _capture_pip(self):
            deadline = time.time() + 30.0
            while not self.mb_read_coil(COIL_CAMERA_CLIENT):
                if time.time() > deadline:
                    return -1
                time.sleep(POLL_INTERVAL)
            self.cam_client.wait_for_service()
            from fanuc_interfaces.srv import CaptureAndCount as CC
            req = CC.Request()
            fut = self.cam_client.call_async(req)
            rclpy.spin_until_future_complete(self, fut)
            resp = fut.result()
            return resp.pip_count if resp.success else -1

        def destroy_node(self):
            self.mb.close()
            super().destroy_node()

    return BunsenMaster


def main(args=None):
    import rclpy
    BunsenMaster = _make_ros2_master()
    rclpy.init(args=args)
    node = BunsenMaster()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
