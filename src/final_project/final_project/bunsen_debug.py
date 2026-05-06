#!/usr/bin/env python3
"""
Bunsen debug runner — sends commands through the running BunsenServer.
Requires BunsenServer and modbus_server.py to be running first.

Usage (from any directory):
  python3 fanuc_ros2_drivers/src/final_project/final_project/bunsen_debug.py
"""

import os
import signal
import sys
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)

# ---------------------------------------------------------------------------
# Auto-relaunch under the ROS2 environment if not already sourced.
# This replaces the current process (os.execvp) so there is no extra shell
# hanging around afterwards.
# ---------------------------------------------------------------------------
if 'ROS_DISTRO' not in os.environ:
    _ws_setup = os.path.normpath(os.path.join(_HERE, '..', '..', '..', 'install', 'setup.bash'))
    _cmd = 'source /opt/ros/jazzy/setup.bash'
    if os.path.exists(_ws_setup):
        _cmd += f' && source {_ws_setup}'
    _cmd += f' && exec python3 {os.path.abspath(__file__)}'
    os.execvp('bash', ['bash', '-c', _cmd])

from robot2_master import (
    _make_ros2_master,
    HOME_JOINTS, CONVEYOR_WAIT_POSE,
    REAR_CONV_ABOVE, REAR_CONV_PICKUP,
    CAM_POSE_1, CAM_POSE_2,
    FRONT_CONV_ABOVE, FRONT_CONV_PLACE,
    FINAL_PLACE_ABOVE, FINAL_PLACE_DOWN,
    ROT_TILT_AWAY, ROT_TILT_TOWARD,
    ROT_ROLL_LEFT, ROT_ROLL_RIGHT,
    POLL_INTERVAL,
    CAMERA_SETTLE_SECS, CONVEYOR_TRAVEL_SECS,
)
from modbus_server import (
    REG_PIP_PROGRESS, REG_CONV_CMD, REG_RETRIES,
    COIL_CAMERA_CLIENT,
    CONV_BEAKER_WANTS_SEND, CONV_DIE_ON_REAR,
    CONV_FRONT_RUNNING, CONV_BEAKER_HAS_DIE,
    STATE_SETUP, STATE_WAIT, STATE_GRAB_DIE, STATE_PIP_COUNT,
    STATE_POSITION_PIP, STATE_PLACE_DIE, STATE_FINISH,
    STATE_RECOVER, STATE_FAULT, STATE_NAMES,
)

_MENU_STATIC = """
╔══════════════════════════════════════════════╗
║       BUNSEN DEBUG — state runner            ║
║       (via ROS2 action clients)              ║
╠══════════════════════════════════════════════╣
║  1  Setup                                    ║
║  2  Wait        (polls CONV_CMD; Ctrl-C exits)║
║  3  GrabDie     (sim or real Modbus)         ║
║  4  PipCount    (manual pip entry)           ║
║  5  PositionPip (uses last PipCount or typed)║
║  6  PlaceDie    (sim or real Modbus)         ║
║  7  Finish                                   ║
║  8  Recover     (set context first)          ║
║  9  Fault       (set context first)          ║
║  r  Dump Modbus registers & coils            ║
╠══════════════════════════════════════════════╣
║  DIRECT COMMANDS                             ║
║  o    Gripper open  (OnRobot width/force)    ║
║  k    Gripper close (OnRobot width/force)    ║
║  ff   Front conveyor forward                 ║
║  fb   Front conveyor reverse                 ║
║  fs   Front conveyor stop                    ║
║  ft   Front conveyor timed (asks dir+secs)   ║
║  j    Go to home joints                      ║
║  c    Camera snapshot (shows live window)    ║
║  d1–d5  Define / overwrite stored pose       ║
║  q  Quit                                     ║
╚══════════════════════════════════════════════╝"""


def _print_menu():
    print(_MENU_STATIC)
    print('  Stored poses  (edit POSE_1–5 at top of file, then restart):')
    for slot, p in sorted(_stored_poses.items()):
        if 'joint1' in p:
            print(f'    p{slot} [JOINT]'
                  f'  J1={p["joint1"]:7.2f}  J2={p["joint2"]:7.2f}  J3={p["joint3"]:7.2f}'
                  f'  J4={p["joint4"]:7.2f}  J5={p["joint5"]:7.2f}  J6={p["joint6"]:7.2f}')
        else:
            print(f'    p{slot} [CART] '
                  f'  x={p["x"]:8.2f}  y={p["y"]:8.2f}  z={p["z"]:8.2f}'
                  f'  w={p["w"]:8.3f}  p={p["p"]:7.3f}  r={p["r"]:8.3f}')
    print()

# ---------------------------------------------------------------------------
# Stored test poses — edit these directly, then re-run the script.
# p1–p5 in the menu move to these. d1–d5 overwrite them at runtime.
# w/p/r = 200.0 means "keep current orientation" (FANUC driver sentinel).
# ---------------------------------------------------------------------------
# p1–p3, p6–p7: joint positions  (joint1–joint6, degrees)
# p4–p5:        Cartesian        (x,y,z mm  w,p,r degrees; 200.0 = keep current)
POSE_1 = dict(joint1=-62.667, joint2=13.088, joint3=-25.034, joint4=-1.655, joint5=-65.618, joint6=-26.181)
POSE_2 = dict(joint1=-64.747, joint2=15.950, joint3=-33.644, joint4=-1.767, joint5=-57.065, joint6=-23.823)
POSE_3 = dict(joint1=0.0, joint2=0.0, joint3=0.0, joint4=0.0, joint5=-90.0, joint6=0.0)
POSE_4 = dict(x=-72.719, y=-404.0, z=352.581, w=-175.773, p=0.668, r=-89.634)
POSE_5 = dict(x=126.961, y=-582.177, z=52.102, w=-175.773, p=0.668, r=-89.634)
POSE_6 = dict(joint1=0.0, joint2=0.0, joint3=0.0, joint4=0.0, joint5=-90.0, joint6=0.0)
POSE_7 = dict(joint1=0.0, joint2=0.0, joint3=0.0, joint4=0.0, joint5=-90.0, joint6=0.0)

_stored_poses: dict = {'1': POSE_1, '2': POSE_2, '3': POSE_3, '4': POSE_4,
                       '5': POSE_5, '6': POSE_6, '7': POSE_7}


def _ask(prompt, default=''):
    val = input(f'  {prompt} [{default}]: ').strip()
    return val if val else default

def _ask_int(prompt, default):
    while True:
        try:
            return int(_ask(prompt, str(default)))
        except ValueError:
            print('  Need an integer.')

def _confirm(prompt):
    return _ask(prompt + ' [y/N]', 'n').lower() == 'y'


# ---------------------------------------------------------------------------
# BunsenDebug — BunsenMaster with manual pip entry instead of camera service
# ---------------------------------------------------------------------------

def _make_bunsen_debug():
    BunsenMaster = _make_ros2_master()

    import rclpy
    from rclpy.action import ActionClient
    from fanuc_interfaces.action import OnRobotGripper, Conveyor

    class BunsenDebug(BunsenMaster):
        """BunsenMaster wired to the live action servers; camera replaced by manual input."""

        def __init__(self):
            super().__init__()
            ns         = os.environ.get('BUNSEN_NAME', 'Bunsen')
            front_name = os.environ.get('FRONT_CONV_NAME', 'Bunsen')

            self.onrobot_ac    = ActionClient(self, OnRobotGripper, f'/{ns}/onrobot_gripper')
            self.front_conv_ac = ActionClient(self, Conveyor, f'/{front_name}/conveyor')

        def _send_onrobot(self, width: int, force: int) -> bool:
            if not self.onrobot_ac.wait_for_server(timeout_sec=5.0):
                print('  [WARN] OnRobot gripper server not available (timeout 5 s)')
                return False
            goal = OnRobotGripper.Goal()
            goal.width = width
            goal.force = force
            fut = self.onrobot_ac.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, fut)
            gh = fut.result()
            if not gh.accepted:
                return False
            res = gh.get_result_async()
            rclpy.spin_until_future_complete(self, res)
            return res.result().result.success

        def _send_conv(self, ac, command: str) -> bool:
            if not ac.wait_for_server(timeout_sec=5.0):
                print('  [WARN] Conveyor server not available (timeout 5 s)')
                print('         Check FRONT_CONV_NAME / REAR_CONV_NAME env vars')
                return False
            goal = Conveyor.Goal()
            goal.command = command
            fut = ac.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, fut)
            gh = fut.result()
            if not gh.accepted:
                return False
            res = gh.get_result_async()
            rclpy.spin_until_future_complete(self, res)
            return res.result().result.success

        def _capture_pip(self):
            deadline = time.time() + 30.0
            while not self.mb_read_coil(COIL_CAMERA_CLIENT):
                if time.time() > deadline:
                    print('  [CAM] Timed out waiting for COIL_CAMERA_CLIENT.')
                    return -1
                time.sleep(POLL_INTERVAL)
            while True:
                try:
                    val = int(input('  [CAM] Enter pip count (1-6): '))
                    if 1 <= val <= 6:
                        return val
                except (ValueError, EOFError):
                    pass
                print('  Need 1-6.')

    return BunsenDebug


# ---------------------------------------------------------------------------
# Debug menu
# ---------------------------------------------------------------------------

class DebugMenu:

    def __init__(self, node):
        self.n = node

    def _dump(self):
        n = self.n
        print('\n  ── Modbus ───────────────────────────────────')
        sv = n.mb_read_reg(0)
        print(f'  HR 0 STATE         = {sv}  ({STATE_NAMES.get(sv,"?")})')
        print(f'  HR 1 PIP_PROGRESS  = {n.mb_read_reg(1)}')
        print(f'  HR 2 CONV_CMD      = {n.mb_read_reg(2)}')
        print(f'  HR 3 RETRIES       = {n.mb_read_reg(3)}')
        print(f'  C  0 READY         = {n.mb_read_coil(0)}')
        print(f'  C  1 CAMERA_CLIENT = {n.mb_read_coil(1)}')
        if n.face1 is not None:
            print(f'  [local] face1={n.face1}  face2={n.face2}')
        print()

    # ---- per-state wrappers ------------------------------------------------

    def do_setup(self):
        self.n.state = STATE_SETUP
        self.n._state_setup()

    def do_wait(self):
        pip = _ask_int('PIP_PROGRESS to set', self.n.mb_read_reg(REG_PIP_PROGRESS))
        self.n.mb_write_reg(REG_PIP_PROGRESS, pip)
        self.n.state = STATE_WAIT
        print('  Blocking until CONV_CMD == BEAKER_WANTS_SEND (1). Ctrl-C to abort.')
        try:
            self.n._state_wait()
        except KeyboardInterrupt:
            print('\n  Aborted.')

    def do_grab_die(self):
        sim = _confirm('Simulate BEAKER_WANTS_SEND + DIE_ON_REAR locally?')
        self.n.state = STATE_GRAB_DIE
        if sim:
            orig = self.n._wait_conv
            def patched(target, timeout=60.0):
                if target == CONV_DIE_ON_REAR:
                    input('  [SIM] Press Enter → Beaker places die on rear conveyor...')
                    self.n.mb_write_reg(REG_CONV_CMD, CONV_DIE_ON_REAR)
                    return True
                return True
            self.n._wait_conv = patched
            try:
                self.n._state_grab_die()
            finally:
                self.n._wait_conv = orig
        else:
            self.n._state_grab_die()

    def do_pip_count(self):
        pip = _ask_int('PIP_PROGRESS (target pip)', self.n.mb_read_reg(REG_PIP_PROGRESS))
        self.n.mb_write_reg(REG_PIP_PROGRESS, pip)
        if _confirm('Set COIL_CAMERA_CLIENT=True (give Bunsen camera token)?'):
            self.n.mb_write_coil(COIL_CAMERA_CLIENT, True)
        self.n.state = STATE_PIP_COUNT
        self.n._state_pip_count()
        print(f'  Done. face1={self.n.face1}  face2={self.n.face2}')

    def do_position_pip(self):
        if self.n.face1 is None:
            print('  No face data from a PipCount run.')
            self.n.face1 = _ask_int('face1 (top pip at cam pos 1)', 1)
            self.n.face2 = _ask_int('face2 (top pip at cam pos 2 / front pip)', 2)
        pip = _ask_int('PIP_PROGRESS (target pip)', self.n.mb_read_reg(REG_PIP_PROGRESS))
        self.n.mb_write_reg(REG_PIP_PROGRESS, pip)
        if _confirm('Set COIL_CAMERA_CLIENT=True?'):
            self.n.mb_write_coil(COIL_CAMERA_CLIENT, True)
        self.n.state = STATE_POSITION_PIP
        self.n._state_position_pip()

    def do_place_die(self):
        sim = _confirm('Simulate FRONT_RUNNING + BEAKER_HAS_DIE locally?')
        self.n.state = STATE_PLACE_DIE
        if sim:
            orig = self.n._wait_conv
            def patched(target, timeout=60.0):
                if target == CONV_FRONT_RUNNING:
                    input('  [SIM] Press Enter → Beaker starts front conveyor...')
                    self.n.mb_write_reg(REG_CONV_CMD, CONV_FRONT_RUNNING)
                    return True
                if target == CONV_BEAKER_HAS_DIE:
                    input('  [SIM] Press Enter → Beaker picks up die...')
                    self.n.mb_write_reg(REG_CONV_CMD, CONV_BEAKER_HAS_DIE)
                    return True
                return True
            self.n._wait_conv = patched
            try:
                self.n._state_place_die()
            finally:
                self.n._wait_conv = orig
        else:
            self.n._state_place_die()

    def do_finish(self):
        if not _confirm('Die in gripper with pip 6 on top. Ready?'):
            print('  Aborted.')
            return
        self.n.state = STATE_FINISH
        self.n._state_finish()

    def do_recover(self):
        names = ', '.join(f'{k}={v}' for k, v in STATE_NAMES.items())
        print(f'  States: {names}')
        self.n.prev_state     = _ask_int('prev_state', self.n.prev_state or STATE_WAIT)
        self.n.failed_action  = _ask('failed_action', self.n.failed_action or 'test')
        self.n.recover_attempts = _ask_int('recover_attempts', self.n.recover_attempts)
        self.n.state = STATE_RECOVER
        self.n._state_recover()

    def do_fault(self):
        names = ', '.join(f'{k}={v}' for k, v in STATE_NAMES.items())
        print(f'  States: {names}')
        self.n.prev_state     = _ask_int('prev_state', self.n.prev_state or STATE_WAIT)
        self.n.failed_action  = _ask('failed_action', self.n.failed_action or 'test')
        self.n.recover_attempts = _ask_int('recover_attempts', self.n.recover_attempts)
        self.n.state = STATE_FAULT
        self.n._state_fault()

    # ---- direct commands ---------------------------------------------------

    def do_gripper_open(self):
        width = _ask_int('Open width mm [0-160]', 80)
        force = _ask_int('Force N [0-120]', 20)
        ok = self.n._send_onrobot(width, force)
        print(f'  Gripper open → {"OK" if ok else "FAILED"}')

    def do_gripper_close(self):
        width = _ask_int('Close width mm [0-160]', 30)
        force = _ask_int('Force N [0-120]', 60)
        ok = self.n._send_onrobot(width, force)
        print(f'  Gripper close → {"OK" if ok else "FAILED"}')

    def _conv_cmd(self, ac, label: str, command: str):
        ok = self.n._send_conv(ac, command)
        print(f'  {label} {command} → {"OK" if ok else "FAILED"}')

    def do_conv_timed(self):
        direction = ''
        while direction not in ('f', 'b'):
            direction = _ask('Direction  f=forward  b=reverse', 'f').strip().lower()
        secs = float(_ask('Run for seconds', '2.0'))
        command = 'forward' if direction == 'f' else 'reverse'
        print(f'  Front conv {command} for {secs:.2f} s …')
        ok = self.n._send_conv(self.n.front_conv_ac, command)
        if not ok:
            print('  Start FAILED — not running timer.')
            return
        try:
            time.sleep(secs)
        except KeyboardInterrupt:
            print('\n  Interrupted — sending stop.')
        ok2 = self.n._send_conv(self.n.front_conv_ac, 'stop')
        print(f'  Stop → {"OK" if ok2 else "FAILED"}')

    def do_home_joints(self):
        ok = self.n._send_joint(**HOME_JOINTS)
        print(f'  Home joints → {"OK" if ok else "FAILED"}')

    def do_camera_snapshot(self):
        import cv2
        import mvsdk
        from camera import Camera
        cam_ip = os.environ.get('CAMERA_IP', '10.8.4.19')
        print(f'  Connecting to camera  IP={cam_ip} ...')

        # Enumerate all visible devices and print their IPs so mismatches are obvious.
        dev_list = mvsdk.CameraEnumerateDevice()
        if not dev_list:
            print('  [FAIL] No cameras found by SDK — check GigE cable / network.')
            return
        print(f'  SDK found {len(dev_list)} device(s):')
        for i, dev in enumerate(dev_list):
            try:
                found_ip, _, _, _, _, _ = mvsdk.CameraGigeGetIp(dev)
            except Exception:
                found_ip = '(GiGE IP query failed)'
            print(f'    [{i}] {dev.GetFriendlyName()}  IP={found_ip!r}')

        cam = Camera(camera_ip=cam_ip)
        if cam.hCamera is None:
            print(f'  [FAIL] No device matched IP={cam_ip!r} — '
                  f'check IP above and update CAMERA_IP env var or default in this file.')
            return
        try:
            frame = cam.getFrame()
            print(f'  Frame: {frame.shape[1]}x{frame.shape[0]}  —  press any key to close')
            cv2.imshow('Bunsen camera snapshot', frame)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
        finally:
            cam.disable()

    # ---- stored position slots (1–7) ---------------------------------------

    def do_define_pose(self, slot: str):
        print(f'  Define pose slot {slot} (press Enter to keep current value):')
        existing = _stored_poses.get(slot)
        is_joint = (existing is not None and 'joint1' in existing) or slot in ('1', '2', '3', '6', '7')
        if is_joint:
            default = existing if existing and 'joint1' in existing else dict(joint1=0.0, joint2=0.0, joint3=0.0, joint4=0.0, joint5=-90.0, joint6=0.0)
            pose = {}
            for key in ('joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'):
                pose[key] = float(_ask(key, default[key]))
        else:
            default = existing if existing else dict(x=0.0, y=0.0, z=0.0, w=200.0, p=200.0, r=200.0)
            pose = {}
            for key in ('x', 'y', 'z', 'w', 'p', 'r'):
                pose[key] = float(_ask(key, default[key]))
        _stored_poses[slot] = pose
        print(f'  Slot {slot} saved: {pose}')

    def do_move_pose(self, slot: str):
        if slot not in _stored_poses:
            print(f'  Slot {slot} is empty — define it first with d{slot}')
            if _confirm('Define it now?'):
                self.do_define_pose(slot)
            else:
                return
        pose = _stored_poses[slot]
        print(f'  Moving to slot {slot}: {pose}')
        if 'joint1' in pose:
            ok = self.n._send_joint(**pose)
        else:
            ok = self.n._send_cart(**pose)
        print(f'  Move → {"OK" if ok else "FAILED"}')

    # ---- main loop ---------------------------------------------------------

    def run(self):
        dispatch = {
            '1': self.do_setup,   '2': self.do_wait,
            '3': self.do_grab_die,'4': self.do_pip_count,
            '5': self.do_position_pip, '6': self.do_place_die,
            '7': self.do_finish,  '8': self.do_recover,
            '9': self.do_fault,   'r': self._dump,
            'o': self.do_gripper_open,
            'k': self.do_gripper_close,
            'j': self.do_home_joints,
            'c': self.do_camera_snapshot,
        }
        while True:
            _print_menu()
            choice = input('Choice: ').strip()

            if choice.lower() == 'q':
                print('Bye.')
                break

            # Conveyor commands
            if choice == 'ff':
                fn = lambda: self._conv_cmd(self.n.front_conv_ac, 'Front conv', 'forward')
            elif choice == 'fb':
                fn = lambda: self._conv_cmd(self.n.front_conv_ac, 'Front conv', 'reverse')
            elif choice == 'fs':
                fn = lambda: self._conv_cmd(self.n.front_conv_ac, 'Front conv', 'stop')
            elif choice == 'ft':
                fn = self.do_conv_timed
            # Stored pose slots
            elif len(choice) == 2 and choice[0] == 'p' and choice[1] in '1234567':
                slot = choice[1]
                fn = lambda s=slot: self.do_move_pose(s)
            elif len(choice) == 2 and choice[0] == 'd' and choice[1] in '1234567':
                slot = choice[1]
                fn = lambda s=slot: self.do_define_pose(s)
            else:
                fn = dispatch.get(choice.lower())

            if fn is None:
                print('  Unknown option.')
                continue
            print()
            try:
                fn()
            except KeyboardInterrupt:
                print('\n  Interrupted.')
            except Exception as exc:
                import traceback
                print(f'\n  !! Exception: {exc}')
                traceback.print_exc()
            print(f'\n  State after: {STATE_NAMES.get(self.n.state, self.n.state)}')


def main():
    import rclpy

    os.environ.setdefault('BUNSEN_NAME', 'Bunsen')
    mb_host = os.environ.get('MODBUS_HOST', '127.0.0.1')
    mb_port = int(os.environ.get('MODBUS_PORT', '5020'))

    print(f'Connecting to Modbus at {mb_host}:{mb_port} ...')
    rclpy.init()

    # rclpy installs a SIGINT handler that calls rclpy.shutdown(), which
    # invalidates the node context and breaks all subsequent action calls.
    # Override it so Ctrl+C only raises KeyboardInterrupt (caught by the menu
    # loop), leaving the context intact for the next command.
    signal.signal(signal.SIGINT, lambda _s, _f: (_ for _ in ()).throw(KeyboardInterrupt()))

    BunsenDebug = _make_bunsen_debug()
    node = BunsenDebug()

    print('Waiting for action servers...')
    node._wait_for_servers()
    print('Ready.\n')

    menu = DebugMenu(node)
    try:
        menu.run()
    except KeyboardInterrupt:
        print('\nBye.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
