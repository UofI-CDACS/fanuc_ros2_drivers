#!/usr/bin/env python3
"""
Bunsen debug runner — sends commands through the running BunsenServer.
Requires BunsenServer and modbus_server.py to be running first.

Usage (from any directory):
  python3 fanuc_ros2_drivers/src/final_project/final_project/bunsen_debug.py
"""

import os
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

MENU = """
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
║  q  Quit                                     ║
╚══════════════════════════════════════════════╝"""


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

    class BunsenDebug(BunsenMaster):
        """BunsenMaster wired to the live action servers; camera replaced by manual input."""

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

    # ---- main loop ---------------------------------------------------------

    def run(self):
        dispatch = {
            '1': self.do_setup,   '2': self.do_wait,
            '3': self.do_grab_die,'4': self.do_pip_count,
            '5': self.do_position_pip, '6': self.do_place_die,
            '7': self.do_finish,  '8': self.do_recover,
            '9': self.do_fault,   'r': self._dump,
        }
        while True:
            print(MENU)
            choice = input('Choice: ').strip().lower()
            if choice == 'q':
                print('Bye.')
                break
            fn = dispatch.get(choice)
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
