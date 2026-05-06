#!/usr/bin/env python3
"""
bunsen_main.py — Bunsen (Robot 2) production game controller.

Prerequisites — run in separate terminals before this:
  Terminal 1: ros2 launch launch/start.launch.py robot_name:=Bunsen robot_ip:=10.8.4.6
  Terminal 2: python3 fanuc_ros2_drivers/src/final_project/final_project/modbus_server.py

Bunsen handles EVEN pip targets: 2, 4, 6.
Beaker handles ODD pip targets:  1, 3, 5.

Flow per round:
  Wait  →  GrabDie (rear conveyor)  →  PipCount  →  PlaceDie (front conveyor, back 9.9 s)
  └── if pip == 6: Finish (place die in front of robot, exit)

Env vars (all optional):
  BUNSEN_NAME       robot namespace    (default: Bunsen)
  MODBUS_HOST       Modbus server IP   (default: 127.0.0.1)
  MODBUS_PORT       Modbus server port (default: 5020)
  CAMERA_IP         GigE camera IP     (default: 10.8.4.19)
"""

import os
import signal
import sys
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)

# Auto-source ROS2 environment if not already done.
if 'ROS_DISTRO' not in os.environ:
    _ws = os.path.normpath(os.path.join(_HERE, '..', '..', '..', 'install', 'setup.bash'))
    _cmd = 'source /opt/ros/jazzy/setup.bash'
    if os.path.exists(_ws):
        _cmd += f' && source {_ws}'
    _cmd += f' && exec python3 {os.path.abspath(__file__)}'
    os.execvp('bash', ['bash', '-c', _cmd])

from modbus_server import (
    REG_BUNSEN_STATE, REG_PIP_PROGRESS, REG_CONV_CMD, REG_RETRIES, REG_BEAKER_STATE,
    COIL_BUNSEN_READY, COIL_CAMERA_CLIENT, COIL_BEAKER_READY,
    STATE_SETUP, STATE_WAIT, STATE_GRAB_DIE, STATE_PIP_COUNT,
    STATE_PLACE_DIE, STATE_FINISH, STATE_FAULT,
    STATE_NAMES,
    CONV_IDLE, CONV_REAR_RUNNING, CONV_DIE_ON_REAR, CONV_BUNSEN_HAS_DIE,
)

# ---------------------------------------------------------------------------
# Tuning constants — adjust before each run
# ---------------------------------------------------------------------------
GRIPPER_OPEN_WIDTH  = 80    # mm  (OnRobot)
GRIPPER_OPEN_FORCE  = 20    # N
GRIPPER_CLOSE_WIDTH = 30    # mm
GRIPPER_CLOSE_FORCE = 60    # N

FRONT_CONV_REVERSE_SECS = 9.9   # how long to run front belt backward after placing die
CAMERA_SETTLE_SECS      = 0.4   # pause after moving before capturing
POLL_INTERVAL           = 0.2   # Modbus polling rate (seconds)
CONV_TIMEOUT            = 60.0  # max wait for a conveyor handshake step
MAX_PIP_RETRIES         = 20    # give up pip search after this many put-down + re-picks

# ---------------------------------------------------------------------------
# Robot poses — CALIBRATE all zeroed entries before first run
# ---------------------------------------------------------------------------
HOME_JOINTS = dict(joint1=0.0, joint2=0.0, joint3=0.0,
                   joint4=0.0, joint5=-90.0, joint6=0.0)

# Where Bunsen parks while waiting for Beaker to deliver die
CONVEYOR_WAIT_POSE = dict(x=-72.719, y=-404.0, z=352.581,
                          w=-175.773, p=0.668, r=-89.634)

# Rear conveyor — Beaker delivers die here
REAR_CONV_ABOVE  = dict(x=0.0, y=0.0, z=0.0, w=0.0, p=0.0, r=0.0)   # CALIBRATE
REAR_CONV_PICKUP = dict(x=0.0, y=0.0, z=0.0, w=0.0, p=0.0, r=0.0)   # CALIBRATE

# Central safe hold location — robot moves here after picking die
SAFE_HOLD_POSE = dict(x=0.0, y=0.0, z=0.0, w=0.0, p=0.0, r=0.0)     # CALIBRATE

# Camera positions — joint angles at which Bunsen presents die to overhead camera
CAM_POSE_1 = dict(joint1=-70.999, joint2=51.903, joint3=8.141,
                  joint4=174.404, joint5=-79.550, joint6=23.121)
CAM_POSE_2 = dict(joint1=-70.999, joint2=51.903, joint3=8.141,
                  joint4=174.404, joint5=-79.550, joint6=23.121)      # CALIBRATE second angle

# Table spot for die re-orientation — put die down here between pip-count attempts
TABLE_PLACE_ABOVE = dict(x=0.0, y=0.0, z=0.0, w=0.0, p=0.0, r=0.0)  # CALIBRATE
TABLE_PLACE_DOWN  = dict(x=0.0, y=0.0, z=0.0, w=0.0, p=0.0, r=0.0)  # CALIBRATE

# Front conveyor — Bunsen places die here to send back to Beaker
FRONT_CONV_ABOVE = dict(x=126.961, y=-582.177, z=152.102,
                        w=-175.773, p=0.668, r=-89.634)
FRONT_CONV_PLACE = dict(x=126.961, y=-582.177, z=52.102,
                        w=-175.773, p=0.668, r=-89.634)

# Final placement — pip 6, placed in front of Bunsen for display
FINAL_PLACE_ABOVE = dict(x=450.577, y=3.423, z=0.0,
                         w=-175.671, p=0.681, r=-89.632)
FINAL_PLACE_DOWN  = dict(x=450.577, y=3.423, z=-141.722,
                         w=-175.671, p=0.681, r=-89.632)


# ===========================================================================
# BunsenMain — ROS2 node, full state machine
# ===========================================================================

def _make_node():
    import rclpy
    from rclpy.node import Node
    from rclpy.action import ActionClient
    from fanuc_interfaces.action import CartPose, JointPose, OnRobotGripper, Conveyor
    from pymodbus.client import ModbusTcpClient
    from camera import Camera
    from pip_counter import count_pips

    class BunsenMain(Node):

        def __init__(self):
            ns      = os.environ.get('BUNSEN_NAME',  'Bunsen')
            mb_host = os.environ.get('MODBUS_HOST',  '127.0.0.1')
            mb_port = int(os.environ.get('MODBUS_PORT', '5020'))
            self._cam_ip = os.environ.get('CAMERA_IP', '10.8.4.19')

            Node.__init__(self, 'bunsen_main')

            self._cart    = ActionClient(self, CartPose,       f'/{ns}/cartesian_pose')
            self._joint   = ActionClient(self, JointPose,      f'/{ns}/joint_pose')
            self._gripper = ActionClient(self, OnRobotGripper, f'/{ns}/onrobot_gripper')
            self._conv    = ActionClient(self, Conveyor,       f'/{ns}/conveyor')

            self.mb = ModbusTcpClient(mb_host, port=mb_port)
            if not self.mb.connect():
                raise RuntimeError(f'Cannot connect to Modbus at {mb_host}:{mb_port}')
            self.get_logger().info(f'Modbus connected to {mb_host}:{mb_port}')

            self._camera = None
            self._bunsen_retries = 0

        # ── Modbus helpers ────────────────────────────────────────────────────

        def _mb_read(self, addr):
            return self.mb.read_holding_registers(addr, 1).registers[0]

        def _mb_write(self, addr, val):
            self.mb.write_register(addr, val)

        def _mb_read_coil(self, addr):
            return bool(self.mb.read_coils(addr, 1).bits[0])

        def _mb_write_coil(self, addr, val):
            self.mb.write_coil(addr, bool(val))

        def _set_state(self, state):
            self._mb_write(REG_BUNSEN_STATE, state)
            self.get_logger().info(f'→ {STATE_NAMES.get(state, state)}')

        def _wait_mb(self, reg_or_coil, target, is_coil=False, timeout=CONV_TIMEOUT):
            deadline = time.time() + timeout
            while time.time() < deadline:
                val = self._mb_read_coil(reg_or_coil) if is_coil \
                      else self._mb_read(reg_or_coil)
                if val == target:
                    return True
                time.sleep(POLL_INTERVAL)
            return False

        # ── Action helpers ────────────────────────────────────────────────────

        def _send_cart(self, **kw) -> bool:
            if not self._cart.wait_for_server(timeout_sec=5.0):
                self.get_logger().error('CartPose server timeout')
                return False
            goal = CartPose.Goal()
            for k, v in kw.items():
                setattr(goal, k, float(v))
            fut = self._cart.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, fut)
            gh = fut.result()
            if not gh.accepted: return False
            res = gh.get_result_async()
            rclpy.spin_until_future_complete(self, res)
            return res.result().result.success

        def _send_joint(self, **kw) -> bool:
            if not self._joint.wait_for_server(timeout_sec=5.0):
                self.get_logger().error('JointPose server timeout')
                return False
            goal = JointPose.Goal()
            for k, v in kw.items():
                setattr(goal, k, float(v))
            fut = self._joint.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, fut)
            gh = fut.result()
            if not gh.accepted: return False
            res = gh.get_result_async()
            rclpy.spin_until_future_complete(self, res)
            return res.result().result.success

        def _open_gripper(self) -> bool:
            if not self._gripper.wait_for_server(timeout_sec=5.0):
                self.get_logger().error('Gripper server timeout')
                return False
            goal = OnRobotGripper.Goal()
            goal.width = GRIPPER_OPEN_WIDTH
            goal.force = GRIPPER_OPEN_FORCE
            fut = self._gripper.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, fut)
            gh = fut.result()
            if not gh.accepted: return False
            res = gh.get_result_async()
            rclpy.spin_until_future_complete(self, res)
            return res.result().result.success

        def _close_gripper(self) -> bool:
            if not self._gripper.wait_for_server(timeout_sec=5.0):
                self.get_logger().error('Gripper server timeout')
                return False
            goal = OnRobotGripper.Goal()
            goal.width = GRIPPER_CLOSE_WIDTH
            goal.force = GRIPPER_CLOSE_FORCE
            fut = self._gripper.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, fut)
            gh = fut.result()
            if not gh.accepted: return False
            res = gh.get_result_async()
            rclpy.spin_until_future_complete(self, res)
            return res.result().result.success

        def _run_conveyor(self, command: str) -> bool:
            if not self._conv.wait_for_server(timeout_sec=5.0):
                self.get_logger().error('Conveyor server timeout')
                return False
            goal = Conveyor.Goal()
            goal.command = command
            fut = self._conv.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, fut)
            gh = fut.result()
            if not gh.accepted: return False
            res = gh.get_result_async()
            rclpy.spin_until_future_complete(self, res)
            return res.result().result.success

        def _capture_pip(self) -> int:
            """Grab a frame and count pips; returns 1–6 or -1 on failure."""
            if self._camera is None or self._camera.hCamera is None:
                self.get_logger().warn('Camera not available')
                return -1
            try:
                frame = self._camera.getFrame()
                pips = count_pips(frame)
                self.get_logger().info(f'Camera: {pips} pip(s)')
                return pips
            except Exception as e:
                self.get_logger().error(f'Camera capture failed: {e}')
                return -1

        # ====================================================================
        # State: SETUP
        # ====================================================================

        def _state_setup(self):
            print('\n' + '=' * 55)
            print('  BUNSEN starting up — clearing Modbus registers...')
            print('=' * 55)

            # Clear everything
            self._mb_write(REG_BUNSEN_STATE, STATE_SETUP)
            self._mb_write(REG_BEAKER_STATE, 0)
            self._mb_write(REG_PIP_PROGRESS, 1)      # pip 1 = Beaker's first target
            self._mb_write(REG_CONV_CMD,     CONV_IDLE)
            self._mb_write(REG_RETRIES,      0)
            self._mb_write_coil(COIL_BUNSEN_READY,  False)
            self._mb_write_coil(COIL_CAMERA_CLIENT, False)
            self._mb_write_coil(COIL_BEAKER_READY,  False)

            print('  Waiting for ROS2 action servers...')
            self._cart.wait_for_server()
            self._joint.wait_for_server()
            self._gripper.wait_for_server()
            self._conv.wait_for_server()
            print('  Action servers ready.')

            print(f'  Opening camera at {self._cam_ip}...')
            self._camera = Camera(camera_ip=self._cam_ip)
            if self._camera.hCamera is not None:
                print('  Camera ready.')
            else:
                print('  [WARN] Camera not found — captures will return -1.')

            self._send_joint(**HOME_JOINTS)
            self._set_state(STATE_WAIT)
            print('  Setup complete. pip_progress=1 (Beaker targets pip 1 first).\n')

        # ====================================================================
        # State: WAIT
        # ====================================================================

        def _state_wait(self):
            self._set_state(STATE_WAIT)
            self._mb_write_coil(COIL_BUNSEN_READY, False)

            pip = self._mb_read(REG_PIP_PROGRESS)
            print(f'  [Wait] Moved to conveyor wait pose.  '
                  f'Expecting Beaker to deliver pip {pip + 1 if pip % 2 == 1 else pip}...')

            self._send_cart(**CONVEYOR_WAIT_POSE)

            print('  [Wait] Polling for Beaker state=PlaceDie AND Beaker ready coil...')
            while True:
                beaker_state = self._mb_read(REG_BEAKER_STATE)
                beaker_ready = self._mb_read_coil(COIL_BEAKER_READY)
                if beaker_state == STATE_PLACE_DIE and beaker_ready:
                    break
                time.sleep(POLL_INTERVAL)

            print('  [Wait] Beaker ready — moving to GrabDie.')
            self._set_state(STATE_GRAB_DIE)

        # ====================================================================
        # State: GRAB_DIE
        # ====================================================================

        def _state_grab_die(self):
            self._set_state(STATE_GRAB_DIE)
            print('  [GrabDie] Starting rear conveyor to receive die...')

            self._run_conveyor('forward')
            self._mb_write(REG_CONV_CMD, CONV_REAR_RUNNING)

            print('  [GrabDie] Waiting for Beaker to signal die on belt...')
            if not self._wait_mb(REG_CONV_CMD, CONV_DIE_ON_REAR):
                print('  [GrabDie] ERROR: timeout waiting for CONV_DIE_ON_REAR.')
                self._run_conveyor('stop')
                self._set_state(STATE_FAULT)
                return

            self._run_conveyor('stop')

            print('  [GrabDie] Picking die from rear conveyor...')
            self._open_gripper()
            self._send_cart(**REAR_CONV_ABOVE)
            self._send_cart(**REAR_CONV_PICKUP)
            self._close_gripper()
            self._send_cart(**REAR_CONV_ABOVE)

            # Acknowledge pickup in Modbus
            self._mb_write(REG_CONV_CMD, CONV_BUNSEN_HAS_DIE)
            # Beaker clears COIL_BEAKER_READY when it sees BUNSEN_HAS_DIE
            # Wait briefly for Beaker to reset, then clear conv to idle
            time.sleep(1.0)
            self._mb_write(REG_CONV_CMD, CONV_IDLE)

            print('  [GrabDie] Die in hand — moving to safe hold pose.')
            self._send_cart(**SAFE_HOLD_POSE)
            self._set_state(STATE_PIP_COUNT)

        # ====================================================================
        # State: PIP_COUNT
        # ====================================================================

        def _state_pip_count(self):
            self._set_state(STATE_PIP_COUNT)
            target = self._mb_read(REG_PIP_PROGRESS)

            if target % 2 == 1:
                # pip_progress still shows odd (Beaker's pip) — Bunsen targets next even
                target = target + 1

            print(f'  [PipCount] Target pip: {target}')

            for attempt in range(1, MAX_PIP_RETRIES + 1):
                print(f'  [PipCount] Attempt {attempt}/{MAX_PIP_RETRIES}...')

                # Photo 1
                self._send_joint(**CAM_POSE_1)
                time.sleep(CAMERA_SETTLE_SECS)
                face1 = self._capture_pip()
                print(f'    CAM_POSE_1 → {face1} pip(s)')

                if face1 == target:
                    print(f'  [PipCount] Target pip {target} confirmed at CAM_POSE_1!')
                    self._mb_write(REG_PIP_PROGRESS, target)
                    self._set_state(STATE_FINISH if target == 6 else STATE_PLACE_DIE)
                    return

                # Photo 2
                self._send_joint(**CAM_POSE_2)
                time.sleep(CAMERA_SETTLE_SECS)
                face2 = self._capture_pip()
                print(f'    CAM_POSE_2 → {face2} pip(s)')

                if face2 == target:
                    print(f'  [PipCount] Target pip {target} confirmed at CAM_POSE_2!')
                    self._mb_write(REG_PIP_PROGRESS, target)
                    self._set_state(STATE_FINISH if target == 6 else STATE_PLACE_DIE)
                    return

                # Neither position shows target — put die on table and re-pick
                print(f'  [PipCount] pip {target} not visible — '
                      f'placing die on table to rotate...')
                self._bunsen_retries += 1
                self._mb_write(REG_RETRIES, self._bunsen_retries)

                self._send_cart(**TABLE_PLACE_ABOVE)
                self._send_cart(**TABLE_PLACE_DOWN)
                self._open_gripper()
                self._send_cart(**TABLE_PLACE_ABOVE)
                # Re-grip from same position; contact physics re-orients the die
                self._send_cart(**TABLE_PLACE_DOWN)
                self._close_gripper()
                self._send_cart(**TABLE_PLACE_ABOVE)

            print(f'  [PipCount] ERROR: exceeded {MAX_PIP_RETRIES} attempts.')
            self._set_state(STATE_FAULT)

        # ====================================================================
        # State: PLACE_DIE
        # ====================================================================

        def _state_place_die(self):
            self._set_state(STATE_PLACE_DIE)
            target = self._mb_read(REG_PIP_PROGRESS)
            print(f'  [PlaceDie] Placing pip {target} on front conveyor...')

            self._send_cart(**FRONT_CONV_ABOVE)
            self._send_cart(**FRONT_CONV_PLACE)
            self._open_gripper()
            self._send_cart(**FRONT_CONV_ABOVE)

            print(f'  [PlaceDie] Running front conveyor backward for '
                  f'{FRONT_CONV_REVERSE_SECS}s...')
            self._run_conveyor('reverse')
            time.sleep(FRONT_CONV_REVERSE_SECS)
            self._run_conveyor('stop')

            # Signal Beaker that die is on front conveyor and ready to grab
            self._mb_write_coil(COIL_BUNSEN_READY, True)
            self._mb_write(REG_PIP_PROGRESS, target + 1)  # next odd pip for Beaker
            print(f'  [PlaceDie] Done. pip_progress → {target + 1}. '
                  f'BUNSEN_READY=1 (Beaker may pick up).')

            self._set_state(STATE_WAIT)

        # ====================================================================
        # State: FINISH
        # ====================================================================

        def _state_finish(self):
            self._set_state(STATE_FINISH)
            print('\n  [Finish] Pip 6 confirmed — placing die in front of Bunsen...')

            self._send_cart(**FINAL_PLACE_ABOVE)
            self._send_cart(**FINAL_PLACE_DOWN)
            self._open_gripper()
            self._send_cart(**FINAL_PLACE_ABOVE)
            self._send_joint(**HOME_JOINTS)

            self._mb_write_coil(COIL_BUNSEN_READY,  False)
            self._mb_write_coil(COIL_CAMERA_CLIENT, False)

            retries = self._mb_read(REG_RETRIES)
            print('\n' + '=' * 55)
            print('         BUNSEN DONE — pip 6 placed!')
            print(f'         Bunsen repositions : {retries}')
            print('=' * 55 + '\n')

        # ====================================================================
        # Main loop
        # ====================================================================

        def run(self):
            self._state_setup()

            while True:
                state = self._mb_read(REG_BUNSEN_STATE)

                if state == STATE_WAIT:
                    self._state_wait()

                elif state == STATE_GRAB_DIE:
                    self._state_grab_die()

                elif state == STATE_PIP_COUNT:
                    self._state_pip_count()

                elif state == STATE_PLACE_DIE:
                    self._state_place_die()

                elif state == STATE_FINISH:
                    self._state_finish()
                    break

                elif state == STATE_FAULT:
                    print('\n  [FAULT] Entering safe stop.')
                    self._run_conveyor('stop')
                    self._open_gripper()
                    self._send_joint(**HOME_JOINTS)
                    self._mb_write_coil(COIL_BUNSEN_READY, False)
                    break

                else:
                    print(f'  [WARN] Unknown state {state} — resetting to Wait.')
                    self._set_state(STATE_WAIT)

        def destroy_node(self):
            if self._camera is not None:
                self._camera.disable()
            self.mb.close()
            super().destroy_node()

    return BunsenMain


def main():
    import rclpy

    rclpy.init()
    # Override SIGINT: let Ctrl+C raise KeyboardInterrupt without destroying context.
    signal.signal(signal.SIGINT, lambda _s, _f: (_ for _ in ()).throw(KeyboardInterrupt()))

    BunsenMain = _make_node()
    node = BunsenMain()
    try:
        node.run()
    except KeyboardInterrupt:
        print('\nInterrupted — shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
