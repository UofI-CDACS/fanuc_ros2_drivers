#!/usr/bin/env python3
"""
collect_training.py — Robot pick-and-hold for YOLOv8 training image collection.

Workflow:
  1. Place die on table, press ENTER → robot picks and moves to AT_CAMERA
  2. Type in terminal (no need to click camera window):
       SPACE : save current frame
       N     : open/close gripper (to reposition die by hand)
       M     : grab die and move back to AT_CAMERA (use after N to re-grip)
       R     : return die to table, loop back to step 1
       ESC   : return die and quit

Images saved to ~/Desktop/training_images/die_NNNN.png

Usage: python3 collect_training.py <robot_name>
"""
import os, sys, select, termios, tty


def _load_config():
    path = os.path.join(os.path.dirname(__file__), 'config.env')
    try:
        with open(path) as f:
            for line in f:
                line = line.strip()
                if line and not line.startswith('#') and '=' in line:
                    k, _, v = line.partition('=')
                    os.environ.setdefault(k.strip(), v.strip())
    except FileNotFoundError:
        pass

_load_config()

# ── Mindvision SDK ────────────────────────────────────────────────────────────
_SDK_BASE = os.path.abspath(
    os.path.join(os.path.dirname(__file__), '..', 'Mindvision SDK for linux V2.1.0.37')
)
_LIB_PATH = os.path.join(_SDK_BASE, 'lib', 'x64')
os.environ['LD_LIBRARY_PATH'] = _LIB_PATH + ':' + os.environ.get('LD_LIBRARY_PATH', '')

import ctypes
try:
    ctypes.cdll.LoadLibrary('libMVSDK.so')
except OSError:
    os.execve(sys.executable, [sys.executable] + sys.argv,
              {**os.environ, 'LD_LIBRARY_PATH': os.environ['LD_LIBRARY_PATH']})

sys.path.insert(0, os.path.dirname(__file__))
import mvsdk
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from fanuc_interfaces.action import CartPose, JointPose, SchunkGripper

# ── Positions (from beaker.py) ────────────────────────────────────────────────
HOME      = dict(joint1=1.1, joint2=1.5, joint3=-2.0, joint4=-1.7, joint5=-88.6, joint6=-30.0)
ABOVE_DIE = dict(x=465.0, y=-15.0, z=-145.0, w=179.9, p=0.0,   r=30.0)
DIE_HOME  = dict(x=465.0, y=-15.0, z=-185.0, w=179.9, p=0.0,   r=30.0)
AT_CAMERA = dict(x=477.0, y=1145.0, z=842.0,  w=85.0,  p=-63.0, r=-177.0)

# ── Config ────────────────────────────────────────────────────────────────────
CAMERA_IP = os.environ.get('CAMERA_IP', '')
SAVE_DIR  = os.path.expanduser('~/Desktop/training_images')


class TrainingCollector(Node):
    def __init__(self, robot_name: str):
        super().__init__('training_collector')
        self.joint_ac   = ActionClient(self, JointPose,     f'/{robot_name}/joint_pose')
        self.cart_ac    = ActionClient(self, CartPose,      f'/{robot_name}/cartesian_pose')
        self.gripper_ac = ActionClient(self, SchunkGripper, f'/{robot_name}/schunk_gripper')

    def _send_joint(self, pos: dict) -> bool:
        self.joint_ac.wait_for_server()
        goal = JointPose.Goal()
        for k in ('joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'):
            setattr(goal, k, float(pos[k]))
        future = self.joint_ac.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        gh = future.result()
        if not gh.accepted:
            return False
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rf)
        return rf.result().result.success

    def _send_cart(self, pos: dict) -> bool:
        self.cart_ac.wait_for_server()
        goal = CartPose.Goal()
        goal.x, goal.y, goal.z = float(pos['x']), float(pos['y']), float(pos['z'])
        goal.w, goal.p, goal.r = float(pos['w']), float(pos['p']), float(pos['r'])
        future = self.cart_ac.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        gh = future.result()
        if not gh.accepted:
            return False
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rf)
        return rf.result().result.success

    def _send_gripper(self, command: str) -> bool:
        self.gripper_ac.wait_for_server()
        goal = SchunkGripper.Goal()
        goal.command = command
        future = self.gripper_ac.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        gh = future.result()
        if not gh.accepted:
            return False
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rf)
        return rf.result().result.success

    def pick(self) -> bool:
        print('Opening gripper...')
        if not self._send_gripper('open'):
            return False
        print('Moving to ABOVE_DIE...')
        if not self._send_cart(ABOVE_DIE):
            return False
        print('Moving to DIE_HOME...')
        if not self._send_cart(DIE_HOME):
            return False
        print('Closing gripper...')
        if not self._send_gripper('close'):
            return False
        print('Moving to HOME...')
        if not self._send_joint(HOME):
            return False
        print('Moving to AT_CAMERA...')
        if not self._send_cart(AT_CAMERA):
            return False
        return True

    def regrab(self) -> bool:
        """From AT_CAMERA with gripper open: go pick die and return to AT_CAMERA."""
        print('Moving to HOME...')
        if not self._send_joint(HOME):
            return False
        print('Moving to ABOVE_DIE...')
        if not self._send_cart(ABOVE_DIE):
            return False
        print('Moving to DIE_HOME...')
        if not self._send_cart(DIE_HOME):
            return False
        print('Closing gripper...')
        if not self._send_gripper('close'):
            return False
        print('Moving to HOME...')
        if not self._send_joint(HOME):
            return False
        print('Moving to AT_CAMERA...')
        if not self._send_cart(AT_CAMERA):
            return False
        return True

    def return_die(self) -> bool:
        print('Moving to HOME...')
        if not self._send_joint(HOME):
            return False
        print('Moving to ABOVE_DIE...')
        if not self._send_cart(ABOVE_DIE):
            return False
        print('Moving to DIE_HOME...')
        if not self._send_cart(DIE_HOME):
            return False
        print('Opening gripper...')
        if not self._send_gripper('open'):
            return False
        print('Moving to HOME...')
        if not self._send_joint(HOME):
            return False
        return True


def init_camera():
    DevList = mvsdk.CameraEnumerateDevice()
    DevInfo = None
    for dev in DevList:
        try:
            cam_ip, *_ = mvsdk.CameraGigeGetIp(dev)
            if cam_ip == CAMERA_IP:
                DevInfo = dev
                break
        except Exception:
            pass
    if DevInfo is None:
        raise RuntimeError(f'No camera found at {CAMERA_IP}')
    hCamera = mvsdk.CameraInit(DevInfo, -1, -1)
    cap = mvsdk.CameraGetCapability(hCamera)
    mono = cap.sIspCapacity.bMonoSensor != 0
    mvsdk.CameraSetIspOutFormat(
        hCamera,
        mvsdk.CAMERA_MEDIA_TYPE_MONO8 if mono else mvsdk.CAMERA_MEDIA_TYPE_BGR8
    )
    mvsdk.CameraSetTriggerMode(hCamera, 0)
    mvsdk.CameraSetAeState(hCamera, 1)
    channels = 1 if mono else 3
    buf_size = cap.sResolutionRange.iWidthMax * cap.sResolutionRange.iHeightMax * channels
    pFrameBuffer = mvsdk.CameraAlignMalloc(buf_size, 16)
    mvsdk.CameraPlay(hCamera)
    return hCamera, pFrameBuffer, channels


def grab_frame(hCamera, pFrameBuffer, channels):
    pRawData, FrameHead = mvsdk.CameraGetImageBuffer(hCamera, 200)
    mvsdk.CameraImageProcess(hCamera, pRawData, pFrameBuffer, FrameHead)
    mvsdk.CameraReleaseImageBuffer(hCamera, pRawData)
    frame_data = (mvsdk.c_ubyte * FrameHead.uBytes).from_address(pFrameBuffer)
    return np.frombuffer(frame_data, dtype=np.uint8).reshape(
        (FrameHead.iHeight, FrameHead.iWidth, channels)
    ).copy()


def viewfinder_loop(hCamera, pFrameBuffer, channels, save_count, node):
    """Live view with terminal key input. Returns (new_save_count, action).
    action: 'home' = go to HOME holding die, 'quit' = put die back and quit.
    """
    gripper_open = False  # gripper is closed when we arrive (holding die)
    print('SPACE=save  N=open/close gripper  M=regrab+AT_CAMERA  R=go HOME  ESC=put back+quit')

    while True:
        try:
            frame = grab_frame(hCamera, pFrameBuffer, channels)
        except mvsdk.CameraException as e:
            if e.error_code != mvsdk.CAMERA_STATUS_TIME_OUT:
                print(f'Grab error: {e.message}')
            continue

        gripper_label = 'OPEN' if gripper_open else 'CLOSED'
        display = frame.copy()
        cv2.putText(display,
                    f'[{save_count} saved] gripper={gripper_label}  SPACE/N/M/R/ESC',
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)
        cv2.imshow('Training Capture', display)
        cv2.waitKey(1)

        if not select.select([sys.stdin], [], [], 0)[0]:
            continue
        ch = sys.stdin.read(1)

        if ch == ' ':
            filename = os.path.join(SAVE_DIR, f'die_{save_count:04d}.png')
            cv2.imwrite(filename, frame)
            print(f'Saved: {filename}')
            save_count += 1
        elif ch in ('n', 'N'):
            if gripper_open:
                print('Closing gripper...')
                node._send_gripper('close')
                gripper_open = False
                print('Gripper closed.')
            else:
                print('Opening gripper...')
                node._send_gripper('open')
                gripper_open = True
                print('Gripper open — reposition die then press M to regrab.')
        elif ch in ('m', 'M'):
            if not gripper_open:
                print('Gripper already closed. Press N to open first.')
            else:
                print('Grabbing die and returning to AT_CAMERA...')
                if node.regrab():
                    gripper_open = False
                    print('Ready.')
                else:
                    print('ERROR: regrab failed.')
        elif ch in ('r', 'R'):
            if gripper_open:
                node._send_gripper('close')
                gripper_open = False
            print('Moving to HOME...')
            node._send_joint(HOME)
            return save_count, 'home'
        elif ch == '\x1b':  # ESC
            if gripper_open:
                node._send_gripper('close')
                gripper_open = False
            return save_count, 'quit'


def main():
    if len(sys.argv) < 2:
        print('Usage: python3 collect_training.py <robot_name>')
        sys.exit(1)

    os.makedirs(SAVE_DIR, exist_ok=True)
    existing = [f for f in os.listdir(SAVE_DIR) if f.endswith('.png')]
    save_count = len(existing)
    print(f'Saving to: {SAVE_DIR}')
    print(f'Starting from die_{save_count:04d}.png\n')

    rclpy.init()
    node = TrainingCollector(sys.argv[1])
    hCamera, pFrameBuffer, channels = init_camera()

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    try:
        while True:
            # Restore normal terminal for ENTER prompt
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            print('Place die on table, then press ENTER to pick...')
            input()

            if not node.pick():
                print('ERROR: pick sequence failed, returning HOME')
                node._send_joint(HOME)
                break

            # Inner loop: viewfinder → HOME → back to AT_CAMERA, until ESC
            while True:
                tty.setcbreak(fd)
                save_count, action = viewfinder_loop(hCamera, pFrameBuffer, channels, save_count, node)

                if action == 'quit':
                    node.return_die()
                    break

                # action == 'home': robot is at HOME holding die
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
                print('Adjust die, then press ENTER to go back to AT_CAMERA...')
                input()
                print('Moving to AT_CAMERA...')
                node._send_cart(AT_CAMERA)

            if action == 'quit':
                break

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        cv2.destroyAllWindows()
        mvsdk.CameraUnInit(hCamera)
        mvsdk.CameraAlignFree(pFrameBuffer)
        node.destroy_node()
        rclpy.shutdown()
        print(f'\nDone. {save_count} total image(s) in {SAVE_DIR}')


if __name__ == '__main__':
    main()
