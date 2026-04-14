#!/usr/bin/env python3
"""
beaker.py — Three-face die inspection sequence.

For each of 3 picks:
  1. Pick die from table (always same pick orientation)
  2. Move to camera, capture image, count pips, print count
  3. Set die back down rotated 90° from how it was picked up
  4. Return to HOME  (next pick grabs the die at the new orientation)

After all 3 picks: print a summary table with per-pic pip counts and total sum.

Usage: python3 beaker.py <robot_name>
"""
import os, sys, numpy as np


def _load_config():
    """Load key=value pairs from config.env (same directory) into os.environ."""
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

# ── Mindvision SDK — ensure libMVSDK.so is loadable before any mvsdk import ──
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

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from fanuc_interfaces.action import CartPose, JointPose, SchunkGripper

# ── Positions ─────────────────────────────────────────────────────────────────
HOME      = dict(joint1=1.1, joint2=1.5, joint3=-2.0, joint4=-1.7, joint5=-88.6, joint6=-30.0)
ABOVE_DIE = dict(x=465.0, y=-15.0, z=-145.0, w=179.9, p=0.0, r=30.0)
DIE_HOME  = dict(x=465.0, y=-15.0, z=-185.0, w=179.9, p=0.0, r=30.0)
AT_CAMERA = dict(x=477.0, y=1145.0, z=842.0,  w=85.0, p=-63.0, r=-177.0)

# Set-down rotates the wrist 90° relative to the pick so each subsequent pick
# grabs the die at a new face without the robot wrist angle accumulating
SETDOWN_R_OFFSET = 90.0

# ── Camera ────────────────────────────────────────────────────────────────────
CAMERA_IP = os.environ.get('CAMERA_IP', '')
DESKTOP   = os.path.expanduser('~/Desktop')

# ── YOLO model ────────────────────────────────────────────────────────────────
_MODEL_NAME = os.environ.get('MODEL_NAME', 'dice_class5')
MODEL_PATH  = os.path.join(os.path.dirname(__file__), 'runs', 'classify', _MODEL_NAME, 'weights', 'best.pt')
_CLASS_TO_PIPS = {'one': 1, 'two': 2, 'three': 3, 'four': 4, 'five': 5, 'six': 6}


# ── Camera helpers ────────────────────────────────────────────────────────────

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
    pRawData, FrameHead = mvsdk.CameraGetImageBuffer(hCamera, 2000)
    mvsdk.CameraImageProcess(hCamera, pRawData, pFrameBuffer, FrameHead)
    mvsdk.CameraReleaseImageBuffer(hCamera, pRawData)
    frame_data = (mvsdk.c_ubyte * FrameHead.uBytes).from_address(pFrameBuffer)
    return np.frombuffer(frame_data, dtype=np.uint8).reshape(
        (FrameHead.iHeight, FrameHead.iWidth, channels)
    ).copy()


def flush_and_grab(hCamera, pFrameBuffer, channels):
    """Drain any frames that queued up while the robot was moving, then grab a fresh one."""
    while True:
        try:
            pRawData, _ = mvsdk.CameraGetImageBuffer(hCamera, 50)
            mvsdk.CameraReleaseImageBuffer(hCamera, pRawData)
        except mvsdk.CameraException as e:
            if e.error_code == mvsdk.CAMERA_STATUS_TIME_OUT:
                break
            raise
    return grab_frame(hCamera, pFrameBuffer, channels)


# ── Pip counting ──────────────────────────────────────────────────────────────

def count_pips(frame, model, annot_path: str) -> int:
    results = model(frame, verbose=False)
    top_class = results[0].names[results[0].probs.top1]
    count = _CLASS_TO_PIPS.get(top_class, 0)
    annotated = frame.copy()
    cv2.putText(annotated, f'Pips: {count} ({top_class})', (10, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 1.4, (0, 0, 255), 2)
    cv2.imwrite(annot_path, annotated)
    return count


# ── Robot node ────────────────────────────────────────────────────────────────

class Beaker(Node):
    def __init__(self, robot_name: str):
        super().__init__('beaker')
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

    def _move(self, label: str, fn, pos: dict) -> bool:
        print(f'  Moving to {label}...')
        if not fn(pos):
            print(f'  ERROR: failed to reach {label}')
            return False
        return True

    def run(self):
        from ultralytics import YOLO

        print('Initialising camera...')
        hCamera, pFrameBuffer, channels = init_camera()
        print('Loading YOLO model...')
        model = YOLO(MODEL_PATH)

        pip_counts = []

        try:
            print('\nMoving to HOME...')
            if not self._send_joint(HOME):
                print('ERROR: failed to reach HOME'); return

            # Set-down positions: same XYZ as pick but wrist rotated 90°
            above_down = {**ABOVE_DIE, 'r': ABOVE_DIE['r'] + SETDOWN_R_OFFSET}
            pick_down  = {**DIE_HOME,  'r': DIE_HOME['r']  + SETDOWN_R_OFFSET}

            for i in range(3):
                pic_num = i + 1
                print(f'\n{"─"*40}')
                print(f'Pick {pic_num}/3')
                print(f'{"─"*40}')

                # ── Pick up die (always same pick orientation) ─────────────────
                if not self._send_gripper('open'):
                    print('ERROR: gripper open failed'); return
                if not self._move('ABOVE_DIE', self._send_cart, ABOVE_DIE): return
                if not self._move('DIE_HOME',  self._send_cart, DIE_HOME):  return
                if not self._send_gripper('close'):
                    print('ERROR: gripper close failed'); return
                if not self._move('HOME', self._send_joint, HOME): return

                # ── Camera ────────────────────────────────────────────────────
                if not self._move('AT_CAMERA', self._send_cart, AT_CAMERA): return

                print('  Capturing image...')
                frame = flush_and_grab(hCamera, pFrameBuffer, channels)
                save_path  = os.path.join(DESKTOP, f'capture_{pic_num}.png')
                annot_path = os.path.join(DESKTOP, f'annotated_{pic_num}.png')
                cv2.imwrite(save_path, frame)

                count = count_pips(frame, model, annot_path)
                pip_counts.append(count)
                print(f'  >>> Pic {pic_num}: {count} pip(s)')

                # ── Set die down rotated 90° from pick ────────────────────────
                if not self._move('HOME',      self._send_joint, HOME):       return
                if not self._move('ABOVE_DIE', self._send_cart,  above_down): return
                if not self._move('DIE_HOME',  self._send_cart,  pick_down):  return
                if not self._send_gripper('open'):
                    print('ERROR: gripper open failed'); return
                if not self._move('HOME', self._send_joint, HOME): return

            # ── Summary table ─────────────────────────────────────────────────
            print(f'\n{"═"*30}')
            print(f'  {"Pic":<8} {"Pips":>6}')
            print(f'  {"─"*20}')
            for idx, c in enumerate(pip_counts):
                print(f'  Pic {idx+1:<4}  {c:>6}')
            print(f'  {"─"*20}')
            print(f'  {"Total":<8} {sum(pip_counts):>6}')
            print(f'{"═"*30}')

        finally:
            mvsdk.CameraUnInit(hCamera)
            mvsdk.CameraAlignFree(pFrameBuffer)


def main():
    if len(sys.argv) < 2:
        print('Usage: python3 beaker.py <robot_name>')
        sys.exit(1)

    rclpy.init()
    node = Beaker(sys.argv[1])
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
