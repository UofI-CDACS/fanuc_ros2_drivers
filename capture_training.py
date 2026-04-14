#!/usr/bin/env python3
"""
capture_training.py — Live camera viewer for collecting YOLOv8 training images.

Type in the terminal (no need to click the camera window):
  SPACE  : save current frame
  ESC    : quit

Images are saved to SAVE_DIR with sequential filenames (die_0000.png, die_0001.png, ...).
After capturing, label them with a tool like LabelImg or Roboflow, then train with train.py.
"""
import os
import sys
import select
import termios
import tty


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

# ── Config ────────────────────────────────────────────────────────────────────
CAMERA_IP = os.environ.get('CAMERA_IP', '')
SAVE_DIR  = os.path.expanduser('~/Desktop/training_images')
# ─────────────────────────────────────────────────────────────────────────────


def main():
    os.makedirs(SAVE_DIR, exist_ok=True)
    existing = [f for f in os.listdir(SAVE_DIR) if f.endswith('.png')]
    save_count = len(existing)

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
        print(f'ERROR: no camera found at {CAMERA_IP}')
        sys.exit(1)

    print(f'Camera: {DevInfo.GetFriendlyName()} ({DevInfo.GetPortType()})')

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

    print(f'Saving to: {SAVE_DIR}')
    print(f'Already have {save_count} image(s). New images will continue from die_{save_count:04d}.png')
    print('Type in this terminal: SPACE=save  ESC=quit')

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    tty.setcbreak(fd)

    try:
        while True:
            try:
                pRawData, FrameHead = mvsdk.CameraGetImageBuffer(hCamera, 200)
                mvsdk.CameraImageProcess(hCamera, pRawData, pFrameBuffer, FrameHead)
                mvsdk.CameraReleaseImageBuffer(hCamera, pRawData)
            except mvsdk.CameraException as e:
                if e.error_code != mvsdk.CAMERA_STATUS_TIME_OUT:
                    print(f'Grab error: {e.message}')
                continue

            frame_data = (mvsdk.c_ubyte * FrameHead.uBytes).from_address(pFrameBuffer)
            frame = np.frombuffer(frame_data, dtype=np.uint8).reshape(
                (FrameHead.iHeight, FrameHead.iWidth, channels)
            ).copy()

            display = frame.copy()
            cv2.putText(display, f'Saved: {save_count}  |  SPACE=save  ESC=quit',
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.imshow('Training Image Capture', display)
            cv2.waitKey(1)

            if not select.select([sys.stdin], [], [], 0)[0]:
                continue
            ch = sys.stdin.read(1)
            if ch == ' ':
                filename = os.path.join(SAVE_DIR, f'die_{save_count:04d}.png')
                cv2.imwrite(filename, frame)
                print(f'Saved: {filename}')
                save_count += 1
            elif ch == '\x1b':  # ESC
                break

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        cv2.destroyAllWindows()
        mvsdk.CameraUnInit(hCamera)
        mvsdk.CameraAlignFree(pFrameBuffer)
        print(f'Done. {save_count} total image(s) in {SAVE_DIR}')


if __name__ == '__main__':
    main()
