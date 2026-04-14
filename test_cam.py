#!/usr/bin/env python3
"""
test_cam.py — Mindvision camera viewer
  SPACE  : save image to disk
  ESC    : exit

For a GigE/IP camera set CAMERA_IP to the camera's IP address (e.g. '192.168.1.100').
Leave empty to connect to the first enumerated camera.
"""
import sys
import os
import numpy as np
import cv2


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

# ── Camera IP — set CAMERA_IP in config.env (see config.env.example) ─────────
CAMERA_IP = os.environ.get('CAMERA_IP', '')   # leave empty to use first enumerated camera
# ─────────────────────────────────────────────────────────────────────────────

# Point to the Mindvision Python SDK and its native library
_BASE = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'Mindvision SDK for linux V2.1.0.37'))
_LIB_PATH = os.path.join(_BASE, 'lib', 'x64')
os.environ['LD_LIBRARY_PATH'] = _LIB_PATH + ':' + os.environ.get('LD_LIBRARY_PATH', '')

# Re-exec with updated LD_LIBRARY_PATH if the library wasn't already loadable
import ctypes
try:
    ctypes.cdll.LoadLibrary('libMVSDK.so')
except OSError:
    import subprocess
    os.execve(sys.executable, [sys.executable] + sys.argv,
              {**os.environ, 'LD_LIBRARY_PATH': os.environ['LD_LIBRARY_PATH']})

SDK_PATH = os.path.join(_BASE, 'demo', 'python_demo')
sys.path.insert(0, SDK_PATH)
import mvsdk


def main():
    # ── Enumerate cameras ────────────────────────────────────────────────────
    DevList = mvsdk.CameraEnumerateDevice()
    if len(DevList) < 1:
        print('No camera found.')
        return

    DevInfo = None
    if CAMERA_IP:
        for dev in DevList:
            try:
                cam_ip, _, _, _, _, _ = mvsdk.CameraGigeGetIp(dev)
                if cam_ip == CAMERA_IP:
                    DevInfo = dev
                    print(f'Found camera at IP {CAMERA_IP}: {dev.GetFriendlyName()} ({dev.GetPortType()})')
                    break
            except Exception:
                pass
        if DevInfo is None:
            print(f'No camera found at IP {CAMERA_IP}.')
            print('Available cameras:')
            for dev in DevList:
                try:
                    cam_ip, _, _, _, _, _ = mvsdk.CameraGigeGetIp(dev)
                    print(f'  {dev.GetFriendlyName()} ({dev.GetPortType()}) — IP: {cam_ip}')
                except Exception:
                    print(f'  {dev.GetFriendlyName()} ({dev.GetPortType()})')
            return
    else:
        DevInfo = DevList[0]
        print(f'Using camera: {DevInfo.GetFriendlyName()} ({DevInfo.GetPortType()})')

    # ── Init ─────────────────────────────────────────────────────────────────
    try:
        hCamera = mvsdk.CameraInit(DevInfo, -1, -1)
    except mvsdk.CameraException as e:
        print(f'CameraInit failed ({e.error_code}): {e.message}')
        return

    cap = mvsdk.CameraGetCapability(hCamera)
    mono = cap.sIspCapacity.bMonoSensor != 0
    mvsdk.CameraSetIspOutFormat(
        hCamera,
        mvsdk.CAMERA_MEDIA_TYPE_MONO8 if mono else mvsdk.CAMERA_MEDIA_TYPE_BGR8
    )
    mvsdk.CameraSetTriggerMode(hCamera, 0)   # continuous
    mvsdk.CameraSetAeState(hCamera, 1)        # auto exposure

    channels = 1 if mono else 3
    buf_size = (cap.sResolutionRange.iWidthMax *
                cap.sResolutionRange.iHeightMax * channels)
    pFrameBuffer = mvsdk.CameraAlignMalloc(buf_size, 16)

    mvsdk.CameraPlay(hCamera)
    print('Camera running. SPACE = save image, ESC = quit.')

    save_count = 0

    try:
        while True:
            # ── Grab frame ───────────────────────────────────────────────────
            try:
                pRawData, FrameHead = mvsdk.CameraGetImageBuffer(hCamera, 200)
                mvsdk.CameraImageProcess(hCamera, pRawData, pFrameBuffer, FrameHead)
                mvsdk.CameraReleaseImageBuffer(hCamera, pRawData)
            except mvsdk.CameraException as e:
                if e.error_code != mvsdk.CAMERA_STATUS_TIME_OUT:
                    print(f'Grab error: {e.message}')
                continue

            # ── Convert to numpy / OpenCV ─────────────────────────────────
            frame_data = (mvsdk.c_ubyte * FrameHead.uBytes).from_address(pFrameBuffer)
            frame = np.frombuffer(frame_data, dtype=np.uint8).reshape(
                (FrameHead.iHeight, FrameHead.iWidth, channels)
            )

            cv2.imshow('Camera Feed', frame)

            key = cv2.waitKey(1) & 0xFF
            if key == 27:           # ESC
                break
            elif key == 32:         # SPACE
                filename = f'capture_{save_count:04d}.png'
                cv2.imwrite(filename, frame)
                print(f'Saved: {filename}')
                save_count += 1

    finally:
        cv2.destroyAllWindows()
        mvsdk.CameraUnInit(hCamera)
        mvsdk.CameraAlignFree(pFrameBuffer)
        print('Camera released.')


if __name__ == '__main__':
    main()
