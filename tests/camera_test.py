#!/usr/bin/env python3
"""
Standalone camera test -- no ROS required.

Enumerates MindVision cameras, grabs one frame, runs pip detection,
and saves raw + annotated images to /tmp/dice_images/.

Usage:
    python3 tests/camera_test.py
"""

import os
import sys

# ── Load .env (same logic as camera_node.py) ─────────────────────────────────
def _load_env():
    here = os.path.abspath(os.path.dirname(__file__))
    for _ in range(10):
        candidate = os.path.join(here, '.env')
        if os.path.exists(candidate):
            env_dir = here
            with open(candidate) as f:
                for line in f:
                    line = line.strip()
                    if not line or line.startswith('#') or '=' not in line:
                        continue
                    key, _, val = line.partition('=')
                    key, val = key.strip(), val.strip()
                    if key.endswith('_PATH') and not os.path.isabs(val):
                        val = os.path.normpath(os.path.join(env_dir, val))
                    os.environ.setdefault(key, val)
            return
        parent = os.path.dirname(here)
        if parent == here:
            break
        here = parent

_load_env()

mvsdk_path = os.environ.get('MVSDK_PATH')
if not mvsdk_path:
    sys.exit('ERROR: MVSDK_PATH not set. Copy .env.example to .env and fill it in.')

print(f'Using MVSDK_PATH: {mvsdk_path}')
sys.path.append(mvsdk_path)

import mvsdk  # noqa: E402
import cv2
import numpy as np

OUT_DIR = '/tmp/dice_images'
os.makedirs(OUT_DIR, exist_ok=True)

# ── Enumerate cameras ─────────────────────────────────────────────────────────
dev_list = mvsdk.CameraEnumerateDevice()
print(f'\nFound {len(dev_list)} camera(s):')
for i, d in enumerate(dev_list):
    print(f'  [{i}] {d.GetFriendlyName()}  SN:{d.GetSn()}  Port:{d.GetPortType()}')

if not dev_list:
    sys.exit('No cameras found. Is the camera plugged in?')

# ── Init camera ───────────────────────────────────────────────────────────────
print('\nInitialising camera [0]...')
hCamera = mvsdk.CameraInit(dev_list[0], -1, -1)
print('  CameraInit OK')

cap_info = mvsdk.CameraGetCapability(hCamera)
mono = cap_info.sIspCapacity.bMonoSensor != 0
fmt = mvsdk.CAMERA_MEDIA_TYPE_MONO8 if mono else mvsdk.CAMERA_MEDIA_TYPE_BGR8
mvsdk.CameraSetIspOutFormat(hCamera, fmt)
mvsdk.CameraSetTriggerMode(hCamera, 0)   # continuous
mvsdk.CameraSetAeState(hCamera, 0)       # manual exposure
mvsdk.CameraSetExposureTime(hCamera, 30 * 1000)
mvsdk.CameraPlay(hCamera)
print(f'  Sensor: {"mono" if mono else "color"}  '
      f'Max res: {cap_info.sResolutionRange.iWidthMax}x{cap_info.sResolutionRange.iHeightMax}')

# ── Grab one frame ────────────────────────────────────────────────────────────
buf_size = (cap_info.sResolutionRange.iWidthMax
            * cap_info.sResolutionRange.iHeightMax
            * (1 if mono else 3))
pFrameBuffer = mvsdk.CameraAlignMalloc(buf_size, 16)

print('\nGrabbing frame (2s timeout)...')
pRawData, FrameHead = mvsdk.CameraGetImageBuffer(hCamera, 2000)
mvsdk.CameraImageProcess(hCamera, pRawData, pFrameBuffer, FrameHead)
mvsdk.CameraReleaseImageBuffer(hCamera, pRawData)

channels = 1 if mono else 3
frame_data = (mvsdk.c_ubyte * FrameHead.uBytes).from_address(pFrameBuffer)
frame = np.frombuffer(frame_data, dtype=np.uint8).reshape(
    (FrameHead.iHeight, FrameHead.iWidth, channels)
).copy()
print(f'  Frame size: {FrameHead.iWidth}x{FrameHead.iHeight}')

bgr = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR) if mono else frame

raw_path = os.path.join(OUT_DIR, 'test_raw.jpg')
cv2.imwrite(raw_path, bgr)
print(f'  Raw image saved -> {raw_path}')

# ── Pip detection ─────────────────────────────────────────────────────────────
MIN_PIP_AREA    = 50
MAX_PIP_AREA    = 2000
MIN_CIRCULARITY = 0.60

gray    = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
blurred = cv2.GaussianBlur(gray, (7, 7), 0)
thresh  = cv2.adaptiveThreshold(
    blurred, 255,
    cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV,
    blockSize=11, C=2,
)
contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

annotated  = bgr.copy()
pip_count  = 0
for cnt in contours:
    area = cv2.contourArea(cnt)
    if not (MIN_PIP_AREA < area < MAX_PIP_AREA):
        continue
    perimeter = cv2.arcLength(cnt, True)
    if perimeter == 0:
        continue
    circularity = 4 * np.pi * area / (perimeter ** 2)
    if circularity < MIN_CIRCULARITY:
        continue
    pip_count += 1
    (cx, cy), radius = cv2.minEnclosingCircle(cnt)
    cv2.circle(annotated, (int(cx), int(cy)), int(radius) + 2, (0, 255, 0), 2)

cv2.putText(annotated, f'Pips: {pip_count}',
            (10, 35), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)

ann_path = os.path.join(OUT_DIR, 'test_annotated.jpg')
cv2.imwrite(ann_path, annotated)
print(f'  Annotated image saved -> {ann_path}')

print(f'\nResult: detected {pip_count} pip(s)')

# ── Cleanup ───────────────────────────────────────────────────────────────────
mvsdk.CameraUnInit(hCamera)
mvsdk.CameraAlignFree(pFrameBuffer)
print('Camera released. Done.')
