"""
Standalone MindVision camera stream — no ROS required.

Shows a live 3 fps OpenCV window directly from the camera SDK.
Useful for verifying camera connectivity and framing before
the full camera_node is running.

Controls:
  SPACE  — run pip detection on current frame and overlay result
  s      — save current frame to /tmp/dice_frame_<timestamp>.png
  q/Esc  — quit

Usage:
    python3 camera_stream.py

Note: camera must be reachable first.  If you see CameraInit err:-14 run:
    sudo ip addr add 192.168.0.100/24 dev enp0s31f6
"""

import os
import sys
import time
from datetime import datetime

import cv2
import numpy as np

# ── Load .env to find MVSDK_PATH ─────────────────────────────────────────────
def _load_env():
    here = os.path.abspath(os.path.dirname(__file__))
    for _ in range(10):
        candidate = os.path.join(here, '.env')
        if os.path.exists(candidate):
            with open(candidate) as f:
                for line in f:
                    line = line.strip()
                    if not line or line.startswith('#') or '=' not in line:
                        continue
                    key, _, val = line.partition('=')
                    key, val = key.strip(), val.strip()
                    if key.endswith('_PATH') and not os.path.isabs(val):
                        val = os.path.normpath(os.path.join(here, val))
                    os.environ.setdefault(key, val)
            return
        parent = os.path.dirname(here)
        if parent == here:
            break
        here = parent

_load_env()

mvsdk_path = os.environ.get('MVSDK_PATH')
if not mvsdk_path:
    print('ERROR: MVSDK_PATH not set. Copy .env.example to .env and fill it in.')
    sys.exit(1)
sys.path.append(mvsdk_path)
import mvsdk

# ── Pip detection (same params as camera_node) ────────────────────────────────
MIN_PIP_AREA    = 50
MAX_PIP_AREA    = 2000
MIN_CIRCULARITY = 0.60
ADAPTIVE_C      = 1

def detect_pips(image: np.ndarray):
    """Returns (pip_count, annotated_image)."""
    gray    = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (7, 7), 0)
    thresh  = cv2.adaptiveThreshold(
        blurred, 255,
        cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV,
        blockSize=11, C=ADAPTIVE_C,
    )
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    annotated  = image.copy()
    pip_count  = 0
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if not (MIN_PIP_AREA < area < MAX_PIP_AREA):
            continue
        perimeter = cv2.arcLength(cnt, True)
        if perimeter == 0:
            continue
        if 4 * np.pi * area / (perimeter ** 2) < MIN_CIRCULARITY:
            continue
        pip_count += 1
        (cx, cy), radius = cv2.minEnclosingCircle(cnt)
        cv2.circle(annotated, (int(cx), int(cy)), int(radius) + 2, (0, 255, 0), 2)
    return pip_count, annotated

# ── Camera init ───────────────────────────────────────────────────────────────
def open_camera():
    dev_list = mvsdk.CameraEnumerateDevice()
    if not dev_list:
        print('ERROR: No MindVision camera found.')
        sys.exit(1)

    dev_info = dev_list[0]
    print(f'Opening camera: {dev_info.GetFriendlyName()} ({dev_info.GetPortType()})')

    hCamera = mvsdk.CameraInit(dev_info, -1, -1)
    cap_info = mvsdk.CameraGetCapability(hCamera)

    mono = (cap_info.sIspCapacity.bMonoSensor != 0)
    fmt  = mvsdk.CAMERA_MEDIA_TYPE_MONO8 if mono else mvsdk.CAMERA_MEDIA_TYPE_BGR8
    mvsdk.CameraSetIspOutFormat(hCamera, fmt)
    mvsdk.CameraSetTriggerMode(hCamera, 0)   # continuous
    mvsdk.CameraSetAeState(hCamera, 0)       # manual exposure
    mvsdk.CameraSetExposureTime(hCamera, 30 * 1000)
    mvsdk.CameraPlay(hCamera)

    buf_size = (
        cap_info.sResolutionRange.iWidthMax
        * cap_info.sResolutionRange.iHeightMax
        * (1 if mono else 3)
    )
    pFrameBuffer = mvsdk.CameraAlignMalloc(buf_size, 16)
    return hCamera, pFrameBuffer, mono

def grab_frame(hCamera, pFrameBuffer, mono):
    try:
        pRawData, FrameHead = mvsdk.CameraGetImageBuffer(hCamera, 200)
        mvsdk.CameraImageProcess(hCamera, pRawData, pFrameBuffer, FrameHead)
        mvsdk.CameraReleaseImageBuffer(hCamera, pRawData)
        channels  = 1 if mono else 3
        frame_data = (mvsdk.c_ubyte * FrameHead.uBytes).from_address(pFrameBuffer)
        frame = np.frombuffer(frame_data, dtype=np.uint8).reshape(
            (FrameHead.iHeight, FrameHead.iWidth, channels)
        )
        if mono:
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        return frame.copy()
    except mvsdk.CameraException as e:
        if e.error_code != mvsdk.CAMERA_STATUS_TIME_OUT:
            print(f'Frame grab error ({e.error_code}): {e.message}')
        return None

# ── Main loop ─────────────────────────────────────────────────────────────────
def main():
    hCamera, pFrameBuffer, mono = open_camera()
    print('Camera open.  Controls:  SPACE=count pips   s=save frame   q/Esc=quit')

    WINDOW    = 'Camera Stream  (SPACE=pips  s=save  q=quit)'
    TARGET_FPS = 3
    INTERVAL   = 1.0 / TARGET_FPS

    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW, 900, 700)

    pip_label  = ''
    pip_colour = (0, 255, 0)
    last_grab  = 0.0
    frame      = None

    try:
        while True:
            now = time.monotonic()
            if now - last_grab >= INTERVAL:
                f = grab_frame(hCamera, pFrameBuffer, mono)
                if f is not None:
                    frame = f
                last_grab = now

            display = frame.copy() if frame is not None else np.zeros((480, 640, 3), np.uint8)

            if pip_label:
                cv2.putText(display, pip_label,
                            (10, 55), cv2.FONT_HERSHEY_SIMPLEX,
                            1.8, pip_colour, 3, cv2.LINE_AA)

            # FPS indicator
            cv2.putText(display, f'{TARGET_FPS} fps',
                        (display.shape[1] - 90, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (180, 180, 180), 1)

            cv2.imshow(WINDOW, display)
            key = cv2.waitKey(1) & 0xFF

            if key == ord(' ') and frame is not None:
                count, annotated = detect_pips(frame)
                frame       = annotated
                pip_colour  = (0, 220, 0) if count > 0 else (0, 0, 220)
                pip_label   = f'Pips: {count}'
                print(f'Pip count: {count}')

            elif key == ord('s') and frame is not None:
                fname = f'/tmp/dice_frame_{datetime.now().strftime("%H%M%S")}.png'
                cv2.imwrite(fname, frame)
                print(f'Saved: {fname}')

            elif key in (ord('q'), 27):
                break

    finally:
        cv2.destroyAllWindows()
        mvsdk.CameraUnInit(hCamera)
        mvsdk.CameraAlignFree(pFrameBuffer)
        print('Camera closed.')


if __name__ == '__main__':
    main()
