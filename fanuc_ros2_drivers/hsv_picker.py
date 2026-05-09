#!/usr/bin/env python3
"""
hsv_picker.py
=============
Camera HSV calibration tool.

Shows a live camera feed. Left-click anywhere on the dice face; the tool
samples a 15×15-pixel patch around each click, accumulates all samples, and
continuously prints a suggested LOWER / UPPER HSV pair ready to paste into
pip_test.py or AllClaudeLookatDice.py.

The green overlay on the live feed shows which pixels the current suggested
range would match — aim for the dice face to be fully covered and the
background to stay dark.

Controls
--------
  Left-click  — sample the HSV patch under the cursor
  r           — reset all samples and start fresh
  q / Esc     — quit and print the final suggested range
"""

import os
import sys
import time
from ctypes import addressof, c_ubyte

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mvsdk

# ---------------------------------------------------------------------------
CAMERA_INDEX  = 0
AE_TARGET     = 80     # AE brightness target (0–255); default ~120 overexposes yellow
PATCH_RADIUS  = 7      # sample a (2r+1)×(2r+1) = 15×15 patch per click
MARGIN_H      = 8      # ± hue margin added around the observed min/max
MARGIN_SV     = 30     # ± saturation/value margin
# ---------------------------------------------------------------------------


def open_camera(index: int):
    dev_list = mvsdk.CameraEnumerateDevice()
    if len(dev_list) <= index:
        raise RuntimeError(f'No MindVision camera at index {index} '
                           f'({len(dev_list)} device(s) found).')
    h = mvsdk.CameraInit(dev_list[index])
    max_w, max_h, b_color = mvsdk.CameraGetCapabilityEx2(h)
    is_color = b_color != 0
    if is_color:
        mvsdk.CameraSetIspOutFormat(h, mvsdk.CAMERA_MEDIA_TYPE_BGR8)
        ch = 3
    else:
        mvsdk.CameraSetIspOutFormat(h, mvsdk.CAMERA_MEDIA_TYPE_MONO8)
        ch = 1
    buf = (c_ubyte * (max_w * max_h * ch))()
    mvsdk.CameraSetAeState(h, True)
    mvsdk.CameraSetAeTarget(h, AE_TARGET)
    mvsdk.CameraSetTriggerMode(h, 1)
    mvsdk.CameraPlay(h)
    print(f'Camera opened: {"colour" if is_color else "mono"}, max {max_w}×{max_h}')

    print('Warming up AE in trigger mode...')
    for _ in range(50):
        try:
            mvsdk.CameraSoftTrigger(h)
            raw, head = mvsdk.CameraGetImageBuffer(h, 1000)
            mvsdk.CameraReleaseImageBuffer(h, raw)
        except mvsdk.CameraException:
            pass
        time.sleep(0.15)

    exposure_us = mvsdk.CameraGetExposureTime(h)
    mvsdk.CameraSetAeState(h, False)
    mvsdk.CameraSetExposureTime(h, exposure_us)
    print(f'AE locked — exposure={exposure_us:.0f} µs')

    return h, buf, is_color, ch


def grab_frame(h, buf, is_color, ch) -> np.ndarray:
    mvsdk.CameraSoftTrigger(h)
    time.sleep(0.5)
    raw, head = mvsdk.CameraGetImageBuffer(h, 2000)
    mvsdk.CameraImageProcess(h, raw, addressof(buf), head)
    mvsdk.CameraReleaseImageBuffer(h, raw)
    n = head.iWidth * head.iHeight * ch
    view = (c_ubyte * n).from_address(addressof(buf))
    img = np.frombuffer(view, dtype=np.uint8).reshape(
        (head.iHeight, head.iWidth, ch)).copy()
    if not is_color:
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    return img


def sample_patch(hsv_img: np.ndarray, cx: int, cy: int, radius: int):
    """Return flat list of [H, S, V] values in a square patch around (cx, cy)."""
    h_img, w_img = hsv_img.shape[:2]
    x0, x1 = max(cx - radius, 0), min(cx + radius + 1, w_img)
    y0, y1 = max(cy - radius, 0), min(cy + radius + 1, h_img)
    return hsv_img[y0:y1, x0:x1].reshape(-1, 3).tolist()


def compute_range(pixels):
    """Compute LOWER / UPPER bounds with margin from all accumulated samples."""
    arr = np.array(pixels, dtype=np.int32)
    lo = arr.min(axis=0) - [MARGIN_H, MARGIN_SV, MARGIN_SV]
    hi = arr.max(axis=0) + [MARGIN_H, MARGIN_SV, MARGIN_SV]
    lo = np.clip(lo, 0, [179, 255, 255]).tolist()
    hi = np.clip(hi, 0, [179, 255, 255]).tolist()
    return lo, hi


def print_range(lo, hi, n_samples: int):
    print(f'  Suggested range ({n_samples} sample pixels):')
    print(f'    LOWER_YELLOW = np.array({lo})')
    print(f'    UPPER_YELLOW = np.array({hi})')
    print()


def main():
    h_cam, buf, is_color, ch = open_camera(CAMERA_INDEX)

    WIN = 'HSV Picker  |  left-click=sample  r=reset  q=quit'
    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WIN, 1280, 960)

    # Shared mutable state (updated from both main loop and mouse callback)
    state = {
        'hsv':      None,      # current HSV frame
        'pixels':   [],        # all sampled [H,S,V] values
        'clicks':   [],        # (x, y) click positions for dot overlay
        'lower':    None,
        'upper':    None,
    }

    def on_mouse(event, x, y, flags, _param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return

        patch = sample_patch(state['hsv'], x, y, PATCH_RADIUS)
        state['pixels'].extend(patch)
        state['clicks'].append((x, y))

        lo, hi = compute_range(state['pixels'])
        state['lower'] = lo
        state['upper'] = hi

        mean_hsv = np.mean(patch, axis=0).astype(int)
        print(f'  Click ({x}, {y})  —  patch mean: H={mean_hsv[0]}  S={mean_hsv[1]}  V={mean_hsv[2]}')
        print_range(lo, hi, len(state['pixels']))

        cv2.imshow(WIN, render())

    cv2.setMouseCallback(WIN, on_mouse)

    # Snap one frame then release the camera immediately
    print('Capturing image...')
    try:
        frame = grab_frame(h_cam, buf, is_color, ch)
    finally:
        mvsdk.CameraStop(h_cam)
        mvsdk.CameraUnInit(h_cam)
    print('Camera closed.')

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    state['hsv'] = hsv

    print()
    print('Click on the dice face in the window.')
    print('Click multiple spots (centre, edges, pips if you want them included)')
    print('to build up a robust range.')
    print('Controls: r = reset samples   q / Esc = quit')
    print()

    def render():
        """Rebuild the display from the fixed frame + current sample state."""
        display = frame.copy()

        if state['lower'] is not None:
            mask = cv2.inRange(hsv,
                               np.array(state['lower']),
                               np.array(state['upper']))
            # Grey-out unmatched pixels; matched pixels keep their original colour
            gray_bg   = cv2.cvtColor(
                cv2.cvtColor(display, cv2.COLOR_BGR2GRAY), cv2.COLOR_GRAY2BGR
            )
            composite = gray_bg.copy()
            composite[mask > 0] = display[mask > 0]
            cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(composite, cnts, -1, (0, 220, 0), 2)
            display = composite

            lo_txt = str(state['lower'])
            hi_txt = str(state['upper'])
            for txt, y_pos in [(f'LOWER: {lo_txt}', 45), (f'UPPER: {hi_txt}', 90)]:
                cv2.putText(display, txt, (10, y_pos),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 0), 4)
                cv2.putText(display, txt, (10, y_pos),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2)

            n_clicks = len(state['clicks'])
            footer = f'{n_clicks} click(s)  |  r=reset  q=quit'
            cv2.putText(display, footer, (10, display.shape[0] - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 4)
            cv2.putText(display, footer, (10, display.shape[0] - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

        for cx, cy in state['clicks']:
            cv2.circle(display, (cx, cy), PATCH_RADIUS, (0, 0, 255), 2)
            cv2.circle(display, (cx, cy), 2, (0, 0, 255), -1)

        return display

    cv2.imshow(WIN, render())

    while True:
        key = cv2.waitKey(50) & 0xFF   # 50 ms — just enough to stay responsive

        if key in (ord('q'), 27):
            break
        elif key == ord('r'):
            state['pixels'].clear()
            state['clicks'].clear()
            state['lower'] = None
            state['upper'] = None
            print('--- Samples reset ---')
            print()
            cv2.imshow(WIN, render())

    cv2.destroyAllWindows()

    if state['pixels']:
        lo, hi = compute_range(state['pixels'])
        print('=== Final suggested HSV range ===')
        print(f'LOWER_YELLOW = np.array({lo})')
        print(f'UPPER_YELLOW = np.array({hi})')
    else:
        print('No samples collected.')


if __name__ == '__main__':
    main()
