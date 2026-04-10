#!/usr/bin/env python3
"""
calibrate_hough.py
------------------
Interactive HoughCircles tuner for pip detection.

Usage:
    python3 scripts/calibrate_hough.py [image_path]

If no image path is given, loads /tmp/grab.bmp (from 'just grab').

Adjust the sliders until the correct number of pips are highlighted.
Press 'q' or ESC to quit — final parameters are printed to the terminal
ready to paste into dice_roller.py.
"""
import sys
import cv2
import numpy as np

IMAGE_PATH = sys.argv[1] if len(sys.argv) > 1 else '/tmp/grab_cropped.bmp'

img = cv2.imread(IMAGE_PATH)
if img is None:
    print(f"Could not load image: {IMAGE_PATH}")
    print("Run 'just grab' first to capture a calibration image.")
    raise SystemExit(1)

gray    = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
blurred = cv2.GaussianBlur(gray, (11, 11), 2)

WIN = 'HoughCircles Calibration  (q / ESC to quit)'
cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
cv2.resizeWindow(WIN, 900, 700)

# Slider defaults match the current constants in dice_roller.py
# dp is stored as dp*10 because trackbars are integer-only
cv2.createTrackbar('dp  x10',    WIN,  12,  30,  lambda _: None)   # 1.2 default
cv2.createTrackbar('minDist',    WIN,  20,  300, lambda _: None)
cv2.createTrackbar('param1',     WIN,  50,  300, lambda _: None)
cv2.createTrackbar('param2',     WIN,  25,  100, lambda _: None)
cv2.createTrackbar('minRadius',  WIN,   5,  150, lambda _: None)
cv2.createTrackbar('maxRadius',  WIN,  40,  300, lambda _: None)

last_params = None

while True:
    dp        = max(0.1, cv2.getTrackbarPos('dp  x10',   WIN) / 10.0)
    minDist   = max(1,   cv2.getTrackbarPos('minDist',   WIN))
    param1    = max(1,   cv2.getTrackbarPos('param1',    WIN))
    param2    = max(1,   cv2.getTrackbarPos('param2',    WIN))
    minRadius =          cv2.getTrackbarPos('minRadius', WIN)
    maxRadius =          cv2.getTrackbarPos('maxRadius', WIN)

    circles = cv2.HoughCircles(
        blurred,
        cv2.HOUGH_GRADIENT,
        dp=dp,
        minDist=minDist,
        param1=param1,
        param2=param2,
        minRadius=minRadius,
        maxRadius=maxRadius,
    )

    display = img.copy()
    count = 0

    if circles is not None:
        count = len(circles[0])
        for (cx, cy, r) in np.round(circles[0]).astype(int):
            cv2.circle(display, (cx, cy), r, (0, 255, 0), 2)
            cv2.circle(display, (cx, cy), 3, (0, 0, 255), -1)

    colour = (0, 255, 0) if count > 0 else (0, 0, 255)
    cv2.putText(display, f'Pips detected: {count}', (20, 50),
                cv2.FONT_HERSHEY_SIMPLEX, 1.4, colour, 3)

    cv2.imshow(WIN, display)

    params = (dp, minDist, param1, param2, minRadius, maxRadius)
    if params != last_params:
        last_params = params
        print(f"Pips: {count:2d}  |  dp={dp:.1f}  minDist={minDist}  "
              f"param1={param1}  param2={param2}  "
              f"minRadius={minRadius}  maxRadius={maxRadius}")

    if cv2.waitKey(50) & 0xFF in (ord('q'), 27):
        break

cv2.destroyAllWindows()

# ── Auto-update dice_roller.py ──────────────────────────────────────────────────────────────────────────────
import re
from pathlib import Path

DICE_ROLLER = Path(__file__).parent.parent / 'src' / 'dice_task' / 'dice_task' / 'dice_roller.py'

replacements = {
    'HOUGH_DP':       f'{dp:.1f}',
    'HOUGH_MIN_DIST': str(minDist),
    'HOUGH_PARAM1':   str(param1),
    'HOUGH_PARAM2':   str(param2),
    'HOUGH_MIN_R':    str(minRadius),
    'HOUGH_MAX_R':    str(maxRadius),
}

text = DICE_ROLLER.read_text()
for name, value in replacements.items():
    text = re.sub(
        rf'^({re.escape(name)}\s*=\s*)[\d.]+',
        lambda m, v=value: m.group(1) + v,
        text,
        flags=re.MULTILINE,
    )
DICE_ROLLER.write_text(text)

print(f"\nUpdated {DICE_ROLLER}:")
for name, value in replacements.items():
    print(f"  {name} = {value}")
