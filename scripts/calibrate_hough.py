#!/usr/bin/env python3
"""
calibrate_hough.py
------------------
Interactive HoughCircles tuner for pip detection across multiple dice.

Usage:
    python3 scripts/calibrate_hough.py

Loads the pre-cropped die images saved by 'just calibrate-hsv'
(/tmp/grab_cropped_0.bmp, _1.bmp, ...) so Hough runs on exactly the same
crops you tuned in the HSV tool.

Press 'q' or ESC to quit — final parameters are written back into robot1.py.
"""
import re
import glob
import cv2
import numpy as np
from pathlib import Path

SRC_DIR = Path(__file__).parent.parent / 'src' / 'dual_fanuc' / 'dual_fanuc'
ROBOT1  = SRC_DIR / 'robot1.py'
ROBOT2  = SRC_DIR / 'robot2.py'

# ── Load pre-cropped images from calibrate_hsv ────────────────────────────────
crop_paths = sorted(glob.glob('/tmp/grab_cropped_*.bmp'))
if not crop_paths:
    # Fall back to legacy single-crop name
    if Path('/tmp/grab_cropped.bmp').exists():
        crop_paths = ['/tmp/grab_cropped.bmp']
    else:
        print("No cropped images found. Run 'just calibrate-hsv' first.")
        raise SystemExit(1)

crops = []
for p in crop_paths:
    img = cv2.imread(p)
    if img is not None:
        crops.append(img)
        print(f"Loaded: {p}  ({img.shape[1]}×{img.shape[0]})")

if not crops:
    print("Could not read any crop files.")
    raise SystemExit(1)

print(f"{len(crops)} die crop(s) loaded.")

# ── Read current Hough defaults from robot1.py ────────────────────────────────
def _read_int(text, name, default):
    m = re.search(rf'^{re.escape(name)}\s*=\s*(\d+)', text, re.MULTILINE)
    return int(m.group(1)) if m else default

def _read_float(text, name, default):
    m = re.search(rf'^{re.escape(name)}\s*=\s*([\d.]+)', text, re.MULTILINE)
    return float(m.group(1)) if m else default

robot1_text   = ROBOT1.read_text()
dp_init       = int(_read_float(robot1_text, 'HOUGH_DP',       1.2) * 10)
minDist_init  = _read_int(robot1_text, 'HOUGH_MIN_DIST', 20)
param1_init   = _read_int(robot1_text, 'HOUGH_PARAM1',   50)
param2_init   = _read_int(robot1_text, 'HOUGH_PARAM2',   25)
minR_init     = _read_int(robot1_text, 'HOUGH_MIN_R',    0)
maxR_init     = _read_int(robot1_text, 'HOUGH_MAX_R',    20)

# Pre-blur each crop once (blur doesn't depend on Hough params)
blurred_crops = []
for crop in crops:
    gray    = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (11, 11), 2)
    blurred_crops.append(blurred)

WIN = 'HoughCircles Calibration  (q / ESC to quit)'
cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
cv2.resizeWindow(WIN, 1200, 700)

cv2.createTrackbar('dp  x10',   WIN, dp_init,      30,  lambda _: None)
cv2.createTrackbar('minDist',   WIN, minDist_init,  300, lambda _: None)
cv2.createTrackbar('param1',    WIN, param1_init,   300, lambda _: None)
cv2.createTrackbar('param2',    WIN, param2_init,   100, lambda _: None)
cv2.createTrackbar('minRadius', WIN, minR_init,     150, lambda _: None)
cv2.createTrackbar('maxRadius', WIN, maxR_init,     300, lambda _: None)

PANEL_H   = 700
SUMMARY_W = 400
last_params = None


def make_mosaic(annotated_crops, panel_h):
    if not annotated_crops:
        return np.zeros((panel_h, 400, 3), dtype=np.uint8)
    n    = len(annotated_crops)
    cols = int(np.ceil(np.sqrt(n)))
    rows = int(np.ceil(n / cols))
    max_h = max(c.shape[0] for c in annotated_crops)
    max_w = max(c.shape[1] for c in annotated_crops)
    scale = min((panel_h // rows) / max(max_h, 1), 600 / max(max_w, 1))
    cw    = max(1, int(max_w * scale))
    ch    = max(1, int(max_h * scale))
    canvas = np.zeros((rows * ch, cols * cw, 3), dtype=np.uint8)
    for i, crop in enumerate(annotated_crops):
        r, c = divmod(i, cols)
        resized = cv2.resize(crop, (cw, ch))
        canvas[r*ch:(r+1)*ch, c*cw:(c+1)*cw] = resized
    return canvas


while True:
    dp        = max(0.1, cv2.getTrackbarPos('dp  x10',   WIN) / 10.0)
    minDist   = max(1,   cv2.getTrackbarPos('minDist',   WIN))
    param1    = max(1,   cv2.getTrackbarPos('param1',    WIN))
    param2    = max(1,   cv2.getTrackbarPos('param2',    WIN))
    minRadius =          cv2.getTrackbarPos('minRadius', WIN)
    maxRadius =          cv2.getTrackbarPos('maxRadius', WIN)

    total_pips      = 0
    annotated_crops = []
    counts          = []

    for crop, blurred in zip(crops, blurred_crops):
        circles = cv2.HoughCircles(blurred, cv2.HOUGH_GRADIENT,
                                   dp=dp, minDist=minDist,
                                   param1=param1, param2=param2,
                                   minRadius=minRadius, maxRadius=maxRadius)
        display = crop.copy()
        count = 0
        if circles is not None:
            count = len(circles[0])
            for (cx, cy, r) in np.round(circles[0]).astype(int):
                cv2.circle(display, (cx, cy), r, (0, 255, 0), 2)
                cv2.circle(display, (cx, cy), 3, (0, 0, 255), -1)
        colour = (0, 255, 0) if count > 0 else (0, 0, 255)
        cv2.putText(display, f'Pips: {count}', (5, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, colour, 2)
        annotated_crops.append(display)
        counts.append(count)
        total_pips += count

    mosaic = make_mosaic(annotated_crops, PANEL_H)
    scale_m = PANEL_H / max(mosaic.shape[0], 1)
    mosaic  = cv2.resize(mosaic, (int(mosaic.shape[1] * scale_m), PANEL_H))

    summary = np.zeros((PANEL_H, SUMMARY_W, 3), dtype=np.uint8)
    cv2.putText(summary, f'Dice: {len(crops)}', (20, 50),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
    cv2.putText(summary, f'Total pips: {total_pips}', (20, 100),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
    for i, cnt in enumerate(counts):
        col = (0, 255, 0) if cnt > 0 else (0, 0, 255)
        cv2.putText(summary, f'  Die #{i+1}: {cnt}', (20, 160 + i * 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, col, 2)
    cv2.putText(summary, 'q/ESC to save+quit', (20, PANEL_H - 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (180, 180, 180), 1)

    frame = np.hstack([summary, mosaic])
    cv2.imshow(WIN, frame)

    params = (dp, minDist, param1, param2, minRadius, maxRadius)
    if params != last_params:
        last_params = params
        counts_str = '  '.join(f'#{i+1}:{c}' for i, c in enumerate(counts))
        print(f"Total: {total_pips}  [{counts_str}]  |  "
              f"dp={dp:.1f}  minDist={minDist}  param1={param1}  "
              f"param2={param2}  minR={minRadius}  maxR={maxRadius}")

    if cv2.waitKey(50) & 0xFF in (ord('q'), 27):
        break

cv2.destroyAllWindows()

# ── Auto-update robot1.py ─────────────────────────────────────────────────────
replacements = {
    'HOUGH_DP':       f'{dp:.1f}',
    'HOUGH_MIN_DIST': str(minDist),
    'HOUGH_PARAM1':   str(param1),
    'HOUGH_PARAM2':   str(param2),
    'HOUGH_MIN_R':    str(minRadius),
    'HOUGH_MAX_R':    str(maxRadius),
}

for path in (ROBOT1, ROBOT2):
    text = path.read_text()
    for name, value in replacements.items():
        text = re.sub(
            rf'^({re.escape(name)}\s*=\s*)[\d.]+',
            lambda m, v=value: m.group(1) + v,
            text,
            flags=re.MULTILINE,
        )
    path.write_text(text)
    print(f"\nUpdated {path}:")
    for name, value in replacements.items():
        print(f"  {name} = {value}")
