#!/usr/bin/env python3
"""
calibrate_hsv.py
----------------
Interactive HSV tuner for isolating the yellow die face before pip counting.

Usage:
    python3 scripts/calibrate_hsv.py [image_path]

If no image path is given, loads /tmp/grab.bmp (from 'just grab').

Left panel  — original image with the detected bounding box drawn on it.
Right panel — the cropped region that will be passed to pip counting.

Adjust the sliders until the bounding box tightly frames the die face.
Press 'q' or ESC to quit — final values are written back into dice_roller.py.
"""
import sys
import cv2
import numpy as np

IMAGE_PATH = sys.argv[1] if len(sys.argv) > 1 else '/tmp/grab.bmp'

img = cv2.imread(IMAGE_PATH)
if img is None:
    print(f'Could not load image: {IMAGE_PATH}')
    print("Run 'just grab' first to capture a calibration image.")
    raise SystemExit(1)

WIN = 'HSV Crop Calibration  (q / ESC to quit)'
cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
cv2.resizeWindow(WIN, 1200, 600)

# Slider defaults match the current constants in dice_roller.py
cv2.createTrackbar('H low',  WIN,  20, 179, lambda _: None)
cv2.createTrackbar('S low',  WIN, 100, 255, lambda _: None)
cv2.createTrackbar('V low',  WIN, 100, 255, lambda _: None)
cv2.createTrackbar('H high', WIN,  35, 179, lambda _: None)
cv2.createTrackbar('S high', WIN, 255, 255, lambda _: None)
cv2.createTrackbar('V high', WIN, 255, 255, lambda _: None)

kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
PAD = 10
last_params = None

while True:
    h_low  = cv2.getTrackbarPos('H low',  WIN)
    s_low  = cv2.getTrackbarPos('S low',  WIN)
    v_low  = cv2.getTrackbarPos('V low',  WIN)
    h_high = cv2.getTrackbarPos('H high', WIN)
    s_high = cv2.getTrackbarPos('S high', WIN)
    v_high = cv2.getTrackbarPos('V high', WIN)

    hsv  = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv,
                       np.array([h_low,  s_low,  v_low]),
                       np.array([h_high, s_high, v_high]))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    annotated = img.copy()
    crop_display = np.zeros((img.shape[0], img.shape[1] // 2, 3), dtype=np.uint8)

    if contours:
        x, y, w, h = cv2.boundingRect(max(contours, key=cv2.contourArea))
        x1 = max(0, x - PAD)
        y1 = max(0, y - PAD)
        x2 = min(img.shape[1], x + w + PAD)
        y2 = min(img.shape[0], y + h + PAD)

        cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 3)
        cv2.putText(annotated, 'Die region', (x1, max(y1 - 10, 20)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        crop = img[y1:y2, x1:x2]
        # Scale crop to fill the right panel
        scale = min(crop_display.shape[0] / max(crop.shape[0], 1),
                    crop_display.shape[1] / max(crop.shape[1], 1))
        cw = int(crop.shape[1] * scale)
        ch = int(crop.shape[0] * scale)
        resized = cv2.resize(crop, (cw, ch))
        oy = (crop_display.shape[0] - ch) // 2
        ox = (crop_display.shape[1] - cw) // 2
        crop_display[oy:oy+ch, ox:ox+cw] = resized
    else:
        cv2.putText(annotated, 'No yellow region found', (20, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 3)

    # Stack: annotated (left) | crop preview (right)
    left  = cv2.resize(annotated,     (img.shape[1] // 2, img.shape[0]))
    frame = np.hstack([left, crop_display])
    cv2.imshow(WIN, frame)

    params = (h_low, s_low, v_low, h_high, s_high, v_high)
    if params != last_params:
        last_params = params
        found = 'yes' if contours else 'NO'
        print(f'Found: {found}  |  H [{h_low}-{h_high}]  S [{s_low}-{s_high}]  V [{v_low}-{v_high}]')

    if cv2.waitKey(50) & 0xFF in (ord('q'), 27):
        break

cv2.destroyAllWindows()

# ── Save cropped image for calibrate_hough.py ───────────────────────────────────────────────────────────────────────────────
if contours:
    x, y, w, h = cv2.boundingRect(max(contours, key=cv2.contourArea))
    x1 = max(0, x - PAD);  y1 = max(0, y - PAD)
    x2 = min(img.shape[1], x + w + PAD);  y2 = min(img.shape[0], y + h + PAD)
    crop = img[y1:y2, x1:x2]
    cv2.imwrite('/tmp/grab_cropped.bmp', crop)
    print('\nSaved cropped image to /tmp/grab_cropped.bmp')
else:
    print('\nWARN: no yellow region found — /tmp/grab_cropped.bmp not saved.')

# ── Auto-update dice_roller.py ──────────────────────────────────────────────────────────────────────────────
import re
from pathlib import Path

DICE_ROLLER = Path(__file__).parent.parent / 'src' / 'dice_task' / 'dice_task' / 'dice_roller.py'

replacements = {
    'HSV_H_LOW':  str(h_low),
    'HSV_S_LOW':  str(s_low),
    'HSV_V_LOW':  str(v_low),
    'HSV_H_HIGH': str(h_high),
    'HSV_S_HIGH': str(s_high),
    'HSV_V_HIGH': str(v_high),
}

text = DICE_ROLLER.read_text()
for name, value in replacements.items():
    text = re.sub(
        rf'^({re.escape(name)}\s*=\s*)\d+',
        lambda m, v=value: m.group(1) + v,
        text,
        flags=re.MULTILINE,
    )
DICE_ROLLER.write_text(text)

print(f'\nUpdated {DICE_ROLLER}:')
for name, value in replacements.items():
    print(f'  {name} = {value}')
