#!/usr/bin/env python3
"""
calibrate_hsv.py
----------------
Interactive ROI + HSV tuner for isolating yellow die faces before pip counting.

Usage:
    python3 scripts/calibrate_hsv.py [image_path]

If no image path is given, loads /tmp/grab.bmp (from 'just grab').

Sliders:
  ROI X1 / Y1 / X2 / Y2 — outer crop applied before HSV detection
  H/S/V low+high        — yellow-mask bounds
  min area /100         — discard contours below this many pixels²

Left panel  — original image with the ROI rectangle and per-die boxes.
Right panel — mosaic of the cropped die regions (these are saved to
              /tmp/grab_cropped_*.bmp for calibrate_hough).

Press 'q' or ESC to quit — final values are written back into BOTH
robot1.py and robot2.py.
"""
import sys
import re
import cv2
import numpy as np
from pathlib import Path

IMAGE_PATH = sys.argv[1] if len(sys.argv) > 1 else '/tmp/grab.bmp'
SRC_DIR    = Path(__file__).parent.parent / 'src' / 'dual_fanuc' / 'dual_fanuc'
ROBOT1     = SRC_DIR / 'robot1.py'
ROBOT2     = SRC_DIR / 'robot2.py'

img = cv2.imread(IMAGE_PATH)
if img is None:
    print(f'Could not load image: {IMAGE_PATH}')
    print("Run 'just grab' first to capture a calibration image.")
    raise SystemExit(1)

H, W = img.shape[:2]


# ── Read current values from robot1.py so sliders persist between runs ────────
def _read_int(text, name, default):
    m = re.search(rf'^{re.escape(name)}\s*=\s*(\d+)', text, re.MULTILINE)
    return int(m.group(1)) if m else default


robot1_text     = ROBOT1.read_text()
H_LOW_INIT      = _read_int(robot1_text, 'HSV_H_LOW',  20)
S_LOW_INIT      = _read_int(robot1_text, 'HSV_S_LOW',  100)
V_LOW_INIT      = _read_int(robot1_text, 'HSV_V_LOW',  100)
H_HIGH_INIT     = _read_int(robot1_text, 'HSV_H_HIGH', 35)
S_HIGH_INIT     = _read_int(robot1_text, 'HSV_S_HIGH', 255)
V_HIGH_INIT     = _read_int(robot1_text, 'HSV_V_HIGH', 255)
MIN_AREA_INIT   = _read_int(robot1_text, 'MIN_CONTOUR_AREA', 2000) // 100
ROI_X1_INIT     = _read_int(robot1_text, 'ROI_X1', 0)
ROI_Y1_INIT     = _read_int(robot1_text, 'ROI_Y1', 0)
ROI_X2_INIT     = _read_int(robot1_text, 'ROI_X2', W)
ROI_Y2_INIT     = _read_int(robot1_text, 'ROI_Y2', H)

# Clamp ROI defaults to image bounds in case the constants outpace this image.
ROI_X1_INIT = min(max(ROI_X1_INIT, 0), W - 1)
ROI_Y1_INIT = min(max(ROI_Y1_INIT, 0), H - 1)
ROI_X2_INIT = min(max(ROI_X2_INIT, 1), W)
ROI_Y2_INIT = min(max(ROI_Y2_INIT, 1), H)

WIN = 'HSV + ROI Crop Calibration  (q / ESC to quit)'
cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
cv2.resizeWindow(WIN, 1600, 900)

# Short trackbar names ↔ readable in-frame labels (defined below).
_TRACKBARS = [
    ('roi_x1', 'ROI x1',  ROI_X1_INIT,   max(W - 1, 1)),
    ('roi_y1', 'ROI y1',  ROI_Y1_INIT,   max(H - 1, 1)),
    ('roi_x2', 'ROI x2',  ROI_X2_INIT,   W),
    ('roi_y2', 'ROI y2',  ROI_Y2_INIT,   H),
    ('h_lo',   'H low',   H_LOW_INIT,    179),
    ('s_lo',   'S low',   S_LOW_INIT,    255),
    ('v_lo',   'V low',   V_LOW_INIT,    255),
    ('h_hi',   'H high',  H_HIGH_INIT,   179),
    ('s_hi',   'S high',  S_HIGH_INIT,   255),
    ('v_hi',   'V high',  V_HIGH_INIT,   255),
    ('area',   'min area /100', MIN_AREA_INIT, 500),
]
for short, _label, init, vmax in _TRACKBARS:
    cv2.createTrackbar(short, WIN, init, vmax, lambda _: None)


def _draw_readout(canvas, values, count, ok):
    """Render a sidebar showing all parameter values with full labels."""
    panel_w  = 240
    h        = canvas.shape[0]
    sidebar  = np.full((h, panel_w, 3), 30, dtype=np.uint8)
    line_h   = 28
    y        = 30

    sections = [
        ('ROI',       [(0, 'roi_x1'), (1, 'roi_y1'), (2, 'roi_x2'), (3, 'roi_y2')], (90, 200, 255)),
        ('HSV low',   [(4, 'h_lo'),   (5, 's_lo'),   (6, 'v_lo')],                  (255, 200, 90)),
        ('HSV high',  [(7, 'h_hi'),   (8, 's_hi'),   (9, 'v_hi')],                  (160, 255, 160)),
        ('Filter',    [(10, 'area')],                                              (220, 220, 220)),
    ]

    label_lookup = {short: full for short, full, _, _ in _TRACKBARS}
    for title, rows, colour in sections:
        cv2.putText(sidebar, title, (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, colour, 1, cv2.LINE_AA)
        y += line_h
        for idx, short in rows:
            label = label_lookup[short]
            val   = values[idx]
            cv2.putText(sidebar, f'  {label:<14} {val}',
                        (12, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (240, 240, 240), 1, cv2.LINE_AA)
            y += line_h
        y += 8

    # Found-die count.
    y += 10
    col = (90, 255, 90) if ok else (90, 90, 255)
    cv2.putText(sidebar, f'{count} die(s) found', (12, y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, col, 2, cv2.LINE_AA)
    cv2.putText(sidebar, "q / ESC to save+quit", (12, h - 16),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (160, 160, 160), 1, cv2.LINE_AA)
    return sidebar


kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
PAD = 10
last_params = None

PANEL_W = img.shape[1] // 2
PANEL_H = img.shape[0]


def make_mosaic(crops, panel_w, panel_h):
    """Tile crops into a panel. Returns a blank panel if no crops."""
    if not crops:
        blank = np.zeros((panel_h, panel_w, 3), dtype=np.uint8)
        cv2.putText(blank, 'No dice found', (20, panel_h // 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
        return blank

    n = len(crops)
    cols = int(np.ceil(np.sqrt(n)))
    rows = int(np.ceil(n / cols))
    cell_w = panel_w // cols
    cell_h = panel_h // rows

    canvas = np.zeros((panel_h, panel_w, 3), dtype=np.uint8)
    for i, crop in enumerate(crops):
        r, c = divmod(i, cols)
        scale = min(cell_w / max(crop.shape[1], 1), cell_h / max(crop.shape[0], 1))
        cw = int(crop.shape[1] * scale)
        ch = int(crop.shape[0] * scale)
        resized = cv2.resize(crop, (cw, ch))
        ox = c * cell_w + (cell_w - cw) // 2
        oy = r * cell_h + (cell_h - ch) // 2
        canvas[oy:oy+ch, ox:ox+cw] = resized
        cv2.putText(canvas, f'#{i+1}', (c * cell_w + 5, r * cell_h + 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
    return canvas


while True:
    roi_x1   = cv2.getTrackbarPos('roi_x1', WIN)
    roi_y1   = cv2.getTrackbarPos('roi_y1', WIN)
    roi_x2   = cv2.getTrackbarPos('roi_x2', WIN)
    roi_y2   = cv2.getTrackbarPos('roi_y2', WIN)
    h_low    = cv2.getTrackbarPos('h_lo',   WIN)
    s_low    = cv2.getTrackbarPos('s_lo',   WIN)
    v_low    = cv2.getTrackbarPos('v_lo',   WIN)
    h_high   = cv2.getTrackbarPos('h_hi',   WIN)
    s_high   = cv2.getTrackbarPos('s_hi',   WIN)
    v_high   = cv2.getTrackbarPos('v_hi',   WIN)
    min_area = cv2.getTrackbarPos('area',   WIN) * 100

    # Sanitise ROI so x2>x1 and y2>y1.
    if roi_x2 <= roi_x1: roi_x2 = roi_x1 + 1
    if roi_y2 <= roi_y1: roi_y2 = roi_y1 + 1

    roi = img[roi_y1:roi_y2, roi_x1:roi_x2]
    hsv  = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv,
                       np.array([h_low,  s_low,  v_low]),
                       np.array([h_high, s_high, v_high]))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)

    all_contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = [c for c in all_contours if cv2.contourArea(c) >= min_area]

    annotated = img.copy()
    cv2.rectangle(annotated, (roi_x1, roi_y1), (roi_x2, roi_y2), (0, 255, 255), 2)
    crops = []

    for i, c in enumerate(contours):
        # Bounding rect is in ROI coords; shift back to full-image coords for the overlay.
        x, y, w, h = cv2.boundingRect(c)
        x1 = max(0, x - PAD);                  y1 = max(0, y - PAD)
        x2 = min(roi.shape[1], x + w + PAD);   y2 = min(roi.shape[0], y + h + PAD)
        gx1, gy1 = roi_x1 + x1, roi_y1 + y1
        gx2, gy2 = roi_x1 + x2, roi_y1 + y2

        cv2.rectangle(annotated, (gx1, gy1), (gx2, gy2), (0, 255, 0), 3)
        cv2.putText(annotated, f'#{i+1}', (gx1, max(gy1 - 8, 18)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        # Per-die fill mask so the saved crop is the die only — everything outside
        # this contour goes to black. Pips remain visible (they're inside the
        # contour's bounding box but originally black, so bitwise_and keeps them).
        die_mask = np.zeros(roi.shape[:2], dtype=np.uint8)
        cv2.drawContours(die_mask, [c], -1, 255, thickness=cv2.FILLED)
        masked = cv2.bitwise_and(roi, roi, mask=die_mask)
        crops.append(masked[y1:y2, x1:x2])

    count_str = f'{len(contours)} die(s) found'
    colour = (0, 255, 0) if contours else (0, 0, 255)
    cv2.putText(annotated, count_str, (20, img.shape[0] - 20),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, colour, 2)

    left    = cv2.resize(annotated, (PANEL_W, PANEL_H))
    right   = make_mosaic(crops, PANEL_W, PANEL_H)
    sidebar = _draw_readout(
        np.zeros((PANEL_H, 1, 3), dtype=np.uint8),  # height ref only
        [roi_x1, roi_y1, roi_x2, roi_y2,
         h_low, s_low, v_low, h_high, s_high, v_high, min_area // 100],
        len(contours),
        bool(contours),
    )
    frame = np.hstack([sidebar, left, right])
    cv2.imshow(WIN, frame)

    params = (roi_x1, roi_y1, roi_x2, roi_y2,
              h_low, s_low, v_low, h_high, s_high, v_high, min_area)
    if params != last_params:
        last_params = params
        print(f'Found: {len(contours)}  |  '
              f'ROI [{roi_x1},{roi_y1}]–[{roi_x2},{roi_y2}]  |  '
              f'H [{h_low}-{h_high}]  S [{s_low}-{s_high}]  V [{v_low}-{v_high}]  '
              f'min_area={min_area}')

    if cv2.waitKey(50) & 0xFF in (ord('q'), 27):
        break

cv2.destroyAllWindows()

# ── Save crops for calibrate_hough.py ────────────────────────────────────────
import glob as _glob, os as _os
for _old in _glob.glob('/tmp/grab_cropped_*.bmp') + ['/tmp/grab_cropped.bmp']:
    try: _os.remove(_old)
    except FileNotFoundError: pass

saved = 0
if contours:
    for i, c in enumerate(contours):
        x, y, w, h = cv2.boundingRect(c)
        x1 = max(0, x - PAD);                  y1 = max(0, y - PAD)
        x2 = min(roi.shape[1], x + w + PAD);   y2 = min(roi.shape[0], y + h + PAD)
        die_mask = np.zeros(roi.shape[:2], dtype=np.uint8)
        cv2.drawContours(die_mask, [c], -1, 255, thickness=cv2.FILLED)
        masked = cv2.bitwise_and(roi, roi, mask=die_mask)
        cv2.imwrite(f'/tmp/grab_cropped_{i}.bmp', masked[y1:y2, x1:x2])
        saved += 1
    # Legacy single-file copy (also masked)
    x, y, w, h = cv2.boundingRect(contours[0])
    die_mask0 = np.zeros(roi.shape[:2], dtype=np.uint8)
    cv2.drawContours(die_mask0, [contours[0]], -1, 255, thickness=cv2.FILLED)
    masked0 = cv2.bitwise_and(roi, roi, mask=die_mask0)
    cv2.imwrite('/tmp/grab_cropped.bmp', masked0[
        max(0, y - PAD):min(roi.shape[0], y + h + PAD),
        max(0, x - PAD):min(roi.shape[1], x + w + PAD),
    ])
    print(f'\nSaved {saved} crop(s) to /tmp/grab_cropped_0.bmp … /tmp/grab_cropped_{saved-1}.bmp')
else:
    print('\nWARN: no dice found — no crops saved.')

# ── Auto-update both robot files ──────────────────────────────────────────────
replacements = {
    'HSV_H_LOW':        str(h_low),
    'HSV_S_LOW':        str(s_low),
    'HSV_V_LOW':        str(v_low),
    'HSV_H_HIGH':       str(h_high),
    'HSV_S_HIGH':       str(s_high),
    'HSV_V_HIGH':       str(v_high),
    'MIN_CONTOUR_AREA': str(min_area),
    'ROI_X1':           str(roi_x1),
    'ROI_Y1':           str(roi_y1),
    'ROI_X2':           str(roi_x2),
    'ROI_Y2':           str(roi_y2),
}

for path in (ROBOT1, ROBOT2):
    text = path.read_text()
    for name, value in replacements.items():
        text = re.sub(
            rf'^({re.escape(name)}\s*=\s*)\d+',
            lambda m, v=value: m.group(1) + v,
            text,
            flags=re.MULTILINE,
        )
    path.write_text(text)
    print(f'\nUpdated {path}:')
    for name, value in replacements.items():
        print(f'  {name} = {value}')
