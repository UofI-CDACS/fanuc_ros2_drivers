"""
debug_colors.py — Live HSV colour picker for dice pip tuning.

  - Hover over any pixel to see its HSV value overlaid on the frame.
  - Click to lock a sample and print it to the terminal.
  - The yellow dice outline (cyan) and detected pip circles (green) update live
    so you can see exactly what the current tuning values are picking up.
  - Press 'q' to quit.

Run from the repo root:
  source install/setup.bash
  python3 debug_colors.py
"""

import cv2
import numpy as np
from camera import Camera

CAMERA_IP = 'Camera_IP'

# ── Copy updated values back into Controlling_robots_using_claude.py ──────────
YELLOW_LO    = (18, 180, 150)
YELLOW_HI    = (24, 255, 255)
BLACK_V_MAX  = 60
PIP_AREA_MIN = 50
PIP_AREA_MAX = 3000
# ─────────────────────────────────────────────────────────────────────────────

_hover_hsv  = None
_locked_hsv = None


def _mouse_cb(event, x, y, flags, hsv_ref):
    global _hover_hsv, _locked_hsv
    h, w = hsv_ref[0].shape[:2]
    if 0 <= y < h and 0 <= x < w:
        _hover_hsv = tuple(int(v) for v in hsv_ref[0][y, x])
    if event == cv2.EVENT_LBUTTONDOWN and _hover_hsv:
        _locked_hsv = _hover_hsv
        h2, s, v = _locked_hsv
        print(f'Locked  H={h2:3d}  S={s:3d}  V={v:3d}')


def _annotate(frame, hsv):
    # Yellow mask — fill the dice face solid so pip holes don't block the AND
    yellow_mask = cv2.inRange(hsv, np.array(YELLOW_LO), np.array(YELLOW_HI))
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
    yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_CLOSE, k)
    yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN,  k)
    y_cnts_raw, _ = cv2.findContours(yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    filled_mask = np.zeros_like(yellow_mask)
    if y_cnts_raw:
        largest = max(y_cnts_raw, key=cv2.contourArea)
        cv2.drawContours(filled_mask, [largest], -1, 255, cv2.FILLED)
    yellow_mask = filled_mask

    # Black pip mask inside yellow region (no morphological open — show raw blobs)
    black_mask = cv2.inRange(hsv, np.array([0, 0, 0]),
                                   np.array([180, 255, BLACK_V_MAX]))
    pip_mask = cv2.bitwise_and(black_mask, black_mask, mask=yellow_mask)

    # Show intermediate masks for diagnosis
    cv2.imshow('Black mask (all dark pixels)', black_mask)
    cv2.imshow('Yellow mask', yellow_mask)

    # All contours — print every area so we know what threshold to set
    all_contours, _ = cv2.findContours(pip_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    all_areas = sorted([int(cv2.contourArea(c)) for c in all_contours], reverse=True)
    if all_areas:
        print(f'Blob areas (largest first): {all_areas[:12]}')

    pip_contours = [c for c in all_contours
                    if PIP_AREA_MIN < cv2.contourArea(c) < PIP_AREA_MAX]

    out = frame.copy()

    # Show pip_mask as a second window (white = dark pixels inside dice face)
    cv2.imshow('Pip mask', pip_mask)

    # Draw yellow region outline in cyan
    y_cnts, _ = cv2.findContours(yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(out, y_cnts, -1, (255, 255, 0), 2)

    # Draw all raw blobs in red, filtered pips in green
    for c in all_contours:
        (cx, cy), r = cv2.minEnclosingCircle(c)
        cv2.circle(out, (int(cx), int(cy)), max(int(r), 4), (0, 0, 255), 1)
    for c in pip_contours:
        (cx, cy), r = cv2.minEnclosingCircle(c)
        cv2.circle(out, (int(cx), int(cy)), max(int(r), 4), (0, 255, 0), 2)

    # Text overlay
    lines = []
    if _hover_hsv:
        h2, s, v = _hover_hsv
        lines.append((f'Hover  H:{h2:3d} S:{s:3d} V:{v:3d}', (0, 255, 255)))
    if _locked_hsv:
        h2, s, v = _locked_hsv
        lines.append((f'Locked H:{h2:3d} S:{s:3d} V:{v:3d}', (0, 165, 255)))
    lines.append((f'Pips detected: {len(pip_contours)}  (all blobs: {len(all_contours)})', (0, 255, 0)))

    y_off = 35
    for text, color in lines:
        cv2.putText(out, text, (10, y_off),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.75, color, 2)
        y_off += 30

    return out


def main():
    camera = Camera(camera_ip=CAMERA_IP)
    cv2.namedWindow('HSV Picker',               cv2.WINDOW_NORMAL)
    cv2.namedWindow('Pip mask',                 cv2.WINDOW_NORMAL)
    cv2.namedWindow('Black mask (all dark pixels)', cv2.WINDOW_NORMAL)
    cv2.namedWindow('Yellow mask',              cv2.WINDOW_NORMAL)

    # hsv_ref is a mutable container so the mouse callback always sees the latest frame
    hsv_ref = [None]
    cv2.setMouseCallback('HSV Picker', _mouse_cb, hsv_ref)

    print('Hover to sample  |  Click to lock & print  |  q to quit')

    while True:
        frame = camera.getFrame()
        if frame is None:
            continue

        hsv_ref[0] = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        cv2.imshow('HSV Picker', _annotate(frame, hsv_ref[0]))

        if cv2.waitKey(30) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()
    camera.disable()


if __name__ == '__main__':
    main()
