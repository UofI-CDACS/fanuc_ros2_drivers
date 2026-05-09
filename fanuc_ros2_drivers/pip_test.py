#!/usr/bin/env python3
"""
pip_test.py
===========
Standalone pip-detection test — no ROS2, no robot required.

Snaps a single image, runs the full detection pipeline, prints the pip count,
saves debug images to /tmp/, and shows the results. Press any key to close.

Debug windows shown
-------------------
  Feed        — annotated frame (blue dice outline, numbered red pip contours, count)
  Dice mask   — brightness-threshold mask showing what was found as the dice face
  Warped face — the 200×200 normalised dice crop (grayscale)
  Pip binary  — adaptive-threshold result on the warped face
                (white blobs = pip candidates; accepted ones get a red number)

Tuning guide
------------
  Dice not found in mask?      → lower LOWER_YELLOW[2] (V) toward 0
  Wood/floor showing in mask?  → raise LOWER_YELLOW[0] (H) or LOWER_YELLOW[1] (S)
  Mask has holes/gaps?         → increase CLOSE_SIZE
  Binary all one colour?       → Otsu needs contrast; check Warped face window
  Wrong pip count?             → adjust MIN_AREA / MAX_AREA or MIN_CIRC
"""

import os
import sys
from ctypes import addressof, c_ubyte

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mvsdk

# ---------------------------------------------------------------------------
#  Tuning constants — change these without touching the logic below
# ---------------------------------------------------------------------------

CAMERA_INDEX  = 0  # 0 = first MindVision device found

# Auto-exposure brightness target (0–255).  Default is ~120, which overexposes
# the yellow dice to near-white, destroying the hue/saturation information the
# detector relies on.  80 keeps the dice as a saturated yellow.
# After changing this, re-run hsv_picker.py to recalibrate LOWER/UPPER_YELLOW.
AE_TARGET     = 80

# HSV range for the yellow dice face.
# Hue (H) is stable across lighting levels; Value (V) is permissive so dim
# yellow is still caught even when underexposed.
# Brown wood typically has H < 18 and lower saturation, so it is excluded.
# Calibrated with hsv_picker.py against the actual dice under real lighting.
# H 7–30  = yellow hue band
# S ≥ 126 = excludes low-saturation objects (white, grey, metallic gripper)
# V 39–119 = excludes both very dark and very bright (overexposed) regions
LOWER_YELLOW  = np.array([7, 131, 29])
UPPER_YELLOW  = np.array([32, 255, 116])

# Morphological close kernel: fills pip holes AND small gaps from partial HSV coverage.
CLOSE_SIZE    = 21   # pixels — increase if the mask has large gaps inside the dice

WARP_SIZE     = 250  # normalised dice-face canvas (pixels)

MIN_AREA      = 900    # min pip area in WARP_SIZE×WARP_SIZE space  (px²)
MAX_AREA      = 15000 # max pip area — must scale with WARP_SIZE; the single pip on
                      # a "1" die fills ~20% of the 250×250 canvas (~12 500 px²)
MIN_CIRC      = 0.50 # minimum circularity  (1.0 = perfect circle)

# Fraction of frame height to black out before dice detection.
# Auto-exposure brightens the wood table edges at the top and bottom of the
# frame so they fall inside the yellow HSV range.  The dice on the black cloth
# is always in the central strip, so blanking these edges costs nothing.
IGNORE_TOP_FRAC    = 0.15   # ignore top 15 % of the frame
IGNORE_BOTTOM_FRAC = 0.15   # ignore bottom 15 % of the frame

# ---------------------------------------------------------------------------
#  Camera helpers
# ---------------------------------------------------------------------------

def open_camera(index: int):
    """Open a MindVision camera and return (hCamera, pFrameBuffer, is_color)."""
    dev_list = mvsdk.CameraEnumerateDevice()
    if len(dev_list) <= index:
        raise RuntimeError(f'No MindVision camera at index {index} '
                           f'({len(dev_list)} device(s) found).')

    h = mvsdk.CameraInit(dev_list[index])
    max_w, max_h, b_color = mvsdk.CameraGetCapabilityEx2(h)
    is_color = (b_color != 0)

    if is_color:
        mvsdk.CameraSetIspOutFormat(h, mvsdk.CAMERA_MEDIA_TYPE_BGR8)
        ch = 3
    else:
        mvsdk.CameraSetIspOutFormat(h, mvsdk.CAMERA_MEDIA_TYPE_MONO8)
        ch = 1

    buf = (c_ubyte * (max_w * max_h * ch))()
    mvsdk.CameraSetAeState(h, True)
    mvsdk.CameraSetAeTarget(h, AE_TARGET)
    mvsdk.CameraSetTriggerMode(h, 1)   # 1 = software trigger (request/response)
    mvsdk.CameraPlay(h)
    print(f'Camera opened: {"colour" if is_color else "mono"}, max {max_w}×{max_h}')

    return h, buf, is_color


def grab_frame(h, buf, is_color) -> np.ndarray:
    """Send a software trigger and return the captured frame as a BGR numpy array."""
    mvsdk.CameraSoftTrigger(h)
    raw, head = mvsdk.CameraGetImageBuffer(h, 2000)
    mvsdk.CameraImageProcess(h, raw, addressof(buf), head)
    mvsdk.CameraReleaseImageBuffer(h, raw)

    ch = 3 if is_color else 1
    n  = head.iWidth * head.iHeight * ch
    view = (c_ubyte * n).from_address(addressof(buf))
    img  = np.frombuffer(view, dtype=np.uint8).reshape(
               (head.iHeight, head.iWidth, ch)).copy()

    if not is_color:
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    return img


# ---------------------------------------------------------------------------
#  Pip detection
# ---------------------------------------------------------------------------

def detect_pips(frame: np.ndarray):
    """
    Run the full pip-detection pipeline on a BGR frame.

    Returns
    -------
    annotated   : BGR frame with dice outline and numbered pip contours
    mask_vis    : BGR visualisation of the dice brightness mask
    warped_gray : grayscale 200×200 normalised dice face (or None)
    pip_binary  : annotated binary 200×200 threshold result (or None)
    pip_count   : int
    """
    gray    = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    # ---- 1. Find dice face by hue ----
    # Hue is stable even in dim lighting, unlike brightness.
    # V threshold is near-zero so underexposed yellow still passes.
    # Brown wood has H < 18 and lower S, so it is naturally excluded.
    hsv      = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    raw_mask = cv2.inRange(hsv, LOWER_YELLOW, UPPER_YELLOW)

    # Morphological close: fills dark pip holes AND small gaps where underexposed
    # yellow pixels fell just outside the HSV range.
    close_kernel = np.ones((CLOSE_SIZE, CLOSE_SIZE), np.uint8)
    closed_mask  = cv2.morphologyEx(raw_mask, cv2.MORPH_CLOSE, close_kernel)

    # Blank the top and bottom edge strips so overexposed wood at the frame
    # border cannot be mistaken for the dice face.
    h_f = closed_mask.shape[0]
    top_px    = int(h_f * IGNORE_TOP_FRAC)
    bottom_px = int(h_f * IGNORE_BOTTOM_FRAC)
    closed_mask[:top_px, :]        = 0
    closed_mask[h_f - bottom_px:, :] = 0

    contours_d, _ = cv2.findContours(closed_mask, cv2.RETR_EXTERNAL,
                                      cv2.CHAIN_APPROX_SIMPLE)

    # Pick the most square contour within a plausible size range.
    # Taking the largest contour fails when the floor/table edge is also yellow-ish.
    # A dice is roughly square (aspect ratio ≈ 1); the floor edge is elongated.
    img_area     = frame.shape[0] * frame.shape[1]
    dice_contour = None
    best_area    = -1.0
    print(f'  Yellow contours found: {len(contours_d)}')
    for cnt in contours_d:
        area = cv2.contourArea(cnt)
        # minAreaRect gives the true rotated bounding rectangle — aspect ≈ 1.0
        # for a square die regardless of rotation; elongated regions stay low.
        rect        = cv2.minAreaRect(cnt)
        rw, rh      = rect[1]
        mar_aspect  = min(rw, rh) / max(rw, rh) if max(rw, rh) > 0 else 0
        print(f'    area={area:.0f}  minRect={rw:.0f}x{rh:.0f}  mar_aspect={mar_aspect:.2f}')
        if area < 5000 or area > img_area * 0.80:
            print(f'      → skipped (area out of range)')
            continue
        if mar_aspect < 0.50:
            print(f'      → skipped (too elongated, aspect={mar_aspect:.2f})')
            continue
        # Among candidates that are square enough, take the LARGEST —
        # the dice is the biggest square yellow object in the frame.
        if area > best_area:
            best_area    = area
            dice_contour = cnt
            print(f'      → new best (area={area:.0f}, aspect={mar_aspect:.2f})')
    print(f'  Dice contour selected: {dice_contour is not None}'
          + (f'  (area={best_area:.0f})' if dice_contour is not None else ''))

    # Mask visualisation — green tint over detected dice region
    mask_vis = frame.copy()
    if dice_contour is not None:
        dice_fill = np.zeros_like(closed_mask)
        cv2.drawContours(dice_fill, [dice_contour], -1, 255, cv2.FILLED)
        mask_vis[dice_fill > 0] = (
            mask_vis[dice_fill > 0].astype(np.float32) * 0.5
            + np.array([0, 80, 0]) * 0.5
        ).astype(np.uint8)
        cv2.drawContours(mask_vis, [dice_contour], -1, (255, 100, 0), 2)

    # ---- 2. Perspective-warp dice face to fixed 200×200 canvas ----
    pip_contours_full = []
    warped_gray       = None
    pip_binary        = None
    valid_warped      = []

    if dice_contour is not None:
        rect    = cv2.minAreaRect(dice_contour)
        box     = cv2.boxPoints(rect)
        box     = box[np.argsort(box[:, 1])]
        top_row = box[:2][np.argsort(box[:2, 0])]
        bot_row = box[2:][np.argsort(box[2:, 0])]
        src_pts = np.array([top_row[0], top_row[1],
                             bot_row[1], bot_row[0]], dtype=np.float32)
        dst_pts = np.array([[0, 0], [WARP_SIZE-1, 0],
                             [WARP_SIZE-1, WARP_SIZE-1], [0, WARP_SIZE-1]],
                            dtype=np.float32)
        M_warp   = cv2.getPerspectiveTransform(src_pts, dst_pts)
        M_unwarp = cv2.getPerspectiveTransform(dst_pts, src_pts)

        warped_gray = cv2.warpPerspective(blurred, M_warp, (WARP_SIZE, WARP_SIZE))

        # Build a solid filled contour mask (no pip holes) and warp it into the
        # WARP_SIZE×WARP_SIZE space.  Using closed_mask directly left a large
        # hole at the "1" pip position because the 21-px close kernel is too
        # small to fill the single large pip; that hole then excluded the real
        # pip from detection after erosion.
        dice_filled_mask = np.zeros(closed_mask.shape, dtype=np.uint8)
        cv2.drawContours(dice_filled_mask, [dice_contour], -1, 255, cv2.FILLED)
        warped_face_mask = cv2.warpPerspective(
            dice_filled_mask, M_warp, (WARP_SIZE, WARP_SIZE)
        )

        # ---- 3. Otsu threshold ----
        _, pip_binary = cv2.threshold(
            warped_gray, 0, 255,
            cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU
        )

        # Erode the face mask before applying it — only dark blobs sitting well
        # inside a large bright yellow region can be counted as pips.
        # 40-px kernel → ~20 px exclusion border around the WARP_SIZE canvas.
        # A real die pip is ≥60 px from the edge on a 250-px canvas (safe).
        # The corner artifact from the gripper bleed-in sits ~10-15 px from
        # the edge and is excluded by this margin.
        erode_kernel    = np.ones((40, 40), np.uint8)
        face_mask_inner = cv2.erode(warped_face_mask, erode_kernel)
        pip_binary[face_mask_inner == 0] = 0

        # ---- 4. Filter contours by area and circularity ----
        pip_cnts, _ = cv2.findContours(pip_binary, cv2.RETR_EXTERNAL,
                                        cv2.CHAIN_APPROX_SIMPLE)
        for cnt in pip_cnts:
            area = cv2.contourArea(cnt)
            if area < MIN_AREA or area > MAX_AREA:
                continue
            perim = cv2.arcLength(cnt, True)
            if perim < 1:
                continue
            if 4 * np.pi * area / (perim ** 2) < MIN_CIRC:
                continue
            valid_warped.append(cnt)

        # Project accepted pip contours back to full-frame coordinates
        for cnt in valid_warped:
            pts = cnt.reshape(-1, 1, 2).astype(np.float32)
            pip_contours_full.append(
                cv2.perspectiveTransform(pts, M_unwarp).astype(np.int32)
            )

    pip_count = len(pip_contours_full)

    # ---- 5. Annotate main frame ----
    annotated = frame.copy()
    if dice_contour is not None:
        cv2.drawContours(annotated, [dice_contour], -1, (255, 100, 0), 2)

    for i, cnt in enumerate(pip_contours_full):
        cv2.drawContours(annotated, [cnt], -1, (0, 0, 255), 2)
        M = cv2.moments(cnt)
        if M['m00'] > 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            cv2.putText(annotated, str(i + 1), (cx + 8, cy + 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 4)
            cv2.putText(annotated, str(i + 1), (cx + 8, cy + 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

    for color, thickness in [((0, 0, 0), 5), ((0, 255, 0), 2)]:
        cv2.putText(annotated, f'Pips: {pip_count}', (10, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.4, color, thickness)

    # Annotate pip_binary with accepted pip numbers
    if pip_binary is not None:
        pip_binary_bgr = cv2.cvtColor(pip_binary, cv2.COLOR_GRAY2BGR)
        for i, cnt in enumerate(valid_warped):
            cv2.drawContours(pip_binary_bgr, [cnt], -1, (0, 0, 255), 1)
            M = cv2.moments(cnt)
            if M['m00'] > 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                cv2.putText(pip_binary_bgr, str(i + 1), (cx + 4, cy + 4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 180, 255), 2)
        pip_binary = pip_binary_bgr

    return annotated, mask_vis, warped_gray, pip_binary, pip_count


# ---------------------------------------------------------------------------
#  Main — snap one image, show results, press any key to close
# ---------------------------------------------------------------------------

def main():
    h, buf, is_color = open_camera(CAMERA_INDEX)

    try:
        print('Capturing image...')
        frame = grab_frame(h, buf, is_color)
    finally:
        mvsdk.CameraStop(h)
        mvsdk.CameraUnInit(h)

    annotated, mask_vis, warped_gray, pip_binary, pip_count = detect_pips(frame)

    print(f'Pips detected: {pip_count}')

    # Save all debug images to /tmp/
    cv2.imwrite('/tmp/pip_test_feed.png',   annotated)
    cv2.imwrite('/tmp/pip_test_mask.png',   mask_vis)
    if warped_gray is not None:
        cv2.imwrite('/tmp/pip_test_warp.png',   warped_gray)
    if pip_binary is not None:
        cv2.imwrite('/tmp/pip_test_binary.png', pip_binary)
    print('Debug images saved to /tmp/pip_test_*.png')

    # Show all windows — press any key to close
    for win, img in [('Feed',        annotated),
                     ('Dice mask',   mask_vis),
                     ('Warped face', warped_gray),
                     ('Pip binary',  pip_binary)]:
        if img is None:
            continue
        cv2.namedWindow(win, cv2.WINDOW_NORMAL)
        cv2.imshow(win, img)

    cv2.resizeWindow('Feed',        1280, 960)
    cv2.resizeWindow('Dice mask',    640, 480)
    cv2.resizeWindow('Warped face',  400, 400)
    cv2.resizeWindow('Pip binary',   400, 400)

    print('Press any key to close.')
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
