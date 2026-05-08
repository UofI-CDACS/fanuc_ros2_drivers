#!/usr/bin/env python3
"""
Step 1 — tune HSV range:
    python3 tests/camera.py --tune
    Drag sliders until dice are solid white in the mask. Note the H/S/V values.

Step 2 — run detection with tuned values:
    python3 tests/camera.py
    Edit HSV_LOW / HSV_HIGH below once you know the right range.

No ROS2 required.
"""

import argparse
import platform
import sys

import cv2
import numpy as np

sys.path.insert(0, __file__.rsplit("/", 1)[0])
import mvsdk

# ── Tune these after running with --tune / --tune-pips ───────────────────────
HSV_LOW  = (10, 140, 40)
HSV_HIGH = (45, 255, 255)

PIP_AREA_MIN =30   # pip contours smaller than this are ignored (noise)
PIP_AREA_MAX = 90  # pip contours larger than this are ignored (dice border etc.)
PIP_AREA_MAX_SINGLE = 160  # relaxed max area used only for single-pip fallback
PIP_CIRCULARITY_MIN = 0.35  # 1.0 is a perfect circle
PIP_SOLIDITY_MIN = 0.70     # reject fragmented/noisy shapes
PIP_EDGE_MARGIN = 0.06      # reject border artifacts near crop edges
PIP_SINGLE_CENTER_TOL = 0.32  # fallback candidate must be near die center
# ─────────────────────────────────────────────────────────────────────────────


def _pip_candidates(thresh, area_min, area_max):
    """Return all contours, strict pip candidates, and relaxed single-pip candidates."""
    h, w = thresh.shape[:2]
    edge_x = int(w * PIP_EDGE_MARGIN)
    edge_y = int(h * PIP_EDGE_MARGIN)
    cx = w * 0.5
    cy = h * 0.5
    center_r2 = (min(w, h) * PIP_SINGLE_CENTER_TOL) ** 2

    dot_contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    kept = []
    relaxed_single = []

    for d in dot_contours:
        area = cv2.contourArea(d)

        x, y, ww, hh = cv2.boundingRect(d)
        if x <= edge_x or y <= edge_y or (x + ww) >= (w - edge_x) or (y + hh) >= (h - edge_y):
            continue

        bx = x + (ww * 0.5)
        by = y + (hh * 0.5)
        dist2 = (bx - cx) * (bx - cx) + (by - cy) * (by - cy)

        per = cv2.arcLength(d, True)
        if per <= 0:
            continue
        circularity = (4.0 * np.pi * area) / (per * per)

        hull = cv2.convexHull(d)
        hull_area = cv2.contourArea(hull)
        if hull_area <= 0:
            continue
        solidity = area / hull_area

        if (
            area_min < area < area_max
            and circularity >= PIP_CIRCULARITY_MIN
            and solidity >= PIP_SOLIDITY_MIN
        ):
            kept.append(d)

        if (
            area_max <= area < PIP_AREA_MAX_SINGLE
            and circularity >= (PIP_CIRCULARITY_MIN * 0.85)
            and solidity >= (PIP_SOLIDITY_MIN * 0.85)
            and dist2 <= center_r2
        ):
            relaxed_single.append(d)

    return dot_contours, kept, relaxed_single


def _select_final_pips(strict_pips, relaxed_single):
    """Use strict pips first; if none, allow one relaxed center pip candidate."""
    if strict_pips:
        return strict_pips, "strict"

    if len(relaxed_single) == 1:
        return [relaxed_single[0]], "single-fallback"

    return [], "none"


def grab_frame():
    DevList = mvsdk.CameraEnumerateDevice()
    if len(DevList) < 1:
        print("No camera found.")
        sys.exit(1)

    DevInfo = DevList[0]
    print(f"Camera: {DevInfo.GetFriendlyName()}")
    hCamera = mvsdk.CameraInit(DevInfo, -1, -1)
    cap = mvsdk.CameraGetCapability(hCamera)
    monoCamera = (cap.sIspCapacity.bMonoSensor != 0)

    fmt = mvsdk.CAMERA_MEDIA_TYPE_MONO8 if monoCamera else mvsdk.CAMERA_MEDIA_TYPE_BGR8
    mvsdk.CameraSetIspOutFormat(hCamera, fmt)
    mvsdk.CameraSetTriggerMode(hCamera, 0)
    mvsdk.CameraSetAeState(hCamera, 0)
    mvsdk.CameraSetExposureTime(hCamera, 60 * 1000)
    mvsdk.CameraPlay(hCamera)

    buf_size = (cap.sResolutionRange.iWidthMax *
                cap.sResolutionRange.iHeightMax *
                (1 if monoCamera else 3))
    pFrameBuffer = mvsdk.CameraAlignMalloc(buf_size, 16)

    try:
        pRawData, FrameHead = mvsdk.CameraGetImageBuffer(hCamera, 200)
        mvsdk.CameraImageProcess(hCamera, pRawData, pFrameBuffer, FrameHead)
        mvsdk.CameraReleaseImageBuffer(hCamera, pRawData)

        if platform.system() == "Windows":
            mvsdk.CameraFlipFrameBuffer(pFrameBuffer, FrameHead, 1)

        channels = 1 if monoCamera else 3
        frame_data = (mvsdk.c_ubyte * FrameHead.uBytes).from_address(pFrameBuffer)
        frame = np.frombuffer(frame_data, dtype=np.uint8).reshape(
            (FrameHead.iHeight, FrameHead.iWidth, channels)
        ).copy()
        return frame
    finally:
        mvsdk.CameraUnInit(hCamera)
        mvsdk.CameraAlignFree(pFrameBuffer)


def tune(img):
    """Interactive HSV slider window — drag until dice are white in the mask."""
    img = cv2.resize(img, (640, 480))
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    cv2.namedWindow("Tune HSV", cv2.WINDOW_NORMAL)
    cv2.createTrackbar("H low",       "Tune HSV", HSV_LOW[0],    179,  lambda x: None)
    cv2.createTrackbar("H high",      "Tune HSV", HSV_HIGH[0],   179,  lambda x: None)
    cv2.createTrackbar("S low",       "Tune HSV", HSV_LOW[1],    255,  lambda x: None)
    cv2.createTrackbar("S high",      "Tune HSV", HSV_HIGH[1],   255,  lambda x: None)
    cv2.createTrackbar("V low",       "Tune HSV", HSV_LOW[2],    255,  lambda x: None)
    cv2.createTrackbar("V high",      "Tune HSV", HSV_HIGH[2],   255,  lambda x: None)
    cv2.createTrackbar("Pip min area","Tune HSV", PIP_AREA_MIN,  500,  lambda x: None)
    cv2.createTrackbar("Pip max area","Tune HSV", PIP_AREA_MAX,  2000, lambda x: None)

    print("Drag sliders until the dice are solid white in the mask.")
    print("Press 'q' when done — values will be printed to the terminal.")

    while True:
        hl  = cv2.getTrackbarPos("H low",        "Tune HSV")
        hh  = cv2.getTrackbarPos("H high",       "Tune HSV")
        sl  = cv2.getTrackbarPos("S low",        "Tune HSV")
        sh  = cv2.getTrackbarPos("S high",       "Tune HSV")
        vl  = cv2.getTrackbarPos("V low",        "Tune HSV")
        vh  = cv2.getTrackbarPos("V high",       "Tune HSV")
        pmn = cv2.getTrackbarPos("Pip min area", "Tune HSV")
        pmx = cv2.getTrackbarPos("Pip max area", "Tune HSV")

        mask = cv2.inRange(hsv, (hl, sl, vl), (hh, sh, vh))
        kernel = np.ones((5, 5), np.uint8)
        mask_clean = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask_clean = cv2.morphologyEx(mask_clean, cv2.MORPH_CLOSE, kernel)

        # Live pip detection overlay
        overlay = img.copy()
        contours, _ = cv2.findContours(mask_clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        total_pips = 0
        for c in contours:
            if cv2.contourArea(c) < 500:
                continue
            x, y, w, h = cv2.boundingRect(c)
            x = max(x, 0); y = max(y, 0)
            orig_crop = img[y:y+h, x:x+w]
            gray = cv2.cvtColor(orig_crop, cv2.COLOR_BGR2GRAY)
            _, thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
            dot_contours, strict_pips, relaxed_single = _pip_candidates(thresh, pmn, pmx)
            kept, _ = _select_final_pips(strict_pips, relaxed_single)
            pip_count = len(kept)
            total_pips += pip_count
            cv2.rectangle(overlay, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(overlay, f"{pip_count} pips", (x, y - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            for d in kept:
                px, py, pw, ph = cv2.boundingRect(d)
                area = int(cv2.contourArea(d))
                cv2.rectangle(overlay[y:y+h, x:x+w], (px, py), (px+pw, py+ph), (0, 0, 255), 1)
                cv2.putText(overlay[y:y+h, x:x+w], str(area), (px, py - 2),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 200, 255), 1)

        cv2.putText(overlay, f"Total pips: {total_pips}", (8, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        cv2.imshow("Tune HSV", mask_clean)
        cv2.imshow("Original", img)
        cv2.imshow("Pip Detection", overlay)

        if cv2.waitKey(30) & 0xFF == ord('q'):
            print(f"\nCopy these into camera.py / pick_and_home.py:")
            print(f"HSV_LOW      = ({hl}, {sl}, {vl})")
            print(f"HSV_HIGH     = ({hh}, {sh}, {vh})")
            print(f"PIP_AREA_MIN = {pmn}")
            print(f"PIP_AREA_MAX = {pmx}")
            break

    cv2.destroyAllWindows()


def detect_pips(img, debug=False):
    img = cv2.resize(img, (640, 480))

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, HSV_LOW, HSV_HIGH)

    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    result = img.copy()

    print(f"Dice contours found: {len(contours)}, "
          f"areas: {[round(cv2.contourArea(c)) for c in contours]}")

    for i, c in enumerate(contours):
        if cv2.contourArea(c) < 500:
            continue

        # Draw oriented bounding box around dice (green)
        rect = cv2.minAreaRect(c)
        box = cv2.boxPoints(rect)
        cv2.drawContours(result, [np.intp(box)], -1, (0, 255, 0), 2)

        # No crop inset — use the full bounding rect so edge pips aren't clipped
        x, y, w, h = cv2.boundingRect(c)
        x = max(x, 0);  y = max(y, 0)
        dice_crop  = result[y:y+h, x:x+w]   # view into result for drawing
        orig_crop  = img[y:y+h, x:x+w]      # clean copy for thresholding

        gray = cv2.cvtColor(orig_crop, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)

        dot_contours, strict_pips, relaxed_single = _pip_candidates(thresh, PIP_AREA_MIN, PIP_AREA_MAX)
        kept, mode = _select_final_pips(strict_pips, relaxed_single)
        dot_areas = sorted([cv2.contourArea(d) for d in dot_contours], reverse=True)
        print(f"  Dice {i} — all pip areas: {[round(a) for a in dot_areas[:15]]}")
        print(
            "  Keeping: "
            f"{PIP_AREA_MIN}<area<{PIP_AREA_MAX}, "
            f"single_fallback<{PIP_AREA_MAX_SINGLE}, "
            f"circ>{PIP_CIRCULARITY_MIN}, solid>{PIP_SOLIDITY_MIN}, edge_margin={PIP_EDGE_MARGIN}"
        )
        if mode == "single-fallback":
            print("  Mode: single-pip fallback")

        dot_count = 0
        for d in kept:
            dot_count += 1
            px, py, pw, ph = cv2.boundingRect(d)
            cv2.rectangle(dice_crop, (px, py), (px + pw, py + ph), (0, 0, 255), 2)

        print(f"  Pips counted: {dot_count}")
        cv2.putText(result, str(dot_count), (x + w, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3, cv2.LINE_AA)

        if debug:
            cv2.imshow(f"Dice {i} thresh", thresh)

    cv2.imshow("Mask", mask)
    cv2.imshow("Detected Dice", result)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def tune_pips(img):
    """Show thresh image with sliders to calibrate pip min/max area."""
    img = cv2.resize(img, (640, 480))

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, HSV_LOW, HSV_HIGH)
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    dice_contours = [c for c in contours if cv2.contourArea(c) >= 500]

    if not dice_contours:
        print("No dice found — tune HSV first with --tune.")
        return

    # Use first detected dice
    c = dice_contours[0]
    x, y, w, h = cv2.boundingRect(c)
    crop = img[y:y+h, x:x+w]
    gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
    _, thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)

    cv2.namedWindow("Tune Pips", cv2.WINDOW_NORMAL)
    cv2.createTrackbar("Min area", "Tune Pips", PIP_AREA_MIN, 1000, lambda x: None)
    cv2.createTrackbar("Max area", "Tune Pips", PIP_AREA_MAX, 5000, lambda x: None)

    print("Drag sliders until only the pips are highlighted. Press 'q' when done.")

    while True:
        mn = cv2.getTrackbarPos("Min area", "Tune Pips")
        mx = cv2.getTrackbarPos("Max area", "Tune Pips")

        display = cv2.cvtColor(thresh, cv2.COLOR_GRAY2BGR)
        dot_contours, strict_pips, relaxed_single = _pip_candidates(thresh, mn, mx)
        kept, mode = _select_final_pips(strict_pips, relaxed_single)
        count = 0
        for d in kept:
            count += 1
            px, py, pw, ph = cv2.boundingRect(d)
            cv2.rectangle(display, (px, py), (px + pw, py + ph), (0, 0, 255), 2)

        cv2.putText(display, f"Pips: {count}", (5, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(display, f"All blobs: {len(dot_contours)}", (5, 45),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        cv2.putText(display, f"Mode: {mode}", (5, 70),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)
        cv2.imshow("Tune Pips", display)

        if cv2.waitKey(30) & 0xFF == ord('q'):
            print(f"\nCopy these into camera.py:")
            print(f"PIP_AREA_MIN = {mn}")
            print(f"PIP_AREA_MAX = {mx}")
            break

    cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--tune", action="store_true",
                        help="Tune HSV color range to detect dice")
    parser.add_argument("--tune-pips", action="store_true",
                        help="Tune pip min/max area thresholds")
    parser.add_argument("--debug", action="store_true",
                        help="Show thresholded crop for each dice")
    args = parser.parse_args()

    frame = grab_frame()
    print(f"Frame size: {frame.shape[1]}x{frame.shape[0]}")

    if args.tune:
        tune(frame)
    elif args.tune_pips:
        tune_pips(frame)
    else:
        detect_pips(frame, debug=args.debug)


if __name__ == "__main__":
    main()
