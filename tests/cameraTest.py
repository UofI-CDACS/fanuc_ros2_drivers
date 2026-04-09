"""
Camera Connection Test
======================
Standalone script to verify the MindVision camera and debug pip detection,
completely independent of ROS2 and dice_task.

Usage:
    python3 tests/cameraTest.py                        # single capture, show result
    python3 tests/cameraTest.py --live                 # live preview until 'q' pressed
    python3 tests/cameraTest.py --save                 # save frame to tests/test_frame_<index>.png
    python3 tests/cameraTest.py --scan                 # try every detected camera one by one
    python3 tests/cameraTest.py --pips                 # live pip detection with annotated overlay
    python3 tests/cameraTest.py --pips --index 1       # use camera index 1

Run from the fanuc_ros2_drivers directory:
    cd ~/ros2_ws_Claude/fanuc_ros2_drivers
    python3 tests/cameraTest.py --pips --index 1
"""

import sys
import os
import argparse

# Add the dice_task source directory to the path so mvsdk and camera are importable
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src', 'dice_task', 'dice_task'))

import cv2
import numpy as np
import mvsdk


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def test_enumerate():
    """Check how many cameras are visible on the network."""
    print("\n── Camera Enumeration ──────────────────────────────────────")
    dev_list = mvsdk.CameraEnumerateDevice()
    n = len(dev_list)
    print(f"  Cameras found: {n}")
    for i, dev in enumerate(dev_list):
        print(f"  [{i}] {dev.GetFriendlyName()}  port: {dev.GetPortType()}")
    if n == 0:
        print("  FAIL: No cameras detected. Check cable and power.")
        return None
    print("  PASS")
    return dev_list


def test_open(dev_list, index=0, exposure_ms=30):
    """Open a specific camera by index."""
    print(f"\n── Camera Open (index={index}, exposure={exposure_ms}ms) ──────")
    dev_info = dev_list[index]
    h_camera = 0
    try:
        h_camera = mvsdk.CameraInit(dev_info, -1, -1)
    except mvsdk.CameraException as e:
        print(f"  FAIL: CameraInit error ({e.error_code}): {e.message}")
        return None, None

    cap = mvsdk.CameraGetCapability(h_camera)
    mono = (cap.sIspCapacity.bMonoSensor != 0)
    fmt = mvsdk.CAMERA_MEDIA_TYPE_MONO8 if mono else mvsdk.CAMERA_MEDIA_TYPE_BGR8
    mvsdk.CameraSetIspOutFormat(h_camera, fmt)
    mvsdk.CameraSetTriggerMode(h_camera, 0)
    mvsdk.CameraSetAeState(h_camera, 0)
    mvsdk.CameraSetExposureTime(h_camera, exposure_ms * 1000)
    mvsdk.CameraPlay(h_camera)

    buf_size = (cap.sResolutionRange.iWidthMax *
                cap.sResolutionRange.iHeightMax *
                (1 if mono else 3))
    frame_buf = mvsdk.CameraAlignMalloc(buf_size, 16)

    print(f"  Model    : {dev_info.GetFriendlyName()}")
    print(f"  Mono     : {mono}")
    print(f"  Max res  : {cap.sResolutionRange.iWidthMax} x {cap.sResolutionRange.iHeightMax}")
    print("  PASS: Camera opened and streaming")
    return h_camera, frame_buf


def test_capture(h_camera, frame_buf, label=''):
    """Grab one frame and report pixel stats."""
    print(f"\n── Frame Capture {label} ───────────────────────────────────")
    try:
        raw, head = mvsdk.CameraGetImageBuffer(h_camera, 2000)
        mvsdk.CameraImageProcess(h_camera, raw, frame_buf, head)
        mvsdk.CameraReleaseImageBuffer(h_camera, raw)

        channels = 1 if head.uiMediaType == mvsdk.CAMERA_MEDIA_TYPE_MONO8 else 3
        data = (mvsdk.c_ubyte * head.uBytes).from_address(frame_buf)
        frame = np.frombuffer(data, dtype=np.uint8).reshape(
            (head.iHeight, head.iWidth, channels)
        )
        print(f"  Shape    : {frame.shape}")
        print(f"  Dtype    : {frame.dtype}")
        print(f"  Min/Max  : {frame.min()} / {frame.max()}")
        mean = frame.mean()
        print(f"  Mean     : {mean:.1f}")
        if frame.max() == 0:
            print("  WARN: Frame is all black — wrong camera, lens cap, or exposure too low")
        elif mean < 5:
            print("  WARN: Frame is very dark — try a higher exposure with --exposure")
        else:
            print("  PASS: Frame captured successfully")
        return frame
    except mvsdk.CameraException as e:
        print(f"  FAIL: Capture error ({e.error_code}): {e.message}")
        return None


def close_camera(h_camera, frame_buf):
    mvsdk.CameraUnInit(h_camera)
    mvsdk.CameraAlignFree(frame_buf)


# ---------------------------------------------------------------------------
# Modes
# ---------------------------------------------------------------------------

def run_single(index=0, exposure_ms=30, save=False):
    """Enumerate, open, capture one frame, optionally save it."""
    dev_list = test_enumerate()
    if dev_list is None:
        return

    h_camera, frame_buf = test_open(dev_list, index=index, exposure_ms=exposure_ms)
    if h_camera is None:
        return

    frame = test_capture(h_camera, frame_buf, label=f'index={index}')

    if frame is not None:
        if save:
            out_path = os.path.join(os.path.dirname(__file__), f'test_frame_{index}.png')
            cv2.imwrite(out_path, frame)
            print(f"\n  Saved frame to: {out_path}")

        print("\n── Display ─────────────────────────────────────────────────")
        print("  Press any key to close.")
        cv2.imshow(f"Camera {index} — {exposure_ms}ms exposure", frame)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    close_camera(h_camera, frame_buf)
    print("\nDone.\n")


def run_scan(exposure_ms=30):
    """Try every detected camera one at a time and show a frame from each."""
    dev_list = test_enumerate()
    if dev_list is None:
        return

    print(f"\nScanning all {len(dev_list)} camera(s) — press any key to advance.\n")
    for i in range(len(dev_list)):
        h_camera, frame_buf = test_open(dev_list, index=i, exposure_ms=exposure_ms)
        if h_camera is None:
            continue

        frame = test_capture(h_camera, frame_buf, label=f'index={i}')
        if frame is not None:
            print(f"  Showing camera {i} — press any key to continue.")
            cv2.imshow(f"Camera {i} — press any key", frame)
            cv2.waitKey(0)
            cv2.destroyAllWindows()

        close_camera(h_camera, frame_buf)

    print("\nScan complete. Set camera_index in task_config.yaml to the correct index.\n")


def run_live(index=0, exposure_ms=30):
    """Continuous preview until 'q' is pressed."""
    dev_list = test_enumerate()
    if dev_list is None:
        return

    h_camera, frame_buf = test_open(dev_list, index=index, exposure_ms=exposure_ms)
    if h_camera is None:
        return

    print(f"\n── Live Preview (camera {index}, {exposure_ms}ms) ──────────────")
    print("  Press 'q' in the preview window to quit.")

    while True:
        try:
            raw, head = mvsdk.CameraGetImageBuffer(h_camera, 200)
            mvsdk.CameraImageProcess(h_camera, raw, frame_buf, head)
            mvsdk.CameraReleaseImageBuffer(h_camera, raw)

            channels = 1 if head.uiMediaType == mvsdk.CAMERA_MEDIA_TYPE_MONO8 else 3
            data = (mvsdk.c_ubyte * head.uBytes).from_address(frame_buf)
            frame = np.frombuffer(data, dtype=np.uint8).reshape(
                (head.iHeight, head.iWidth, channels)
            )
            cv2.imshow(f"Camera {index} live (q to quit)", frame)
        except mvsdk.CameraException:
            pass

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()
    close_camera(h_camera, frame_buf)
    print("\nDone.\n")


# ---------------------------------------------------------------------------
# Pip detection debug
# ---------------------------------------------------------------------------

# ── Dice colour range — keep in sync with dice_vision.py ─────────────────────
DICE_HSV_LOWER = np.array([ 18, 150, 100], dtype=np.uint8)   # ← TUNE THIS
DICE_HSV_UPPER = np.array([ 35, 255, 255], dtype=np.uint8)   # ← TUNE THIS
# ─────────────────────────────────────────────────────────────────────────────


def _find_dice_roi_by_color(frame):
    """Colour-based ROI — mirror of dice_vision._find_dice_roi_by_color."""
    if frame is None or frame.ndim != 3 or frame.shape[2] != 3:
        return None, None
    img_area = frame.shape[0] * frame.shape[1]
    hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, DICE_HSV_LOWER, DICE_HSV_UPPER)
    k_close = np.ones((15, 15), np.uint8)
    k_open  = np.ones(( 5,  5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k_close)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  k_open)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best = None
    best_area = 0
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < img_area * 0.01 or area > img_area * 0.90:  # 1% min
            continue
        x, y, w, h = cv2.boundingRect(cnt)
        aspect = w / h if h > 0 else 0
        if not (0.5 <= aspect <= 2.0):
            continue
        if area > best_area:
            best_area = area
            best = (x, y, w, h)
    return best, cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)


def _find_dice_roi_grayscale(blurred):
    """Grayscale fallback ROI — mirror of dice_vision._find_dice_roi_grayscale."""
    img_area = blurred.shape[0] * blurred.shape[1]
    _, binary = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    kernel = np.ones((25, 25), np.uint8)
    closed = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best = None
    best_area = 0
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < img_area * 0.01 or area > img_area * 0.60:  # 1% min
            continue
        x, y, w, h = cv2.boundingRect(cnt)
        aspect = w / h if h > 0 else 0
        if not (0.5 <= aspect <= 2.0):
            continue
        hull_area = cv2.contourArea(cv2.convexHull(cnt))
        if hull_area == 0 or (area / hull_area) < 0.6:
            continue
        if area > best_area:
            best_area = area
            best = (x, y, w, h)
    return best


def _detect_pips(frame):
    """
    Run the same blob detection pipeline used in dice_vision.py.
    Returns (pip_count, annotated_frame, binary_frame, color_mask_frame).
    Tune DICE_HSV_LOWER/UPPER above until only the dice face is white in the
    colour mask panel, then copy those values into dice_vision.py.
    """
    # Grayscale — handle (H,W,1) mono and (H,W,3) colour frames
    if frame.ndim == 3 and frame.shape[2] == 1:
        gray = frame[:, :, 0]
    elif frame.ndim == 3:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    else:
        gray = frame

    blurred = cv2.GaussianBlur(gray, (7, 7), 0)
    _, binary = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)

    # ── Blob detector parameters — mirror these in dice_vision.py ────────────
    params = cv2.SimpleBlobDetector_Params()
    params.filterByColor       = True
    params.blobColor           = 255
    params.filterByArea        = True
    params.minArea             = 200
    params.maxArea             = 1000
    params.filterByCircularity = True
    params.minCircularity      = 0.6
    params.filterByConvexity   = True
    params.minConvexity        = 0.8
    params.filterByInertia     = True
    params.minInertiaRatio     = 0.4
    # ─────────────────────────────────────────────────────────────────────────

    detector = cv2.SimpleBlobDetector_create(params)

    # Try colour ROI first, then grayscale fallback
    roi, color_mask_bgr = _find_dice_roi_by_color(frame)
    if roi is None:
        roi = _find_dice_roi_grayscale(blurred)
        color_mask_bgr = np.zeros((*binary.shape, 3), dtype=np.uint8)  # blank if not colour

    if roi is not None:
        rx, ry, rw, rh = roi
        pad = 5
        x1 = max(0, rx - pad)
        y1 = max(0, ry - pad)
        x2 = min(binary.shape[1], rx + rw + pad)
        y2 = min(binary.shape[0], ry + rh + pad)
        cropped = binary[y1:y2, x1:x2]
        kps_local = detector.detect(cropped)
        keypoints = [
            cv2.KeyPoint(kp.pt[0] + x1, kp.pt[1] + y1, kp.size,
                         kp.angle, kp.response, kp.octave, kp.class_id)
            for kp in kps_local
        ]
    else:
        keypoints = detector.detect(binary)

    display = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    annotated = cv2.drawKeypoints(
        display, keypoints, np.array([]),
        (0, 0, 255),
        cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS,
    )
    if roi is not None:
        rx, ry, rw, rh = roi
        cv2.rectangle(annotated, (rx, ry), (rx + rw, ry + rh), (0, 255, 0), 2)
        if color_mask_bgr is not None:
            cv2.rectangle(color_mask_bgr, (rx, ry), (rx + rw, ry + rh), (0, 255, 0), 2)
    cv2.putText(
        annotated, f"Pips: {len(keypoints)}",
        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2,
    )

    binary_bgr = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)
    return len(keypoints), annotated, binary_bgr, color_mask_bgr


def run_pips(index=0, exposure_ms=30):
    """
    Live pip detection debug.

    Shows three side-by-side panels:
      Left  — raw camera frame
      Centre — binary (thresholded) image used for blob detection
      Right — annotated frame with detected blobs circled and pip count

    Press 'q' to quit.
    Tune minArea / maxArea / minCircularity in _detect_pips() above until
    the count is consistently correct, then copy the values to dice_vision.py.
    """
    dev_list = test_enumerate()
    if dev_list is None:
        return

    h_camera, frame_buf = test_open(dev_list, index=index, exposure_ms=exposure_ms)
    if h_camera is None:
        return

    print(f"\n── Pip Detection Debug (camera {index}, {exposure_ms}ms) ────────")
    print("  Left: raw | Centre-left: colour mask | Centre-right: binary | Right: detected blobs")
    print("  Tune DICE_HSV_LOWER/UPPER at the top of this file until only the")
    print("  dice face is white in the colour mask panel, then copy to dice_vision.py.")
    print("  Press 'q' to quit.\n")

    while True:
        try:
            raw, head = mvsdk.CameraGetImageBuffer(h_camera, 200)
            mvsdk.CameraImageProcess(h_camera, raw, frame_buf, head)
            mvsdk.CameraReleaseImageBuffer(h_camera, raw)

            channels = 1 if head.uiMediaType == mvsdk.CAMERA_MEDIA_TYPE_MONO8 else 3
            data  = (mvsdk.c_ubyte * head.uBytes).from_address(frame_buf)
            frame = np.frombuffer(data, dtype=np.uint8).reshape(
                (head.iHeight, head.iWidth, channels)
            )

            pip_count, annotated, binary_bgr, color_mask_bgr = _detect_pips(frame)

            # Make raw BGR for concatenation
            if frame.ndim == 3 and frame.shape[2] == 1:
                raw_bgr = cv2.cvtColor(frame[:, :, 0], cv2.COLOR_GRAY2BGR)
            elif frame.ndim == 2:
                raw_bgr = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
            else:
                raw_bgr = frame.copy()

            combined = np.hstack([raw_bgr, color_mask_bgr, binary_bgr, annotated])
            # Scale down if too wide for the screen
            h, w = combined.shape[:2]
            if w > 1800:
                scale = 1800 / w
                combined = cv2.resize(combined, (int(w * scale), int(h * scale)))

            cv2.imshow("Pip Debug — raw | colour mask | binary | detected  (q to quit)", combined)
            print(f"\r  Pip count: {pip_count}   ", end='', flush=True)

        except mvsdk.CameraException:
            pass

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    print()
    cv2.destroyAllWindows()
    close_camera(h_camera, frame_buf)
    print("\nDone.\n")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='MindVision camera connection test')
    parser.add_argument('--index',    type=int, default=0,  help='Camera index to use (default: 0)')
    parser.add_argument('--exposure', type=int, default=30, help='Exposure time in ms (default: 30)')
    group = parser.add_mutually_exclusive_group()
    group.add_argument('--live', action='store_true', help='Continuous live preview')
    group.add_argument('--save', action='store_true', help='Save frame to tests/test_frame_<index>.png')
    group.add_argument('--scan', action='store_true', help='Try every detected camera one by one')
    group.add_argument('--pips', action='store_true', help='Live pip detection with annotated overlay')
    args = parser.parse_args()

    if args.live:
        run_live(index=args.index, exposure_ms=args.exposure)
    elif args.scan:
        run_scan(exposure_ms=args.exposure)
    elif args.pips:
        run_pips(index=args.index, exposure_ms=args.exposure)
    else:
        run_single(index=args.index, exposure_ms=args.exposure, save=args.save)
