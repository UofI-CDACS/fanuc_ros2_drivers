"""
Dice Vision Module
==================
Captures an image from the MindVision camera and counts the pips on the
dice face using OpenCV blob detection.

The Camera object is created once on first use and kept alive for the
duration of the task (avoiding repeated connect/disconnect overhead).
Call ``shutdown_camera()`` when the task is complete to release hardware.

Dependencies: opencv-python, numpy, mvsdk (bundled as dice_task/mvsdk.py)
"""

import logging

import cv2
import numpy as np

from dice_task.camera import Camera

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Camera singleton — connected once, reused across all repetitions
# ---------------------------------------------------------------------------

_camera: Camera | None = None


def _get_camera(index: int = 0) -> Camera | None:
    """Return the shared Camera instance, initialising it on first call."""
    global _camera
    if _camera is None:
        try:
            _camera = Camera(index=index)
        except Exception as exc:
            logger.error(f"Failed to initialise camera: {exc}")
            _camera = None
    return _camera


def connect_and_verify(index: int = 0):
    """
    Connect to the camera and capture a test frame to confirm it is working.

    Raises
    ------
    RuntimeError
        If the camera cannot be found, opened, or does not return a valid frame.
    """
    camera = _get_camera(index=index)
    if camera is None:
        raise RuntimeError("Camera not found — check cable and power")

    try:
        frame = camera.getFrame()
    except Exception as exc:
        raise RuntimeError(f"Camera connected but getFrame() failed: {exc}") from exc

    if frame is None:
        raise RuntimeError("Camera returned None frame")
    if not hasattr(frame, 'shape') or frame.size == 0:
        raise RuntimeError("Camera returned an empty frame")

    logger.info(
        f"Camera verified — frame shape {frame.shape}, "
        f"min/max pixel {frame.min()}/{frame.max()}"
    )


def shutdown_camera():
    """Release the camera hardware. Call this when the task is finished."""
    global _camera
    if _camera is not None:
        try:
            _camera.disable()
        except Exception as exc:
            logger.warning(f"Error while disabling camera: {exc}")
        _camera = None


# ---------------------------------------------------------------------------
# Image capture
# ---------------------------------------------------------------------------

def capture_image():
    """
    Capture a single frame from the MindVision camera.

    Returns
    -------
    numpy.ndarray
        Image array with shape (H, W, 3) for colour or (H, W, 1) for mono,
        or None if capture failed.
    """
    camera = _get_camera()
    if camera is None:
        logger.error("capture_image: camera not available")
        return None

    try:
        return camera.getFrame()
    except Exception as exc:
        logger.error(f"capture_image: getFrame() failed — {exc}")
        return None


# ---------------------------------------------------------------------------
# Pip counting
# ---------------------------------------------------------------------------

# ── Dice colour range (HSV) ──────────────────────────────────────────────────
# Tune these two constants to match the colour of your dice.
# HSV scale: Hue 0-180, Saturation 0-255, Value 0-255
#
# To find the right values:
#   python3 tests/cameraTest.py --pips --index 1
# The centre panel shows the colour mask — adjust until only the dice face
# is white.
#
# Common starting points:
#   White dice : lower=[0,   0, 160]  upper=[180,  60, 255]
#   Red dice   : lower=[0, 120,  60]  upper=[ 10, 255, 255]  (wraps; also try 170-180)
#   Blue dice  : lower=[100, 80, 60]  upper=[130, 255, 255]
# ─────────────────────────────────────────────────────────────────────────────
DICE_HSV_LOWER = np.array([ 18, 150, 100], dtype=np.uint8)   # ← TUNE THIS
DICE_HSV_UPPER = np.array([ 35, 255, 255], dtype=np.uint8)   # ← TUNE THIS


def _find_dice_roi_by_color(image):
    """
    Locate the dice face using HSV colour masking.

    Converts the BGR image to HSV, applies the DICE_HSV_LOWER/UPPER range,
    cleans up the mask with morphological ops, then returns the bounding box
    of the largest contour that looks like a dice face.

    Returns (x, y, w, h) or None.
    Only usable on colour (3-channel BGR) images.
    """
    if image is None or image.ndim != 3 or image.shape[2] != 3:
        return None   # mono image — caller should use grayscale fallback

    img_h, img_w = image.shape[:2]
    img_area = img_h * img_w

    hsv  = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, DICE_HSV_LOWER, DICE_HSV_UPPER)

    # Close small gaps inside the face, open noise outside it
    k_close = np.ones((15, 15), np.uint8)
    k_open  = np.ones(( 5,  5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k_close)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  k_open)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    best = None
    best_area = 0
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < img_area * 0.01 or area > img_area * 0.90:  # 1% min — dice is small in frame
            continue
        x, y, w, h = cv2.boundingRect(cnt)
        aspect = w / h if h > 0 else 0
        if not (0.5 <= aspect <= 2.0):
            continue
        if area > best_area:
            best_area = area
            best = (x, y, w, h)

    return best


def _find_dice_roi_grayscale(blurred):
    """
    Fallback ROI finder for mono images (no colour information).

    Uses morphological closing to fill pip holes so the face reads as a
    solid region, then finds the largest roughly-square contour.

    Returns (x, y, w, h) or None.
    """
    img_h, img_w = blurred.shape[:2]
    img_area = img_h * img_w

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


def _count_pips_with_keypoints(image):
    """
    Core detection logic shared by count_pips() and capture_save_and_count().

    Finds the dice face first, then restricts blob detection to that region
    so surrounding objects do not generate false pip counts.

    Returns
    -------
    (pip_count, keypoints, roi)
        pip_count : int
        keypoints : list of cv2.KeyPoint  (coordinates in full-image space)
        roi       : (x, y, w, h) or None
    """
    if image is None:
        return -1, [], None

    # Handle mono (H, W, 1) and colour (H, W, 3) frames from camera.py
    if image.ndim == 3 and image.shape[2] == 1:
        gray = image[:, :, 0]
    elif image.ndim == 3:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    else:
        gray = image  # already 2-D

    blurred = cv2.GaussianBlur(gray, (7, 7), 0)
    _, binary = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)

    # ── Blob detector parameters ─────────────────────────────────────────────
    # Tune these values if detection is unreliable for your dice / lighting.
    params = cv2.SimpleBlobDetector_Params()

    params.filterByColor  = True
    params.blobColor      = 255     # white blobs in the inverted image = dark pips

    params.filterByArea   = True
    params.minArea        = 200     # px² — raise if noise triggers false pips
    params.maxArea        = 2000    # px² — lower if two pips merge into one blob

    params.filterByCircularity = True
    params.minCircularity = 0.6     # 1.0 = perfect circle; lower = allow oval pips

    params.filterByConvexity = True
    params.minConvexity   = 0.8

    params.filterByInertia = True
    params.minInertiaRatio = 0.4    # lower = allow more elongated blobs
    # ────────────────────────────────────────────────────────────────────────

    detector = cv2.SimpleBlobDetector_create(params)

    # Try colour-based ROI first; fall back to grayscale morphology for mono images
    roi = _find_dice_roi_by_color(image)
    if roi is None:
        roi = _find_dice_roi_grayscale(blurred)
    if roi is None:
        logger.debug("No dice face found — running blob detection on full image")

    if roi is not None:
        rx, ry, rw, rh = roi
        pad = 5
        x1 = max(0, rx - pad)
        y1 = max(0, ry - pad)
        x2 = min(binary.shape[1], rx + rw + pad)
        y2 = min(binary.shape[0], ry + rh + pad)
        cropped = binary[y1:y2, x1:x2]
        kps_local = detector.detect(cropped)
        # Shift keypoint coordinates back to full-image space
        keypoints = [
            cv2.KeyPoint(kp.pt[0] + x1, kp.pt[1] + y1, kp.size,
                         kp.angle, kp.response, kp.octave, kp.class_id)
            for kp in kps_local
        ]
    else:
        keypoints = detector.detect(binary)

    return len(keypoints), keypoints, roi


def count_pips(image, debug: bool = False) -> int:
    """
    Count the number of pips on a dice face.

    Parameters
    ----------
    image : numpy.ndarray
        Frame returned by ``capture_image()``.
    debug : bool
        If True, open an annotated window (use only from cameraTest.py, not the task node).

    Returns
    -------
    int
        Number of pips detected (expected 1–6), or -1 on failure.
    """
    if image is None:
        logger.error("count_pips: received None image")
        return -1

    pip_count, keypoints, roi = _count_pips_with_keypoints(image)

    if debug:
        if image.ndim == 3 and image.shape[2] == 1:
            display = cv2.cvtColor(image[:, :, 0], cv2.COLOR_GRAY2BGR)
        elif image.ndim == 2:
            display = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        else:
            display = image.copy()
        annotated = cv2.drawKeypoints(
            display, keypoints, np.array([]),
            (0, 0, 255),
            cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS,
        )
        if roi is not None:
            rx, ry, rw, rh = roi
            cv2.rectangle(annotated, (rx, ry), (rx + rw, ry + rh), (0, 255, 0), 2)
        cv2.imshow(f"Detected pips: {pip_count}", annotated)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    return pip_count


# ---------------------------------------------------------------------------
# Save + count entry point (used by the task node)
# ---------------------------------------------------------------------------

def capture_save_and_count(save_dir: str, rep: int) -> tuple[int, str]:
    """
    Capture a still image, save both the raw and annotated result to disk,
    then return the pip count.

    Files written:
        dice_rep_01_raw.png        — unmodified camera frame
        dice_rep_01_detected.png   — annotated frame with blobs circled and count overlaid

    Parameters
    ----------
    save_dir : str
        Directory to write images into (created if it doesn't exist).
    rep : int
        Repetition number used to name the files.

    Returns
    -------
    (pip_count, annotated_path)
        pip_count      : int — number of pips detected, or -1 on failure
        annotated_path : str — path of the annotated image
    """
    import os
    os.makedirs(save_dir, exist_ok=True)

    image = capture_image()
    if image is None:
        logger.warning("capture_save_and_count: no image captured")
        return -1, ''

    # Save raw frame
    raw_path = os.path.join(save_dir, f'dice_rep_{rep:02d}_raw.png')
    cv2.imwrite(raw_path, image)
    logger.info(f"Saved raw frame: {raw_path}")

    # Run detection
    pip_count, keypoints, roi = _count_pips_with_keypoints(image)

    # Build and save annotated frame
    if image.ndim == 3 and image.shape[2] == 1:
        display = cv2.cvtColor(image[:, :, 0], cv2.COLOR_GRAY2BGR)
    elif image.ndim == 2:
        display = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    else:
        display = image.copy()

    annotated = cv2.drawKeypoints(
        display, keypoints, np.array([]),
        (0, 0, 255),
        cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS,
    )
    # Draw the detected dice face bounding box in green
    if roi is not None:
        rx, ry, rw, rh = roi
        cv2.rectangle(annotated, (rx, ry), (rx + rw, ry + rh), (0, 255, 0), 2)
    cv2.putText(
        annotated, f"Pips: {pip_count}",
        (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 2,
    )
    annotated_path = os.path.join(save_dir, f'dice_rep_{rep:02d}_detected.png')
    cv2.imwrite(annotated_path, annotated)
    logger.info(f"Saved annotated frame: {annotated_path}")

    logger.info(f"Rep {rep} pip count: {pip_count}")
    return pip_count, annotated_path
