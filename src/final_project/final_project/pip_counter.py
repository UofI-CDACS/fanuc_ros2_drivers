"""
pip_counter.py
Shared pip-detection logic (extracted from Controlling_robots_using_claude.py).
Both robot controllers import count_pips() after receiving an image from the camera server.
"""
import cv2
import numpy as np

# ── Tuning constants ──────────────────────────────────────────────────────────
YELLOW_LO    = (18, 180, 150)
YELLOW_HI    = (24, 255, 255)
BLACK_V_MAX  = 60
PIP_AREA_MIN = 30
PIP_AREA_MAX = 2000


def count_pips(image: np.ndarray) -> int:
    """
    Count black pips on a yellow dice face using HSV colour masking.

    Returns pip count 1-6, or 0 if the dice face cannot be found.
    image must be a BGR numpy array (as returned by the camera server).
    """
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Isolate yellow dice face
    yellow_mask = cv2.inRange(hsv, np.array(YELLOW_LO), np.array(YELLOW_HI))
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
    yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_CLOSE, kernel)
    yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN, kernel)

    y_cnts, _ = cv2.findContours(yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not y_cnts:
        return 0
    filled_mask = np.zeros_like(yellow_mask)
    cv2.drawContours(filled_mask, [max(y_cnts, key=cv2.contourArea)], -1, 255, cv2.FILLED)
    yellow_mask = filled_mask

    # Find dark pips inside the dice face
    black_mask = cv2.inRange(hsv, np.array([0, 0, 0]), np.array([180, 255, BLACK_V_MAX]))
    pip_mask = cv2.bitwise_and(black_mask, black_mask, mask=yellow_mask)
    pip_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    pip_mask = cv2.morphologyEx(pip_mask, cv2.MORPH_OPEN, pip_kernel)

    contours, _ = cv2.findContours(pip_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    pip_contours = [c for c in contours if PIP_AREA_MIN < cv2.contourArea(c) < PIP_AREA_MAX]
    return min(len(pip_contours), 6)


def save_debug_image(image: np.ndarray, pip_count: int, save_path: str) -> None:
    """Save an annotated copy of the image with detected pips circled."""
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    yellow_mask = cv2.inRange(hsv, np.array(YELLOW_LO), np.array(YELLOW_HI))
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
    yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_CLOSE, kernel)
    yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN, kernel)
    y_cnts, _ = cv2.findContours(yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if y_cnts:
        filled_mask = np.zeros_like(yellow_mask)
        cv2.drawContours(filled_mask, [max(y_cnts, key=cv2.contourArea)], -1, 255, cv2.FILLED)
        black_mask = cv2.inRange(hsv, np.array([0, 0, 0]), np.array([180, 255, BLACK_V_MAX]))
        pip_mask = cv2.bitwise_and(black_mask, black_mask, mask=filled_mask)
        pip_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        pip_mask = cv2.morphologyEx(pip_mask, cv2.MORPH_OPEN, pip_kernel)
        contours, _ = cv2.findContours(pip_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        pip_contours = [c for c in contours if PIP_AREA_MIN < cv2.contourArea(c) < PIP_AREA_MAX]

        annotated = image.copy()
        for c in pip_contours:
            (cx, cy), r = cv2.minEnclosingCircle(c)
            cv2.circle(annotated, (int(cx), int(cy)), max(int(r), 4), (0, 255, 0), 2)
        cv2.putText(annotated, f'Pips: {pip_count}', (10, 35),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 2)
        cv2.imwrite(save_path, annotated)
