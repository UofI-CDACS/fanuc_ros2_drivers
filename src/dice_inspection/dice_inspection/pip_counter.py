#!/usr/bin/env python3
"""
pip_counter.py

Captures an image from the MindVision camera, saves it as die_image.jpg,
counts the pips on the yellow die, saves the annotated result as
die_processed.jpg, displays it for 2 seconds, and prints the pip count.

Usage:
  python3 pip_counter.py
"""

import os
import sys
import time
import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mvsdk


# -----------------------------------------------------------------------
# Camera capture
# -----------------------------------------------------------------------
def capture_image() -> np.ndarray:
    devs = mvsdk.CameraEnumerateDevice()
    if len(devs) < 1:
        raise RuntimeError('No MindVision camera found! Check ethernet connection.')

    hCamera = mvsdk.CameraInit(devs[0], -1, -1)
    cap = mvsdk.CameraGetCapability(hCamera)
    mono = (cap.sIspCapacity.bMonoSensor != 0)

    fmt = mvsdk.CAMERA_MEDIA_TYPE_MONO8 if mono else mvsdk.CAMERA_MEDIA_TYPE_BGR8
    mvsdk.CameraSetIspOutFormat(hCamera, fmt)
    mvsdk.CameraSetTriggerMode(hCamera, 0)
    mvsdk.CameraSetAeState(hCamera, 0)
    mvsdk.CameraSetExposureTime(hCamera, 30 * 1000)
    mvsdk.CameraPlay(hCamera)

    channels = 1 if mono else 3
    buf_size = cap.sResolutionRange.iWidthMax * cap.sResolutionRange.iHeightMax * channels
    pFrameBuffer = mvsdk.CameraAlignMalloc(buf_size, 16)

    try:
        pRawData, FrameHead = mvsdk.CameraGetImageBuffer(hCamera, 2000)
        mvsdk.CameraImageProcess(hCamera, pRawData, pFrameBuffer, FrameHead)
        mvsdk.CameraReleaseImageBuffer(hCamera, pRawData)
        frame_data = (mvsdk.c_ubyte * FrameHead.uBytes).from_address(pFrameBuffer)
        frame = np.frombuffer(frame_data, dtype=np.uint8).reshape(
            (FrameHead.iHeight, FrameHead.iWidth, channels)
        )
        return frame.copy()
    finally:
        mvsdk.CameraUnInit(hCamera)
        mvsdk.CameraAlignFree(pFrameBuffer)


# -----------------------------------------------------------------------
# Die detection
# -----------------------------------------------------------------------
def find_die_bbox(image: np.ndarray):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_yellow = np.array([15, 60, 40])
    upper_yellow = np.array([45, 255, 255])
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel, iterations=1)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None, mask

    largest = max(contours, key=cv2.contourArea)
    x, y, w, h = cv2.boundingRect(largest)

    margin = 8
    x = max(0, x - margin)
    y = max(0, y - margin)
    w = min(image.shape[1] - x, w + 2 * margin)
    h = min(image.shape[0] - y, h + 2 * margin)

    return (x, y, w, h), mask


# -----------------------------------------------------------------------
# Pip counting
# -----------------------------------------------------------------------
def count_pips(image: np.ndarray, bbox: tuple):
    x, y, w, h = bbox
    roi = image[y:y+h, x:x+w].copy()

    # Boost brightness/contrast — image is dark
    roi_bright = cv2.convertScaleAbs(roi, alpha=2.5, beta=30)
    gray = cv2.cvtColor(roi_bright, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (3, 3), 0)

    # Otsu's threshold to find dark regions (pips)
    _, dark = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    dark = cv2.morphologyEx(dark, cv2.MORPH_OPEN, np.ones((2, 2), np.uint8))

    # Save debug images
    cv2.imwrite('debug_roi.jpg', roi)
    cv2.imwrite('debug_roi_bright.jpg', roi_bright)
    cv2.imwrite('debug_dark_mask.jpg', dark)

    contours, _ = cv2.findContours(dark, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    print(f'  Total contours in ROI: {len(contours)}')

    # Inner zone: exclude outer 22% border to avoid pips on side faces
    border_x = int(w * 0.18)
    border_y = int(h * 0.18)

    pip_count = 0
    for c in contours:
        area = cv2.contourArea(c)
        perimeter = cv2.arcLength(c, True)
        if perimeter == 0:
            continue
        circularity = 4 * np.pi * area / (perimeter ** 2)

        if area < 100 or area > 3000:
            continue
        if circularity <= 0.65:
            continue

        M = cv2.moments(c)
        if M['m00'] == 0:
            continue
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])

        # Skip pips near the edges — those are on side faces
        if cx < border_x or cx > w - border_x or cy < border_y or cy > h - border_y:
            print(f'    SKIPPED (edge/side face): area={area:.1f}, circularity={circularity:.2f}, center=({cx},{cy})')
            continue

        print(f'    PIP: area={area:.1f}, circularity={circularity:.2f}, center=({cx},{cy})')
        pip_count += 1
        cv2.drawContours(roi, [c], -1, (0, 255, 0), 2)
        cv2.circle(roi, (cx, cy), 3, (0, 0, 255), -1)

    return pip_count, roi


# -----------------------------------------------------------------------
# Main
# -----------------------------------------------------------------------
def main():
    print('Capturing image from MindVision camera...')
    frame = capture_image()
    print(f'Captured: {frame.shape[1]}x{frame.shape[0]}')

    cv2.imwrite('die_image.jpg', frame)
    print('Raw image saved as: die_image.jpg')

    bbox, _ = find_die_bbox(frame)
    if bbox is None:
        print('ERROR: No yellow die detected in image.')
        sys.exit(1)

    x, y, w, h = bbox
    print(f'Die detected at: x={x}, y={y}, w={w}, h={h}')

    pip_count, annotated_roi = count_pips(frame, bbox)

    output = frame.copy()
    output[y:y+h, x:x+w] = annotated_roi
    cv2.rectangle(output, (x, y), (x + w, y + h), (0, 255, 255), 3)
    cv2.putText(output, f'Pips: {pip_count}', (x, max(y - 10, 20)),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)

    cv2.imwrite('die_processed.jpg', output)
    print('Processed image saved as: die_processed.jpg')
    print('Debug images saved: debug_roi.jpg, debug_gray.jpg, debug_dark_mask.jpg')

    print(f'\n=============================')
    print(f'  Pip count: {pip_count}')
    print(f'=============================\n')

    # Display for 2 seconds then close
    cv2.imshow('Pip Counter Result', output)
    cv2.waitKey(2000)
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
