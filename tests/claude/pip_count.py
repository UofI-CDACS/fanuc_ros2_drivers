import cv2
import numpy as np


def pipCount():
    """
    Loads 'grab.bmp', finds the die closest to the camera (largest apparent area),
    counts pips, draws bounding boxes around all dice and circles on each pip,
    and saves the annotated image as 'grab_processed.bmp'.
    Returns the number of pips on the closest die.
    """
    img = cv2.imread("./grab.bmp")
    if img is None:
        raise FileNotFoundError("Could not open 'grab.bmp'")

    output = img.copy()

    # --- 1. Locate all dice faces using yellow colour mask ---
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    yellow_mask = cv2.inRange(hsv, np.array([15, 80, 80]), np.array([40, 255, 255]))

    contours, _ = cv2.findContours(
        yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )

    dice = []
    for c in contours:
        area = cv2.contourArea(c)
        if area < 2000:
            continue
        x, y, w, h = cv2.boundingRect(c)
        aspect_ratio = w / h
        fill_ratio = area / (w * h)
        # Dice faces are roughly square and fill their bounding box well
        if 0.8 <= aspect_ratio <= 1.25 and fill_ratio >= 0.65:
            dice.append((area, x, y, w, h, c))

    if not dice:
        raise RuntimeError("No dice found in the image")

    # --- 2. Draw bounding boxes around all dice ---
    dice.sort(reverse=True)
    for i, (area, x, y, w, h, c) in enumerate(dice):
        color = (0, 255, 0) if i == 0 else (255, 165, 0)  # green=closest, blue=others
        cv2.rectangle(output, (x, y), (x + w, y + h), color, 2)
        label = f"Die {i+1} (closest)" if i == 0 else f"Die {i+1}"
        cv2.putText(output, label, (x, y - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

    # --- 3. Count pips on closest die using blob detection ---
    _, x, y, w, h, _ = dice[0]
    roi = cv2.cvtColor(img[y : y + h, x : x + w], cv2.COLOR_BGR2GRAY)

    params = cv2.SimpleBlobDetector_Params()
    params.filterByArea = True
    params.minArea = 20
    params.maxArea = 500
    params.filterByCircularity = True
    params.minCircularity = 0.5
    params.filterByConvexity = True
    params.minConvexity = 0.7
    params.filterByInertia = False
    params.minThreshold = 10
    params.maxThreshold = 200

    detector = cv2.SimpleBlobDetector_create(params)
    keypoints = detector.detect(roi)

    # --- 4. Draw circles on each detected pip (offset to full image coords) ---
    for kp in keypoints:
        cx = int(kp.pt[0]) + x
        cy = int(kp.pt[1]) + y
        radius = max(4, int(kp.size / 2))
        cv2.circle(output, (cx, cy), radius, (0, 0, 255), 2)  # red circles

    pip_count = len(keypoints)
    cv2.putText(
        output,
        f"Pips: {pip_count}",
        (x, y + h + 18),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (0, 255, 0),
        2,
    )

    cv2.imwrite("./grab_processed.bmp", output)

    return pip_count


if __name__ == "__main__":
    pips = pipCount()
    print(f"Pips on the closest die: {pips}")
    print("Annotated image saved to grab_processed.bmp")
