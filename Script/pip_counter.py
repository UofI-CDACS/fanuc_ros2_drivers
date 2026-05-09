import cv2
import numpy as np


def pipCount():
    """
    Loads 'grab.bmp', finds the die closest to the camera (largest apparent area),
    counts pips, draws rotated bounding boxes around all dice and circles on each pip,
    and saves the annotated image as 'grab_processed.bmp'.
    Returns the number of pips on the closest die.
    """
    img = cv2.imread("./grab.bmp")
    if img is None:
        raise FileNotFoundError("Could not open 'grab.bmp'")

    output = img.copy()
    h_img, w_img = img.shape[:2]

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
        rect = cv2.minAreaRect(c)
        bw, bh = rect[1]
        if bw == 0 or bh == 0:
            continue
        aspect = max(bw, bh) / min(bw, bh)
        fill = area / (bw * bh)
        # Dice faces are roughly square and fill their bounding box well
        if aspect <= 1.4 and fill >= 0.65:
            dice.append((area, rect, c))

    if not dice:
        raise RuntimeError("No dice found in the image")

    # --- 2. Draw rotated bounding boxes around all dice ---
    dice.sort(key=lambda d: d[0], reverse=True)
    for i, (area, rect, c) in enumerate(dice):
        box = np.intp(cv2.boxPoints(rect))
        color = (0, 255, 0) if i == 0 else (255, 165, 0)
        cv2.drawContours(output, [box], 0, color, 2)
        lx = int(rect[0][0]) - 40
        ly = int(rect[0][1]) - int(max(rect[1]) / 2) - 6
        label = f"Die {i+1} (closest)" if i == 0 else f"Die {i+1}"
        cv2.putText(output, label, (lx, ly), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

    # --- 3. Deskew the closest die face into an axis-aligned ROI ---
    _, rect, _ = dice[0]
    center, (bw, bh), angle = rect

    # Normalize so the longer dimension becomes horizontal after rotation
    if bw < bh:
        angle += 90
    long_side = max(bw, bh)
    short_side = min(bw, bh)

    M = cv2.getRotationMatrix2D(center, angle, 1.0)
    rotated = cv2.warpAffine(img, M, (w_img, h_img), flags=cv2.INTER_CUBIC)

    x0 = max(0, int(center[0] - long_side / 2))
    y0 = max(0, int(center[1] - short_side / 2))
    x1 = min(w_img, int(center[0] + long_side / 2))
    y1 = min(h_img, int(center[1] + short_side / 2))
    roi = rotated[y0:y1, x0:x1]

    # --- 4. Isolate just the top (camera-facing) die face ---
    # The top face is brighter than the foreshortened side faces.
    # Use the 75th-percentile V per column so dark pip pixels don't bias the result.
    roi_hsv_tmp = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    col_v75 = np.percentile(roi_hsv_tmp[:, :, 2].astype(float), 75, axis=0)
    v_thresh = col_v75.max() * 0.85
    face_cols = np.where(col_v75 > v_thresh)[0]
    if len(face_cols) > 10:
        face_l = int(face_cols[0])
        face_r = int(face_cols[-1]) + 1
        roi = roi[:, face_l:face_r]
        x0 += face_l  # keep offset correct for pip coordinate mapping

    # --- 4b. Shrink ROI inward to exclude edge artifacts ---
    margin_frac = 0.10
    rh, rw = roi.shape[:2]
    mx = int(rw * margin_frac)
    my = int(rh * margin_frac)
    if mx > 0 and my > 0 and rw - 2 * mx > 0 and rh - 2 * my > 0:
        roi = roi[my : rh - my, mx : rw - mx]
        x0 += mx
        y0 += my

    # --- 5. Count pips on closest die using blob detection ---
    roi_gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

    params = cv2.SimpleBlobDetector_Params()
    params.filterByArea = True
    params.minArea = 20
    params.maxArea = 2000
    params.filterByCircularity = True
    params.minCircularity = 0.5
    params.filterByConvexity = True
    params.minConvexity = 0.7
    params.filterByInertia = False
    params.minThreshold = 10
    params.maxThreshold = 200

    detector = cv2.SimpleBlobDetector_create(params)
    keypoints = detector.detect(roi_gray)

    # --- 6. Map pip positions back to original image coords and draw ---
    M_inv = cv2.getRotationMatrix2D(center, -angle, 1.0)
    for kp in keypoints:
        # Position in the deskewed full image
        px = float(kp.pt[0]) + x0
        py = float(kp.pt[1]) + y0
        # Un-rotate back to original image coords
        pt = np.array([[[px, py]]], dtype=np.float32)
        orig = cv2.transform(pt, M_inv)[0][0]
        radius = max(4, int(kp.size / 2))
        cv2.circle(output, (int(orig[0]), int(orig[1])), radius, (0, 0, 255), 2)

    pip_count = len(keypoints)
    die_cx = int(rect[0][0])
    die_cy = int(rect[0][1])
    cv2.putText(
        output,
        f"Pips: {pip_count}",
        (die_cx - 30, die_cy + int(max(rect[1]) / 2) + 18),
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
