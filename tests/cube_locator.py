import cv2
import numpy as np

import matplotlib.pyplot as plt
import matplotlib.image as mpimg

#Thresholds for mask
lower = np.array([0,173,61])
upper = np.array([25,255,255])
#View params
windowName = "Dice View"
displaySize = (640,480)
boundingColor = (255, 0, 0)
textColor = (0, 255, 0)

#Test lines
line_len = 190
line_width = 60

mtx = np.array([[4.65559641e+03, 0.00000000e+00, 6.39841114e+02],
 [0.00000000e+00, 4.66860013e+03, 5.12644840e+02],
 [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])

dist = np.array([-2.44010497e+00,  1.45289168e+02, -9.26494050e-03,
                   2.25284589e-02, -2.86541719e+03])



def processFrame(frame):
        #cv2.imwrite("image_dis.png", frame)
        #frame = cv2.undistort(frame, mtx, dist, None, mtx)
        #cv2.imwrite("image_un.png", frame)
        
        mask = makeMask(frame)

        # Find contours for cubes
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        min_area = 2000
        pickup_order = []      # [cx, cy, angle, priority, robot]
        cube_lines = []        # each cube candidate pickup lines
        cube_contours = []     # contours for hit testing

        # Process cubes
        for contour in contours:
                area = cv2.contourArea(contour)
                if area < min_area:
                        continue

                cube_idx = len(pickup_order)

                # Bounding box
                rect = cv2.minAreaRect(contour)
                (cx, cy), (w, h), angle = rect
                cx, cy = int(cx), int(cy)
                box = np.intp(cv2.boxPoints(rect))
                
                robot = 'dave'
                
                cube_contours.append(contour)
                # Draw bounding box
                cv2.drawContours(frame, [box], 0, boundingColor, 2)

                pickup_order.append([cx,cy, 0, None,robot])

                # Generate pickup lines
                makePickUpLines(angle, cx, cy, cube_lines, cube_idx)

        
        blocked = sortCubes(pickup_order, cube_lines, cube_contours)
        blocked = makeBlockList(pickup_order,blocked)
        # Draw selected pickup lines
        for cube_idx, cube in enumerate(pickup_order):
                cx, cy, chosen_angle, pri, robot = cube
                if chosen_angle is None:
                        continue

                # chosen_angle = correct_angle(cx,cy,chosen_angle)
                # pickup_order[cube_idx][2] = chosen_angle

                rad = np.deg2rad(chosen_angle)
                direction = np.array([np.cos(rad), np.sin(rad)], dtype=float)

                pt1 = (int(cx - direction[0] * line_len/2),
                        int(cy - direction[1] * line_len/2))
                pt2 = (int(cx + direction[0] * line_len/2),
                        int(cy + direction[1] * line_len/2))

                cv2.line(frame, pt1, pt2, (0, 0, 255), 2)
                cv2.putText(frame, f'A:{chosen_angle:.3f}', (int(cx), int(cy)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 2)
                # cv2.putText(frame, f'({cx},{cy}) P:{pri} R:{robot}', (int(cx), int(cy)),
                #                 cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 2)
        
        return frame, pickup_order, blocked

def makeMask(frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower, upper)
        #Clean mask up
        kernel = np.ones((5, 5), np.uint8)
        mask_clean = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask_clean = cv2.morphologyEx(mask_clean, cv2.MORPH_OPEN, kernel)

        #Blot out unnecessary bottom for now -- this works weird
        w, h, c = frame.shape
        y = int(h*.60)
        cv2.rectangle(mask_clean, (0, y), (h, h),(0,0,0) , cv2.FILLED)
                

        #Find Edges
        v = hsv[:,:,2]
        grad_x = cv2.Sobel(v, cv2.CV_32F, 1, 0, ksize=3)
        grad_y = cv2.Sobel(v, cv2.CV_32F, 0, 1, ksize=3)
        grad_mag = cv2.magnitude(grad_x, grad_y)
        grad_mag = cv2.convertScaleAbs(grad_mag)
        grad_blur = cv2.GaussianBlur(grad_mag, (3,3), 0)

        grad_norm = cv2.normalize(grad_blur, None, 0, 255, cv2.NORM_MINMAX)
        grad_enhanced = cv2.convertScaleAbs(grad_norm, alpha=2.5, beta=0)  # increase alpha for stronger edges
        grad_denoise = cv2.GaussianBlur(grad_enhanced, (3, 3), 0)

        grad_thresh = cv2.adaptiveThreshold(
                grad_denoise,
                255,
                cv2.ADAPTIVE_THRESH_MEAN_C,
                cv2.THRESH_BINARY,
                15,   # smaller blocksize = more local adaptation
                -5    # negative constant = more sensitive to light differences
        )

        #Final polished mask!
        return cv2.subtract(mask, grad_thresh)

def makePickUpLines(angle, cx, cy, cube_lines, cube_idx):
    base_rad = np.deg2rad(angle)
    dirs = np.array([
        [np.cos(base_rad), np.sin(base_rad)],        # main angle
        [np.cos(base_rad + np.pi/2), np.sin(base_rad + np.pi/2)] #90 turn
    ])

    center = np.array([cx, cy], dtype=float)

    for i, d in enumerate(dirs):
        p1 = center - d * (line_len/2)
        p2 = center + d * (line_len/2)

        cube_lines.append({
            "cube_idx": cube_idx,
            "pt1": p1.astype(int),
            "pt2": p2.astype(int),
            "angle": angle if i == 0 else (angle + 90)
        })

def lineHitsContour(pt1, pt2, contour):
        p1 = np.array(pt1, dtype=float)
        p2 = np.array(pt2, dtype=float)

        # Extract (N,2) list of contour vertices
        verts = contour[:, 0, :].astype(float)

        # Edges q1→q2
        q1 = verts
        q2 = np.roll(verts, -1, axis=0)

        # Vectorized ccw test:
        def ccw(a, b, c):
                return (c[...,1] - a[...,1]) * (b[...,0] - a[...,0]) > \
                        (b[...,1] - a[...,1]) * (c[...,0] - a[...,0])

        # Broadcast p1/p2 into shape (N,2) for vector tests
        p1B = np.broadcast_to(p1, q1.shape)
        p2B = np.broadcast_to(p2, q1.shape)

        # Vectorized segment intersection test
        inter = (ccw(p1B, q1, q2) != ccw(p2B, q1, q2)) & \
                (ccw(p1B, p2B, q1) != ccw(p1B, p2B, q2))

        # True if ANY edge intersects
        return np.any(inter)

def getOffsetLines(pt1, pt2, thickness):
    p1 = np.array(pt1, dtype=float)
    p2 = np.array(pt2, dtype=float)

    # Direction vector (normalized)
    d = p2 - p1
    d = d / np.linalg.norm(d)

    # Perpendicular vector (normal)
    n = np.array([-d[1], d[0]])

    half = thickness / 2.0

    # Offset lines
    p1L = (p1 + n * half).astype(int)
    p2L = (p2 + n * half).astype(int)
    p1R = (p1 - n * half).astype(int)
    p2R = (p2 - n * half).astype(int)

    return (p1L, p2L), (p1R, p2R)

def sortCubes(pickup_order, cube_lines, cube_contours):
    num_cubes = len(pickup_order)
    remaining = set(range(num_cubes))
    priority = 1

    while remaining:
        assigned_this_round = []

        # Try to assign cubes with 0 hit lines
        for cube_idx in list(remaining):

            candidate_lines = [l for l in cube_lines if l["cube_idx"] == cube_idx]
            if not candidate_lines:
                continue

            best_line = None
            best_hits = float('inf')

            # Test each line for this cube
            for line in candidate_lines:
                hits = 0

                pt1 = line["pt1"]
                pt2 = line["pt2"]

                # Generate gripper boundary lines
                (left_p1, left_p2), (right_p1, right_p2) = getOffsetLines(pt1, pt2, line_width)

                for other_idx, contour in enumerate(cube_contours):
                    if other_idx == cube_idx:
                        continue
                    if other_idx not in remaining:
                        continue

                    # Check hits against contour
                    if (lineHitsContour(pt1, pt2, contour) or
                        lineHitsContour(left_p1, left_p2, contour) or
                        lineHitsContour(right_p1, right_p2, contour)):
                        hits += 1

                # Track best line for this cube
                if hits < best_hits:
                    best_hits = hits
                    best_line = line

            # Found a valid pickup line (0 hits)
            if best_hits == 0 and best_line is not None:
                pickup_order[cube_idx][2] = best_line["angle"]   # chosen angle
                pickup_order[cube_idx][3] = priority             # priority
                assigned_this_round.append(cube_idx)

        # If NO cube had a 0 hit line
        if not assigned_this_round:
            # Mark all remaining cubes as unreachable
            for cube_idx in remaining:
                pickup_order[cube_idx][2] = None   # no angle
                pickup_order[cube_idx][3] = None   # no priority
            return remaining

        # Remove assigned cubes
        for cube_idx in assigned_this_round:
            remaining.remove(cube_idx)

        priority += 1

def makeBlockList(pick, block):
        if block is None:
              return []
        else:
                b = []
                for i in list(block)[::-1]:
                        b.append(pick[i])
                        pick.pop(i)
                return b
def showResize(frame):
        img = cv2.resize(frame, displaySize, interpolation = cv2.INTER_LINEAR)
        #cv2.imshow(windowName,frame)
        #I switched to matplotlib for displaying as it allows a pause, as opencv decided it doesnt like displaying images for me.
        matplotlib_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        plt.imshow(matplotlib_image)
        plt.axis('off')  # Hide axes for a cleaner image display
        plt.show()

def getFrameInfo(frame):
        # frame = cv2.imread('testimgs/image_dis.png')
        frame, pick, block = processFrame(frame)
        showResize(frame)
        return pick, block

