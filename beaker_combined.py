"""
beaker_combined.py  —  Die face scanner

For each of 6 orientations the robot:
  1. Picks the die from a fixed position on the table
  2. Holds it under the overhead camera
  3. Counts pips on the visible face and saves the result
  4. Places the die back down

At the end, the pip count for each face is printed.

Usage:
  python beaker_combined.py

Press 'q' in the Live Stream window to abort.
"""

import cv2
import numpy as np
import mvsdk
import sys
import threading
from time import sleep
from datetime import datetime
from robot_controller import robot

LOG_FILE = 'die_scan_log.txt'

# ---------------------------------------------------------------------------
# Configuration  ← adjust these to match your setup
# ---------------------------------------------------------------------------

ROBOT_IP     = '10.8.4.16'
TRAVEL_SPEED = 80          # mm/s — slower for careful die handling

# Where the die sits on the table (robot X, Y coordinates)
#[452.9798583984375, -16.71965217590332, -188.15927124023438,
DIE_X = 452
DIE_Y = -16

PICK_Z  = -188    # Z height to grip die
SAFE_Z  =    1 # safe travel height (clear of obstacles)
# Verified show pose — confirmed reachable on the TP.
# The die is physically reoriented between faces so this one pose covers all scans.
SHOW_POSE = [649.497, 753.368, 464.626, 94.397, -65.012, -159.167]

NUM_FACES = 3   # number of faces to scan

# Home joint pose
HOME_JOINTS = [0.0504, -0.0500, -0.0473, 0.0480, -90.0499, 29.9936]

# ---------------------------------------------------------------------------
# 6 pick orientations — each shows a different face to the overhead camera.
# The first element of each tuple is a label; W, P, R define the end-effector
# orientation when picking.  Varying R rotates around the approach axis (shows
# 4 side faces); varying P tilts the die forward/back (shows top and bottom).
# Adjust values if a face doesn't read cleanly.
# ---------------------------------------------------------------------------

BASE_W = 178.1
BASE_P =   0.0688
BASE_R =  47.1492
#1, 

PIP_SAMPLE_FRAMES = 5    # frames to average pip count over while holding under camera
PIP_FRAME_DELAY   = 1.0  # seconds to display each counted frame (increase to observe longer)

# ---------------------------------------------------------------------------
# Pip counting (from BeakerFindDice3)
# ---------------------------------------------------------------------------

PIP_MIN_AREA        = 30
PIP_MAX_AREA        = 2000
PIP_MIN_CIRCULARITY = 0.55
PIP_MIN_CONVEXITY   = 0.70
PIP_MIN_INERTIA     = 0.30

LOWER_YELLOW = np.array([15, 100,  40])
UPPER_YELLOW = np.array([35, 255, 200])
MIN_DICE_AREA = 200


def _build_detector():
    p = cv2.SimpleBlobDetector_Params()
    p.filterByArea        = True;  p.minArea         = PIP_MIN_AREA;  p.maxArea    = PIP_MAX_AREA
    p.filterByCircularity = True;  p.minCircularity  = PIP_MIN_CIRCULARITY
    p.filterByConvexity   = True;  p.minConvexity    = PIP_MIN_CONVEXITY
    p.filterByInertia     = True;  p.minInertiaRatio = PIP_MIN_INERTIA
    p.filterByColor       = True;  p.blobColor       = 0
    return cv2.SimpleBlobDetector_create(p)

_detector = _build_detector()


def count_pips_in_frame(frame):
    """
    Detect yellow die in frame, count pips.
    Returns (pip_count, annotated_frame, die_found).
    die_found is False if no yellow region above MIN_DICE_AREA was detected.
    """
    display = frame.copy()
    hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, LOWER_YELLOW, UPPER_YELLOW)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    best_count = 0
    best_area  = 0

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < MIN_DICE_AREA:
            continue

        if area > best_area:
            best_area = area

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            x, y, w, h = cv2.boundingRect(cnt)
            pad = 4
            x0 = max(0, x - pad);  y0 = max(0, y - pad)
            x1 = min(gray.shape[1], x + w + pad)
            y1 = min(gray.shape[0], y + h + pad)
            roi = cv2.equalizeHist(gray[y0:y1, x0:x1])
            keypoints = _detector.detect(roi)
            best_count = len(keypoints)

            cv2.drawContours(display, [cnt], -1, (0, 220, 255), 2)

            # Draw each detected pip as a circle on the display frame
            for kp in keypoints:
                kx = int(kp.pt[0]) + x0
                ky = int(kp.pt[1]) + y0
                kr = max(3, int(kp.size / 2))
                cv2.circle(display, (kx, ky), kr, (0, 0, 255), 2)

            M = cv2.moments(cnt)
            if M['m00']:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                cv2.putText(display, f'{best_count} pips', (cx - 30, cy),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

    if best_area == 0:
        # Find largest contour area regardless of threshold for debugging
        all_areas = [cv2.contourArea(c) for c in contours]
        largest = max(all_areas) if all_areas else 0
        if largest > 0:
            print(f'    [DEBUG] Die not detected — largest contour area: {largest:.0f} (MIN_DICE_AREA={MIN_DICE_AREA})')

    die_found = best_area > 0
    return best_count, display, die_found


def draw_results_overlay(frame, results):
    """
    Draw a semi-transparent panel on the bottom-left of frame showing
    per-face pip counts and running total.
    """
    if not results:
        return
    lines = []
    total = 0
    for label, pips in results.items():
        lines.append(f'{label}: {pips} pip(s)')
        total += pips
    lines.append(f'Total: {total} pip(s)')

    font       = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.6
    thickness  = 1
    padding    = 8
    line_h     = 22

    panel_w = 220
    panel_h = padding * 2 + line_h * len(lines)
    h, w    = frame.shape[:2]
    px, py  = 10, h - panel_h - 10

    overlay = frame.copy()
    cv2.rectangle(overlay, (px, py), (px + panel_w, py + panel_h), (30, 30, 30), -1)
    cv2.addWeighted(overlay, 0.6, frame, 0.4, 0, frame)

    for i, line in enumerate(lines):
        ty = py + padding + line_h * i + 14
        colour = (0, 255, 100) if line.startswith('Total') else (0, 220, 255)
        cv2.putText(frame, line, (px + padding, ty), font, font_scale, colour, thickness)


def grab_frame(hCamera, pFrameBuffer):
    """Grab one frame from the mvsdk camera. Returns BGR numpy array."""
    pRawData, FrameHead = mvsdk.CameraGetImageBuffer(hCamera, 2000)
    mvsdk.CameraImageProcess(hCamera, pRawData, pFrameBuffer, FrameHead)
    mvsdk.CameraReleaseImageBuffer(hCamera, pRawData)
    frame_data = (mvsdk.c_ubyte * FrameHead.uBytes).from_address(pFrameBuffer)
    frame = np.frombuffer(frame_data, dtype=np.uint8).reshape(
        (FrameHead.iHeight, FrameHead.iWidth, 3))
    return cv2.flip(frame, 0)


# ---------------------------------------------------------------------------
# Thread coordination
# ---------------------------------------------------------------------------

# Robot thread sets _start_counting when the die is at the show position.
# Main camera loop counts PIP_SAMPLE_FRAMES frames then sets _counting_done.
_start_counting = threading.Event()
_counting_done  = threading.Event()
_pip_result     = [0]      # written by camera loop, read by robot thread
_robot_done     = [False]  # robot thread sets True when finished

# ---------------------------------------------------------------------------
# Robot motion helpers
# ---------------------------------------------------------------------------

def reorient_die(beaker):
    """
    Physically reorients the die by picking it up, rotating the gripper 90°,
    placing it back down, then returning the gripper to the standard orientation.
    Each call rotates the die 90° so the next pick exposes a different face.
    Returns True on success.
    """
    try:
        beaker.write_joint_pose(HOME_JOINTS)
        beaker.schunk_gripper('open')

        beaker.write_cartesian_position([DIE_X, DIE_Y, SAFE_Z, BASE_W, BASE_P, BASE_R])
        beaker.write_cartesian_position([DIE_X, DIE_Y, PICK_Z,  BASE_W, BASE_P, BASE_R])
        beaker.schunk_gripper('close')

        # Lift, then rotate gripper 90°
        beaker.write_cartesian_position([DIE_X, DIE_Y, SAFE_Z, BASE_W, BASE_P, BASE_R])
        beaker.write_cartesian_position([DIE_X, DIE_Y, SAFE_Z, BASE_W, BASE_P, BASE_R + 90])

        # Place die back down in new orientation
        beaker.write_cartesian_position([DIE_X, DIE_Y, PICK_Z,  BASE_W, BASE_P, BASE_R + 90])
        beaker.schunk_gripper('open')

        # Lift and rotate gripper back to standard pick orientation
        beaker.write_cartesian_position([DIE_X, DIE_Y, SAFE_Z, BASE_W, BASE_P, BASE_R + 90])
        beaker.write_cartesian_position([DIE_X, DIE_Y, SAFE_Z, BASE_W, BASE_P, BASE_R])

        beaker.write_joint_pose(HOME_JOINTS)
        print('[ROBOT] Die reoriented.')
        return True

    except Exception as e:
        print(f'[ROBOT] Reorient failed: {e}')
        print('  >>> Clear the TP alarm, then restart the program. <<<')
        return False


def pick_die(beaker):
    """
    Pick the die from the table using the fixed BASE orientation.
    Always starts and recovers from HOME_JOINTS.
    Returns True on success.
    """
    try:
        beaker.write_joint_pose(HOME_JOINTS)
        beaker.schunk_gripper('open')

        print('[ROBOT] Moving above die...')
        beaker.write_cartesian_position([DIE_X, DIE_Y, SAFE_Z, BASE_W, BASE_P, BASE_R])
        print('[ROBOT] Descending to pick height...')
        beaker.write_cartesian_position([DIE_X, DIE_Y, PICK_Z,  BASE_W, BASE_P, BASE_R])

        beaker.schunk_gripper('close')

        beaker.write_cartesian_position([DIE_X, DIE_Y, SAFE_Z, BASE_W, BASE_P, BASE_R])
        return True

    except Exception as e:
        print(f'[ROBOT] Pick failed: {e}')
        try:
            beaker.write_joint_pose(HOME_JOINTS)
        except:
            pass
        return False


def show_die(beaker, pose):
    """
    Move die to the camera view position using a verified show pose.
    pose is a 6-element list [X, Y, Z, W, P, R].
    Caller must have already sent the robot to HOME_JOINTS before calling this.
    """
    try:
        print(f'[ROBOT] Moving to show position (W={pose[3]:.1f}, P={pose[4]:.1f}, R={pose[5]:.1f})...')
        beaker.write_cartesian_position(pose)
        return True
    except Exception as e:
        print(f'[ROBOT] Show failed: {e}')
        return False


def place_die(beaker):
    """
    Return die to the table and release.
    Caller must have already sent the robot to HOME_JOINTS before calling this.
    """
    try:
        beaker.write_cartesian_position([DIE_X, DIE_Y, SAFE_Z, BASE_W, BASE_P, BASE_R])
        beaker.write_cartesian_position([DIE_X, DIE_Y, PICK_Z,  BASE_W, BASE_P, BASE_R])
        beaker.schunk_gripper('open')
        beaker.write_cartesian_position([DIE_X, DIE_Y, SAFE_Z, BASE_W, BASE_P, BASE_R])
        beaker.write_joint_pose(HOME_JOINTS)
        return True
    except Exception as e:
        print(f'[ROBOT] Place failed: {e}')
        try:
            beaker.write_joint_pose(HOME_JOINTS)
        except:
            pass
        return False

# ---------------------------------------------------------------------------
# Robot sequence (runs in background thread)
# ---------------------------------------------------------------------------

def safe_home(beaker):
    """Move to HOME_JOINTS, returning False on failure."""
    try:
        beaker.write_joint_pose(HOME_JOINTS)
        return True
    except Exception as e:
        print(f'[ROBOT] HOME failed: {e}')
        print('  >>> Clear the TP alarm, then restart the program. <<<')
        return False


def robot_sequence(beaker, results):
    """
    Runs all pick/reorient/show/place operations in a background thread.
    Signals the main camera loop when the die is ready to be counted,
    then waits for the count before continuing.
    """
    try:
        for face_num in range(1, NUM_FACES + 1):
            label = f'Face {face_num}'
            print(f'\n── {label} ──')

            if face_num > 1:
                print('  Reorienting die...')
                if not reorient_die(beaker):
                    break
                if not safe_home(beaker):
                    break

            print('  Picking die...')
            if not pick_die(beaker):
                print('  Skipping face due to pick error.')
                continue

            if not safe_home(beaker):
                break

            if not show_die(beaker, SHOW_POSE):
                print('  Show failed — robot may be in fault state.')
                print('  >>> Clear the TP alarm, then restart the program. <<<')
                break

            sleep(0.5)   # let arm settle

            # Signal the camera loop to start counting
            print(f'  Counting pips ({PIP_SAMPLE_FRAMES} frames)...')
            _counting_done.clear()
            _start_counting.set()
            _counting_done.wait()   # block until camera loop finishes counting

            results[label] = _pip_result[0]
            print(f'  Result: {_pip_result[0]} pip(s)')

            if not safe_home(beaker):
                break
            place_die(beaker)
            sleep(0.5)

    except Exception as e:
        print(f'[ROBOT] Unexpected error in robot sequence: {e}')
        print('  >>> Clear the TP alarm, then restart the program. <<<')

    finally:
        _robot_done[0] = True


# ---------------------------------------------------------------------------
# Main — camera loop runs here, robot runs in background thread
# ---------------------------------------------------------------------------

def main():
    # ── Robot init ──────────────────────────────────────────────────────────
    print(f'Connecting to robot at {ROBOT_IP}...')
    try:
        beaker = robot(ROBOT_IP)
        beaker.set_speed(TRAVEL_SPEED)
        print('Robot connected.')
    except Exception as e:
        print(f'Failed to connect to robot: {e}')
        sys.exit(1)

    # ── Camera init ─────────────────────────────────────────────────────────
    DevList = mvsdk.CameraEnumerateDevice()
    if not DevList:
        print('No camera detected!')
        sys.exit(1)

    hCamera  = mvsdk.CameraInit(DevList[0], -1, -1)
    cap_info = mvsdk.CameraGetCapability(hCamera)
    mvsdk.CameraSetIspOutFormat(hCamera, mvsdk.CAMERA_MEDIA_TYPE_BGR8)
    mvsdk.CameraSetTriggerMode(hCamera, 0)
    mvsdk.CameraSetAeState(hCamera, 0)
    mvsdk.CameraSetExposureTime(hCamera, 30 * 1000)
    mvsdk.CameraPlay(hCamera)
    sleep(1)

    buf_size     = cap_info.sResolutionRange.iWidthMax * cap_info.sResolutionRange.iHeightMax * 3
    pFrameBuffer = mvsdk.CameraAlignMalloc(buf_size, 16)

    cv2.namedWindow('Live Stream', cv2.WINDOW_NORMAL)
    cv2.namedWindow('Mask', cv2.WINDOW_NORMAL)
    print('Camera ready. Starting robot sequence...')

    results    = {}
    pip_frames      = []
    count_start_t   = None   # time when counting started (for timeout)
    COUNT_TIMEOUT   = 10.0   # seconds to wait for die before giving up

    # Start robot operations in background thread
    t = threading.Thread(target=robot_sequence, args=(beaker, results), daemon=True)
    t.start()

    try:
        while not _robot_done[0]:
            # Grab frame
            try:
                pRawData, FrameHead = mvsdk.CameraGetImageBuffer(hCamera, 200)
                mvsdk.CameraImageProcess(hCamera, pRawData, pFrameBuffer, FrameHead)
                mvsdk.CameraReleaseImageBuffer(hCamera, pRawData)
                frame_data = (mvsdk.c_ubyte * FrameHead.uBytes).from_address(pFrameBuffer)
                frame = np.frombuffer(frame_data, dtype=np.uint8).reshape(
                    (FrameHead.iHeight, FrameHead.iWidth, 3))
                frame = cv2.flip(frame, 0)
            except Exception:
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                continue

            if _start_counting.is_set():
                if count_start_t is None:
                    count_start_t = __import__('time').time()
                    # Print HSV value of the most saturated pixel — likely the die
                    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                    sat = hsv_frame[:, :, 1]
                    idx = np.unravel_index(np.argmax(sat), sat.shape)
                    h, s, v = hsv_frame[idx[0], idx[1]]
                    print(f'    [DEBUG] Most saturated pixel HSV: H={h}, S={s}, V={v} at ({idx[1]}, {idx[0]})')
                    # Draw a crosshair on the live stream at the sampled pixel
                    px, py = idx[1], idx[0]
                    scale_x = 800 / frame.shape[1]
                    scale_y = 600 / frame.shape[0]
                    dx, dy = int(px * scale_x), int(py * scale_y)
                    cv2.drawMarker(frame, (dx, dy), (0, 0, 255),
                                   cv2.MARKER_CROSS, 30, 2)
                    cv2.putText(frame, f'H={h} S={s} V={v}', (dx + 15, dy - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

                # Compute mask for Mask window
                hsv_f = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                mask  = cv2.inRange(hsv_f, LOWER_YELLOW, UPPER_YELLOW)
                cv2.imshow('Mask', cv2.resize(mask, (800, 600)))

                count, display, die_found = count_pips_in_frame(frame)

                if die_found:
                    pip_frames.append(count)
                    resized = cv2.resize(display, (800, 600))
                    draw_results_overlay(resized, results)
                    cv2.imshow('Live Stream', resized)
                    sleep(PIP_FRAME_DELAY)
                else:
                    # Die not yet visible — show warning overlay
                    warn = cv2.resize(frame, (800, 600))
                    cv2.putText(warn, 'Waiting for die...', (20, 50),
                                cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 165, 255), 3)
                    draw_results_overlay(warn, results)
                    cv2.imshow('Live Stream', warn)

                # Finish when enough die-detected frames collected, or timeout
                elapsed = __import__('time').time() - count_start_t
                if len(pip_frames) >= PIP_SAMPLE_FRAMES:
                    _pip_result[0] = round(sum(pip_frames) / len(pip_frames))
                    print(f'    Pip samples: {pip_frames}  →  averaged: {_pip_result[0]}')
                    pip_frames.clear()
                    count_start_t = None
                    _start_counting.clear()
                    _counting_done.set()
                elif elapsed > COUNT_TIMEOUT:
                    print(f'    [WARN] No die detected after {COUNT_TIMEOUT}s — skipping face.')
                    pip_frames.clear()
                    count_start_t = None
                    _pip_result[0] = 0
                    _start_counting.clear()
                    _counting_done.set()
            else:
                hsv_f = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                mask  = cv2.inRange(hsv_f, LOWER_YELLOW, UPPER_YELLOW)
                cv2.imshow('Mask', cv2.resize(mask, (800, 600)))
                idle = cv2.resize(frame, (800, 600))
                draw_results_overlay(idle, results)
                cv2.imshow('Live Stream', idle)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                print('Aborted by user.')
                break

        t.join(timeout=5)

    finally:
        mvsdk.CameraUnInit(hCamera)
        mvsdk.CameraAlignFree(pFrameBuffer)
        cv2.destroyAllWindows()

    # ── Summary ─────────────────────────────────────────────────────────────
    print('\n═══════════════════════════════')
    print('       DIE FACE SCAN RESULTS   ')
    print('═══════════════════════════════')
    for label, pips in results.items():
        print(f'  {label}: {pips} pip(s)')
    total_pips = sum(results.values()) if results else 0
    if results:
        print(f'  Total pips across all faces: {total_pips}')
        print(f'  Total unique faces scanned: {len(results)}')
    print('═══════════════════════════════')

    # ── Log file ─────────────────────────────────────────────────────────────
    if results:
        try:
            with open(LOG_FILE, 'a') as f:
                ts = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
                f.write(f'\n[{ts}] Scan session\n')
                for label, pips in results.items():
                    f.write(f'  {label}: {pips} pip(s)\n')
                f.write(f'  Total pips: {total_pips}\n')
                f.write(f'  Faces scanned: {len(results)}\n')
            print(f'Results saved to {LOG_FILE}')
        except Exception as e:
            print(f'[WARN] Could not write log file: {e}')


if __name__ == '__main__':
    main()
