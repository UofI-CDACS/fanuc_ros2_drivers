#!/usr/bin/env python3
"""
AllClaudeLookatDice.py
======================
ROS2 program for automated dice inspection using a Fanuc robot and a USB camera.

OVERVIEW
--------
This file contains two ROS2 nodes that work together:

  1. DiceCameraNode           – Hosts a ROS2 service. When the master calls it,
                                the camera captures one frame, counts the pips on
                                the visible dice face with OpenCV, and returns the
                                count in the service response.

  2. DiceInspectionMasterNode – The control brain. Drives the Fanuc robot through
                                a pick → present → return sequence for each of 3
                                dice, calls the camera service to get pip counts,
                                and logs which dice showed which count.

COMMUNICATION BETWEEN NODES
----------------------------
  Master calls service:  /dice_inspection/read_dice  (std_srvs/Trigger)
    - Request:  empty  (just the call itself is the trigger)
    - Response: success (bool), message (string containing the pip count integer)

ROBOT ACTIONS USED
------------------
  /<ROBOT_NAMESPACE>/cartesian_pose   (fanuc_interfaces/action/CartPose)
  /<ROBOT_NAMESPACE>/o
  
-----------------
  /<ROBOT_NAMESPACE>/is_moving  (fanuc_interfaces/msg/IsMoving)
      Published by move_check.py (msg_publishers) at 0.5 s intervals.
      Used to detect when the robot has ACTUALLY finished each move,
      working around the action server's is_moving() race condition.
      Requires msg_publishers to be running (included in start.launch.py).

HOW TO RUN
----------
  1. Start the Fanuc action servers AND message publishers:
         ros2 launch fanuc_ros2_drivers start.launch.py robot_name:=<your_robot> robot_ip:=<your_robot_ip>
  2. Plug in USB camera.
  3. Calibrate the pose constants below for your robot cell.
  4. python3 AllClaudeLookatDice.py

CALIBRATION REQUIRED
--------------------
  Before running, update ROBOT_NAMESPACE and the pose lists in the
  "=== USER-CONFIGURABLE CONSTANTS ===" section to match your robot cell.
"""

import os
import sys
import threading
import time
from ctypes import addressof, c_ubyte

import cv2
import numpy as np
import rclpy

# MindVision MVSDK Python wrapper lives alongside this script
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mvsdk
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger

from fanuc_interfaces.action import CartPose, OnRobotGripper, SJointPose
from fanuc_interfaces.msg import CurJoints, IsMoving

# =============================================================================
#  === USER-CONFIGURABLE CONSTANTS ===
#  Calibrate these values to match your physical robot cell before running.
# =============================================================================

# Namespace used by the running Fanuc action servers (see their launch files)
ROBOT_NAMESPACE = 'your_robot'   # set to your robot_name launch parameter

# Safe resting position the robot returns to between operations [X, Y, Z, W, P, R]
# Units: millimetres for X/Y/Z, degrees for W/P/R
HOME_POSE = [447.854, -6.335, 282, 179.605, 1.089, 1.409]

# Cartesian pose where the die sits on the table [X, Y, Z, W, P, R].
# The robot descends straight down here to pick up the die.
# Add more entries to inspect multiple dice in sequence.
DICE_PICK_POSES = [
    [447.854, -6.335, -122.356, 179.605, 1.089, 1.409],   # Die 1
]

# How high (mm) above each pick pose the robot travels before descending.
# This avoids knocking things over during lateral moves.
PRE_PICK_OFFSET_Z = 80.0

# Where the robot holds the dice steady in front of the camera for inspection.
CAMERA_PRESENT_POSE = [497.894, -938.334, 728, 149.150, 89.3, 58.349]

# MindVision camera device index — 0 = first camera found by CameraEnumerateDevice()
CAMERA_INDEX = 0

# How long (seconds) to wait for the camera service to respond.
# Must be longer than warmup (≤3 s) + grab (2 s) + processing overhead.
CAMERA_TIMEOUT_SEC = 30.0

# How long (seconds) to wait for a robot action to complete before giving up
ROBOT_ACTION_TIMEOUT_SEC = 30.0

# Number of die faces to read per inspection.  Each face after the first
# involves one J6 rotation, a put-down, a HOME transit, and a re-pick.
NUM_READINGS = 3

# Minimum area (px²) the yellow dice contour must have before pip detection runs.
# Raise this if a small yellow piece on the robot arm keeps being mistaken for
# the dice face.  Lower it only if the dice is very far from the camera.
MIN_DICE_AREA_PX = 5000

# Auto-exposure brightness target (0–255).  Default ~120 overexposes yellow dice
# to near-white, collapsing hue and saturation so the detector can't tell dice
# from gripper.  80 keeps the dice as a saturated yellow.
AE_TARGET = 80

# Fraction of the frame height to blank before dice detection.
# Overexposed wood/background at the top and bottom edges can match the yellow
# HSV range; the dice on the black cloth always sits in the central strip.
IGNORE_TOP_FRAC    = .30   # ignore top 15 % of the frame
IGNORE_BOTTOM_FRAC = 0.15   # ignore bottom 15 % of the frame

# OnRobot gripper jaw positions and grip force.
# Adjust GRIPPER_CLOSE_WIDTH to match the size of your dice (standard dice ~16mm).
GRIPPER_OPEN_WIDTH  = 120  # mm  — wide enough to clear the dice when approaching
GRIPPER_CLOSE_WIDTH = 40  # mm  — adjust until gripper holds dice without pushing it out
GRIPPER_FORCE       = 40   # N   — grip force (0–120 N); increase if dice slip


# =============================================================================
#  CAMERA NODE
# =============================================================================

class DiceCameraNode(Node):
    """
    Provides a ROS2 service that the master node calls when it wants a pip count.

    The camera is opened once at startup and kept open so there is no delay
    or warm-up period when a service request arrives.

    Pip detection algorithm:
      - Greyscale → Gaussian blur → colour inversion
      - OpenCV SimpleBlobDetector tuned for small circular pips
      - Pip count returned as a string inside the Trigger response message field
    """

    SERVICE_NAME = '/dice_inspection/read_dice'

    def __init__(self):
        super().__init__('dice_camera_node')

        self.hCamera          = None   # MindVision camera handle
        self.pFrameBuffer     = None   # pre-allocated ISP output buffer (ctypes array)
        self._is_color        = True   # True = BGR output; False = mono (greyscale)
        self.latest_debug_img = None   # most recent annotated frame; read by master on main thread

        # Try to open the camera now; if it fails (e.g. another process still
        # holds it after a crashed run) _init_camera() will be retried once
        # inside _capture_and_count_pips before the first capture.
        self._init_camera()

        # Host the service — master calls this whenever it needs a pip count
        self.srv = self.create_service(Trigger, self.SERVICE_NAME, self._handle_read_request)

        self.get_logger().info(f'Camera node ready — service available at {self.SERVICE_NAME}')

    # -------------------------------------------------------------------------

    def _init_camera(self) -> bool:
        """
        Open the MindVision camera, configure AE, allocate the frame buffer,
        start the stream, and run a short warmup to let AE settle.

        Sets self.hCamera / self.pFrameBuffer on success; leaves them None on
        failure.  Returns True on success, False on failure.

        Safe to call more than once — if the camera is already open this is a
        no-op (returns True immediately).
        """
        if self.hCamera is not None and self.pFrameBuffer is not None:
            return True  # already open

        DevList = mvsdk.CameraEnumerateDevice()
        n_found = len(DevList)
        self.get_logger().info(f'CameraEnumerateDevice found {n_found} device(s).')
        if n_found <= CAMERA_INDEX:
            self.get_logger().error(
                f'No MindVision camera found at index {CAMERA_INDEX} '
                f'({n_found} device(s) detected). '
                'Check USB connection.'
            )
            return False

        h = None
        try:
            h = mvsdk.CameraInit(DevList[CAMERA_INDEX])

            MaxWidth, MaxHeight, bColorCamera = mvsdk.CameraGetCapabilityEx2(h)
            self._is_color = (bColorCamera != 0)

            if self._is_color:
                mvsdk.CameraSetIspOutFormat(h, mvsdk.CAMERA_MEDIA_TYPE_BGR8)
                channels = 3
            else:
                mvsdk.CameraSetIspOutFormat(h, mvsdk.CAMERA_MEDIA_TYPE_MONO8)
                channels = 1

            buf_size = MaxWidth * MaxHeight * channels
            self.pFrameBuffer = (c_ubyte * buf_size)()

            mvsdk.CameraSetAeState(h, True)
            mvsdk.CameraSetAeTarget(h, AE_TARGET)
            mvsdk.CameraPlay(h)

            self.get_logger().info('Priming camera — letting AE settle...')
            for _ in range(15):
                try:
                    raw_p, _ = mvsdk.CameraGetImageBuffer(h, 500)
                    mvsdk.CameraReleaseImageBuffer(h, raw_p)
                except mvsdk.CameraException:
                    pass

            # Only assign to self.hCamera once fully configured — if anything
            # above raised, h is cleaned up in the except block below.
            self.hCamera = h
            self.get_logger().info(
                f'MindVision camera {CAMERA_INDEX} opened '
                f'({"colour" if self._is_color else "mono"}, '
                f'max {MaxWidth}×{MaxHeight})'
            )
            return True

        except mvsdk.CameraException as e:
            self.get_logger().error(
                f'Failed to initialise camera (error {e}). '
                'If the previous run did not shut down cleanly, try: '
                'unplug and replug the USB cable, then restart.'
            )
            # Release the handle if CameraInit succeeded but a later call failed.
            if h is not None:
                try:
                    mvsdk.CameraUnInit(h)
                except mvsdk.CameraException:
                    pass
            self.hCamera      = None
            self.pFrameBuffer = None
            return False

    def destroy_node(self):
        """Clean up the camera before the node is torn down."""
        if self.hCamera is not None:
            try:
                mvsdk.CameraStop(self.hCamera)
                mvsdk.CameraUnInit(self.hCamera)
                self.get_logger().info('Camera closed.')
            except mvsdk.CameraException as e:
                self.get_logger().warn(f'Camera cleanup error (ignored): {e}')
            self.hCamera = None
        super().destroy_node()

    # -------------------------------------------------------------------------

    def _handle_read_request(self, request, response):
        """
        Service callback: called by the master node each time it presents a dice.

        Fills in:
          response.success  — True if a valid pip count was detected, False on error
          response.message  — The pip count as a string (e.g. "4"), or "-1" on error
        """
        self.get_logger().info('Service called — capturing image and counting pips...')
        pip_count = self._capture_and_count_pips()

        response.success = pip_count > 0
        response.message = str(pip_count)   # Master parses int(response.message)
        return response

    # -------------------------------------------------------------------------

    def _capture_and_count_pips(self) -> int:
        """
        Grabs one frame from the MindVision camera and counts the dice pips.

        Capture pipeline:
          1. CameraGetImageBuffer  — fetches a raw sensor frame into a driver-owned buffer.
          2. CameraImageProcess    — runs the ISP (debayer / colour-correction / etc.) and
                                     writes the result into our pre-allocated pFrameBuffer
                                     as BGR8 (or MONO8 for greyscale sensors).
          3. CameraReleaseImageBuffer — returns the driver-owned raw buffer immediately.
          4. numpy view of pFrameBuffer → standard OpenCV BGR frame for pip detection.

        Returns:
            int: Number of pips detected (ideally 1–6).
                 Returns -1 if no camera is open or the frame could not be grabbed.
        """
        if self.hCamera is None or self.pFrameBuffer is None:
            # Startup init may have failed because another process held the
            # camera (e.g. a crashed pip_test.py run).  Try once more now.
            self.get_logger().warn('Camera handle is None — attempting re-initialisation...')
            if not self._init_camera():
                self.get_logger().error(
                    'Camera re-initialisation failed — cannot capture frame. '
                    'Ensure no other process (pip_test.py, hsv_picker.py) is using the camera.'
                )
                return -1

        # Warmup + stall recovery.
        # Attempt up to MAX_RESTART_ATTEMPTS full stop→play→warmup cycles.
        # Each attempt probes 5 frames at 400 ms; if ≥3 fail the stream is
        # considered stalled and we restart before trying again.
        MAX_RESTART_ATTEMPTS = 10
        for attempt in range(MAX_RESTART_ATTEMPTS):
            self.get_logger().info(
                f'Letting auto-exposure settle (attempt {attempt + 1}/{MAX_RESTART_ATTEMPTS})...'
            )
            failures = 0
            for _ in range(5):
                try:
                    raw_w, _ = mvsdk.CameraGetImageBuffer(self.hCamera, 400)
                    mvsdk.CameraReleaseImageBuffer(self.hCamera, raw_w)
                except mvsdk.CameraException:
                    failures += 1

            if failures < 3:
                break   # stream healthy — proceed to real grab

            # Stream stalled — restart and try again
            self.get_logger().warn(
                f'Camera stream stalled (attempt {attempt + 1}), restarting...'
            )
            try:
                mvsdk.CameraStop(self.hCamera)
                time.sleep(0.5 * (attempt + 1))   # back-off: 0.5 s, 1 s, 1.5 s …
                mvsdk.CameraPlay(self.hCamera)
                time.sleep(0.5)
            except mvsdk.CameraException as e:
                self.get_logger().error(f'Stream restart failed: {e}')
        else:
            self.get_logger().error(
                f'Camera stream did not recover after {MAX_RESTART_ATTEMPTS} attempts.'
            )
            return -1

        # Step 1: grab raw frame from the driver (2000 ms timeout)
        try:
            pRawData, FrameHead = mvsdk.CameraGetImageBuffer(self.hCamera, 2000)
        except mvsdk.CameraException as e:
            self.get_logger().error(f'CameraGetImageBuffer failed: {e}')
            return -1

        # Step 2: run ISP — debayer, white-balance, etc. → BGR8 or MONO8 in pFrameBuffer
        mvsdk.CameraImageProcess(
            self.hCamera,
            pRawData,
            addressof(self.pFrameBuffer),   # address of our pre-allocated output buffer
            FrameHead
        )

        # Step 3: release the driver's raw buffer as quickly as possible
        mvsdk.CameraReleaseImageBuffer(self.hCamera, pRawData)

        # Step 4: wrap pFrameBuffer as a numpy array without copying
        channels = 3 if self._is_color else 1
        pixel_count = FrameHead.iWidth * FrameHead.iHeight * channels
        raw_view = (c_ubyte * pixel_count).from_address(addressof(self.pFrameBuffer))
        frame = np.frombuffer(raw_view, dtype=np.uint8).reshape(
            (FrameHead.iHeight, FrameHead.iWidth, channels)
        )

        # Mono sensors deliver a single-channel array; promote to 3-channel BGR so
        # the rest of the pipeline (cvtColor, blob detector, imwrite) is uniform.
        if not self._is_color:
            frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)

        # --- Pre-process the image ---
        gray    = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # --- Isolate the dice face ---
        # HSV range calibrated with hsv_picker.py against the actual dice.
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([ 20,  91, 200])
        upper_yellow = np.array([ 37, 246, 255])
        raw_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # Morphological close fills pip holes and small HSV-range gaps.
        close_kernel = np.ones((21, 21), np.uint8)
        closed_mask  = cv2.morphologyEx(raw_mask, cv2.MORPH_CLOSE, close_kernel)

        # Blank the top and bottom edge strips so overexposed wood/background
        # cannot be mistaken for the dice face.
        h_f       = closed_mask.shape[0]
        top_px    = int(h_f * IGNORE_TOP_FRAC)
        bottom_px = int(h_f * IGNORE_BOTTOM_FRAC)
        closed_mask[:top_px, :]           = 0
        closed_mask[h_f - bottom_px:, :]  = 0

        contours_d, _ = cv2.findContours(closed_mask, cv2.RETR_EXTERNAL,
                                          cv2.CHAIN_APPROX_SIMPLE)

        # Select the LARGEST contour whose minAreaRect aspect ratio is ≥ 0.50.
        # - Elongated floor/table edges are rejected by the aspect filter.
        # - Small noise patches lose to the dice on area.
        # - minAreaRect aspect is rotation-invariant: a square die scores ≈ 1.0
        #   at any angle, unlike the axis-aligned boundingRect.
        img_area     = frame.shape[0] * frame.shape[1]
        dice_contour = None
        best_area    = -1.0
        for cnt in contours_d:
            area = cv2.contourArea(cnt)
            if area < MIN_DICE_AREA_PX or area > img_area * 0.80:
                continue
            rect       = cv2.minAreaRect(cnt)
            rw, rh     = rect[1]
            mar_aspect = min(rw, rh) / max(rw, rh) if max(rw, rh) > 0 else 0
            if mar_aspect < 0.50:
                continue
            if area > best_area:
                best_area    = area
                dice_contour = cnt

        # --- Normalise dice face to a fixed 250×250 canvas, then count pips ---
        WARP_SIZE = 250
        pip_contours_full = []
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

            # Build a solid filled contour mask (no pip holes) and warp it.
            # Using closed_mask directly left a large hole at the "1" pip
            # position because the 21-px close kernel cannot fill a pip that
            # large; the hole then excluded the real pip after erosion.
            dice_filled_mask = np.zeros(closed_mask.shape, dtype=np.uint8)
            cv2.drawContours(dice_filled_mask, [dice_contour], -1, 255, cv2.FILLED)
            warped_face_mask = cv2.warpPerspective(
                dice_filled_mask, M_warp, (WARP_SIZE, WARP_SIZE)
            )

            # Otsu threshold automatically finds the best split between the bright
            # dice face and the dark pips — more stable than adaptive threshold
            # across lighting changes.
            _, pip_binary = cv2.threshold(
                warped_gray, 0, 255,
                cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU
            )

            # Erode the face mask before applying it.
            # A 20 px erosion in the 200×200 space removes the outer ~10% of the
            # face perimeter from the detection zone.  Arm/gripper pixels that
            # bleed into the warp corners appear near the edge of the face mask;
            # eroding it means only dark blobs sitting well inside a large bright
            # yellow region can ever be counted as pips.
            erode_kernel      = np.ones((40, 40), np.uint8)  # ~20 px border exclusion
            face_mask_inner   = cv2.erode(warped_face_mask, erode_kernel)
            pip_binary[face_mask_inner == 0] = 0

            pip_cnts, _ = cv2.findContours(pip_binary, cv2.RETR_EXTERNAL,
                                            cv2.CHAIN_APPROX_SIMPLE)
            for cnt in pip_cnts:
                area = cv2.contourArea(cnt)
                # MAX_AREA 15000: scales with WARP_SIZE=250; the single pip on
                # a "1" die fills ~20% of the 250×250 canvas (~12 500 px²).
                if area < 120 or area > 15000:
                    continue
                perim = cv2.arcLength(cnt, True)
                if perim < 1:
                    continue
                # MIN_CIRC lowered to 0.40: a large circle has more perimeter
                # pixelisation per unit area, so its measured circularity is
                # naturally lower than a small circle of the same shape.
                if 4 * np.pi * area / (perim ** 2) < 0.40:
                    continue
                valid_warped.append(cnt)

            for cnt in valid_warped:
                pts = cnt.reshape(-1, 1, 2).astype(np.float32)
                pip_contours_full.append(
                    cv2.perspectiveTransform(pts, M_unwarp).astype(np.int32)
                )

        pip_count = len(pip_contours_full)

        if pip_count < 1 or pip_count > 6:
            self.get_logger().warn(
                f'Detected {pip_count} pip(s) — outside valid range [1,6]. '
                'Check camera angle, lighting, or tune threshold/area constants.'
            )

        # --- Build annotated debug image ---
        debug_img = frame.copy()

        # Blue outline around the detected dice face
        if dice_contour is not None:
            cv2.drawContours(debug_img, [dice_contour], -1, (255, 100, 0), 2)

        # Red filled contour + cyan number label for each accepted pip
        for i, cnt in enumerate(pip_contours_full):
            cv2.drawContours(debug_img, [cnt], -1, (0, 0, 255), 2)
            M = cv2.moments(cnt)
            if M['m00'] > 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                cv2.putText(debug_img, str(i + 1), (cx + 8, cy + 6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 4)
                cv2.putText(debug_img, str(i + 1), (cx + 8, cy + 6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

        # Total count banner across the top
        cv2.putText(
            debug_img,
            f'Pips detected: {pip_count}',
            (10, 50),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.4,
            (0, 0, 0),      # black outline
            5
        )
        cv2.putText(
            debug_img,
            f'Pips detected: {pip_count}',
            (10, 50),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.4,
            (0, 255, 0),    # green fill
            2
        )

        # Store the annotated frame so the master node can display it from the
        # main thread. cv2.imshow must NOT be called here (service callback runs
        # on the executor thread, which crashes on snap Qt/libpthread environments).
        self.latest_debug_img = debug_img.copy()

        # Save the annotated frame and the pip threshold mask for diagnosis.
        debug_path = f'/tmp/dice_debug_{pip_count}pips.png'
        cv2.imwrite(debug_path, debug_img)
        if pip_binary is not None:
            cv2.imwrite('/tmp/dice_pip_binary.png', pip_binary)
        self.get_logger().info(f'Debug images saved — {debug_path}, /tmp/dice_pip_binary.png')

        return pip_count

    # -------------------------------------------------------------------------

    def destroy_node(self):
        """Stop streaming and close the MindVision camera on node shutdown."""
        if self.hCamera is not None:
            mvsdk.CameraStop(self.hCamera)
            mvsdk.CameraUnInit(self.hCamera)
            self.hCamera = None
        super().destroy_node()


# =============================================================================
#  MASTER / CONTROL NODE
# =============================================================================

class DiceInspectionMasterNode(Node):
    """
    Orchestrates the full three-dice inspection sequence.

    For each dice (1 → 3):
      1.  Approach above the dice (travel height)
      2.  Open gripper
      3.  Descend to the dice
      4.  Close gripper  (pick up)
      5.  Lift back to travel height
      6.  Move to camera presentation pose
      7.  Call camera service → receive pip count
      8.  Record dice number → pip count
      9.  Return dice to its original location and release gripper
      10. Return to HOME_POSE

    After all three dice, print a results summary.
    """

    SERVICE_NAME = '/dice_inspection/read_dice'

    def __init__(self, camera_node: 'DiceCameraNode'):
        super().__init__('dice_inspection_master')

        # Reference to the camera node so the main thread can display debug frames
        self._camera_node = camera_node

        # --- Action clients (connect to existing Fanuc action servers) ---
        self.cart_ac    = ActionClient(self, CartPose,       f'/{ROBOT_NAMESPACE}/cartesian_pose')
        self.gripper_ac = ActionClient(self, OnRobotGripper, f'/{ROBOT_NAMESPACE}/onrobot_gripper')
        self.sjoint_ac  = ActionClient(self, SJointPose,     f'/{ROBOT_NAMESPACE}/single_joint_pose')

        # --- Camera service client ---
        self.camera_client = self.create_client(Trigger, self.SERVICE_NAME)

        # --- is_moving subscriber ---
        self._robot_is_moving = False
        self.create_subscription(
            IsMoving,
            f'/{ROBOT_NAMESPACE}/is_moving',
            self._on_is_moving,
            10
        )

        # --- cur_joints subscriber — used to read actual J6 before rotating ---
        self._current_joints = [0.0] * 6
        self.create_subscription(
            CurJoints,
            f'/{ROBOT_NAMESPACE}/cur_joints',
            self._on_cur_joints,
            10
        )

        # Final results: {dice_number (1-3): pip_count}
        self.dice_results: dict[int, int] = {}

        self.get_logger().info('Master node initialised.')

    # -------------------------------------------------------------------------
    #  Camera communication
    # -------------------------------------------------------------------------

    def _request_pip_count(self) -> int:
        """
        Call the camera service and block until a pip count is returned or
        the timeout expires.

        The call is sent asynchronously and a threading.Event is used to block
        the master's sequence thread without stalling the ROS executor thread
        that is processing callbacks in the background.

        Returns:
            int: Pip count from the camera node, or -1 on failure/timeout.
        """
        # Block until the camera service is available
        self.camera_client.wait_for_service()

        done_event   = threading.Event()
        result_holder = [-1]   # List so the nested callback can write to it

        def on_response(future):
            """Receives the service response and unblocks the waiting thread."""
            resp = future.result()
            if resp.success:
                result_holder[0] = int(resp.message)
            else:
                self.get_logger().warn('Camera service returned success=False.')
                result_holder[0] = -1
            done_event.set()

        # Send the service request asynchronously
        future = self.camera_client.call_async(Trigger.Request())
        future.add_done_callback(on_response)
        self.get_logger().info('Camera service called — waiting for pip count...')

        # Wait for on_response to fire; the executor is running on another thread
        got_result = done_event.wait(timeout=CAMERA_TIMEOUT_SEC)
        if not got_result:
            self.get_logger().error('Camera service timed out!')
            return -1

        return result_holder[0]

    # -------------------------------------------------------------------------
    #  Robot movement tracking
    # -------------------------------------------------------------------------

    def _on_is_moving(self, msg: IsMoving):
        self._robot_is_moving = msg.moving

    def _on_cur_joints(self, msg: CurJoints):
        self._current_joints = list(msg.joints)

    def _wait_until_stopped(self, timeout: float = ROBOT_ACTION_TIMEOUT_SEC):
        """
        Block until the robot has finished its current move.

        Two-phase approach:

        Phase 1 — wait for the robot to START moving (up to START_TIMEOUT).
          The action server has a race condition and often calls succeed() before
          the EthernetIP motion command takes effect.  Polling here ensures we
          don't declare the move done before the arm has left its current pose.
          If we never see is_moving=True within START_TIMEOUT the move was either
          very short (completed within one 0.5 s topic cycle) or already done —
          either way it is safe to fall through to Phase 2.

        Phase 2 — wait for the robot to STOP moving (up to timeout).
          Once moving is confirmed (or Phase 1 times out), we poll until
          is_moving=False before returning, so the next command is only sent
          after the arm has physically arrived at its target pose.
        """
        poll = 0.1
        START_TIMEOUT = 3.0   # seconds to wait for is_moving to go True

        # Phase 1: wait for motion to begin
        t0 = time.monotonic()
        while time.monotonic() - t0 < START_TIMEOUT:
            if self._robot_is_moving:
                break
            time.sleep(poll)

        # Phase 2: wait for motion to finish
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            if not self._robot_is_moving:
                return
            time.sleep(poll)

        self.get_logger().error('Timed out waiting for robot to stop moving!')

    # -------------------------------------------------------------------------
    #  Robot movement helpers
    # -------------------------------------------------------------------------

    def _send_action_and_wait(self, action_client: ActionClient, goal, timeout: float) -> bool:
        """
        Send an action goal and block until it completes or times out.

        Uses threading.Event so this function can block the caller without
        stalling the ROS executor thread that processes action server callbacks.

        Args:
            action_client: The rclpy ActionClient to use.
            goal:          The populated Goal message.
            timeout:       Maximum seconds to wait before giving up.

        Returns:
            bool: True if the action server reported success, False otherwise.
        """
        done_event    = threading.Event()
        result_holder = [False]

        def on_goal_response(future):
            """Step 1 — server accepted or rejected our goal."""
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error('Action goal was rejected by the server.')
                done_event.set()
                return
            # Goal accepted: register a callback for when execution finishes
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(on_result)

        def on_result(future):
            """Step 2 — action finished; grab the success flag and unblock."""
            result_holder[0] = future.result().result.success
            done_event.set()

        action_client.wait_for_server()
        send_future = action_client.send_goal_async(goal)
        send_future.add_done_callback(on_goal_response)

        timed_out = not done_event.wait(timeout=timeout)
        if timed_out:
            self.get_logger().error('Action timed out waiting for completion!')
            return False

        return result_holder[0]

    def _move_to_pose(self, pose: list) -> bool:
        """
        Move the robot end-effector to a Cartesian pose and block until done.

        Args:
            pose: [X, Y, Z] or [X, Y, Z, W, P, R] in mm / degrees.
                  W, P, R default to 200.0 (keep current orientation) when omitted.

        Returns:
            bool: True on success.
        """
        goal = CartPose.Goal()
        goal.x = float(pose[0])
        goal.y = float(pose[1])
        goal.z = float(pose[2])
        if len(pose) == 6:
            goal.w = float(pose[3])
            goal.p = float(pose[4])
            goal.r = float(pose[5])

        self.get_logger().info(f'Moving to pose: {pose}')
        success = self._send_action_and_wait(self.cart_ac, goal, ROBOT_ACTION_TIMEOUT_SEC)

        # The action server has a race condition and often returns before the robot
        # moves. Wait here until the is_moving topic confirms the robot has actually
        # started and finished its move before sending the next command.
        self._wait_until_stopped()

        if not success:
            self.get_logger().error(f'Failed to reach pose: {pose}')
        return success

    def _set_gripper(self, command: str) -> bool:
        """
        Open or close the OnRobot gripper and block until done.

        The OnRobot action uses jaw width (mm) and force (N) rather than an
        open/close string, so 'open' and 'close' are translated to the
        GRIPPER_OPEN_WIDTH / GRIPPER_CLOSE_WIDTH constants defined at the top.

        Args:
            command: 'open' or 'close'

        Returns:
            bool: True on success.
        """
        goal = OnRobotGripper.Goal()

        if command == 'open':
            goal.width = GRIPPER_OPEN_WIDTH
            goal.force = GRIPPER_FORCE
        else:  # 'close'
            goal.width = GRIPPER_CLOSE_WIDTH
            goal.force = GRIPPER_FORCE

        self.get_logger().info(f'Gripper: {command} (width={goal.width}mm, force={goal.force}N)')
        return self._send_action_and_wait(self.gripper_ac, goal, ROBOT_ACTION_TIMEOUT_SEC)

    def _rotate_joint6(self, angle_deg: float) -> bool:
        """
        Rotate joint 6 to an absolute angle (degrees) and wait until done.

        Wraps the angle into [-179, 179] so that e.g. 270° becomes -90°,
        keeping the value within the action server's accepted range.
        """
        # Wrap to [-180, 180) then clamp to ±179 (server rejects ≥ 179.9)
        wrapped = ((angle_deg + 180.0) % 360.0) - 180.0
        wrapped = max(-179.0, min(179.0, wrapped))

        goal = SJointPose.Goal()
        goal.joint = 6
        goal.angle = float(wrapped)
        self.get_logger().info(f'Rotating joint 6 → {wrapped:.1f}°')
        success = self._send_action_and_wait(self.sjoint_ac, goal, ROBOT_ACTION_TIMEOUT_SEC)
        self._wait_until_stopped()
        if not success:
            self.get_logger().error(f'Joint 6 rotation to {wrapped:.1f}° failed.')
        return success

    # -------------------------------------------------------------------------
    #  Main inspection sequence
    # -------------------------------------------------------------------------

    def run_inspection(self):
        """
        Top-level entry point. Inspects all 3 dice in order, then prints results.
        Called once from main() after the executor has started spinning.
        """
        self.get_logger().info('=== Dice Inspection Sequence START ===')

        # Failsafe: ensure the gripper starts open regardless of its last state
        self.get_logger().info('Failsafe: opening gripper before sequence begins...')
        self._set_gripper('open')

        input('\nPlace the die at its pick location, then press Enter...')

        for dice_index in range(len(DICE_PICK_POSES)):
            dice_number = dice_index + 1
            self.get_logger().info(f'--- Die {dice_number} / {len(DICE_PICK_POSES)} ---')
            self._inspect_one_dice(dice_number, DICE_PICK_POSES[dice_index])
            # _inspect_one_dice always ends at HOME_POSE (success or error)

        self._print_summary()

        # _inspect_one_dice always ends at HOME_POSE (dice returned, gripper open),
        # so no extra return/home step is needed here.
        self.get_logger().info('=== Dice Inspection Sequence COMPLETE ===')

    def _inspect_one_dice(self, dice_number: int, pick_pose: list):
        """
        Pick → face 1 → rotate J6 (while holding) → place rotated → pick again → face 2 → return → home.

        Sequence:
          For each of NUM_READINGS faces:
            Move to camera → read pip count → show debug image
            If not the last face:
              Return to pre-pick (still holding) → rotate J6 +90° →
              place with rotated_pick_pose → lift → HOME → re-pick at pick_pose
          Return die → home.

        Each rotation accumulates on the table: the die is placed 90° further
        each time, so successive picks always present a new face to the camera.
        """
        pre_pick_pose = list(pick_pose)
        pre_pick_pose[2] += PRE_PICK_OFFSET_Z

        # Placing pose (R+90°) used for every rotate-and-place step.
        r_rot = ((pick_pose[5] + 90.0 + 180.0) % 360.0) - 180.0
        rotated_pick_pose     = list(pick_pose);          rotated_pick_pose[5]     = r_rot
        rotated_pre_pick_pose = list(rotated_pick_pose);  rotated_pre_pick_pose[2] += PRE_PICK_OFFSET_Z

        error_result = tuple(-1 for _ in range(NUM_READINGS))

        # =================================================================
        #  INITIAL PICK UP
        # =================================================================
        self.get_logger().info('Approach above die...')
        if not self._move_to_pose(pre_pick_pose):
            self.get_logger().error(f'Die {dice_number}: cannot reach approach pose — aborting.')
            self._move_to_pose(HOME_POSE)
            self.dice_results[dice_number] = error_result
            return

        self._set_gripper('open')

        self.get_logger().info('Descend to pick pose...')
        if not self._move_to_pose(pick_pose):
            self.get_logger().error(f'Die {dice_number}: cannot reach pick pose — aborting.')
            self._move_to_pose(HOME_POSE)
            self.dice_results[dice_number] = error_result
            return

        self._set_gripper('close')
        self._move_to_pose(pre_pick_pose)

        # =================================================================
        #  READING LOOP
        # =================================================================
        pip_counts = []

        for face_num in range(1, NUM_READINGS + 1):
            self.get_logger().info(f'Move to camera (face {face_num})...')
            if not self._move_to_pose(CAMERA_PRESENT_POSE):
                self.get_logger().error(
                    f'Die {dice_number}: cannot reach camera for face {face_num}.'
                )
                self._return_dice(pre_pick_pose, pick_pose)
                self._move_to_pose(HOME_POSE)
                # Pad missing readings with -1
                while len(pip_counts) < NUM_READINGS:
                    pip_counts.append(-1)
                self.dice_results[dice_number] = tuple(pip_counts)
                return

            self.get_logger().info(f'Read face {face_num}...')
            pip_count = self._request_pip_count()
            pip_counts.append(pip_count)
            self.get_logger().info(f'Die {dice_number} face {face_num}: {pip_count} pip(s)')

            img = self._camera_node.latest_debug_img
            if img is not None:
                win = f'Die {dice_number} face {face_num} — {pip_count} pip(s)  [press any key]'
                cv2.namedWindow(win, cv2.WINDOW_NORMAL)
                cv2.resizeWindow(win, 1280, 960)
                cv2.imshow(win, img)
                cv2.waitKey(0)
                cv2.destroyAllWindows()

            if face_num == NUM_READINGS:
                break  # last face — skip the rotate-and-re-pick

            # ----------------------------------------------------------
            #  Rotate J6 while holding, place die, reset to HOME, re-pick
            # ----------------------------------------------------------
            self.get_logger().info(f'Return to pre-pick (holding, face {face_num} done)...')
            self._move_to_pose(pre_pick_pose)

            j6_now = self._current_joints[5]
            self.get_logger().info(
                f'Rotate J6 {j6_now:.1f}° → {j6_now + 90.0:.1f}° (holding die)...'
            )
            self._rotate_joint6(j6_now + 90.0)

            self.get_logger().info('Place die (rotated)...')
            self._move_to_pose(rotated_pick_pose)
            self._set_gripper('open')
            self._move_to_pose(rotated_pre_pick_pose)

            self.get_logger().info('HOME to reset joint config...')
            self._move_to_pose(HOME_POSE)

            self.get_logger().info('Re-pick die (new orientation)...')
            self._move_to_pose(pre_pick_pose)
            self._set_gripper('open')
            self._move_to_pose(pick_pose)
            self._set_gripper('close')
            self._move_to_pose(pre_pick_pose)

        # =================================================================
        #  RETURN DIE AND GO HOME
        # =================================================================
        self.get_logger().info('Return die to pick location...')
        self._return_dice(pre_pick_pose, pick_pose)

        self.dice_results[dice_number] = tuple(pip_counts)

        self.get_logger().info('Return to HOME_POSE...')
        self._move_to_pose(HOME_POSE)

    def _return_dice(self, pre_pick_pose: list, pick_pose: list):
        """
        Place the dice back on the table and release it.
        Also used as a safe fallback when an error occurs mid-sequence.
        """
        self._move_to_pose(pre_pick_pose)   # Travel to above the drop spot
        self._move_to_pose(pick_pose)        # Lower to table level
        self._set_gripper('open')            # Release the dice
        self._move_to_pose(pre_pick_pose)   # Lift clear so the next move is safe

    # -------------------------------------------------------------------------
    #  Results reporting
    # -------------------------------------------------------------------------

    def _print_summary(self):
        """Print a formatted summary table of the die and all face pip counts."""
        face_headers = '   '.join(
            f'Face {i+1}({i*90}deg)' for i in range(NUM_READINGS)
        )
        lines = [
            '========================================',
            '       DICE INSPECTION SUMMARY          ',
            '========================================',
            f'  Die    {face_headers}',
            '----------------------------------------',
        ]
        total = 0
        for dice_num in sorted(self.dice_results):
            counts = self.dice_results[dice_num]
            face_strs = [f'{p} pip(s)' if p > 0 else 'ERROR' for p in counts]
            lines.append(f'  Die {dice_num}:  ' + '   '.join(f'{s:<12}' for s in face_strs))
            total += sum(p for p in counts if p > 0)
        lines.append('----------------------------------------')
        lines.append(f'  TOTAL PIPS: {total}')
        lines.append('========================================')

        for line in lines:
            print(line)
            self.get_logger().info(line)


# =============================================================================
#  ENTRY POINT
# =============================================================================

def main(args=None):
    """
    Launch both nodes in one process.

    The MultiThreadedExecutor spins in a background thread, keeping all ROS
    callbacks (action results, service responses) alive. The master's sequential
    run_inspection() runs on the main thread and uses threading.Event to block
    between steps without starving the executor.
    """
    rclpy.init(args=args)

    camera_node = DiceCameraNode()
    master_node = DiceInspectionMasterNode(camera_node)

    # MultiThreadedExecutor lets both nodes handle callbacks concurrently
    executor = MultiThreadedExecutor()
    executor.add_node(camera_node)
    executor.add_node(master_node)

    # Spin in a background daemon thread so main() stays free for the sequence
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    # wait_for_server() and wait_for_service() inside the helper methods will
    # block until each server is up — no sleep needed here.
    try:
        master_node.run_inspection()
    except KeyboardInterrupt:
        print('\nInterrupted by user.')
    finally:
        # Signal executor to stop, then wait for the spin thread to fully exit
        # before destroying nodes. Destroying a node while the executor is still
        # dispatching callbacks into it causes a segfault.
        executor.shutdown()
        spin_thread.join(timeout=5.0)
        camera_node.destroy_node()
        master_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
