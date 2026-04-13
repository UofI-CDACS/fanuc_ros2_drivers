#!/usr/bin/env python3
"""
dice_inspection_system.py

All-in-one dice inspection script for FANUC robots.
Runs all nodes in a single process using a MultiThreadedExecutor:

  OrchestratorNode    — state machine for the 3-dice pipeline
  RobotControlNode    — drives the FANUC through the pick-inspect-drop cycle
  CameraNode          — MindVision camera; provides /capture_image service
  PipCounterNode      — HSV + HoughCircles pip detection; provides /count_pips service
  DisplayNode         — real-time OpenCV window
  ResultsLoggerNode   — console pretty-printer

External dependencies (must be built/sourced separately):
  - fanuc_interfaces  (already in this repo)
  - dice_interfaces   (CMake package — msg/srv/action definitions for the dice system)
  - mvsdk.py          (MindVision SDK; place alongside this script or on PYTHONPATH)

Usage:
  python3 dice_inspection_system.py \
      --ros-args -p robot_name:=fanuc -p robot_ip:=172.29.208.1

  Or add it to a launch file as a Node() with parameters.
"""

import json
import os
import sys
import threading
import time
from datetime import datetime

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image

from dice_interfaces.action import InspectDice
from dice_interfaces.msg import DiceResult, InspectionSummary
from dice_interfaces.srv import CaptureImage, CountPips
from fanuc_interfaces.action import CartPose, JointPose, SchunkGripper

# mvsdk.py must be in the same directory as this script or on PYTHONPATH
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mvsdk  # noqa: E402  # type: ignore[import-not-found]

# ---------------------------------------------------------------------------
# OrchestratorNode
# ---------------------------------------------------------------------------

NUM_DICE = 3


class OrchestratorNode(Node):
    """Top-level state machine that runs the 3-dice inspection pipeline.

    For each die (1..3):
      1. Sends InspectDice action goal to RobotControlNode
      2. Waits for 'at_camera' feedback
      3. Calls /capture_image service
      4. Calls /count_pips service
      5. Records result and waits for robot to return home

    After all dice:
      6. Publishes InspectionSummary
      7. Writes JSON results file

    Publishes:
      /dice_result          (dice_interfaces/msg/DiceResult)
      /inspection_summary   (dice_interfaces/msg/InspectionSummary)
    """

    def __init__(self):
        super().__init__('orchestrator_node')

        self.declare_parameter('results_dir', '/tmp/dice_results')
        self.declare_parameter('num_dice', NUM_DICE)

        self._cb_group = ReentrantCallbackGroup()

        self._result_pub = self.create_publisher(DiceResult, 'dice_result', 10)
        self._summary_pub = self.create_publisher(
            InspectionSummary, 'inspection_summary', 10
        )

        self._robot_client = ActionClient(
            self, InspectDice, 'inspect_dice',
            callback_group=self._cb_group,
        )
        self._capture_client = self.create_client(
            CaptureImage, 'capture_image',
            callback_group=self._cb_group,
        )
        self._count_client = self.create_client(
            CountPips, 'count_pips',
            callback_group=self._cb_group,
        )

        self._at_camera_event = threading.Event()

        # _wait_for_dependencies() is called inside the pipeline thread so
        # __init__ returns immediately and the executor can start before we
        # block on wait_for_server / wait_for_service.
        threading.Thread(target=self._run_pipeline, daemon=True).start()
        self.get_logger().info('orchestrator_node started — pipeline thread launched')

    # ------------------------------------------------------------------

    def _wait_for_dependencies(self):
        self.get_logger().info('Waiting for /inspect_dice action server…')
        self._robot_client.wait_for_server()
        self.get_logger().info('Waiting for /capture_image service…')
        self._capture_client.wait_for_service()
        self.get_logger().info('Waiting for /count_pips service…')
        self._count_client.wait_for_service()
        self.get_logger().info('All dependencies available.')

    def _run_pipeline(self):
        self._wait_for_dependencies()

        num_dice = self.get_parameter('num_dice').value
        results = []

        for dice_num in range(1, num_dice + 1):
            self.get_logger().info(f'═══ Starting dice {dice_num}/{num_dice} ═══')
            dice_result = self._inspect_one_dice(dice_num)
            results.append(dice_result)

            self._result_pub.publish(dice_result)
            self.get_logger().info(
                f'Dice {dice_num}: {dice_result.pip_count} pip(s) '
                f'(success={dice_result.success})'
            )

        summary = self._build_summary(results)
        self._summary_pub.publish(summary)
        self.get_logger().info(
            f'Inspection complete — total pips: {summary.total_pips}'
        )
        self._save_results(results, summary)

    def _inspect_one_dice(self, dice_number: int) -> DiceResult:
        """Full single-die cycle. Returns a DiceResult."""
        result_msg = DiceResult()
        result_msg.dice_number = dice_number
        result_msg.timestamp = self.get_clock().now().to_msg()

        # ── Step 1: send robot goal ─────────────────────────────────────
        self._at_camera_event.clear()

        goal = InspectDice.Goal()
        goal.dice_number = dice_number

        goal_response_event = threading.Event()
        goal_handle_box = [None]

        def _goal_response_cb(future):
            goal_handle_box[0] = future.result()
            goal_response_event.set()

        send_future = self._robot_client.send_goal_async(
            goal,
            feedback_callback=self._robot_feedback_cb,
        )
        send_future.add_done_callback(_goal_response_cb)

        if not goal_response_event.wait(timeout=10.0):
            result_msg.success = False
            result_msg.notes = 'Timeout waiting for robot goal response'
            return result_msg

        goal_handle = goal_handle_box[0]
        if not goal_handle.accepted:
            result_msg.success = False
            result_msg.notes = 'Robot goal rejected'
            return result_msg

        # ── Step 2: wait until robot is at camera position ──────────────
        self.get_logger().info('Waiting for robot to reach camera position…')
        if not self._at_camera_event.wait(timeout=60.0):
            goal_handle.cancel_goal_async()
            result_msg.success = False
            result_msg.notes = 'Timeout waiting for camera position'
            return result_msg

        # ── Step 3: capture image ───────────────────────────────────────
        self.get_logger().info('Capturing image…')
        cap_req = CaptureImage.Request()
        cap_resp = self._capture_client.call(cap_req)

        if not cap_resp.success:
            goal_handle.cancel_goal_async()
            result_msg.success = False
            result_msg.notes = f'Image capture failed: {cap_resp.message}'
            return result_msg

        # ── Step 4: count pips ──────────────────────────────────────────
        self.get_logger().info('Counting pips…')
        count_req = CountPips.Request()
        count_req.image = cap_resp.image
        count_resp = self._count_client.call(count_req)

        result_msg.pip_count = count_resp.pip_count if count_resp.success else -1
        result_msg.success = count_resp.success
        result_msg.notes = count_resp.message

        # ── Step 5: wait for robot to finish returning home ─────────────
        result_done_event = threading.Event()
        result_box = [None]

        def _result_cb(future):
            result_box[0] = future.result()
            result_done_event.set()

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(_result_cb)

        if not result_done_event.wait(timeout=60.0):
            result_msg.notes += ' | Robot did not return home (timeout)'
        elif result_box[0] is None:
            result_msg.notes += ' | Robot did not return home cleanly'

        return result_msg

    def _robot_feedback_cb(self, feedback_msg):
        phase = feedback_msg.feedback.phase
        self.get_logger().info(f'Robot phase: {phase}')
        if phase == 'at_camera':
            self._at_camera_event.set()

    @staticmethod
    def _build_summary(results: list) -> InspectionSummary:
        summary = InspectionSummary()
        summary.results = results
        summary.total_pips = sum(r.pip_count for r in results if r.pip_count > 0)
        summary.complete = all(r.success for r in results)
        return summary

    def _save_results(self, results: list, summary: InspectionSummary):
        results_dir = self.get_parameter('results_dir').value
        os.makedirs(results_dir, exist_ok=True)
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        filepath = os.path.join(results_dir, f'inspection_{ts}.json')

        data = {
            'timestamp': ts,
            'total_pips': summary.total_pips,
            'complete': summary.complete,
            'dice': [
                {
                    'dice_number': r.dice_number,
                    'pip_count': r.pip_count,
                    'success': r.success,
                    'notes': r.notes,
                }
                for r in results
            ],
        }
        with open(filepath, 'w') as f:
            json.dump(data, f, indent=2)
        self.get_logger().info(f'Results saved to {filepath}')


# ---------------------------------------------------------------------------
# RobotControlNode
# ---------------------------------------------------------------------------


class RobotControlNode(Node):
    """Drives the FANUC robot through one dice-inspection cycle per action goal.

    10-step cycle:
      1.  HOME (joint)
      2.  PICK APPROACH (cart)
      3.  Gripper OPEN
      4.  PICK (cart)
      5.  Gripper CLOSE
      6.  CAMERA (joint)  → publishes 'at_camera' feedback
      7.  DROP (cart)
      8.  Gripper OPEN
      9.  MOVE_UP (cart)
      10. HOME (joint)

    Action server : /inspect_dice       (dice_interfaces/action/InspectDice)
    Action clients: /<robot>/cartesian_pose  (fanuc_interfaces/action/CartPose)
                    /<robot>/joint_pose      (fanuc_interfaces/action/JointPose)
                    /<robot>/schunk_gripper  (fanuc_interfaces/action/SchunkGripper)
    """

    def __init__(self):
        super().__init__('robot_control_node')

        self.declare_parameter('robot_name', 'fanuc')
        self.declare_parameter('move_speed', 30)

        # Joint poses  [J1..J6]  degrees
        self.declare_parameter('pose_home',   [0.0] * 6)
        self.declare_parameter('pose_camera', [0.0] * 6)

        # Cartesian poses  [X, Y, Z, W, P, R]
        self.declare_parameter('pose_pick_approach', [0.0] * 6)
        self.declare_parameter('pose_pick',          [0.0] * 6)
        self.declare_parameter('pose_drop',          [0.0] * 6)
        self.declare_parameter('pose_move_up',       [0.0] * 6)

        self._cb_group = ReentrantCallbackGroup()
        robot = self.get_parameter('robot_name').value

        self._cart_client = ActionClient(
            self, CartPose, f'/{robot}/cartesian_pose',
            callback_group=self._cb_group)
        self._joint_client = ActionClient(
            self, JointPose, f'{robot}/joint_pose',
            callback_group=self._cb_group)
        self._gripper_client = ActionClient(
            self, SchunkGripper, f'{robot}/schunk_gripper',
            callback_group=self._cb_group)

        self._action_server = ActionServer(
            self,
            InspectDice,
            'inspect_dice',
            execute_callback=self._execute,
            goal_callback=lambda _: GoalResponse.ACCEPT,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=self._cb_group,
        )
        self.get_logger().info('robot_control_node ready')

    # ------------------------------------------------------------------

    async def _execute(self, goal_handle):
        dice_number = goal_handle.request.dice_number
        self.get_logger().info(f'Inspection cycle start — dice #{dice_number}')

        feedback = InspectDice.Feedback()
        result = InspectDice.Result()
        result.result.dice_number = dice_number
        result.result.success = False

        def cancelled():
            result.result.notes = 'Cancelled'
            goal_handle.canceled()

        def failed(phase):
            result.result.notes = f'Failed during: {phase}'
            self.get_logger().error(f'Inspection aborted: {phase}')
            goal_handle.abort()

        def fb(phase, progress):
            feedback.phase = phase
            feedback.progress = progress
            goal_handle.publish_feedback(feedback)

        # ── 1. Home ──────────────────────────────────────────────────────
        fb('homing', 0.0)
        if goal_handle.is_cancel_requested:
            cancelled()
            return result
        if not await self._joint(self.get_parameter('pose_home').value):
            failed('homing')
            return result

        # ── 2. Pick approach ─────────────────────────────────────────────
        fb('pick_approach', 0.1)
        if goal_handle.is_cancel_requested:
            cancelled()
            return result
        if not await self._cart(self.get_parameter('pose_pick_approach').value):
            failed('pick_approach')
            return result

        # ── 3. Gripper open ──────────────────────────────────────────────
        fb('gripper_open', 0.2)
        if not await self._gripper('open'):
            failed('gripper_open')
            return result

        # ── 4. Pick ──────────────────────────────────────────────────────
        fb('picking', 0.3)
        if goal_handle.is_cancel_requested:
            cancelled()
            return result
        if not await self._cart(self.get_parameter('pose_pick').value):
            failed('picking')
            return result

        # ── 5. Gripper close ─────────────────────────────────────────────
        fb('gripper_close', 0.4)
        if not await self._gripper('close'):
            failed('gripper_close')
            return result

        # ── 6. Camera pose ───────────────────────────────────────────────
        fb('moving_to_camera', 0.5)
        if goal_handle.is_cancel_requested:
            cancelled()
            return result
        if not await self._joint(self.get_parameter('pose_camera').value):
            failed('moving_to_camera')
            return result

        fb('at_camera', 0.6)  # signals orchestrator to capture image

        # ── 7. Drop ──────────────────────────────────────────────────────
        fb('dropping', 0.7)
        if goal_handle.is_cancel_requested:
            cancelled()
            return result
        if not await self._cart(self.get_parameter('pose_drop').value):
            failed('dropping')
            return result

        # ── 8. Gripper open ──────────────────────────────────────────────
        fb('gripper_open_drop', 0.8)
        if not await self._gripper('open'):
            failed('gripper_open_drop')
            return result

        # ── 9. Move up ───────────────────────────────────────────────────
        fb('move_up', 0.9)
        if not await self._cart(self.get_parameter('pose_move_up').value):
            failed('move_up')
            return result

        # ── 10. Home ─────────────────────────────────────────────────────
        fb('homing', 0.95)
        if not await self._joint(self.get_parameter('pose_home').value):
            failed('final_home')
            return result

        result.result.success = True
        goal_handle.succeed()
        return result

    async def _joint(self, pose: list) -> bool:
        self.get_logger().debug('Waiting for joint_pose server…')
        if not self._joint_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('JointPose action server not available after 30s')
            return False

        goal = JointPose.Goal()
        goal.joint1 = float(pose[0])
        goal.joint2 = float(pose[1])
        goal.joint3 = float(pose[2])
        goal.joint4 = float(pose[3])
        goal.joint5 = float(pose[4])
        goal.joint6 = float(pose[5])

        fh = await self._joint_client.send_goal_async(goal)
        if not fh.accepted:
            self.get_logger().error('JointPose goal rejected')
            return False
        res = await fh.get_result_async()
        return res.result.success

    async def _cart(self, pose: list) -> bool:
        if not self._cart_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('CartPose action server not available')
            return False

        goal = CartPose.Goal()
        goal.x = float(pose[0])
        goal.y = float(pose[1])
        goal.z = float(pose[2])
        goal.w = float(pose[3])
        goal.p = float(pose[4])
        goal.r = float(pose[5])

        fh = await self._cart_client.send_goal_async(goal)
        if not fh.accepted:
            self.get_logger().error('CartPose goal rejected')
            return False
        res = await fh.get_result_async()
        return res.result.success

    async def _gripper(self, command: str) -> bool:
        if not self._gripper_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('SchunkGripper action server not available')
            return False

        goal = SchunkGripper.Goal()
        goal.command = command

        fh = await self._gripper_client.send_goal_async(goal)
        if not fh.accepted:
            self.get_logger().error(f'SchunkGripper goal rejected ({command})')
            return False
        res = await fh.get_result_async()
        return res.result.success

    def destroy_node(self):
        self._action_server.destroy()
        super().destroy_node()


# ---------------------------------------------------------------------------
# CameraNode
# ---------------------------------------------------------------------------


class CameraNode(Node):
    """Wraps a MindVision industrial camera.

    Publishes a live 10 Hz stream to /camera/image_raw and exposes a
    /capture_image service that returns a single high-quality frame.

    Service: /capture_image  (dice_interfaces/srv/CaptureImage)
    """

    def __init__(self):
        super().__init__('camera_node')

        self.declare_parameter('device_index', 0)
        self.declare_parameter('exposure_ms', 30.0)

        self._bridge = CvBridge()
        self._hCamera = 0
        self._pFrameBuffer = None
        self._monoCamera = False
        self._open_camera()

        self._image_pub = self.create_publisher(Image, 'camera/image_raw', 10)
        self.create_timer(0.1, self._publish_frame)  # 10 Hz live stream

        self._srv = self.create_service(
            CaptureImage,
            'capture_image',
            self._handle_capture,
        )
        self.get_logger().info('camera_node ready — /capture_image service active')

    # ------------------------------------------------------------------

    def _open_camera(self):
        idx = self.get_parameter('device_index').value
        exposure_ms = self.get_parameter('exposure_ms').value

        DevList = mvsdk.CameraEnumerateDevice()
        nDev = len(DevList)
        if nDev < 1:
            self.get_logger().error('No MindVision camera found')
            return

        if idx >= nDev:
            self.get_logger().warn(
                f'device_index {idx} out of range ({nDev} found), falling back to 0')
            idx = 0

        DevInfo = DevList[idx]
        self.get_logger().info(
            f'Opening camera [{idx}]: {DevInfo.GetFriendlyName()} '
            f'({DevInfo.GetPortType()})')

        try:
            self._hCamera = mvsdk.CameraInit(DevInfo, -1, -1)
        except mvsdk.CameraException as e:
            self.get_logger().error(
                f'CameraInit failed ({e.error_code}): {e.message}')
            return

        cap = mvsdk.CameraGetCapability(self._hCamera)
        self._monoCamera = (cap.sIspCapacity.bMonoSensor != 0)

        if self._monoCamera:
            mvsdk.CameraSetIspOutFormat(
                self._hCamera, mvsdk.CAMERA_MEDIA_TYPE_MONO8)
        else:
            mvsdk.CameraSetIspOutFormat(
                self._hCamera, mvsdk.CAMERA_MEDIA_TYPE_BGR8)

        mvsdk.CameraSetTriggerMode(self._hCamera, 0)          # continuous
        mvsdk.CameraSetAeState(self._hCamera, 0)               # manual exposure
        mvsdk.CameraSetExposureTime(
            self._hCamera, int(exposure_ms * 1000))

        mvsdk.CameraPlay(self._hCamera)

        channels = 1 if self._monoCamera else 3
        buf_size = (cap.sResolutionRange.iWidthMax *
                    cap.sResolutionRange.iHeightMax * channels)
        self._pFrameBuffer = mvsdk.CameraAlignMalloc(buf_size, 16)
        self.get_logger().info('MindVision camera opened successfully')

    def _grab_frame(self, timeout_ms: int = 100):
        """Grab one frame from the camera. Returns numpy array or None."""
        pRawData, FrameHead = mvsdk.CameraGetImageBuffer(
            self._hCamera, timeout_ms)
        mvsdk.CameraImageProcess(
            self._hCamera, pRawData, self._pFrameBuffer, FrameHead)
        mvsdk.CameraReleaseImageBuffer(self._hCamera, pRawData)

        frame_data = (mvsdk.c_ubyte * FrameHead.uBytes).from_address(
            self._pFrameBuffer)
        frame = np.frombuffer(frame_data, dtype=np.uint8)
        channels = (1 if FrameHead.uiMediaType == mvsdk.CAMERA_MEDIA_TYPE_MONO8
                    else 3)
        return frame.reshape((FrameHead.iHeight, FrameHead.iWidth, channels))

    def _publish_frame(self):
        if not self._hCamera or self._pFrameBuffer is None:
            return
        try:
            frame = self._grab_frame(timeout_ms=100)
            encoding = 'mono8' if self._monoCamera else 'bgr8'
            msg = self._bridge.cv2_to_imgmsg(frame, encoding=encoding)
            msg.header.stamp = self.get_clock().now().to_msg()
            self._image_pub.publish(msg)
        except mvsdk.CameraException:
            pass  # frame not ready yet, skip silently

    def _handle_capture(self, request, response):
        # request.camera_id is intentionally unused — node always captures
        # from the device configured via the device_index parameter.
        _ = request.camera_id
        if not self._hCamera or self._pFrameBuffer is None:
            response.success = False
            response.message = 'Camera is not open'
            return response

        try:
            frame = self._grab_frame(timeout_ms=2000)
            encoding = 'mono8' if self._monoCamera else 'bgr8'
            response.image = self._bridge.cv2_to_imgmsg(frame, encoding=encoding)
            response.success = True
            response.message = 'OK'
            self.get_logger().info('Image captured and returned')
        except mvsdk.CameraException as e:
            response.success = False
            response.message = (
                f'CameraGetImageBuffer failed ({e.error_code}): {e.message}')
            self.get_logger().error(response.message)

        return response

    def destroy_node(self):
        if self._hCamera:
            mvsdk.CameraUnInit(self._hCamera)
            self._hCamera = 0
        if self._pFrameBuffer is not None:
            mvsdk.CameraAlignFree(self._pFrameBuffer)
            self._pFrameBuffer = None
        super().destroy_node()


# ---------------------------------------------------------------------------
# PipCounterNode
# ---------------------------------------------------------------------------


class PipCounterNode(Node):
    """Counts dice pips in two stages.

    Stage 1: HSV colour thresholding to isolate the yellow die face.
    Stage 2: HoughCircles inside that region to count black pips.

    Service: /count_pips  (dice_interfaces/srv/CountPips)
    Publishes: /dice_vision/debug_image  — annotated frame
    """

    def __init__(self):
        super().__init__('pip_counter_node')

        # Yellow detection (HSV)
        self.declare_parameter('h_low',  11)
        self.declare_parameter('h_high', 42)
        self.declare_parameter('s_low',  182)
        self.declare_parameter('v_low',  67)

        # HoughCircles
        self.declare_parameter('dp',          0.3)
        self.declare_parameter('min_dist',    19)
        self.declare_parameter('param1',      61)
        self.declare_parameter('param2',      20)
        self.declare_parameter('min_radius',  7)
        self.declare_parameter('max_radius',  21)
        self.declare_parameter('blur_kernel', 9)   # must be odd

        self._bridge = CvBridge()
        self._debug_pub = self.create_publisher(
            Image, 'dice_vision/debug_image', 10)
        self._srv = self.create_service(
            CountPips, 'count_pips', self._handle_count)
        self.get_logger().info(
            'pip_counter_node ready — /count_pips service active')

    # ------------------------------------------------------------------

    def _handle_count(self, request, response):
        try:
            frame = self._bridge.imgmsg_to_cv2(
                request.image, desired_encoding='bgr8')
        except Exception as e:
            response.success = False
            response.message = f'Image conversion failed: {e}'
            response.pip_count = -1
            return response

        pip_count, debug_frame = self._count_pips(frame)

        response.pip_count = pip_count
        response.success = True
        response.message = f'Detected {pip_count} pip(s)'
        response.debug_image = self._bridge.cv2_to_imgmsg(
            debug_frame, encoding='bgr8')

        self._debug_pub.publish(response.debug_image)
        self.get_logger().info(f'Pip count: {pip_count}')
        return response


    def _count_pips(self, frame: np.ndarray):
        debug = frame.copy()

        # ── Stage 1: find yellow die face ────────────────────────────────
        h_low  = self.get_parameter('h_low').value
        h_high = self.get_parameter('h_high').value
        s_low  = self.get_parameter('s_low').value
        v_low  = self.get_parameter('v_low').value

        hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv,
                           np.array([h_low,  s_low, v_low]),
                           np.array([h_high, 255,   255]))

        kernel = np.ones((7, 7), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            cv2.putText(debug, 'No yellow die found', (10, 45),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 2)
            return 0, debug

        # Largest yellow blob = die face
        largest = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(largest)
        cv2.rectangle(debug, (x, y), (x + w, y + h), (0, 255, 255), 3)

        # ── Stage 2: count pips inside ROI ───────────────────────────────
        roi = frame[y:y + h, x:x + w]

        k = self.get_parameter('blur_kernel').value
        k = k if k % 2 == 1 else k + 1
        k = max(k, 1)

        gray    = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (k, k), 2)

        circles = cv2.HoughCircles(
            blurred,
            cv2.HOUGH_GRADIENT,
            dp=self.get_parameter('dp').value,
            minDist=self.get_parameter('min_dist').value,
            param1=self.get_parameter('param1').value,
            param2=self.get_parameter('param2').value,
            minRadius=self.get_parameter('min_radius').value,
            maxRadius=self.get_parameter('max_radius').value,
        )

        count = 0
        if circles is not None:
            circles = np.round(circles[0, :]).astype(int)
            count = len(circles)
            for (cx, cy, r) in circles:
                cv2.circle(debug, (cx + x, cy + y), r, (0, 255, 0), 2)
                cv2.circle(debug, (cx + x, cy + y), 2, (0, 0, 255), 3)

        cv2.putText(debug, f'Pips: {count}', (10, 45),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 255), 3)
        return count, debug


# ---------------------------------------------------------------------------
# DisplayNode
# ---------------------------------------------------------------------------


class DisplayNode(Node):
    """Real-time OpenCV window showing live camera feed + inspection results.

    Subscribes to:
      /camera/image_raw          — live feed
      /dice_vision/debug_image   — pip-detection overlay (held for 3 s)
      /dice_result               — per-die pip count
      /inspection_summary        — final total
    """

    def __init__(self):
        super().__init__('display_node')

        self._bridge = CvBridge()
        self._lock = threading.Lock()

        self._live_frame = None
        self._debug_frame = None
        self._debug_time = 0.0
        self._debug_hold_secs = 3.0

        self._results = []    # list of (dice_number, pip_count)
        self._total_pips = 0

        self.create_subscription(
            Image, 'camera/image_raw', self._live_cb, 10)
        self.create_subscription(
            Image, 'dice_vision/debug_image', self._debug_cb, 10)
        self.create_subscription(
            DiceResult, 'dice_result', self._result_cb, 10)
        self.create_subscription(
            InspectionSummary, 'inspection_summary', self._summary_cb, 10)

        # No timer — main() calls render_frame() on the main thread to satisfy Qt
        self.get_logger().info('display_node ready — opening video window')

    # ------------------------------------------------------------------

    def _live_cb(self, msg):
        with self._lock:
            self._live_frame = self._bridge.imgmsg_to_cv2(
                msg, desired_encoding='bgr8')

    def _debug_cb(self, msg):
        with self._lock:
            self._debug_frame = self._bridge.imgmsg_to_cv2(
                msg, desired_encoding='bgr8')
            self._debug_time = time.time()

    def _result_cb(self, msg):
        with self._lock:
            self._results.append((msg.dice_number, msg.pip_count))

    def _summary_cb(self, msg):
        with self._lock:
            self._total_pips = msg.total_pips

    def render_frame(self) -> bool:
        """Render one frame to the OpenCV window.

        Must be called from the **main thread** to satisfy Qt's threading
        requirements.  Returns False if the user pressed 'q' (caller should
        shut down).
        """
        with self._lock:
            now = time.time()
            if (self._debug_frame is not None and
                    (now - self._debug_time) < self._debug_hold_secs):
                frame = self._debug_frame.copy()
            elif self._live_frame is not None:
                frame = self._live_frame.copy()
            else:
                return True  # nothing to show yet

            results    = list(self._results)
            total_pips = self._total_pips

        h, w = frame.shape[:2]
        panel_w = 230

        overlay = frame.copy()
        cv2.rectangle(overlay, (w - panel_w, 0), (w, h), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.55, frame, 0.45, 0, frame)

        y = 40
        cv2.putText(frame, 'DICE RESULTS', (w - panel_w + 10, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
        y += 10
        cv2.line(frame, (w - panel_w + 10, y), (w - 10, y), (150, 150, 150), 1)
        y += 28

        for dice_num, pips in results:
            label = f'Dice {dice_num}:  {pips} pip{"s" if pips != 1 else ""}'
            cv2.putText(frame, label, (w - panel_w + 10, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0, 255, 255), 2)
            y += 32

        y += 5
        cv2.line(frame, (w - panel_w + 10, y), (w - 10, y), (150, 150, 150), 1)
        y += 28
        cv2.putText(frame, f'TOTAL:  {total_pips}', (w - panel_w + 10, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 0), 2)

        cv2.imshow('Dice Inspection', frame)
        return (cv2.waitKey(1) & 0xFF) != ord('q')


# ---------------------------------------------------------------------------
# ResultsLoggerNode
# ---------------------------------------------------------------------------


class ResultsLoggerNode(Node):
    """Lightweight console logger for inspection results.

    Subscribes to /dice_result and /inspection_summary and pretty-prints
    them to the terminal.
    """

    def __init__(self):
        super().__init__('results_logger_node')

        self.create_subscription(
            DiceResult, 'dice_result', self._on_result, 10)
        self.create_subscription(
            InspectionSummary, 'inspection_summary', self._on_summary, 10)
        self.get_logger().info('results_logger_node ready')

    def _on_result(self, msg: DiceResult):
        status = 'OK' if msg.success else 'FAIL'
        self.get_logger().info(
            f'[{status}] Dice #{msg.dice_number} -> {msg.pip_count} pip(s)'
            + (f'  ({msg.notes})' if msg.notes else '')
        )

    def _on_summary(self, msg: InspectionSummary):
        lines = [
            '',
            '╔══════════════════════════════╗',
            '║   INSPECTION SUMMARY          ║',
            '╚══════════════════════════════╝',
        ]
        for r in msg.results:
            lines.append(
                f'  Dice {r.dice_number}: {r.pip_count} pip(s) '
                f'{"OK" if r.success else "FAIL"}'
            )
        lines.append('  ─────────────────────────────')
        lines.append(f'  TOTAL : {msg.total_pips} pips')
        lines.append(f'  Status: {"Complete" if msg.complete else "Incomplete"}')
        self.get_logger().info('\n'.join(lines))


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def main(args=None):
    rclpy.init(args=args)

    executor = MultiThreadedExecutor()

    # Create server/provider nodes first so their services are registered
    # before OrchestratorNode's background thread starts looking for them.
    robot_ctrl   = RobotControlNode()
    camera       = CameraNode()
    pip_counter  = PipCounterNode()
    logger       = ResultsLoggerNode()
    display      = DisplayNode()
    orchestrator = OrchestratorNode()

    for node in (robot_ctrl, camera, pip_counter, logger, display, orchestrator):
        executor.add_node(node)

    # Spin ROS executor in a background thread so the main thread is free to
    # own the Qt/OpenCV GUI (cv2.imshow must run on the main thread on Linux).
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        while rclpy.ok():
            if not display.render_frame():
                break          # user pressed 'q'
            time.sleep(0.05)   # ~20 Hz display loop
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        cv2.destroyAllWindows()
        for node in (robot_ctrl, camera, pip_counter, logger, display, orchestrator):
            node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)


if __name__ == '__main__':
    main()



# Terminal 1:                                                                                                                           
#   cd ~/Downloads/Robotics/NishanThapa_Claude_proj/fanuc_ros2_drivers/scripts && ./run_action_servers.sh                               
                                                                                                                                        
#   Terminal 2 (after Terminal 1 is ready):                                                                                               
#   cd ~/Downloads/Robotics/NishanThapa_Claude_proj/fanuc_ros2_drivers/scripts && ./run_dice.sh                                           
                                                                                                