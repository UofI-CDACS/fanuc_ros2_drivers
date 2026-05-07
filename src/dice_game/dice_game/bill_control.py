#!/usr/bin/env python3
"""
bill_control.py

Master control node for BILL (Robot 2, RIGHT, IP: 10.8.4.6).
BILL is responsible for pips 2, 4, and 6.

Conveyor assignment:
  BILL controls the FRONT (closest) conveyor.
  Even pips → front conveyor (BILL runs it directly).
  Odd pips  → back conveyor  (DJ runs it — BILL just receives).

State machine:
  WAIT_FOR_DICE   → idle until DJ delivers die and calls /BILL/request_handoff
  RECEIVE         → pick up from DJ's back conveyor end
  PRESENT_COUNT   → present to camera, loop until expected pip found
  SEND_TO_DJ      → place on front conveyor, run until delivered, signal DJ
  loop for pips 2, 4, 6 — DJ places pip 6 at start position to end game
"""

import sys
import time
import threading

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from fanuc_interfaces.action import JointPose, OnRobotGripper, Conveyor
from fanuc_interfaces.srv import RequestHandoff
from fanuc_interfaces.msg import DiceState, ProxReadings

sys.path.insert(0, 'src/dice_game/dice_game')
import mvsdk

# ============================================================
# JOINT POSITIONS  — all values in degrees, [J1, J2, J3, J4, J5, J6]
# ============================================================
JOINTS = {
    # ── General ──────────────────────────────────────────────────────────
    'home':                       [0.0,      0.0,    0.0,    0.0,   -90.0,  -45.0],

    # ── Pick from DJ's back conveyor ─────────────────────────────────────
    'pre_pick_dj_conveyor':       [-112.73,  25.87,  -7.33,  0.01,  -82.68, -22.83],
    'pick_dj_conveyor':           [-112.73,  29.14, -22.28, -0.01,  -67.72, -22.83],

    # ── Waypoint between DJ conveyor and BILL's set-on-conveyor area ─────
    'intermediate_bill_conveyor': [-86.88,   27.74,   6.81,  0.36,  -97.26, -44.93],

    # ── Dice imaging — normal horizontal grip ────────────────────────────
    'cam_pickup':                 [-51.39,   45.68, -24.74, -0.256, -65.65, -80.36],
    'cam_pickup_up':              [-51.39,   42.42, -15.89, -0.242, -74.5,  -80.36],

    # ── Dice imaging — twist grip (vertical) ─────────────────────────────
    'cam_pickup_twist':           [-49.68,   44.85, -25.84, -0.41,  -64.43,   4.71],
    'cam_pickup_twist_up':        [-49.68,   41.86, -18.16,  0.386, -72.1,    4.77],

    # ── Camera presentation ───────────────────────────────────────────────
    'cam_present':                [-78.68,   24.16, -42.14, 162.96, -43.58, -207.59],

    # ── Set die down on table ─────────────────────────────────────────────
    'setdown':                    [-49.74,   54.33, -92.74, 139.64, -92.23, -223.70],
    'setdown_up':                 [-49.64,   38.44, -78.20, 138.99, -81.18, -214.21],

    # ── BILL's front conveyor (send back to DJ) ───────────────────────────
    'pre_set_on_conveyor':        [-89.51,   18.89, -12.84,  1.14,  -75.99, -48.22],
    'set_on_conveyor':            [-89.51,   22.82, -27.22,  1.25,  -61.61, -48.54],
}

# OnRobot gripper parameters
GRIPPER_OPEN_WIDTH  = 100   # mm
GRIPPER_CLOSE_WIDTH = 35    # mm
GRIPPER_FORCE       = 40    # N

# Proximity sensors for BILL's front conveyor
FRONT_CONVEYOR_START_SENSOR = 'right'   # near BILL's drop end
FRONT_CONVEYOR_STOP_SENSOR  = 'left'    # far end (DJ's side)

# Sensor that trips when die arrives from DJ's back conveyor
BACK_CONVEYOR_ARRIVE_SENSOR = 'right'   # TODO: confirm

PROX_TIMEOUT          = 15.0
MOTION_COMPLETE_TIMEOUT = 60.0

BILL_IP   = '10.8.4.6'
BILL_NAME = 'BILL'


class BillControl(Node):
    def __init__(self):
        super().__init__('bill_control')

        # Callback groups
        self._action_cb = ReentrantCallbackGroup()
        self._state_cb  = MutuallyExclusiveCallbackGroup()

        # --- Action clients ---
        self._joint_client   = ActionClient(self, JointPose,      'BILL/joint_pose',
                                            callback_group=self._action_cb)
        self._onrobot_client = ActionClient(self, OnRobotGripper, 'BILL/onrobot_gripper',
                                            callback_group=self._action_cb)
        self._convey_client  = ActionClient(self, Conveyor,        'BILL/conveyor',
                                            callback_group=self._action_cb)

        # --- Service clients ---
        self._dj_handoff_cli = self.create_client(RequestHandoff, '/DJ/request_handoff',
                                                   callback_group=self._action_cb)

        # --- Handoff service server (DJ calls this to send dice to BILL) ---
        self._handoff_server = self.create_service(
            RequestHandoff, '/BILL/request_handoff',
            self._handle_handoff,
            callback_group=self._action_cb,
        )

        # --- State publisher ---
        self._state_pub = self.create_publisher(DiceState, '/dice_state', 10)

        # --- Internal state ---
        self._bill_retries = 0
        self._dj_retries   = 0
        self._expected_pip = 2
        self._last_prox    = {'right': 0, 'left': 0}
        self._handoff_received = threading.Event()
        self._annotated    = None

        self.get_logger().info('BILL control node ready.')

    # ------------------------------------------------------------------
    # Utilities
    # ------------------------------------------------------------------
    def _wait(self, future, timeout: float = 30.0):
        deadline = time.time() + timeout
        while not future.done():
            if time.time() > deadline:
                raise TimeoutError(f'Future timed out after {timeout}s')
            time.sleep(0.02)
        return future.result()

    # ------------------------------------------------------------------
    # Robot primitives
    # ------------------------------------------------------------------
    def _move(self, joints: list, speed: int = 0):
        self.get_logger().info(f'Move → {joints}' + (f' @ {speed}mm/s' if speed else ''))
        goal = JointPose.Goal()
        goal.joint1, goal.joint2, goal.joint3 = joints[0], joints[1], joints[2]
        goal.joint4, goal.joint5, goal.joint6 = joints[3], joints[4], joints[5]
        goal.speed = speed
        self._joint_client.wait_for_server()
        gh = self._wait(self._joint_client.send_goal_async(goal))
        if not gh.accepted:
            raise RuntimeError('Joint move goal rejected')
        self._wait(gh.get_result_async(), timeout=MOTION_COMPLETE_TIMEOUT)

    def _gripper(self, width: int):
        goal = OnRobotGripper.Goal()
        goal.width = width
        goal.force = GRIPPER_FORCE
        self._onrobot_client.wait_for_server()
        gh = self._wait(self._onrobot_client.send_goal_async(goal))
        if not gh.accepted:
            raise RuntimeError(f'Gripper goal rejected')
        self._wait(gh.get_result_async())
        state = 'OPEN' if width >= GRIPPER_OPEN_WIDTH else 'CLOSE'
        self.get_logger().info(f'Gripper {state}')
        time.sleep(2.0)

    def _conveyor(self, command: str):
        goal = Conveyor.Goal()
        goal.command = command
        self._convey_client.wait_for_server()
        gh = self._wait(self._convey_client.send_goal_async(goal))
        if not gh.accepted:
            raise RuntimeError(f'Conveyor goal rejected: {command}')
        self._wait(gh.get_result_async())

    def _wait_for_prox_trip(self, side: str, timeout: float = PROX_TIMEOUT):
        deadline = time.time() + timeout
        self.get_logger().info(f'  Waiting for {side} sensor to trip...')
        while self._last_prox.get(side, 0) == 0:
            if time.time() > deadline:
                raise TimeoutError(f'{side} sensor never tripped (timeout {timeout}s)')
            time.sleep(0.05)
        self.get_logger().info(f'  {side} sensor tripped.')

    def _run_conveyor_sensors(self, start_sensor: str, stop_sensor: str):
        self.get_logger().info('Waiting for die to load (start sensor)...')
        self._wait_for_prox_trip(start_sensor, timeout=20.0)
        self._conveyor('reverse')
        self.get_logger().info('Conveyor running — waiting for die to arrive (stop sensor)...')
        try:
            self._wait_for_prox_trip(stop_sensor, timeout=20.0)
            self.get_logger().info('Stop sensor tripped — running 2 more seconds...')
            time.sleep(2.0)
        finally:
            self._conveyor('stop')
        self.get_logger().info('Conveyor stopped — die delivered.')

    # ------------------------------------------------------------------
    # Camera
    # ------------------------------------------------------------------
    def _capture_image(self) -> np.ndarray:
        devs = mvsdk.CameraEnumerateDevice()
        if len(devs) < 1:
            raise RuntimeError('No MindVision camera found. Check ethernet cable.')

        hCamera = None
        for attempt in range(5):
            try:
                hCamera = mvsdk.CameraInit(devs[0], -1, -1)
                break
            except Exception as e:
                self.get_logger().warn(f'CameraInit attempt {attempt+1} failed: {e}')
                time.sleep(2.0)
        if hCamera is None:
            raise RuntimeError('CameraInit failed after 5 attempts')

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
                (FrameHead.iHeight, FrameHead.iWidth, channels))
            return frame.copy()
        finally:
            mvsdk.CameraUnInit(hCamera)
            mvsdk.CameraAlignFree(pFrameBuffer)

    def _count_pips(self, image: np.ndarray) -> int:
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array([15, 60, 40]), np.array([45, 255, 255]))
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel, iterations=1)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            self.get_logger().warn('No yellow die detected in image')
            self._annotated = image.copy()
            return -1

        x, y, w, h = cv2.boundingRect(max(contours, key=cv2.contourArea))
        margin = 8
        x = max(0, x - margin);  y = max(0, y - margin)
        w = min(image.shape[1] - x, w + 2 * margin)
        h = min(image.shape[0] - y, h + 2 * margin)

        roi = image[y:y+h, x:x+w].copy()
        roi_bright = cv2.convertScaleAbs(roi, alpha=3.5, beta=50)
        gray = cv2.cvtColor(roi_bright, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (3, 3), 0)

        _, dark_otsu = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        dark_adapt = cv2.adaptiveThreshold(
            blur, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 21, 8)
        dark = cv2.bitwise_or(dark_otsu, dark_adapt)
        dark = cv2.morphologyEx(dark, cv2.MORPH_OPEN, np.ones((2, 2), np.uint8))

        pip_contours, _ = cv2.findContours(dark, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        border_x = int(w * 0.12);  border_y = int(h * 0.12)
        pip_count = 0
        annotated = image.copy()
        cv2.rectangle(annotated, (x, y), (x+w, y+h), (255, 0, 0), 2)

        for c in pip_contours:
            area = cv2.contourArea(c)
            if area < 20 or area > 6000:
                continue
            perimeter = cv2.arcLength(c, True)
            if perimeter == 0 or (4 * np.pi * area / (perimeter ** 2)) <= 0.55:
                continue
            M = cv2.moments(c)
            if M['m00'] == 0:
                continue
            cx = int(M['m10'] / M['m00']) + x
            cy = int(M['m01'] / M['m00']) + y
            if (cx - x) < border_x or (cx - x) > w - border_x:
                continue
            if (cy - y) < border_y or (cy - y) > h - border_y:
                continue
            pip_count += 1
            cv2.drawContours(annotated, [c + np.array([[[x, y]]])], -1, (0, 255, 0), 2)
            cv2.circle(annotated, (cx, cy), 4, (0, 0, 255), -1)

        cv2.putText(annotated, f'pips: {pip_count}', (10, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 2)
        self._annotated = annotated
        return pip_count

    def _capture_and_show(self, label: str, target: int = 0) -> int:
        self.get_logger().info(f'Capturing ({label})...')
        frame = self._capture_image()
        count = self._count_pips(frame)
        self.get_logger().info(f'Pip count: {count}')
        cv2.imwrite('dice_capture.jpg', frame)
        cv2.imwrite('dice_annotated.jpg', self._annotated)
        return count

    # ------------------------------------------------------------------
    # Movement sub-routines (proven in bill_cam_test.py)
    # ------------------------------------------------------------------
    def _normal_pickup_present(self, label: str, target: int) -> int:
        """Pick up from table (horizontal), present to camera. Still holding die."""
        self._move(JOINTS['cam_pickup_up'], speed=300)
        self._move(JOINTS['cam_pickup'])
        self._gripper(GRIPPER_CLOSE_WIDTH)
        self._move(JOINTS['cam_pickup_up'])
        self._move(JOINTS['cam_present'], speed=50)
        return self._capture_and_show(label, target)

    def _twist_pickup_present(self, label: str, target: int) -> int:
        """Pick up from table (twist/vertical), present to camera. Still holding die."""
        self._move(JOINTS['cam_pickup_twist_up'], speed=300)
        self._move(JOINTS['cam_pickup_twist'])
        self._gripper(GRIPPER_CLOSE_WIDTH)
        self._move(JOINTS['cam_pickup_twist_up'])
        self._move(JOINTS['cam_pickup_up'])
        self._move(JOINTS['cam_present'], speed=50)
        return self._capture_and_show(label, target)

    def _set_down(self):
        """Set die back down on table from cam_present position."""
        self._move(JOINTS['setdown_up'])
        self._move(JOINTS['setdown'])
        self._gripper(GRIPPER_OPEN_WIDTH)
        self._move(JOINTS['setdown_up'])
        self._bill_retries += 1

    def _place_on_conveyor(self):
        """Set die down normally, re-pick with twist grip, place on BILL's front conveyor."""
        self._set_down()
        self._move(JOINTS['cam_pickup_twist_up'], speed=300)
        self._move(JOINTS['cam_pickup_twist'])
        self._gripper(GRIPPER_CLOSE_WIDTH)
        self._move(JOINTS['cam_pickup_twist_up'])
        self._move(JOINTS['intermediate_bill_conveyor'])
        self._move(JOINTS['pre_set_on_conveyor'])
        self._move(JOINTS['set_on_conveyor'])
        self._gripper(GRIPPER_OPEN_WIDTH)
        self._move(JOINTS['pre_set_on_conveyor'])

    # ------------------------------------------------------------------
    # State
    # ------------------------------------------------------------------
    def _publish_state(self, robot_holding: str):
        msg = DiceState()
        msg.expected_pip  = self._expected_pip
        msg.dj_retries    = self._dj_retries
        msg.bill_retries  = self._bill_retries
        msg.robot_holding = robot_holding
        self._state_pub.publish(msg)

    # ------------------------------------------------------------------
    # Handoff service server (DJ calls this to send dice to BILL)
    # ------------------------------------------------------------------
    def _handle_handoff(self, request: RequestHandoff.Request,
                        response: RequestHandoff.Response):
        self.get_logger().info('Handoff request from DJ — moving to pre-pick position.')
        self._move(JOINTS['home'])
        self._gripper(GRIPPER_OPEN_WIDTH)
        self._move(JOINTS['pre_pick_dj_conveyor'])
        response.accepted = True
        self._handoff_received.set()
        return response

    # ------------------------------------------------------------------
    # State machine sub-steps
    # ------------------------------------------------------------------
    def _present_and_count(self, target: int):
        """
        Already holding die (just picked from DJ's conveyor, at pre_pick_dj_conveyor).
        Route to cam_present, loop until target pip found. Returns holding die.
        """
        self.get_logger().info(f'=== BILL searching for pip {target} ===')
        attempt = 0
        twist_done = False
        capture_num = 0

        # First present: route through intermediate waypoint from DJ conveyor
        capture_num += 1
        self._move(JOINTS['intermediate_bill_conveyor'])
        self._move(JOINTS['cam_present'], speed=50)
        count = self._capture_and_show(f'dj_conveyor_{capture_num}', target)
        self.get_logger().info(f'  Pip count: {count}  (need {target})')
        self._publish_state('BILL')
        if count == target:
            self.get_logger().info(f'  Found pip {target} on first present!')
            return
        self._set_down()
        attempt += 1

        while True:
            # After 3 failed normal attempts, do one twist if not already done
            if attempt == 4 and not twist_done:
                self.get_logger().info('3 normal attempts failed — trying twist grip')
                capture_num += 1
                count = self._twist_pickup_present(f'twist_{capture_num}', target)
                self.get_logger().info(f'  Pip count: {count}  (need {target})')
                self._publish_state('BILL')
                if count == target:
                    self.get_logger().info(f'  Found pip {target} on twist!')
                    return
                self._set_down()
                twist_done = True

            # Normal pickup
            capture_num += 1
            count = self._normal_pickup_present(f'normal_{capture_num}', target)
            self.get_logger().info(f'  Pip count: {count}  (need {target})')
            self._publish_state('BILL')
            if count == target:
                self.get_logger().info(f'  Found pip {target}!')
                return
            self._set_down()
            attempt += 1

    def _send_to_dj(self):
        """
        Already holding die at cam_present. Place on front conveyor, run belt until
        delivered, then signal DJ to come pick up.
        """
        self.get_logger().info('Placing die on front conveyor...')
        self._place_on_conveyor()
        self._move(JOINTS['home'])

        self._publish_state('IN_TRANSIT')
        self._run_conveyor_sensors(FRONT_CONVEYOR_START_SENSOR, FRONT_CONVEYOR_STOP_SENSOR)

        # Die is now at DJ's end — signal DJ to pick up
        self.get_logger().info('Die delivered — signaling DJ to pick up...')
        req = RequestHandoff.Request()
        req.conveyor = 'front'
        self._dj_handoff_cli.wait_for_service()
        future = self._dj_handoff_cli.call_async(req)
        resp = self._wait(future, timeout=30.0)
        if not resp.accepted:
            raise RuntimeError('DJ rejected handoff request')

    # ------------------------------------------------------------------
    # Main state machine
    # ------------------------------------------------------------------
    def run(self):
        self.get_logger().info('=== BILL state machine starting — waiting for first handoff ===')

        # Subscribe to prox readings
        self.create_subscription(
            ProxReadings, 'BILL/prox_readings',
            lambda msg: self._last_prox.update({'right': msg.right, 'left': msg.left}),
            10,
        )

        for expected in [2, 4, 6]:
            self._expected_pip = expected
            self.get_logger().info(f'--- BILL waiting to receive die (need pip {expected}) ---')

            # Block until DJ calls our handoff service
            # (_handle_handoff moves us to pre_pick_dj_conveyor and opens gripper)
            self._handoff_received.clear()
            self._handoff_received.wait()

            # Die is already at pick position (DJ ran conveyor before calling handoff)

            # Pick up die
            self._move(JOINTS['pick_dj_conveyor'])
            self._gripper(GRIPPER_CLOSE_WIDTH)
            self._move(JOINTS['pre_pick_dj_conveyor'])
            self.get_logger().info('Die picked up from DJ conveyor.')

            # Search for target pip
            self._present_and_count(expected)

            # Send back to DJ (all pips including 6)
            self.get_logger().info(f'--- BILL sending pip {expected} back to DJ ---')
            self._send_to_dj()

            if expected == 6:
                self._publish_state('PLACED')
                self.get_logger().info('=== BILL done. Game complete! ===')
                self._print_results()
                return

        self.get_logger().error('Unexpected exit from run() loop.')

    def _print_results(self):
        total = self._dj_retries + self._bill_retries
        self.get_logger().info('==============================')
        self.get_logger().info('         FINAL RESULTS        ')
        self.get_logger().info('==============================')
        self.get_logger().info(f'  DJ   retries : {self._dj_retries}')
        self.get_logger().info(f'  BILL retries : {self._bill_retries}')
        self.get_logger().info(f'  Total retries: {total}')
        self.get_logger().info('==============================')


def main(args=None):
    rclpy.init(args=args)
    node = BillControl()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
