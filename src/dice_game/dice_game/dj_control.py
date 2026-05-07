#!/usr/bin/env python3
"""
dj_control.py

Master control node for DJ (Robot 1, LEFT, IP: 10.8.4.16).
DJ is responsible for pips 1, 3, and 5.

Conveyor assignment:
  DJ controls the BACK (furthest) conveyor.
  Odd pips  → back conveyor  (DJ runs it directly to send to BILL).
  Even pips → front conveyor (BILL runs it — DJ just receives at its end).

State machine:
  FIND_START      → keep picking from table until pip == 1
  SEND_TO_BILL    → request handoff, place on back conveyor, run until die clears
  WAIT_FOR_RETURN → idle until BILL calls /DJ/request_handoff
  RECEIVE         → wait for prox sensor, pick up from front conveyor
  PRESENT_COUNT   → present to camera, loop until expected pip found
  loop back to SEND_TO_BILL for pips 3 and 5
  DONE            → game over (BILL places pip 6)
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

from fanuc_interfaces.action import JointPose, SchunkGripper, Conveyor
from fanuc_interfaces.srv import RequestHandoff
from fanuc_interfaces.msg import DiceState, ProxReadings

sys.path.insert(0, 'src/dice_game/dice_game')
import mvsdk

# ============================================================
# JOINT POSITIONS  — all values in degrees, [J1, J2, J3, J4, J5, J6]
# ============================================================
JOINTS = {
    # ── General ──────────────────────────────────────────────────────────
    'home':                       [0.0,    0.0,    0.0,    0.0,   -90.0,   30.0],
    'initial_pickup':             [12.34,  24.21, -51.03, -0.01,  -38.97,  17.66],
    'initial_pickup_up':          [12.34,  14.69, -33.40, -0.01,  -56.60,  17.66],

    # ── Dice imaging — normal horizontal grip ────────────────────────────
    'cam_pickup':                 [71.35,  42.61, -25.51, -0.91,  -66.07,  44.90],
    'cam_pickup_up':              [71.35,  39.17, -15.94, -0.86,  -75.64,  44.75],

    # ── Dice imaging — twist grip (vertical) ─────────────────────────────
    'cam_pickup_twist':           [69.43,  40.24, -27.88, -1.85,  -61.39, -41.50],
    'cam_pickup_twist_up':        [69.43,  36.39, -17.89, -1.71,  -71.37, -41.84],

    # ── Camera presentation ───────────────────────────────────────────────
    'cam_present':                [65.87,  20.85, -27.87, -135.54, -37.71, 168.74],

    # ── Set die down on table ─────────────────────────────────────────────
    'setdown':                    [34.03,  53.90, -75.45, -122.12, -83.09, 194.72],
    'setdown_up':                 [34.30,  42.04, -65.02, -120.87, -77.55, 185.76],

    # ── Waypoint between BILL conveyor receive and DJ's set-on-conveyor ──
    'intermediate_dj_conveyor':   [88.60,  26.60,  -7.94,  -0.69,  -83.14,  30.37],

    # ── DJ's back conveyor (send to BILL) ────────────────────────────────
    'pre_set_on_conveyor':        [128.42, 36.69,  -0.81,  -1.22,  -89.58,  -9.54],
    'set_on_conveyor':            [128.42, 38.38, -13.94,  -1.26,  -76.45,  -9.25],

    # ── Receive from BILL's front conveyor ───────────────────────────────
    'pick_bill_conveyor':     [119.02, 19.12, -26.84,  0.01,  -63.15,   -2.12],
    'pre_pick_bill_conveyor':         [119.02, 14.84, -9.51,  0.01,  -80.49,   -2.12],
}

# Proximity sensors for DJ's back conveyor
BACK_CONVEYOR_START_SENSOR  = 'left'    # near DJ's drop end
BACK_CONVEYOR_STOP_SENSOR   = 'right'   # far end (BILL's side)

# Sensor that trips when die arrives from BILL's front conveyor (DJ's receive end)
FRONT_CONVEYOR_ARRIVE_SENSOR = 'left'   # TODO: confirm

PROX_TIMEOUT          = 15.0
MOTION_COMPLETE_TIMEOUT = 60.0

DJ_IP   = '10.8.4.16'
DJ_NAME = 'DJ'


class DJControl(Node):
    def __init__(self):
        super().__init__('dj_control')

        # Callback groups
        self._action_cb = ReentrantCallbackGroup()
        self._state_cb  = MutuallyExclusiveCallbackGroup()

        # --- Action clients ---
        self._joint_client  = ActionClient(self, JointPose,    'DJ/joint_pose',
                                           callback_group=self._action_cb)
        self._schunk_client = ActionClient(self, SchunkGripper, 'DJ/schunk_gripper',
                                           callback_group=self._action_cb)
        self._convey_client = ActionClient(self, Conveyor,      'DJ/conveyor',
                                           callback_group=self._action_cb)

        # --- Service clients ---
        self._bill_handoff_cli = self.create_client(RequestHandoff, '/BILL/request_handoff',
                                                     callback_group=self._action_cb)

        # --- Handoff service server (BILL calls this to send dice back to DJ) ---
        self._handoff_server = self.create_service(
            RequestHandoff, '/DJ/request_handoff',
            self._handle_handoff,
            callback_group=self._action_cb,
        )

        # --- State publisher ---
        self._state_pub = self.create_publisher(DiceState, '/dice_state', 10)

        # --- Internal state ---
        self._dj_retries   = 0
        self._bill_retries = 0
        self._expected_pip = 1
        self._last_prox    = {'right': 0, 'left': 0}
        self._handoff_received = threading.Event()
        self._annotated    = None

        # Subscribe to BILL's retry updates
        self.create_subscription(DiceState, '/dice_state', self._state_cb_fn, 10)

        self.get_logger().info('DJ control node ready.')

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

    def _gripper(self, command: str):
        """'open' or 'close' the Schunk gripper."""
        goal = SchunkGripper.Goal()
        goal.command = command
        self._schunk_client.wait_for_server()
        gh = self._wait(self._schunk_client.send_goal_async(goal))
        if not gh.accepted:
            raise RuntimeError(f'Gripper goal rejected: {command}')
        self._wait(gh.get_result_async())
        self.get_logger().info(f'Gripper {command.upper()}')
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
        self._conveyor('forward')
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
    # Movement sub-routines (proven in dj_cam_test.py)
    # ------------------------------------------------------------------
    def _normal_pickup_present(self, label: str, target: int) -> int:
        """Pick up from table (horizontal), present to camera. Still holding die."""
        self._move(JOINTS['cam_pickup_up'], speed=300)
        self._move(JOINTS['cam_pickup'])
        self._gripper('close')
        self._move(JOINTS['cam_pickup_up'])
        self._move(JOINTS['cam_present'], speed=50)
        return self._capture_and_show(label, target)

    def _twist_pickup_present(self, label: str, target: int) -> int:
        """Pick up from table (twist/vertical), present to camera. Still holding die."""
        self._move(JOINTS['cam_pickup_twist_up'], speed=300)
        self._move(JOINTS['cam_pickup_twist'])
        self._gripper('close')
        self._move(JOINTS['cam_pickup_twist_up'])
        self._move(JOINTS['cam_pickup_up'])
        self._move(JOINTS['cam_present'], speed=50)
        return self._capture_and_show(label, target)

    def _set_down(self):
        """Set die back down on table from cam_present position."""
        self._move(JOINTS['setdown_up'])
        self._move(JOINTS['setdown'])
        self._gripper('open')
        self._move(JOINTS['setdown_up'])
        self._dj_retries += 1

    def _place_on_back_conveyor(self):
        """Set die down normally, re-pick with twist, place on DJ's back conveyor."""
        self._set_down()
        self._move(JOINTS['cam_pickup_up'], speed=300)
        self._move(JOINTS['cam_pickup'])
        self._gripper('close')
        self._move(JOINTS['cam_pickup_up'])
        self._move(JOINTS['intermediate_dj_conveyor'])
        self._move(JOINTS['pre_set_on_conveyor'])
        self._move(JOINTS['set_on_conveyor'])
        self._gripper('open')
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

    def _state_cb_fn(self, msg: DiceState):
        self._bill_retries = msg.bill_retries

    # ------------------------------------------------------------------
    # Handoff service server (BILL calls this when sending dice back to DJ)
    # ------------------------------------------------------------------
    def _handle_handoff(self, request: RequestHandoff.Request,
                        response: RequestHandoff.Response):
        self.get_logger().info('Handoff request from BILL — moving to pre-pick position.')
        self._move(JOINTS['home'])
        self._gripper('open')
        self._move(JOINTS['pre_pick_bill_conveyor'])
        response.accepted = True
        self._handoff_received.set()
        return response

    # ------------------------------------------------------------------
    # State machine sub-steps
    # ------------------------------------------------------------------
    def _find_pip(self, target: int):
        """
        Initial search — die starts at initial_pickup position.
        First pick uses initial_pickup, then loops using cam_pickup.
        """
        self.get_logger().info(f'=== DJ searching for pip {target} (initial pickup) ===')
        self._gripper('open')
        attempt = 0
        twist_done = False
        capture_num = 0

        # First pick from initial position
        capture_num += 1
        self.get_logger().info(f'Move → {JOINTS["initial_pickup_up"]} @ 300mm/s')
        self._move(JOINTS['initial_pickup_up'], speed=300)
        self._move(JOINTS['initial_pickup'])
        self._gripper('close')
        self._move(JOINTS['initial_pickup_up'])
        self._move(JOINTS['cam_present'], speed=50)
        count = self._capture_and_show(f'initial_{capture_num}', target)
        self.get_logger().info(f'  Pip count: {count}  (need {target})')
        self._publish_state('DJ')
        if count == target:
            self.get_logger().info(f'  Found pip {target} on first pick!')
            return
        self._set_down()
        attempt += 1

        while True:
            if attempt == 3 and not twist_done:
                self.get_logger().info('3 attempts failed — trying twist grip')
                capture_num += 1
                count = self._twist_pickup_present(f'twist_{capture_num}', target)
                self.get_logger().info(f'  Pip count: {count}  (need {target})')
                self._publish_state('DJ')
                if count == target:
                    self.get_logger().info(f'  Found pip {target} on twist!')
                    return
                self._set_down()
                twist_done = True

            capture_num += 1
            count = self._normal_pickup_present(f'normal_{capture_num}', target)
            self.get_logger().info(f'  Pip count: {count}  (need {target})')
            self._publish_state('DJ')
            if count == target:
                self.get_logger().info(f'  Found pip {target}!')
                return
            self._set_down()
            attempt += 1

    def _present_and_count(self, target: int):
        """
        Already holding die (just picked from BILL's conveyor, at pre_pick_bill_conveyor).
        Route to cam_present, loop until target pip found. Returns holding die.
        """
        self.get_logger().info(f'=== DJ searching for pip {target} ===')
        attempt = 0
        twist_done = False
        capture_num = 0

        # First present: route through intermediate waypoint from BILL's conveyor
        capture_num += 1
        self._move(JOINTS['intermediate_dj_conveyor'])
        self._move(JOINTS['cam_present'], speed=50)
        count = self._capture_and_show(f'bill_conveyor_{capture_num}', target)
        self.get_logger().info(f'  Pip count: {count}  (need {target})')
        self._publish_state('DJ')
        if count == target:
            self.get_logger().info(f'  Found pip {target} on first present!')
            return
        self._set_down()
        attempt += 1

        while True:
            if attempt == 4 and not twist_done:
                self.get_logger().info('3 normal attempts failed — trying twist grip')
                capture_num += 1
                count = self._twist_pickup_present(f'twist_{capture_num}', target)
                self.get_logger().info(f'  Pip count: {count}  (need {target})')
                self._publish_state('DJ')
                if count == target:
                    self.get_logger().info(f'  Found pip {target} on twist!')
                    return
                self._set_down()
                twist_done = True

            capture_num += 1
            count = self._normal_pickup_present(f'normal_{capture_num}', target)
            self.get_logger().info(f'  Pip count: {count}  (need {target})')
            self._publish_state('DJ')
            if count == target:
                self.get_logger().info(f'  Found pip {target}!')
                return
            self._set_down()
            attempt += 1

    def _send_to_bill(self):
        """
        Already holding die at cam_present. Place on back conveyor, run belt until
        delivered, then signal BILL to come pick up.
        """
        self.get_logger().info('Placing die on back conveyor...')
        self._place_on_back_conveyor()
        self._move(JOINTS['home'])

        self._publish_state('IN_TRANSIT')
        self._run_conveyor_sensors(BACK_CONVEYOR_START_SENSOR, BACK_CONVEYOR_STOP_SENSOR)

        # Die is now at BILL's end — signal BILL to pick up
        self.get_logger().info('Die delivered — signaling BILL to pick up...')
        req = RequestHandoff.Request()
        req.conveyor = 'back'
        self._bill_handoff_cli.wait_for_service()
        future = self._bill_handoff_cli.call_async(req)
        resp = self._wait(future, timeout=30.0)
        if not resp.accepted:
            raise RuntimeError('BILL rejected handoff request')

    def _wait_for_return(self):
        """
        Wait for BILL to signal die has been delivered, then pick up from BILL's conveyor.
        (_handle_handoff moves DJ to pre_pick_bill_conveyor — die is already there.)
        """
        self.get_logger().info('Waiting for BILL to send die back...')
        self._handoff_received.clear()
        self._handoff_received.wait()
        # Die is already at DJ's pick position (BILL ran conveyor before calling handoff)
        # _handle_handoff already moved DJ to pre_pick_bill_conveyor

        # Pick up die
        self._move(JOINTS['pick_bill_conveyor'])
        self._gripper('close')
        self._move(JOINTS['pre_pick_bill_conveyor'])
        self.get_logger().info('Die received from BILL.')

    # ------------------------------------------------------------------
    # Main state machine
    # ------------------------------------------------------------------
    def run(self):
        self.get_logger().info('=== DJ state machine starting ===')

        # Subscribe to prox readings
        self.create_subscription(
            ProxReadings, 'DJ/prox_readings',
            lambda msg: self._last_prox.update({'right': msg.right, 'left': msg.left}),
            10,
        )

        # ---- Phase 1: Find pip = 1 from table ----------------------------
        self._expected_pip = 1
        self.get_logger().info('--- Phase 1: Finding pip = 1 from table ---')
        self._find_pip(target=1)

        # ---- DJ handles 1 → send to BILL, receive back for 3 → send, etc.
        for expected_dj in [1, 3, 5]:
            self._expected_pip = expected_dj
            self.get_logger().info(f'--- DJ sending pip {expected_dj} to BILL ---')
            self._send_to_bill()

            next_dj_pip = expected_dj + 2   # 3, 5, or 7 (7 = pip 6 return)
            self.get_logger().info(f'--- DJ waiting to receive die back ---')
            self._wait_for_return()

            if expected_dj == 5:
                # BILL found pip 6 and sent it back — set it down at the start position
                self.get_logger().info('--- Pip 6 received — setting die down at start position ---')
                self._move(JOINTS['intermediate_dj_conveyor'])
                self._move(JOINTS['initial_pickup_up'])
                self._move(JOINTS['initial_pickup'])
                self._gripper('open')
                self._move(JOINTS['initial_pickup_up'])
                self._move(JOINTS['home'])
                self.get_logger().info('=== Game complete! ===')
                self._publish_state('DONE')
                return

            self._expected_pip = next_dj_pip
            self._present_and_count(target=next_dj_pip)

        self.get_logger().info('=== DJ done. ===')
        self._publish_state('DONE')


def main(args=None):
    rclpy.init(args=args)
    node = DJControl()

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
