#!/usr/bin/env python3
"""

Scan method:
  1. Scan top face  (should be pip 1 — robot1 guarantee)
  2. Rotate −90° about x  (exposes adjacent face)
  3. Scan new top face  (originally the left face)
  (top, left) → lookup table → single rotation to bring pip 2,4, or 6 to top

State published to /dual_fanuc/robot2_dice_location:
  "rotating"         — actively manipulating die
  "at_back_conveyer" — in hard-stop position, ready for Robot 1 to run conveyor
  "on_conveyer"      — die placed on conveyor, Robot 1 can grab

Standard die chirality: opposite faces sum to 7; with 1 on top and 2 facing viewer, 3 is on the right.
"""

import json
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
import cv2
import numpy as np

from fanuc_interfaces.action import CartPose, JointPose, OnRobotGripper, Conveyor
from fanuc_interfaces.srv import SetSpeed

from dual_fanuc.pip_lookup import get_action


# ══════════════════════════════════════════════════════════════════════════════
#  Configuration
# ══════════════════════════════════════════════════════════════════════════════

ROBOT_NAME   = os.environ.get('ROBOT_2_NAME', 'robot2')
ROBOT_1_NAME = os.environ.get('ROBOT_1_NAME', 'robot1')
ROBOT_SPEED = 300  # mm/s
DICE_WIDTH  = 80.0

R1_LOCATION_TOPIC = '/dual_fanuc/robot1_dice_location'
R2_LOCATION_TOPIC = '/dual_fanuc/robot2_dice_location'

HOME_JOINTS    = (0, 0, 0, 0, -90, 0)
CONVEYOR_PICK  = {'x': -221.1,  'y': -693.88, 'z':  60.747, 'w': -179.9, 'p': 0.0, 'r': -90.0}
CONVEYOR_HARD_STOP = {'x': -221.1, 'y': -634.93, 'z': 60.747, 'w': -179.9, 'p': 0.0, 'r': -90.0}
DICE_PLACE     = {'x':  450.52, 'y': -375.16-150, 'z': -76.28, 'w': -179.9, 'p': 0.0, 'r': -90.0}
# Terminal/end position — Robot 2 grabs from here in `just rotate2` and drops here after pip 6.
DICE_END       = {'x':  462.65, 'y':  -12.938, 'z': -115.594, 'w': 179.9, 'p': 0.0, 'r': -90.0}
# ── X-flip (−90° about x) ────────────────────────────────────────────────────
# Approach from +y by DICE_WIDTH
FLIP_X_GRAB = {'x': 450.52, 'y': -300.6-150, 'z': -159.873, 'w': 165.474, 'p': -89.89, 'r': -75.278}

CONVEYOR_PLACE_FRONT = dict(CONVEYOR_PICK, x=136.0)  # front conveyor; shares y/z/w/p/r with back

# OnRobot gripper
ONROBOT_OPEN_WIDTH  = 100   # mm
ONROBOT_CLOSE_WIDTH = 5     # mm
ONROBOT_FORCE       = 40    # N

# Camera
CAMERA_TOPIC   = '/mv_camera/image_raw'
IMAGE_SAVE_DIR = os.path.expanduser('~/ClaudeDualFanuc2/tmp/')

HSV_H_LOW  = 14;  HSV_S_LOW  = 100;  HSV_V_LOW  = 100
HSV_H_HIGH = 35;  HSV_S_HIGH = 255;  HSV_V_HIGH = 255
MIN_CONTOUR_AREA = 2500

# Region of interest applied before HSV detection (camera-pixel coords).
ROI_X1 = 0
ROI_Y1 = 361
ROI_X2 = 1280
ROI_Y2 = 860

HOUGH_DP     = 1.0
HOUGH_MIN_DIST = 8
HOUGH_PARAM1 = 8
HOUGH_PARAM2 = 14
HOUGH_MIN_R  = 0
HOUGH_MAX_R  = 8

VALID_PIP_COUNTS = frozenset(range(1, 7))  # {1, 2, 3, 4, 5, 6}

# Lookup tables live in pip_lookup.py (PIP2_LOOKUP, PIP4_LOOKUP, PIP6_LOOKUP).
# Use get_action(target_pip, scan_top, scan_after_xflip) to query them.


# ══════════════════════════════════════════════════════════════════════════════
#  Node
# ══════════════════════════════════════════════════════════════════════════════

class Robot2Node(Node):

    def __init__(self):
        super().__init__('robot2')

        ns = ROBOT_NAME
        self.cart_ac      = ActionClient(self, CartPose,       f'/{ns}/cartesian_pose')
        self.joints_ac    = ActionClient(self, JointPose,      f'/{ns}/joint_pose')
        self.onrobot_ac   = ActionClient(self, OnRobotGripper, f'/{ns}/onrobot_gripper')
        self.conveyor_ac  = ActionClient(self, Conveyor,       f'/{ns}/conveyor')
        self.speed_client = self.create_client(SetSpeed,       f'/{ns}/set_speed')

        self.bridge       = CvBridge()
        self.latest_image = None
        self.create_subscription(Image, CAMERA_TOPIC, self._image_cb, 10)
        self.capture_client = self.create_client(Trigger, '/mv_camera/capture')

        self.r2_location_pub = self.create_publisher(String, R2_LOCATION_TOPIC, 10)
        self.robot1_location = ''
        self.create_subscription(String, R1_LOCATION_TOPIC, self._r1_location_cb, 10)

        # Flip-count tracking (physical die rotations per oriented pip; incremented in _rotate_x_neg_90)
        # plus cross-robot visibility via /flip_log topics.
        self.target_pip      = None
        self.flip_count      = 0
        self.flip_log        = {}
        self.robot1_flip_log = {}
        self.last_gripper_width = None  # skips redundant gripper commands
        self.flip_log_pub    = self.create_publisher(String, f'/{ns}/flip_log', 10)
        self.create_subscription(String, f'/{ROBOT_1_NAME}/flip_log',
                                 self._r1_flip_log_cb, 10)

        self.get_logger().info('Waiting for action servers...')
        # Essential — all in robot 2's own namespace, started by `just launch2`. Block until available.
        self.cart_ac.wait_for_server()
        self.joints_ac.wait_for_server()
        self.onrobot_ac.wait_for_server()
        self.conveyor_ac.wait_for_server()
        # Optional — camera node is only started by `just launch1` / `just launch`. Warn and continue
        # so rotate-only tests work with just `launch2`.
        if not self.capture_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn('Camera capture service /mv_camera/capture not available after 5 s — continuing.')
        self.get_logger().info(f'Robot 2 ready — namespace: /{ns}')

    # ── Coordination ──────────────────────────────────────────────────────────

    def _r1_location_cb(self, msg: String):
        self.robot1_location = msg.data

    def _publish_location(self, state: str):
        msg = String()
        msg.data = state
        self.r2_location_pub.publish(msg)
        self.get_logger().info(f'robot2_dice_location: {state}')

    def _wait_for_robot1(self, *states: str):
        """Wait until robot1_location matches any given state. Returns the matched state."""
        label = ' or '.join(states)
        self.get_logger().info(f'Waiting for robot1_dice_location: {label}')
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.robot1_location in states:
                return self.robot1_location

    def _r1_flip_log_cb(self, msg: String):
        try:
            self.robot1_flip_log = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f'Could not parse robot1 flip_log: {msg.data}')

    def _publish_flip_log(self):
        msg = String()
        msg.data = json.dumps({str(k): v for k, v in self.flip_log.items()})
        self.flip_log_pub.publish(msg)

    def _publish_done_state(self, target_pip):
        state = 'run_complete' if target_pip == 6 else 'on_conveyer'
        self._publish_location(state)

    def _print_summary(self):
        own_total   = sum(self.flip_log.values())
        other       = {int(k): v for k, v in self.robot1_flip_log.items()}
        other_total = sum(other.values())
        lines = ['═══ Robot 1 flip summary ═══']
        if other:
            for pip in sorted(other):
                lines.append(f'  pip {pip}: {other[pip]} flip(s)')
            lines.append(f'  Robot 1 total: {other_total}')
        else:
            lines.append('  (Robot 1 flip log not received)')
        lines.append('═══ Robot 2 flip summary ═══')
        for pip in sorted(self.flip_log):
            lines.append(f'  pip {pip}: {self.flip_log[pip]} flip(s)')
        lines.append(f'  Robot 2 total: {own_total}')
        lines.append(f'═══ Combined total: {own_total + other_total} ═══')
        for line in lines:
            self.get_logger().info(line)

    # ── Camera ────────────────────────────────────────────────────────────────

    def _image_cb(self, msg):
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def _capture_image(self):
        self.latest_image = None
        future = self.capture_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future)
        if not future.result().success:
            self.get_logger().error('Camera capture service returned failure.')
            return None
        deadline_ns = self.get_clock().now().nanoseconds + int(3.0 * 1e9)
        while self.latest_image is None:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.get_clock().now().nanoseconds > deadline_ns:
                self.get_logger().error('Timed out waiting for camera frame.')
                return None
        return self.latest_image.copy()

    def _crop_die(self, image):
        image = image[ROI_Y1:ROI_Y2, ROI_X1:ROI_X2]   # narrow search to ROI
        hsv  = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv,
                           np.array([HSV_H_LOW,  HSV_S_LOW,  HSV_V_LOW]),
                           np.array([HSV_H_HIGH, HSV_S_HIGH, HSV_V_HIGH]))
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        mask   = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask   = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = [c for c in contours if cv2.contourArea(c) >= MIN_CONTOUR_AREA]
        if not contours:
            self.get_logger().warn('HSV crop: no region above MIN_CONTOUR_AREA — using full image.')
            return image
        # Match calibrate-hsv exactly: fill the chosen contour, bitwise_and with the
        # ROI so everything outside the die contour goes black, then crop.
        c = max(contours, key=cv2.contourArea)
        die_mask = np.zeros(image.shape[:2], dtype=np.uint8)
        cv2.drawContours(die_mask, [c], -1, 255, thickness=cv2.FILLED)
        masked = cv2.bitwise_and(image, image, mask=die_mask)
        x, y, w, h = cv2.boundingRect(c)
        pad = 10
        return masked[max(0, y-pad):min(image.shape[0], y+h+pad),
                      max(0, x-pad):min(image.shape[1], x+w+pad)]

    def _count_pips(self, image):
        gray    = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (11, 11), 2)
        circles = cv2.HoughCircles(blurred, cv2.HOUGH_GRADIENT,
                                   dp=HOUGH_DP, minDist=HOUGH_MIN_DIST,
                                   param1=HOUGH_PARAM1, param2=HOUGH_PARAM2,
                                   minRadius=HOUGH_MIN_R, maxRadius=HOUGH_MAX_R)
        debug = image.copy()
        if circles is None:
            cv2.putText(debug, 'No pips detected', (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
            return 0, debug
        for (cx, cy, r) in np.round(circles[0]).astype(int):
            cv2.circle(debug, (cx, cy), r, (0, 255, 0), 2)
            cv2.circle(debug, (cx, cy), 3, (0, 0, 255), -1)
        count = len(circles[0])
        cv2.putText(debug, f'Pips: {count}', (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
        return count, debug

    def _capture_face(self, die_x, die_y, die_z, label='Face'):
        """Capture image, count pips, save debug images. Returns (count, debug).
        Returns (None, None) on camera failure; count may be 0 if no pips detected."""
        image = self._capture_image()
        if image is None:
            self.get_logger().error(f'{label}: camera did not return a frame.')
            return None, None
        slug = label.lower().replace(' ', '_')
        cv2.imwrite(f'{IMAGE_SAVE_DIR}r2_{slug}_full.png', image)
        cropped = self._crop_die(image)
        count, debug = self._count_pips(cropped)
        cv2.imwrite(f'{IMAGE_SAVE_DIR}r2_{slug}.png', debug)
        self.get_logger().info(f'Expected Pip: {self.target_pip} ---- Saw Pip: {count}')
        return count, debug

    def _is_valid_pip(self, count, label):
        """Return True if count is a valid die face (1–6); log a clear error otherwise."""
        if count is None:
            self.get_logger().error(
                f'{label}: camera failure — cannot read pip count. Aborting.')
            return False
        if count not in VALID_PIP_COUNTS:
            self.get_logger().error(
                f'{label}: invalid pip count {count} (must be 1–6). '
                f'Check camera, lighting, or Hough calibration. Aborting.')
            return False
        return True

    # ── Motion primitives ─────────────────────────────────────────────────────

    def _send_goal(self, client, goal):
        future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        gh = future.result()
        if not gh.accepted:
            self.get_logger().error('Goal rejected.')
            return None
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rf)
        return rf.result().result

    def _move_cart(self, x, y, z, w=200.0, p=200.0, r=200.0):
        goal = CartPose.Goal()
        goal.x, goal.y, goal.z = float(x), float(y), float(z)
        goal.w, goal.p, goal.r = float(w), float(p), float(r)
        res = self._send_goal(self.cart_ac, goal)
        return res is not None and res.success

    def _move_joints(self, j1, j2, j3, j4, j5, j6):
        goal = JointPose.Goal()
        goal.joint1, goal.joint2, goal.joint3 = float(j1), float(j2), float(j3)
        goal.joint4, goal.joint5, goal.joint6 = float(j4), float(j5), float(j6)
        res = self._send_goal(self.joints_ac, goal)
        return res is not None and res.success

    def _onrobot(self, command, width=None, delay=3.0):
        # Resolve the actual width that would be commanded.
        if width is not None:
            target_width = width
        elif command == 'open':
            target_width = ONROBOT_OPEN_WIDTH
        else:
            target_width = ONROBOT_CLOSE_WIDTH

        # Skip if the gripper is already at the same width — saves the 3 s settle time.
        if self.last_gripper_width == target_width:
            self.get_logger().info(
                f'Gripper already at width {target_width} — skipping {command}.')
            return True

        goal = OnRobotGripper.Goal()
        goal.width = target_width
        goal.force = ONROBOT_FORCE
        res = self._send_goal(self.onrobot_ac, goal)
        time.sleep(delay)
        ok = res is not None and res.success
        if ok:
            self.last_gripper_width = target_width
        return ok

    def _conveyor(self, command):
        goal = Conveyor.Goal()
        goal.command = command
        res = self._send_goal(self.conveyor_ac, goal)
        return res is not None and res.success

    def _set_speed(self, speed=ROBOT_SPEED):
        if not self.speed_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn('set_speed service not available.')
            return
        req = SetSpeed.Request()
        req.speed = speed
        future = self.speed_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

    def go_home(self):
        self.get_logger().info('Going home...')
        self._move_joints(*HOME_JOINTS)
        self._onrobot('open')

    def _pick_from_dice_end(self):
        """Pick die from DICE_END, place at DICE_PLACE, retreat for camera."""
        pk = DICE_END
        pl = DICE_PLACE

        self._onrobot('open')
        self._move_cart(pk['x'], pk['y'], pk['z'] + 2*DICE_WIDTH, pk['w'], pk['p'], pk['r'])  # hover
        self._move_cart(pk['x'], pk['y'], pk['z'],                pk['w'], pk['p'], pk['r'])  # descend
        self._onrobot('close')
        self._move_cart(pk['x'], pk['y'], pk['z'] + 2*DICE_WIDTH, pk['w'], pk['p'], pk['r'])  # lift straight up to safe z
        self._move_cart(pl['x'], pl['y'], pl['z'] + 2*DICE_WIDTH, pl['w'], pl['p'], pl['r'])  # transit horizontally at safe z
        self._move_cart(pl['x'], pl['y'], pl['z'],                pl['w'], pl['p'], pl['r'])  # lower to place
        self._onrobot('open')
        self._move_cart(pl['x'], pl['y'], pl['z'] + 2*DICE_WIDTH,   pl['w'], pl['p'], pl['r'])  # retreat

    # ── Flip primitives ───────────────────────────────────────────────────────

    def _rotate_x_neg_90(self):
        """−90° about x: grab from current die position, transit to FLIP_X_GRAB, release, retreat +y.
        Counts as one physical flip; wrappers (flip_x_180, rotate_x_pos_90, y rotates) compose."""
        self.flip_count += 1
        dp = DICE_PLACE
        fg = FLIP_X_GRAB
        dw, dpw, dr = dp['w'], dp['p'], dp['r']

        self._move_cart(dp['x'], dp['y'], dp['z'] + 2*DICE_WIDTH, dw, dpw, dr)              # hover above die
        self._onrobot('open')
        self._move_cart(dp['x'], dp['y'], dp['z'],              dw, dpw, dr)                 # lower to grab
        self._onrobot('close', width=80)
        self._move_cart(dp['x'], dp['y'], dp['z'] + DICE_WIDTH, dw, dpw, dr)                # lift
        self._move_cart(fg['x'], fg['y'], fg['z'] + DICE_WIDTH, fg['w'], fg['p'], fg['r'])  # transit + rotate wrist
        self._move_cart(fg['x'], fg['y'], fg['z'],              fg['w'], fg['p'], fg['r'])  # lower to flip position
        self._onrobot('open')
        self._move_cart(fg['x'], fg['y'] + DICE_WIDTH, fg['z'], fg['w'], fg['p'], fg['r'])  # retreat +y

    def _rotate_x_pos_90(self):
        """+90° about x via three consecutive −90° rotations. Pure motion (no scan)."""
        self._rotate_x_neg_90()
        self._rotate_x_neg_90()
        self._rotate_x_neg_90()


    def _rotate_x_neg_90_and_scan(self, label='flip'):
        """−90° about x, then capture and return pip count. Returns None and aborts on bad read."""
        self._rotate_x_neg_90()
        fg = FLIP_X_GRAB
        count, _ = self._capture_face(fg['x'], fg['y'], fg['z'], label)
        if not self._is_valid_pip(count, label):
            return None
        return count

    def _rotate_y_neg_90(self):
        """Rotate die −90° about y: J6 −90° at current die position, set down, then single _rotate_x_neg_90.
        With DICE_PLACE['r'] = -90, dr_rot = -180 wraps to +180 then clamps to +179.9 to stay inside J6 limits;
        the controller takes the shorter J6 −90° path to reach the wrapped target."""
        dp = DICE_PLACE
        dw, dpw, dr = dp['w'], dp['p'], dp['r']
        dr_rot = dr - 90
        if dr_rot <= -180:
            dr_rot += 360  # wrap to equivalent +angle inside ±180
        dr_rot = max(min(dr_rot, 179.9), -179.9)  # stay inside J6 ±179.9° limit

        self._move_cart(dp['x'], dp['y'], dp['z'] + 2*DICE_WIDTH, dw, dpw, dr)          # hover
        self._onrobot('open')
        self._move_cart(dp['x'], dp['y'], dp['z'],              dw, dpw, dr)             # lower to grab
        self._onrobot('close', width=80)
        self._move_cart(dp['x'], dp['y'], dp['z'] + DICE_WIDTH, dw, dpw, dr)            # lift
        self._move_cart(dp['x'], dp['y'], dp['z'] + DICE_WIDTH, dw, dpw, dr_rot)        # J6 −90° (yaw die)
        self._move_cart(dp['x'], dp['y'], dp['z'],              dw, dpw, dr_rot)         # lower with new orientation
        self._onrobot('open')
        self._move_cart(dp['x'], dp['y'], dp['z'] + DICE_WIDTH, dw, dpw, dr_rot)        # lift
        self._move_cart(dp['x'], dp['y'], dp['z'] + 2*DICE_WIDTH, dw, dpw, dr)          # J6 +90° (back to standard)
        self._rotate_x_neg_90()

    def _rotate_y_pos_90(self):
        """Rotate die +90° about y: J6 +90° at current die position, set down, then single _rotate_x_neg_90.
        With DICE_PLACE['r'] = -90, dr_rot = 0 — no wrap needed; clamp is a no-op for this robot."""
        dp = DICE_PLACE
        dw, dpw, dr = dp['w'], dp['p'], dp['r']
        dr_rot = dr + 90
        if dr_rot >= 180:
            dr_rot -= 360  # wrap to equivalent −angle inside ±180
        dr_rot = max(min(dr_rot, 179.9), -179.9)  # stay inside J6 ±179.9° limit

        self._move_cart(dp['x'], dp['y'], dp['z'] + 2*DICE_WIDTH, dw, dpw, dr)          # hover
        self._onrobot('open')
        self._move_cart(dp['x'], dp['y'], dp['z'],              dw, dpw, dr)             # lower to grab
        self._onrobot('close', width=80)
        self._move_cart(dp['x'], dp['y'], dp['z'] + DICE_WIDTH, dw, dpw, dr)            # lift
        self._move_cart(dp['x'], dp['y'], dp['z'] + DICE_WIDTH, dw, dpw, dr_rot)        # J6 +90° (yaw die)
        self._move_cart(dp['x'], dp['y'], dp['z'],              dw, dpw, dr_rot)         # lower with new orientation
        self._onrobot('open')
        self._move_cart(dp['x'], dp['y'], dp['z'] + DICE_WIDTH, dw, dpw, dr_rot)        # lift
        self._move_cart(dp['x'], dp['y'], dp['z'] + 2*DICE_WIDTH, dw, dpw, dr)          # J6 −90° (back to standard)
        self._rotate_x_neg_90()

    def _flip_x_180(self):
        """180° about x via two −90° rotations."""
        self._rotate_x_neg_90()
        self._rotate_x_neg_90()

    
    # ── Pip orientation ───────────────────────────────────────────────────────

    def _orient_pip(self, target_pip, scan_top, scan_left):
        """Look up (target_pip, scan_top, scan_left) and apply the rotation.
        Verification happens in _scan_orient_place's pre-conveyor scan."""
        action = get_action(target_pip, scan_top, scan_left)
        if action is None:
            self.get_logger().error(
                f'Unrecognised orientation for pip {target_pip}: '
                f'top={scan_top} left={scan_left}. '
                f'Check detection accuracy or die chirality.')
            return False
        self.get_logger().info(
            f'Pip {target_pip} — orientation ({scan_top},{scan_left}) → action: {action}')

        if action == 'none':
            pass
        elif action == 'rotate_y_neg90':
            self._rotate_y_neg_90()
        elif action == 'rotate_y_pos90':
            self._rotate_y_pos_90()
        elif action == 'flip_x_180':
            self._flip_x_180()
        elif action == 'rotate_x_neg90':
            self._rotate_x_neg_90()
        elif action == 'rotate_x_pos90':
            self._rotate_x_pos_90()
        return True

    # ── Placement ─────────────────────────────────────────────────────────────

    def _place_on_front_conveyor(self):
        """Pick die from current die position (top-down grip), transit to front conveyor, release."""
        cp = CONVEYOR_PLACE_FRONT
        dp = DICE_PLACE
        pickup_hover_z = dp['z'] + 2 * DICE_WIDTH   # hover above die
        safe_z         = cp['z'] + DICE_WIDTH        # clear height for transit

        # ── Pick up die ───────────────────────────────────────────────────────
        self._move_cart(dp['x'], dp['y'], pickup_hover_z, dp['w'], dp['p'], dp['r'])  # hover over die
        self._onrobot('open')
        self._move_cart(dp['x'], dp['y'], dp['z'],        dp['w'], dp['p'], dp['r'])  # descend to die
        self._onrobot('close', width=80)
        self._move_cart(dp['x'], dp['y'], safe_z,         dp['w'], dp['p'], dp['r'])  # lift to safe z

        # ── Place on front conveyor ───────────────────────────────────────────
        self._move_cart(cp['x'], cp['y'], safe_z,  cp['w'], cp['p'], cp['r'])  # transit
        self._move_cart(cp['x'], cp['y'], cp['z'], cp['w'], cp['p'], cp['r'])  # lower to conveyor
        self._onrobot('open')
        self._move_cart(cp['x'], cp['y'], safe_z,  cp['w'], cp['p'], cp['r'])  # lift away

    def _place_at_dice_end(self):
        """Pick die from DICE_PLACE (top-down grip), transit to DICE_END, release. Used after pip 6."""
        de = DICE_END
        dp = DICE_PLACE
        pickup_hover_z = dp['z'] + 2 * DICE_WIDTH
        safe_z         = max(dp['z'], de['z']) + 2 * DICE_WIDTH  # clears both source and destination

        # ── Pick up die ───────────────────────────────────────────────────────
        self._move_cart(dp['x'], dp['y'], pickup_hover_z, dp['w'], dp['p'], dp['r'])  # hover
        self._onrobot('open')
        self._move_cart(dp['x'], dp['y'], dp['z'],        dp['w'], dp['p'], dp['r'])  # descend
        self._onrobot('close', width=80)
        self._move_cart(dp['x'], dp['y'], safe_z,         dp['w'], dp['p'], dp['r'])  # lift to safe z

        # ── Place at DICE_END ─────────────────────────────────────────────────
        self._move_cart(de['x'], de['y'], safe_z,  de['w'], de['p'], de['r'])  # transit
        self._move_cart(de['x'], de['y'], de['z'], de['w'], de['p'], de['r'])  # lower to dice end
        self._onrobot('open')
        self._move_cart(de['x'], de['y'], safe_z,  de['w'], de['p'], de['r'])  # lift away

    # ── Task sequence ─────────────────────────────────────────────────────────

    def _scan_orient_place(self, target_pip=2):
        """Scan die at DICE_PLACE, orient target_pip to top, place on front conveyor.
        Called both from the full sequence (after conveyor pickup) and from test mode."""
        self.target_pip = target_pip
        self.flip_count = 0
        dp = DICE_PLACE
        hover_z = dp['z'] + 2 * DICE_WIDTH

        # ── Scan 1: top face ──────────────────────────────────────────────────
        self._move_cart(dp['x'], dp['y'] + 3*DICE_WIDTH, hover_z, dp['w'], dp['p'], dp['r'])  # retreat for photo
        scan_top, _ = self._capture_face(dp['x'], dp['y'], dp['z'], 'top')
        if not self._is_valid_pip(scan_top, 'Scan 1'):
            return False

        if scan_top == target_pip:
            self.get_logger().info(f'Pip {target_pip} already on top — skipping flip and scan 2.')
            if target_pip == 6:
                self._place_at_dice_end()
            else:
                self._place_on_front_conveyor()
            self.flip_log[target_pip] = self.flip_count
            self._publish_flip_log()
            self._publish_done_state(target_pip)
            return True

        self._move_cart(dp['x'], dp['y'], hover_z, dp['w'], dp['p'], dp['r'])  # goto hover

        # ── X-flip to expose adjacent face ───────────────────────────────────
        self._rotate_x_neg_90()

        # ── Scan 2: left/adjacent face (now on top after flip) ────────────────
        fg = FLIP_X_GRAB
        scan_left, _ = self._capture_face(fg['x'], fg['y'], fg['z'], 'left')
        if not self._is_valid_pip(scan_left, 'Scan 2'):
            return False

        # ── Orient target pip on top (camera-verified) ────────────────────────
        if not self._orient_pip(target_pip, scan_top, scan_left):
            self.get_logger().error(f'Cannot orient pip {target_pip} — aborting cycle.')
            return False

        # ── Pre-conveyor verification scan ────────────────────────────────────
        verify_count, _ = self._capture_face(dp['x'], dp['y'], dp['z'], 'verify_top')
        if verify_count != target_pip:
            self.get_logger().error(
                f'Pre-conveyor verification FAILED — expected pip {target_pip} on top, '
                f'camera sees {verify_count}. Aborting placement.')
            return False
        self.get_logger().info(f'Pre-conveyor verification: pip {target_pip} confirmed on top.')

        # ── Pick up and place ─────────────────────────────────────────────────
        if target_pip == 6:
            self._place_at_dice_end()
        else:
            self._place_on_front_conveyor()
        self.flip_log[target_pip] = self.flip_count
        self._publish_flip_log()
        self._publish_done_state(target_pip)
        return True

    def _execute_sequence(self, target_pip):
        self._publish_location('rotating')

        # ── Conveyor pickup ───────────────────────────────────────────────────
        hs = CONVEYOR_HARD_STOP
        self.get_logger().info('Moving to conveyor hard stop...')
        self._move_cart(hs['x'], hs['y'], hs['z'], hs['w'], hs['p'], hs['r'])
        self._onrobot('close')

        self._publish_location('at_back_conveyer')
        self._wait_for_robot1('conveyer_done')
        self.get_logger().info('Conveyor done — grabbing die.')

        cp = CONVEYOR_PICK
        self._onrobot('open', delay=3.0)
        self._move_cart(cp['x'], cp['y'], cp['z'], cp['w'], cp['p'], cp['r'])
        self._onrobot('close', width=80)

        hover_z = cp['z'] + 2 * DICE_WIDTH
        self._move_cart(cp['x'], cp['y'], hover_z, cp['w'], cp['p'], cp['r'])

        # ── Transport to DICE_PLACE ───────────────────────────────────────────
        dp = DICE_PLACE
        self._move_cart(dp['x'], dp['y'], hover_z, dp['w'], dp['p'], dp['r'])  # transit at height
        self._move_cart(dp['x'], dp['y'], dp['z'],  dp['w'], dp['p'], dp['r'])  # lower
        self._onrobot('open')

        self._scan_orient_place(target_pip)

        if target_pip == 6:
            return  # End of run — robot 1 won't pick up; main loop will home + exit

        # ── Wait for Robot 1 at front conveyor, then run it ──────────────────
        self.robot1_location = ''
        self.get_logger().info('Waiting for Robot 1 at front conveyor hard stop...')
        self._wait_for_robot1('at_front_conveyer')
        self.robot1_location = ''
        self.get_logger().info('Robot 1 in position — running front conveyor 10 seconds...')
        self._conveyor('reverse')
        import time; time.sleep(10)
        self._conveyor('stop')
        self._publish_location('front_conveyer_done')

        # ── Return to hard stop — pre-positioned for next pickup ─────────────
        hs = CONVEYOR_HARD_STOP
        self.get_logger().info('Returning to hard stop — ready for next cycle.')
        self._move_cart(hs['x'], hs['y'], hs['z'], hs['w'], hs['p'], hs['r'])
        self._onrobot('close')

    # ── Main loop ─────────────────────────────────────────────────────────────

    def run(self):
        self._set_speed()
        self.go_home()

        for target_pip in (2, 4, 6):
            self.get_logger().info(f'Waiting for Robot 1 — pip {target_pip} cycle...')
            self._wait_for_robot1('on_conveyer')
            self.robot1_location = ''
            self._execute_sequence(target_pip)

        # ── Run complete (pip 6 placed) — print summary, return home, exit ────
        self._print_summary()
        self.go_home()


def main(args=None):
    rclpy.init(args=args)
    node = Robot2Node()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


def main_rotate(args=None):
    """Assume die is already at DICE_PLACE; run all five rotation primitives."""
    rclpy.init(args=args)
    node = Robot2Node()
    node._set_speed()

    node.go_home()

    de = DICE_PLACE
    node._move_cart(de['x'], de['y'], de['z'], de['w'], de['p'], de['r'])  # lower to dice end

    node.get_logger().info('Rotating about x positive 90 degrees')
    node._rotate_x_pos_90()

    node.get_logger().info('Rotating about x negative 90 degrees')
    node._rotate_x_neg_90()

    node.get_logger().info('Rotating about y negative 90 degrees')
    node._rotate_y_neg_90()

    node.get_logger().info('Rotating about y positive 90 degrees')
    node._rotate_y_pos_90()

    node.get_logger().info('Flipping about x 180 degrees')
    node._flip_x_180()

    

    node.go_home()
    node.get_logger().info('--- Rotate test complete ---')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
