#!/usr/bin/env python3
"""

Scan method:
  1. Scan top face
  2. Rotate +90° about x  (exposes adjacent face)
  3. Scan new top face  (originally the right face)
  (top, right) → lookup table → single rotation to bring pip 1,3, or 5 to top

State published to /dual_fanuc/robot1_dice_location:
  "rotating"         — actively manipulating die
  "at_front_conveyer" — in hard-stop position, ready for Robot 2 to run conveyor
  "on_conveyer"      — die placed on conveyor, Robot 2 can grab

Standard die chirality: opposite faces sum to 7; with 1 on top and 2 facing viewer, 3 is on the right.
"""

import json
import os
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from fanuc_interfaces.action import CartPose, JointPose, SchunkGripper, Conveyor
from fanuc_interfaces.srv import SetSpeed

from dual_fanuc.pip_lookup import get_action

from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
import cv2
import numpy as np


# ══════════════════════════════════════════════════════════════════════════════
#  Configuration
# ══════════════════════════════════════════════════════════════════════════════

ROBOT_NAME   = os.environ.get('ROBOT_1_NAME', 'robot1')
ROBOT_2_NAME = os.environ.get('ROBOT_2_NAME', 'robot2')
ROBOT_SPEED  = 300

DICE_WIDTH        = 80.0
APPROACH_OFFSET_Z = 2 * DICE_WIDTH   # 160 mm

# Die pick position
DICE_PICK = {'x': 469.0, 'y': -15.4, 'z': -178.5, 'w': 179.9, 'p': 0.0, 'r': 30.0}

# Place position — die set down here for camera and all flip operations
DICE_PLACE = {'x': 540.0, 'y': 399.12, 'z': -114.976, 'w': -179.9, 'p': 0.0, 'r': 120.0}

# Conveyor drop position (back conveyor — odd pips)
CONVEYOR_PLACE = {'x': -191.905, 'y': 667.664, 'z': 17.884, 'w': -179.9, 'p': 0.0, 'r': 120.0}

# Front conveyor — hard stop where Robot 1 waits, and pick position
FRONT_CONVEYOR_HARD_STOP = {'x': 144.786, 'y': 669.906,  'z': -13.595, 'w': 179.9, 'p': 0.0, 'r': 120.0}
CONVEYOR_PICK_FRONT      = {'x': 144.786, 'y': 725.969,  'z':  14.437, 'w': 179.9, 'p': 0.0, 'r': 120.0}

# Absolute side-grip position: die released here falls back to DICE_PLACE
FLIP_X_GRAB = {'x': 540.0, 'y': 399.12, 'z': -143.407, 'w': 87.276, 'p': -59.453, 'r': -177.638}

RETREAT_Z = DICE_WIDTH        # lift up 80 mm after releasing
HOVER_WPR = (DICE_PLACE['w'], DICE_PLACE['p'], DICE_PLACE['r'])  # flat orientation above die

CAMERA_TOPIC      = '/mv_camera/image_raw'
R1_LOCATION_TOPIC = '/dual_fanuc/robot1_dice_location'
R2_LOCATION_TOPIC = '/dual_fanuc/robot2_dice_location'
IMAGE_SAVE_DIR    = os.path.expanduser('~/ClaudeDualFanuc2/tmp/')

HSV_H_LOW  = 14;  HSV_S_LOW  = 100;  HSV_V_LOW  = 100
HSV_H_HIGH = 35;  HSV_S_HIGH = 255;  HSV_V_HIGH = 255
MIN_CONTOUR_AREA = 2500

# Region of interest applied before HSV detection (camera-pixel coords).
ROI_X1 = 0
ROI_Y1 = 361
ROI_X2 = 1280
ROI_Y2 = 860

HOUGH_DP       = 1.0
HOUGH_MIN_DIST = 8
HOUGH_PARAM1   = 8
HOUGH_PARAM2   = 14
HOUGH_MIN_R    = 0
HOUGH_MAX_R    = 8

VALID_PIP_COUNTS = frozenset(range(1, 7))
XFLIP_Y_OFFSET   = 32.0  # mm die slides in +y after each x-flip


# ══════════════════════════════════════════════════════════════════════════════
#  Node
# ══════════════════════════════════════════════════════════════════════════════

class Robot1Node(Node):

    def __init__(self):
        super().__init__('robot1')

        ns = ROBOT_NAME
        self.cart_ac      = ActionClient(self, CartPose,      f'/{ns}/cartesian_pose')
        self.joints_ac    = ActionClient(self, JointPose,     f'/{ns}/joint_pose')
        self.schunk_ac    = ActionClient(self, SchunkGripper, f'/{ns}/schunk_gripper')
        self.conveyor_back_ac  = ActionClient(self, Conveyor, f'/{ns}/conveyor')            # odd  pips → Robot 2
        self.conveyor_front_ac = ActionClient(self, Conveyor, f'/{ROBOT_2_NAME}/conveyor')  # even pips → Robot 1
        self.speed_client = self.create_client(SetSpeed,      f'/{ns}/set_speed')

        self.bridge       = CvBridge()
        self.latest_image = None
        self.create_subscription(Image, CAMERA_TOPIC, self._image_cb, 10)
        self.capture_client = self.create_client(Trigger, '/mv_camera/capture')

        self.die_place = dict(DICE_PLACE)

        self.r1_location_pub  = self.create_publisher(String, R1_LOCATION_TOPIC, 10)
        self.robot2_location  = ''
        self.create_subscription(String, R2_LOCATION_TOPIC, self._r2_location_cb, 10)

        # Flip-count tracking (count of physical die rotations per oriented pip; incremented in _rotate_x_pos_90)
        self.target_pip   = None
        self.flip_count   = 0
        self.flip_log     = {}
        self.flip_log_pub = self.create_publisher(String, f'/{ns}/flip_log', 10)

        self.get_logger().info('Waiting for action servers and camera...')
        # Essential for any operation — block until available.
        self.cart_ac.wait_for_server()
        self.joints_ac.wait_for_server()
        self.schunk_ac.wait_for_server()
        # Optional — only needed for full pipeline. Warn and continue so rotate-only tests work
        # without launch2 / a working camera.
        for ac, label in (
            (self.conveyor_back_ac,  f'/{ns}/conveyor (back)'),
            (self.conveyor_front_ac, f'/{ROBOT_2_NAME}/conveyor (front)'),
        ):
            if not ac.wait_for_server(timeout_sec=5.0):
                self.get_logger().warn(f'Action server not available after 5 s: {label} — continuing.')
        if not self.capture_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn('Camera capture service /mv_camera/capture not available after 5 s — continuing.')
        self.get_logger().info(f'Robot 1 ready — namespace: /{ns}')

    # ── Camera ────────────────────────────────────────────────────────────────

    def _r2_location_cb(self, msg: String):
        self.robot2_location = msg.data

    def _publish_location(self, state: str):
        msg = String()
        msg.data = state
        self.r1_location_pub.publish(msg)
        self.get_logger().info(f'robot1_dice_location: {state}')

    def _wait_for_robot2(self, *states: str):
        """Wait until robot2_location matches any given state. Returns the matched state."""
        label = ' or '.join(states)
        self.get_logger().info(f'Waiting for robot2_dice_location: {label}')
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.robot2_location in states:
                return self.robot2_location

    def _publish_flip_log(self):
        msg = String()
        msg.data = json.dumps({str(k): v for k, v in self.flip_log.items()})
        self.flip_log_pub.publish(msg)

    def _print_summary(self):
        total = sum(self.flip_log.values())
        lines = ['═══ Robot 1 flip summary ═══']
        for pip in sorted(self.flip_log):
            lines.append(f'  pip {pip}: {self.flip_log[pip]} flip(s)')
        lines.append(f'  Robot 1 total: {total}')
        for line in lines:
            self.get_logger().info(line)

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

    def _schunk(self, command):
        goal = SchunkGripper.Goal()
        goal.command = command
        res = self._send_goal(self.schunk_ac, goal)
        return res is not None and res.success

    def _conveyor(self, command, pip):
        """Send command to back conveyor (odd pip) or front conveyor (even pip)."""
        ac    = self.conveyor_back_ac  if pip % 2 != 0 else self.conveyor_front_ac
        label = 'back'                 if pip % 2 != 0 else 'front'
        self.get_logger().info(f'Conveyor {label} ({command}) — pip {pip}')
        goal = Conveyor.Goal()
        goal.command = command
        res = self._send_goal(ac, goal)
        return res is not None and res.success

    def _place_on_conveyor(self, pip):
        """Pick die from DICE_PLACE and place on back (odd pip) or front (even pip) conveyor."""
        cp    = CONVEYOR_PLACE
        pl    = self.die_place
        label = 'back' if pip % 2 != 0 else 'front'

        lift_z = pl['z'] + 3 * DICE_WIDTH   # clearance for transit to conveyor

        self.get_logger().info(f'Placing pip {pip} on {label} conveyor.')
        self._schunk('open')
        self._move_cart(pl['x'], pl['y'], pl['z'] + APPROACH_OFFSET_Z, pl['w'], pl['p'], pl['r'])  # approach
        self._move_cart(pl['x'], pl['y'], pl['z'],                      pl['w'], pl['p'], pl['r'])  # grab
        self._schunk('close')
        self._move_cart(pl['x'], pl['y'], lift_z,                       pl['w'], pl['p'], pl['r'])  # lift high
        self._move_cart(cp['x'], cp['y'], lift_z,                       cp['w'], cp['p'], cp['r'])  # transit above conveyor
        self._move_cart(cp['x'], cp['y'], cp['z'],                      cp['w'], cp['p'], cp['r'])  # lower to conveyor
        self._schunk('open')
        self._move_cart(cp['x'], cp['y'], cp['z'] + RETREAT_Z,         cp['w'], cp['p'], cp['r'])  # retreat

        self._publish_location('on_conveyer')
        self._wait_for_robot2('at_back_conveyer')
        self.get_logger().info('Robot 2 in position — running conveyor 10 seconds...')
        self._conveyor('forward', pip)
        import time; time.sleep(10)
        self._conveyor('stop', pip)
        self._publish_location('conveyer_done')
        self.get_logger().info('Done.')

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
        self._move_joints(0, 0, 0, 0, -90, 30)
        self._schunk('open')

    # ── X-axis flip ───────────────────────────────────────────────────────────
    # Top-down pick from current die position, transit to absolute FLIP_X_GRAB, release.
    # Die tips over and lands ~XFLIP_Y_OFFSET (+y) from DICE_PLACE — self.die_place is updated to track it.

    def _rotate_x_pos_90(self):
        """Rotate die +90° about x: pick top-down at current die position, carry to FLIP_X_GRAB, release.
        Updates self.die_place y by XFLIP_Y_OFFSET after each flip.
        Counts as one physical flip; wrappers (flip_x_180, rotate_x_neg_90, y rotates) compose."""
        self.flip_count += 1
        dp = self.die_place
        fg = FLIP_X_GRAB
        dw, dpw, dr = dp['w'], dp['p'], dp['r']

        self._move_cart(dp['x'], dp['y'], dp['z'] + 2*DICE_WIDTH, dw, dpw, dr)              # hover above
        self._schunk('open')
        self._move_cart(dp['x'], dp['y'], dp['z'],               dw, dpw, dr)               # lower to grab
        self._schunk('close')
        self._move_cart(dp['x'], dp['y'], dp['z'] + DICE_WIDTH,  dw, dpw, dr)               # lift
        self._move_cart(fg['x'], fg['y'], fg['z'] + DICE_WIDTH,  fg['w'], fg['p'], fg['r']) # transit + rotate wrist
        self._move_cart(fg['x'], fg['y'], fg['z'],               fg['w'], fg['p'], fg['r']) # lower to flip position
        self._schunk('open')
        self.die_place = dict(DICE_PLACE, y=DICE_PLACE['y'] + XFLIP_Y_OFFSET)              # die slides +y by XFLIP_Y_OFFSET after each x-flip
        self._move_cart(fg['x'], fg['y'] - DICE_WIDTH, fg['z'],  fg['w'], fg['p'], fg['r']) # retreat −y

    def _rotate_x_neg_90(self):
        """Rotate die −90° about x via three consecutive +90° rotations."""
        self.get_logger().info('Rotate x−90: step 1 of 3')
        self._rotate_x_pos_90()
        self.get_logger().info('Rotate x−90: step 2 of 3')
        self._rotate_x_pos_90()
        self.get_logger().info('Rotate x−90: step 3 of 3')
        self._rotate_x_pos_90()

    def _flip_x_180(self):
        """Flip die 180° about x via two +90° rotations."""
        self.get_logger().info('Flip x-axis: step 1 of 2')
        self._rotate_x_pos_90()
        self.get_logger().info('Flip x-axis: step 2 of 2')
        self._rotate_x_pos_90()
        self.get_logger().info('Flip x-axis: complete')

    # ── Y-axis flip (J6 rotation then single X-pos-90 primitive) ─────────────
    # Yaw the die 90° about Z using J6, lower to table, then do a single _rotate_x_pos_90.
    # Net effect: ±90° about y. The trailing x-flip slides the die +y by XFLIP_Y_OFFSET (tracked in self.die_place).

    def _rotate_y_neg_90(self):
        """Rotate die −90° about y: J6 +90° at current die position, then _rotate_x_pos_90."""
        dp = self.die_place
        dw, dpw, dr = dp['w'], dp['p'], dp['r']
        dr_rot = (dr + 90.0 + 180.0) % 360.0 - 180.0

        self._move_cart(dp['x'], dp['y'], dp['z'] + 2*DICE_WIDTH, dw, dpw, dr)
        self._schunk('open')
        self._move_cart(dp['x'], dp['y'], dp['z'],                dw, dpw, dr)     # lower to grab
        self._schunk('close')
        self._move_cart(dp['x'], dp['y'], dp['z'] + DICE_WIDTH,   dw, dpw, dr)     # lift
        self._move_cart(dp['x'], dp['y'], dp['z'] + DICE_WIDTH,   dw, dpw, dr_rot) # J6 +90°
        self._move_cart(dp['x'], dp['y'], dp['z'],                dw, dpw, dr_rot) # lower
        self._schunk('open')
        self._move_cart(dp['x'], dp['y'], dp['z'] + DICE_WIDTH,   dw, dpw, dr_rot)
        self._move_cart(dp['x'], dp['y'], dp['z'] + 2*DICE_WIDTH, dw, dpw, dr)     # J6 back
        self._rotate_x_pos_90()

    def _rotate_y_pos_90(self):
        """Rotate die +90° about y: J6 −90° at current die position, then _rotate_x_pos_90."""
        dp = self.die_place
        dw, dpw, dr = dp['w'], dp['p'], dp['r']
        dr_rot = (dr - 90.0 + 180.0) % 360.0 - 180.0

        self._move_cart(dp['x'], dp['y'], dp['z'] + 2*DICE_WIDTH, dw, dpw, dr)
        self._schunk('open')
        self._move_cart(dp['x'], dp['y'], dp['z'],                dw, dpw, dr)     # lower to grab
        self._schunk('close')
        self._move_cart(dp['x'], dp['y'], dp['z'] + DICE_WIDTH,   dw, dpw, dr)     # lift
        self._move_cart(dp['x'], dp['y'], dp['z'] + DICE_WIDTH,   dw, dpw, dr_rot) # J6 −90°
        self._move_cart(dp['x'], dp['y'], dp['z'],                dw, dpw, dr_rot) # lower
        self._schunk('open')
        self._move_cart(dp['x'], dp['y'], dp['z'] + DICE_WIDTH,   dw, dpw, dr_rot)
        self._move_cart(dp['x'], dp['y'], dp['z'] + 2*DICE_WIDTH, dw, dpw, dr)     # J6 back
        self._rotate_x_pos_90()

    def _flip_y_180(self):
        """Flip die 180° about y via two −90° rotations."""
        self.get_logger().info('Flip y-axis: step 1 of 2')
        self._rotate_y_neg_90()
        self.get_logger().info('Flip y-axis: step 2 of 2')
        self._rotate_y_neg_90()
        self.get_logger().info('Flip y-axis: complete')

    # ── Main sequence ─────────────────────────────────────────────────────────

    def _pick_and_place(self):
        """Pick die from DICE_PICK, place at DICE_PLACE, retreat for camera."""
        pk = DICE_PICK
        pl = DICE_PLACE

        self._schunk('open')
        self._move_cart(pk['x'], pk['y'], pk['z'] + APPROACH_OFFSET_Z,
                        pk['w'], pk['p'], pk['r'])
        self._move_cart(pk['x'], pk['y'], pk['z'],
                        pk['w'], pk['p'], pk['r'])
        self._schunk('close')
        self._move_cart(pk['x'], pk['y'], pk['z'] + APPROACH_OFFSET_Z,
                        pk['w'], pk['p'], pk['r'])
        self._move_cart(pl['x'], pl['y'], pl['z'] + APPROACH_OFFSET_Z,
                        pl['w'], pl['p'], pl['r'])
        self._move_cart(pl['x'], pl['y'], pl['z'],
                        pl['w'], pl['p'], pl['r'])
        self._schunk('open')
        self._move_cart(pl['x'], pl['y'], pl['z'] + RETREAT_Z,
                        pl['w'], pl['p'], pl['r'])

    def _capture_face(self, die_x, die_y, die_z, label='Face'):
        """Hover → retreat −Y → capture → count pips (logged immediately) → return to hover. Returns (pip_count, debug)."""
        self._move_cart(die_x, die_y,                   die_z + RETREAT_Z, *HOVER_WPR)  # hover
        self._move_cart(die_x, die_y - 3 * DICE_WIDTH,  die_z + RETREAT_Z, *HOVER_WPR)  # retreat
        image = self._capture_image()
        if image is None:
            self.get_logger().error(f'{label}: camera capture failed.')
            self._move_cart(die_x, die_y, die_z + RETREAT_Z, *HOVER_WPR)  # return to hover
            return 0, None
        slug = label.lower().replace(' ', '_')
        cv2.imwrite(f'{IMAGE_SAVE_DIR}{slug}_full.png', image)
        cropped = self._crop_die(image)
        pip_count, debug = self._count_pips(cropped)
        self.get_logger().info(f'Expected Pip: {self.target_pip} ---- Saw Pip: {pip_count}')
        self._move_cart(die_x, die_y,                   die_z + RETREAT_Z, *HOVER_WPR)  # return to hover
        return pip_count, debug

    def _is_valid_pip(self, count, label):
        if count not in VALID_PIP_COUNTS:
            self.get_logger().error(
                f'{label}: invalid pip count {count} (must be 1–6). '
                f'Check camera, lighting, or Hough calibration. Aborting.')
            return False
        return True

    def _save_debug(self, debug_imgs):
        for name, img in debug_imgs.items():
            if img is not None:
                cv2.imwrite(f'{IMAGE_SAVE_DIR}{name}.png', img)

    def _grab_from_front_conveyor(self):
        """Pick die from front conveyor (placed by Robot 2), transport to DICE_PLACE."""
        cp  = CONVEYOR_PICK_FRONT
        pl  = DICE_PLACE
        fhs = FRONT_CONVEYOR_HARD_STOP
        hover_z = cp['z'] + 2 * DICE_WIDTH

        self._schunk('open')
        self._move_cart(fhs['x'], fhs['y'] - DICE_WIDTH, fhs['z'],  fhs['w'], fhs['p'], fhs['r'])  # retreat −y
        self._move_cart(fhs['x'], fhs['y'] - DICE_WIDTH, hover_z,   fhs['w'], fhs['p'], fhs['r'])  # rise up
        self._move_cart(cp['x'],  cp['y'],               hover_z,   cp['w'],  cp['p'],  cp['r'])   # above die
        self._move_cart(cp['x'],  cp['y'],               cp['z'],   cp['w'],  cp['p'],  cp['r'])   # lower to pick
        self._schunk('close')
        self._move_cart(cp['x'],  cp['y'],               hover_z,   cp['w'],  cp['p'],  cp['r'])   # rise
        self._move_cart(pl['x'],  pl['y'],               hover_z,   pl['w'],  pl['p'],  pl['r'])   # transit
        self._move_cart(pl['x'],  pl['y'],               pl['z'],   pl['w'],  pl['p'],  pl['r'])   # lower
        self._schunk('open')
        self._move_cart(pl['x'],  pl['y'],               pl['z'] + RETREAT_Z, pl['w'],  pl['p'],  pl['r'])

    def _scan_and_orient(self, target_pip):
        """Two-scan orient: scan top, x-flip, scan again, lookup action, apply. Returns True on success."""
        self.target_pip = target_pip
        self.flip_count = 0
        self.die_place = dict(DICE_PLACE)  # die freshly placed at DICE_PLACE; reset tracker
        x, y, z = DICE_PLACE['x'], DICE_PLACE['y'], DICE_PLACE['z']
        debug_imgs = {}

        # ── Scan 1: top face ──────────────────────────────────────────────────
        face_top, dbg = self._capture_face(x, y, z, 'scan1_top')
        debug_imgs['scan1_top'] = dbg
        if not self._is_valid_pip(face_top, 'Scan 1'):
            self._save_debug(debug_imgs)
            return False

        if face_top == target_pip:
            self.get_logger().info(f'Pip {target_pip} already on top — no rotation needed.')
            self.flip_log[target_pip] = self.flip_count
            self._publish_flip_log()
            self._save_debug(debug_imgs)
            return True

        # ── X-flip to expose adjacent face ────────────────────────────────────
        self._publish_location('rotating')
        self._rotate_x_pos_90()

        # ── Scan 2: face now on top after x-flip ──────────────────────────────
        face_left, dbg = self._capture_face(x, y, z, 'scan2_left')
        debug_imgs['scan2_left'] = dbg
        if not self._is_valid_pip(face_left, 'Scan 2'):
            self._save_debug(debug_imgs)
            return False

        # ── Lookup + apply corrective rotation ────────────────────────────────
        action = get_action(target_pip, face_top, face_left)
        if action is None:
            self.get_logger().error(
                f'No lookup entry for pip {target_pip}: top={face_top} left={face_left}. '
                f'Check detection or die chirality.')
            self._save_debug(debug_imgs)
            return False

        self.get_logger().info(
            f'Pip {target_pip} — orientation ({face_top},{face_left}) → action: {action}')

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

        # ── Pre-conveyor verification scan ────────────────────────────────────
        dp = self.die_place
        verify_count, dbg = self._capture_face(dp['x'], dp['y'], dp['z'], 'verify_top')
        debug_imgs['verify_top'] = dbg
        if verify_count != target_pip:
            self.get_logger().error(
                f'Pre-conveyor verification FAILED — expected pip {target_pip} on top, '
                f'camera sees {verify_count}. Aborting placement.')
            self._save_debug(debug_imgs)
            return False
        self.get_logger().info(f'Pre-conveyor verification: pip {target_pip} confirmed on top.')

        self.flip_log[target_pip] = self.flip_count
        self._publish_flip_log()
        self._save_debug(debug_imgs)
        return True

    def run(self):
        self._set_speed()
        self.go_home()
        self._publish_location('home')

        while rclpy.ok():
            # ── Pip 1: pick fresh die from pile ───────────────────────────────
            self._pick_and_place()
            if not self._scan_and_orient(1):
                self.get_logger().error('Failed to orient pip 1 — restarting cycle.')
                continue
            self._place_on_conveyor(1)   # → back conveyor, runs conveyor, publishes "conveyer_done"

            # ── Pip 3: Robot 2 placed pip 2 on front conveyor ────────────────
            self.robot2_location = ''
            self._wait_for_robot2('on_conveyer')
            self.robot2_location = ''
            fhs = FRONT_CONVEYOR_HARD_STOP
            self.get_logger().info('Moving to front conveyor hard stop...')
            self._move_cart(fhs['x'], fhs['y'], fhs['z'], fhs['w'], fhs['p'], fhs['r'])
            self._schunk('close')
            self._publish_location('at_front_conveyer')
            self._wait_for_robot2('front_conveyer_done')
            self.robot2_location = ''
            self._grab_from_front_conveyor()
            if not self._scan_and_orient(3):
                self.get_logger().error('Failed to orient pip 3 — restarting cycle.')
                continue
            self._place_on_conveyor(3)   # → back conveyor

            # ── Pip 5: Robot 2 placed pip 4 on front conveyor ────────────────
            self.robot2_location = ''
            self._wait_for_robot2('on_conveyer')
            self.robot2_location = ''
            self.get_logger().info('Moving to front conveyor hard stop...')
            self._move_cart(fhs['x'], fhs['y'], fhs['z'], fhs['w'], fhs['p'], fhs['r'])
            self._schunk('close')
            self._publish_location('at_front_conveyer')
            self._wait_for_robot2('front_conveyer_done')
            self.robot2_location = ''
            self._grab_from_front_conveyor()
            if not self._scan_and_orient(5):
                self.get_logger().error('Failed to orient pip 5 — restarting cycle.')
                continue
            self._place_on_conveyor(5)   # → back conveyor

            # ── Wait for Robot 2 to finish pip 6 (or signal run_complete) ────
            self.robot2_location = ''
            matched = self._wait_for_robot2('on_conveyer', 'run_complete')
            self.robot2_location = ''
            if matched == 'run_complete':
                self.get_logger().info('Robot 2 signalled run_complete — finishing run.')
                self._print_summary()
                self.go_home()
                break
            self.get_logger().info('Full 6-pip cycle complete — starting next die.')


def main(args=None):
    rclpy.init(args=args)
    node = Robot1Node()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


def main_rotate(args=None):
    """Pick die from home, place at DICE_PLACE, then run all five rotation primitives."""
    rclpy.init(args=args)
    node = Robot1Node()
    node._set_speed()

    node.go_home()
    node._pick_and_place()

    node.get_logger().info('Rotating about x positive 90 degrees')
    node._rotate_x_pos_90()

    node.get_logger().info('Rotating about y positive 90 degrees')
    node._rotate_y_pos_90()

    node.get_logger().info('Rotating about x negative 90 degrees')
    node._rotate_x_neg_90()

    node.get_logger().info('Rotating about y negative 90 degrees')
    node._rotate_y_neg_90()

    node.get_logger().info('Flipping about x 180 degrees')
    node._flip_x_180()

    node.go_home()
    node.get_logger().info('--- Rotate test complete ---')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
