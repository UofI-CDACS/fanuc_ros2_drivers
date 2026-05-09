"""
RobotTaskBase — shared infrastructure for both robot task nodes.

Two-conveyor design
-------------------
There are two physical conveyor belts:
  Odd conveyor  — R1 drops here; R2 picks up from here.
  Even conveyor — R2 drops here; R1 picks up from here.

Each robot has two conveyor action clients:
  _drop_conveyor_client    the belt this robot places dice on
  _pickup_conveyor_client  the belt this robot receives dice from

Conveyor handoff uses physical proximity sensors only — each robot polls its
own prox sensor to know when the dice has arrived.  No Modbus coil signalling
is needed between robots for conveyor events.

Camera access is still coordinated via Modbus (shared camera, one at a time).

Robot 1 full loop
-----------------
  [cycle 0]  HOME → pick dice → camera loop → drop on odd conveyor → start odd
             → HOME → wait own sensor (even) → stop even → pick up → ...
  [cycle 1]  ... camera loop → drop on odd → start odd → HOME → wait own sensor
             → stop even → pick up → ...
  [cycle 2]  ... camera loop → drop on odd → start odd → FINAL_HOME (done)

Robot 2 full loop
-----------------
  [cycle 0]  HOME → wait own sensor (odd) → stop odd → pick up → camera loop
             → drop on even → start even → HOME → wait own sensor (odd) → ...
  [cycle 1]  ... camera loop → drop on even → start even → HOME → wait own
             sensor (odd) → stop odd → pick up → ...
  [cycle 2]  ... camera loop → place at dice_pose → FINAL_HOME (done)
"""

import asyncio
import os as _os
import sys as _sys
import threading
import time

import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from ament_index_python.packages import get_package_share_directory

from fanuc_interfaces.action import CartPose, JointPose, OnRobotGripper, Conveyor
from fanuc_interfaces.msg import ProxReadings

# Add fanuc_ros2_drivers root so dice_vision is importable when running from
# the source tree.  In installed mode the launch environment (setup_ws.bash)
# sets PYTHONPATH to include that directory, so the path arithmetic — which
# lands in the install tree — is intentionally skipped when dice_vision is
# not present at the computed location.
_DRIVERS_ROOT = _os.path.abspath(
    _os.path.join(_os.path.dirname(_os.path.abspath(__file__)), '..', '..', '..', '..')
)
if _os.path.isdir(_os.path.join(_DRIVERS_ROOT, 'dice_vision')):
    if _DRIVERS_ROOT not in _sys.path:
        _sys.path.insert(0, _DRIVERS_ROOT)

from dice_vision.dice_model import from_visible_faces
from dice_vision.rotation_planner import plan_to_value
from robot_task.modbus_client import RobotModbusClient
from robot_task.states import State
from modbus_server.register_map import (
    ROBOT_STATE_IDLE,
    ROBOT_STATE_HOMING,
    ROBOT_STATE_PICKING,
    ROBOT_STATE_TO_CAMERA,
    ROBOT_STATE_INSPECTING,
    ROBOT_STATE_TO_CONVEYOR,
    ROBOT_STATE_RETRY,
    ROBOT_STATE_COMPLETE,
    ROBOT_STATE_ERROR,
)


class RobotTaskBase(Node):

    # Subclasses must assign these before calling super().__init__
    _PIP_SEQUENCE           = []
    _FIRST_STATE_AFTER_HOME = State.OPEN_GRIPPER

    def __init__(self, node_name: str):
        super().__init__(node_name)
        self._declare_parameters()
        self._create_action_clients()
        self._create_subscriptions()

        self.state                = State.IDLE
        self._cycle               = 0
        self._retry_count         = 0
        self._last_pip_count      = 0
        self._cart_distance_left  = 0.0
        self._shutdown_event      = threading.Event()
        self._latest_prox         = ProxReadings()
        self._current_pickup_pose = {}

        # ── Camera-pass state (reset each cycle) ──────────────────────────
        # _camera_pass: 0=first read (face_1), 1=second read (face_2), 2=final confirm
        self._camera_pass          = 0
        self._face1                = 0    # pip count from first camera read
        self._rotation_queue: list = []   # BFS-planned rotation names, consumed by ROTATE_DICE
        self._current_rotation     = ''   # rotation name currently being executed
        self._post_dice_pick_state = State.SAFE_HOME_TO_CAMERA  # where ASCEND_FROM_DICE goes

        self._travel_z        = 60.0
        self._rotation_steps  = {}
        self._load_rotation_config()

        self._modbus = RobotModbusClient(
            host=self.get_parameter('modbus_host').value,
            port=self.get_parameter('modbus_port').value,
            robot_index=self.get_parameter('robot_index').value,
        )

        self.get_logger().info(
            f"{node_name} ready — "
            f"robot_index={self.robot_index}  pip_sequence={self._PIP_SEQUENCE}"
        )

    # ------------------------------------------------------------------
    # Parameters
    # ------------------------------------------------------------------

    def _declare_parameters(self):
        self.declare_parameter('robot_name',             'bunsen')
        self.declare_parameter('robot_index',            1)
        self.declare_parameter('prox_sensor_side',       'left')     # 'left' or 'right'
        self.declare_parameter('approach_height_offset', 100.0)
        self.declare_parameter('camera_settle_time',     1.0)
        self.declare_parameter('capture_timeout',        10.0)
        self.declare_parameter('camera_poll_interval',   0.5)
        self.declare_parameter('camera_claim_delay',     0.1)
        self.declare_parameter('gripper_settle_time',    2.0)
        self.declare_parameter('max_retries',            6)
        self.declare_parameter('sensor_poll_interval',   0.2)
        self.declare_parameter('modbus_host',            'localhost')
        self.declare_parameter('modbus_port',            1502)

        # Conveyor namespaces — each robot has a dedicated drop and pickup belt
        self.declare_parameter('drop_conveyor_ns',    'bunsen')  # namespace of this robot's drop belt
        self.declare_parameter('pickup_conveyor_ns',  'beaker')  # namespace of this robot's pickup belt
        self.declare_parameter('drop_conveyor_cmd',   'forward') # command to start the drop belt

        # Dice pose — R1: initial pickup; R2: rotation setdown surface
        self.declare_parameter('dice_x',   454.079)
        self.declare_parameter('dice_y',    -1.565)
        self.declare_parameter('dice_z',  -116.954)
        self.declare_parameter('dice_w',   179.0)
        self.declare_parameter('dice_p',     0.0)
        self.declare_parameter('dice_r',     0.0)

        # Conveyor pickup pose (where this robot picks dice off the belt)
        self.declare_parameter('conveyor_pickup_x',  300.0)
        self.declare_parameter('conveyor_pickup_y',  500.0)
        self.declare_parameter('conveyor_pickup_z', -100.0)
        self.declare_parameter('conveyor_pickup_w',  180.0)
        self.declare_parameter('conveyor_pickup_p',    0.0)
        self.declare_parameter('conveyor_pickup_r',    0.0)

        # Conveyor drop pose (where this robot places dice on the belt)
        self.declare_parameter('conveyor_drop_x',  400.0)
        self.declare_parameter('conveyor_drop_y',  600.0)
        self.declare_parameter('conveyor_drop_z', -100.0)
        self.declare_parameter('conveyor_drop_w',  180.0)
        self.declare_parameter('conveyor_drop_p',    0.0)
        self.declare_parameter('conveyor_drop_r',    0.0)

        # Camera inspection pose
        self.declare_parameter('camera_x',  516.265)
        self.declare_parameter('camera_y', -963.244)
        self.declare_parameter('camera_z',  921.766)
        self.declare_parameter('camera_w',   -2.3)
        self.declare_parameter('camera_p',  -87.916)
        self.declare_parameter('camera_r',   90.260)

        # Gripper
        self.declare_parameter('gripper_open_width',  120.0)
        self.declare_parameter('gripper_close_width',  75.0)
        self.declare_parameter('gripper_force',        30.0)

        # Home joint pose
        self.declare_parameter('home_j1',   0.0)
        self.declare_parameter('home_j2',   0.0)
        self.declare_parameter('home_j3',   0.0)
        self.declare_parameter('home_j4',   0.0)
        self.declare_parameter('home_j5', -90.0)
        self.declare_parameter('home_j6',   0.0)

    # Convenience accessors

    @property
    def robot_index(self):
        return self.get_parameter('robot_index').value

    @property
    def dice_pose(self):
        return {k: self.get_parameter(f'dice_{k}').value for k in ('x','y','z','w','p','r')}

    @property
    def conveyor_pickup_pose(self):
        return {k: self.get_parameter(f'conveyor_pickup_{k}').value for k in ('x','y','z','w','p','r')}

    @property
    def conveyor_drop_pose(self):
        return {k: self.get_parameter(f'conveyor_drop_{k}').value for k in ('x','y','z','w','p','r')}

    @property
    def camera_pose(self):
        return {k: self.get_parameter(f'camera_{k}').value for k in ('x','y','z','w','p','r')}

    @property
    def home_joints(self):
        return [self.get_parameter(f'home_j{i}').value for i in range(1, 7)]

    @property
    def gripper_open_width(self):
        return self.get_parameter('gripper_open_width').value

    @property
    def gripper_close_width(self):
        return self.get_parameter('gripper_close_width').value

    @property
    def gripper_force(self):
        return self.get_parameter('gripper_force').value

    @property
    def approach_offset(self):
        return self.get_parameter('approach_height_offset').value

    @property
    def gripper_settle_time(self):
        return self.get_parameter('gripper_settle_time').value

    @property
    def _target_pip(self) -> int:
        return self._PIP_SEQUENCE[self._cycle]

    @property
    def _setdown_pose(self) -> dict:
        """
        Where the dice is placed for rotation.
        Default: same spot it was last picked up from.
        Robot2TaskNode overrides this to always return dice_pose.
        """
        return self._current_pickup_pose

    # ------------------------------------------------------------------
    # Action clients
    # ------------------------------------------------------------------

    def _create_action_clients(self):
        robot_ns     = self.get_parameter('robot_name').value
        drop_ns      = self.get_parameter('drop_conveyor_ns').value
        pickup_ns    = self.get_parameter('pickup_conveyor_ns').value
        self._cart_client             = ActionClient(self, CartPose,       f'/{robot_ns}/cartesian_pose')
        self._joint_client            = ActionClient(self, JointPose,      f'/{robot_ns}/joint_pose')
        self._gripper_client          = ActionClient(self, OnRobotGripper, f'/{robot_ns}/onrobot_gripper')
        self._drop_conveyor_client    = ActionClient(self, Conveyor,       f'/{drop_ns}/conveyor')
        self._pickup_conveyor_client  = ActionClient(self, Conveyor,       f'/{pickup_ns}/conveyor')

    def _create_subscriptions(self):
        pickup_ns = self.get_parameter('pickup_conveyor_ns').value
        self._prox_sub = self.create_subscription(
            ProxReadings,
            f'/{pickup_ns}/prox_readings',
            self._prox_callback,
            10,
        )

    def _prox_callback(self, msg: ProxReadings):
        self._latest_prox = msg

    @property
    def _own_sensor_active(self) -> bool:
        side = self.get_parameter('prox_sensor_side').value
        return bool(getattr(self._latest_prox, side, False))

    # ------------------------------------------------------------------
    # Async bridge
    # ------------------------------------------------------------------

    async def _await_future(self, rclpy_future):
        done = threading.Event()
        rclpy_future.add_done_callback(lambda _: done.set())
        loop = asyncio.get_running_loop()
        await loop.run_in_executor(None, done.wait)
        return rclpy_future.result()

    async def _wait_for_servers(self):
        self.get_logger().info("Waiting for action servers...")
        loop = asyncio.get_running_loop()
        await loop.run_in_executor(None, self._cart_client.wait_for_server)
        await loop.run_in_executor(None, self._joint_client.wait_for_server)
        await loop.run_in_executor(None, self._gripper_client.wait_for_server)
        await loop.run_in_executor(None, self._drop_conveyor_client.wait_for_server)
        await loop.run_in_executor(None, self._pickup_conveyor_client.wait_for_server)
        self.get_logger().info("All action servers ready")

    # ------------------------------------------------------------------
    # Movement helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _clamp_deg(val: float) -> float:
        return max(-179.9, min(179.9, val))

    async def _send_cart_pose(self, x, y, z, w=200.0, p=200.0, r=200.0):
        w, p, r = self._clamp_deg(w), self._clamp_deg(p), self._clamp_deg(r)
        self.get_logger().info(
            f"  CartPose  x={x:.1f} y={y:.1f} z={z:.1f} w={w:.1f} p={p:.1f} r={r:.1f}"
        )
        goal = CartPose.Goal()
        goal.x, goal.y, goal.z = float(x), float(y), float(z)
        goal.w, goal.p, goal.r = float(w), float(p), float(r)
        self._cart_distance_left = float('inf')
        goal_handle = await self._await_future(
            self._cart_client.send_goal_async(goal, feedback_callback=self._cart_feedback)
        )
        if not goal_handle.accepted:
            raise RuntimeError("CartPose goal rejected")
        result = await self._await_future(goal_handle.get_result_async())
        if not result.result.success:
            raise RuntimeError("CartPose failed")
        loop = asyncio.get_running_loop()
        await loop.run_in_executor(None, self._wait_cart_arrived)

    async def _send_joint_pose(self, joints):
        joints = [self._clamp_deg(v) for v in joints]
        self.get_logger().info(f"  JointPose  {[f'{v:.1f}' for v in joints]}")
        goal = JointPose.Goal()
        goal.joint1, goal.joint2, goal.joint3 = float(joints[0]), float(joints[1]), float(joints[2])
        goal.joint4, goal.joint5, goal.joint6 = float(joints[3]), float(joints[4]), float(joints[5])
        goal_handle = await self._await_future(
            self._joint_client.send_goal_async(goal, feedback_callback=self._joint_feedback)
        )
        if not goal_handle.accepted:
            raise RuntimeError("JointPose goal rejected")
        result = await self._await_future(goal_handle.get_result_async())
        if not result.result.success:
            raise RuntimeError("JointPose failed")

    async def _send_gripper(self, width, force):
        self.get_logger().info(f"  Gripper  width={int(width)}mm  force={int(force)}N")
        done  = threading.Event()
        error = [None]
        goal = OnRobotGripper.Goal()
        goal.width = int(width)
        goal.force = int(force)

        def _on_result(future):
            try:
                result = future.result()
                if not result.result.success:
                    error[0] = RuntimeError("OnRobotGripper failed")
            except Exception as exc:
                error[0] = exc
            finally:
                done.set()

        def _on_goal_accepted(future):
            try:
                gh = future.result()
                if not gh.accepted:
                    error[0] = RuntimeError("OnRobotGripper goal rejected")
                    done.set()
                    return
                gh.get_result_async().add_done_callback(_on_result)
            except Exception as exc:
                error[0] = exc
                done.set()

        self._gripper_client.send_goal_async(goal).add_done_callback(_on_goal_accepted)
        loop = asyncio.get_running_loop()
        await loop.run_in_executor(None, done.wait)
        if error[0]:
            raise error[0]
        await loop.run_in_executor(None, time.sleep, self.gripper_settle_time)

    async def _send_drop_conveyor(self, command: str):
        self.get_logger().info(f"  DropConveyor  command={command}")
        goal = Conveyor.Goal()
        goal.command = command
        goal_handle = await self._await_future(
            self._drop_conveyor_client.send_goal_async(goal)
        )
        if not goal_handle.accepted:
            raise RuntimeError(f"Drop conveyor goal '{command}' rejected")
        result = await self._await_future(goal_handle.get_result_async())
        if not result.result.success:
            raise RuntimeError(f"Drop conveyor '{command}' failed")

    async def _send_pickup_conveyor(self, command: str):
        self.get_logger().info(f"  PickupConveyor  command={command}")
        goal = Conveyor.Goal()
        goal.command = command
        goal_handle = await self._await_future(
            self._pickup_conveyor_client.send_goal_async(goal)
        )
        if not goal_handle.accepted:
            raise RuntimeError(f"Pickup conveyor goal '{command}' rejected")
        result = await self._await_future(goal_handle.get_result_async())
        if not result.result.success:
            raise RuntimeError(f"Pickup conveyor '{command}' failed")

    # ------------------------------------------------------------------
    # Feedback / arrival
    # ------------------------------------------------------------------

    def _cart_feedback(self, feedback_msg):
        d = feedback_msg.feedback.distance_left
        self._cart_distance_left = float(max(d)) if hasattr(d, '__len__') and d else float(d)

    def _joint_feedback(self, feedback_msg):
        pass

    def _wait_cart_arrived(self, threshold=2.0, timeout=10.0, stale_after=2.0):
        deadline     = time.time() + timeout
        last_value   = self._cart_distance_left
        last_changed = time.time()
        while not self._shutdown_event.is_set():
            d = self._cart_distance_left
            if d <= threshold:
                return
            if d != last_value:
                last_value   = d
                last_changed = time.time()
            elif time.time() - last_changed > stale_after:
                self.get_logger().warn(f"cart arrival stale ({d:.1f}mm) — proceeding")
                return
            if time.time() > deadline:
                self.get_logger().warn(f"cart arrival timeout ({d:.1f}mm) — proceeding")
                return
            time.sleep(0.05)

    # ------------------------------------------------------------------
    # Modbus helpers (blocking, called via run_in_executor)
    # ------------------------------------------------------------------

    def _mb_set_state(self, val):
        try:
            self._modbus.set_state(val)
        except Exception as e:
            self.get_logger().warn(f"Modbus set_state: {e}")

    def _mb_set_ready(self, val):
        try:
            self._modbus.set_ready(val)
        except Exception as e:
            self.get_logger().warn(f"Modbus set_ready: {e}")

    def _mb_release_camera(self):
        try:
            self._modbus.release_camera()
        except Exception as e:
            self.get_logger().warn(f"Modbus release_camera: {e}")

    def _mb_set_pip_done(self, pip):
        try:
            self._modbus.set_pip_done(pip)
        except Exception as e:
            self.get_logger().warn(f"Modbus set_pip_done: {e}")

    def _mb_wait_camera_free(self):
        poll = self.get_parameter('camera_poll_interval').value
        while not self._shutdown_event.is_set():
            if not self._modbus.other_robot_has_camera():
                return
            self.get_logger().info("  waiting — other robot has camera")
            time.sleep(poll)

    def _mb_claim_camera(self) -> bool:
        self._modbus.claim_camera()
        time.sleep(self.get_parameter('camera_claim_delay').value)
        if self._modbus.camera_collision():
            self._modbus.release_camera()
            return False
        return True

    def _mb_request_capture_and_wait(self) -> int:
        timeout = self.get_parameter('capture_timeout').value
        poll    = self.get_parameter('camera_poll_interval').value
        self._modbus.request_capture()
        deadline = time.time() + timeout
        while not self._shutdown_event.is_set():
            if not self._modbus.is_capture_pending():
                return self._modbus.read_pip_result()
            if time.time() > deadline:
                self.get_logger().warn(f"Capture timeout after {timeout:.0f}s")
                return 0
            time.sleep(poll)
        return 0

    # ------------------------------------------------------------------
    # Proximity sensor poll (blocking, no Modbus coil needed)
    # ------------------------------------------------------------------

    def _poll_own_sensor(self):
        """Block until this robot's proximity sensor on the pickup conveyor fires."""
        poll = self.get_parameter('sensor_poll_interval').value
        while not self._shutdown_event.is_set():
            if self._own_sensor_active:
                return
            time.sleep(poll)

    # ------------------------------------------------------------------
    # Pose helper
    # ------------------------------------------------------------------

    def _approach(self, pose: dict) -> dict:
        return {**pose, 'z': pose['z'] + self.approach_offset}

    # ------------------------------------------------------------------
    # Shared state handlers
    # ------------------------------------------------------------------

    async def _handle_idle(self):
        self._cycle                = 0
        self._retry_count          = 0
        self._camera_pass          = 0
        self._face1                = 0
        self._rotation_queue       = []
        self._current_rotation     = ''
        self._post_dice_pick_state = State.SAFE_HOME_TO_CAMERA
        loop = asyncio.get_running_loop()
        await loop.run_in_executor(None, self._mb_set_state, ROBOT_STATE_IDLE)
        return State.MOVE_HOME

    async def _handle_move_home(self):
        self.get_logger().info("[MOVE_HOME]")
        loop = asyncio.get_running_loop()
        await loop.run_in_executor(None, self._mb_set_state, ROBOT_STATE_HOMING)
        await self._send_joint_pose(self.home_joints)
        await self._send_gripper(self.gripper_open_width, force=30)
        await loop.run_in_executor(None, self._mb_set_ready, True)
        return self._FIRST_STATE_AFTER_HOME

    # ── Dice pickup (shared: R1 cycle 0 + both robots after rotation) ────

    async def _handle_open_gripper(self):
        self.get_logger().info("[OPEN_GRIPPER]")
        loop = asyncio.get_running_loop()
        await loop.run_in_executor(None, self._mb_set_state, ROBOT_STATE_PICKING)
        await self._send_gripper(self.gripper_open_width, force=30)
        return State.APPROACH_DICE

    async def _handle_approach_dice(self):
        self.get_logger().info("[APPROACH_DICE]")
        self._current_pickup_pose = self.dice_pose
        await self._send_cart_pose(**self._approach(self.dice_pose))
        return State.DESCEND_DICE

    async def _handle_descend_dice(self):
        self.get_logger().info("[DESCEND_DICE]")
        await self._send_cart_pose(**self.dice_pose)
        return State.GRAB_DICE

    async def _handle_grab_dice(self):
        self.get_logger().info("[GRAB_DICE]")
        await self._send_gripper(self.gripper_close_width, self.gripper_force)
        return State.ASCEND_FROM_DICE

    async def _handle_ascend_from_dice(self):
        self.get_logger().info("[ASCEND_FROM_DICE]")
        await self._send_cart_pose(**self._approach(self.dice_pose))
        return self._post_dice_pick_state

    # ── Conveyor pickup: wait → stop → grab ──────────────────────────

    async def _handle_wait_own_sensor(self):
        self.get_logger().info(
            f"[WAIT_OWN_SENSOR] side={self.get_parameter('prox_sensor_side').value}"
        )
        loop = asyncio.get_running_loop()
        await loop.run_in_executor(None, self._poll_own_sensor)
        self.get_logger().info("[WAIT_OWN_SENSOR] sensor triggered — waiting 1.25s for dice to settle")
        await loop.run_in_executor(None, time.sleep, 1.25)
        return State.STOP_CONVEYOR

    async def _handle_stop_conveyor(self):
        self.get_logger().info("[STOP_CONVEYOR]")
        await self._send_pickup_conveyor('stop')
        return State.OPEN_GRIPPER_CV

    async def _handle_open_gripper_cv(self):
        self.get_logger().info("[OPEN_GRIPPER_CV]")
        loop = asyncio.get_running_loop()
        await loop.run_in_executor(None, self._mb_set_state, ROBOT_STATE_PICKING)
        await self._send_gripper(self.gripper_open_width, force=30)
        return State.APPROACH_CV_PICKUP

    async def _handle_approach_cv_pickup(self):
        self.get_logger().info("[APPROACH_CV_PICKUP]")
        self._current_pickup_pose = self.conveyor_pickup_pose
        await self._send_cart_pose(**self._approach(self.conveyor_pickup_pose))
        return State.DESCEND_CV_PICKUP

    async def _handle_descend_cv_pickup(self):
        self.get_logger().info("[DESCEND_CV_PICKUP]")
        await self._send_cart_pose(**self.conveyor_pickup_pose)
        return State.GRAB_FROM_CONVEYOR

    async def _handle_grab_from_conveyor(self):
        self.get_logger().info("[GRAB_FROM_CONVEYOR]")
        await self._send_gripper(self.gripper_close_width, self.gripper_force)
        return State.ASCEND_CV_PICKUP

    async def _handle_ascend_cv_pickup(self):
        self.get_logger().info("[ASCEND_CV_PICKUP]")
        await self._send_cart_pose(**self._approach(self.conveyor_pickup_pose))
        return State.SAFE_HOME_TO_CAMERA

    # ── Camera inspection loop ────────────────────────────────────────

    async def _handle_safe_home_to_camera(self):
        self.get_logger().info("[SAFE_HOME_TO_CAMERA]")
        loop = asyncio.get_running_loop()
        await loop.run_in_executor(None, self._mb_set_state, ROBOT_STATE_TO_CAMERA)
        return State.WAIT_FOR_CAMERA

    async def _handle_wait_for_camera(self):
        self.get_logger().info("[WAIT_FOR_CAMERA]")
        loop = asyncio.get_running_loop()
        await loop.run_in_executor(None, self._mb_wait_camera_free)
        return State.CLAIM_CAMERA

    async def _handle_claim_camera(self):
        loop = asyncio.get_running_loop()
        claimed = await loop.run_in_executor(None, self._mb_claim_camera)
        if not claimed:
            self.get_logger().info("[CLAIM_CAMERA] collision — retrying")
            return State.WAIT_FOR_CAMERA
        self.get_logger().info("[CLAIM_CAMERA] camera claimed")
        return State.APPROACH_CAMERA

    async def _handle_approach_camera(self):
        self.get_logger().info("[APPROACH_CAMERA]")
        loop = asyncio.get_running_loop()
        await loop.run_in_executor(None, self._mb_set_state, ROBOT_STATE_INSPECTING)
        await self._send_cart_pose(**self.camera_pose)
        return State.INSPECT_AT_CAMERA

    async def _handle_inspect_at_camera(self):
        loop = asyncio.get_running_loop()
        settle = self.get_parameter('camera_settle_time').value
        if settle > 0:
            await loop.run_in_executor(None, time.sleep, settle)

        max_retries = 3
        for attempt in range(max_retries + 1):
            self.get_logger().info(
                f"[INSPECT_AT_CAMERA] requesting capture "
                f"(attempt {attempt + 1}/{max_retries + 1}  "
                f"cycle={self._cycle}  target_pip={self._target_pip})"
            )
            self._last_pip_count = await loop.run_in_executor(
                None, self._mb_request_capture_and_wait
            )
            self.get_logger().info(
                f"[INSPECT_AT_CAMERA] result={self._last_pip_count}  target={self._target_pip}"
            )
            if self._last_pip_count != 0:
                break
            if attempt < max_retries:
                self.get_logger().warn(
                    f"[INSPECT_AT_CAMERA] got 0 — retrying ({attempt + 1}/{max_retries})"
                )
                await loop.run_in_executor(None, time.sleep, settle if settle > 0 else 0.5)

        if self._last_pip_count == 0:
            pass_label = {0: 'face 1 (current top)', 1: 'face 2 (after discovery roll)',
                          2: 'final confirm (current top)'}.get(self._camera_pass, f'pass {self._camera_pass}')
            self.get_logger().warn(
                f"[INSPECT_AT_CAMERA] camera failed — prompting operator for manual input"
            )
            def _prompt_manual():
                # ros2 launch redirects stdin to /dev/null, so input() gets EOF
                # immediately. Open /dev/tty directly to reach the controlling
                # terminal regardless of how the process was launched.
                prompt = (
                    f"\n  [robot {self.robot_index}] Camera unavailable. "
                    f"Enter pip count for {pass_label} (1-6): "
                )
                with open('/dev/tty', 'r') as tty:
                    while True:
                        print(prompt, end='', flush=True)
                        line = tty.readline().strip()
                        try:
                            val = int(line)
                            if 1 <= val <= 6:
                                return val
                            print("  Value must be 1-6.")
                        except ValueError:
                            print("  Please enter a number between 1 and 6.")
            self._last_pip_count = await loop.run_in_executor(None, _prompt_manual)
            self.get_logger().info(
                f"[INSPECT_AT_CAMERA] operator entered: {self._last_pip_count}"
            )

        return State.RELEASE_CAMERA

    async def _handle_release_camera(self):
        self.get_logger().info("[RELEASE_CAMERA]")
        loop = asyncio.get_running_loop()
        await loop.run_in_executor(None, self._mb_release_camera)
        return State.CHECK_PIP_RESULT

    async def _handle_check_pip_result(self):
        pip    = self._last_pip_count
        target = self._target_pip

        if pip == 0:
            self.get_logger().error(
                f"[CHECK_PIP_RESULT] camera returned 0 (pass {self._camera_pass}) — aborting"
            )
            return State.ERROR

        if self._camera_pass == 0:
            # ── First read: face_1 ────────────────────────────────────────
            if pip == target:
                self.get_logger().info(
                    f"[CHECK_PIP_RESULT] face_1={pip} == target — early confirm"
                )
                return State.MARK_PIP_DONE
            self._face1 = pip
            self._camera_pass = 1
            self._rotation_queue = ['roll_forward']   # mandatory discovery rotation
            self._current_rotation = self._rotation_queue.pop(0)
            self.get_logger().info(
                f"[CHECK_PIP_RESULT] face_1={pip} — queued discovery rotation"
            )
            return State.ROTATE_DICE

        elif self._camera_pass == 1:
            # ── Second read: face_2 ───────────────────────────────────────
            if pip == target:
                self.get_logger().info(
                    f"[CHECK_PIP_RESULT] face_2={pip} == target — early confirm"
                )
                return State.MARK_PIP_DONE
            self.get_logger().info(
                f"[CHECK_PIP_RESULT] face_1={self._face1}  face_2={pip} "
                f"— computing orientation"
            )
            self._last_pip_count = pip   # keep face_2 available for COMPUTE_ORIENTATION
            return State.COMPUTE_ORIENTATION

        else:
            # ── Final confirm after BFS rotations ─────────────────────────
            self.get_logger().info(
                f"[CHECK_PIP_RESULT] final confirm pip={pip}"
            )
            if pip == target:
                self.get_logger().info(
                    f"[CHECK_PIP_RESULT] final confirm pip={pip} == target — confirm success"
                )
                return State.MARK_PIP_DONE
            else:
                self.get_logger().error(
                    f"[CHECK_PIP_RESULT] final confirm pip={pip} != target={target} — confirm failed"
                )
                return State.ERROR

    def _next_state_after_pip_found(self) -> State:
        """Overridden by Robot2TaskNode for the final cycle (pip 6 → dice_pose)."""
        return State.SAFE_HOME_TO_CV_DROP

    # ── Rotation retry ────────────────────────────────────────────────

    async def _handle_safe_home_to_setdown(self):
        self.get_logger().info("[SAFE_HOME_TO_SETDOWN]")
        loop = asyncio.get_running_loop()
        await loop.run_in_executor(None, self._mb_set_state, ROBOT_STATE_RETRY)
        await self._send_joint_pose(self.home_joints)
        return State.APPROACH_SETDOWN

    async def _handle_approach_setdown(self):
        self.get_logger().info("[APPROACH_SETDOWN]")
        await self._send_cart_pose(**self._approach(self._setdown_pose))
        return State.DESCEND_SETDOWN

    async def _handle_descend_setdown(self):
        self.get_logger().info("[DESCEND_SETDOWN]")
        await self._send_cart_pose(**self._setdown_pose)
        return State.RELEASE_SETDOWN

    async def _handle_release_setdown(self):
        self.get_logger().info("[RELEASE_SETDOWN]")
        await self._send_gripper(self.gripper_open_width, force=30)
        return State.ASCEND_SETDOWN

    async def _handle_ascend_setdown(self):
        self.get_logger().info("[ASCEND_SETDOWN]")
        await self._send_cart_pose(**self._approach(self._setdown_pose))
        return State.ROTATE_DICE

    async def _handle_rotate_dice(self):
        """Execute _current_rotation — sequences end holding the die, no re-pick needed."""
        self.get_logger().info(
            f"[ROTATE_DICE] executing '{self._current_rotation}'  "
            f"queue remaining: {len(self._rotation_queue)}"
        )
        await self._execute_named_rotation(self._current_rotation)

        if self._rotation_queue:
            self._current_rotation = self._rotation_queue.pop(0)
            return State.ROTATE_DICE
        else:
            self._current_rotation = ''
            return State.SAFE_HOME_TO_CAMERA

    def _load_rotation_config(self):
        """Load rotations_r{robot_index}.yaml from the installed package share directory."""
        robot_index = self.get_parameter('robot_index').value
        config_file = _os.path.join(
            get_package_share_directory('robot_task'),
            'config',
            f'rotations_r{robot_index}.yaml',
        )
        try:
            with open(config_file) as f:
                data = yaml.safe_load(f)
            self._travel_z       = float(data.get('travel_z', 60.0))
            self._rotation_steps = data.get('sequences', {})
            self.get_logger().info(
                f"Loaded rotation config: rotations_r{robot_index}.yaml "
                f"({len(self._rotation_steps)} sequences, travel_z={self._travel_z}mm)"
            )
        except Exception as exc:
            self.get_logger().error(
                f"Failed to load rotation config {config_file}: {exc}"
            )
            self._travel_z       = 60.0
            self._rotation_steps = {}

    async def _execute_named_rotation(self, rotation_name: str):
        """Execute a named rotation sequence loaded from rotations_r{N}.yaml."""
        steps = self._rotation_steps.get(rotation_name)
        if steps is None:
            self.get_logger().error(f"[EXECUTE_ROTATION] unknown rotation '{rotation_name}'")
            return

        total = len(steps)
        self.get_logger().info(f"[EXECUTE_ROTATION] '{rotation_name}' ({total} steps)")

        for i, step in enumerate(steps):
            kind = step[0]
            x, y, z, w, p, r = step[1]
            prefix = f"[EXECUTE_ROTATION] step {i+1}/{total} ({kind})"

            if kind == 'move':
                self.get_logger().info(f"{prefix}")
                await self._send_cart_pose(x, y, z, w, p, r)

            elif kind == 'place':
                self.get_logger().info(f"{prefix}  approach")
                await self._send_cart_pose(x, y, z + self._travel_z, w, p, r)
                self.get_logger().info(f"{prefix}  lower")
                await self._send_cart_pose(x, y, z, w, p, r)
                self.get_logger().info(f"{prefix}  open gripper")
                await self._send_gripper(self.gripper_open_width, force=30)
                self.get_logger().info(f"{prefix}  lift")
                await self._send_cart_pose(x, y, z + self._travel_z, w, p, r)

            elif kind == 'pick':
                self.get_logger().info(f"{prefix}  approach")
                await self._send_cart_pose(x, y, z + self._travel_z, w, p, r)
                self.get_logger().info(f"{prefix}  lower")
                await self._send_cart_pose(x, y, z, w, p, r)
                self.get_logger().info(f"{prefix}  close gripper")
                await self._send_gripper(self.gripper_close_width, self.gripper_force)
                self.get_logger().info(f"{prefix}  lift")
                await self._send_cart_pose(x, y, z + self._travel_z, w, p, r)

            else:
                self.get_logger().warn(f"{prefix} — unknown kind, skipped")

    async def _handle_compute_orientation(self):
        """
        Reconstruct full die orientation from face_1 + face_2 and BFS-plan the
        minimum rotation sequence to bring target pip to the top.

        face_1 = initial top (camera_pass 0 read, before discovery roll).
        face_2 = top after mandatory roll_forward = initial front face.
        Together they uniquely determine the full die state.
        """
        face1  = self._face1
        face2  = self._last_pip_count
        target = self._target_pip
        self.get_logger().info(
            f"[COMPUTE_ORIENTATION] face_1={face1}  face_2={face2}  target={target}"
        )

        # Reconstruct die state before the discovery roll
        initial = from_visible_faces(top=face1, front=face2)
        if initial is None:
            self.get_logger().error(
                f"[COMPUTE_ORIENTATION] top={face1} + front={face2} is ambiguous or invalid"
            )
            return State.ERROR

        # Apply the roll_forward already executed → current physical state
        current = initial.apply('roll_forward')

        # BFS: shortest rotation sequence to put target pip on top
        rotation_list = plan_to_value(current, target)
        if rotation_list is None:
            self.get_logger().error(
                f"[COMPUTE_ORIENTATION] pip {target} unreachable from current state {current}"
            )
            return State.ERROR

        self.get_logger().info(f"[COMPUTE_ORIENTATION] planned: {rotation_list}")
        self._camera_pass    = 2
        self._rotation_queue = rotation_list

        if not self._rotation_queue:
            # BFS returned empty — target already on top (shouldn't happen here, but safe)
            return State.MARK_PIP_DONE

        self._current_rotation = self._rotation_queue.pop(0)
        return State.ROTATE_DICE

    # ── Pip confirmed: write coil, then proceed ───────────────────────

    async def _handle_mark_pip_done(self):
        """
        Single convergence point for all three confirmation paths.
        Writes the pip_done Modbus coil then transitions to conveyor or final placement.
        """
        pip = self._target_pip
        self.get_logger().info(f"[MARK_PIP_DONE] pip {pip} confirmed — setting coil")
        loop = asyncio.get_running_loop()
        await loop.run_in_executor(None, self._mb_set_pip_done, pip)
        return self._next_state_after_pip_found()

    # ── Conveyor drop: place dice, start drop belt ────────────────────

    async def _handle_safe_home_to_cv_drop(self):
        self.get_logger().info("[SAFE_HOME_TO_CV_DROP]")
        loop = asyncio.get_running_loop()
        await loop.run_in_executor(None, self._mb_set_state, ROBOT_STATE_TO_CONVEYOR)
        return State.APPROACH_CV_DROP

    async def _handle_approach_cv_drop(self):
        self.get_logger().info("[APPROACH_CV_DROP]")
        await self._send_cart_pose(**self._approach(self.conveyor_drop_pose))
        return State.DESCEND_CV_DROP

    async def _handle_descend_cv_drop(self):
        self.get_logger().info("[DESCEND_CV_DROP]")
        await self._send_cart_pose(**self.conveyor_drop_pose)
        return State.RELEASE_CV_DROP

    async def _handle_release_cv_drop(self):
        self.get_logger().info("[RELEASE_CV_DROP]")
        await self._send_gripper(self.gripper_open_width, force=30)
        return State.ASCEND_CV_DROP

    async def _handle_ascend_cv_drop(self):
        self.get_logger().info("[ASCEND_CV_DROP]")
        await self._send_cart_pose(**self._approach(self.conveyor_drop_pose))
        return State.START_CONVEYOR

    async def _handle_start_conveyor(self):
        cmd = self.get_parameter('drop_conveyor_cmd').value
        self.get_logger().info(f"[START_CONVEYOR] command={cmd}")
        await self._send_drop_conveyor(cmd)
        return State.ADVANCE_CYCLE

    # ── Cycle bookkeeping ─────────────────────────────────────────────

    async def _handle_advance_cycle(self):
        self._cycle               += 1
        self._retry_count          = 0
        self._camera_pass          = 0
        self._face1                = 0
        self._rotation_queue       = []
        self._current_rotation     = ''
        self._post_dice_pick_state = State.SAFE_HOME_TO_CAMERA
        self.get_logger().info(f"[ADVANCE_CYCLE] cycle → {self._cycle}")
        if self._cycle >= len(self._PIP_SEQUENCE):
            return State.FINAL_HOME
        return State.SAFE_HOME_TO_WAIT

    async def _handle_safe_home_to_wait(self):
        """Return to home joint pose, then wait for the pickup conveyor sensor."""
        self.get_logger().info("[SAFE_HOME_TO_WAIT]")
        loop = asyncio.get_running_loop()
        await loop.run_in_executor(None, self._mb_set_state, ROBOT_STATE_IDLE)
        await self._send_joint_pose(self.home_joints)
        return State.WAIT_OWN_SENSOR

    # ── Terminal states ───────────────────────────────────────────────

    async def _handle_final_home(self):
        self.get_logger().info("[FINAL_HOME]")
        loop = asyncio.get_running_loop()
        await loop.run_in_executor(None, self._mb_set_state, ROBOT_STATE_COMPLETE)
        await self._send_joint_pose(self.home_joints)
        await loop.run_in_executor(None, self._mb_set_ready, False)
        return State.COMPLETE

    async def _handle_error(self):
        loop = asyncio.get_running_loop()
        await loop.run_in_executor(None, self._mb_set_state, ROBOT_STATE_ERROR)
        await loop.run_in_executor(None, self._mb_release_camera)
        self.get_logger().error("[ERROR] task halted")
        return State.ERROR

    # ------------------------------------------------------------------
    # Handler dict
    # ------------------------------------------------------------------

    def _base_handlers(self) -> dict:
        return {
            State.IDLE:                  self._handle_idle,
            State.MOVE_HOME:             self._handle_move_home,
            State.OPEN_GRIPPER:          self._handle_open_gripper,
            State.APPROACH_DICE:         self._handle_approach_dice,
            State.DESCEND_DICE:          self._handle_descend_dice,
            State.GRAB_DICE:             self._handle_grab_dice,
            State.ASCEND_FROM_DICE:      self._handle_ascend_from_dice,
            State.WAIT_OWN_SENSOR:       self._handle_wait_own_sensor,
            State.STOP_CONVEYOR:         self._handle_stop_conveyor,
            State.OPEN_GRIPPER_CV:       self._handle_open_gripper_cv,
            State.APPROACH_CV_PICKUP:    self._handle_approach_cv_pickup,
            State.DESCEND_CV_PICKUP:     self._handle_descend_cv_pickup,
            State.GRAB_FROM_CONVEYOR:    self._handle_grab_from_conveyor,
            State.ASCEND_CV_PICKUP:      self._handle_ascend_cv_pickup,
            State.SAFE_HOME_TO_CAMERA:   self._handle_safe_home_to_camera,
            State.WAIT_FOR_CAMERA:       self._handle_wait_for_camera,
            State.CLAIM_CAMERA:          self._handle_claim_camera,
            State.APPROACH_CAMERA:       self._handle_approach_camera,
            State.INSPECT_AT_CAMERA:     self._handle_inspect_at_camera,
            State.RELEASE_CAMERA:        self._handle_release_camera,
            State.CHECK_PIP_RESULT:      self._handle_check_pip_result,
            State.COMPUTE_ORIENTATION:   self._handle_compute_orientation,
            State.SAFE_HOME_TO_SETDOWN:  self._handle_safe_home_to_setdown,
            State.APPROACH_SETDOWN:      self._handle_approach_setdown,
            State.DESCEND_SETDOWN:       self._handle_descend_setdown,
            State.RELEASE_SETDOWN:       self._handle_release_setdown,
            State.ASCEND_SETDOWN:        self._handle_ascend_setdown,
            State.ROTATE_DICE:           self._handle_rotate_dice,
            State.MARK_PIP_DONE:         self._handle_mark_pip_done,
            State.SAFE_HOME_TO_CV_DROP:  self._handle_safe_home_to_cv_drop,
            State.APPROACH_CV_DROP:      self._handle_approach_cv_drop,
            State.DESCEND_CV_DROP:       self._handle_descend_cv_drop,
            State.RELEASE_CV_DROP:       self._handle_release_cv_drop,
            State.ASCEND_CV_DROP:        self._handle_ascend_cv_drop,
            State.START_CONVEYOR:        self._handle_start_conveyor,
            State.ADVANCE_CYCLE:         self._handle_advance_cycle,
            State.SAFE_HOME_TO_WAIT:     self._handle_safe_home_to_wait,
            State.FINAL_HOME:            self._handle_final_home,
            State.ERROR:                 self._handle_error,
        }

    # ------------------------------------------------------------------
    # State machine runner
    # ------------------------------------------------------------------

    async def _run(self, handlers: dict):
        await self._wait_for_servers()
        loop = asyncio.get_running_loop()
        if not await loop.run_in_executor(None, self._modbus.connect):
            self.get_logger().error("Cannot connect to Modbus server — aborting")
            return

        while self.state not in (State.COMPLETE, State.ERROR):
            self.get_logger().info(f"━━━ {self.state.name} ━━━")
            try:
                self.state = await handlers[self.state]()
            except Exception as exc:
                self.get_logger().error(f"Exception in {self.state.name}: {exc}")
                self.state = State.ERROR

        if self.state == State.COMPLETE:
            self.get_logger().info("Task COMPLETE")
        else:
            self.get_logger().error("Task ended in ERROR")
        self._modbus.disconnect()
