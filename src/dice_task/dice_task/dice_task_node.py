"""
Dice Inspection Task Node
=========================
Async state machine that moves a FANUC CRX-10 with an OnRobot gripper to:
  1. Pick up a dice from a known location
  2. Show it to an external camera
  3. Set it back down, rotate the approach angle, and repeat

Key design choices:
  - asyncio event loop drives the state machine in the main thread
  - MultiThreadedExecutor spins rclpy in a background thread
  - rclpy Futures are bridged to asyncio via an event-based helper
  - Each state is an async method returning the next State value
  - All positions are ROS2 parameters for easy calibration

Usage:
    ros2 run dice_task dice_task_node --ros-args \\
        -p robot_name:=bunsen \\
        -p num_repetitions:=6 \\
        -p dice_x:=300.0 -p dice_y:=400.0 -p dice_z:=-150.0 \\
        -p dice_w:=180.0 -p dice_p:=0.0   -p dice_r:=0.0   \\
        -p camera_x:=200.0 -p camera_y:=500.0 -p camera_z:=100.0 \\
        -p camera_w:=180.0 -p camera_p:=0.0   -p camera_r:=0.0
"""

import asyncio
import threading
import time
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor

from fanuc_interfaces.action import CartPose, JointPose, OnRobotGripper
from dice_task.dice_vision import capture_save_and_count, connect_and_verify, shutdown_camera


# ---------------------------------------------------------------------------
# State definitions
# ---------------------------------------------------------------------------

class State(Enum):
    IDLE            = auto()
    MOVE_HOME       = auto()
    CHECK_CAMERA    = auto()   # verify camera before picking up dice
    OPEN_GRIPPER    = auto()
    APPROACH_DICE   = auto()   # move above dice at approach height
    DESCEND_DICE    = auto()   # lower straight down to dice
    GRAB_DICE       = auto()   # close gripper
    LIFT_DICE       = auto()   # raise back to approach height
    MOVE_TO_CAMERA   = auto()   # carry dice to camera view position
    SETTLE_AT_CAMERA = auto()   # blocking wait for robot to physically arrive and stop
    SHOW_DICE        = auto()   # capture image, then hold
    RETURN_APPROACH = auto()   # move back above set-down location
    DESCEND_SETDOWN = auto()   # lower dice to surface
    RELEASE_DICE    = auto()   # open gripper
    ASCEND_SETDOWN  = auto()   # lift gripper clear of dice
    CHECK_COMPLETE  = auto()   # decide whether to loop or finish
    FINAL_HOME      = auto()   # return home after all reps
    COMPLETE        = auto()
    ERROR           = auto()


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class DiceTaskNode(Node):
    """
    Parameters (all settable via --ros-args -p name:=value)
    ---------------------------------------------------------
    robot_name          : str   Robot namespace              (default: bunsen)
    num_repetitions     : int   Times to show the dice       (default: 6)
    rotation_step_deg   : float End-effector roll added each rep (deg) (default: 60.0)
    camera_hold_time    : float Seconds to hold at camera    (default: 2.0)
    approach_height_offset : float mm above pick/place to approach from (default: 60.0)

    dice_x/y/z          : float Dice pickup cartesian position (mm) — CALIBRATE
    dice_w/p/r          : float Dice pickup WPR orientation (deg) — CALIBRATE
    camera_x/y/z        : float Camera display position (mm)    — CALIBRATE
    camera_w/p/r        : float Camera display orientation (deg) — CALIBRATE

    gripper_open_width  : float Gripper jaw gap when open (mm)  (default: 70)
    gripper_close_width : float Gripper jaw gap on dice (mm)    (default: 22)
    gripper_force       : float Grip force in Newtons           (default: 20)

    home_j1..j6         : float Home joint angles (deg)
    """

    def __init__(self):
        super().__init__('dice_task_node')
        self._declare_parameters()
        self._create_action_clients()
        self.state = State.IDLE
        self.current_rep = 0
        self.pip_results = []           # pip count recorded for each repetition
        self._cart_distance_left = 0.0  # updated by CartPose feedback callback
        self._shutdown_event = threading.Event()  # set on exit to unblock waits
        self.get_logger().info(
            f"DiceTaskNode ready — {self.num_reps} reps, "
            f"{self.rotation_step}° rotation per rep"
        )

    # ------------------------------------------------------------------
    # Parameters
    # ------------------------------------------------------------------

    def _declare_parameters(self):
        self.declare_parameter('robot_name',             'bunsen')
        self.declare_parameter('num_repetitions',        6)
        self.declare_parameter('rotation_step_deg',      60.0)
        self.declare_parameter('camera_hold_time',       2.0)
        self.declare_parameter('approach_height_offset', 60.0)
        self.declare_parameter('gripper_settle_time',    2.0)   # extra wait after gripper cmd (s)
        self.declare_parameter('camera_settle_time',     2.0)  # wait after arriving at camera pose before capture (s)
        self.declare_parameter('camera_index',           0)    # which camera to use (0-based)
        self.declare_parameter('image_save_dir',         '/home/ben-kopf/ros2_ws_Claude/fanuc_ros2_drivers/src/dice_task/images')

        # Dice pickup pose — replace with real values after calibration
        self.declare_parameter('dice_x',   300.0)
        self.declare_parameter('dice_y',   400.0)
        self.declare_parameter('dice_z',  -150.0)
        self.declare_parameter('dice_w',   180.0)
        self.declare_parameter('dice_p',     0.0)
        self.declare_parameter('dice_r',     0.0)

        # Camera display pose — replace with real values after calibration
        self.declare_parameter('camera_x',  200.0)
        self.declare_parameter('camera_y',  500.0)
        self.declare_parameter('camera_z',  100.0)
        self.declare_parameter('camera_w',  180.0)
        self.declare_parameter('camera_p',    0.0)
        self.declare_parameter('camera_r',    0.0)

        # Gripper
        self.declare_parameter('gripper_open_width',  70.0)
        self.declare_parameter('gripper_close_width', 22.0)
        self.declare_parameter('gripper_force',       20.0)

        # Home joint pose
        self.declare_parameter('home_j1',   0.0)
        self.declare_parameter('home_j2',   0.0)
        self.declare_parameter('home_j3',   0.0)
        self.declare_parameter('home_j4',   0.0)
        self.declare_parameter('home_j5', -90.0)
        self.declare_parameter('home_j6',   0.0)

    # Convenience accessors

    @property
    def robot_name(self):
        return self.get_parameter('robot_name').value

    @property
    def num_reps(self):
        return self.get_parameter('num_repetitions').value

    @property
    def rotation_step(self):
        return self.get_parameter('rotation_step_deg').value

    @property
    def gripper_settle_time(self):
        return self.get_parameter('gripper_settle_time').value

    @property
    def camera_hold_time(self):
        return self.get_parameter('camera_hold_time').value

    @property
    def camera_settle_time(self):
        return self.get_parameter('camera_settle_time').value

    @property
    def approach_offset(self):
        return self.get_parameter('approach_height_offset').value

    @property
    def dice_pose(self):
        return {k: self.get_parameter(f'dice_{k}').value for k in ('x', 'y', 'z', 'w', 'p', 'r')}

    @property
    def camera_pose(self):
        return {k: self.get_parameter(f'camera_{k}').value for k in ('x', 'y', 'z', 'w', 'p', 'r')}

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

    # ------------------------------------------------------------------
    # Action clients
    # ------------------------------------------------------------------

    def _create_action_clients(self):
        ns = self.robot_name
        self._cart_client    = ActionClient(self, CartPose,        f'/{ns}/cartesian_pose')
        self._joint_client   = ActionClient(self, JointPose,       f'/{ns}/joint_pose')
        self._gripper_client = ActionClient(self, OnRobotGripper,  f'/{ns}/onrobot_gripper')

    # ------------------------------------------------------------------
    # Async bridge: rclpy Future → asyncio awaitable
    # ------------------------------------------------------------------

    async def _await_future(self, rclpy_future):
        """
        Bridge an rclpy Future to asyncio without race conditions.

        Uses threading.Event + run_in_executor so that:
          - The background rclpy executor can call done.set() from any thread
          - If the future is already resolved before we start waiting,
            threading.Event.wait() returns immediately (no signal lost)
          - The asyncio event loop is never blocked
        """
        done = threading.Event()
        rclpy_future.add_done_callback(lambda _: done.set())
        loop = asyncio.get_running_loop()
        await loop.run_in_executor(None, done.wait)
        return rclpy_future.result()

    # ------------------------------------------------------------------
    # Action servers — wait for all to be ready (called once at startup)
    # ------------------------------------------------------------------

    async def _wait_for_servers(self):
        """Wait for all action servers without blocking the event loop."""
        self.get_logger().info("Waiting for action servers...")
        loop = asyncio.get_running_loop()
        await loop.run_in_executor(None, self._cart_client.wait_for_server)
        await loop.run_in_executor(None, self._joint_client.wait_for_server)
        await loop.run_in_executor(None, self._gripper_client.wait_for_server)
        self.get_logger().info("All action servers ready")

    # ------------------------------------------------------------------
    # Action helpers
    # ------------------------------------------------------------------

    async def _send_cart_pose(self, x, y, z, w=200.0, p=200.0, r=200.0):
        """
        Move to a cartesian pose and wait until the robot physically arrives.

        The FANUC action server returns its result as soon as the motion
        command is accepted by the controller, before the robot reaches the
        target.  After receiving the result, this method blocks via
        run_in_executor until distance_left (from feedback) reaches zero,
        ensuring callers only proceed once motion is truly complete.
        """
        self.get_logger().info(
            f"  CartPose  x={x:.1f}  y={y:.1f}  z={z:.1f}  "
            f"w={w:.1f}  p={p:.1f}  r={r:.1f}"
        )
        goal = CartPose.Goal()
        goal.x, goal.y, goal.z = float(x), float(y), float(z)
        goal.w, goal.p, goal.r = float(w), float(p), float(r)

        # Reset distance so _wait_cart_arrived doesn't exit immediately
        self._cart_distance_left = float('inf')

        goal_handle = await self._await_future(
            self._cart_client.send_goal_async(goal, feedback_callback=self._cart_feedback)
        )
        if not goal_handle.accepted:
            raise RuntimeError("CartPose goal rejected by server")

        result = await self._await_future(goal_handle.get_result_async())
        if not result.result.success:
            raise RuntimeError("CartPose reported failure")

        # Block until feedback confirms physical arrival
        loop = asyncio.get_running_loop()
        await loop.run_in_executor(None, self._wait_cart_arrived)
        return True

    async def _send_joint_pose(self, joints):
        """Move to a joint pose and wait for completion."""
        self.get_logger().info(
            f"  JointPose  {[f'{v:.1f}' for v in joints]}"
        )
        goal = JointPose.Goal()
        goal.joint1, goal.joint2, goal.joint3 = float(joints[0]), float(joints[1]), float(joints[2])
        goal.joint4, goal.joint5, goal.joint6 = float(joints[3]), float(joints[4]), float(joints[5])

        goal_handle = await self._await_future(
            self._joint_client.send_goal_async(goal, feedback_callback=self._joint_feedback)
        )
        if not goal_handle.accepted:
            raise RuntimeError("JointPose goal rejected by server")

        result = await self._await_future(goal_handle.get_result_async())
        if not result.result.success:
            raise RuntimeError("JointPose reported failure")
        return True

    async def _send_gripper(self, width, force):
        """
        Command the OnRobot gripper and wait for physical completion.

        Uses native ROS2 action callbacks (not asyncio.sleep) so the asyncio
        event loop is never yielded mid-gripper — preventing out-of-order state
        transitions.  The settle sleep is a real blocking sleep in the ROS2
        callback thread; completion is signalled via threading.Event which the
        asyncio coroutine waits on through run_in_executor.
        """
        self.get_logger().info(f"  Gripper  width={int(width)}mm  force={int(force)}N")

        done  = threading.Event()
        error = [None]   # mutable slot for the callback to store exceptions

        goal = OnRobotGripper.Goal()
        goal.width = int(width)
        goal.force = int(force)

        def _on_result(future):
            try:
                result = future.result()
                if not result.result.success:
                    error[0] = RuntimeError("OnRobotGripper reported failure")
            except Exception as exc:
                error[0] = exc
            finally:
                # Blocking settle: jaws physically finish before we signal done
                time.sleep(self.gripper_settle_time)
                done.set()

        def _on_goal_accepted(future):
            try:
                goal_handle = future.result()
                if not goal_handle.accepted:
                    error[0] = RuntimeError("OnRobotGripper goal rejected by server")
                    done.set()
                    return
                goal_handle.get_result_async().add_done_callback(_on_result)
            except Exception as exc:
                error[0] = exc
                done.set()

        self._gripper_client.send_goal_async(goal).add_done_callback(_on_goal_accepted)

        loop = asyncio.get_running_loop()
        await loop.run_in_executor(None, done.wait)

        if error[0]:
            raise error[0]
        return True

    # ------------------------------------------------------------------
    # Feedback callbacks (logged at debug level to avoid noise)
    # ------------------------------------------------------------------

    def _cart_feedback(self, feedback_msg):
        d = feedback_msg.feedback.distance_left
        # distance_left is an array.array (one value per axis) — use the max
        # so we wait until every axis has arrived, not just the first one.
        if hasattr(d, '__len__'):
            self._cart_distance_left = float(max(d)) if len(d) > 0 else 0.0
        else:
            self._cart_distance_left = float(d)
        self.get_logger().debug(f"CartPose remaining: {self._cart_distance_left:.1f}")

    def _joint_feedback(self, feedback_msg):
        self.get_logger().debug(
            f"JointPose remaining: {feedback_msg.feedback.distance_left}"
        )

    def _wait_cart_arrived(self, threshold: float = 2.0, timeout: float = 10.0,
                           stale_after: float = 2.0):
        """
        Block until the robot physically arrives at the target pose.

        Exits when ANY of the following occur:
          - distance_left drops to threshold (robot arrived)
          - feedback stops updating for stale_after seconds (robot stopped but
            feedback never reached threshold — proceed anyway)
          - timeout seconds have elapsed since the call (safety ceiling)
          - _shutdown_event is set (process shutting down — exit immediately)

        Parameters
        ----------
        threshold : float
            Distance (mm) below which the robot is considered arrived.
        timeout : float
            Hard upper bound in seconds.
        stale_after : float
            Seconds without a feedback change before giving up waiting.
        """
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
                self.get_logger().warn(
                    f"_wait_cart_arrived: feedback stale for {stale_after:.0f}s "
                    f"(distance_left={d:.1f}) — proceeding"
                )
                return

            if time.time() > deadline:
                self.get_logger().warn(
                    f"_wait_cart_arrived: timeout after {timeout:.0f}s "
                    f"(distance_left={d:.1f}) — proceeding"
                )
                return

            time.sleep(0.05)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _pickup_pose(self, z_override=None):
        """Return dice pose dict for the current repetition's rotation.

        Rotation cycles through three positions per group of reps:

            rep % 3 == 0 →  0°         (base)
            rep % 3 == 1 → -step       (e.g. -90°)
            rep % 3 == 2 → -2*step     (e.g. -180° → clamped to -179.9°)

        The pattern then repeats: rep 3 back to 0°, rep 4 to -90°, etc.
        """
        dice = self.dice_pose
        # 3-step cycle: 0, -step, -2*step, 0, -step, -2*step ...
        rotation = -(self.current_rep % 3) * self.rotation_step
        # Normalize to (-180, 180] then clamp to the server's [-179.9, 179.9] limit
        r = ((dice['r'] + rotation) + 180.0) % 360.0 - 180.0
        r = max(-179.9, min(179.9, r))
        return {
            'x': dice['x'],
            'y': dice['y'],
            'z': z_override if z_override is not None else dice['z'],
            'w': dice['w'],
            'p': dice['p'],
            'r': r,
        }

    # ------------------------------------------------------------------
    # State handlers — each returns the next State
    # ------------------------------------------------------------------

    async def _handle_idle(self):
        self.current_rep = 0
        self.get_logger().info(
            f"Starting dice inspection: {self.num_reps} repetitions, "
            f"{self.rotation_step}° roll per rep"
        )
        return State.MOVE_HOME

    async def _handle_move_home(self):
        self.get_logger().info("[MOVE_HOME] Moving to home joint pose")
        await self._send_joint_pose(self.home_joints)
        return State.CHECK_CAMERA

    async def _handle_check_camera(self):
        index = self.get_parameter('camera_index').value
        self.get_logger().info(f"[CHECK_CAMERA] Verifying camera index {index}")
        loop = asyncio.get_running_loop()
        await loop.run_in_executor(None, lambda: connect_and_verify(index=index))
        self.get_logger().info("[CHECK_CAMERA] Camera OK — proceeding")
        return State.OPEN_GRIPPER

    async def _handle_open_gripper(self):
        self.get_logger().info("[OPEN_GRIPPER] Opening gripper")
        await self._send_gripper(self.gripper_open_width, force=30)
        return State.APPROACH_DICE

    async def _handle_approach_dice(self):
        pose = self._pickup_pose(z_override=self.dice_pose['z'] + self.approach_offset)
        self.get_logger().info(
            f"[APPROACH_DICE] Rep {self.current_rep + 1}/{self.num_reps} — "
            f"approach above dice (R={pose['r']:.1f}°)"
        )
        await self._send_cart_pose(**pose)
        return State.DESCEND_DICE

    async def _handle_descend_dice(self):
        pose = self._pickup_pose()
        self.get_logger().info("[DESCEND_DICE] Descending to dice")
        await self._send_cart_pose(**pose)
        return State.GRAB_DICE

    async def _handle_grab_dice(self):
        self.get_logger().info("[GRAB_DICE] Closing gripper on dice")
        await self._send_gripper(self.gripper_close_width, self.gripper_force)
        return State.LIFT_DICE

    async def _handle_lift_dice(self):
        pose = self._pickup_pose(z_override=self.dice_pose['z'] + self.approach_offset)
        self.get_logger().info("[LIFT_DICE] Lifting dice to approach height")
        await self._send_cart_pose(**pose)
        return State.MOVE_TO_CAMERA

    async def _handle_move_to_camera(self):
        cam = self.camera_pose
        self.get_logger().info("[MOVE_TO_CAMERA] Carrying dice to camera position")
        await self._send_cart_pose(**cam)
        return State.SETTLE_AT_CAMERA

    async def _handle_settle_at_camera(self):
        self.get_logger().info(
            f"[SETTLE_AT_CAMERA] Waiting {self.camera_settle_time:.1f}s for robot to arrive and stop"
        )
        loop = asyncio.get_running_loop()
        await loop.run_in_executor(None, time.sleep, self.camera_settle_time)
        return State.SHOW_DICE

    async def _handle_show_dice(self):
        loop = asyncio.get_running_loop()

        save_dir = self.get_parameter('image_save_dir').value
        rep      = self.current_rep + 1

        pip_count, save_path = await loop.run_in_executor(
            None, lambda: capture_save_and_count(save_dir, rep)
        )

        self.pip_results.append(pip_count)
        if pip_count >= 0:
            self.get_logger().info(
                f"[SHOW_DICE] Rep {rep}: {pip_count} pip(s) — saved {save_path}"
            )
        else:
            self.get_logger().warn(f"[SHOW_DICE] Rep {rep}: capture failed")

        # Blocking post-capture hold (0 = skip).
        if self.camera_hold_time > 0:
            self.get_logger().info(
                f"[SHOW_DICE] Holding for {self.camera_hold_time:.1f}s after capture"
            )
            await loop.run_in_executor(None, time.sleep, self.camera_hold_time)

        return State.RETURN_APPROACH

    async def _handle_return_approach(self):
        pose = self._pickup_pose(z_override=self.dice_pose['z'] + self.approach_offset)
        self.get_logger().info("[RETURN_APPROACH] Moving above set-down location")
        await self._send_cart_pose(**pose)
        return State.DESCEND_SETDOWN

    async def _handle_descend_setdown(self):
        pose = self._pickup_pose()
        self.get_logger().info("[DESCEND_SETDOWN] Lowering dice to surface")
        await self._send_cart_pose(**pose)
        return State.RELEASE_DICE

    async def _handle_release_dice(self):
        self.get_logger().info("[RELEASE_DICE] Releasing dice")
        await self._send_gripper(self.gripper_open_width, force=30)
        return State.ASCEND_SETDOWN

    async def _handle_ascend_setdown(self):
        pose = self._pickup_pose(z_override=self.dice_pose['z'] + self.approach_offset)
        self.get_logger().info("[ASCEND_SETDOWN] Lifting gripper clear of dice")
        await self._send_cart_pose(**pose)
        return State.CHECK_COMPLETE

    async def _handle_check_complete(self):
        self.current_rep += 1
        if self.current_rep >= self.num_reps:
            self.get_logger().info(
                f"[CHECK_COMPLETE] All {self.num_reps} repetitions done"
            )
            return State.FINAL_HOME
        self.get_logger().info(
            f"[CHECK_COMPLETE] Completed rep {self.current_rep}/{self.num_reps} — looping"
        )
        return State.OPEN_GRIPPER

    async def _handle_final_home(self):
        self.get_logger().info("[FINAL_HOME] Returning to home position")
        await self._send_joint_pose(self.home_joints)
        return State.COMPLETE

    async def _handle_error(self):
        self.get_logger().error("[ERROR] Task halted in error state")
        return State.ERROR  # stay here

    # ------------------------------------------------------------------
    # Main state machine
    # ------------------------------------------------------------------

    async def execute_task(self):
        """Drive the state machine until COMPLETE or ERROR."""
        await self._wait_for_servers()

        handlers = {
            State.IDLE:            self._handle_idle,
            State.MOVE_HOME:       self._handle_move_home,
            State.CHECK_CAMERA:    self._handle_check_camera,
            State.OPEN_GRIPPER:    self._handle_open_gripper,
            State.APPROACH_DICE:   self._handle_approach_dice,
            State.DESCEND_DICE:    self._handle_descend_dice,
            State.GRAB_DICE:       self._handle_grab_dice,
            State.LIFT_DICE:       self._handle_lift_dice,
            State.MOVE_TO_CAMERA:   self._handle_move_to_camera,
            State.SETTLE_AT_CAMERA: self._handle_settle_at_camera,
            State.SHOW_DICE:        self._handle_show_dice,
            State.RETURN_APPROACH: self._handle_return_approach,
            State.DESCEND_SETDOWN: self._handle_descend_setdown,
            State.RELEASE_DICE:    self._handle_release_dice,
            State.ASCEND_SETDOWN:  self._handle_ascend_setdown,
            State.CHECK_COMPLETE:  self._handle_check_complete,
            State.FINAL_HOME:      self._handle_final_home,
            State.ERROR:           self._handle_error,
        }

        while self.state not in (State.COMPLETE, State.ERROR):
            self.get_logger().info(f"━━━ STATE: {self.state.name} ━━━")
            try:
                self.state = await handlers[self.state]()
            except Exception as exc:
                self.get_logger().error(
                    f"Exception in state {self.state.name}: {exc}"
                )
                self.state = State.ERROR

        if self.state == State.COMPLETE:
            self.get_logger().info("✓ Dice inspection task COMPLETE")
            if self.pip_results:
                summary = ", ".join(
                    f"rep{i+1}={v}" if v >= 0 else f"rep{i+1}=?"
                    for i, v in enumerate(self.pip_results)
                )
                self.get_logger().info(f"  Pip counts — {summary}")
        else:
            self.get_logger().error("✗ Dice inspection task ended in ERROR")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = DiceTaskNode()

    # Spin rclpy in a background thread so action callbacks are processed
    # while the asyncio event loop drives the state machine in the main thread.
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        asyncio.run(node.execute_task())
    except KeyboardInterrupt:
        node.get_logger().info("Task interrupted by user")
    finally:
        # Unblock any thread-pool waits (e.g. _wait_cart_arrived) so the
        # process can exit cleanly without needing SIGKILL.
        node._shutdown_event.set()
        shutdown_camera()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
