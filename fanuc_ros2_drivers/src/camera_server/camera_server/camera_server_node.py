"""
Camera Server Node
==================
Handles image capture and pip counting on behalf of both robots.

Capture handshake
-----------------
1. A robot moves into position and sets its CAPTURE_REQUEST coil via Modbus.
2. This node detects the set coil on its polling timer.
3. Image is captured and pips are counted via the MindVision camera pipeline
   implemented in pip_test.py (detect_pips).
4. The pip count (1–6, or 0 on failure) is written to that robot's
   PIP_RESULT holding register.
5. The CAPTURE_REQUEST coil is cleared — this is the robot's signal that
   the result is ready to read.

Only one robot can hold CAMERA_ACTIVE at a time (enforced by the task
nodes), so simultaneous requests should not occur.  If they do, robot 1
is served first.

Requires: pymodbus >= 3.0, mvsdk, opencv-python
"""

import contextlib
import io
import os as _os
import signal
import sys as _sys
import time

import rclpy
from rclpy.node import Node

try:
    from pymodbus.client import ModbusTcpClient
    from pymodbus.exceptions import ModbusException
except ImportError as exc:
    raise ImportError("pymodbus >= 3.0 is required: pip install pymodbus") from exc

from modbus_server.register_map import (
    coil_capture_request,
    reg_pip_result,
)

# fanuc_ros2_drivers root — needed for mvsdk and pip_test.
# setup_ws.bash adds this to PYTHONPATH on the partner's machine; the
# fallback below covers running directly from the source tree.
_DRIVERS_ROOT = _os.path.abspath(
    _os.path.join(_os.path.dirname(_os.path.abspath(__file__)), '..', '..', '..', '..')
)
if _os.path.isfile(_os.path.join(_DRIVERS_ROOT, 'mvsdk.py')):
    if _DRIVERS_ROOT not in _sys.path:
        _sys.path.insert(0, _DRIVERS_ROOT)

try:
    import mvsdk
    from pip_test import open_camera, grab_frame, detect_pips
    _CAMERA_AVAILABLE = True
    _CAMERA_ERR = ''
except ImportError as _e:
    _CAMERA_AVAILABLE = False
    _CAMERA_ERR = str(_e)


class CameraServerNode(Node):
    """
    Parameters
    ----------
    modbus_host      : str    Modbus server address          (default: localhost)
    modbus_port      : int    Modbus server port             (default: 1502)
    poll_interval    : float  Seconds between coil checks    (default: 0.1)
    camera_index     : int    MindVision camera device index (default: 0)
    capture_timeout  : float  Max seconds to wait for frame  (default: 5.0)
    """

    def __init__(self):
        super().__init__('camera_server_node')
        self.declare_parameter('modbus_host',     'localhost')
        self.declare_parameter('modbus_port',     1502)
        self.declare_parameter('poll_interval',   0.1)
        self.declare_parameter('camera_index',    0)
        self.declare_parameter('capture_timeout', 5.0)

        # ── Camera ───────────────────────────────────────────────────────
        self._h_cam     = None
        self._frame_buf = None
        self._is_color  = None

        if _CAMERA_AVAILABLE:
            self._open_camera()
        else:
            self.get_logger().error(
                f"mvsdk / pip_test not importable — camera disabled: {_CAMERA_ERR}"
            )

        # ── Modbus ───────────────────────────────────────────────────────
        self._client = ModbusTcpClient(
            self.get_parameter('modbus_host').value,
            port=self.get_parameter('modbus_port').value,
        )
        self._connect_with_retry()

        poll = self.get_parameter('poll_interval').value
        self.create_timer(poll, self._poll)
        self.get_logger().info(
            f"Camera server ready — polling every {poll:.2f}s  "
            f"camera_index={self.get_parameter('camera_index').value}"
        )

    # ------------------------------------------------------------------
    # Camera lifecycle
    # ------------------------------------------------------------------

    def _release_camera(self):
        """Uninitialize the camera handle, silently ignoring errors."""
        if self._h_cam is not None:
            try:
                mvsdk.CameraStop(self._h_cam)
                mvsdk.CameraUnInit(self._h_cam)
            except Exception:
                pass
            self._h_cam = None

    def _open_camera(self, retries: int = 5, delay: float = 5.0):
        idx = self.get_parameter('camera_index').value
        for attempt in range(1, retries + 1):
            # Always release any stale handle before re-attempting — this clears
            # err:-14 (device busy) left by a previous crashed session.
            self._release_camera()
            try:
                self._h_cam, self._frame_buf, self._is_color = open_camera(idx)
                self.get_logger().info(f"MindVision camera {idx} opened")
                return
            except Exception as exc:
                self.get_logger().warn(
                    f"Camera open failed (attempt {attempt}/{retries}): {exc}"
                    + (f" — retrying in {delay:.0f}s" if attempt < retries else "")
                )
                if attempt < retries:
                    time.sleep(delay)
        self.get_logger().error("Could not open camera after retries")

    def _grab_frame(self):
        if self._h_cam is None:
            return None
        try:
            return grab_frame(self._h_cam, self._frame_buf, self._is_color)
        except Exception as exc:
            # err:-37 (network send error) means the GigE session is stale.
            # Release and reopen so the next capture attempt gets a fresh handle.
            self.get_logger().error(f"Frame grab failed: {exc} — reinitializing camera")
            self._release_camera()
            self._open_camera()
            return None

    # ------------------------------------------------------------------
    # Modbus connection
    # ------------------------------------------------------------------

    def _connect_with_retry(self, retries: int = 10, delay: float = 1.0):
        for attempt in range(1, retries + 1):
            if self._client.connect():
                self.get_logger().info("Connected to Modbus server")
                return
            self.get_logger().warn(
                f"Modbus connection attempt {attempt}/{retries} failed — retrying in {delay}s"
            )
            time.sleep(delay)
        raise RuntimeError("Could not connect to Modbus server after retries")

    # ------------------------------------------------------------------
    # Polling loop
    # ------------------------------------------------------------------

    def _poll(self):
        """Check capture request coils and serve any pending request."""
        for robot_index in (1, 2):
            coil_addr = coil_capture_request(robot_index)
            result = self._client.read_coils(coil_addr, count=1)
            if result.isError():
                self.get_logger().warn(
                    f"Failed to read capture_request coil for robot {robot_index}"
                )
                continue
            if result.bits[0]:
                self._serve_capture(robot_index)
                return

    # ------------------------------------------------------------------
    # Capture and write result
    # ------------------------------------------------------------------

    def _serve_capture(self, robot_index: int):
        self.get_logger().info(f"[robot {robot_index}] Capture request received")

        pip_count = self._detect_pip_count()

        self.get_logger().info(f"[robot {robot_index}] pip count = {pip_count}")

        reg = reg_pip_result(robot_index)
        write_result = self._client.write_register(reg, pip_count)
        if write_result.isError():
            self.get_logger().error(
                f"[robot {robot_index}] Failed to write pip result to register {reg}"
            )

        coil_addr = coil_capture_request(robot_index)
        clear_result = self._client.write_coil(coil_addr, False)
        if clear_result.isError():
            self.get_logger().error(
                f"[robot {robot_index}] Failed to clear capture_request coil"
            )

    # ------------------------------------------------------------------
    # Pip detection
    # ------------------------------------------------------------------

    def _detect_pip_count(self) -> int:
        """Capture a frame and return the pip count (1–6), or 0 on failure."""
        if not _CAMERA_AVAILABLE or self._h_cam is None:
            self.get_logger().error("Camera not available — returning 0")
            return 0

        frame = self._grab_frame()
        if frame is None:
            return 0

        try:
            # detect_pips has diagnostic print statements; redirect to suppress them
            with contextlib.redirect_stdout(io.StringIO()):
                _, _, _, _, pip_count = detect_pips(frame)
            return pip_count
        except Exception as exc:
            self.get_logger().error(f"detect_pips raised: {exc}")
            return 0

    # ------------------------------------------------------------------
    # Shutdown
    # ------------------------------------------------------------------

    def destroy_node(self):
        self._release_camera()
        self._client.close()
        super().destroy_node()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = CameraServerNode()

    # SIGTERM (sent by ros2 launch on shutdown) does not raise KeyboardInterrupt,
    # so the finally block below would never run without an explicit handler.
    # Release the camera on SIGTERM so the GigE session is not left open.
    def _on_sigterm(sig, frame):
        node.destroy_node()
        rclpy.shutdown()
        _sys.exit(0)

    signal.signal(signal.SIGTERM, _on_sigterm)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
