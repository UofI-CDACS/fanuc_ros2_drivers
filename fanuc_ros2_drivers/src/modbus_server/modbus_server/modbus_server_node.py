"""
Modbus TCP Server Node
======================
Runs a pymodbus TCP server that holds shared state for both robots:
  - Holding registers: robot state (one per robot)
  - Coils: robot ready, camera ownership, pip-done flags

The server runs in a background asyncio event loop so the ROS2 node
can still spin and respond to shutdown signals normally.

Requires: pymodbus >= 3.0
    pip install pymodbus

Usage:
    ros2 run modbus_server modbus_server_node
    ros2 run modbus_server modbus_server_node --ros-args -p port:=1502
"""

import asyncio
import signal
import socket
import sys
import threading

import rclpy
from rclpy.node import Node

from modbus_server.register_map import TOTAL_COILS, TOTAL_REGISTERS

try:
    from pymodbus.server import StartAsyncTcpServer
    from pymodbus.datastore import (
        ModbusSequentialDataBlock,
        ModbusSlaveContext,
        ModbusServerContext,
    )
except ImportError as exc:
    raise ImportError(
        "pymodbus >= 3.0 is required: pip install pymodbus"
    ) from exc


class ModbusServerNode(Node):
    """
    Parameters
    ----------
    host : str   Bind address for the TCP server  (default: 0.0.0.0)
    port : int   TCP port                          (default: 1502)
    log_interval : float  Seconds between state log prints (default: 10.0, 0 = off)
    """

    def __init__(self):
        super().__init__('modbus_server_node')
        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 1502)
        self.declare_parameter('log_interval', 10.0)

        self._mb_context = self._build_context()

        # Start the pymodbus server in its own asyncio event loop / thread
        self._server_loop = asyncio.new_event_loop()
        self._server_thread = threading.Thread(
            target=self._run_server_loop, daemon=True, name='modbus_server'
        )
        self._server_thread.start()

        log_interval = self.get_parameter('log_interval').value
        if log_interval > 0:
            self.create_timer(log_interval, self._log_state)

        host = self.get_parameter('host').value
        port = self.get_parameter('port').value
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                s.connect(('8.8.8.8', 80))
                machine_ip = s.getsockname()[0]
        except OSError:
            machine_ip = 'unknown'
        self.get_logger().info(
            f"Modbus server listening on {host}:{port}  "
            f"(machine IP: {machine_ip})"
        )

    # ------------------------------------------------------------------
    # Datastore
    # ------------------------------------------------------------------

    def _build_context(self) -> ModbusServerContext:
        store = ModbusSlaveContext(
            di=ModbusSequentialDataBlock(1, [0]),
            co=ModbusSequentialDataBlock(1, [False] * TOTAL_COILS),
            hr=ModbusSequentialDataBlock(1, [0]   * TOTAL_REGISTERS),
            ir=ModbusSequentialDataBlock(1, [0]),
        )
        return ModbusServerContext(slaves=store, single=True)

    # ------------------------------------------------------------------
    # Server lifecycle
    # ------------------------------------------------------------------

    def _run_server_loop(self):
        """Entry point for the background server thread."""
        asyncio.set_event_loop(self._server_loop)
        try:
            self._server_loop.run_until_complete(self._serve())
        except Exception as exc:
            # Log via print since the ROS2 logger is not thread-safe here
            print(f"[modbus_server] server error: {exc}")

    async def _serve(self):
        host = self.get_parameter('host').value
        port = self.get_parameter('port').value
        await StartAsyncTcpServer(context=self._mb_context, address=(host, port))

    # ------------------------------------------------------------------
    # Debug logging
    # ------------------------------------------------------------------

    def _log_state(self):
        """Periodically log all register and coil values."""
        store = self._mb_context[0]
        regs  = store.getValues(3, 0, TOTAL_REGISTERS)
        coils = store.getValues(1, 0, TOTAL_COILS)

        coil_names = [
            'r1_ready', 'r2_ready',
            'r1_cam',   'r2_cam',
            'pip1', 'pip2', 'pip3', 'pip4', 'pip5', 'pip6',
            'r1_capture_req', 'r2_capture_req',
        ]
        coil_str = '  '.join(
            f"{name}={'1' if v else '0'}"
            for name, v in zip(coil_names, coils)
        )
        self.get_logger().info(
            f"Modbus state | "
            f"r1_state={regs[0]}  r2_state={regs[1]}  "
            f"r1_pip={regs[2]}  r2_pip={regs[3]} | {coil_str}"
        )

    # ------------------------------------------------------------------
    # Shutdown
    # ------------------------------------------------------------------

    def destroy_node(self):
        self._server_loop.call_soon_threadsafe(self._server_loop.stop)
        super().destroy_node()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = ModbusServerNode()

    # SIGTERM (sent by ros2 launch on shutdown) does not raise KeyboardInterrupt.
    # Without this handler the asyncio event loop is never stopped cleanly, leaving
    # port 1502 in TIME_WAIT and blocking the next launch for ~60 seconds.
    def _on_sigterm(sig, frame):
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

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
