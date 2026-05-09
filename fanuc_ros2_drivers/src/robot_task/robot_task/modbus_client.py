"""
Modbus client helper for the robot task node.

Wraps pymodbus synchronous TCP client with robot-aware read/write methods
keyed to the shared register map.  All public methods are blocking and
intended to be called via asyncio.run_in_executor so they never block the
asyncio event loop.

Requires: pymodbus >= 3.0
"""

import time

try:
    from pymodbus.client import ModbusTcpClient
    from pymodbus.exceptions import ModbusException
except ImportError as exc:
    raise ImportError(
        "pymodbus >= 3.0 is required: pip install pymodbus"
    ) from exc

from modbus_server.register_map import (
    coil_robot_ready,
    coil_robot_camera,
    coil_other_robot_camera,
    coil_capture_request,
    coil_pip_done,
    reg_robot_state,
    reg_pip_result,
    TOTAL_COILS,
    TOTAL_REGISTERS,
)


class RobotModbusClient:
    """
    Synchronous Modbus client scoped to one robot.

    Parameters
    ----------
    host         : str   Modbus server host
    port         : int   Modbus server port
    robot_index  : int   1 or 2 — determines which registers/coils belong to us
    """

    def __init__(self, host: str, port: int, robot_index: int):
        if robot_index not in (1, 2):
            raise ValueError(f"robot_index must be 1 or 2, got {robot_index}")
        self._robot_index = robot_index
        self._client = ModbusTcpClient(host, port=port)

    # ------------------------------------------------------------------
    # Connection
    # ------------------------------------------------------------------

    def connect(self) -> bool:
        return self._client.connect()

    def disconnect(self):
        self._client.close()

    # ------------------------------------------------------------------
    # State register
    # ------------------------------------------------------------------

    def set_state(self, state_value: int):
        self._write_register(reg_robot_state(self._robot_index), state_value)

    # ------------------------------------------------------------------
    # Ready coil
    # ------------------------------------------------------------------

    def set_ready(self, ready: bool):
        self._write_coil(coil_robot_ready(self._robot_index), ready)

    # ------------------------------------------------------------------
    # Camera coils
    # ------------------------------------------------------------------

    def claim_camera(self):
        self._write_coil(coil_robot_camera(self._robot_index), True)

    def release_camera(self):
        self._write_coil(coil_robot_camera(self._robot_index), False)

    def other_robot_has_camera(self) -> bool:
        result = self._client.read_coils(
            coil_other_robot_camera(self._robot_index), count=1
        )
        if result.isError():
            return False
        return bool(result.bits[0])

    def camera_collision(self) -> bool:
        """Return True if both robots claimed the camera simultaneously (TOCTOU race)."""
        result = self._client.read_coils(
            min(coil_robot_camera(1), coil_robot_camera(2)), count=2
        )
        if result.isError():
            return False
        return all(result.bits[:2])

    # ------------------------------------------------------------------
    # Capture request / pip result handshake
    # ------------------------------------------------------------------

    def request_capture(self):
        self._write_coil(coil_capture_request(self._robot_index), True)

    def is_capture_pending(self) -> bool:
        result = self._client.read_coils(
            coil_capture_request(self._robot_index), count=1
        )
        if result.isError():
            return True
        return bool(result.bits[0])

    def read_pip_result(self) -> int:
        result = self._client.read_holding_registers(
            reg_pip_result(self._robot_index), count=1
        )
        if result.isError():
            return 0
        return int(result.registers[0])

    # ------------------------------------------------------------------
    # Pip done coils
    # ------------------------------------------------------------------

    def set_pip_done(self, pip: int):
        self._write_coil(coil_pip_done(pip), True)

    def is_pip_done(self, pip: int) -> bool:
        result = self._client.read_coils(coil_pip_done(pip), count=1)
        if result.isError():
            return False
        return bool(result.bits[0])

    # ------------------------------------------------------------------
    # Low-level helpers with simple retry
    # ------------------------------------------------------------------

    def _write_coil(self, address: int, value: bool, retries: int = 3):
        for attempt in range(retries):
            result = self._client.write_coil(address, value)
            if not result.isError():
                return
            time.sleep(0.05)
        raise ModbusException(
            f"Failed to write coil {address}={value} after {retries} attempts"
        )

    def _write_register(self, address: int, value: int, retries: int = 3):
        for attempt in range(retries):
            result = self._client.write_register(address, value)
            if not result.isError():
                return
            time.sleep(0.05)
        raise ModbusException(
            f"Failed to write register {address}={value} after {retries} attempts"
        )
