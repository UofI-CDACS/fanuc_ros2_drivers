"""
Modbus register and coil address map — shared by server and all clients.

Layout
------
Holding Registers  (FC 3 read / FC 6 write)  16-bit unsigned integers
  0  robot_1_state
  1  robot_2_state
  2  robot_1_pip_result    pip count written by camera server after capture (1-6, 0 = no result)
  3  robot_2_pip_result

Coils  (FC 1 read / FC 5 write)  boolean
  0   robot_1_ready
  1   robot_2_ready
  2   robot_1_camera_active    robot 1 currently holds exclusive camera access
  3   robot_2_camera_active    robot 2 currently holds exclusive camera access
  4   pip_1_done               dice face showing 1 pip delivered to conveyor
  5   pip_2_done
  6   pip_3_done
  7   pip_4_done
  8   pip_5_done
  9   pip_6_done
  10  robot_1_capture_request  robot 1 is in position and requesting a capture
  11  robot_2_capture_request  robot 2 is in position and requesting a capture

Capture handshake
-----------------
  Robot sets CAPTURE_REQUEST coil.
  Camera server sees coil, captures image, writes pip count to PIP_RESULT register,
  then clears CAPTURE_REQUEST coil to signal completion.
  Robot polls until coil is cleared, then reads PIP_RESULT register.

Conveyor coordination
---------------------
  Each robot has a dedicated drop conveyor and pickup conveyor (two physical belts).
  Robots wait on their own physical proximity sensor — no Modbus coil needed for
  conveyor handoff.
"""

# ── Holding register addresses ────────────────────────────────────────────
REG_ROBOT_1_STATE      = 0
REG_ROBOT_2_STATE      = 1
REG_ROBOT_1_PIP_RESULT = 2
REG_ROBOT_2_PIP_RESULT = 3

TOTAL_REGISTERS = 4

# ── Coil addresses ────────────────────────────────────────────────────────
COIL_ROBOT_1_READY           = 0
COIL_ROBOT_2_READY           = 1
COIL_ROBOT_1_CAMERA_ACTIVE   = 2
COIL_ROBOT_2_CAMERA_ACTIVE   = 3
COIL_PIP_1_DONE              = 4
COIL_PIP_2_DONE              = 5
COIL_PIP_3_DONE              = 6
COIL_PIP_4_DONE              = 7
COIL_PIP_5_DONE              = 8
COIL_PIP_6_DONE              = 9
COIL_ROBOT_1_CAPTURE_REQUEST = 10
COIL_ROBOT_2_CAPTURE_REQUEST = 11

TOTAL_COILS = 12

# ── Robot state values (written to REG_ROBOT_x_STATE) ────────────────────
ROBOT_STATE_IDLE        = 0
ROBOT_STATE_HOMING      = 1
ROBOT_STATE_PICKING     = 2
ROBOT_STATE_TO_CAMERA   = 3
ROBOT_STATE_INSPECTING  = 4
ROBOT_STATE_TO_CONVEYOR = 5
ROBOT_STATE_RETRY       = 6
ROBOT_STATE_COMPLETE    = 7
ROBOT_STATE_ERROR       = 99

# ── Helpers ───────────────────────────────────────────────────────────────

def coil_robot_ready(robot_index: int) -> int:
    return COIL_ROBOT_1_READY if robot_index == 1 else COIL_ROBOT_2_READY


def coil_robot_camera(robot_index: int) -> int:
    return COIL_ROBOT_1_CAMERA_ACTIVE if robot_index == 1 else COIL_ROBOT_2_CAMERA_ACTIVE


def coil_other_robot_camera(robot_index: int) -> int:
    return COIL_ROBOT_2_CAMERA_ACTIVE if robot_index == 1 else COIL_ROBOT_1_CAMERA_ACTIVE


def coil_capture_request(robot_index: int) -> int:
    return COIL_ROBOT_1_CAPTURE_REQUEST if robot_index == 1 else COIL_ROBOT_2_CAPTURE_REQUEST


def reg_robot_state(robot_index: int) -> int:
    return REG_ROBOT_1_STATE if robot_index == 1 else REG_ROBOT_2_STATE


def reg_pip_result(robot_index: int) -> int:
    return REG_ROBOT_1_PIP_RESULT if robot_index == 1 else REG_ROBOT_2_PIP_RESULT


def coil_pip_done(pip: int) -> int:
    if not 1 <= pip <= 6:
        raise ValueError(f"pip must be 1–6, got {pip}")
    return COIL_PIP_1_DONE + (pip - 1)
