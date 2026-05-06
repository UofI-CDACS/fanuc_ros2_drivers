#!/usr/bin/env python3
"""
Modbus TCP server — runs on Bunsen (10.8.4.6).

Register map
  Holding Registers (FC3/FC6):
    0  BUNSEN_STATE  : Bunsen state machine (see STATE_* constants)
    1  PIP_PROGRESS  : pip currently being targeted (1–6; whoever just finished sets it to next)
    2  CONV_CMD      : conveyor coordination state machine (see CONV_* constants)
    3  RETRIES       : Bunsen reposition/retry count (cumulative)
    4  BEAKER_STATE  : Beaker state machine (written by Beaker)

  Coils (FC1/FC5):
    0  BUNSEN_READY  : 1 = Bunsen has placed die on front conveyor, Beaker may grab
    1  CAMERA_CLIENT : 1 = Bunsen holds camera token
                       0 = Beaker holds camera token
    2  BEAKER_READY  : 1 = Beaker has placed die on rear conveyor, Bunsen may grab
"""
import os
from pymodbus.server import StartTcpServer
from pymodbus.datastore import (
    ModbusSequentialDataBlock,
    ModbusSlaveContext,
    ModbusServerContext,
)

# ---------------------------------------------------------------------------
# Register addresses
# ---------------------------------------------------------------------------
REG_STATE        = 0   # Bunsen's state (alias: REG_BUNSEN_STATE)
REG_BUNSEN_STATE = 0
REG_PIP_PROGRESS = 1
REG_CONV_CMD     = 2
REG_RETRIES      = 3
REG_BEAKER_STATE = 4   # Beaker's state (written by Beaker)

# ---------------------------------------------------------------------------
# Coil addresses
# ---------------------------------------------------------------------------
COIL_READY         = 0   # Bunsen ready (alias: COIL_BUNSEN_READY)
COIL_BUNSEN_READY  = 0
COIL_CAMERA_CLIENT = 1
COIL_BEAKER_READY  = 2   # Beaker ready (written by Beaker)

# ---------------------------------------------------------------------------
# State machine states (1-9)
# ---------------------------------------------------------------------------
STATE_SETUP        = 1
STATE_WAIT         = 2
STATE_GRAB_DIE     = 3
STATE_PIP_COUNT    = 4
STATE_POSITION_PIP = 5
STATE_PLACE_DIE    = 6
STATE_FINISH       = 7
STATE_RECOVER      = 8
STATE_FAULT        = 9

STATE_NAMES = {
    STATE_SETUP:        'Setup',
    STATE_WAIT:         'Wait',
    STATE_GRAB_DIE:     'GrabDie',
    STATE_PIP_COUNT:    'PipCount',
    STATE_POSITION_PIP: 'PositionPip',
    STATE_PLACE_DIE:    'PlaceDie',
    STATE_FINISH:       'Finish',
    STATE_RECOVER:      'Recover',
    STATE_FAULT:        'Fault',
}

# ---------------------------------------------------------------------------
# Conveyor coordination state machine (CONV_CMD values)
#
# Rear conveyor (Bunsen-owned): carries die from Beaker → Bunsen
# Front conveyor (Beaker-owned): carries die from Bunsen → Beaker
#
# Sequence for Beaker→Bunsen transfer:
#   IDLE → BEAKER_WANTS_SEND → REAR_RUNNING → DIE_ON_REAR → BUNSEN_HAS_DIE → IDLE
#
# Sequence for Bunsen→Beaker transfer:
#   IDLE → BUNSEN_WANTS_SEND → FRONT_RUNNING → DIE_ON_FRONT → BEAKER_HAS_DIE → IDLE
# ---------------------------------------------------------------------------
CONV_IDLE              = 0
CONV_BEAKER_WANTS_SEND = 1   # Beaker: die ready, requesting Bunsen start rear conveyor
CONV_REAR_RUNNING      = 2   # Bunsen: rear conveyor running, Beaker may place die
CONV_DIE_ON_REAR       = 3   # Beaker: die placed on rear conveyor
CONV_BUNSEN_HAS_DIE    = 4   # Bunsen: die picked up — Beaker may reset to IDLE
CONV_BUNSEN_WANTS_SEND = 5   # Bunsen: die ready, requesting Beaker start front conveyor
CONV_FRONT_RUNNING     = 6   # Beaker: front conveyor running, Bunsen may place die
CONV_DIE_ON_FRONT      = 7   # Bunsen: die placed on front conveyor
CONV_BEAKER_HAS_DIE    = 8   # Beaker: die picked up — Bunsen resets to IDLE

NUM_REGS  = 8
NUM_COILS = 8


def main():
    host = os.environ.get('MODBUS_HOST', '0.0.0.0')
    port = int(os.environ.get('MODBUS_PORT', '5020'))

    store = ModbusSlaveContext(
        co=ModbusSequentialDataBlock(0, [False] * NUM_COILS),
        hr=ModbusSequentialDataBlock(0, [0] * NUM_REGS),
    )
    context = ModbusServerContext(slaves=store, single=True)

    print(f'Modbus TCP server on {host}:{port}')
    print('  HR 0  BUNSEN_STATE  1=Setup 2=Wait 3=GrabDie 4=PipCount'
          ' 5=PositionPip 6=PlaceDie 7=Finish 8=Recover 9=Fault')
    print('  HR 1  PIP_PROGRESS  current target pip (1–6; odd=Beaker, even=Bunsen)')
    print('  HR 2  CONV_CMD      conveyor state machine (0=idle … 8=BeakerHasDie)')
    print('  HR 3  RETRIES       Bunsen cumulative reposition count')
    print('  HR 4  BEAKER_STATE  Beaker state (same codes as BUNSEN_STATE)')
    print('  C  0  BUNSEN_READY  1=Bunsen placed die on front conveyor')
    print('  C  1  CAMERA_CLIENT 1=Bunsen holds camera token')
    print('  C  2  BEAKER_READY  1=Beaker placed die on rear conveyor')
    StartTcpServer(context=context, address=(host, port))


if __name__ == '__main__':
    main()
