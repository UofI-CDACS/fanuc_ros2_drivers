import asyncio
from pymodbus.server import StartAsyncTcpServer
from pymodbus.datastore import (
    ModbusSequentialDataBlock,
    ModbusDeviceContext,
    ModbusServerContext,
)
from pymodbus.pdu.device import ModbusDeviceIdentification
import random
from pymodbus.server import ModbusTcpServer
from matrix_math import MatrixMath

import rclpy
from rclpy.node import Node
import ros_robot
from ros_robot import FanucRosNode

#### ROBOT DATA ####
rest_joint_beaker  = [18.446,-7.714,-12.393,.285,-76.969,99.095]
cube_set_pos_beaker = [462.427,-5.95,-179.537,179.414,0.375,117.602]
ready_joint_beaker = [-2.147,3.130,-26.156,-89.491,-91.284,143.768]
cart_rotation_beaker = [89.764,-62.392,179.439]

rest_joint_bunsun = [0,0,0,0,-90,0]
ready_joint_bunsun = [-21.918,7.383,-47.036,107.729,-76.786,-46.637]
cart_rotation_bunsun = [88.005,2.852,-.118]

beaker_calibration_points = [
    [225.115,960.197,337.039],
    [434.779,1232.836,426.126],
    [368.731,1238.980,436.878],
    [504.667,1305.796,326.286]
]

bunsun_calibration_points = [
    [231.214,-1192.713,315.056], 
    [432.814,-919.149,403.453], 
    [368.302,-913.928,417.661],
    [501.935,-846.344,305.532]
]

mins = [225.115,960.197,326.286]
maxs = [504.667,1305.796,436.878]

matrix_math = MatrixMath(beaker_calibration_points, bunsun_calibration_points) #beaker is A, bunsun is B
robot = None


#### MODBUS DATA ####
X_POS = 0
Y_POS = 1
Z_POS = 2
BEAKER_STATE = 3 # 0 = idle, 1 = moving, 2 = approach me, 3 = shutdown
BEAKER_GRIP = 4 # 0 = open, 1 = closed
BUNSUN_STATE = 5
BUNSUN_GRIP = 6
CYCLE_COUNT = 7
MAX_CYCLES = 8

STATE_IDLE = 0
STATE_HOLDING_CUBE = 4
STATE_MOVING = 1
STATE_APPROACH_ME = 2
STATE_SHUTDOWN = 3
STATE_STARTUP = 5

GRIP_OPEN = 0
GRIP_CLOSED = 1

#  data store (holding registers)
datablock = ModbusSequentialDataBlock(0, [0] * 100)  # 100 registers, init to 0
device = ModbusDeviceContext(hr=datablock)
# direct device when just one, or dict when multiple
context = ModbusServerContext(devices=device, single=True)
#constants
fc_as_hex = 3  # Holding registers
max_loops = 2

### Helper Functions ###
def update_state(state=None, grip=None):
    if state is not None:
        device.setValues(fc_as_hex, BEAKER_STATE, [state])
    if grip is not None:
        device.setValues(fc_as_hex, BEAKER_GRIP, [grip])
def update_pos(pos):
    device.setValues(fc_as_hex, X_POS, [int(pos[0])]) #x
    device.setValues(fc_as_hex, Y_POS, [int(pos[1])]) #y
    device.setValues(fc_as_hex, Z_POS, [int(pos[2])]) #z
def generate_random_pos():
    x = int(random.uniform(mins[0], maxs[0]))
    y = int(random.uniform(mins[1], maxs[1]))
    z = int(random.uniform(mins[2], maxs[2]))
    return [x,y,z]
def get_pos_from_device():
    x = device.getValues(fc_as_hex, X_POS, count=1)[0]
    y = device.getValues(fc_as_hex, Y_POS, count=1)[0]
    z = device.getValues(fc_as_hex, Z_POS, count=1)[0]
    return [x,y,z]
def increment_cycle_count():
    count = device.getValues(fc_as_hex, CYCLE_COUNT, count=1)[0]
    device.setValues(fc_as_hex, CYCLE_COUNT, [count + 1])
    return count + 1 == max_loops
def print_registers():
    x = device.getValues(fc_as_hex, X_POS, count=1)[0]
    y = device.getValues(fc_as_hex, Y_POS, count=1)[0]
    z = device.getValues(fc_as_hex, Z_POS, count=1)[0]
    state = device.getValues(fc_as_hex, BEAKER_STATE, count=1)[0]
    grip = device.getValues(fc_as_hex, BEAKER_GRIP, count=1)[0]
    bunsun_state = device.getValues(fc_as_hex, BUNSUN_STATE, count=1)[0]
    bunsun_grip = device.getValues(fc_as_hex, BUNSUN_GRIP, count=1)[0]
    print(f"X: {x}, Y: {y}, Z: {z}, BEAKER_STATE: {state}, BEAKER_GRIP: {grip}, BUNSUN_STATE: {bunsun_state}, BUNSUN_GRIP: {bunsun_grip}")
### State Functions ###
async def startup_state(context):
    update_state(state=STATE_STARTUP)
    device = context[1]
    #Go to neutral
    await robot.move_joints(rest_joint_beaker)
    #grab cube
    await robot.open_gripper_schunk('open')
    await robot.move_cartesian(cube_set_pos_beaker)
    await robot.open_gripper_schunk('close')
    #Go to neutral
    await robot.move_joints(rest_joint_beaker)
    #go to idle
    await robot.move_joints(ready_joint_beaker)

    print("Ready to send cube")
    update_state(state = STATE_MOVING, grip = GRIP_CLOSED)

    #Go to next state
    task = asyncio.create_task(approach_state(context))

async def approach_state(context):
    device = context[1]
    #generate random pos
    pos = generate_random_pos()
    #go to pos
    await robot.move_cartesian(pos)
    #send pos
    update_pos(pos)
    
    print("Sent pos, ready for bunsun to approach")
    update_state(state=STATE_APPROACH_ME) #approach me

    #wait for bunsun to grip state should look like grip 1 and state 0
    while True:
        grip = device.getValues(fc_as_hex, BUNSUN_GRIP, count=1)
        state = device.getValues(fc_as_hex, BUNSUN_STATE, count=1)
        if grip and state and grip[0] == GRIP_CLOSED and state[0] == STATE_HOLDING_CUBE:
            print("Bunsun is ready with grip, moving back to idle")
            break
        #print_registers()
        await asyncio.sleep(0.1)

    
   
    #release
    await robot.open_gripper_schunk('open')
    update_state(grip=GRIP_OPEN)
    update_state(state=STATE_MOVING) 
    #scoot back
    await robot.move_cartesian(matrix_math.scoot(pos, -100, direction=1)) #scoot back in y by 5cm
    #go to idle
    await robot.move_joints(ready_joint_beaker)

    print("Ready to grab cube from bunsun")
    update_state(state=STATE_IDLE, grip=GRIP_OPEN) #idle and grip released

    #go to next state
    task = asyncio.create_task(idle_state(context))

async def idle_state(context):
    device = context[1]
    update_state(state=STATE_IDLE, grip=GRIP_OPEN)
    #wait for bunsun to have a new pos and be ready to send it, state should look like grip 1 and state 2
    while True:
        grip = device.getValues(fc_as_hex, BUNSUN_GRIP, count=1)
        state = device.getValues(fc_as_hex, BUNSUN_STATE, count=1)
        if grip and state and grip[0] == GRIP_CLOSED and state[0] == STATE_APPROACH_ME:
            print("Bunsun is ready with a new pos")
            break
        await asyncio.sleep(0.1)
    
    update_state(state=STATE_MOVING)
    #get pos
    pos = get_pos_from_device()
    #move to pos with scoot
    await robot.move_cartesian(matrix_math.scoot(pos, -100, direction=1)) #scoot back in y by 5cm
    #move to pos
    await robot.move_cartesian(pos)
    #grab
    await robot.open_gripper_schunk('close')
    update_state(state=STATE_HOLDING_CUBE, grip=GRIP_CLOSED) #idle and gripped

    #go to next state
    task = asyncio.create_task(holding_state(context))

async def holding_state(context):
    device = context[1]
    
    #Wait for bunsun to release so that I can move safely
    while True:
        grip = device.getValues(fc_as_hex, BUNSUN_GRIP, count=1)
        state = device.getValues(fc_as_hex, BUNSUN_STATE, count=1)
        if grip and state and grip[0] == GRIP_OPEN and state[0] == STATE_IDLE:
            print("Bunsun released, ready to move to idle")
            break
        await asyncio.sleep(0.1)

    update_state(state=STATE_MOVING)
    #scoot back
    pos = get_pos_from_device()
    await robot.move_cartesian(matrix_math.scoot(pos, -100, direction=1)) #scoot back in y by 5cm
    #go to idle
    await robot.move_joints(ready_joint_beaker)

    #increment cycle count and check if done
    if increment_cycle_count():
        print("Completed all cycles, shutting down")
        task = asyncio.create_task(shutdown_state(context))
    else:
        print("Ready for next cycle")
        task = asyncio.create_task(approach_state(context))

async def shutdown_state(context):
    update_state(state=STATE_SHUTDOWN)
    #move to neutral
    await robot.move_joints(rest_joint_beaker)
    #move to cube set
    await robot.move_cartesian(cube_set_pos_beaker)
    #release cube
    await robot.open_gripper_schunk('open')
    #move to neutral
    await robot.move_joints(rest_joint_beaker)
    print("Shutdown complete")

### MAIN FUNCTION ###
async def run():
    # create server
    server = ModbusTcpServer(
        context,
        address=(
            "0.0.0.0",
            5020,
        )
    )
    # setup robot node
    rclpy.init()
    global robot
    robot = FanucRosNode('beaker')

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(robot)
    asyncio.create_task(ros_robot.spin_robot(executor))

    #set initial state
    device.setValues(fc_as_hex, BEAKER_STATE, [STATE_STARTUP])
    device.setValues(fc_as_hex, BEAKER_GRIP, [GRIP_OPEN])
    device.setValues(fc_as_hex, CYCLE_COUNT, [0])
    device.setValues(fc_as_hex, MAX_CYCLES, [max_loops])
    # start state machine
    task = asyncio.create_task(startup_state(context))

    print("Starting Modbus server on port 5020...")
    await server.serve_forever()


if __name__ == "__main__":
    asyncio.run(run())
