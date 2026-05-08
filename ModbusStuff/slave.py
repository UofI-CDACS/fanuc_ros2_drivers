import asyncio
from pymodbus.server import StartAsyncTcpServer
from pymodbus.datastore import (
    ModbusSequentialDataBlock,
    ModbusSlaveContext,
    ModbusServerContext,
)
from pymodbus.device import ModbusDeviceIdentification
import random
from pymodbus.server import ModbusTcpServer

#  data store (holding registers)
device = ModbusSlaveContext(
    hr=ModbusSequentialDataBlock(0, [0] * 16),
    co=ModbusSequentialDataBlock(0, [0] * 16), 
    )
# direct device when just one, or dict when multiple
context = ModbusServerContext(slaves=device, single=True)

COIL_REGISTER_MAP = {
    # Bill
    "Ready_To_Pass": 0, 
    "Holding_Dice": 1,
    "In_Hand_Off_Position": 2,
    #DJ
    "Pose_valid": 3,
    "Ready_To_Receive": 4,
    "In_Recieve_Position": 5,
    "Gripper_Closed": 6,
    "Has_Dice": 7,
    #Shared
    "Fault": 8,
    "Reset": 9,
    "Cycle_Active": 10,
}
HOLDING_REGISTER_MAP = {
    # Bill
    "X": 0,
    "Y": 2,
    "Z": 4,
}

# Device info (for funsies)
identity = ModbusDeviceIdentification()
identity.VendorName = "Team Pregnancy"
identity.ProductName = "Bill and Djs Amazing Adventure Through Passing Parts and Not Dying"
identity.ModelName = "The Greatest Dice Passer In The Known Universe"


async def updating_task(context):
    """Continuously scan and display register values (masters can write to any register)."""
    slave_id = 0  # Use 0 for single=True context
    fc_holding = 3  # Function code 3 = holding registers
    fc_coils = 1    # Function code 1 = coils

    while True:
        device = context[slave_id]

        # Read current holding register values
        holding_values = device.getValues(fc_holding, 0, count=len(HOLDING_REGISTER_MAP))
        
        # Read current coil values
        coil_values = device.getValues(fc_coils, 0, count=len(COIL_REGISTER_MAP))

        # Display holding registers
        print("\n--- Holding Registers ---")
        for name, addr in HOLDING_REGISTER_MAP.items():
            if addr <= len(holding_values):
                print(f"  {name}: {holding_values[addr]}")

        # Display coils
        print("--- Coils ---")
        for name, addr in COIL_REGISTER_MAP.items():
            if addr < len(coil_values):
                print(f"  {name}: {bool(coil_values[addr])}")

        await asyncio.sleep(1)  # Scan every 1s


async def run():
    # create server
    server = ModbusTcpServer(
        context,
        address=(
            "0.0.0.0",
            5020,
        ),  # use port 5020 instead of 502 (requires root). can do with 502, but thats on you to figure out
        identity=identity,
    )

    # start background update task
    task = asyncio.create_task(updating_task(context))

    print("Starting Modbus server on port 5020...")
    await server.serve_forever()


if __name__ == "__main__":
    asyncio.run(run())
