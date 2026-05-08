import asyncio
from json import decoder
from unittest import result
from pymodbus.client import AsyncModbusTcpClient
import random
from pymodbus.payload import BinaryPayloadDecoder
from pymodbus.constants import Endian

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

async def run_client():
    # Connect to slave (use "localhost" or 0.0.0.0)
    client = AsyncModbusTcpClient("localhost", port=5020)
    await client.connect()

    try:
        while True:
            # Flip the reset pin to reset the robot
            await client.write_coil(COIL_REGISTER_MAP["Reset"], [1])
            """await client.write_coil(COIL_REGISTER_MAP["Ready_To_Receive"], [1])
            await client.write_coil(COIL_REGISTER_MAP["In_Recieve_Position"], [1])
            await client.write_coil(COIL_REGISTER_MAP["Gripper_Closed"], [1])
            await client.write_coil(COIL_REGISTER_MAP["Has_Dice"], [1])
            # Read Bill's X, Y, Z holding registers and decode as floats
            result_x = await client.read_holding_registers(address=HOLDING_REGISTER_MAP["X"], count=2, slave=1)
            decoder = BinaryPayloadDecoder.fromRegisters(result_x.registers, byteorder=Endian.BIG, wordorder=Endian.BIG)
            value_x = decoder.decode_32bit_float()
            result_y = await client.read_holding_registers(address=HOLDING_REGISTER_MAP["Y"], count=2, slave=1)
            decoder = BinaryPayloadDecoder.fromRegisters(result_y.registers, byteorder=Endian.BIG, wordorder=Endian.BIG)
            value_y = decoder.decode_32bit_float()
            result_z = await client.read_holding_registers(address=HOLDING_REGISTER_MAP["Z"], count=2, slave=1)
            decoder = BinaryPayloadDecoder.fromRegisters(result_z.registers, byteorder=Endian.BIG, wordorder=Endian.BIG)
            value_z = decoder.decode_32bit_float()
            print(f"X: {value_x}, Y: {value_y}, Z: {value_z}")
            # Read Bill's coils and print their values
            Ready_to_pass = await client.read_coils(address=COIL_REGISTER_MAP["Ready_To_Pass"],slave=1, count=1)
            Holding_dice = await client.read_coils(address=COIL_REGISTER_MAP["Holding_Dice"],slave=1, count=1)
            In_hand_off_position = await client.read_coils(address=COIL_REGISTER_MAP["In_Hand_Off_Position"],slave=1, count=1)
            print(f"Ready_To_Pass: {Ready_to_pass.bits[0]}, Holding_Dice: {Holding_dice.bits[0]}, In_Hand_Off_Position: {In_hand_off_position.bits[0]}")
            """
            await asyncio.sleep(1)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        client.close()


if __name__ == "__main__":
    asyncio.run(run_client())
