##
# Manufacturing Line

# Tasks:
#   Working with your partner and using assistance from Claude, we will write ROS2 code to perform a few tasks:

#     Pick up the Dice from in front of robot 1 (team decides robot 1 and robot 2)
#     Robot 1 presents it to the camera and captures an image (overhead for FANUC’s)
#     Run process to count the pips in the dice, make sure to save this somewhere in your program and communicate it to the other robot. You will need to do this until you see pip count of 1. That marks the “start” state.
#         NOTE: Only one camera server node is allowed. You decide who will spin that up, then you will both have client nodes for each robot when camera is needed.
#     You will then pass the dice to robot 2 using the conveyor
#         If pip count is even, use the "front" (closest) conveyor

#         If pip count is odd, use the "back" (furthest) conveyor
#     With this in mind, each robot controls one conveyor so you must communicate in order to use both conveyors.

# Process will be continued until all pips have been counted SEQUENTIALLY (start at 1, finish on 6), keeping track of retries (how many times robot takes to get the correct number of pips - TOTAL COUNT for each robot + together). So you will have total retries for robot 1 and 2, as well as both combined, presented neatly.

#     Robot who has pip 6 at the end is in charge of placing the dice down in front of itself (this will be robot 2).

import sys
sys.path.append("../src/dependencies/")

import time
from time import sleep

# Machine Vision
import mvsdk
import cv2 as cv
import numpy as np

# Modbus packages
import asyncio
import os
from dotenv import load_dotenv
from pymodbus.client import AsyncModbusTcpClient

load_dotenv(os.path.join(os.path.dirname(__file__), '.env'))
MODBUS_IP = os.environ["MODBUS_IP"]
MODBUS_PORT = int(os.environ["MODBUS_PORT"])

positions = [
    [0.0, 0.0, 0.0, 0.0, -90.0, -50.0], # home pos (joint)
    [300.0, -600.0, 400.0, 90.0, 45.0, 0.0],   # standby pos (cart)
    [0.0, 0.0, 0.0, 90.0, 45.0, 0.0],  # transfer pos (cart)
    [-60.0, 10.0, -40.0, -30.0, 50.0, 0.0]  # standby pos (joint)
]

SIGNALS = {
    # DJ
    "DJ_GRIPPER_CLOSED":        0,
    "DJ_HAS_DICE":              1,
    "DJ_READY_FOR_PICTURE":     2,
    "DJ_AT_CONVEYOR":           3, 
    "DJ_CONVEYOR_ACTIVE":       4,
    "DJ_CONVEYOR_DICE_READY":   5,
    # Bill
    "BILL_GRIPPER_CLOSED":      10,
    "BILL_HAS_DICE":            11,
    "BILL_READY_FOR_PICTURE":   12,
    "BILL_AT_CONVEYOR":         13, 
    "BILL_CONVEYOR_ACTIVE":     14,
    "BILL_CONVEYOR_DICE_READY": 15,
    # Camera
    "CAMERA_READY":     20,
    "CAMERA_DONE":      21,
    # Shared / Safety
    "FAULT":            29,
    "RESET":            30,
    "CYCLE_ACTIVE":     31,
}

# Holding Registers
# 0:    Target Number
# 1:    Pip Count
# 2:    DJ Number of Tries
# 3:    Bill Number of Tries

async def main():
    client = AsyncModbusTcpClient(MODBUS_IP, port=MODBUS_PORT)
    await client.connect()
    Pip_current = 0

    # initialize camera
    DevList = mvsdk.CameraEnumerateDevice()
    nDev = len(DevList)
    if nDev < 1:
        print("No camera was found!")
        return
    for i, DevInfo in enumerate(DevList):
        print("{}: {} {}".format(i, DevInfo.GetFriendlyName(),
        DevInfo.GetPortType()))
    i = 0 if nDev == 1 else int(input("Select camera: "))
    DevInfo = DevList[i]
    print(DevInfo)
    hCamera = 0
    try:
        hCamera = mvsdk.CameraInit(DevInfo, -1, -1)
    except mvsdk.CameraException as e:
        print("CameraInit Failed({}): {}".format(e.error_code, e.message) )
        exit()
    cap = mvsdk.CameraGetCapability(hCamera)
    monoCamera = (cap.sIspCapacity.bMonoSensor != 0)
    if monoCamera:
        mvsdk.CameraSetIspOutFormat(hCamera, mvsdk.CAMERA_MEDIA_TYPE_MONO8)
    else:
        mvsdk.CameraSetIspOutFormat(hCamera, mvsdk.CAMERA_MEDIA_TYPE_BGR8)
    mvsdk.CameraSetTriggerMode(hCamera, 0)
    mvsdk.CameraSetAeState(hCamera, 0)
    mvsdk.CameraSetExposureTime(hCamera, 50 * 1000)
    mvsdk.CameraPlay(hCamera)
    FrameBufferSize = cap.sResolutionRange.iWidthMax * cap.sResolutionRange.iHeightMax * (1 if monoCamera else 3)
    pFrameBuffer = mvsdk.CameraAlignMalloc(FrameBufferSize, 16)

    # Standby for pictures
    while(1):
        # Reset coils
        await write_coil(SIGNALS["CAMERA_DONE"], False)
        await write_coil(SIGNALS["CAMERA_READY"], True)

        # Check robot status
        DJ_ready = await read_coil(SIGNALS["DJ_READY_FOR_PICTURE"])
        Bill_ready = await read_coil(SIGNALS["BILL_READY_FOR_PICTURE"])

        if(DJ_ready or Bill_ready):
            # Take picture and analyze pips
            print("Capturing image and analyzing")
            await write_coil(SIGNALS["CAMERA_READY"], False)
            
            # dice analyzing
            try:
                pRawData, FrameHead = mvsdk.CameraGetImageBuffer(hCamera, 200)
                mvsdk.CameraImageProcess(hCamera, pRawData, pFrameBuffer, FrameHead)
                mvsdk.CameraReleaseImageBuffer(hCamera, pRawData)
                frame_data = (mvsdk.c_ubyte * FrameHead.uBytes).from_address(pFrameBuffer)
                frame = np.frombuffer(frame_data, dtype=np.uint8)
                frame = frame.reshape((FrameHead.iHeight, FrameHead.iWidth, 1 if FrameHead.uiMediaType == mvsdk.CAMERA_MEDIA_TYPE_MONO8 else 3) )
                frame = cv.resize(frame, (640,480), interpolation = cv.INTER_LINEAR)
                # DO STUFF
                Pip_current, image = countPips(frame)

                # Debugging
                cv.imshow('Display Window', image)
                cv.waitKey(0)
                cv.destroyAllWindows()
            except mvsdk.CameraException as e:
                if e.error_code != mvsdk.CAMERA_STATUS_TIME_OUT:
                    print("CameraGetImageBuffer failed({}):{}".format(e.error_code, e.message) )

            # inform and update pip count
            print(f"This dice has {Pip_current} pip(s).")
            registers = await read_registers(client)
            await write_registers(  client,
                                    registers[0],
                                    Pip_current,
                                    registers[2],
                                    registers[3]
                                )
            await write_coil(SIGNALS["CAMERA_DONE"], True)
            Pip_current = 0
            sleep(5)
        else:
            print("Waiting for dice...")
            sleep(5)

def countPips(image):
    pipcount = 0

    # # Crop to bottom-left quadrant to reduce noise
    # image = image[240:480, 0:320]

    # Isolate yellow die face
    hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)
    yellow_mask = cv.inRange(hsv, (15, 80, 80), (35, 255, 255))
    yellow_mask = cv.dilate(yellow_mask, None, iterations=2)
    yellow_mask = cv.erode(yellow_mask, None, iterations=2)

    # Find die contour from yellow mask
    contours, _ = cv.findContours(yellow_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    if not contours:
        return 0, image

    die_contour = max(contours, key=cv.contourArea)
    x, y, w, h = cv.boundingRect(die_contour)
    crop = image[y:y+h, x:x+w]

    # --- Approach 1: Contour + circularity filter ---
    # gray = cv.cvtColor(crop, cv.COLOR_BGR2GRAY)
    # _, pip_mask = cv.threshold(gray, 50, 255, cv.THRESH_BINARY_INV)
    # pip_mask = cv.erode(pip_mask, None, iterations=1)
    # pip_mask = cv.dilate(pip_mask, None, iterations=1)
    # pip_contours, _ = cv.findContours(pip_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    # for pip in pip_contours:
    #     area = cv.contourArea(pip)
    #     if not (25 < area < 500):
    #         continue
    #     perimeter = cv.arcLength(pip, True)
    #     if perimeter == 0:
    #         continue
    #     circularity = 4 * np.pi * area / (perimeter ** 2)
    #     if circularity > 0.5:
    #         px, py, pw, ph = cv.boundingRect(pip)
    #         cv.rectangle(crop, (px, py), (px + pw, py + ph), (0, 255, 0), 1)
    #         pipcount += 1

    # --- Approach 2: Hough circle detection ---
    # Tuning knobs: param2 (accumulator threshold — lower += more circles, higher = fewer)                               
    #               minRadius/maxRadius — adjust to match +pip size in frame   
    gray = cv.cvtColor(crop, cv.COLOR_BGR2GRAY)
    gray = cv.GaussianBlur(gray, (5, 5), 0)
    circles = cv.HoughCircles(gray, cv.HOUGH_GRADIENT, dp=1, minDist=10,
                                param1=50, param2=15, minRadius=3, maxRadius=20)
    if circles is not None:
        for (cx, cy, r) in np.round(circles[0]).astype(int):
            cv.rectangle(die_contour, (cx - r, cy - r), (cx + r, cy + r), (0, 255, 0), 1)
            pipcount += 1

    return pipcount, image

async def write_coil(address, value):
    client = AsyncModbusTcpClient(MODBUS_IP, port=MODBUS_PORT)
    await client.connect()
    result = await client.write_coil(address, value)
    if result.isError():
        print(f"Error writing coil {address}: {result}")
    else:
        print(f"Set coil ({address}) = {value}")
    client.close()

async def read_coil(address):
    client = AsyncModbusTcpClient(MODBUS_IP, port=MODBUS_PORT)
    await client.connect()
    result = await client.read_coils(address, count=1)
    client.close()
    if result.isError():
        print(f"Error reading coil {address}: {result}")
        return None
    return result.bits[0]

# async def read_pose_registers(client):
#     pose_result = await client.read_holding_registers(address=0, count=3, device_id=1)
#     if not pose_result.isError():
#         pose_regs = pose_result.registers
#         print(f"Read Pose Registers: X={pose_regs[0]}, Y={pose_regs[1]}, Z={pose_regs[2]}")
#         positions[2][0] = float(pose_regs[0])
#         positions[2][1] = float(pose_regs[1])
#         positions[2][2] = float(pose_regs[2])
#         return pose_regs
#     else:
#         print("Error reading pose registers!")
#         return None

async def write_signal(client, signal_name, value):
    """Write a single signal/coil to the server."""
    address = SIGNALS[signal_name]
    result = await client.write_coil(address, value, device_id=1)
    if result.isError():
        print(f"Error writing {signal_name}: {result}")
    else:
        print(f"Wrote {signal_name} = {value}")


async def write_registers(client, target_number, pip_count, dj_tries, bill_tries):
    """Write values to holding registers."""
    values = [target_number, pip_count, dj_tries, bill_tries]
    result = await client.write_registers(
        address=0, values=values, device_id=1
    )
    if result.isError():
        print(f"Error writing registers: {result}")
    else:
        print(f"Wrote TARGET_NUMBER={target_number}, PIP_COUNT={pip_count}, "
              f"DJ_NUMBER_OF_TRIES={dj_tries}, BILL_NUMBER_OF_TRIES={bill_tries}")


async def read_registers(client):
    """Read holding registers."""
    result = await client.read_holding_registers(
        address=0, count=4, device_id=1
    )
    if not result.isError():
        regs = result.registers
        print(f"Read Registers - Target: {regs[0]}, Pips: {regs[1]}, "
              f"DJ Tries: {regs[2]}, BILL Tries: {regs[3]}")
        return regs
    else:
        print("Error reading registers!")
        return None
    
# await write_registers(client, 6, 3, 1, 2)
# await write_signal(client, "DJ_HAS_DICE", True)
# await write_signal(client, "BILL_GRIPPER_CLOSED", True)
# await write_signal(client, "CYCLE_ACTIVE", True)
# await write_signal(client, "CAMERA_READY", True)

if __name__ == '__main__':
    asyncio.run(main())