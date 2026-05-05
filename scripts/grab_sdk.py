#!/usr/bin/env python3
"""
grab_sdk.py
-----------
Grabs a single frame directly from the MindVision camera using the SDK
(no ROS2 required) and saves it to /tmp/grab.bmp.

NOTE: The camera can only be opened by one process at a time.
      Stop mv_camera_node before running this (i.e. don't use during just launch).

Run with: just grab-sdk
"""

import sys
import subprocess
import numpy as np
import cv2
import mvsdk

SAVE_PATH = '/tmp/grab.bmp'


def _pick_routable_device(dev_list):
    """When the host has multiple NICs, the SDK enumerates the camera once per route.
    PortType is '<camera_ip>-<host_ip>'. Prefer the entry whose host_ip shares a /16
    with the camera_ip (i.e. the path that actually carries data). Fall back to [0]."""
    for d in dev_list:
        port = d.GetPortType()
        if '-' not in port:
            continue
        cam_ip, host_ip = port.split('-', 1)
        cam_oct  = cam_ip.split('.')
        host_oct = host_ip.split('.')
        if len(cam_oct) >= 2 and len(host_oct) >= 2 and cam_oct[:2] == host_oct[:2]:
            return d
    return dev_list[0]


def main():
    dev_list = mvsdk.CameraEnumerateDevice()
    if len(dev_list) < 1:
        print('ERROR: No camera found. Check GigE connection and adapter IP.')
        sys.exit(1)

    dev_info = _pick_routable_device(dev_list)
    print(f'Opening camera: {dev_info.GetFriendlyName()} via {dev_info.GetPortType()}')

    try:
        hCamera = mvsdk.CameraInit(dev_info, -1, -1)
    except mvsdk.CameraException as e:
        print(f'ERROR: CameraInit failed ({e.error_code}): {e.message}')
        sys.exit(1)

    try:
        cap = mvsdk.CameraGetCapability(hCamera)
        mono = cap.sIspCapacity.bMonoSensor != 0

        if mono:
            mvsdk.CameraSetIspOutFormat(hCamera, mvsdk.CAMERA_MEDIA_TYPE_MONO8)
        else:
            mvsdk.CameraSetIspOutFormat(hCamera, mvsdk.CAMERA_MEDIA_TYPE_BGR8)

        mvsdk.CameraSetTriggerMode(hCamera, 0)       # continuous
        mvsdk.CameraSetAeState(hCamera, 0)            # manual exposure
        mvsdk.CameraSetExposureTime(hCamera, 30 * 1000)  # 30 ms

        mvsdk.CameraPlay(hCamera)

        buf_size = cap.sResolutionRange.iWidthMax * cap.sResolutionRange.iHeightMax * (1 if mono else 3)
        pFrameBuffer = mvsdk.CameraAlignMalloc(buf_size, 16)

        try:
            pRawData, FrameHead = mvsdk.CameraGetImageBuffer(hCamera, 2000)
            mvsdk.CameraImageProcess(hCamera, pRawData, pFrameBuffer, FrameHead)
            mvsdk.CameraReleaseImageBuffer(hCamera, pRawData)

            frame_data = (mvsdk.c_ubyte * FrameHead.uBytes).from_address(pFrameBuffer)
            frame = np.frombuffer(frame_data, dtype=np.uint8)

            if mono:
                frame = frame.reshape((FrameHead.iHeight, FrameHead.iWidth))
                frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
            else:
                frame = frame.reshape((FrameHead.iHeight, FrameHead.iWidth, 3))

            cv2.imwrite(SAVE_PATH, frame)
            print(f'Saved: {SAVE_PATH}')
            subprocess.Popen(['xdg-open', SAVE_PATH])

        except mvsdk.CameraException as e:
            print(f'ERROR: CameraGetImageBuffer failed ({e.error_code}): {e.message}')
            sys.exit(1)

        finally:
            mvsdk.CameraAlignFree(pFrameBuffer)

    finally:
        mvsdk.CameraStop(hCamera)
        mvsdk.CameraUnInit(hCamera)


if __name__ == '__main__':
    main()
