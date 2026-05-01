import asyncio
import cv2
import numpy as np
import mvsdk

class Camera:
    def __init__(self):
        DevList = mvsdk.CameraEnumerateDevice()
        nDev = len(DevList)
        if nDev < 1:
            print("No camera was found!")
            return

        for i, DevInfo in enumerate(DevList):
            print("{}: {} {}".format(i, DevInfo.GetFriendlyName(), DevInfo.GetPortType()))
        i = 0 #if nDev == 1 else int(input("Select camera: "))
        DevInfo = DevList[i]
        print(DevInfo)

        hCamera = 0
        try:
            hCamera = mvsdk.CameraInit(DevInfo, -1, -1)
        except mvsdk.CameraException as e:
            print("CameraInit Failed({}): {}".format(e.error_code, e.message))
            return

        cap = mvsdk.CameraGetCapability(hCamera)
        monoCamera = (cap.sIspCapacity.bMonoSensor != 0)

        if monoCamera:
            mvsdk.CameraSetIspOutFormat(hCamera, mvsdk.CAMERA_MEDIA_TYPE_MONO8)
        else:
            mvsdk.CameraSetIspOutFormat(hCamera, mvsdk.CAMERA_MEDIA_TYPE_BGR8)

        mvsdk.CameraSetTriggerMode(hCamera, 1)
        mvsdk.CameraSetAeState(hCamera, 0)
        mvsdk.CameraSetExposureTime(hCamera, 30 * 1000)
        mvsdk.CameraPlay(hCamera)

        FrameBufferSize = cap.sResolutionRange.iWidthMax * cap.sResolutionRange.iHeightMax * (1 if monoCamera else 3)
        pFrameBuffer = mvsdk.CameraAlignMalloc(FrameBufferSize, 16)

        self.hCamera = hCamera
        self.pFrameBuffer = pFrameBuffer
        self.monoCamera = monoCamera

    def getFrame(self):
        """Synchronous frame grab - use getFrameAsync in async contexts."""
        mvsdk.CameraSoftTrigger(self.hCamera)
        # Increased timeout to 2000ms to be safe on GigE
        pRawData, FrameHead = mvsdk.CameraGetImageBuffer(self.hCamera, 2000)
        mvsdk.CameraImageProcess(self.hCamera, pRawData, self.pFrameBuffer, FrameHead)
        mvsdk.CameraReleaseImageBuffer(self.hCamera, pRawData)

        frame_data = (mvsdk.c_ubyte * FrameHead.uBytes).from_address(self.pFrameBuffer)
        frame = np.frombuffer(frame_data, dtype=np.uint8)
        frame = frame.reshape((FrameHead.iHeight, FrameHead.iWidth,
                                1 if FrameHead.uiMediaType == mvsdk.CAMERA_MEDIA_TYPE_MONO8 else 3))
        return frame.copy()  # copy so buffer can be reused safely

    async def getFrameAsync(self, retries=10, delay=0.1):
        """Non-blocking frame grab with retry logic."""
        loop = asyncio.get_event_loop()
        last_error = None

        for attempt in range(retries):
                try:
                        frame = await loop.run_in_executor(None, self.getFrame)
                        return frame
                except mvsdk.CameraException as e:
                        last_error = e
                        print(f"Frame grab failed (attempt {attempt + 1}/{retries}): {e}")
                        await asyncio.sleep(delay)

        raise mvsdk.CameraException(last_error.error_code)

    def disable(self):
        mvsdk.CameraUnInit(self.hCamera)
        mvsdk.CameraAlignFree(self.pFrameBuffer)