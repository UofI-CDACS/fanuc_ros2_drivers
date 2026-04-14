## lily Mason
# USAGE: Import this class                              from camera import Camera
# Create camera object                                  camera = Camera()
# If camera is detectable on the network it should automatically connect.
# Get a frame from the camera                           frame = camera.getFrame()
# When finished, disable the camera                     camera.disable()
import cv2
import numpy as np
import mvsdk

class Camera:
    def __init__(self, camera_ip=None):
        self.hCamera = None
        self.pFrameBuffer = None

        # Enumerate cameras on USB and network
        DevList = mvsdk.CameraEnumerateDevice()
        nDev = len(DevList)
        if nDev < 1:
                print("No camera was found!")
                return

        if camera_ip is not None:
            # Filter by IP address
            DevInfo = None
            for dev in DevList:
                try:
                    cam_ip, _, _, _, _, _ = mvsdk.CameraGigeGetIp(dev)
                    if cam_ip == camera_ip:
                        DevInfo = dev
                        break
                except Exception:
                    pass
            if DevInfo is None:
                print(f"No camera found with IP {camera_ip}")
                return
        elif nDev == 1:
            DevInfo = DevList[0]
        else:
            for i, dev in enumerate(DevList):
                print("{}: {} {}".format(i, dev.GetFriendlyName(), dev.GetPortType()))
            i = int(input("Select camera: "))
            DevInfo = DevList[i]
        print(DevInfo)

        # Open Camera
        hCamera = 0
        try:
                hCamera = mvsdk.CameraInit(DevInfo, -1, -1)
        except mvsdk.CameraException as e:
                print("CameraInit Failed({}): {}".format(e.error_code, e.message) )
                return

        # Obtain Camera Feature Description
        cap = mvsdk.CameraGetCapability(hCamera)

        # What ColorSpace is the camera - BW or Color
        monoCamera = (cap.sIspCapacity.bMonoSensor != 0)

        # Monochrome cameras allow the ISP to directly output MONO data, instead of expanding it into 24-bit grayscale (R=G=B).
        if monoCamera:
                mvsdk.CameraSetIspOutFormat(hCamera, mvsdk.CAMERA_MEDIA_TYPE_MONO8)
        else:
                mvsdk.CameraSetIspOutFormat(hCamera, mvsdk.CAMERA_MEDIA_TYPE_BGR8)

        # Switch Camera Mode to video stream
        mvsdk.CameraSetTriggerMode(hCamera, 0)

        # Manual Exposure 30ms
        mvsdk.CameraSetAeState(hCamera, 0)
        mvsdk.CameraSetExposureTime(hCamera, 30 * 1000)

        # Start image feed
        mvsdk.CameraPlay(hCamera)

        # Calculate size of RGB buffer - based on camera max resolution
        FrameBufferSize = cap.sResolutionRange.iWidthMax * cap.sResolutionRange.iHeightMax * (1 if monoCamera else 3)

        # Allocate RGB bufer to store image
        # Note: The data transferred from the camera to the PC is RAW data, which is then converted to RGB data by the ISP software on the PC (if it is a monochrome camera, there is no need to convert the format, but the ISP has other processing, so this buffer also needs to be allocated).
        pFrameBuffer = mvsdk.CameraAlignMalloc(FrameBufferSize, 16)

        self.hCamera = hCamera
        self.pFrameBuffer = pFrameBuffer

    def getFrame(self):
        if self.hCamera is None:
            raise RuntimeError("Camera not initialised — no camera was found at startup.")
        pRawData, FrameHead = mvsdk.CameraGetImageBuffer(self.hCamera, 200)
        mvsdk.CameraImageProcess(self.hCamera, pRawData, self.pFrameBuffer, FrameHead)
        mvsdk.CameraReleaseImageBuffer(self.hCamera, pRawData)

        #At this point, the image is already stored in pFrameBuffer. For a color camera, pFrameBuffer = RGB data; for a monochrome camera, pFrameBuffer = 8-bit grayscale data.
        #Convert pFrameBuffer to OpenCV image format for subsequent algorithm processing.
        frame_data = (mvsdk.c_ubyte * FrameHead.uBytes).from_address(self.pFrameBuffer)
        frame = np.frombuffer(frame_data, dtype=np.uint8)
        frame = frame.reshape((FrameHead.iHeight, FrameHead.iWidth, 1 if FrameHead.uiMediaType == mvsdk.CAMERA_MEDIA_TYPE_MONO8 else 3) )
        return frame

    def disable(self):
        if self.hCamera is None:
            return
        # Turn off camera
        mvsdk.CameraUnInit(self.hCamera)
        # Release framebuffer
        mvsdk.CameraAlignFree(self.pFrameBuffer)
