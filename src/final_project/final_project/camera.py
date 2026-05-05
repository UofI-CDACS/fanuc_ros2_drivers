import cv2
import numpy as np
import mvsdk


class Camera:
    def __init__(self, camera_ip=None):
        self.hCamera = None
        self.pFrameBuffer = None

        DevList = mvsdk.CameraEnumerateDevice()
        nDev = len(DevList)
        if nDev < 1:
            print('No camera was found!')
            return

        if camera_ip is not None:
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
                print(f'No camera found with IP {camera_ip}')
                return
        elif nDev == 1:
            DevInfo = DevList[0]
        else:
            for i, dev in enumerate(DevList):
                print('{}: {} {}'.format(i, dev.GetFriendlyName(), dev.GetPortType()))
            i = int(input('Select camera: '))
            DevInfo = DevList[i]

        hCamera = 0
        try:
            hCamera = mvsdk.CameraInit(DevInfo, -1, -1)
        except mvsdk.CameraException as e:
            print('CameraInit Failed({}): {}'.format(e.error_code, e.message))
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

        FrameBufferSize = (cap.sResolutionRange.iWidthMax
                          * cap.sResolutionRange.iHeightMax
                          * (1 if monoCamera else 3))
        pFrameBuffer = mvsdk.CameraAlignMalloc(FrameBufferSize, 16)

        self.hCamera = hCamera
        self.pFrameBuffer = pFrameBuffer

    def getFrame(self):
        if self.hCamera is None:
            raise RuntimeError('Camera not initialised.')
        pRawData, FrameHead = mvsdk.CameraGetImageBuffer(self.hCamera, 200)
        mvsdk.CameraImageProcess(self.hCamera, pRawData, self.pFrameBuffer, FrameHead)
        mvsdk.CameraReleaseImageBuffer(self.hCamera, pRawData)
        frame_data = (mvsdk.c_ubyte * FrameHead.uBytes).from_address(self.pFrameBuffer)
        frame = np.frombuffer(frame_data, dtype=np.uint8)
        frame = frame.reshape((
            FrameHead.iHeight, FrameHead.iWidth,
            1 if FrameHead.uiMediaType == mvsdk.CAMERA_MEDIA_TYPE_MONO8 else 3
        ))
        return frame

    def disable(self):
        if self.hCamera is None:
            return
        mvsdk.CameraUnInit(self.hCamera)
        mvsdk.CameraAlignFree(self.pFrameBuffer)
