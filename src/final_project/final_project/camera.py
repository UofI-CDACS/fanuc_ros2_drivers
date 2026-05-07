import cv2
import numpy as np
import threading
import mvsdk

_INIT_TIMEOUT = 5.0  # seconds before giving up on CameraInit


def _camera_init_with_timeout(DevInfo):
    """Run CameraInit in a thread; return hCamera or raise on timeout/error."""
    result = [None]
    exc    = [None]

    def _run():
        try:
            result[0] = mvsdk.CameraInit(DevInfo, -1, -1)
        except Exception as e:
            exc[0] = e

    t = threading.Thread(target=_run, daemon=True)
    t.start()
    t.join(_INIT_TIMEOUT)
    if t.is_alive():
        raise TimeoutError(f'CameraInit timed out after {_INIT_TIMEOUT}s '
                           f'(camera likely on wrong network)')
    if exc[0] is not None:
        raise exc[0]
    return result[0]


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
                    found_ip, _, _, _, _, _ = mvsdk.CameraGigeGetIp(dev)
                    if found_ip == camera_ip:
                        DevInfo = dev
                        break
                except Exception:
                    pass
            if DevInfo is None:
                # IP didn't match — fall back to first found device
                try:
                    actual_ip, _, _, _, _, _ = mvsdk.CameraGigeGetIp(DevList[0])
                except Exception:
                    actual_ip = '?'
                print(f'Camera not found at {camera_ip}, '
                      f'using first available device (IP={actual_ip})')
                DevInfo = DevList[0]
        elif nDev == 1:
            DevInfo = DevList[0]
        else:
            for i, dev in enumerate(DevList):
                print('{}: {} {}'.format(i, dev.GetFriendlyName(), dev.GetPortType()))
            i = int(input('Select camera: '))
            DevInfo = DevList[i]

        try:
            hCamera = _camera_init_with_timeout(DevInfo)
        except TimeoutError as e:
            print(f'CameraInit timed out: {e}')
            return
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
        mvsdk.CameraSoftTrigger(self.hCamera)
        pRawData, FrameHead = mvsdk.CameraGetImageBuffer(self.hCamera, 2000)
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
