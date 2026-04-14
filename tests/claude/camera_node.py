import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

import mvsdk


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.srv = self.create_service(Trigger, 'take_picture', self.take_picture_callback)
        self.get_logger().info('Camera node ready, service: take_picture')

    def take_picture_callback(self, request, response):
        try:
            DevList = mvsdk.CameraEnumerateDevice()
            if len(DevList) < 1:
                response.success = False
                response.message = 'No camera found'
                return response

            hCamera = mvsdk.CameraInit(DevList[0], -1, -1)
            cap = mvsdk.CameraGetCapability(hCamera)

            monoCamera = (cap.sIspCapacity.bMonoSensor != 0)
            if monoCamera:
                mvsdk.CameraSetIspOutFormat(hCamera, mvsdk.CAMERA_MEDIA_TYPE_MONO8)

            mvsdk.CameraSetTriggerMode(hCamera, 0)
            mvsdk.CameraSetAeState(hCamera, 0)
            mvsdk.CameraSetExposureTime(hCamera, 30 * 1000)
            mvsdk.CameraPlay(hCamera)

            FrameBufferSize = (
                cap.sResolutionRange.iWidthMax
                * cap.sResolutionRange.iHeightMax
                * (1 if monoCamera else 3)
            )
            pFrameBuffer = mvsdk.CameraAlignMalloc(FrameBufferSize, 16)

            pRawData, FrameHead = mvsdk.CameraGetImageBuffer(hCamera, 2000)
            mvsdk.CameraImageProcess(hCamera, pRawData, pFrameBuffer, FrameHead)
            mvsdk.CameraReleaseImageBuffer(hCamera, pRawData)

            status = mvsdk.CameraSaveImage(
                hCamera, './grab.bmp', pFrameBuffer, FrameHead, mvsdk.FILE_BMP, 100
            )

            mvsdk.CameraUnInit(hCamera)
            mvsdk.CameraAlignFree(pFrameBuffer)

            if status == mvsdk.CAMERA_STATUS_SUCCESS:
                response.success = True
                response.message = f'Image saved: {FrameHead.iWidth}x{FrameHead.iHeight}'
            else:
                response.success = False
                response.message = f'Save failed, err={status}'

        except mvsdk.CameraException as e:
            response.success = False
            response.message = f'Camera error ({e.error_code}): {e.message}'

        return response


if __name__ == '__main__':
    rclpy.init()
    node = CameraNode()
    rclpy.spin(node)
    rclpy.shutdown()
