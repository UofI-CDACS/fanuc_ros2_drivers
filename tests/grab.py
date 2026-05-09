#coding=utf-8
import mvsdk

def main():
	# Enumerate cameras
	DevList = mvsdk.CameraEnumerateDevice()
	nDev = len(DevList)
	if nDev < 1:
		print("No camera was found!")
		return

	for i, DevInfo in enumerate(DevList):
		print("{}: {} {}".format(i, DevInfo.GetFriendlyName(), DevInfo.GetPortType()))
	#i = 0 if nDev == 1 else int(input("Select camera: "))
	i = 0
	DevInfo = DevList[i]
	print(DevInfo)

	# Open camera
	hCamera = 0
	try:
		hCamera = mvsdk.CameraInit(DevInfo, -1, -1)
	except mvsdk.CameraException as e:
		print("CameraInit Failed({}): {}".format(e.error_code, e.message) )
		return

	# Get camera capability description
	cap = mvsdk.CameraGetCapability(hCamera)
	PrintCapbility(cap)

	# Determine if this is a mono or color camera
	monoCamera = (cap.sIspCapacity.bMonoSensor != 0)

	# For mono cameras, have the ISP output MONO data directly instead of expanding to 24-bit R=G=B grayscale
	if monoCamera:
		mvsdk.CameraSetIspOutFormat(hCamera, mvsdk.CAMERA_MEDIA_TYPE_MONO8)

	# Set camera to continuous capture mode
	mvsdk.CameraSetTriggerMode(hCamera, 0)

	# Manual exposure, exposure time 30ms
	mvsdk.CameraSetAeState(hCamera, 0)
	mvsdk.CameraSetExposureTime(hCamera, 30 * 1000)

	# Start the SDK's internal capture thread
	mvsdk.CameraPlay(hCamera)

	# Calculate the required RGB buffer size, allocated here based on the camera's maximum resolution
	FrameBufferSize = cap.sResolutionRange.iWidthMax * cap.sResolutionRange.iHeightMax * (1 if monoCamera else 3)

	# Allocate RGB buffer to hold the ISP output image
	# Note: data transmitted from the camera to the PC is RAW; the software ISP on the PC converts it to RGB
	# (mono cameras don't need format conversion, but the ISP still does other processing, so the buffer is still needed)
	pFrameBuffer = mvsdk.CameraAlignMalloc(FrameBufferSize, 16)

	# Grab one frame from the camera
	try:
		pRawData, FrameHead = mvsdk.CameraGetImageBuffer(hCamera, 2000)
		mvsdk.CameraImageProcess(hCamera, pRawData, pFrameBuffer, FrameHead)
		mvsdk.CameraReleaseImageBuffer(hCamera, pRawData)

		# The image is now stored in pFrameBuffer: RGB data for color cameras, 8-bit grayscale for mono cameras
		# In this example we simply save the image to disk
		status = mvsdk.CameraSaveImage(hCamera, "./grab.bmp", pFrameBuffer, FrameHead, mvsdk.FILE_BMP, 100)
		if status == mvsdk.CAMERA_STATUS_SUCCESS:
			print("Save image successfully. image_size = {}X{}".format(FrameHead.iWidth, FrameHead.iHeight) )
		else:
			print("Save image failed. err={}".format(status) )
	except mvsdk.CameraException as e:
		print("CameraGetImageBuffer failed({}): {}".format(e.error_code, e.message) )

	# Close camera
	mvsdk.CameraUnInit(hCamera)

	# Free frame buffer
	mvsdk.CameraAlignFree(pFrameBuffer)

def PrintCapbility(cap):
	for i in range(cap.iTriggerDesc):
		desc = cap.pTriggerDesc[i]
		print("{}: {}".format(desc.iIndex, desc.GetDescription()) )
	for i in range(cap.iImageSizeDesc):
		desc = cap.pImageSizeDesc[i]
		print("{}: {}".format(desc.iIndex, desc.GetDescription()) )
	for i in range(cap.iClrTempDesc):
		desc = cap.pClrTempDesc[i]
		print("{}: {}".format(desc.iIndex, desc.GetDescription()) )
	for i in range(cap.iMediaTypeDesc):
		desc = cap.pMediaTypeDesc[i]
		print("{}: {}".format(desc.iIndex, desc.GetDescription()) )
	for i in range(cap.iFrameSpeedDesc):
		desc = cap.pFrameSpeedDesc[i]
		print("{}: {}".format(desc.iIndex, desc.GetDescription()) )
	for i in range(cap.iPackLenDesc):
		desc = cap.pPackLenDesc[i]
		print("{}: {}".format(desc.iIndex, desc.GetDescription()) )
	for i in range(cap.iPresetLut):
		desc = cap.pPresetLutDesc[i]
		print("{}: {}".format(desc.iIndex, desc.GetDescription()) )
	for i in range(cap.iAeAlmSwDesc):
		desc = cap.pAeAlmSwDesc[i]
		print("{}: {}".format(desc.iIndex, desc.GetDescription()) )
	for i in range(cap.iAeAlmHdDesc):
		desc = cap.pAeAlmHdDesc[i]
		print("{}: {}".format(desc.iIndex, desc.GetDescription()) )
	for i in range(cap.iBayerDecAlmSwDesc):
		desc = cap.pBayerDecAlmSwDesc[i]
		print("{}: {}".format(desc.iIndex, desc.GetDescription()) )
	for i in range(cap.iBayerDecAlmHdDesc):
		desc = cap.pBayerDecAlmHdDesc[i]
		print("{}: {}".format(desc.iIndex, desc.GetDescription()) )

main()
