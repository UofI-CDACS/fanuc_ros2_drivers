"""
find_camera.py — Scan for all MindVision cameras (USB and GigE) and print their details.
Run with: python3 find_camera.py
"""
import sys
sys.path.insert(0, '/home/colin/Desktop/fanuc_ros2_drivers')
import mvsdk

DevList = mvsdk.CameraEnumerateDevice()
print(f"Found {len(DevList)} camera(s).\n")

for i, dev in enumerate(DevList):
    name      = dev.GetFriendlyName()
    port_type = dev.GetPortType()
    sn        = dev.GetSN() if hasattr(dev, 'GetSN') else 'N/A'

    print(f"[{i}] {name}  ({port_type})  SN: {sn}")

    # Try to get GigE IP (only works for network cameras)
    try:
        cam_ip, cam_mask, cam_gw, et_ip, et_mask, et_gw = mvsdk.CameraGigeGetIp(dev)
        print(f"     Camera IP   : {cam_ip}")
        print(f"     Camera Mask : {cam_mask}")
        print(f"     Camera GW   : {cam_gw}")
        print(f"     Host NIC IP : {et_ip}")
    except Exception as e:
        print(f"     (GigE IP lookup failed: {e})")

    print()

if not DevList:
    print("No cameras detected. Check that:")
    print("  - The camera is powered on and connected")
    print("  - For GigE cameras: the camera and PC are on the same network/subnet")
    print("  - udev rules are installed (88-mvusb.rules / 99-mvusb.rules)")
