# import mvsdk                                                                                                                                   
# devs = mvsdk.CameraEnumerateDevice()                                                                                                           
# for i, d in enumerate(devs):                                                                                                                   
#    print(f'Trying device [{i}]: {d.GetPortType()}')                                                                                           
#    try:                                                                                                                                       
#          h = mvsdk.CameraInit(d, -1, -1)                                                                                                        
#          print(f'  SUCCESS, handle={h}')                                                                                                        
#          mvsdk.CameraUnInit(h)                                                                                                                  
#    except mvsdk.CameraException as e:
#          print(f'  FAILED({e.error_code}): {e.message}')                                                                                        


import mvsdk                                                                                                                                
devs = mvsdk.CameraEnumerateDevice()                                                                                                        
print(f'Found {len(devs)} device(s)')                                                                                                       
for i, d in enumerate(devs):                                                                                                                
      print(f'  [{i}] PortType={d.GetPortType()}  FriendlyName={d.GetFriendlyName()}')