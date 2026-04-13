#############
#
# FANUC Ethernet/IP Driver
# Version 1.1
# 10/6/2023
# Shovic, et.al.
# Center for Intelligent Industrial Robotics
# University of Idaho
#
############
# Requires Ethernet/IP FANUC driver and 30 Series Controller
# 

DEBUG = False

#import sys
#sys.path.append('./pycomm3/pycomm3')
from enum import Enum
from pycomm3 import CIPDriver
from pycomm3 import Services
from pycomm3 import DataTypes, DataType
from pycomm3.logger import configure_default_logger, LOG_VERBOSE
import pycomm3
import struct
import random
import time
import math

# Helper functions

def returnBit(bitNumber,list):
      byteCount= int(bitNumber/8)
      bit=list[byteCount]>>(bitNumber%8-1)
      return bit



#################
# Cartiseian Functions
#################

# Read Current Position Registers Cart 
# returns list [UTOOL, UFRAME, X, Y, Z, W, P, R, Turn1, Turn2, Bitflip, EXT_0, EXT_1, EXT_2, bytearray]
# bytearray contains full return value

def returnCartesianCurrentPostion(drive_path):

# read all Position Registers Cartesian Outputs class 0x7D, I-0x1, A-0x01
  with CIPDriver(drive_path) as drive:
        myPRTag = drive.generic_message(
            service=Services.get_attribute_single,
            class_code=0x7D,
            instance=0x01,
            attribute=0x01,
            data_type=None,
            connected=False,
            unconnected_send=False,
            route_path=True,
            name='fanucCURPOSread'
        )
        if (DEBUG == True):
          print("CURPOS Current Cartesian Coordinates 0x7D, IA< 0x01")
          print(myPRTag)
          print("myPRTag.type=", myPRTag.type)
        myList = list(myPRTag.value)
        if (DEBUG == True):
          print("myList=", myList)

  UTOOL = myList[1]*8+myList[0]
  UFRAME = myList[3]*8+myList[2]
  X=struct.unpack('f', bytes(myList[4:8]))
  Y=struct.unpack('f', bytes(myList[8:12]))
  Z=struct.unpack('f', bytes(myList[12:16]))
  W=struct.unpack('f', bytes(myList[16:20]))
  P=struct.unpack('f', bytes(myList[20:24]))
  R=struct.unpack('f', bytes(myList[24:28]))
  turn1=struct.unpack('B', bytes(myList[28:29]))
  turn2=struct.unpack('B', bytes(myList[29:30]))
  turn3=struct.unpack('B', bytes(myList[30:31]))
  bitflip=struct.unpack('B', bytes(myList[31:32]))
  E0=struct.unpack('f', bytes(myList[32:36]))
  E1=struct.unpack('f', bytes(myList[36:40]))
  E2=struct.unpack('f', bytes(myList[40:44]))

  if (DEBUG == True):
    print("UTOOL=", UTOOL)
    print("UFRAME=", UFRAME)
    print("X=", X)
    print("Y=", Y)
    print("Z=", Z)
    print("W=", W)
    print("P=", P)
    print("R=", R)
    print("turn1=", turn1)
    print("turn2=", turn2)
    print("turn3=", turn3)
    print("bitflip=0x", hex(bitflip[0]))
    print("EXT[0]=", E0)
    print("EXT[1]=", E1)
    print("EXT[2]=", E2)
  
  returnList = []  
  returnList.append(UTOOL)
  returnList.append(UFRAME)
  returnList.append(X[0])
  returnList.append(Y[0])
  returnList.append(Z[0])
  returnList.append(W[0])
  returnList.append(P[0])
  returnList.append(R[0])
  returnList.append(turn1[0])
  returnList.append(turn2[0])
  returnList.append(turn3[0])
  returnList.append(bitflip[0])
  returnList.append(E0[0])
  returnList.append(E1[0])
  returnList.append(E2[0])
  returnList.append(myList) 

  return returnList

# Read Position Registers Cartesian  (PR[])
# argument PRNumber is the PR register being written to inside the robot - copied to Postition Register by TP Program
# to the position register
# returns list [UTOOL, UFRAME, X, Y, Z, W, P, R, Turn1, Turn2, Bitflip, EXT_0, EXT_1, EXT_2]

def readCartesianPositionRegister(drive_path, PRNumber):


   with CIPDriver(drive_path) as drive:
        myTag = drive.generic_message(
            service=Services.get_attribute_single,
            class_code=0x7B,
            instance=0x01,
            attribute=PRNumber,
            data_type=None,
            connected=False,
            unconnected_send=False,
            route_path=False,
            name='fanucPRSread'
        )
        if (DEBUG == True):
          print("Read PR registers Cartesian Coordinates 0x7B ")
          print(myTag)
          print('myTag.error=', myTag.error)
          print("myTag.type=", myTag.type)

        myList = list(myTag.value)
        UTOOL = myList[1]*8+myList[0]
        UFRAME = myList[3]*8+myList[2]
        X=struct.unpack('f', bytes(myList[4:8]))
        Y=struct.unpack('f', bytes(myList[8:12]))
        Z=struct.unpack('f', bytes(myList[12:16]))
        W=struct.unpack('f', bytes(myList[16:20]))
        P=struct.unpack('f', bytes(myList[20:24]))
        R=struct.unpack('f', bytes(myList[24:28]))
        turn1=struct.unpack('B', bytes(myList[28:29]))
        turn2=struct.unpack('B', bytes(myList[29:30]))
        turn3=struct.unpack('B', bytes(myList[30:31]))
        bitflip=struct.unpack('B', bytes(myList[31:32]))
        E0=struct.unpack('f', bytes(myList[32:36]))
        E1=struct.unpack('f', bytes(myList[36:40]))
        E2=struct.unpack('f', bytes(myList[40:44]))
        if (DEBUG == True):
          print("myList=", myList)

          print("UTOOL=", UTOOL)
          print("UFRAME=", UFRAME)
          print("X=", X)
          print("Y=", Y)
          print("Z=", Z)
          print("W=", W)
          print("P=", P)
          print("R=", R)
          print("turn1=", turn1)
          print("turn2=", turn2)
          print("turn3=", turn3)
          print("bitflip=0x", hex(bitflip[0]))
          print("EXT[0]=", E0)
          print("EXT[1]=", E1)
          print("EXT[2]=", E2)
      
        returnList = []  
        returnList.append(UTOOL)
        returnList.append(UFRAME)
        returnList.append(X)
        returnList.append(Y)
        returnList.append(Z)
        returnList.append(W)
        returnList.append(P)
        returnList.append(R)
        returnList.append(turn1)
        returnList.append(turn2)
        returnList.append(turn3)
        returnList.append(bitflip)
        returnList.append(E0)
        returnList.append(E1)
        returnList.append(E2)
        returnList.append(myList) 
      
        return returnList

# Write Position Registers Cartesian  (PR[])
# argument PRNumber is the PR register being written to inside the robot - copied to Postition Register by TP Program
# argument SyncDInput is the DI[x] register being written to inside the robot to tell the robot to start the transfer 
# to the position register
# other argument as list [UTOOL, UFRAME, X, Y, Z, W, P, R, Turn1, Turn2, Bitflip, EXT_0, EXT_1, EXT_2]


def writeCartesianPositionRegister(drive_path, PRNumber, myList):

   #must set UT/UF to 0


   #Build myByteArray
   myByteArray = bytearray(struct.pack('H', 0x0000)) #UTOOL
   myByteArray.extend(struct.pack('H', 0))      #UFRAME
   myByteArray.extend(struct.pack('f', myList[2]))      #X
   myByteArray.extend(struct.pack('f', myList[3]))      #Y
   myByteArray.extend(struct.pack('f', myList[4]))      #Z
   myByteArray.extend(struct.pack('f', myList[5]))      #W
   myByteArray.extend(struct.pack('f', myList[6]))      #P
   myByteArray.extend(struct.pack('f', myList[7]))      #R
   myByteArray.extend(struct.pack('B', myList[8]))      #Turn1
   myByteArray.extend(struct.pack('B', myList[9]))      #Turn2
   myByteArray.extend(struct.pack('B', myList[10]))     #Turn3
   myByteArray.extend(struct.pack('B', myList[11]))     #Bitflip
   myByteArray.extend(struct.pack('f', myList[12]))     #EXT_0
   myByteArray.extend(struct.pack('f', myList[13]))     #EXT_1
   myByteArray.extend(struct.pack('f', myList[14]))     #EXT_2
   
   if (DEBUG == True):
     print("len(bytes(myByteArray=)", len(bytes(myByteArray)))

     print(myByteArray)

   with CIPDriver(drive_path) as drive:
        myTag = drive.generic_message(
            service=Services.set_attribute_single,
            class_code=0x7B,
            instance=0x01,
            attribute=PRNumber,
            data_type=None,
            connected=False,
            request_data=bytes(myByteArray[0:44]),
            unconnected_send=False,
            route_path=False,
            name='fanucPRSwrite'
        )
        if (DEBUG == True):
          print("Write PR registers Current Cartesian Coordinates 0x7B ")
          print(myTag)
          print('myTag.error=', myTag.error)
          print("myTag.type=", myTag.type)


   return myTag.error

#################
# Joint Space Functions
#################

def returnJointCurrentPosition(drive_path):

# read all CURJPOS Joint Outputs class 0x7D, I-0x1, A-0x01
  with CIPDriver(drive_path) as drive:
        myPRTag = drive.generic_message(
            service=Services.get_attribute_single,
            class_code=0x7E,
            instance=0x01,
            attribute=0x01,
            data_type=None,
            connected=False,
            unconnected_send=False,
            route_path=True,
            name='fanucCURJPOSread'
        )
        if (DEBUG == True):
          print("CURJPOS Current Joint Coordinates 0x7E, IA< 0x01")
          print(myPRTag)
          print("myPRTag.type=", myPRTag.type)
        myList = list(myPRTag.value)
        if (DEBUG == True):
          print("myList=", myList)

  UTOOL = myList[1]*8+myList[0]
  UFRAME = myList[3]*8+myList[2]
  J1=struct.unpack('f', bytes(myList[4:8]))
  J2=struct.unpack('f', bytes(myList[8:12]))
  J3=struct.unpack('f', bytes(myList[12:16]))
  J4=struct.unpack('f', bytes(myList[16:20]))
  J5=struct.unpack('f', bytes(myList[20:24]))
  J6=struct.unpack('f', bytes(myList[24:28]))
  J7=struct.unpack('f', bytes(myList[28:32]))
  J8=struct.unpack('f', bytes(myList[32:36]))
  J9=struct.unpack('f', bytes(myList[36:40]))
  if (math.isnan(J7[0]) == True):
     J7=(0.0,)
  J8=struct.unpack('f', bytes(myList[32:36]))
  if (math.isnan(J8[0]) == True):
     J8=(0.0,)
  J9=struct.unpack('f', bytes(myList[36:40]))
  if (math.isnan(J9[0]) == True):
     J9=(0.0,)

  if (DEBUG == True):
    print("UTOOL=", UTOOL)
    print("UFRAME=", UFRAME)
    print("J1=", J1)
    print("J2=", J2)
    print("J3=", J3)
    print("J4=", J4)
    print("J5=", J5)
    print("J6=", J6)
    print("J7=", J7)
    print("J8=", J8)
    print("J9=", J9)
  
  returnList = []  
  returnList.append(UTOOL)
  returnList.append(UFRAME)
  returnList.append(J1[0])
  returnList.append(J2[0])
  returnList.append(J3[0])
  returnList.append(J4[0])
  returnList.append(J5[0])
  returnList.append(J6[0])
  returnList.append(J7[0])
  returnList.append(J8[0])
  returnList.append(J9[0])
  returnList.append(myList) 

  return returnList


def readJointPositionRegister(drive_path, PRNumber):


  with CIPDriver(drive_path) as drive:
        myTag = drive.generic_message(
            service=Services.get_attribute_single,
            class_code=0x7C,
            instance=0x01,
            attribute=0x01,
            data_type=None,
            connected=False,
            unconnected_send=False,
            route_path=False,
            name='fanucPRSread'
        )


        if (DEBUG == True):
          print("PR Contents Joint Coordinates 0x7C, IA< 0x01")
          print(myTag)
          print("myTag.type=", myTag.type)
        myList = list(myTag.value)
        if (DEBUG == True):
          print("myList=", myList)

  UTOOL = myList[1]*8+myList[0]
  UFRAME = myList[3]*8+myList[2]
  J1=struct.unpack('f', bytes(myList[4:8]))
  J2=struct.unpack('f', bytes(myList[8:12]))
  J3=struct.unpack('f', bytes(myList[12:16]))
  J4=struct.unpack('f', bytes(myList[16:20]))
  J5=struct.unpack('f', bytes(myList[20:24]))
  J6=struct.unpack('f', bytes(myList[24:28]))
  J7=struct.unpack('f', bytes(myList[28:32]))
  if (math.isnan(J7[0]) == True):
     J7=(0.0,)
  J8=struct.unpack('f', bytes(myList[32:36]))
  if (math.isnan(J8[0]) == True):
     J8=(0.0,)
  J9=struct.unpack('f', bytes(myList[36:40]))
  if (math.isnan(J9[0]) == True):
     J9=(0.0,)

  if (DEBUG == True):
    print("UTOOL=", UTOOL)
    print("UFRAME=", UFRAME)
    print("J1=", J1)
    print("J2=", J2)
    print("J3=", J3)
    print("J4=", J4)
    print("J5=", J5)
    print("J6=", J6)
    print("J7=", J7)
    print("J8=", J8)
    print("J9=", J9)
  
  returnList = []  
  returnList.append(UTOOL)
  returnList.append(UFRAME)
  returnList.append(J1[0])
  returnList.append(J2[0])
  returnList.append(J3[0])
  returnList.append(J4[0])
  returnList.append(J5[0])
  returnList.append(J6[0])
  returnList.append(J7[0])
  returnList.append(J8[0])
  returnList.append(J9[0])
  returnList.append(myList) 

  return returnList


# Write Position Registers Joint  (PR[])
# argument PRNumber is the PR register being written to inside the robot - copied to Postition Register by TP Program
# argument SyncDInput is the DI[x] register being written to inside the robot to tell the robot to start the transfer 
# to the position register
# other argument as list [UTOOL, UFRAME, J1, J2, J3, J4, J5, J6, J7, J8, J9] 




def writeJointPositionRegister(drive_path, PRNumber, myList):

   #must set UT/UF to 0


   #Build myByteArray
   myByteArray = bytearray(struct.pack('H', 0x0000)) #UTOOL
   myByteArray.extend(struct.pack('H', 0x0000))      #UFRAME
   myByteArray.extend(struct.pack('f', myList[2]))      #J1
   myByteArray.extend(struct.pack('f', myList[3]))      #J2
   myByteArray.extend(struct.pack('f', myList[4]))      #J3
   myByteArray.extend(struct.pack('f', myList[5]))      #J4
   myByteArray.extend(struct.pack('f', myList[6]))      #J5
   myByteArray.extend(struct.pack('f', myList[7]))      #J6
   myByteArray.extend(struct.pack('f', myList[8]))      #J7
   myByteArray.extend(struct.pack('f', myList[9]))      #J8
   myByteArray.extend(struct.pack('f', myList[10]))     #J9
   
   if (DEBUG == True):
     print("len(bytes(myByteArray=)", len(bytes(myByteArray)))

     print(myByteArray)

   with CIPDriver(drive_path) as drive:
        myTag = drive.generic_message(
            service=Services.set_attribute_single,
            class_code=0x7C,
            instance=0x01,
            attribute=PRNumber,
            data_type=None,
            connected=False,
            request_data=bytes(myByteArray[0:40]),
            unconnected_send=False,
            route_path=False,
            name='fanucPRSwrite'
        )
        if (DEBUG == True):
          print("Write PR registers Current Joint Coordinates 0x7C ")
          print(myTag)
          print('myTag.error=', myTag.error)
          print("myTag.type=", myTag.type)


   return myTag.error





# write to 16 bit R[] Register 
# RegNumb is R[] Register Number, Value is up to 16 bits
# returns error code
def writeR_Register(drive_path, RegNum, Value):

   myBytes = Value.to_bytes(4,'little')
   with CIPDriver(drive_path) as drive:
        myTag = drive.generic_message(
            service=0x10,
            class_code=0x6B,
            instance=0x1,
            attribute=RegNum,
            request_data=myBytes[0:4],
            data_type=None,
            connected=False,
            unconnected_send=False,
            route_path=False,
            name='fanucDOread'
        )
   return myTag.error


# read from 16 bit R[] Register 
# RegNumb is R[] Register Number
# returns Value
def readR_Register(drive_path, RegNum):
	
   with CIPDriver(drive_path) as drive:
        myTag = drive.generic_message(
            service=0xe,
            class_code=0x6B,
            instance=0x1,  
            attribute=RegNum,
            data_type=None,
            connected=False,
            unconnected_send=False,
            route_path=False,
            name='fanucRread'
        )
        if (DEBUG == True):
          print("R[%d]= %x",RegNum,myTag.value)
          print(myTag)
          print("myTag.type=", myTag.type)
        myList = list(myTag.value)
        if (DEBUG == True):
          print("myList=", myList)
   return myList[0]

def readDigitalInputs(drive_path):

    with CIPDriver(drive_path) as drive:
        myTag = drive.generic_message(
            service=Services.get_attribute_single,
            class_code=0x04,
            instance=0x320,
            attribute=0x03,
            data_type=None,
            connected=False,
            unconnected_send=False,
            route_path=False,
            name='fanucDIread'
        )
        if (DEBUG == True):
          print("Digital Input 0x320")
        myList = list(myTag.value)
        if (DEBUG == True):
          print("myList=", myList)
        return myList


def readDigitalOutputs(drive_path):
  # read all Digital Outputs 0x321
  with CIPDriver(drive_path) as drive:
        myTag = drive.generic_message(
            service=Services.get_attribute_single,
            class_code=0x04,
            instance=0x321,
            attribute=0x03,
            data_type=None,
            connected=False,
            unconnected_send=False,
            route_path=False,
            name='fanucDOread'
        )
        if (DEBUG == True):
          print("Digital Outputs 0x321")
          print(myTag)
          print("myTag.type=", myTag.type)
        myList = list(myTag.value)
        if (DEBUG == True):
          print("myList=", myList) 
        return myList
  


def readDigitalInput(drive_path, InputNumber):
  '''
  Read one digital input value (I.E. Read the value at DI[1])

  :param str drive-path: IP location of the robot
  :param int InputNumber: Digital Input (DI) you want to read
  :return: Value at DI[ InputNumber ]
  :rtype: int (1 or 0)

  :raises ValueError: if InputNumber <=0
  '''
  # Each input register is 8-bits long (I.E. R1 holds DI[1:8])
  if not InputNumber: # If Input is 0
    raise ValueError("Cannot select 0-th register, does not exist")

  inputs = readDigitalInputs(drive_path) # Read in all registers
  register = ((InputNumber-1)//8) # What register block the input is in
  value = inputs[register] >> ((InputNumber-1)  % 8) # Get the value at the input position requested and whatever is to the right
  value = value &1 # Truncate to just the position
  #print("Register Num:",register)
  #print("Full Register:",inputs[register])
  print("Value:", value)
  return value


def readDigitalOutput(drive_path, OutputNumber):
  '''
  Read one digital output value (I.E. Read the value at DO[1])

  :param str drive-path: IP location of the robot
  :param int OutputNumber: Digital Output (DO) you want to read
  :return: Value at DO[ OutputNumber ]
  :rtype: int (1 or 0)

  :raises ValueError: if OutputNumber <=0
  '''
  # Each output register is 8-bits long (I.E. R1 holds DO[1:8])
  if not OutputNumber: # If Input is 0
    raise ValueError("Cannot select 0-th register, does not exist")

  outputs = readDigitalOutputs(drive_path) # Read in all registers
  register = ((OutputNumber-1)//8) # What register block the output is in
  value = outputs[register] >> ((OutputNumber-1)  % 8) # Get the value at the output position requested and whatever is to the right
  value = value &1 # Truncate to just the position
  #print("Register Num:",register)
  #print("Full Register:",outputs[register])
  print("Value:", value)
  return value
   

def writeDigitalInput(drive_path, OutputNumber, Value):
  """
  Fanuc does not support writes to DI/DO with explicit messaging (what we are doing here).
  Possible work around in the future?
  """
  raise NotImplementedError("writeDigialInput: This function will do nothing. WIP")
  register = ((OutputNumber-1)//8) # What register block the output is in
  bit = ((OutputNumber-1) % 8) # What bit in that register needs to be edited

  print("Register:", register+1)
  print("Bit:",bit)


  outputs = readDigitalOutputs(drive_path) # Get current register values
  Old_R = outputs[register] # Old value at output register X
  print("Old Register Value:", Old_R)
  print("Old Bin:",bin(Old_R))
  print("R:",readR_Register(drive_path,register+1))

  # This prevents us from changing the value of nearby digital registers
  if Value:
    if Value > 1:
      print("Value is larger than 1, setting to 1...")
      Value = 1 
    New_R = Old_R | (1<<bit)
  elif Value == 0:
    New_R = Old_R & ~(1<<bit)
  else:
     raise ValueError("Value cannot be a negative integer.")
  
  print("New Register Value:", New_R)
  print("New Bin:",bin(New_R))

  outputs[register] = New_R
  
  bytesMessage = bytes(outputs)
  print("Message:",bytesMessage)
  with CIPDriver(drive_path) as drive:
        myTag = drive.generic_message(
            service=Services.set_attribute_single,
            class_code=0x04,
            instance=0x320,
            attribute=0x03,
            request_data=bytesMessage,
            data_type=None,
            connected=False,
            unconnected_send=False,
            route_path=False,
            name='fanucDIread'
        )
  return myTag.error



class FANUCAlarm:
    '''!
        Class implementing the FANUC EIP Alarm objects. Alarm classes (types) are listed in FANUCAlarm.types.
        Data contained by each class type can be requested using the get attribute class methods.

        Get attribute single method fetches one of the attributes documented in FANUCAlarm.attributes enum
            and decodes it by variable type.

        Get attribute all method fetches all attributes listed in FANUCAlarm.attributes enum, decodes them
            and places them in a dictionary which can be keyed with FANUCAlarm.attributes.<attribute>.name
    '''

    # fanuc alarm classes
    class types(Enum):
        '''!
        FANUC alarm classes. 

        These define classes of alarms which can be used to filter and select alarms
        which the user would like access to.

        Note: you may occasionally receive a CIP error with a message such as -
            "Destination unknown, class unsupported, instance undefined or structure 
             element undefined (see extended status) - Extended status out of memory  (05, 00)"

             When this error occurs, the get attribute (single or all) will return None and print
             the error message from the CIP tag object.
        '''
        ## Currently active alarm on the system. Each instance corresponds to an active alarm.
        active_alarm=0xA0           # errors
        ## Each object corresponds to an alarm in the history.
        alarm_history=0xA1
        ## A motion alarm object. Instance 1 corresponds to the most recent.
        motion_alarm=0xA2
        ## A system alarm object. Instance 1 corresponds to the most recent.
        system_alarm=0xA3 
        ## Application alarm object. Instance 1 is most recent.
        application_alarm=0xA4
        ## Recovery alarm object. Instance 1 is most recent.
        recovery_alarm=0xA5
        ## Communications alarm object. Instance 1 is most recent.
        communications_alarm=0xA6   # errors

    class attributes(Enum):
        '''!
        FANUC alarm object attributes.

        These define information fields attached to each alarm object created by FANUC
        and exposed to the EIP interface.
        
        Enum member is tuple containing (attribute #, data type, *offset*, *length*).
        * = optional

        Note: None types assumed to be strings.
        '''
        ## The alarm ID or alarm code.
        alarm_id = (0x01, DataTypes.int)                # 16 bit int
        ## The alarm number.
        alarm_number = (0x02, DataTypes.int)            # 16 bit int
        ## The cause code of the alarm ID.
        alarm_id_cause_code = (0x03, DataTypes.int)     # 16 bit int
        ## The cause code of the alarm number.
        alarm_num_cause_code = (0x04, DataTypes.int)    # 16 bit int
        ## The alarm severity.
        alarm_severity = (0x05, DataTypes.int)          # 16 bit int
        ## The alarm time stamp in 32-bit MSDOS format.
        time_stamp = (0x06, DataTypes.dint)             # 32 bit int
        ## The alarm time stamp in human readable string.
        date_time_str = (0x07, None, 16, 28)            # 28 byte string
        ## The alarm message in a human readable string.
        alarm_message = (0x08, None, 44, 88)            # 88 byte string
        ## The alarm cause code message in a human readable string.
        cause_code_message = (0x09, None, 132, 88)      # 88 byte string
        ## The alarm severity in a human readable string.
        alarm_severity_str = (0x0A, None, 220, 28)      # 28 byte string

    
    def __init__(self):
        self.buff = None                 # byte string representation

    # Decodes byte strings into string and returns 
    #   fanuc strings follow convention similar to pascal strings:
    #       2 byte length N
    #       2 byte pad
    #       N length string
    #       pad 
    @classmethod
    def __string_decode__(self, buff, offset:int, length:int):
        '''!
        Decodes byte strings into string and returns.

        FANUC strings follow convention similar to pascal strings:
               2 byte: length N
               2 byte: pad
               N length string
               (2? byte:) pad 

        @param[in] buff A byte string buffer received from the robot controller through the EIP interface.
        @param[in] offset The starting offset in bytes required for Python's unpack_from() method.
        '''
        msg_len = struct.unpack_from('h', buff, offset)
        msg_pad = length - (msg_len[0] - 1) - 5
        format = f'h2x{msg_len[0]-1}s{msg_pad}x'

        msg_str = struct.unpack_from(format, buff, offset)
        msg_str = msg_str[1].decode('utf-8')

        return msg_str


    @classmethod
    def get_attribute_single(self, 
                             drive_path: str, 
                             class_code: int=types.alarm_history, 
                             instance: int=1, 
                             attribute: int=attributes.alarm_number) -> dict:
        '''!
        Uses CIP service get_attribute_single to get one attribute defined by FANUCAlarm.attributes enum,
        decode it, and return the value in a dictionary keyed by FANUCAlarm.attributes.<attribute>.name.

        @param[in] drive_path Robot IP address as a string.
        @param[in] class_code A byte identifier defined by FANUCAlarm.types, specifies what type of alarm object to return.
        @param[in] instance An integer indexing which alarm instance to request. Instance 1 selects most recent object, higher values select older objects.
        @param[in] attribute The attribute or field to retrieve from the requested alarm object.

        @return A dictionary containing the requested attribute, indexed by the attribute enum name property. Returns None if CIP tag contains an error.
        '''
        # get a single attribute from list above

        with CIPDriver(drive_path) as driver:

            cip_tag = driver.generic_message(
                                service=Services.get_attribute_single,
                                class_code=class_code.value,
                                instance=1,
                                attribute=attribute.value[0],
                                data_type=attribute.value[1],
                                connected=False,
                                unconnected_send=False,
                                route_path=False,
                                name=attribute.name
                            )

            if not cip_tag:
                print(f'[ERROR] CIP tag: \"{cip_tag.tag}\"\n\tmessage: \"{cip_tag.error}\"')
                return None

            else:

                output = {}
                if attribute.value[1] is None:
                    # interpret this as a string and do
                    #   complicated decode
                    output_val = self.__string_decode__(cip_tag.value, 0, attribute.value[3])
                    output[attribute.name] = output_val
                    return output

                else:
                    output[attribute.name] = cip_tag.value
                    return output
                
    @classmethod
    def get_attributes_all(self, 
                           drive_path: str, 
                           class_code: int=types.alarm_history, 
                           instance: int=1) -> dict:
        '''!
        Get all attributes of a FANUC alarm object in a dictionary.

        @param[in] drive_path Robot IP address as a string.
        @param[in] class_code A byte identifier defined by FANUCAlarm.types, specifies what type of alarm object to return.
        @param[in] instance An integer indexing which alarm instance to request. Instance 1 selects most recent object, higher values select older objects.

        @return A dict object containing all attributes of the desired alarm object, keyed by the attribute enum name property. Returns none if CIP tag contains an error.
        '''
        with CIPDriver(drive_path) as driver:

            cip_tag = driver.generic_message(
                                service=Services.get_attributes_all,
                                class_code=class_code.value,
                                instance=1,
                                attribute=None,
                                data_type=None,
                                connected=False,
                                unconnected_send=False,
                                route_path=False,
                                name=class_code.name
                            )

            if not cip_tag:
                print(f'[ERROR] CIP tag: \"{cip_tag.tag}\"\n\tmessage: \"{cip_tag.error}\"')
                return None

            else:
                # process and return as dict
                buff = cip_tag.value

                alarm_dict = {
                        'alarm_id': DataTypes.int.decode(buff[:2]),
                        'alarm_number': DataTypes.int.decode(buff[2:4]),
                        'alarm_id_cause_code': DataTypes.int.decode(buff[4:6]),
                        'alarm_num_cause_code': DataTypes.int.decode(buff[6:8]),
                        'alarm_severity': DataTypes.int.decode(buff[8:10]),
                        'pad': DataTypes.int.decode(buff[10:12]),
                        'time_stamp': DataTypes.dint.decode(buff[12:12+4]),
                        }

                # time stamp string: 2 bytes len | 2 bytes pad | <=20 bytes str | >=2 bytes pad
                #ts_str = struct.unpack_from('h2x22s2x', buff, 16)
                #ts_str = ts_str[1].decode('utf-8')
                ts_str = self.__string_decode__(buff, 16, 28)
                # error message: 2 bytes len | 2 bytes pad | <=82 bytes str | >=2 bytes pad
                msg_str = self.__string_decode__(buff, 44, 88)
                # cause code message: 2 bytes len | 2 bytes pad | <=82 bytes str | >= 2 bytes pad
                cc_str = self.__string_decode__(buff, 132, 88)
                ss_str = self.__string_decode__(buff, 220, 28)

                # place in dictionary
                alarm_dict['date_time_str'] = ts_str
                alarm_dict['alarm_message'] = msg_str
                alarm_dict['cause_code_message'] = cc_str
                alarm_dict['alarm_severity_str'] = ss_str

                if DEBUG:
                    print('decoded = ', alarm_dict)

                return alarm_dict


# Get the most recent alarm from the robot
def returnMostRecentAlarm(drive_path):
    # DEPRACTED! DEPRACTED I SAY!!!
    # ie, this is just an example of a crude way to retrieve alarms

    with CIPDriver(drive_path) as driver:
        cip_tag = driver.generic_message(
                    service=Services.get_attributes_all,
                    class_code=0xA1,
                    instance=0x01,
                    data_type=None,
                    connected=False,
                    unconnected_send=False,
                    route_path=False,
                    name='fanuc alarm history read'
                )

        if not cip_tag:
            print('[ERROR]', cip_tag.tag, cip_tag.error)

        else:
            # save byte buffer
            buff = cip_tag.value

            alarm_dict = {
                    'alarm_id': DataTypes.int.decode(buff[:2]),
                    'alarm_number': DataTypes.int.decode(buff[2:4]),
                    'cause_code_id': DataTypes.int.decode(buff[4:6]),
                    'cause_code_num': DataTypes.int.decode(buff[6:8]),
                    'severity': DataTypes.int.decode(buff[8:10]),
                    'pad': DataTypes.int.decode(buff[10:12]),
                    'time_stamp': DataTypes.dint.decode(buff[12:12+4]),
                    }

            # time stamp string: 2 bytes len | 2 bytes pad | <=20 bytes str | >=2 bytes pad
            ts_str = struct.unpack_from('h2x22s2x', buff, 16)
            ts_str = ts_str[1].decode('utf-8')
            # error message: 2 bytes len | 2 bytes pad | <=82 bytes str | >=2 bytes pad
            msg_len = struct.unpack_from('h', buff, 44)
            msg_pad = 82 - msg_len[0]
            format = f'h2x{msg_len[0]}s{msg_pad}x'
            msg_str = struct.unpack_from(format, buff, 44)
            msg_str = msg_str[1].decode('utf-8')
            # cause code message: 2 bytes len | 2 bytes pad | <=82 bytes str | >= 2 bytes pad
            cc_len = struct.unpack_from('hh', buff, 132)
            cc_pad = 82 - cc_len[0]
            format = f'h2x{cc_len[0]}s{cc_pad}x'
            # unpack to tuple of values
            cc_str = struct.unpack_from(format, buff, 132)
            cc_str = cc_str[1].decode('utf-8')

            # place in dictionary
            alarm_dict['time_stamp_str'] = ts_str
            alarm_dict['message'] = msg_str
            alarm_dict['cause_code'] = cc_str

            if DEBUG:
                print('decoded = ', alarm_dict)

            return alarm_dict



# get the alarm history as a string or something
# TODO convert from byte strings
def returnAlarmHistory(drive_path):
    # DEPRACTED! DEPRACTED I SAY!!!
    
    with CIPDriver(drive_path) as driver:
        cip_tag = driver.generic_message(
                    service=Services.get_attributes_all,
                    class_code=0xA1,
                    instance=0x01,
                    attribute=0x00,
                    data_type=None,
                    connected=False,
                    unconnected_send=False,
                    route_path=False,
                    name='fanuc alarm history read'
                )

        if not cip_tag:
            print('[ERROR]', cip_tag.tag, cip_tag.error)

        else:
            print(cip_tag.tag, cip_tag.value)
            return cip_tag.value
