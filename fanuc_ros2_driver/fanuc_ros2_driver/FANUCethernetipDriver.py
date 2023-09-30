#############
#
# FANUC Ethernet/IP Driver
# Version 1.0
# 6/2/2023
# Shovic, et.al.
# Center for Intelligent Industrial Robotics
# University of Idaho
#
############
# Requires Ethernet/IP FANUC driver and 30 Series Controller
# 

DEBUG = True

import sys
sys.path.append('./pycomm3/pycomm3')
from pycomm3 import CIPDriver
from pycomm3 import Services
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

   
   return


def readDigitalOutput(drive_path, OutputNumber):

   return

def writeDigitalInput(drive_path, OutputNumber, Value):

   return
