import os
import random
import time
import sys
sys.path.append('fanuc_ethernet_ip_drivers/src')
from robot_controller import robot
import paho.mqtt.client as mqtt_client
import json
# MQTT broker host (set MQTT_BROKER environment variable)
broker = os.environ.get('MQTT_BROKER', 'localhost')
# broker port
port = 1883
# Just a declaration of the global variable
topic = "robot/test"
# IP of the DJ robot (set ROBOT_IP environment variable)
dj_ip = os.environ.get('ROBOT_IP', '0.0.0.0')
# Topic that DJ will publish to
dj_topic = "robot/dj"
# Topic that DJ will Subscribe to
bill_topic = "robot/bill"
# Offsets are always added
z_offset = -8
y_offset = 1750
# Global flag for received message
msg_rec = False
# Global variables to store Bill's data
Bill_gripper= False
Bill_x_coordinate = 0
Bill_y_coordinate = 0
Bill_z_coordinate = 0
loop_finished = False
# The position where the two robots meet to handoff the object
handoff_middle_pos = [250,850,100,-90.0,-30.0,0.0]
# the handoff position that is far away safe from bumping into Bill
handoff_stay_away_pos =[250,500,100,-90.0,-30.0,0.0]

def find_random_position():
   """Find a random position near the handoff position"""
   new_pos=handoff_middle_pos.copy()
   x=handoff_middle_pos[0]
   y=handoff_middle_pos[1]
   z=handoff_middle_pos[2]
   new_pos[0]=random.randint(x-150,x+150)
   new_pos[1]=random.randint(y-150,y+150)
   new_pos[2]=random.randint(z-125,z+125)
   return new_pos

def package_json(coordinate, gripper_status, loop_finished=False):
   """Package the JSON string to send over MQTT to other robot

   Args:
       coordinate (List): current position to send to other robot
       gripper_status (Bool): 0=closed, 1=open
       loop_done (bool, optional): _description_. Defaults to False.
   """
   data = {
       "gripper": gripper_status,
       "x_coordinate": coordinate[0],
       "y_coordinate": coordinate[1],
       "z_coordinate": coordinate[2],
       "loop_finished": loop_finished
       }
   encoded_msg = json.dumps(data, indent=4)
   return encoded_msg

def parse_msg(msg):
    """Parse the JSON string received over MQTT from other robot
    
    Args:
        msg (String): JSON string received from other robot
    """
    msg_dict = json.loads(msg)
    return msg_dict["gripper"], msg_dict["x_coordinate"], msg_dict["y_coordinate"] + y_offset, msg_dict["z_coordinate"] + z_offset, msg_dict["loop_finished"]

def connect_mqtt() -> mqtt_client:
    """Connect to the MQTT broker"""
    def on_connect(client, userdata, flags, rc):
        """Callback function for when the client connects to the broker"""
        if rc == 0:
            print("Connected to MQTT Broker!")
        else:
            print("Failed to connect, return code %d\n", rc)

    client = mqtt_client.Client()
    # client.username_pw_set(username, password)
    client.on_connect = on_connect
    client.connect(broker, port)
    return client


def subscribe(client: mqtt_client):
    """Subscribe to the MQTT topics"""
    topic = bill_topic
    message = None
    def on_message(client, userdata, msg):
        """Callback function for when a message is received from the broker"""
        print(f"Received `{msg.payload.decode()}` from `{msg.topic}` topic")
        global msg_rec
        msg_rec = True
        global Bill_gripper_state
        global Bill_x_coordinate
        global Bill_y_coordinate
        global Bill_z_coordinate
        global loop_finished
        Bill_gripper, Bill_x_coordinate, Bill_y_coordinate, Bill_z_coordinate, loop_finished = parse_msg(msg.payload.decode())
    client.subscribe(topic)
    client.on_message = on_message
def publish(client, msg):
        """Publish a message to the MQTT broker"""
        topic = dj_topic
        result = client.publish(topic, msg)
        status = result[0]
        if status == 0:
            print(f"Sent `{msg}` to topic `{topic}`")
        else:
            print(f"Failed to send message to topic {topic}")
        time.sleep(1)

def run():
    global msg_rec
    DJ = robot(dj_ip)
    DJ.set_speed(300)
    DJ.schunk_gripper('open')
    time.sleep(1)
    home_position = [0.0,0.0,0.0,0.0,-90.0,30.0]
    DJ.write_joint_pose(home_position)
    client = connect_mqtt()
    client.loop_start()
    DJ.write_cartesian_position(handoff_stay_away_pos)
    while not loop_finished:
        subscribe(client)
        while msg_rec == False:
            time.sleep(0.2)
        msg_rec = False
        print(f"Bill x coordinate: {Bill_x_coordinate}\nBill y coordinate: {Bill_y_coordinate}\nBill z coordinate: {Bill_z_coordinate}")
        DJ.write_cartesian_position([Bill_x_coordinate,Bill_y_coordinate-50,Bill_z_coordinate,-90.0,-30.0,0.0])
        DJ.write_cartesian_position([Bill_x_coordinate,Bill_y_coordinate,Bill_z_coordinate,-90.0,-30.0,0.0])
        DJ.schunk_gripper('close')
        msg = package_json([0.0,0.0,0.0], False, loop_finished)
        client.publish(dj_topic, msg)
        subscribe(client)
        while msg_rec == False:
            time.sleep(0.2)
        msg_rec = False
        random_pos = find_random_position()
        DJ.write_cartesian_position(random_pos)
        msg = package_json(random_pos, False, loop_finished)
        time.sleep(.5)
        publish(client, msg)
        subscribe(client)
        while msg_rec == False:
            time.sleep(0.2)
        msg_rec = False
        DJ.schunk_gripper('open')
        DJ.write_cartesian_position([DJ.read_current_cartesian_pose()[0],DJ.read_current_cartesian_pose()[1]-50,DJ.read_current_cartesian_pose()[2],-90.0,-30.0,0.0])
        DJ.write_cartesian_position(handoff_stay_away_pos)
        msg = package_json(handoff_stay_away_pos, True, loop_finished)
        time.sleep(.5)
        publish(client, msg)
        msg_rec = False
    client.loop_stop()
    client.disconnect()
    DJ.schunk_gripper('close')
    DJ.schunk_gripper('open')
    DJ.schunk_gripper('close')
    DJ.schunk_gripper('open')
    DJ.schunk_gripper('close')
    DJ.schunk_gripper('open')
    DJ.write_joint_pose(home_position)
    print("Finished all loops")

if __name__ == '__main__':
    run()

