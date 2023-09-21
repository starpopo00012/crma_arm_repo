#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
import math
import serial
import time

# Initialize the node
rospy.init_node("joint_publish_gui")
# Publisher object
publisherObject = rospy.Publisher("/joint_states", JointState, queue_size=10)
# Rate controller
rateController = rospy.Rate(50)
# Variables
msg = JointState()
msg.header.frame_id = ""
msg.name = ["joint1","joint2","joint3","joint4","joint5","joint6_1","joint6_2","joint7_1","joint7_2"]       # Joint name (array assignment)

j1 = 0
j2 = 0
j3 = 0
j4 = 0
j5 = 0
j6 = 0
j7 = 0
j8 = 0
j9 = 0

robot_comport = rospy.get_param('~robot_comport') # get parameter from launch agrument
comport = serial.Serial(robot_comport,115200,timeout=1)
comport.close()
comport.open()

# waiting for arduino ready
string_rev = ""
while True:
    string_rev = comport.read_until('\r'.encode()).decode('utf-8')
    if "Home" in string_rev:
        break
    time.sleep(0.25)

# Main loop
while (not rospy.is_shutdown()):
    try:
        #print("1")
        string_rev = ""
        joint_degree = 0
        while len(string_rev) < 2:
            string_rev = comport.read_until('\r'.encode()).decode('utf-8')
            #print (string_rev)
        joint_degree = string_rev.split(',')
        #print(joint_degree)
        
        j1 = int(joint_degree[1])
        j2 = int(joint_degree[2])
        j3 = int(joint_degree[3])
        j4 = int(joint_degree[4])
        j5 = int(joint_degree[5])
        j6 = int(joint_degree[6])

        j_rad1 = math.pi - (j1*math.pi / 180) - 22*math.pi/180
        j_rad2 = math.pi - (j2*math.pi / 180) - 12*math.pi/180
        j_rad3 = math.pi - (j3*math.pi / 180) - 16*math.pi/180
        j_rad4 = (j4*math.pi / 180) - 10*math.pi/180
        j_rad5 = math.pi - (j5*math.pi / 180) - 19*math.pi/180
        j_rad6 = j6*math.pi / 180
        j_rad7 = j_rad6
        j_rad8 = -j_rad6
        j_rad9 = j_rad8

        # Initialize the time of publishing
        msg.header.stamp = rospy.Time.now()
        # Joint angle values
        msg.position = [j_rad1,j_rad2,j_rad3,j_rad4,j_rad5,j_rad6, j_rad7,j_rad8, j_rad9]
        # Publish message
        publisherObject.publish(msg)
        # Increase sequence
        msg.header.seq += 1
        rateController.sleep()
    
    except KeyboardInterrupt:# or rospy.ROSInternalException:
        rospy.logfatal("Node crashed due to an internal exception")
        comport.close()
        print("comport close and End program")
