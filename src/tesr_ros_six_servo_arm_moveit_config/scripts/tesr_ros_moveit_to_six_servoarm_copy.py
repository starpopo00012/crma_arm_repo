#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
import math
import serial
import time

#robot_comport = rospy.get_param('~robot_comport') # get parameter from launch agrument
comport = serial.Serial("/dev/ttyUSB1",115200,timeout=1)
comport.close()
comport.open()
# Initialize the node
rospy.init_node("joint_subscribe_gui")

# Subscriber moveit
rateController = rospy.Rate(100)

def get_joint(robot_pose):
    global j1_deg,j2_deg,j3_deg,j4_deg,j5_deg
    current_pose = robot_pose
    cur_pos = current_pose.position
    j1 = cur_pos[0]
    j2 = cur_pos[1]
    j3 = cur_pos[2]
    j4 = cur_pos[3]
    j5 = cur_pos[4]
    # j6 = cur_pos[5]
    j1_deg = 180 - int(j1*180/math.pi) - 22 #Inverse_joint_value - Joint1_deg - offset
    j2_deg = 180 - int(j2*180/math.pi) - 12 #Inverse_joint_value - Joint2_deg - offset
    j3_deg = 180 - int(j3*180/math.pi) - 10#Inverse_joint_value - Joint3_deg - offset
    j4_deg = int(j4*180/math.pi) + 5 #Joint4_deg + offset
    j5_deg = 180 - int(j5*180/math.pi) - 19 #Inverse_joint_value - Joint5_deg - offset
    # j6_deg = int(j6*180/math.pi) + 29

    if j1_deg < 10:
        j1_deg = "00"+ str(j1_deg)
    elif j1_deg < 100:
        j1_deg = "0"+ str(j1_deg)
    if j2_deg < 10:
        j2_deg = "00"+ str(j2_deg)
    elif j2_deg < 100:
        j2_deg = "0"+ str(j2_deg)
    if j3_deg < 10:
        j3_deg = "00"+ str(j3_deg)
    elif j3_deg < 100:
        j3_deg = "0"+ str(j3_deg)
    if j4_deg < 10:
        j4_deg = "00"+ str(j4_deg)
    elif j4_deg < 100:
        j4_deg = "0"+ str(j4_deg)
    if j5_deg < 10:
        j5_deg = "00"+ str(j5_deg)
    elif j5_deg < 100:
        j5_deg = "0"+ str(j5_deg)
    # if j6_deg < 10:
    #     j6_deg = "00"+ str(j6_deg)
    # elif j6_deg < 100:
    #     j6_deg = "0"+ str(j6_deg)
    print(cur_pos)
    pos_deg = "S" + "," + str(j1_deg) + "," + str(j2_deg) + "," + str(j3_deg) + "," + str(j4_deg) + "," + str(j5_deg) + "," + "000" + "," + '\r'
    print(pos_deg)
    comport.write(bytes(str(pos_deg),'utf-8'))
    #data = comport.readline()
    #return data
    
subscriberMoveit = rospy.Subscriber("/move_group/fake_controller_joint_states", JointState, get_joint)
# Main loop
while (not rospy.is_shutdown()):
    try:
        get_joint
        
    except KeyboardInterrupt:# or rospy.ROSInternalException:
        rospy.logfatal("Node crashed due to an internal exception")
