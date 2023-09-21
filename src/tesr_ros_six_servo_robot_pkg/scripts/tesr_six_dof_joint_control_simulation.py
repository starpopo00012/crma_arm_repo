#!/usr/bin/env python3
from __future__ import print_function
import roslib
import rospy
import sys
import copy
from serial.tools import list_ports
from sensor_msgs.msg import JointState
import math
import geometry_msgs.msg
import _thread
import time
import numpy as np
from tf import transformations
from std_msgs.msg import String
from geometry_msgs.msg import *

# Initialize the node
rospy.init_node("joint_publish_gui")
# Publisher object
publisherObject = rospy.Publisher("joint_states", JointState, queue_size=10)
# Rate controller
rateController = rospy.Rate(100)
# Variables
msg = JointState()
msg.header.frame_id = ""
msg.name = ["joint1","joint2","joint3","joint4","joint5","joint6_1","joint6_2","joint7_1","joint7_2"]       # Joint name (array assignment)
j1 = 90*math.pi/180
j2 = 90
j3 = 45
j4 = 45
j5 = 90
j6 = 25
j7 = 0
j8 = 0
j9 = 0
i = 0
increment = True

try:
         # Main loop
    while(not rospy.is_shutdown()):
        time.sleep(0.1)
        # Initialize the time of publishing
        msg.header.stamp = rospy.Time.now()
        # Joint angle values
        msg.position = [j1, j2, j3, j4, j5, j6, j7, j8, j9]
        # Publish message
        publisherObject.publish(msg)
        # Increase sequence
        msg.header.seq += 1
        # Change angle value
        if increment:
            i += 1
            j1 += 90.0/30.0 * math.pi / 180.0
            j2 = 90 * math.pi / 180.0
            j3 += 0.5 * math.pi / 180.0
            j4 += 0.5 * math.pi / 180.0
            j5 += 0.5 * math.pi / 180.0
            j6 += 0.5 * math.pi / 180.0
            j7 = j6
            j8 = -j6
            j9 = j8

        else:
            i -= 1
            j1 -= 90.0/30.0 * math.pi / 180.0
            j2 = 90 * math.pi / 180.0
            j3 -= 0.5 * math.pi / 180.0
            j4 -= 0.5 * math.pi / 180.0
            j5 -= 0.5 * math.pi / 180.0
            j6 -= 0.5 * math.pi / 180.0
            j7 = j6
            j8 = -j6
            j9 = j8

        if i > 30 or i < -30:
            increment = not increment
        # Delay execution to match rate
        rateController.sleep()
    
except KeyboardInterrupt or rospy.ROSInternalException:
    print("END program")
