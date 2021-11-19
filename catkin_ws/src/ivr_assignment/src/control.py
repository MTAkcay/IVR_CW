#!/usr/bin/env python3

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from cv_bridge import CvBridge, CvBridgeError


class control:
    def __init__(self):
        rospy.init_node("control", anonymous=True)
        self.joint1Sub = rospy.Subscriber("joint_angle_1", Float64, self.callback0)
        self.joint3Sub = rospy.Subscriber("joint_angle_3" , Float64, self.callback1)
        self.joint4Sub = rospy.Subscriber("joint_angle_4" , Float64, self.callback2)
        self.targetSub = rospy.Subscriber("/target_control/target_pos", Float64MultiArray, self.callback3)
        self.joint1Pub = rospy.Publisher("/robot/joint1_position_controller/command", Float64, queue_size=10)
        self.joint3Pub = rospy.Publisher("/robot/joint3_position_controller/command" , Float64, queue_size=10)
        self.joint4Pub = rospy.Publisher("/robot/joint4_position_controller/command" , Float64, queue_size=10)
        
    def callback0(self, data):
    # Receive the image
	self.joint1 = data
        
    def callback1(self, data):
    # Receive the image
	self.joint3 = data

    def callback2(self, data):
    # Receive the image
    	self.joint4 = data
    	
    def callback3(self, data):
    # Receive the image
    	self.target = data

# call the class
def main(args):
    c = control()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)


