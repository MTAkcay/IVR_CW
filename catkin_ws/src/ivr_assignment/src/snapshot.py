#!/usr/bin/env python3

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError

class snapshot:

    # Defines publisher and subscriber
    def __init__(self):
                # initialize the bridge between openCV and ROS
        self.bridge = CvBridge()
        # initialize the node named image_processing
        rospy.init_node('snapshot', anonymous=True)
        # initialize a subscriber to receive messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
        self.image_sub1 = rospy.Subscriber("/camera1/robot/image_raw", Image, self.callback1)
        self.image_sub2 = rospy.Subscriber("/camera2/robot/image_raw", Image, self.callback2)
        # initialize a publisher to send robot end-effector position

    # Receive data, process it, and publish
    def callback1(self, data):
        # Receive the image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            cv2.imwrite('camera_1.png', cv_image)
        except CvBridgeError as e:
            print(e)

    def callback2(self, data):
        # Receive the image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            cv2.imwrite('camera_2.png', cv_image)
        except CvBridgeError as e:
            print(e)


# call the class
def main(args):
    ss = snapshot()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)
