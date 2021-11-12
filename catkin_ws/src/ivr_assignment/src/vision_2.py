#!/usr/bin/env python3

import sys

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Float64


class vision_2:
    def __init__(self):
        rospy.init_node("vision_2", anonymous=True)

        # initialize the bridge between openCV and ROS
        self.bridge = CvBridge()
        self.image_sub1 = rospy.Subscriber("/camera1/robot/image_raw", Image, self.callback1)
        self.image_sub2 = rospy.Subscriber("/camera2/robot/image_raw", Image, self.callback2)
        self.joint1Pub = rospy.Publisher("joint_angle_1", Float64, queue_size=10)
        self.joint3Pub = rospy.Publisher("joint_angle_3", Float64, queue_size=10)
        self.joint4Pub = rospy.Publisher("joint_angle_4a", Float64, queue_size=10)
        self.joint4Pubb = rospy.Publisher("joint_angle_4b", Float64, queue_size=10)
        self.joint4Pubc = rospy.Publisher("joint_angle_4c", Float64, queue_size=10)
        self.joint1 = Float64()
        self.joint3 = Float64()
        self.joint4 = Float64()
        self.joint4b = Float64()
        self.joint4c = Float64()
        self.greenC1 = np.array([])
        self.redC1 = np.array([])
        self.blueC1 = np.array([])
        self.yellowC1 = np.array([])

    def getCentre(self, mask):
        control = sum(sum(mask))
        if control < 10:
            return np.array([])
        M = cv2.moments(mask)
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
        return np.array([cX, cY])

    def findAllPoints(self, img):
        redMask = cv2.inRange(img, np.array([0, 0, 20]), np.array([20, 20, 255]))
        greenMask = cv2.inRange(img, np.array([0, 20, 0]), np.array([20, 255, 20]))
        blueMask = cv2.inRange(img, np.array([10, 0, 0]), np.array([255, 20, 20]))
        yellowMask = cv2.inRange(img, np.array([0, 10, 10]), np.array([0, 255, 255]))

        redCentre = self.getCentre(redMask)
        greenCentre = self.getCentre(greenMask)
        blueCentre = self.getCentre(blueMask)
        yellowCentre = self.getCentre(yellowMask)
        return redCentre, greenCentre, blueCentre, yellowCentre

    def callback1(self, data):
        # Receive the image
        try:
            self.cv_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        self.redC1, self.greenC1, self.blueC1, self.yellowC1 = self.findAllPoints(self.cv_image1)

    def callback2(self, data):
        # Receive the image
        try:
            self.cv_image2 = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        self.redC2, self.greenC2, self.blueC2, self.yellowC2 = self.findAllPoints(self.cv_image2)
        self.combinecenters()
        self.determinejointangles()
        self.publishangles()

    def publishangles(self):
        # Publish the results
        try:
            self.joint1Pub.publish(self.joint1)
            self.joint3Pub.publish(self.joint3)
            # self.joint4Puba.publish(self.joint4a)
            # self.joint4Pubb.publish(self.joint4b)
            self.joint4Pub.publish(self.joint4c)
        except CvBridgeError as e:
            print(e)

    def determinejointangles(self):
        self.vectorYB = self.finalBlueCenter - self.finalYellowCenter
        self.vectorBR = self.finalRedCenter - self.finalBlueCenter

        # when rotating about x axis, the x-coordinate doesn't change - the focus should be on y

        vecjoint2 = np.array([-self.vectorYB[1], self.vectorYB[2]])  # y axis also appears to be flipped from camera perspective
        vecjoint3 = np.array([self.vectorYB[0], self.vectorYB[2]])
        vecjoint4 = np.array([-self.vectorBR[1], self.vectorBR[2]])
        zUnitVector = np.array([0, -1])  # z axis is flipped

        self.joint1.data = self.angleBetweenVectors(zUnitVector, vecjoint2)
        self.joint3.data = self.angleBetweenVectors(zUnitVector, vecjoint3)
        self.joint4.data = self.angleBetweenVectors(vecjoint2, vecjoint4)

    def angleBetweenVectors(self, vectorFrom, vectorTo):
        return np.arctan2(np.cross(vectorTo, vectorFrom), np.dot(vectorFrom, vectorTo))

    def combinecenters(self):
        self.originPoint = self.originhandler(self.greenC1, self.greenC2)
        self.finalRedCenter = self.centercamfusion(self.redC1, self.redC2)
        self.finalBlueCenter = self.centercamfusion(self.blueC1, self.blueC2)
        self.finalYellowCenter = self.centercamfusion(self.yellowC1, self.yellowC2)

    def centercamfusion(self, campoint1, campoint2):
        if campoint1.size == 0:
            return np.array([self.originPoint[0], campoint2[0], campoint2[1]])
        elif campoint2.size == 0:
            return np.array([campoint1[0], self.originPoint[1], campoint1[1]])
        elif (campoint1.size == 2) and (campoint2.size == 2):
            return np.array([campoint1[0], campoint2[0], (campoint2[1] + campoint1[1]) / 2])
        else:
            return np.array([-1.0, -1.0, -1.0])

    def originhandler(self, campoint1, campoint2):
        if campoint1.size == 0:
            return np.array([campoint2[0], campoint2[0], campoint2[1]])
        elif campoint2.size == 0:
            return np.array([campoint1[0], campoint1[0], campoint1[1]])
        elif (campoint1.size == 2) and (campoint2.size == 2):
            return np.array([campoint1[0], campoint2[0], (campoint2[1] + campoint1[1]) / 2])
        else:
            return np.array([-1.0, -1.0, -1.0])


# call the class
def main(args):
    v2 = vision_2()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
    # run the code if the node is called


if __name__ == '__main__':
    main(sys.argv)
