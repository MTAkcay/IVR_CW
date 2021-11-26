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
import time


class control:
    def __init__(self):
        rospy.init_node("control", anonymous=True)
        self.prevTime = rospy.get_time()

        # self.joints = [0.0, 0.0]
        self.joints = [0.0, 0.0, 0.0]
        self.redCenter = [0.0, 0.0, 0.0]
        # self.blueCenter = [0.0, 0.0, 0.0]
        self.target = [0.0, 0.0, 0.0]
        self.generate_seed_locations()

        self.fk_predicted_pos = Float64MultiArray()
        self.joint1 = Float64()
        self.joint3 = Float64()
        self.joint4 = Float64()
        self.effectorError = Float64MultiArray()

        self.fkPub = rospy.Publisher("fk_endpoint", Float64MultiArray, queue_size=10)
        self.joint1Pub = rospy.Publisher("/robot/joint1_position_controller/command", Float64, queue_size=10)
        self.joint3Pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
        self.joint4Pub = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=10)
        self.effectorErrorPub = rospy.Publisher("effector_error", Float64MultiArray, queue_size=10)

        self.joint1Sub = rospy.Subscriber("joint_angle_1", Float64, self.callback0)
        self.joint3Sub = rospy.Subscriber("joint_angle_3", Float64, self.callback1)
        self.joint4Sub = rospy.Subscriber("joint_angle_4", Float64, self.callback2)
        self.meterRedSub = rospy.Subscriber("meter_red", Float64MultiArray, self.callback4)
        # self.meterBlueSub = rospy.Subscriber("meter_blue", Float64MultiArray, self.callback5)
        self.targetSub = rospy.Subscriber("/target_control/target_pos", Float64MultiArray, self.callback3)


    def callback0(self, data):
        self.joints[0] = data.data
        # self.publish_kinematic_endpoint()

    def callback1(self, data):
        # Receive the image
        self.joints[1] = data.data

    def callback2(self, data):
        # Receive the image
        self.joints[2] = data.data

    def callback3(self, data):
        # Receive the image
        self.target = data.data
        self.control_open()

    def callback4(self, data):
        # Receive the image
        self.redCenter = data.data

    def forward_kinematics(self, q1, q2, q3):
        c1 = np.cos(q1)
        s1 = np.sin(q1)
        c2 = np.cos(q2)
        s2 = np.sin(q2)
        c3 = np.cos(q3)
        s3 = np.sin(q3)

        return [3.2 * s1 * s2 + 2.8 * (c1 * s3 + s1 * s2 * c3),
                2.8 * (s1 * s3 - c1 * s2 * c3) - 3.2 * c1 * s2,
                2.8 * c2 * c3 + 3.2 * c2 + 4]

    def calculate_jacobian(self, q1, q2, q3):
        c1 = np.cos(q1)
        s1 = np.sin(q1)
        c2 = np.cos(q2)
        s2 = np.sin(q2)
        c3 = np.cos(q3)
        s3 = np.sin(q3)

        return np.array([[3.2*c1*s2 - 2.8*s1*s3 + 2.8*c1*s2*c3,
                          3.2*s1*c2 + 2.8*s1*c2*c3,
                          2.8*c1*c3 - 2.8*s1*s2*s3],
                         [2.8*c1*s3 + 2.8*s1*s2*c3 + 3.2*s1*s2,
                          -2.8*c1*c2*c3 - 3.2*c1*c2,
                          2.8*s1*c3 + 2.8*c1*s2*s3],
                         [0,
                          -2.8*s2*c3 - 3.2*s2,
                          -2.8*c2*s3]])

    def publish_kinematic_endpoint(self):
        q = self.joints
        self.fk_predicted_pos.data = self.forward_kinematics(q[0], q[1], q[2])
        # print()
        # print("Angles Detected:", q)
        # print("Vision End Effector Position:", self.redCenter)
        # print("Forward Kinematics End Effector Position:", self.fk_predicted_pos.data)
        try:
            self.fkPub.publish(self.fk_predicted_pos)
        except CvBridgeError as e:
            print(e)

    def control_open(self):
        currTime = rospy.get_time()
        dt = currTime - self.prevTime
        self.prevTime = currTime
        q = self.joints
        J_inv = np.linalg.pinv(self.calculate_jacobian(q[0], q[1], q[2]))
        posDesired = np.array(self.target)
        posEffector = np.array(self.redCenter)
        posDelta = posDesired - posEffector
        jointsDelta = dt*np.dot(J_inv, posDelta.T)

        finalAngle = np.array(self.joints) + jointsDelta

        legal = self.isLegal(finalAngle)
        if (not legal):
            self.failureIndex = (self.failureIndex + 1) % len(self.seeds)
            self.pickSeed()
            self.publish_angles_and_error()
            rospy.sleep(0.2)


        self.joint1.data = finalAngle[0]
        self.joint3.data = finalAngle[1]
        self.joint4.data = finalAngle[2]

        self.effectorError.data = posDelta.tolist()
        self.publish_angles_and_error()

    def publish_angles_and_error(self):
        # Publish the results
        try:
            self.joint1Pub.publish(self.joint1)
            self.joint3Pub.publish(self.joint3)
            self.joint4Pub.publish(self.joint4)
            self.effectorErrorPub.publish(self.effectorError)
        except CvBridgeError as e:
            print(e)

    def generate_seed_locations(self):
        self.failureIndex = 0
        self.seeds = []
        for i in np.linspace(-np.pi, np.pi, 4 ):
            for j in np.linspace(-np.pi/2, np.pi/2, 4):
                for k in np.linspace(-np.pi / 2, np.pi / 2, 4):
                    self.seeds.append([i, j, k])

    def isLegal(self, finalAngle):
        if abs(finalAngle[0]) >= np.pi:
            return False
        if abs(finalAngle[1]) >= np.pi/2:
            return False
        if abs(finalAngle[2]) >= np.pi/2:
            return False
        return True

    def pickSeed(self):
        seed = self.seeds[self.failureIndex]
        self.joint1.data = seed[0]
        self.joint3.data = seed[1]
        self.joint4.data = seed[2]


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
