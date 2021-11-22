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
        self.joint3Sub = rospy.Subscriber("joint_angle_3", Float64, self.callback1)
        self.joint4Sub = rospy.Subscriber("joint_angle_4", Float64, self.callback2)
        self.targetSub = rospy.Subscriber("/target_control/target_pos", Float64MultiArray, self.callback3)
        self.meterRedSub = rospy.Subscriber("meter_red", Float64MultiArray, self.callback4)
        self.fkPub = rospy.Publisher("fk_endpoint", Float64MultiArray, queue_size=10)
        self.joint1Pub = rospy.Publisher("/robot/joint1_position_controller/command", Float64, queue_size=10)
        self.joint3Pub = rospy.Publisher("/robot/joint3_position_controller/command", Float64, queue_size=10)
        self.joint4Pub = rospy.Publisher("/robot/joint4_position_controller/command", Float64, queue_size=10)
        self.effectorErrorPub = rospy.Publisher("effector_error", Float64MultiArray, queue_size=10)

        self.fk_predicted_pos = Float64MultiArray()
        self.joint1 = Float64()
        self.joint3 = Float64()
        self.joint4 = Float64()
        self.effectorError = Float64MultiArray()

        self.joints = [0.0, 0.0, 0.0]
        self.redCenter = [0.0, 0.0, 0.0]
        self.redCenter = [0.0, 0.0, 0.0]
        self.prev_Time = 0
        self.target = [0.0, 0.0, 0.0]

    def callback0(self, data):
        self.joints[0] = data
        self.publish_kinematic_endpoint()

    def callback1(self, data):
        # Receive the image
        self.joints[1] = data

    def callback2(self, data):
        # Receive the image
        self.joints[2] = data

    def callback3(self, data):
        # Receive the image
        self.target = data
        self.control_open()

    def callback4(self, data):
        # Receive the image
        self.redCenter = data

    def forward_kinematics(self, q1, q2, q3):
        return [7.8 * np.cos(q1) + 2.8 * (np.cos(q1) * np.cos(q3) - np.sin(q1) * np.sin(q2) * np.sin(q3)),
                2.8 * (np.cos(q1) * np.sin(q2) * np.sin(q3) - np.sin(q1) * np.cos(q3)) - 7.8 * np.sin(q1),
                -2.8 * np.cos(q2) * np.sin(q3)]

    def calculate_jacobian(self, q1, q2, q3):
        return [[-7.8 * np.sin(q1) + 2.8 * (-np.sin(q1) * np.cos(q3) - np.cos(q1) * np.sin(q2) * np.sin(q3)),
                2.8 * -np.sin(q1) * np.cos(q2) * np.sin(q3),
                2.8 * (-np.cos(q1) * np.sin(q3) - np.sin(q1) * np.sin(q2) * np.cos(q3))],
                [2.8 * (-np.sin(q1) * np.sin(q2) * np.sin(q3) - np.cos(q1) * np.cos(q3)) - 7.8 * np.cos(q1),
                 2.8 * np.cos(q1) * np.cos(q2) * np.sin(q3),
                 2.8 * (np.cos(q1) * np.sin(q2) * np.cos(q3) + np.sin(q1) * np.sin(q3))],
                [0,
                 2.8 * np.sin(q2) * np.sin(q3),
                 -2.8 * np.cos(q2) * np.cos(q3)]]

    def publish_kinematic_endpoint(self):
        q = self.joints
        self.fk_predicted_pos.data = self.forward_kinematics(q[0], q[1], q[2])
        try:
            self.fkPub.publish(self.fk_predicted_pos)
        except CvBridgeError as e:
            print(e)

    def control_open(self):
        currTime = rospy.get_time()
        dt = currTime - self.prev_Time
        self.prev_Time = currTime
        q = self.joints
        J_inv = np.linalg.pinv(self.calculate_jacobian(q[0], q[1], q[2]))
        posDesired = np.array(self.target)
        posEffector = np.array(self.redCenter)
        posDelta = posDesired - posEffector
        jointsDelta = dt * np.dot(J_inv, posDelta.T)

        finalAngle = np.array(self.joints) + jointsDelta
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
