import cv2
import numpy as np
import roslib
import sys
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64
class vision_1:
    def __init__(self):
        rospy.init_node("vision_1", anonymous=True)
        
        # initialize the bridge between openCV and ROS
        self.bridge = CvBridge()
        self.image_sub1 = rospy.Subscriber("/camera1/robot/image_raw",Image,self.callback1)
        self.image_sub2 = rospy.Subscriber("/camera2/robot/image_raw",Image,self.callback2)
        self.joint2Pub = rospy.Publisher("joint_angle_2", Float64, queue_size=10)
        self.joint3Pub = rospy.Publisher("joint_angle_3", Float64, queue_size=10)
        self.joint4Pub = rospy.Publisher("joint_angle_4", Float64, queue_size=10)
        self.joint2 = Float64()
        self.joint3 = Float64()
        self.joint4 = Float64()

    def getCentre(self, mask):
        control = sum(sum(mask))
        if control < 10:
            return -1
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
            self.joint2Pub.publish(self.joint2)
            self.joint3Pub.publish(self.joint3)
            self.joint4Pub.publish(self.joint4)
        except CvBridgeError as e:
            print(e)

    def determinejointangles(self):
        self.joint2.data = np.arctan2()
        self.joint3.data = np.arctan2()
        self.joint4.data = np.arctan2()

    def combinecenters(self):
        self.originPoint = self.originhandler(self.greenC1, self.greenC2)
        self.finalRedCenter = self.centercamfusion(self.redC1, self.redC2)
        self.finalBlueCenter = self.centercamfusion(self.blueC1, self.blueC2)
        self.finalYellowCenter = self.centercamfusion(self.yellowC1, self.yellowC2)

    def centercamfusion(self, campoint1, campoint2):
        if campoint1 == -1:
            return [self.originPoint[0], campoint2[0], campoint2[1]]
        elif campoint2 == -1:
            return [campoint1[0], self.originPoint[1], campoint1[1]]
        elif (campoint1 != -1) and (campoint2 != -1):
            return [campoint1[0],campoint2[0], (campoint2[1] + campoint1[1]) / 2]

    def originhandler(self, campoint1, campoint2):
        if campoint1 == -1:
            return [campoint2[0], campoint2[0], campoint2[1]]
        elif campoint2 == -1:
            return [campoint1[0], campoint1[0], campoint1[1]]
        elif (campoint1 != -1) and (campoint2 != -1):
            return [campoint1[0],campoint2[0], (campoint2[1] + campoint1[1]) / 2]

# call the class
def main(args):
    v1 = vision_1()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
    # run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)





