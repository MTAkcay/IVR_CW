import rospy
class sinusoidal_control1:
    def __init__(self):
        rospy.init_node("sinusoidal_control1", anonymous=True)
        self.joint2Pub = rospy.Publisher("/robot/joint2_position_controller/command", Float64, queue_size=10)
        self.joint3Pub = rospy.Publisher("/robot/joint3_position_controller/command" , Float64, queue_size=10)
        self.joint4Pub = rospy.Publisher("/robot/joint4_position_controller/command" , Float64, queue_size=10)
