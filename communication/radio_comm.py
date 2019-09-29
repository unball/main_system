from communication.communication import ROSCommunicator
from main_system.msg import robot_msg
from speed_converter import speeds2motors
import roshandler.roshandler as rh

import rospy

class RadioCommunicator(ROSCommunicator):
    def __init__(self):
        super().__init__()
        
        self.rh = rh.RosHandler()
        self.rh.runProcess("roscore") # Asserts roscore is running

        rospy.init_node('Radio')
        self.pub = rospy.Publisher("radio_topic", robot_msg, queue_size=1)
        self.msg = robot_msg()

    def send(self, msg):
        self.rh.runProcess("roscore") # Asserts roscore is running
        self.rh.runProcess("radioSerial") # Asserts radio is listening

        for i in range(3):
            self.msg.MotorA[i], self.msg.MotorB[i] = speeds2motors(msg[i].v, msg[i].w)
        self.pub.publish(self.msg)
    