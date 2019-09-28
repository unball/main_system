from communication.communication import ROSCommunicator
from main_system.msg import robot_msg
from speed_converter import speeds2motors

import rospy

class RadioCommunicator(ROSCommunicator):
    def __init__(self):
        super().__init__()
        rospy.init_node('Radio')
        self.pub = rospy.Publisher("radio_topic", robot_msg, queue_size=1)
        self.msg = robot_msg()

    def send(self, msg):
        for i in range(3):
            self.msg.MotorA[i], self.msg.MotorB[i] = speeds2motors(msg[i][0][0], msg[i][1][0])
        self.pub.publish(self.msg)
    