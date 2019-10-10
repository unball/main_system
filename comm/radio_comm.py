from comm.communication import ROSCommunicator
from communication.msg import comm_msg
from speed_converter import speeds2motors
import roshandler.roshandler as rh
import statics.configFile
from controllers.ssRegulator import SpeedPair

import rospy

class RadioCommunicator(ROSCommunicator):
    def __init__(self, parent):
        ROSCommunicator.__init__(self, parent)
        
        self.rh = rh.RosHandler()
        self.rh.runProcess("roscore") # Asserts roscore is running

        rospy.init_node('Radio')
        self.pub = rospy.Publisher("radio_topic", comm_msg, queue_size=1)
        self.msg = comm_msg()

    def send(self, msg):
        self.rh.runProcess("roscore") # Asserts roscore is running
        self.rh.runProcess("radioSerial") # Asserts radio is listening
        
        for i in range(3):
            self.msg.MotorA[i], self.msg.MotorB[i] = speeds2motors(msg[i].v, msg[i].w)

        self.pub.publish(self.msg)
    
    def sendZero(self):
        self.send([SpeedPair() for i in range(3)])