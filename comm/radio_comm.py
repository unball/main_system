from comm.communication import ROSCommunicator
from communication.msg import comm_msg
from speed_converter import speeds2motors
import roshandler.roshandler as rh
import comm.frameRenderer
import statics.configFile

import rospy

class RadioCommunicator(ROSCommunicator):
    def __init__(self, parent):
        ROSCommunicator.__init__(self, parent)
        
        self.rh = rh.RosHandler()
        self.rh.runProcess("roscore") # Asserts roscore is running

        rospy.init_node('Radio')
        self.pub = rospy.Publisher("radio_topic", comm_msg, queue_size=1)
        self.msg = comm_msg()

        self.lowCtrlParams = statics.configFile.getValue("lowCtrlParams", [[10, 10, 10, 10, 10, 10] for i in range(3)])

        self.fr = {
            "lowLevelCtrl": comm.frameRenderer.controleBaixoNivel(self)
        }
    
    def setLowCtrlParam(self, robot, index, value):
        self.lowCtrlParams[robot][index] = value
        statics.configFile.setValue("lowCtrlParams", self.lowCtrlParams)

    def send(self, msg):
        self.rh.runProcess("roscore") # Asserts roscore is running
        self.rh.runProcess("radioSerial") # Asserts radio is listening
        #print(self.lowCtrlParams)
        for i in range(3):
            self.msg.MotorA[i], self.msg.MotorB[i] = speeds2motors(msg[i].v, msg[i].w)
            # self.msg.Kp[i]   = self.lowCtrlParams[i][0]
            # self.msg.Kp[i+3] = self.lowCtrlParams[i][3]
            # self.msg.Ki[i]   = self.lowCtrlParams[i][1]
            # self.msg.Ki[i+3] = self.lowCtrlParams[i][4]
            # self.msg.Kd[i]   = self.lowCtrlParams[i][2]
            # self.msg.Kd[i+3] = self.lowCtrlParams[i][5]

        self.pub.publish(self.msg)
    