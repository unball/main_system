#import rospy
from abc import ABC, abstractmethod
from states.system import System

class ROSCommunicator(System):
    def __init__(self, parent):
        System.__init__(self, parent)

    @abstractmethod
    def send(self, msg):
        pass

    