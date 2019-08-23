#import rospy
from abc import ABC, abstractmethod

class ROSCommunicator:
    def __init__(self):
        pass

    @abstractmethod
    def send(self, msg):
        pass

    