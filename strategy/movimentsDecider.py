import dubins 
import numpy as np 
from abc import ABC, abstractmethod

class Robot():
    def __init__(self):
        self.__pose = np.array([0,0,0])        
        
class RobotOwn(Robot):
    def __init__(self):
        super().__init__(self)
        self.__target =  np.array([0,0,0])
        self.__trajectory = [] 

class Entity(ABC):
    def __init__(self):
        self.host = None
        self.__target = (0,0,0)
        self.__path = None
    def possession(id):
        pass
    @abstractmethod
    def tatic():
        pass
#update possession

class Attacker(Entity):
    def __init__():
        super().__init__()
    def tatic():
        pass

class Goalkeeper(Entity):
    def __init__():
        super().__init__()
    def tatic():
        pass

class Defender(Entity):
    def __init__():
        super().__init__()
    def tatic():
        pass

class Midfielder(Entity):
    def __init__():
        super().__init__()
    def tatic():
        pass

class MovimentsDecider():
    def __init__():
        pass

    def calcTragectories(q0,q1, turning_radius):
        path = dubins.shortest_path(q0, q1, turning_radius)
        return path

