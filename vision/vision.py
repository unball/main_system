from abc import ABC, abstractmethod
from statics.static_classes import world
import vision.cameras
import time

class VisionMessage():
    def __init__(self, x_list, y_list, th_list, ball_x, ball_y, found_list):
        self.x = x_list
        self.y = y_list
        self.th = th_list
        self.ball_x = ball_x
        self.ball_y = ball_y
        self.found = found_list

class Vision():
    def __init__(self):
        self.config_init()
        self.ui_init()
    
    @abstractmethod
    def process(self):
        pass
    
    @abstractmethod
    def update(self):
        frame = vision.cameras.uiCamerasList().getFrame()
        if frame is None:
            time.sleep(0.03)
            return
        robosAliados, robosAdversariosIdentificados, bola = self.process(frame)
#        world.update(VisionMessage(
#            [r[1][0] for r in robosAliados],
#            [r[1][1] for r in robosAliados],
#            [r[2] for r in robosAliados],
#            bola[0][0] if bola is not None else None, 
#            bola[0][1] if bola is not None else None,
#            [r[3] for r in robosAliados]
#        ))
    
    @abstractmethod
    def ui_init(self):
        pass
    
    @abstractmethod
    def config_init(self):
        pass
    
    @abstractmethod
    def ui_process(self):
        pass
