from abc import ABC, abstractmethod
from statics.static_classes import world
import vision.cameras
import time
import gui.mainWindow
import cv2
import numpy as np
import strategy.frameRenderer
from states.system import System

class VisionMessage():
    def __init__(self, x_list, y_list, th_list, ball_x, ball_y, found_list):
        self.x = x_list
        self.y = y_list
        self.th = th_list
        self.ball_x = ball_x
        self.ball_y = ball_y
        self.found = found_list

class Vision(System, ABC):
    def __init__(self, parent):
        System.__init__(self, parent)
        self.config_init()
        self.ui_init()
    
    @abstractmethod
    def process(self, frame):
        pass
    
    def update(self):
        if world.manualMode: 
            time.sleep(0.03)
            return
        
        frame = vision.cameras.uiCamerasList().getFrame()
        if frame is None:
            time.sleep(0.03)
            return
        robosAliados, robosAdversariosIdentificados, bola, processed_image = self.process(frame)
        world.update(VisionMessage(
            [r[1][0] for r in robosAliados],
            [r[1][1] for r in robosAliados],
            [r[2]/180*np.pi for r in robosAliados],
            bola[0][0] if bola is not None else world.ball.pos[0], 
            bola[0][1] if bola is not None else world.ball.pos[1],
            [r[3] for r in robosAliados]
        ))
        
        converted_image = cv2.cvtColor(processed_image, cv2.COLOR_RGB2BGR)
        strategy_frame = strategy.frameRenderer.strategyFrame((350,471), step=gui.mainWindow.MainWindow().gameThread.strategySystem.step)
        gui.mainWindow.MainWindow().ui_frame("gameLoop").do_update_frame(strategy_frame)
    
    @abstractmethod
    def ui_init(self):
        pass
    
    @abstractmethod
    def config_init(self):
        pass
    
    @abstractmethod
    def ui_process(self):
        pass
