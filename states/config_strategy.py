from states.state import State
from abc import ABC
import cv2
import time
import vision.cameras
import gui.mainWindow

from states.game_loop import GameLoop
from gui.guiMethod import guiMethod
from statics.static_classes import world

class ConfigStrategy(State):
    def __init__(self, thread):
        super().__init__(thread)
    
    def update(self):
        self.thread.visionSystem.update()
        world.calc_velocities(0.03)
        self.thread.strategySystem.plan()
        
        frame = self.thread.strategySystem.composeUIframe()
        
        gui.mainWindow.MainWindow().ui_frame("configStrategy").do_update_frame(frame)

if __name__ == "__main__":
    pass
