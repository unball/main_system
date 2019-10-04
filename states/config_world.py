from states.state import State
from abc import ABC
import cv2
import time
import vision.cameras
import gui.mainWindow

from states.game_loop import GameLoop
from gui.guiMethod import guiMethod
from statics.static_classes import world

import vision.vision

class ConfigWorld(State):
    def __init__(self, thread):
        super().__init__(thread)
    
    def update(self):
        self.thread.visionSystem.update()
        
#        world.update(vision.vision.VisionMessage(
#            [0,0,0],
#            [0,0,0],
#            [0,0,0],
#            0, 
#            0,
#            [True, True, True]
#        ))
        #time.sleep(0.03)
        
        self.thread.strategySystem.plan()
        
        fr = gui.mainWindow.MainWindow().selectedFrameRenderer("fr_world_notebook")
        if fr is None: return
        
        frame_processed = fr.transformFrame(None, None)
        
        gui.mainWindow.MainWindow().ui_frame("configWorld").do_update_frame(frame_processed)
        
        

if __name__ == "__main__":
    pass
