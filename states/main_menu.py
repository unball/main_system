from states.state import State
from abc import ABC
import cv2
import time
import vision.cameras
import gui.mainWindow

from states.game_loop import GameLoop
from gui.guiMethod import guiMethod

class MainMenu(State):
    def __init__(self, thread):
        super().__init__(thread)

    def update(self):
        
        frame = vision.cameras.uiCamerasList().getFrame()
        if frame is None:
            gui.mainWindow.MainWindow().ui_frame("configVision").clear_image()
            time.sleep(0.03)
            return

        fr = gui.mainWindow.MainWindow().selectedFrameRenderer("fr_notebook")
        if fr is None: return
        
        #processing_time = time.time()
        frame_processed = fr.transformFrame(frame, frame)
        #processing_time = time.time()-processing_time
        
        gui.mainWindow.MainWindow().ui_frame("configVision").do_update_frame(frame_processed)
        

#    def next_state(self):
#        return GameLoop(self.thread)

if __name__ == "__main__":
    pass
