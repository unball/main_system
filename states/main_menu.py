from states.state import State
from abc import ABC
from gi.repository import GLib
import cv2
import time
import vision.cameras
import gui.mainWindow

from states.game_loop import GameLoop

class MainMenu(State):
    def __init__(self, thread):
        super().__init__(thread)
        self._loop_time = 0
        self._processing_time = 0
        
    def compute_average(self, v0, v, p=0.1):
        return v0*(1-p) + v*p
        
    def update_stats(self, processing_time, loop_time):
        self._loop_time = self.compute_average(self._loop_time, loop_time)
        self._processing_time = self.compute_average(self._processing_time, processing_time)
        gui.mainWindow.MainWindow().getObject("stats_label").set_text("Tempo de processamento: {:3.0f} ms\nTempo de loop: {:3.0f} ms".format(self._processing_time*1000, self._loop_time*1000))

    def update(self):
        loop_time = time.time()
        
        frame = vision.cameras.uiCamerasList().getFrame()
        if frame is None:
            GLib.idle_add(gui.mainWindow.MainWindow().ui_frame.clear_image)
            time.sleep(0.03)
            return

        fr = gui.mainWindow.MainWindow().selectedFrameRenderer
        if fr is None: return
        
        processing_time = time.time()
        frame_processed = fr.transformFrame(frame, frame)
        processing_time = time.time()-processing_time
        
        height, width, depth = frame_processed.shape
        GLib.idle_add(gui.mainWindow.MainWindow().ui_frame.do_update_frame, (frame_processed, width, height, depth))
            
        loop_time = time.time()-loop_time
        GLib.idle_add(self.update_stats, processing_time, loop_time)
        

#    def next_state(self):
#        return GameLoop(self.thread)

if __name__ == "__main__":
    pass
