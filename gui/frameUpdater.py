from threading import Thread, Event
from gi.repository import GLib
import gui.frameRenderer
import cv2
import time
import vision.cameras
import statics.configFile
import gui.mainWindow

class frameUpdater():
    
    def __init__(self, main_frame):
        # Private
        self.__running = False
        self.main_frame = main_frame
    
    def stop(self):
        self.__running = False
    
    def run(self):
        self.__running = True
        self.thread = Thread(target=self.__loop__)
        self.thread.start()
        
    def update_stats(self, processing_time, loop_time):
        gui.mainWindow.MainWindow().getObject("stats_label").set_text("Tempo de processamento: {:3.0f} ms\nTempo de loop: {:3.0f} ms".format(processing_time*1000, loop_time*1000))
    
    def __loop__(self):
        #frame_ = cv2.imread("gui/frame.png")
        
        while(self.__running):
            
            loop_time = time.time()
            
            frame = vision.cameras.uiCamerasList().getFrame()
            if frame is None:
                GLib.idle_add(self.main_frame.clear_image)
                time.sleep(0.03)
                continue

            fr = gui.mainWindow.MainWindow().selectedFrameRenderer
            if fr is None: continue
            
            #frame = frame_.copy()
            #frame_resized = cv2.resize(frame, (round(frame.shape[1]/frame.shape[0]*600),600))
            processing_time = time.time()
            frame_processed = fr.transformFrame(frame, frame)
            processing_time = time.time()-processing_time
            
            height, width, depth = frame_processed.shape
            GLib.idle_add(self.main_frame.do_update_frame, (frame_processed, width, height, depth))
            
            #time.sleep(0.03)
                
            loop_time = time.time()-loop_time
            GLib.idle_add(self.update_stats, processing_time, loop_time)
        
