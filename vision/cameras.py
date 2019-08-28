import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk
from os import listdir
import gui.singleton
import statics.configFile
import cv2
import time
import gui.mainWindow

class uiCamerasList(metaclass=gui.singleton.Singleton):
    def __init__(self):
        self.cameras = set()
        self.__camera_changed = False
        self.cap = None
        self.frame = cv2.imread("gui/frame.png")
        
        # Load configuration file
        self.camera_index = statics.configFile.getValue("camera", 0)
        self.frame_scale = statics.configFile.getValue("frame_scale", 1)
        if self.frame_scale <= 0: self.set_camera_scale(1)
        
        self.cap = cv2.VideoCapture(self.camera_index)
    
    def getCameras(self):
        return set(enumerate(sorted([c for c in listdir("/sys/class/video4linux/")])))

    def updateCameras(self, widget_list):
        gui.mainWindow.MainWindow().getObject("camera_scale").set_value(self.frame_scale)
        for camera in sorted([c for c in self.getCameras().difference(self.cameras)]):
            self.cameras.add(camera)
            row = Gtk.ListBoxRow()
            row.index = camera[0]
            row.add(Gtk.Label(camera[1]))
            row.set_size_request(150,30)
            widget_list.insert(row,-1)
            widget_list.show_all()
        
    def setCamera(self, index):
        if self.camera_index == index: return
        self.camera_index = index
        self.__camera_changed = True
        statics.configFile.setValue("camera", self.camera_index)
    
    def set_camera_scale(self, value):
        if value <= 0: return
        self.frame_scale = value
        statics.configFile.setValue("frame_scale", value)
    
    def getFrame(self):
        time.sleep(0.0001)
        frame_resized = cv2.resize(self.frame, (round(self.frame.shape[1]*self.frame_scale),round(self.frame.shape[0]*self.frame_scale)))
        return frame_resized
        
#        if self.__camera_changed:
#            self.__camera_changed = False
#            if self.cap is not None: self.cap.release()
#            self.cap = cv2.VideoCapture(self.camera_index)
#        
#        if self.cap.isOpened():
#            ret, frame = self.cap.read()
#            frame_resized = cv2.resize(frame, (round(frame.shape[1]*self.frame_scale),round(frame.shape[0]*self.frame_scale)))
#            return frame_resized
#        
#        return None
#            
