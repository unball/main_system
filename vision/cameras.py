from os import listdir
import gui.singleton
from gi.repository import Gtk
import statics.configFile
import cv2

class uiCamerasList(metaclass=gui.singleton.Singleton):
    def __init__(self):
        self.cameras = set()
        self.__camera_changed = False
        self.cap = None
        
        # Load configuration file
        self.camera_index = statics.configFile.getValue("camera", 0)
        
        self.cap = cv2.VideoCapture(self.camera_index)
    
    def getCameras(self):
        return set(enumerate(sorted([c for c in listdir("/sys/class/video4linux/")])))

    def updateCameras(self, widget_list):
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
    
    def getFrame(self):
        frame_ = cv2.imread("gui/frame.png")
        return frame_
        
#        if self.__camera_changed:
#            self.__camera_changed = False
#            if self.cap is not None: self.cap.release()
#            self.cap = cv2.VideoCapture(self.camera_index)
#        
#        if self.cap.isOpened():
#            ret, frame = self.cap.read()
#            return frame
#        
#        return None
#            
