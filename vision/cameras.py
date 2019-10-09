import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk
from os import listdir
import gui.singleton
import statics.configFile
import cv2
import time
import gui.mainWindow
from gui.guiMethod import guiMethod
from pkg_resources import resource_filename

class uiCamerasList(metaclass=gui.singleton.Singleton):
    def __init__(self):
        self.__cameras = set()
        self.__camera_changed = False
        self.__cap = None
        self.__frame = cv2.imread(resource_filename(__name__, "../gui/frame.png"))
        
        # Load configuration file
        self.__camera_index = statics.configFile.getValue("camera", 0)
        self.__frame_scale = statics.configFile.getValue("frame_scale", 1)
        self.__use_test_frame = statics.configFile.getValue("use_test_frame", True)
        
        if self.__frame_scale <= 0: self.set_camera_scale(1)

        self.__cap = cv2.VideoCapture(self.__camera_index)
        
        self.ui_config()
    
    @guiMethod
    def ui_config(self):
        gui.mainWindow.MainWindow().getObject("test_frame_switch").set_state(self.__use_test_frame)
        gui.mainWindow.MainWindow().getObject("camera_scale").set_value(self.__frame_scale)
        
    
    def getCameras(self):
        return set([c for c in listdir("/sys/class/video4linux/")])

    def updateCameras(self, widget_list):
        for camera in self.getCameras().difference(self.__cameras):
            self.__cameras.add(camera)
            row = Gtk.ListBoxRow()
            row.index = int(camera.split("video")[1])
            row.add(Gtk.Label(camera))
            row.set_size_request(150,30)
            widget_list.insert(row,-1)
            widget_list.show_all()
        
    def setCamera(self, index):
        gui.mainWindow.MainWindow().getObject("test_frame_switch").set_state(False)
        if self.__camera_index == index: return
        self.__camera_index = index
        self.__camera_changed = True
        statics.configFile.setValue("camera", self.__camera_index)
    
    def set_camera_scale(self, value):
        if value <= 0: return
        self.__frame_scale = value
        statics.configFile.setValue("frame_scale", value)
    
    def use_test_frame(self, value):
        self.__use_test_frame = value
        if self.__use_test_frame is True and self.__cap is not None:
            self.__cap.release()
        elif self.__use_test_frame is False:
            self.__camera_changed = True
        statics.configFile.setValue("use_test_frame", value)
    
    def getFrame(self):
        if self.__use_test_frame:
            time.sleep(0.001)
            frame_resized = cv2.resize(self.__frame, (round(self.__frame.shape[1]*self.__frame_scale),round(self.__frame.shape[0]*self.__frame_scale)))
            return frame_resized
        
        if self.__camera_changed:
            self.__camera_changed = False
            if self.__cap is not None: self.__cap.release()
            self.__cap = cv2.VideoCapture(self.__camera_index)
        
        if self.__cap.isOpened():
            ret, frame = self.__cap.read()
            frame_resized = cv2.resize(frame, (round(frame.shape[1]*self.__frame_scale),round(frame.shape[0]*self.__frame_scale)))
            return frame_resized
        
        return None
            
