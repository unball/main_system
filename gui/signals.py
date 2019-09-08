from gi.repository import Gtk
import gui.mainWindow
import gui.frameRenderer
import vision.cameras
import interfaces.gamecommands
from statics.static_classes import world
from statics.world_standards import *

class Signals:
    def __init__(self):
        pass
        
    def onDestroy(self, *args):
        Gtk.main_quit()
        
    def list_cameras(self, widget_list):
        vision.cameras.uiCamerasList().updateCameras(widget_list)
    
    def select_world_standard(self, widget, widget_selected):
        index = widget_selected.get_index()
        if index == 0:
            gui.mainWindow.MainWindow().gameThread.addEvent(world.setSetting, STANDARD5)
            #world.setSetting(STANDARD5)
        elif index == 1:
            gui.mainWindow.MainWindow().gameThread.addEvent(world.setSetting, STANDARD3)
            #world.setSetting(STANDARD3)
    
    def select_camera(self, widget, widget_selected):
        vision.cameras.uiCamerasList().setCamera(widget_selected.index)
    
    def update_camera_scale(self, widget):
        vision.cameras.uiCamerasList().set_camera_scale(widget.get_value())
    
    def camera_switch_test_frame(self, widget, value):
        vision.cameras.uiCamerasList().use_test_frame(value)
        
    def onConfigPageChange(self, page, widget, num):
        gui.mainWindow.MainWindow().set_frame_renderer(num)
    
    def mainPageChange(self, stack):
        gameThread = gui.mainWindow.MainWindow().gameThread
        if gameThread is not None:
            gameThread.set_state(stack.get_visible_child_name())
    
    def gameCommands_IniciodeJogo(self, widget):
        interfaces.gamecommands.gameCommands().IniciodeJogo()
    
    def gameCommands_mudarTexto(self, widget):
        interfaces.gamecommands.gameCommands().mudarTexto()
    
    def gameCommands_ally(self, widget):
        interfaces.gamecommands.gameCommands().ally()
    
    def gameCommands_enemy(self, widget):
        interfaces.gamecommands.gameCommands().enemy()
    
    def gameCommands_left(self, widget):
        interfaces.gamecommands.gameCommands().left()
    
    def gameCommands_right(self, widget):
        interfaces.gamecommands.gameCommands().right()
