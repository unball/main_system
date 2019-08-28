from gi.repository import Gtk
import gui.mainWindow
import gui.frameRenderer
import vision.cameras
import interfaces.gamecommands

class Signals:
    def __init__(self):
        self.pageSelected = 0
        
    def onDestroy(self, *args):
        Gtk.main_quit()
        
    def list_cameras(self, widget_list):
        vision.cameras.uiCamerasList().updateCameras(widget_list)
        
    def select_camera(self, widget, widget_selected):
        vision.cameras.uiCamerasList().setCamera(widget_selected.index)
    
    def update_camera_scale(self, widget):
        vision.cameras.uiCamerasList().set_camera_scale(widget.get_value())
        
    def onConfigPageChange(self, page, widget, num):
        self.pageSelected = num
        gui.mainWindow.MainWindow().set_frame_renderer(num)
    
    def mainPageChange(self, widget, page):
        gameThread = gui.mainWindow.MainWindow().gameThread
        if gameThread is not None:
            gameThread.set_state(widget.get_stack().get_visible_child_name())
    
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
