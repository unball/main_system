import gui.mainWindow
from gui.guiMethod import guiMethod
from threading import Timer
from gi.repository import Gtk, Gdk

class uiLogger:
    def __init__(self):
        self._last_log_message = 0
        self._error_log_count = 0
        self._new_error = True
        
        self.revealer = gui.mainWindow.MainWindow().getObject("log_revealer")
        self.revealerImage = gui.mainWindow.MainWindow().getObject("log_revealer_image")
        self.logLabel = gui.mainWindow.MainWindow().getObject("log_system_label")
        self.logBuffer = gui.mainWindow.MainWindow().getObject("log_system").get_buffer()
        self.logButton = gui.mainWindow.MainWindow().getObject("log_button")
        
        self.logButton.connect("toggled", self.log_toggled)
        
    def log_toggled(self, widget):
        if self._new_error == False:
            revealer = gui.mainWindow.MainWindow().getObject("log_revealer").set_reveal_child(False)
            t = Timer(0.25, self.timeout2)
            t.start()
        self._new_error = True
        self.logButton.modify_fg(Gtk.StateFlags.NORMAL, Gdk.color_parse("black"))
        
    def timeout2(self):
        revealerImage = self.revealerImage.set_from_icon_name("emblem-ok-symbolic", Gtk.IconSize.MENU)
        revealer = self.revealer.set_reveal_child(True)
    
    @guiMethod
    def timeout(self):
        self.revealerImage.set_from_icon_name("dialog-error-symbolic", Gtk.IconSize.MENU)
        revealer = self.revealer.set_reveal_child(True)
        
    @guiMethod
    def logErrorMessage(self, message):
        if self._new_error:
            self._new_error = False
            revealer = self.revealer.set_reveal_child(False)
            t = Timer(0.25, self.timeout)
            t.start()
            
        self._error_log_count += 1
        self.logLabel.set_text("Log: (Erros {0})".format(self._error_log_count))
        
        if self._last_log_message == message:
            return
        
        self.logButton.modify_fg(Gtk.StateFlags.NORMAL, Gdk.color_parse("red"))
        
        end = self.logBuffer.get_end_iter()
        self.logBuffer.insert(end, message)
        
        self._last_log_message = message
