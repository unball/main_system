from threading import Thread, Event
from gi.repository import Gtk, Gdk
import gui.signals
import gui.uiFrame
import gui.frameUpdater
import gui.singleton

class MainWindow(metaclass=gui.singleton.Singleton):

	def __init__(self):
		self.builder = None
		self.update_frame_thread = None
		self.selectedFrameRenderer = None
		self.frameRenderers = []

	def loadBuilder(self):
		if self.builder is None:
			self.builder = Gtk.Builder()
			self.builder.add_from_file("gui/ui.glade")
	
	def getObject(self, name):
		self.loadBuilder()
		return self.builder.get_object(name)

	def add_frame_renderer(self, fr):
		frNotebook = self.getObject("fr_notebook")
		frNotebook.append_page(fr.create_ui_content(), fr.create_ui_label())
		frNotebook.show_all()
		self.frameRenderers.append(fr)

	def set_frame_renderer(self, index):
		try:
			self.selectedFrameRenderer = self.frameRenderers[index]
		except:
			pass
	
	def run(self):
		# Load static UI
		self.loadBuilder()
		
		# Load CSS
		css_provider = Gtk.CssProvider()
		css_provider.load_from_path("gui/style.css")
		Gtk.StyleContext.add_provider_for_screen(Gdk.Screen.get_default(), css_provider, Gtk.STYLE_PROVIDER_PRIORITY_APPLICATION)
		
		# Connect signals
		self.builder.connect_signals(gui.signals.Signals())
		
		# Show window
		window = self.getObject("window1")
		window.show_all()
		
		# Create webcam thread that updates UI frame
		ui_frame = gui.uiFrame.uiFrame(self.getObject("frame"), self.getObject("frame_event"))
		self.update_frame_thread = gui.frameUpdater.frameUpdater(ui_frame)
		self.update_frame_thread.run()
		
		# Start GTK main loop
		Gtk.main()
		
		# Stops threads
		self.update_frame_thread.stop()
		
