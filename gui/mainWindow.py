from threading import Thread, Event
from gi.repository import Gtk, Gdk
import gui.signals
import gui.uiFrame
import gui.singleton
import states.gameThread
from gui.guiMethod import guiMethod

class MainWindow(metaclass=gui.singleton.Singleton):

	def __init__(self):
		self._builder = None
		self._selectedFrameRenderer = None
		self._frameRenderers = []
		self._ui_frame = None
		self._gameThread = None

	def loadBuilder(self):
		if self._builder is None:
			self._builder = Gtk.Builder()
			self._builder.add_from_file("gui/main.ui")
	
	def getObject(self, name):
		self.loadBuilder()
		return self._builder.get_object(name)
	
	@guiMethod
	def add_frame_renderer(self, fr):
		frNotebook = self.getObject("fr_notebook")
		frNotebook.append_page(fr.create_ui_content(), fr.create_ui_label())
		frNotebook.show_all()
		self._frameRenderers.append(fr)

	@guiMethod
	def set_frame_renderer(self, index):
		try:
			self._selectedFrameRenderer = self._frameRenderers[index]
		except:
			pass
	
	@property
	def selectedFrameRenderer(self):
		return self._selectedFrameRenderer
		
	def ui_frame(self, name):
		return self._ui_frames[name]
		
	@property
	def gameThread(self):
		return self._gameThread
	
	def run(self):
		# Load static UI
		self.loadBuilder()
		
		# Load CSS
		css_provider = Gtk.CssProvider()
		css_provider.load_from_path("gui/style.css")
		Gtk.StyleContext.add_provider_for_screen(Gdk.Screen.get_default(), css_provider, Gtk.STYLE_PROVIDER_PRIORITY_APPLICATION)
		
		# Connect signals
		self._builder.connect_signals(gui.signals.Signals())
		
		# Show window
		window = self.getObject("window1")
		window.show_all()
		
		# Creates uiFrame objects that handles all frame drawning
		self._ui_frames = {
			"configVision": gui.uiFrame.uiFrame(self.getObject("frame"), self.getObject("frame_event")),
			"configStrategy": gui.uiFrame.uiFrame(self.getObject("strategy_frame"), self.getObject("strategy_frame_event")),
			"gameLoop": gui.uiFrame.uiFrame(self.getObject("game_loop_frame"), self.getObject("game_loop_frame_event"))
		}
		
		# Creates game thread
		self._gameThread = states.gameThread.GameThread()
		self._gameThread.run()
		
		# Start GTK main loop
		Gtk.main()
		
		# Stops threads
		#self.update_frame_thread.stop()
		self._gameThread.stop()
		
