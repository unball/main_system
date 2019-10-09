from abc import ABC, abstractmethod
import gui.mainWindow
import cv2

class frameRenderer(ABC):
	def __init__(self, parent, notebook_name):
		self._parent = parent
		self._notebook_name = notebook_name
		gui.mainWindow.MainWindow().add_frame_renderer(self, notebook_name)

	@abstractmethod
	def create_ui_content(self):
		pass

	@abstractmethod
	def create_ui_label(self):
		pass

	@abstractmethod
	def transformFrame(self, frame, originalFrame):
		pass
	
	@property
	def parent(self):
		return self._parent
	
	@property
	def gameThread(self):
		if self._parent is not None:
			return self._parent.gameThread
		return None

	def isFrameRendererSelected(self):
		return gui.mainWindow.MainWindow().selectedFrameRenderer(self._notebook_name) == self

class Identity(frameRenderer):

	def __init__(self):
		#super().__init__(None)
		pass
	
	def transformFrame(self, frame, originalFrame):
		return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
