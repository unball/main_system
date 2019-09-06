from abc import ABC, abstractmethod
import gui.mainWindow
import cv2

class frameRenderer(ABC):
	def __init__(self, vision):
		self._parentVision = vision
		gui.mainWindow.MainWindow().add_frame_renderer(self)

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
	def parentVision(self):
		return self._parentVision

class Identity(frameRenderer):

	def __init__(self):
		#super().__init__(None)
		pass
	
	def transformFrame(self, frame, originalFrame):
		return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
