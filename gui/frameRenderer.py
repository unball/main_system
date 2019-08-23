from abc import ABC, abstractmethod
from gi.repository import GLib
import gui.mainWindow
import cv2

class frameRenderer():
	def __init__(self, vision):
		self.parentVision = vision
		GLib.idle_add(gui.mainWindow.MainWindow().add_frame_renderer, self)

	@abstractmethod
	def create_ui_content(self):
		pass

	@abstractmethod
	def create_ui_label(self):
		pass

	@abstractmethod
	def transformFrame(self, frame, originalFrame):
		pass

class Identity(frameRenderer):

	def __init__(self):
		#super().__init__(None)
		pass
	
	def transformFrame(self, frame, originalFrame):
		return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
