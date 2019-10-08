from gi.repository import Gtk
import statics.configFile
import numpy as np
import gui.mainWindow
import cv2
import gui.singleton
import time
import vision.mainVision
import gui.frameRenderer
from gui.guiMethod import guiMethod
from abc import ABC, abstractmethod

from pkg_resources import resource_filename

class controleBaixoNivel(gui.frameRenderer.frameRenderer):
	def __init__(self, parent):
		super().__init__(parent, "fr_comm_notebook")
		self.__n_robos = None
		self.__uiElements = []
	
	
	def transformFrame(self, frame, originalFrame):
		return None

	def create_ui_label(self):
		return Gtk.Label("Controle de baixo nível")

	def update_lowLevelParams(self, widget, idx):
		self.parent.setLowCtrlParam(0, idx, widget.get_value())
	
	def create_ui_content(self):
		builder = Gtk.Builder.new_from_file(resource_filename(__name__, "controleBaixoNivel.ui"))
		adjustments_names = ["adj_kpA", "adj_kiA", "adj_kdA", "adj_kpB", "adj_kiB", "adj_kdB"]
		for idx,adj_name in enumerate(adjustments_names):
			builder.get_object(adj_name).connect("value-changed", self.update_lowLevelParams, idx)
			builder.get_object(adj_name).set_value(self.parent.lowCtrlParams[0][idx])
		return builder.get_object("main")
