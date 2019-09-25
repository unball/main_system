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

class clickedPoints(ABC):
	def __init__(self, configFileVariableName, maxSize):
		self._points = statics.configFile.getValue(configFileVariableName, [])
		self.__max_size = maxSize
		self.__config_file_variable_name = configFileVariableName
	
	def append(self, point):
		if len(self._points) >= self.__max_size:
			self._points.clear()
			
		if len(self._points) < self.__max_size:
			self._points.append(point)
			
		if len(self._points) == self.__max_size:
			statics.configFile.setValue(self.__config_file_variable_name, self._points)
	
	def isFilled(self):
		return len(self._points) == self.__max_size
		
	def length(self):
		return len(self._points)
	
	@abstractmethod
	def getSorted(self):
		pass
	
	def getPointPixels(self, index, shape):
		return (round(self._points[index][0]*shape[0]), round(self._points[index][1]*shape[1]))

class homographyPoints(clickedPoints):
	def __init__(self, configFileVariableName, maxSize):
		super().__init__(configFileVariableName, maxSize)
	
	def getSorted(self):
		points = self._points.copy()
		if len(points) == 4:
			points.sort(key=sum)
			if points[1][0] > points[2][0]:
				tmp = points[1]
				points[1] = points[2]
				points[2] = tmp
		return points

class cropPoints(clickedPoints):
	def __init__(self, configFileVariableName, maxSize):
		super().__init__(configFileVariableName, maxSize)
	
	def getSorted(self):
		points = self._points.copy()
		if len(points) == 2:
			x1,y1 = points[0]
			x2,y2 = points[1]
			xmin = min([x1,x2])
			xmax = max([x1,x2])
			ymin = min([y1,y2])
			ymax = max([y1,y2])
			return [(xmin, ymin), (xmax, ymax)]

class cortarCampo(gui.frameRenderer.frameRenderer):
	
	def __init__(self, vision):
		super().__init__(vision, "fr_notebook")
		# Variables
		self.__pointer_position = None
		self.__show_warpped = False
		self.__frame_shape = None
		
		self.__points = homographyPoints("points", 4)
		self.__crop_points = cropPoints("cortarCampo_crop_points", 2)
	
	def update_points(self, point):
		if self.__show_warpped: return
		
		if self.parent.use_homography:
			self.__points.append([point[0]/self.__frame_shape[0], point[1]/self.__frame_shape[1]])
			
			if self.__points.isFilled():
				self.parent.updateHomography(self.__points.getSorted(), self.__frame_shape)
		
		else:
			self.__crop_points.append([point[0]/self.__frame_shape[0], point[1]/self.__frame_shape[1]])
			
			if self.__crop_points.isFilled():
				self.parent.updateCropPoints(self.__crop_points.getSorted())
			
	
	def set_pointer_position(self, position):
		self.__pointer_position = position
	
	def set_show_mode(self, widget, value):
		self.__show_warpped = value
	
	def set_crop_mode(self, widget, value):
		self.parent.setUseHomography(value)
	
	def get_point_pixels(self, index):
		return self.__points.getPointPixels(index, self.__frame_shape)
		
	def get_crop_point_pixels(self, index):
		return self.__crop_points.getPointPixels(index, self.__frame_shape)
		
	def transformFrame(self, frame, originalFrame):
		self.__frame_shape = frame.shape
		
		if(self.__show_warpped):
			return cv2.cvtColor(self.parent.warp(frame), cv2.COLOR_RGB2BGR)
		
		if not self.parent.use_homography:
			if self.__crop_points.length() == 1:
				p0 = self.get_crop_point_pixels(0)
				cv2.circle(frame, p0, 5, (255,255,255), thickness=-1)
				cv2.rectangle(frame, p0, (self.__pointer_position[0], self.__pointer_position[1]), (255,255,255))
			elif self.__crop_points.length() == 2:
				p0 = self.get_crop_point_pixels(0)
				p1 = self.get_crop_point_pixels(1)
				cv2.circle(frame, p0, 5, (0,255,0), thickness=-1)
				cv2.circle(frame, p1, 5, (0,255,0), thickness=-1)
				cv2.rectangle(frame, p0, p1, (0,255,0), thickness=2)
			return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
		
		color = (0,255,0) if self.__points.length() == 4 else (255,255,255)
		
		# Draw line for each pair of points
		if self.__points.length() > 1:
			for i in range(self.__points.length()-1):
				cv2.line(frame, self.get_point_pixels(i), self.get_point_pixels(i+1), color, thickness=2)
		
		# Closes rectangle
		if self.__points.length() == 4:
			cv2.line(frame, self.get_point_pixels(0), self.get_point_pixels(3), color, thickness=2)
		
		# Draw helping line from last chosen point to current mouse position
		if self.__pointer_position and self.__points.length() > 0 and self.__points.length() < 4:
			cv2.line(frame, self.get_point_pixels(-1), (self.__pointer_position[0], self.__pointer_position[1]), (255,255,255))
			# Draw extra line from first chosen point to current position when it's the last point to be chosen
			if self.__points.length() == 3:
				cv2.line(frame, self.get_point_pixels(0), (self.__pointer_position[0], self.__pointer_position[1]), color)
		
		for i in range(self.__points.length()):
			cv2.circle(frame, self.get_point_pixels(i), 5, color, thickness=-1)
		
		return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
		
	def cortarCampo_update_points(self, widget, event):
		if gui.mainWindow.MainWindow().selectedFrameRenderer("fr_notebook") == self:
			self.update_points([int(event.x), int(event.y)])
	
	def cortarCampo_mouseOver(self, widget, event):
		if gui.mainWindow.MainWindow().selectedFrameRenderer("fr_notebook") == self:
			self.set_pointer_position((int(event.x), int(event.y)))

	def create_ui_label(self):
		return Gtk.Label("Cortar Campo")
	
	def create_ui_content(self):
		builder = Gtk.Builder.new_from_file("vision/mainVision/cortarCampo.ui")
		builder.get_object("campo_switch").connect("state-set", self.set_show_mode)
		builder.get_object("homografia_switch").connect("state-set", self.set_crop_mode)
		builder.get_object("homografia_switch").set_state(self.parent.use_homography)
		gui.mainWindow.MainWindow().getObject("frame_event").connect("button-press-event", self.cortarCampo_update_points)
		gui.mainWindow.MainWindow().getObject("frame_event").connect("motion-notify-event", self.cortarCampo_mouseOver)

		return builder.get_object("main")
		

class segmentarPreto(gui.frameRenderer.frameRenderer):

	def __init__(self, vision):
		super().__init__(vision, "fr_notebook")
		# Variables
		self.__ui_elements = ["fundo_hmin", "fundo_smin", "fundo_vmin", "fundo_hmax", "fundo_smax", "fundo_vmax"]
		
	def update_hsv_interval(self, widget, index):
		self.parent.atualizarPretoHSV(int(widget.get_value()), index)
	
	def transformFrame(self, frame, originalFrame):
		img_warped = self.parent.warp(frame)
		return cv2.cvtColor(self.parent.segmentarFundo(img_warped), cv2.COLOR_GRAY2RGB)

	def create_ui_label(self):
		return Gtk.Label("Segmentar Preto")
	
	def create_ui_content(self):
		builder = Gtk.Builder.new_from_file("vision/mainVision/segmentarPreto.ui")
		for index,name in enumerate(self.__ui_elements):
			element = builder.get_object(name)
			element.set_value(self.parent.preto_hsv[index])
			element.connect("value-changed", self.update_hsv_interval, index)
		
		return builder.get_object("main")

class segmentarTime(gui.frameRenderer.frameRenderer):
	def __init__(self, vision):
		super().__init__(vision, "fr_notebook")
		# Variables
		self.__ui_elements = ["time_hmin", "time_smin", "time_vmin", "time_hmax", "time_smax", "time_vmax"]

	def update_hsv_interval(self, widget, index):
		self.parent.atualizarTimeHSV(int(widget.get_value()), index)
	
	def transformFrame(self, frame, originalFrame):
		img_warped = self.parent.warp(frame)
		return cv2.cvtColor(self.parent.segmentarTime(img_warped), cv2.COLOR_GRAY2RGB)

	def create_ui_label(self):
		return Gtk.Label("Segmentar Time")
	
	def create_ui_content(self):
		builder = Gtk.Builder.new_from_file("vision/mainVision/segmentarTime.ui")
		for index,name in enumerate(self.__ui_elements):
			element = builder.get_object(name)
			element.set_value(self.parent.time_hsv[index])
			element.connect("value-changed", self.update_hsv_interval, index)
		
		return builder.get_object("main")

class segmentarBola(gui.frameRenderer.frameRenderer):
	def __init__(self, vision):
		super().__init__(vision, "fr_notebook")
		# Variables
		self.__ui_elements = ["bola_hmin", "bola_smin", "bola_vmin", "bola_hmax", "bola_smax", "bola_vmax"]
	
	def update_hsv_interval(self, widget, index):
		self.parent.atualizarBolaHSV(int(widget.get_value()), index)
	
	def transformFrame(self, frame, originalFrame):
		img_warped = self.parent.warp(frame)
		return cv2.cvtColor(self.parent.segmentarBola(img_warped), cv2.COLOR_GRAY2RGB)

	def create_ui_label(self):
		return Gtk.Label("Segmentar Bola")
		
	def create_ui_content(self):
		builder = Gtk.Builder.new_from_file("vision/mainVision/segmentarBola.ui")
		for index,name in enumerate(self.__ui_elements):
			element = builder.get_object(name)
			element.set_value(self.parent.bola_hsv[index])
			element.connect("value-changed", self.update_hsv_interval, index)
		
		return builder.get_object("main")
		
class parametrosVisao(gui.frameRenderer.frameRenderer):

	def __init__(self, vision):
		super().__init__(vision, "fr_notebook")
		
	def transformFrame(self, frame, originalFrame):
		processed_frame = self.parent.ui_process(frame)
		return cv2.cvtColor(processed_frame, cv2.COLOR_RGB2BGR)

	def create_ui_label(self):
		return Gtk.Label("Parâmetros da visão")
	
	def update_area_ratio(self, widget):
		self.parent.atualizarAreaRatio(widget.get_value())
	
	def update_min_internal_area_contour(self, widget):
		self.parent.atualizarMinInternalArea(widget.get_value())
	
	def update_stability_param(self, widget):
		self.parent.atualizarParametroEstabilidade(widget.get_value())
	
	def create_ui_content(self):
		builder = Gtk.Builder.new_from_file("vision/mainVision/parametrosVisao.ui")
		builder.get_object("adj_area_cont_rect").set_value(self.parent.areaRatio)
		builder.get_object("adj_area_cont_rect").connect("value-changed", self.update_area_ratio)
		builder.get_object("adj_min_area_internal_contour").set_value(self.parent.minInternalAreaContour)
		builder.get_object("adj_min_area_internal_contour").connect("value-changed", self.update_min_internal_area_contour)
		builder.get_object("adj_stability_param").set_value(self.parent.stabilityParam)
		builder.get_object("adj_stability_param").connect("value-changed", self.update_stability_param)
		
		return builder.get_object("main")

class identificarRobos(gui.frameRenderer.frameRenderer):
	def __init__(self, vision):
		super().__init__(vision, "fr_notebook")
		self.__n_robos = None
		self.fbcs = []
	
	@guiMethod
	def updateRobotsInfo(self, robos, bola):
		if self.__n_robos != len(robos)/2:
			for fbc in self.fbcs:
				self.__timeFlow.remove(fbc)
			self.fbcs = []
			self.__n_robos = len(robos)/2
		
		for idx,robo in enumerate(robos):
			if robo.ui:
				robo.ui["idLabel"].set_text("{0}".format(robo.identificador))
				robo.ui["posicaoLabel"].set_text("Posição: x: {:.2f} m, y: {:.2f} m".format(robo.centro[0], robo.centro[1]))
				robo.ui["anguloLabel"].set_text("Ângulo {:.1f}º".format(robo.angulo))
				robo.ui["estadoLabel"].set_text("Estado: " + robo.estado)
				if robo.estado == "Identificado":
					robo.ui["fbc"].set_opacity(1)
				else:
					robo.ui["fbc"].set_opacity(0.5)
			else:
				flowBoxChild = Gtk.FlowBoxChild()
				self.fbcs.append(flowBoxChild)
				Gtk.StyleContext.add_class(flowBoxChild.get_style_context(), "roboRow")
				
				builder = Gtk.Builder.new_from_file("vision/mainVision/robo.ui")
				idLabel = builder.get_object("idLabel")
				posicaoLabel = builder.get_object("posicaoLabel")
				estadoLabel = builder.get_object("estadoLabel")
				anguloLabel = builder.get_object("anguloLabel")
				
				if idx < self.__n_robos:
					Gtk.StyleContext.add_class(flowBoxChild.get_style_context(), "roboAliado")
					builder.get_object("tipoRoboLabel").set_text("Tipo: Aliado")
					self.__timeFlow.add(flowBoxChild)
				else:
					Gtk.StyleContext.add_class(flowBoxChild.get_style_context(), "roboInimigo")
					builder.get_object("tipoRoboLabel").set_text("Tipo: Inimigo")
					self.__timeAdversarioFlow.add(flowBoxChild)
				
				flowBoxChild.add(builder.get_object("main"))
				robo.ui = {"idLabel": idLabel, "posicaoLabel": posicaoLabel, "anguloLabel": anguloLabel, "estadoLabel": estadoLabel, "fbc": flowBoxChild}
				
		self.__timeFlow.show_all()
		self.__timeAdversarioFlow.show_all()
		
		if bola is not None:
			self.__bolaEstado.set_text("Estado: Identificada")
			self.__bolaPosicao.set_text("Posição: x: {:.2f} m, y: {:.2f} m".format(bola[0][0], bola[0][1]))
		else:
			self.__bolaEstado.set_text("Estado: Não-Identificada")
			
	
	def transformFrame(self, frame, originalFrame):
		processed_frame = self.parent.ui_process(frame)
		
		robos = self.parent.robos
		bola = self.parent.bola
		
		self.updateRobotsInfo(robos, bola)
		
		return cv2.cvtColor(processed_frame, cv2.COLOR_RGB2BGR)

	def create_ui_label(self):
		return Gtk.Label("Visão em alto nível")
	
	def create_ui_content(self):
		builder = Gtk.Builder.new_from_file("vision/mainVision/identificarRobos.ui")
		self.__timeFlow = builder.get_object("time_flow")
		self.__timeAdversarioFlow = builder.get_object("time_adversario_flow")
		self.__bolaEstado = builder.get_object("bola_estado")
		self.__bolaPosicao = builder.get_object("bola_posicao")
		return builder.get_object("main")
